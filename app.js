import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/loaders/STLLoader.js';
import { OBJLoader } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/loaders/OBJLoader.js';
import { GLTFLoader } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/loaders/GLTFLoader.js';
import { STLExporter } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/exporters/STLExporter.js';
import { OBJExporter } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/exporters/OBJExporter.js';
import { mergeGeometries, mergeVertices } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/utils/BufferGeometryUtils.js';

const $ = (id) => document.getElementById(id);
const viewer = $('viewer');
const fileInput = $('fileInput');
const openBtn = $('openBtn');
const changeBtn = $('changeBtn');
const analyzeBtn = $('analyzeBtn');
const resetBtn = $('resetBtn');
const exportStlBtn = $('exportStlBtn');
const exportObjBtn = $('exportObjBtn');
const originalView = $('originalView');
const cleanView = $('cleanView');
const strength = $('strength');
const angle = $('angle');
const minRegion = $('minRegion');
const strengthValue = $('strengthValue');
const angleValue = $('angleValue');
const regionValue = $('regionValue');
const busy = $('busy');
const dropHint = $('dropHint');
const viewerHud = $('viewerHud');
const fileName = $('fileName');
const meshStats = $('meshStats');
const resultCard = $('resultCard');
const resultText = $('resultText');
const installBtn = $('installBtn');

let scene, camera, renderer, controls, mesh;
let originalGeometry = null;
let cleanedGeometry = null;
let activeName = 'modelo';
let deferredInstallPrompt = null;

init3D();
registerPWA();

function init3D() {
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(42, 1, 0.01, 100000);
  camera.position.set(2.4, 1.7, 2.8);

  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  viewer.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;

  scene.add(new THREE.HemisphereLight(0xffffff, 0x313744, 2.2));
  const key = new THREE.DirectionalLight(0xffffff, 2.2);
  key.position.set(2, 4, 3);
  scene.add(key);
  const rim = new THREE.DirectionalLight(0xaec9ff, 1.2);
  rim.position.set(-3, 1, -2);
  scene.add(rim);

  const grid = new THREE.GridHelper(10, 20, 0x3c434d, 0x252a31);
  grid.name = 'grid';
  scene.add(grid);

  const resize = () => {
    const w = viewer.clientWidth || 1;
    const h = viewer.clientHeight || 1;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
  };
  new ResizeObserver(resize).observe(viewer);
  resize();

  const animate = () => {
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(animate);
  };
  animate();
}

function registerPWA() {
  if ('serviceWorker' in navigator) {
    window.addEventListener('load', () => navigator.serviceWorker.register('./sw.js').catch(console.warn));
  }
  window.addEventListener('beforeinstallprompt', (e) => {
    e.preventDefault();
    deferredInstallPrompt = e;
    installBtn.classList.remove('hidden');
  });
  installBtn.addEventListener('click', async () => {
    if (!deferredInstallPrompt) return;
    deferredInstallPrompt.prompt();
    await deferredInstallPrompt.userChoice;
    deferredInstallPrompt = null;
    installBtn.classList.add('hidden');
  });
}

openBtn.addEventListener('click', () => fileInput.click());
changeBtn.addEventListener('click', () => fileInput.click());
fileInput.addEventListener('change', () => fileInput.files[0] && loadFile(fileInput.files[0]));

$('fitBtn').addEventListener('click', fitCamera);
$('wireBtn').addEventListener('click', () => {
  if (!mesh) return;
  mesh.material.wireframe = !mesh.material.wireframe;
  mesh.material.needsUpdate = true;
});

strength.addEventListener('input', () => strengthValue.value = `${strength.value}%`);
angle.addEventListener('input', () => angleValue.value = `${angle.value}°`);
minRegion.addEventListener('input', () => regionValue.value = `${minRegion.value} caras`);

originalView.addEventListener('click', () => showGeometry('original'));
cleanView.addEventListener('click', () => showGeometry('clean'));
resetBtn.addEventListener('click', () => {
  cleanedGeometry?.dispose();
  cleanedGeometry = null;
  showGeometry('original');
  cleanView.disabled = true;
  exportStlBtn.disabled = true;
  exportObjBtn.disabled = true;
  resultCard.classList.add('hidden');
});

analyzeBtn.addEventListener('click', runCleanup);
exportStlBtn.addEventListener('click', exportSTL);
exportObjBtn.addEventListener('click', exportOBJ);

['dragenter','dragover'].forEach(type => viewer.addEventListener(type, e => { e.preventDefault(); viewer.classList.add('dragging'); }));
['dragleave','drop'].forEach(type => viewer.addEventListener(type, e => { e.preventDefault(); viewer.classList.remove('dragging'); }));
viewer.addEventListener('drop', e => { const f = e.dataTransfer.files[0]; if (f) loadFile(f); });

async function loadGLTF(buffer, ext) {
  const loader = new GLTFLoader();
  const payload = ext === 'gltf' ? new TextDecoder().decode(buffer) : buffer;
  const gltf = await new Promise((resolve, reject) => loader.parse(payload, '', resolve, reject));
  gltf.scene.updateMatrixWorld(true);

  const geos = [];
  gltf.scene.traverse(child => {
    if (!child.isMesh || !child.geometry?.attributes?.position) return;
    let g = child.geometry.clone();
    if (g.index) g = g.toNonIndexed();
    const p = g.attributes.position.clone();
    const flattened = new THREE.BufferGeometry();
    flattened.setAttribute('position', p);
    flattened.applyMatrix4(child.matrixWorld);
    geos.push(flattened);
  });

  if (!geos.length) throw new Error('El GLB/GLTF no contiene geometría de mesh.');
  return geos.length === 1 ? geos[0] : mergeGeometries(geos, false);
}

async function loadFile(file) {
  const ext = file.name.split('.').pop().toLowerCase();
  if (!['stl','obj','glb','gltf'].includes(ext)) return alert('3DLab acepta GLB, GLTF, STL y OBJ.');
  setBusy(true);
  try {
    const buffer = await file.arrayBuffer();
    let geometry;
    if (ext === 'stl') {
      geometry = new STLLoader().parse(buffer);
    } else if (ext === 'obj') {
      const text = new TextDecoder().decode(buffer);
      const obj = new OBJLoader().parse(text);
      obj.updateMatrixWorld(true);
      const geos = [];
      obj.traverse(child => {
        if (!child.isMesh || !child.geometry?.attributes?.position) return;
        const g = new THREE.BufferGeometry();
        const p = child.geometry.attributes.position.clone();
        g.setAttribute('position', p);
        g.applyMatrix4(child.matrixWorld);
        geos.push(g);
      });
      if (!geos.length) throw new Error('El OBJ no contiene geometría de mesh.');
      geometry = geos.length === 1 ? geos[0] : mergeGeometries(geos, false);
    } else {
      geometry = await loadGLTF(buffer, ext);
    }

    geometry = normalizeGeometry(geometry);
    originalGeometry?.dispose();
    cleanedGeometry?.dispose();
    originalGeometry = geometry;
    cleanedGeometry = null;
    activeName = file.name.replace(/\.[^.]+$/, '');
    mountGeometry(originalGeometry);

    dropHint.classList.add('hidden');
    viewerHud.classList.remove('hidden');
    fileName.textContent = file.name;
    updateStats();
    setControlsEnabled(true);
    cleanView.disabled = true;
    exportStlBtn.disabled = true;
    exportObjBtn.disabled = true;
    resultCard.classList.add('hidden');
    showGeometry('original');
    fitCamera();
  } catch (err) {
    console.error(err);
    alert(`No pude abrir el modelo: ${err.message}`);
  } finally {
    setBusy(false);
  }
}

function normalizeGeometry(input) {
  let g = new THREE.BufferGeometry();
  g.setAttribute('position', input.attributes.position.clone());
  g = mergeVertices(g, 1e-5);
  g.computeVertexNormals();
  g.computeBoundingBox();
  g.computeBoundingSphere();
  return g;
}

function mountGeometry(geometry) {
  if (mesh) {
    scene.remove(mesh);
    mesh.material.dispose();
  }
  const material = new THREE.MeshStandardMaterial({ color: 0xe4e7eb, roughness: 0.72, metalness: 0.02, side: THREE.DoubleSide });
  mesh = new THREE.Mesh(geometry, material);
  scene.add(mesh);
}

function showGeometry(which) {
  const g = which === 'clean' && cleanedGeometry ? cleanedGeometry : originalGeometry;
  if (!g || !mesh) return;
  mesh.geometry = g;
  originalView.classList.toggle('active', which !== 'clean');
  cleanView.classList.toggle('active', which === 'clean');
}

function fitCamera() {
  if (!mesh?.geometry?.boundingBox) return;
  const box = mesh.geometry.boundingBox.clone();
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z) || 1;
  const dist = maxDim / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov * .5)));
  camera.near = Math.max(maxDim / 10000, 0.001);
  camera.far = maxDim * 100;
  camera.position.copy(center).add(new THREE.Vector3(1, .72, 1).normalize().multiplyScalar(dist * 1.65));
  camera.updateProjectionMatrix();
  controls.target.copy(center);
  controls.update();
  const grid = scene.getObjectByName('grid');
  if (grid) {
    grid.scale.setScalar(maxDim / 10);
    grid.position.y = box.min.y - maxDim * .02;
  }
}

function setControlsEnabled(enabled) {
  [changeBtn, analyzeBtn, resetBtn, originalView, strength, angle, minRegion].forEach(el => el.disabled = !enabled);
}

function setBusy(on) { busy.classList.toggle('hidden', !on); }

function updateStats() {
  if (!originalGeometry) return;
  const verts = originalGeometry.attributes.position.count;
  const faces = originalGeometry.index ? originalGeometry.index.count / 3 : verts / 3;
  meshStats.textContent = `${verts.toLocaleString('es-AR')} vértices · ${Math.round(faces).toLocaleString('es-AR')} caras`;
}

function runCleanup() {
  if (!originalGeometry) return;
  const index = originalGeometry.index;
  if (!index) return alert('No pude indexar este mesh.');
  const faceCount = index.count / 3;
  if (faceCount > 750000) {
    return alert(`Este MVP limita el análisis CAD a 750.000 caras para cuidar la memoria del celular. Tu modelo tiene ${Math.round(faceCount).toLocaleString('es-AR')}. Podemos subir ese límite después de probar rendimiento.`);
  }

  setBusy(true);
  analyzeBtn.disabled = true;
  const positions = new Float32Array(originalGeometry.attributes.position.array);
  const indices = index.array.constructor.from(index.array);
  const worker = new Worker('./mesh-worker.js');

  worker.onmessage = ({data}) => {
    if (data.type === 'error') {
      worker.terminate(); setBusy(false); analyzeBtn.disabled = false; alert(data.message); return;
    }
    if (data.type !== 'done') return;
    const g = originalGeometry.clone();
    g.setAttribute('position', new THREE.BufferAttribute(data.positions, 3));
    g.computeVertexNormals();
    g.computeBoundingBox();
    g.computeBoundingSphere();
    cleanedGeometry?.dispose();
    cleanedGeometry = g;
    mesh.geometry = cleanedGeometry;
    cleanView.disabled = false;
    exportStlBtn.disabled = false;
    exportObjBtn.disabled = false;
    showGeometry('clean');
    resultText.textContent = `${data.regions} regiones planas corregidas · ${data.verticesMoved.toLocaleString('es-AR')} vértices ajustados · ${(data.elapsedMs/1000).toFixed(1)} s`;
    resultCard.classList.remove('hidden');
    analyzeBtn.disabled = false;
    setBusy(false);
    worker.terminate();
  };
  worker.onerror = (e) => {
    console.error(e); worker.terminate(); setBusy(false); analyzeBtn.disabled = false;
    alert('El procesamiento falló. Probablemente el modelo sea demasiado pesado para la memoria disponible.');
  };

  worker.postMessage({
    type:'clean', positions, indices,
    strength: Number(strength.value)/100,
    angleDeg: Number(angle.value),
    minRegionFaces: Number(minRegion.value)
  }, [positions.buffer, indices.buffer]);
}

function exportSTL() {
  if (!cleanedGeometry) return;
  const temp = new THREE.Mesh(cleanedGeometry);
  const data = new STLExporter().parse(temp, { binary: true });
  downloadBlob(new Blob([data], {type:'model/stl'}), `${activeName}_3dlab.stl`);
}

function exportOBJ() {
  if (!cleanedGeometry) return;
  const temp = new THREE.Mesh(cleanedGeometry);
  const text = new OBJExporter().parse(temp);
  downloadBlob(new Blob([text], {type:'text/plain'}), `${activeName}_3dlab.obj`);
}

function downloadBlob(blob, name) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = name; a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1500);
}

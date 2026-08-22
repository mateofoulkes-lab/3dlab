import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/loaders/STLLoader.js';
import { OBJLoader } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/loaders/OBJLoader.js';
import { GLTFLoader } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/loaders/GLTFLoader.js';
import { STLExporter } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/exporters/STLExporter.js';
import { OBJExporter } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/exporters/OBJExporter.js';
import { mergeGeometries, mergeVertices } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/utils/BufferGeometryUtils.js';

const APP_VERSION='3.0';
const $=id=>document.getElementById(id);
const els={
 viewer:$('viewer'),fileInput:$('fileInput'),openBtn:$('openBtn'),changeBtn:$('changeBtn'),analyzeBtn:$('analyzeBtn'),resetBtn:$('resetBtn'),
 exportStlBtn:$('exportStlBtn'),exportObjBtn:$('exportObjBtn'),originalView:$('originalView'),cleanView:$('cleanView'),strength:$('strength'),angle:$('angle'),
 confidence:$('confidence'),edgeProtect:$('edgeProtect'),minRegion:$('minRegion'),alignParallel:$('alignParallel'),orthogonalSnap:$('orthogonalSnap'),
 sharpIntersections:$('sharpIntersections'),detectThickness:$('detectThickness'),strengthValue:$('strengthValue'),angleValue:$('angleValue'),confidenceValue:$('confidenceValue'),
 edgeValue:$('edgeValue'),regionValue:$('regionValue'),busy:$('busy'),busyStage:$('busyStage'),dropHint:$('dropHint'),viewerHud:$('viewerHud'),fileName:$('fileName'),
 meshStats:$('meshStats'),resultCard:$('resultCard'),resultText:$('resultText'),cadFacts:$('cadFacts'),installBtn:$('installBtn'),versionBadge:$('versionBadge')
};
els.versionBadge.textContent=`v${APP_VERSION}`;

let scene,camera,renderer,controls,mesh,originalGeometry=null,reconstructedGeometry=null,activeName='modelo',deferredInstallPrompt=null,reloadingForUpdate=false;
init3D(); registerPWA(); bindUI();

function init3D(){
 scene=new THREE.Scene(); camera=new THREE.PerspectiveCamera(42,1,.01,100000); camera.position.set(2.4,1.7,2.8);
 renderer=new THREE.WebGLRenderer({antialias:true,alpha:true,powerPreference:'high-performance'}); renderer.setPixelRatio(Math.min(devicePixelRatio,2)); renderer.outputColorSpace=THREE.SRGBColorSpace; els.viewer.appendChild(renderer.domElement);
 controls=new OrbitControls(camera,renderer.domElement); controls.enableDamping=true; controls.dampingFactor=.08;
 scene.add(new THREE.HemisphereLight(0xffffff,0x273445,2.1)); const key=new THREE.DirectionalLight(0xffffff,2.3); key.position.set(2,4,3); scene.add(key); const rim=new THREE.DirectionalLight(0x92bfff,1.1); rim.position.set(-3,1,-2); scene.add(rim);
 const grid=new THREE.GridHelper(10,20,0x415165,0x222b36); grid.name='grid'; scene.add(grid);
 const resize=()=>{const w=els.viewer.clientWidth||1,h=els.viewer.clientHeight||1;camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false)}; new ResizeObserver(resize).observe(els.viewer); resize();
 (function animate(){controls.update();renderer.render(scene,camera);requestAnimationFrame(animate)})();
}

function registerPWA(){
 if('serviceWorker' in navigator){
  navigator.serviceWorker.addEventListener('controllerchange',()=>{if(reloadingForUpdate)return;reloadingForUpdate=true;location.reload()});
  window.addEventListener('load',async()=>{try{const reg=await navigator.serviceWorker.register(`./sw.js?v=${APP_VERSION}`,{updateViaCache:'none'});await reg.update();if(reg.waiting)reg.waiting.postMessage({type:'SKIP_WAITING'});reg.addEventListener('updatefound',()=>{const w=reg.installing;if(!w)return;w.addEventListener('statechange',()=>{if(w.state==='installed'&&navigator.serviceWorker.controller)w.postMessage({type:'SKIP_WAITING'})})})}catch(e){console.warn('PWA update',e)}})
 }
 window.addEventListener('beforeinstallprompt',e=>{e.preventDefault();deferredInstallPrompt=e;els.installBtn.classList.remove('hidden')});
 els.installBtn.addEventListener('click',async()=>{if(!deferredInstallPrompt)return;deferredInstallPrompt.prompt();await deferredInstallPrompt.userChoice;deferredInstallPrompt=null;els.installBtn.classList.add('hidden')});
}

function bindUI(){
 els.openBtn.onclick=()=>els.fileInput.click(); els.changeBtn.onclick=()=>els.fileInput.click(); els.fileInput.onchange=()=>els.fileInput.files[0]&&loadFile(els.fileInput.files[0]);
 $('fitBtn').onclick=fitCamera; $('wireBtn').onclick=()=>{if(mesh){mesh.material.wireframe=!mesh.material.wireframe;mesh.material.needsUpdate=true}};
 els.strength.oninput=()=>els.strengthValue.value=`${els.strength.value}%`; els.angle.oninput=()=>els.angleValue.value=`${els.angle.value}°`; els.confidence.oninput=()=>els.confidenceValue.value=`${els.confidence.value}%`; els.edgeProtect.oninput=()=>els.edgeValue.value=`${els.edgeProtect.value}%`; els.minRegion.oninput=()=>els.regionValue.value=`${els.minRegion.value} caras`;
 els.originalView.onclick=()=>showGeometry('original'); els.cleanView.onclick=()=>showGeometry('clean'); els.analyzeBtn.onclick=runReconstruction; els.resetBtn.onclick=resetResult; els.exportStlBtn.onclick=exportSTL; els.exportObjBtn.onclick=exportOBJ;
 ['dragenter','dragover'].forEach(t=>els.viewer.addEventListener(t,e=>{e.preventDefault();els.viewer.parentElement.classList.add('dragging')})); ['dragleave','drop'].forEach(t=>els.viewer.addEventListener(t,e=>{e.preventDefault();els.viewer.parentElement.classList.remove('dragging')})); els.viewer.addEventListener('drop',e=>{const f=e.dataTransfer.files[0];if(f)loadFile(f)});
}

async function loadGLTF(buffer,ext){const loader=new GLTFLoader(),payload=ext==='gltf'?new TextDecoder().decode(buffer):buffer;const gltf=await new Promise((res,rej)=>loader.parse(payload,'',res,rej));gltf.scene.updateMatrixWorld(true);const geos=[];gltf.scene.traverse(c=>{if(!c.isMesh||!c.geometry?.attributes?.position)return;let g=c.geometry.clone();if(g.index)g=g.toNonIndexed();const f=new THREE.BufferGeometry();f.setAttribute('position',g.attributes.position.clone());f.applyMatrix4(c.matrixWorld);geos.push(f)});if(!geos.length)throw new Error('El GLB/GLTF no contiene meshes.');return geos.length===1?geos[0]:mergeGeometries(geos,false)}

async function loadFile(file){
 const ext=file.name.split('.').pop().toLowerCase(); if(!['stl','obj','glb','gltf'].includes(ext))return alert('3DLab acepta GLB, GLTF, STL y OBJ.'); setBusy(true,'Abriendo modelo');
 try{const buffer=await file.arrayBuffer();let geometry;if(ext==='stl')geometry=new STLLoader().parse(buffer);else if(ext==='obj'){const obj=new OBJLoader().parse(new TextDecoder().decode(buffer));obj.updateMatrixWorld(true);const geos=[];obj.traverse(c=>{if(!c.isMesh||!c.geometry?.attributes?.position)return;const g=new THREE.BufferGeometry();g.setAttribute('position',c.geometry.attributes.position.clone());g.applyMatrix4(c.matrixWorld);geos.push(g)});if(!geos.length)throw new Error('El OBJ no contiene meshes.');geometry=geos.length===1?geos[0]:mergeGeometries(geos,false)}else geometry=await loadGLTF(buffer,ext);
 geometry=normalizeGeometry(geometry); originalGeometry?.dispose(); reconstructedGeometry?.dispose(); originalGeometry=geometry; reconstructedGeometry=null; activeName=file.name.replace(/\.[^.]+$/,''); mountGeometry(originalGeometry); els.dropHint.classList.add('hidden');els.viewerHud.classList.remove('hidden');els.fileName.textContent=file.name;updateStats();setControlsEnabled(true);els.cleanView.disabled=true;els.exportStlBtn.disabled=true;els.exportObjBtn.disabled=true;els.resultCard.classList.add('hidden');showGeometry('original');fitCamera();
 }catch(err){console.error(err);alert(`No pude abrir el modelo: ${err.message}`)}finally{setBusy(false)}
}

function normalizeGeometry(input){let g=new THREE.BufferGeometry();g.setAttribute('position',input.attributes.position.clone());g=mergeVertices(g,1e-5);g.computeVertexNormals();g.computeBoundingBox();g.computeBoundingSphere();return g}
function mountGeometry(g){if(mesh){scene.remove(mesh);mesh.material.dispose()}mesh=new THREE.Mesh(g,new THREE.MeshStandardMaterial({color:0xe7edf3,roughness:.72,metalness:.02,side:THREE.DoubleSide}));scene.add(mesh)}
function showGeometry(which){const g=which==='clean'&&reconstructedGeometry?reconstructedGeometry:originalGeometry;if(!g||!mesh)return;mesh.geometry=g;els.originalView.classList.toggle('active',which!=='clean');els.cleanView.classList.toggle('active',which==='clean')}
function resetResult(){reconstructedGeometry?.dispose();reconstructedGeometry=null;showGeometry('original');els.cleanView.disabled=true;els.exportStlBtn.disabled=true;els.exportObjBtn.disabled=true;els.resultCard.classList.add('hidden')}
function fitCamera(){if(!mesh?.geometry?.boundingBox)return;const box=mesh.geometry.boundingBox.clone(),size=box.getSize(new THREE.Vector3()),center=box.getCenter(new THREE.Vector3()),maxDim=Math.max(size.x,size.y,size.z)||1,dist=maxDim/(2*Math.tan(THREE.MathUtils.degToRad(camera.fov*.5)));camera.near=Math.max(maxDim/10000,.001);camera.far=maxDim*100;camera.position.copy(center).add(new THREE.Vector3(1,.72,1).normalize().multiplyScalar(dist*1.65));camera.updateProjectionMatrix();controls.target.copy(center);controls.update();const grid=scene.getObjectByName('grid');if(grid){grid.scale.setScalar(maxDim/10);grid.position.y=box.min.y-maxDim*.02}}
function setControlsEnabled(v){[els.changeBtn,els.analyzeBtn,els.resetBtn,els.originalView,els.strength,els.angle,els.confidence,els.edgeProtect,els.minRegion,els.alignParallel,els.orthogonalSnap,els.sharpIntersections,els.detectThickness].forEach(e=>e.disabled=!v)}
function setBusy(on,stage='Analizando topología'){els.busy.classList.toggle('hidden',!on);els.busyStage.textContent=stage}
function updateStats(){const v=originalGeometry.attributes.position.count,f=originalGeometry.index?originalGeometry.index.count/3:v/3;els.meshStats.textContent=`${v.toLocaleString('es-AR')} vértices · ${Math.round(f).toLocaleString('es-AR')} caras`}

function runReconstruction(){
 if(!originalGeometry?.index)return alert('No pude indexar este mesh.');const faceCount=originalGeometry.index.count/3;if(faceCount>800000)return alert(`Este motor experimental limita el análisis a 800.000 caras en móvil. El modelo tiene ${Math.round(faceCount).toLocaleString('es-AR')}.`);
 setBusy(true,'Detectando planos y relaciones');els.analyzeBtn.disabled=true;const positions=new Float32Array(originalGeometry.attributes.position.array),indices=originalGeometry.index.array.constructor.from(originalGeometry.index.array),worker=new Worker(`./mesh-worker.js?v=${APP_VERSION}`);
 worker.onmessage=({data})=>{if(data.type==='progress'){els.busyStage.textContent=data.stage;return}if(data.type==='error'){worker.terminate();setBusy(false);els.analyzeBtn.disabled=false;alert(data.message);return}if(data.type!=='done')return;const g=originalGeometry.clone();g.setAttribute('position',new THREE.BufferAttribute(data.positions,3));g.computeVertexNormals();g.computeBoundingBox();g.computeBoundingSphere();reconstructedGeometry?.dispose();reconstructedGeometry=g;mesh.geometry=g;els.cleanView.disabled=false;els.exportStlBtn.disabled=false;els.exportObjBtn.disabled=false;showGeometry('clean');els.resultText.textContent=`${data.regions} planos · ${data.orthogonalEdges} intersecciones rectificadas · ${data.verticesMoved.toLocaleString('es-AR')} vértices corregidos · ${(data.elapsedMs/1000).toFixed(1)} s`;els.cadFacts.innerHTML='';const facts=[`${data.parallelFamilies} familias paralelas`,`${data.rejectedRegions} zonas ambiguas preservadas`];if(data.thicknesses?.length)facts.push(`espesores candidatos: ${data.thicknesses.join(' / ')}`);facts.forEach(t=>{const s=document.createElement('span');s.className='cad-fact';s.textContent=t;els.cadFacts.appendChild(s)});els.resultCard.classList.remove('hidden');els.analyzeBtn.disabled=false;setBusy(false);worker.terminate()};
 worker.onerror=e=>{console.error(e);worker.terminate();setBusy(false);els.analyzeBtn.disabled=false;alert('La reconstrucción falló; probablemente faltó memoria disponible.')};
 worker.postMessage({type:'reconstruct',positions,indices,strength:+els.strength.value/100,angleDeg:+els.angle.value,minRegionFaces:+els.minRegion.value,planeConfidence:+els.confidence.value/100,edgeProtection:+els.edgeProtect.value/100,alignParallel:els.alignParallel.checked,orthogonalSnap:els.orthogonalSnap.checked,sharpIntersections:els.sharpIntersections.checked,detectThickness:els.detectThickness.checked},[positions.buffer,indices.buffer]);
}
function exportSTL(){if(!reconstructedGeometry)return;const data=new STLExporter().parse(new THREE.Mesh(reconstructedGeometry),{binary:true});downloadBlob(new Blob([data],{type:'model/stl'}),`${activeName}_3dlab_cad.stl`)}
function exportOBJ(){if(!reconstructedGeometry)return;const text=new OBJExporter().parse(new THREE.Mesh(reconstructedGeometry));downloadBlob(new Blob([text],{type:'text/plain'}),`${activeName}_3dlab_cad.obj`)}
function downloadBlob(blob,name){const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1500)}

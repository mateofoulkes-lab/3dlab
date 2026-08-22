self.onmessage = ({data}) => {
  if (data.type !== 'clean') return;
  try {
    const started = performance.now();
    const {positions, indices, strength, angleDeg, minRegionFaces} = data;
    const vertexCount = positions.length / 3;
    const faceCount = indices.length / 3;
    if (!vertexCount || !faceCount) throw new Error('Mesh vacío o inválido.');

    const cosLimit = Math.cos(angleDeg * Math.PI / 180);
    const cosSeedLimit = Math.cos(Math.min(angleDeg * 1.6, 42) * Math.PI / 180);
    const normals = new Float32Array(faceCount * 3);
    const areas = new Float32Array(faceCount);

    // Normales de cara y área: la señal principal para distinguir planos de curvas/aristas.
    for (let f = 0; f < faceCount; f++) {
      const ia = indices[f*3]*3, ib = indices[f*3+1]*3, ic = indices[f*3+2]*3;
      const ax=positions[ia], ay=positions[ia+1], az=positions[ia+2];
      const abx=positions[ib]-ax, aby=positions[ib+1]-ay, abz=positions[ib+2]-az;
      const acx=positions[ic]-ax, acy=positions[ic+1]-ay, acz=positions[ic+2]-az;
      let nx=aby*acz-abz*acy, ny=abz*acx-abx*acz, nz=abx*acy-aby*acx;
      const len=Math.hypot(nx,ny,nz) || 1;
      areas[f]=len*.5;
      nx/=len; ny/=len; nz/=len;
      normals[f*3]=nx; normals[f*3+1]=ny; normals[f*3+2]=nz;
    }

    // Adyacencia por arista. Cada triángulo tiene como máximo 3 vecinos útiles.
    const neighbors = new Int32Array(faceCount * 3);
    neighbors.fill(-1);
    const neighborCount = new Uint8Array(faceCount);
    const edgeOwner = new Map();
    const edgeKey = (a,b) => {
      if (a>b) { const t=a; a=b; b=t; }
      return a * vertexCount + b;
    };
    for (let f=0; f<faceCount; f++) {
      const a=indices[f*3], b=indices[f*3+1], c=indices[f*3+2];
      const edges=[[a,b],[b,c],[c,a]];
      for (let e=0;e<3;e++) {
        const key=edgeKey(edges[e][0],edges[e][1]);
        const other=edgeOwner.get(key);
        if (other === undefined) edgeOwner.set(key,f);
        else if (other !== -1) {
          const ca=neighborCount[f], cb=neighborCount[other];
          if (ca<3 && cb<3) {
            neighbors[f*3+ca]=other; neighborCount[f]=ca+1;
            neighbors[other*3+cb]=f; neighborCount[other]=cb+1;
          }
          edgeOwner.set(key,-1);
        }
      }
    }
    edgeOwner.clear();

    const output = new Float32Array(positions);
    const bestRegionSize = new Uint32Array(vertexCount);
    const visited = new Uint8Array(faceCount);
    const queue = new Int32Array(faceCount);
    let acceptedRegions=0, verticesMoved=0;

    for (let seed=0; seed<faceCount; seed++) {
      if (visited[seed]) continue;
      visited[seed]=1;
      let head=0, tail=1;
      queue[0]=seed;
      const snx=normals[seed*3], sny=normals[seed*3+1], snz=normals[seed*3+2];
      let sumNx=snx, sumNy=sny, sumNz=snz;

      while (head<tail) {
        const f=queue[head++];
        let avgLen=Math.hypot(sumNx,sumNy,sumNz) || 1;
        const anx=sumNx/avgLen, any=sumNy/avgLen, anz=sumNz/avgLen;
        const count=neighborCount[f];
        for (let k=0;k<count;k++) {
          const nb=neighbors[f*3+k];
          if (nb<0 || visited[nb]) continue;
          const nx=normals[nb*3], ny=normals[nb*3+1], nz=normals[nb*3+2];
          const dotAvg=nx*anx+ny*any+nz*anz;
          const dotSeed=nx*snx+ny*sny+nz*snz;
          if (dotAvg>=cosLimit && dotSeed>=cosSeedLimit) {
            visited[nb]=1;
            queue[tail++]=nb;
            sumNx+=nx; sumNy+=ny; sumNz+=nz;
          }
        }
      }

      if (tail < minRegionFaces) continue;

      // Plano ideal: normal promedio ponderada por área + offset medio ponderado.
      let pnx=0,pny=0,pnz=0,totalArea=0;
      for (let q=0;q<tail;q++) {
        const f=queue[q], a=areas[f] || 1;
        pnx+=normals[f*3]*a; pny+=normals[f*3+1]*a; pnz+=normals[f*3+2]*a; totalArea+=a;
      }
      let plen=Math.hypot(pnx,pny,pnz);
      if (plen<1e-8) continue;
      pnx/=plen; pny/=plen; pnz/=plen;

      let dsum=0, wsum=0;
      for (let q=0;q<tail;q++) {
        const f=queue[q], w=(areas[f] || 1)/3;
        for (let j=0;j<3;j++) {
          const vi=indices[f*3+j]*3;
          dsum+=(positions[vi]*pnx+positions[vi+1]*pny+positions[vi+2]*pnz)*w;
          wsum+=w;
        }
      }
      const planeD=dsum/(wsum || 1);

      // Rechaza regiones cuya dispersión sea demasiado grande para ser una cara plana.
      let sq=0, samples=0;
      const stride=Math.max(1,Math.floor(tail/2500));
      for (let q=0;q<tail;q+=stride) {
        const f=queue[q];
        for (let j=0;j<3;j++) {
          const vi=indices[f*3+j]*3;
          const dist=positions[vi]*pnx+positions[vi+1]*pny+positions[vi+2]*pnz-planeD;
          sq+=dist*dist; samples++;
        }
      }
      const rms=Math.sqrt(sq/(samples||1));
      // Estima escala típica de triángulo desde área. Tolera ruido importante, pero no una curva marcada.
      const meanArea=totalArea/tail;
      const typicalEdge=Math.sqrt(Math.max(meanArea,1e-16)*2);
      if (rms > typicalEdge * 1.35) continue;

      acceptedRegions++;
      for (let q=0;q<tail;q++) {
        const f=queue[q];
        for (let j=0;j<3;j++) {
          const v=indices[f*3+j];
          if (bestRegionSize[v] >= tail) continue;
          const vi=v*3;
          const dist=positions[vi]*pnx+positions[vi+1]*pny+positions[vi+2]*pnz-planeD;
          output[vi]=positions[vi]-pnx*dist*strength;
          output[vi+1]=positions[vi+1]-pny*dist*strength;
          output[vi+2]=positions[vi+2]-pnz*dist*strength;
          if (bestRegionSize[v]===0 && Math.abs(dist*strength)>1e-8) verticesMoved++;
          bestRegionSize[v]=tail;
        }
      }
    }

    self.postMessage({type:'done',positions:output,regions:acceptedRegions,verticesMoved,elapsedMs:performance.now()-started},[output.buffer]);
  } catch (err) {
    self.postMessage({type:'error',message:err?.message || String(err)});
  }
};

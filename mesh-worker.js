self.onmessage = ({data}) => {
  if (data.type !== 'clean') return;
  try {
    const started = performance.now();
    const {
      positions, indices, strength, angleDeg, minRegionFaces,
      edgeProtection = 0.85, planeConfidence = 0.72, alignParallel = true
    } = data;
    const vertexCount = positions.length / 3;
    const faceCount = indices.length / 3;
    if (!vertexCount || !faceCount) throw new Error('Mesh vacío o inválido.');

    const cosGrow = Math.cos(angleDeg * Math.PI / 180);
    const cosSeed = Math.cos(Math.min(angleDeg * 1.22, 30) * Math.PI / 180);
    const normals = new Float32Array(faceCount * 3);
    const areas = new Float32Array(faceCount);
    const vertexDegree = new Uint16Array(vertexCount);

    for (let f = 0; f < faceCount; f++) {
      const a = indices[f*3], b = indices[f*3+1], c = indices[f*3+2];
      if (vertexDegree[a] < 65535) vertexDegree[a]++;
      if (vertexDegree[b] < 65535) vertexDegree[b]++;
      if (vertexDegree[c] < 65535) vertexDegree[c]++;
      const ia=a*3, ib=b*3, ic=c*3;
      const ax=positions[ia], ay=positions[ia+1], az=positions[ia+2];
      const abx=positions[ib]-ax, aby=positions[ib+1]-ay, abz=positions[ib+2]-az;
      const acx=positions[ic]-ax, acy=positions[ic+1]-ay, acz=positions[ic+2]-az;
      let nx=aby*acz-abz*acy, ny=abz*acx-abx*acz, nz=abx*acy-aby*acx;
      const len=Math.hypot(nx,ny,nz) || 1;
      areas[f]=len*.5;
      normals[f*3]=nx/len; normals[f*3+1]=ny/len; normals[f*3+2]=nz/len;
    }

    // Topología por arista. Mantener conectividad real es crucial para no mezclar caras cercanas que no pertenecen a la misma pieza.
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
      const e0a=a,e0b=b,e1a=b,e1b=c,e2a=c,e2b=a;
      const ea=[e0a,e1a,e2a], eb=[e0b,e1b,e2b];
      for (let e=0;e<3;e++) {
        const key=edgeKey(ea[e],eb[e]);
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

    const visited = new Uint8Array(faceCount);
    const queue = new Int32Array(faceCount);
    const vertexStamp = new Uint32Array(vertexCount);
    const incidence = new Uint16Array(vertexCount);
    const touchedVerts = new Int32Array(vertexCount);
    let stamp = 0;
    let rejectedRegions = 0;

    function smallestEigenVector(xx,xy,xz,yy,yz,zz) {
      // Jacobi para matriz simétrica 3x3. Ocho barridos son suficientes para un ajuste de plano geométrico.
      const A = [xx,xy,xz, xy,yy,yz, xz,yz,zz];
      const V = [1,0,0, 0,1,0, 0,0,1];
      for (let iter=0; iter<10; iter++) {
        let p=0,q=1,max=Math.abs(A[1]);
        if (Math.abs(A[2])>max) { p=0;q=2;max=Math.abs(A[2]); }
        if (Math.abs(A[5])>max) { p=1;q=2;max=Math.abs(A[5]); }
        if (max < 1e-12) break;
        const app=A[p*3+p], aqq=A[q*3+q], apq=A[p*3+q];
        const phi=.5*Math.atan2(2*apq,aqq-app);
        const c=Math.cos(phi), s=Math.sin(phi);
        for (let k=0;k<3;k++) {
          const aik=A[p*3+k], aqk=A[q*3+k];
          A[p*3+k]=c*aik-s*aqk; A[q*3+k]=s*aik+c*aqk;
        }
        for (let k=0;k<3;k++) {
          const akp=A[k*3+p], akq=A[k*3+q];
          A[k*3+p]=c*akp-s*akq; A[k*3+q]=s*akp+c*akq;
        }
        for (let k=0;k<3;k++) {
          const vip=V[k*3+p], viq=V[k*3+q];
          V[k*3+p]=c*vip-s*viq; V[k*3+q]=s*vip+c*viq;
        }
      }
      let col=0;
      if (A[4] < A[col*3+col]) col=1;
      if (A[8] < A[col*3+col]) col=2;
      let x=V[col], y=V[3+col], z=V[6+col];
      const l=Math.hypot(x,y,z)||1;
      return [x/l,y/l,z/l];
    }

    const regions=[];

    for (let seed=0; seed<faceCount; seed++) {
      if (visited[seed]) continue;
      visited[seed]=1;
      let head=0, tail=1;
      queue[0]=seed;
      const snx=normals[seed*3], sny=normals[seed*3+1], snz=normals[seed*3+2];
      let sumNx=snx, sumNy=sny, sumNz=snz;

      while (head<tail) {
        const f=queue[head++];
        const al=Math.hypot(sumNx,sumNy,sumNz)||1;
        const anx=sumNx/al, any=sumNy/al, anz=sumNz/al;
        for (let k=0;k<neighborCount[f];k++) {
          const nb=neighbors[f*3+k];
          if (nb<0 || visited[nb]) continue;
          const nx=normals[nb*3], ny=normals[nb*3+1], nz=normals[nb*3+2];
          const dotAvg=nx*anx+ny*any+nz*anz;
          const dotSeed=nx*snx+ny*sny+nz*snz;
          // Dos límites simultáneos evitan que una superficie curva "camine" gradualmente hasta parecer un plano enorme.
          if (dotAvg>=cosGrow && dotSeed>=cosSeed) {
            visited[nb]=1;
            queue[tail++]=nb;
            sumNx+=nx; sumNy+=ny; sumNz+=nz;
          }
        }
      }

      if (tail < minRegionFaces) continue;

      stamp++;
      if (stamp === 0xffffffff) { vertexStamp.fill(0); stamp=1; }
      let uniqueCount=0,totalArea=0,avgNx=0,avgNy=0,avgNz=0;
      for (let q=0;q<tail;q++) {
        const f=queue[q], ar=areas[f]||1;
        totalArea+=ar;
        avgNx+=normals[f*3]*ar; avgNy+=normals[f*3+1]*ar; avgNz+=normals[f*3+2]*ar;
        for (let j=0;j<3;j++) {
          const v=indices[f*3+j];
          if (vertexStamp[v]!==stamp) {
            vertexStamp[v]=stamp; incidence[v]=1; touchedVerts[uniqueCount++]=v;
          } else if (incidence[v] < 65535) incidence[v]++;
        }
      }

      let cx=0,cy=0,cz=0;
      for (let i=0;i<uniqueCount;i++) {
        const vi=touchedVerts[i]*3;
        cx+=positions[vi]; cy+=positions[vi+1]; cz+=positions[vi+2];
      }
      cx/=uniqueCount; cy/=uniqueCount; cz/=uniqueCount;

      let xx=0,xy=0,xz=0,yy=0,yz=0,zz=0;
      for (let i=0;i<uniqueCount;i++) {
        const vi=touchedVerts[i]*3;
        const x=positions[vi]-cx,y=positions[vi+1]-cy,z=positions[vi+2]-cz;
        xx+=x*x; xy+=x*y; xz+=x*z; yy+=y*y; yz+=y*z; zz+=z*z;
      }
      let [pnx,pny,pnz]=smallestEigenVector(xx,xy,xz,yy,yz,zz);
      if (pnx*avgNx+pny*avgNy+pnz*avgNz < 0) { pnx=-pnx;pny=-pny;pnz=-pnz; }
      const planeD=cx*pnx+cy*pny+cz*pnz;

      let sq=0,absSum=0,maxAbs=0;
      for (let i=0;i<uniqueCount;i++) {
        const vi=touchedVerts[i]*3;
        const dist=positions[vi]*pnx+positions[vi+1]*pny+positions[vi+2]*pnz-planeD;
        const ad=Math.abs(dist); sq+=dist*dist; absSum+=ad; if(ad>maxAbs)maxAbs=ad;
      }
      const rms=Math.sqrt(sq/uniqueCount);
      const meanAbs=absSum/uniqueCount;
      const meanArea=totalArea/tail;
      const typicalEdge=Math.sqrt(Math.max(meanArea,1e-16)*2);
      const rmsLimit=typicalEdge*(1.10-planeConfidence*.75);
      const meanLimit=rmsLimit*.72;
      if (rms>rmsLimit || meanAbs>meanLimit || maxAbs>rmsLimit*4.5) {
        rejectedRegions++;
        continue;
      }

      const verts=new Uint32Array(uniqueCount);
      const membership=new Uint8Array(uniqueCount);
      for(let i=0;i<uniqueCount;i++) {
        const v=touchedVerts[i]; verts[i]=v;
        const deg=vertexDegree[v]||1;
        membership[i]=Math.min(255,Math.round(255*incidence[v]/deg));
      }
      regions.push({
        faces:tail, area:totalArea, normal:[pnx,pny,pnz], center:[cx,cy,cz], d:planeD,
        verts, membership, rms, typicalEdge
      });
    }

    let parallelFamilies=0;
    if (alignParallel && regions.length>1) {
      const cosParallel=Math.cos(4*Math.PI/180);
      const families=[];
      // Grandes primero: las caras extensas actúan como ancla para las pequeñas.
      const order=regions.map((_,i)=>i).sort((a,b)=>regions[b].area-regions[a].area);
      for(const ri of order) {
        const r=regions[ri];
        let best=-1,bestDot=cosParallel;
        for(let fi=0;fi<families.length;fi++) {
          const f=families[fi];
          const dot=Math.abs(r.normal[0]*f.n[0]+r.normal[1]*f.n[1]+r.normal[2]*f.n[2]);
          if(dot>bestDot){bestDot=dot;best=fi;}
        }
        if(best<0) {
          families.push({n:r.normal.slice(),members:[ri],weight:r.area});
        } else {
          const f=families[best];
          const sign=(r.normal[0]*f.n[0]+r.normal[1]*f.n[1]+r.normal[2]*f.n[2])>=0?1:-1;
          const w=r.area;
          let x=f.n[0]*f.weight+r.normal[0]*sign*w;
          let y=f.n[1]*f.weight+r.normal[1]*sign*w;
          let z=f.n[2]*f.weight+r.normal[2]*sign*w;
          const l=Math.hypot(x,y,z)||1;
          f.n=[x/l,y/l,z/l]; f.weight+=w; f.members.push(ri);
        }
      }
      parallelFamilies=families.filter(f=>f.members.length>1).length;
      for(const f of families) {
        if(f.members.length<2) continue;
        for(const ri of f.members) {
          const r=regions[ri];
          const sign=(r.normal[0]*f.n[0]+r.normal[1]*f.n[1]+r.normal[2]*f.n[2])>=0?1:-1;
          r.normal=[f.n[0]*sign,f.n[1]*sign,f.n[2]*sign];
          r.d=r.center[0]*r.normal[0]+r.center[1]*r.normal[1]+r.center[2]*r.normal[2];
        }
      }
    }

    const output=new Float32Array(positions);
    const bestRegionSize=new Uint32Array(vertexCount);
    const minInterior=0.34+edgeProtection*.58;
    let verticesMoved=0;

    for(const r of regions) {
      const [nx,ny,nz]=r.normal;
      for(let i=0;i<r.verts.length;i++) {
        const v=r.verts[i];
        if(bestRegionSize[v]>=r.faces) continue;
        const ratio=r.membership[i]/255;
        if(ratio<minInterior) continue;
        // En el borde la corrección entra progresivamente; en el interior llega a la fuerza solicitada.
        const edgeWeight=Math.min(1,(ratio-minInterior)/Math.max(1e-6,1-minInterior));
        const w=strength*(0.25+0.75*edgeWeight);
        const vi=v*3;
        const dist=positions[vi]*nx+positions[vi+1]*ny+positions[vi+2]*nz-r.d;
        output[vi]=positions[vi]-nx*dist*w;
        output[vi+1]=positions[vi+1]-ny*dist*w;
        output[vi+2]=positions[vi+2]-nz*dist*w;
        if(bestRegionSize[v]===0 && Math.abs(dist*w)>1e-8) verticesMoved++;
        bestRegionSize[v]=r.faces;
      }
    }

    self.postMessage({
      type:'done',positions:output,regions:regions.length,rejectedRegions,parallelFamilies,
      verticesMoved,elapsedMs:performance.now()-started
    },[output.buffer]);
  } catch (err) {
    self.postMessage({type:'error',message:err?.message || String(err)});
  }
};

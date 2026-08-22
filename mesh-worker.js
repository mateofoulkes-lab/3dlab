const rad=d=>d*Math.PI/180;
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const norm=v=>{const l=Math.hypot(v[0],v[1],v[2])||1;return[v[0]/l,v[1]/l,v[2]/l]};
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const absDot=(a,b)=>Math.abs(dot(a,b));

self.onmessage=({data})=>{
 if(data.type!=='reconstruct')return;
 try{
  const started=performance.now();
  const {positions,indices,strength,angleDeg,minRegionFaces,planeConfidence,edgeProtection,alignParallel,orthogonalSnap,sharpIntersections,detectThickness,allowTiltedPlanes}=data;
  const vertexCount=positions.length/3,faceCount=indices.length/3;if(!vertexCount||!faceCount)throw new Error('Mesh vacío o inválido.');
  postMessage({type:'progress',stage:'Calculando normales y escala'});

  const normals=new Float32Array(faceCount*3),areas=new Float32Array(faceCount),centroids=new Float32Array(faceCount*3);let totalArea=0;
  for(let f=0;f<faceCount;f++){
   const a=indices[f*3]*3,b=indices[f*3+1]*3,c=indices[f*3+2]*3;
   const ax=positions[a],ay=positions[a+1],az=positions[a+2],abx=positions[b]-ax,aby=positions[b+1]-ay,abz=positions[b+2]-az,acx=positions[c]-ax,acy=positions[c+1]-ay,acz=positions[c+2]-az;
   let nx=aby*acz-abz*acy,ny=abz*acx-abx*acz,nz=abx*acy-aby*acx;const len=Math.hypot(nx,ny,nz)||1,area=len*.5;areas[f]=area;totalArea+=area;nx/=len;ny/=len;nz/=len;normals[f*3]=nx;normals[f*3+1]=ny;normals[f*3+2]=nz;centroids[f*3]=(positions[a]+positions[b]+positions[c])/3;centroids[f*3+1]=(positions[a+1]+positions[b+1]+positions[c+1])/3;centroids[f*3+2]=(positions[a+2]+positions[b+2]+positions[c+2])/3;
  }
  const typicalEdge=Math.sqrt(Math.max(totalArea/faceCount,1e-16)*2);

  postMessage({type:'progress',stage:'Construyendo adyacencia'});
  const neighbors=new Int32Array(faceCount*3);neighbors.fill(-1);const neighborCount=new Uint8Array(faceCount),edgeOwner=new Map();
  const edgeKey=(a,b)=>{if(a>b){const t=a;a=b;b=t}return a*vertexCount+b};
  for(let f=0;f<faceCount;f++){const a=indices[f*3],b=indices[f*3+1],c=indices[f*3+2],es=[[a,b],[b,c],[c,a]];for(const edge of es){const k=edgeKey(edge[0],edge[1]),other=edgeOwner.get(k);if(other===undefined)edgeOwner.set(k,f);else if(other>=0){const ca=neighborCount[f],cb=neighborCount[other];if(ca<3&&cb<3){neighbors[f*3+ca]=other;neighborCount[f]=ca+1;neighbors[other*3+cb]=f;neighborCount[other]=cb+1}edgeOwner.set(k,-1)}}}edgeOwner.clear();

  postMessage({type:'progress',stage:'Detectando planos estructurales'});
  const visited=new Uint8Array(faceCount),faceRegion=new Int32Array(faceCount);faceRegion.fill(-1);const queue=new Int32Array(faceCount);let regions=[],rejectedRegions=0;
  const cosLimit=Math.cos(rad(angleDeg)),maxRmsFactor=.22+(1-planeConfidence)*1.05;
  for(let seed=0;seed<faceCount;seed++){
   if(visited[seed])continue;visited[seed]=1;let head=0,tail=1;queue[0]=seed;const sn=[normals[seed*3],normals[seed*3+1],normals[seed*3+2]],sc=[centroids[seed*3],centroids[seed*3+1],centroids[seed*3+2]],seedD=dot(sn,sc);
   while(head<tail){const f=queue[head++];for(let k=0;k<neighborCount[f];k++){const nb=neighbors[f*3+k];if(nb<0||visited[nb])continue;const nn=[normals[nb*3],normals[nb*3+1],normals[nb*3+2]];if(dot(nn,sn)<cosLimit)continue;const cc=[centroids[nb*3],centroids[nb*3+1],centroids[nb*3+2]];if(Math.abs(dot(sn,cc)-seedD)>typicalEdge*(1.2+(1-planeConfidence)*3.5))continue;visited[nb]=1;queue[tail++]=nb}}
   if(tail<minRegionFaces){rejectedRegions++;continue}
   let sx=0,sy=0,sz=0,cx=0,cy=0,cz=0,aw=0;for(let q=0;q<tail;q++){const f=queue[q],w=areas[f]||1;sx+=normals[f*3]*w;sy+=normals[f*3+1]*w;sz+=normals[f*3+2]*w;cx+=centroids[f*3]*w;cy+=centroids[f*3+1]*w;cz+=centroids[f*3+2]*w;aw+=w}
   const n=norm([sx,sy,sz]),center=[cx/aw,cy/aw,cz/aw],d=dot(n,center);let sq=0,samples=0,stride=Math.max(1,Math.floor(tail/3000));for(let q=0;q<tail;q+=stride){const f=queue[q];for(let j=0;j<3;j++){const vi=indices[f*3+j]*3,dist=positions[vi]*n[0]+positions[vi+1]*n[1]+positions[vi+2]*n[2]-d;sq+=dist*dist;samples++}}
   const rms=Math.sqrt(sq/Math.max(samples,1));if(rms>typicalEdge*maxRmsFactor){rejectedRegions++;continue}const id=regions.length,faces=new Int32Array(tail);for(let q=0;q<tail;q++){faces[q]=queue[q];faceRegion[queue[q]]=id}regions.push({id,n,d,area:aw,rms,faces});
  }

  let extrusionAxis=null,tiltedRejected=0;
  if(!allowTiltedPlanes&&regions.length){
   postMessage({type:'progress',stage:'Infiriendo eje de extrusión 2.5D'});
   const dominant=regions.reduce((a,b)=>b.area>a.area?b:a);extrusionAxis=dominant.n.slice();
   const capCos=Math.cos(rad(16)),wallSin=Math.sin(rad(16));
   const kept=[];faceRegion.fill(-1);
   for(const r of regions){const ad=absDot(r.n,extrusionAxis);if(ad>=capCos||ad<=wallSin){r.id=kept.length;kept.push(r);for(const f of r.faces)faceRegion[f]=r.id}else tiltedRejected++}
   rejectedRegions+=tiltedRejected;regions=kept;
  }

  postMessage({type:'progress',stage:'Buscando familias paralelas y escuadras'});
  const families=[],famCos=Math.cos(rad(Math.max(5,Math.min(12,angleDeg+2)))),order=regions.map((r,i)=>i).sort((a,b)=>regions[b].area-regions[a].area);
  for(const ri of order){const r=regions[ri];let best=-1,bestDot=0;for(let i=0;i<families.length;i++){const ad=absDot(r.n,families[i].n);if(ad>famCos&&ad>bestDot){best=i;bestDot=ad}}if(best<0)families.push({n:r.n.slice(),area:r.area,members:[ri]});else{const fam=families[best],sgn=dot(r.n,fam.n)>=0?1:-1,w=fam.area+r.area;fam.n=norm([(fam.n[0]*fam.area+r.n[0]*r.area*sgn)/w,(fam.n[1]*fam.area+r.n[1]*r.area*sgn)/w,(fam.n[2]*fam.area+r.n[2]*r.area*sgn)/w]);fam.area=w;fam.members.push(ri)}}

  if(orthogonalSnap&&families.length>=2){
   if(extrusionAxis&&!allowTiltedPlanes){
    const axis=norm(extrusionAxis);
    for(const fam of families){const ad=absDot(fam.n,axis);if(ad>Math.cos(rad(20))){const s=dot(fam.n,axis)>=0?1:-1;fam.n=[axis[0]*s,axis[1]*s,axis[2]*s]}else if(ad<Math.sin(rad(20))){const proj=dot(fam.n,axis);fam.n=norm([fam.n[0]-axis[0]*proj,fam.n[1]-axis[1]*proj,fam.n[2]-axis[2]*proj])}}
   }else{
    families.sort((a,b)=>b.area-a.area);const a=norm(families[0].n);let bi=-1,bscore=999;for(let i=1;i<families.length;i++){const s=Math.abs(dot(a,families[i].n));if(s<bscore){bscore=s;bi=i}}if(bi>=0&&bscore<Math.sin(rad(18))){let b=families[bi].n.slice();const proj=dot(b,a);b=norm([b[0]-a[0]*proj,b[1]-a[1]*proj,b[2]-a[2]*proj]);const c=norm(cross(a,b)),axes=[a,b,c];for(const fam of families){let bestAxis=null,best=0,sign=1;for(const ax of axes){const dd=dot(fam.n,ax),ad=Math.abs(dd);if(ad>best){best=ad;bestAxis=ax;sign=dd>=0?1:-1}}if(best>Math.cos(rad(18)))fam.n=[bestAxis[0]*sign,bestAxis[1]*sign,bestAxis[2]*sign]}}
   }
  }
  if(alignParallel){for(const fam of families)for(const ri of fam.members){const r=regions[ri],sgn=dot(r.n,fam.n)>=0?1:-1;r.n=[fam.n[0]*sgn,fam.n[1]*sgn,fam.n[2]*sgn];let sum=0,w=0;for(const f of r.faces){const a=areas[f]||1;sum+=dot(r.n,[centroids[f*3],centroids[f*3+1],centroids[f*3+2]])*a;w+=a}r.d=sum/Math.max(w,1)}}

  postMessage({type:'progress',stage:'Reconstruyendo caras y aristas rectas'});
  const incident=Array.from({length:vertexCount},()=>[]);for(let f=0;f<faceCount;f++){const rid=faceRegion[f];if(rid<0)continue;for(let j=0;j<3;j++){const v=indices[f*3+j],arr=incident[v];if(!arr.includes(rid))arr.push(rid)}}
  const output=new Float32Array(positions);let verticesMoved=0,orthogonalEdges=0;
  const projectPlane=(p,r)=>{const er=dot(r.n,p)-r.d;return[p[0]-r.n[0]*er,p[1]-r.n[1]*er,p[2]-r.n[2]*er]};
  const projectTwo=(p,r1,r2)=>{const a=dot(r1.n,r1.n),b=dot(r1.n,r2.n),c=dot(r2.n,r2.n),det=a*c-b*b;if(Math.abs(det)<1e-8)return projectPlane(p,r1);const e1=dot(r1.n,p)-r1.d,e2=dot(r2.n,p)-r2.d,l1=(c*e1-b*e2)/det,l2=(-b*e1+a*e2)/det;return[p[0]-r1.n[0]*l1-r2.n[0]*l2,p[1]-r1.n[1]*l1-r2.n[1]*l2,p[2]-r1.n[2]*l1-r2.n[2]*l2]};
  for(let v=0;v<vertexCount;v++){const rs=incident[v];if(!rs.length)continue;const vi=v*3,p=[positions[vi],positions[vi+1],positions[vi+2]];let target=p;if(rs.length===1)target=projectPlane(p,regions[rs[0]]);else{const ranked=rs.slice().sort((a,b)=>regions[b].area-regions[a].area),r1=regions[ranked[0]],r2=regions[ranked[1]],ang=Math.acos(clamp(absDot(r1.n,r2.n),-1,1))*180/Math.PI;if(sharpIntersections&&ang>72&&ang<108){target=projectTwo(p,r1,r2);orthogonalEdges++}else{const p1=projectPlane(p,r1),p2=projectPlane(p,r2);target=[(p1[0]+p2[0])/2,(p1[1]+p2[1])/2,(p1[2]+p2[2])/2]}if(rs.length>2){const preserve=edgeProtection;target=[p[0]+(target[0]-p[0])*(1-preserve),p[1]+(target[1]-p[1])*(1-preserve),p[2]+(target[2]-p[2])*(1-preserve)]}}const nx=p[0]+(target[0]-p[0])*strength,ny=p[1]+(target[1]-p[1])*strength,nz=p[2]+(target[2]-p[2])*strength;if(Math.hypot(nx-p[0],ny-p[1],nz-p[2])>1e-8)verticesMoved++;output[vi]=nx;output[vi+1]=ny;output[vi+2]=nz}
  orthogonalEdges=Math.round(orthogonalEdges/2);

  let thicknesses=[];if(detectThickness&&regions.length>1){postMessage({type:'progress',stage:'Estimando espesores nominales'});const vals=[];let pairs=0;for(let i=0;i<regions.length&&pairs<12000;i++)for(let j=i+1;j<regions.length&&pairs<12000;j++){const a=regions[i],b=regions[j];if(absDot(a.n,b.n)<Math.cos(rad(6)))continue;let bd=b.d;if(dot(a.n,b.n)<0)bd=-bd;const dist=Math.abs(a.d-bd);if(dist>typicalEdge*.4){vals.push(dist);pairs++}}if(vals.length){vals.sort((a,b)=>a-b);const bucket=Math.max(typicalEdge*.35,1e-6),groups=[];for(const v of vals){const g=groups.find(x=>Math.abs(x.mean-v)<bucket);if(g){g.sum+=v;g.count++;g.mean=g.sum/g.count}else groups.push({sum:v,count:1,mean:v})}groups.sort((a,b)=>b.count-a.count);thicknesses=groups.slice(0,3).filter(g=>g.count>=2).map(g=>formatLength(g.mean))}}

  postMessage({type:'done',positions:output,regions:regions.length,rejectedRegions,parallelFamilies:families.length,orthogonalEdges,verticesMoved,thicknesses,extrusionAxis,tiltedRejected,elapsedMs:performance.now()-started},[output.buffer]);
 }catch(err){postMessage({type:'error',message:err?.message||String(err)})}
};
function formatLength(v){if(!isFinite(v))return'?';if(v>=100)return v.toFixed(1);if(v>=10)return v.toFixed(2);if(v>=1)return v.toFixed(3);return v.toPrecision(3)}

const rad=d=>d*Math.PI/180;
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm=v=>{const l=Math.hypot(v[0],v[1],v[2])||1;return[v[0]/l,v[1]/l,v[2]/l]};
const absDot=(a,b)=>Math.abs(dot(a,b));
const add=(a,b)=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]];
const scale=(a,s)=>[a[0]*s,a[1]*s,a[2]*s];
const sub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const dist2=(a,b)=>{const x=a[0]-b[0],y=a[1]-b[1];return x*x+y*y};

self.onmessage=({data})=>{
 if(data.type!=='rebuild')return;
 try{
  const started=performance.now();
  const {positions,indices,angleDeg,minRegionFaces,planeConfidence,simplifyFactor,allowTiltedPlanes}=data;
  const vertexCount=positions.length/3,faceCount=indices.length/3;
  if(!vertexCount||!faceCount)throw new Error('Mesh vacío o inválido.');
  postMessage({type:'progress',stage:'Midiendo superficies del scan'});

  const normals=new Float32Array(faceCount*3),areas=new Float32Array(faceCount),centroids=new Float32Array(faceCount*3);
  let totalArea=0;
  for(let f=0;f<faceCount;f++){
   const ia=indices[f*3]*3,ib=indices[f*3+1]*3,ic=indices[f*3+2]*3;
   const ax=positions[ia],ay=positions[ia+1],az=positions[ia+2];
   const abx=positions[ib]-ax,aby=positions[ib+1]-ay,abz=positions[ib+2]-az;
   const acx=positions[ic]-ax,acy=positions[ic+1]-ay,acz=positions[ic+2]-az;
   let nx=aby*acz-abz*acy,ny=abz*acx-abx*acz,nz=abx*acy-aby*acx;
   const len=Math.hypot(nx,ny,nz)||1,a=len*.5;areas[f]=a;totalArea+=a;nx/=len;ny/=len;nz/=len;
   normals[f*3]=nx;normals[f*3+1]=ny;normals[f*3+2]=nz;
   centroids[f*3]=(positions[ia]+positions[ib]+positions[ic])/3;
   centroids[f*3+1]=(positions[ia+1]+positions[ib+1]+positions[ic+1])/3;
   centroids[f*3+2]=(positions[ia+2]+positions[ib+2]+positions[ic+2])/3;
  }
  const typicalEdge=Math.sqrt(Math.max(totalArea/faceCount,1e-16)*2);

  postMessage({type:'progress',stage:'Detectando parches planos'});
  const neighbors=new Int32Array(faceCount*3);neighbors.fill(-1);const ncount=new Uint8Array(faceCount),edgeOwner=new Map();
  const key=(a,b)=>{if(a>b){const t=a;a=b;b=t}return a*vertexCount+b};
  for(let f=0;f<faceCount;f++){
   const a=indices[f*3],b=indices[f*3+1],c=indices[f*3+2];
   for(const [x,y] of [[a,b],[b,c],[c,a]]){const k=key(x,y),o=edgeOwner.get(k);if(o===undefined)edgeOwner.set(k,f);else if(o>=0){const ca=ncount[f],cb=ncount[o];if(ca<3&&cb<3){neighbors[f*3+ca]=o;ncount[f]=ca+1;neighbors[o*3+cb]=f;ncount[o]=cb+1}edgeOwner.set(k,-1)}}
  }
  edgeOwner.clear();

  const visited=new Uint8Array(faceCount),queue=new Int32Array(faceCount),regions=[];
  const cosLimit=Math.cos(rad(angleDeg));
  const growDistance=typicalEdge*(1.0+(1-planeConfidence)*2.2);
  for(let seed=0;seed<faceCount;seed++){
   if(visited[seed])continue;visited[seed]=1;let head=0,tail=1;queue[0]=seed;
   const sn=[normals[seed*3],normals[seed*3+1],normals[seed*3+2]],sc=[centroids[seed*3],centroids[seed*3+1],centroids[seed*3+2]],sd=dot(sn,sc);
   while(head<tail){const f=queue[head++];for(let k=0;k<ncount[f];k++){const nb=neighbors[f*3+k];if(nb<0||visited[nb])continue;const nn=[normals[nb*3],normals[nb*3+1],normals[nb*3+2]];if(dot(nn,sn)<cosLimit)continue;const cc=[centroids[nb*3],centroids[nb*3+1],centroids[nb*3+2]];if(Math.abs(dot(sn,cc)-sd)>growDistance)continue;visited[nb]=1;queue[tail++]=nb}}
   if(tail<minRegionFaces)continue;
   let sx=0,sy=0,sz=0,cx=0,cy=0,cz=0,aw=0;
   for(let q=0;q<tail;q++){const f=queue[q],w=areas[f]||1;sx+=normals[f*3]*w;sy+=normals[f*3+1]*w;sz+=normals[f*3+2]*w;cx+=centroids[f*3]*w;cy+=centroids[f*3+1]*w;cz+=centroids[f*3+2]*w;aw+=w}
   const n=norm([sx,sy,sz]),center=[cx/aw,cy/aw,cz/aw],d=dot(n,center);
   let sq=0,samples=0;const stride=Math.max(1,Math.floor(tail/2500));
   for(let q=0;q<tail;q+=stride){const f=queue[q];for(let j=0;j<3;j++){const vi=indices[f*3+j]*3,dd=positions[vi]*n[0]+positions[vi+1]*n[1]+positions[vi+2]*n[2]-d;sq+=dd*dd;samples++}}
   const rms=Math.sqrt(sq/Math.max(samples,1));
   const maxRms=typicalEdge*(.18+(1-planeConfidence)*.75);
   if(rms>maxRms)continue;
   const faces=new Int32Array(tail);for(let q=0;q<tail;q++)faces[q]=queue[q];
   regions.push({n,d,area:aw,rms,faces});
  }
  if(!regions.length)throw new Error('No encontré superficies planas suficientemente confiables. Bajá Exigencia o Plano mínimo.');

  regions.sort((a,b)=>b.area-a.area);
  let z=norm(regions[0].n);
  // Elegimos una segunda familia grande perpendicular para construir un marco 2.5D ortogonal.
  let x=null;
  for(let i=1;i<regions.length;i++){const n=regions[i].n;if(absDot(n,z)<Math.sin(rad(20))){x=norm(sub(n,scale(z,dot(n,z))));break}}
  if(!x){const helper=Math.abs(z[0])<.8?[1,0,0]:[0,1,0];x=norm(cross(helper,z))}
  let y=norm(cross(z,x));x=norm(cross(y,z));

  postMessage({type:'progress',stage:'Vectorizando contornos y descartando topología original'});
  const patches=[];let tiltedRejected=0,openRejected=0,rawTriangles=0;
  const axisTol=rad(18),cosAxis=Math.cos(axisTol),sinAxis=Math.sin(axisTol);
  const simplifyTol=typicalEdge*(.45+Math.max(0,simplifyFactor)*4.0);

  for(const r of regions){
   let n=r.n.slice(),kind='tilted';const dz=absDot(n,z);
   if(dz>cosAxis){const s=dot(n,z)>=0?1:-1;n=scale(z,s);kind='horizontal'}
   else if(dz<sinAxis){const candidates=[x,scale(x,-1),y,scale(y,-1)];let best=candidates[0],bd=-1;for(const c of candidates){const d=dot(n,c);if(d>bd){bd=d;best=c}}n=best.slice();kind='vertical'}
   else if(!allowTiltedPlanes){tiltedRejected++;continue}

   // Recalcula el offset sobre la normal ideal usando mediana de centroides muestreados.
   const ds=[];const stride=Math.max(1,Math.floor(r.faces.length/1500));
   for(let q=0;q<r.faces.length;q+=stride){const f=r.faces[q],c=[centroids[f*3],centroids[f*3+1],centroids[f*3+2]];ds.push(dot(n,c))}
   ds.sort((a,b)=>a-b);const d=ds[Math.floor(ds.length/2)]??r.d;

   let u,v;
   if(kind==='horizontal'){u=x.slice();v=y.slice()}
   else if(kind==='vertical'){u=z.slice();v=norm(cross(n,u));if(Math.hypot(v[0],v[1],v[2])<.5){u=y.slice();v=norm(cross(n,u))}}
   else {const h=Math.abs(n[0])<.8?[1,0,0]:[0,1,0];u=norm(cross(h,n));v=norm(cross(n,u))}
   const origin=scale(n,d);

   const em=new Map();
   for(const f of r.faces){rawTriangles++;const a=indices[f*3],b=indices[f*3+1],c=indices[f*3+2];for(const [p,q] of [[a,b],[b,c],[c,a]]){const k=key(p,q),cur=em.get(k);if(cur)cur.count++;else em.set(k,{a:p,b:q,count:1})}}
   const adj=new Map();
   for(const ed of em.values())if(ed.count===1){if(!adj.has(ed.a))adj.set(ed.a,[]);if(!adj.has(ed.b))adj.set(ed.b,[]);adj.get(ed.a).push(ed.b);adj.get(ed.b).push(ed.a)}
   const usable=new Set([...adj.entries()].filter(([,ns])=>ns.length===2).map(([vv])=>vv));
   const seenEdges=new Set(),loops=[];
   const ekey=(a,b)=>a<b?`${a}:${b}`:`${b}:${a}`;
   for(const start of usable){for(const first of adj.get(start)||[]){const sk=ekey(start,first);if(seenEdges.has(sk)||!usable.has(first))continue;const loop=[start];let prev=start,cur=first,guard=0,closed=false;seenEdges.add(sk);while(guard++<20000){loop.push(cur);const ns=adj.get(cur)||[];if(ns.length!==2)break;const next=ns[0]===prev?ns[1]:ns[0];const kk=ekey(cur,next);if(next===start){seenEdges.add(kk);closed=true;break}if(seenEdges.has(kk)||!usable.has(next))break;seenEdges.add(kk);prev=cur;cur=next}if(closed&&loop.length>=3){let pts=loop.map(vi=>{const i=vi*3,p=[positions[i],positions[i+1],positions[i+2]],pp=sub(p,scale(n,dot(n,p)-d)),rel=sub(pp,origin);return[dot(rel,u),dot(rel,v)]});pts=simplifyClosed(pts,simplifyTol);if(pts.length>=3&&Math.abs(polyArea(pts))>typicalEdge*typicalEdge*2)loops.push(pts)}}}
   if(!loops.length){openRejected++;continue}
   loops.sort((a,b)=>Math.abs(polyArea(b))-Math.abs(polyArea(a)));
   // Evitamos agujeros ajenos: solo loops cuyo centro cae dentro del loop exterior se guardan como holes.
   const outer=loops[0],holes=[];for(let i=1;i<loops.length;i++){const c=centroid2(loops[i]);if(pointInPoly(c,outer))holes.push(loops[i])}
   patches.push({kind,n,d,origin,u,v,outer,holes,area:r.area});
  }
  if(!patches.length)throw new Error('Encontré planos, pero no pude obtener contornos cerrados confiables. Probá bajar Simplificación o permitir planos inclinados.');

  postMessage({type:'done',patches,stats:{patches:patches.length,sourceRegions:regions.length,tiltedRejected,openRejected,rawTriangles,elapsedMs:performance.now()-started,axis:z}},[]);
 }catch(err){postMessage({type:'error',message:err?.message||String(err)})}
};

function polyArea(p){let a=0;for(let i=0,j=p.length-1;i<p.length;j=i++)a+=(p[j][0]*p[i][1]-p[i][0]*p[j][1]);return a*.5}
function centroid2(p){let x=0,y=0;for(const q of p){x+=q[0];y+=q[1]}return[x/p.length,y/p.length]}
function pointInPoly(pt,p){let inside=false;for(let i=0,j=p.length-1;i<p.length;j=i++){const xi=p[i][0],yi=p[i][1],xj=p[j][0],yj=p[j][1];const hit=((yi>pt[1])!==(yj>pt[1]))&&(pt[0]<(xj-xi)*(pt[1]-yi)/(yj-yi+1e-20)+xi);if(hit)inside=!inside}return inside}
function pointSegDistSq(p,a,b){const vx=b[0]-a[0],vy=b[1]-a[1],wx=p[0]-a[0],wy=p[1]-a[1],c=vx*vx+vy*vy;if(c<1e-20)return dist2(p,a);let t=(wx*vx+wy*vy)/c;t=Math.max(0,Math.min(1,t));const q=[a[0]+vx*t,a[1]+vy*t];return dist2(p,q)}
function simplifyClosed(points,tol){if(points.length<5)return points;let pts=points.slice(),changed=true,pass=0,t2=tol*tol;while(changed&&pts.length>4&&pass++<8){changed=false;const out=[];for(let i=0;i<pts.length;i++){const a=pts[(i-1+pts.length)%pts.length],b=pts[i],c=pts[(i+1)%pts.length];if(pointSegDistSq(b,a,c)<t2){changed=true;continue}out.push(b)}pts=out}return pts}

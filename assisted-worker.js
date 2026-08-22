const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm=v=>{const l=Math.hypot(v[0],v[1],v[2])||1;return[v[0]/l,v[1]/l,v[2]/l]};
const sub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const scale=(a,s)=>[a[0]*s,a[1]*s,a[2]*s];

self.onmessage=({data})=>{
  if(data.type!=='extractPlane') return;
  try{
    const {positions,indices,axis,coord,seedFaces,angleDeg=18,distanceTol,simplifyTol} = data;
    const faceCount=indices.length/3, vertexCount=positions.length/3;
    if(!seedFaces?.length) throw new Error('Seleccioná al menos una cara del scan.');
    const ax=norm(axis);
    const cosLimit=Math.cos(angleDeg*Math.PI/180);

    postMessage({type:'progress',stage:'Construyendo vecindad de triángulos'});
    const normals=new Float32Array(faceCount*3), centroids=new Float32Array(faceCount*3);
    let meanEdge=0;
    for(let f=0;f<faceCount;f++){
      const ia=indices[f*3]*3,ib=indices[f*3+1]*3,ic=indices[f*3+2]*3;
      const a=[positions[ia],positions[ia+1],positions[ia+2]], b=[positions[ib],positions[ib+1],positions[ib+2]], c=[positions[ic],positions[ic+1],positions[ic+2]];
      const ab=sub(b,a),ac=sub(c,a),n=norm(cross(ab,ac));
      normals[f*3]=n[0];normals[f*3+1]=n[1];normals[f*3+2]=n[2];
      centroids[f*3]=(a[0]+b[0]+c[0])/3;centroids[f*3+1]=(a[1]+b[1]+c[1])/3;centroids[f*3+2]=(a[2]+b[2]+c[2])/3;
      meanEdge+=(Math.hypot(...sub(a,b))+Math.hypot(...sub(b,c))+Math.hypot(...sub(c,a)))/3;
    }
    meanEdge/=Math.max(1,faceCount);
    const tol=distanceTol||meanEdge*1.7;

    const neighbors=new Int32Array(faceCount*3);neighbors.fill(-1);const counts=new Uint8Array(faceCount), edgeOwner=new Map();
    const edgeKey=(a,b)=>{if(a>b){const t=a;a=b;b=t}return a*vertexCount+b};
    for(let f=0;f<faceCount;f++){
      const a=indices[f*3],b=indices[f*3+1],c=indices[f*3+2];
      for(const [x,y] of [[a,b],[b,c],[c,a]]){
        const k=edgeKey(x,y),o=edgeOwner.get(k);
        if(o===undefined) edgeOwner.set(k,f);
        else if(o>=0){if(counts[f]<3&&counts[o]<3){neighbors[f*3+counts[f]++]=o;neighbors[o*3+counts[o]++]=f}edgeOwner.set(k,-1)}
      }
    }

    postMessage({type:'progress',stage:'Expandiendo desde tus selecciones'});
    const accepted=new Uint8Array(faceCount), queued=new Uint8Array(faceCount), queue=new Int32Array(faceCount);let head=0,tail=0;
    for(const sf of seedFaces){if(sf>=0&&sf<faceCount&&!queued[sf]){queued[sf]=1;queue[tail++]=sf}}
    while(head<tail){
      const f=queue[head++],n=[normals[f*3],normals[f*3+1],normals[f*3+2]],c=[centroids[f*3],centroids[f*3+1],centroids[f*3+2]];
      const aligned=Math.abs(dot(n,ax))>=cosLimit, near=Math.abs(dot(c,ax)-coord)<=tol;
      if(!aligned||!near) continue;
      accepted[f]=1;
      for(let k=0;k<counts[f];k++){const nb=neighbors[f*3+k];if(nb>=0&&!queued[nb]){queued[nb]=1;queue[tail++]=nb}}
    }

    let acceptedCount=0;for(const v of accepted)acceptedCount+=v;
    if(acceptedCount<3) throw new Error('No pude expandir una región plana desde esas caras. Probá tocar una zona más limpia.');

    postMessage({type:'progress',stage:'Extrayendo contorno limpio'});
    const boundary=new Map();
    for(let f=0;f<faceCount;f++) if(accepted[f]){
      const a=indices[f*3],b=indices[f*3+1],c=indices[f*3+2];
      for(const [x,y] of [[a,b],[b,c],[c,a]]){const k=edgeKey(x,y),rec=boundary.get(k);if(rec)rec.count++;else boundary.set(k,{a:x,b:y,count:1})}
    }
    const adj=new Map();
    for(const e of boundary.values()) if(e.count===1){if(!adj.has(e.a))adj.set(e.a,[]);if(!adj.has(e.b))adj.set(e.b,[]);adj.get(e.a).push(e.b);adj.get(e.b).push(e.a)}

    const helper=Math.abs(ax[2])<.9?[0,0,1]:[1,0,0],u=norm(cross(helper,ax)),v=norm(cross(ax,u)),origin=scale(ax,coord);
    const p2=vi=>{const i=vi*3,p=[positions[i],positions[i+1],positions[i+2]],r=sub(p,origin);return[dot(r,u),dot(r,v)]};
    const used=new Set(), loops=[];
    const ek=(a,b)=>a<b?`${a}:${b}`:`${b}:${a}`;
    for(const [start,ns0] of adj){
      for(const first of ns0){const k0=ek(start,first);if(used.has(k0))continue;let prev=start,cur=first,loop=[start],closed=false,guard=0;used.add(k0);
        while(guard++<30000){loop.push(cur);const ns=adj.get(cur)||[];if(ns.length<1)break;let next=ns.find(n=>n!==prev&&!used.has(ek(cur,n)));if(next===undefined)next=ns.find(n=>n!==prev);if(next===undefined)break;const kk=ek(cur,next);if(next===start){used.add(kk);closed=true;break}if(used.has(kk))break;used.add(kk);prev=cur;cur=next}
        if(closed&&loop.length>=3){let pts=loop.map(p2);pts=simplifyClosed(pts,simplifyTol||meanEdge*.7);if(pts.length>=3&&Math.abs(area(pts))>meanEdge*meanEdge)loops.push(pts)}
      }
    }
    if(!loops.length) throw new Error('La región existe pero su borde no forma un contorno cerrado confiable.');
    loops.sort((a,b)=>Math.abs(area(b))-Math.abs(area(a)));
    const outer=loops[0],holes=[];for(let i=1;i<loops.length;i++){const c=centroid(loops[i]);if(pointInPoly(c,outer))holes.push(loops[i])}
    postMessage({type:'done',plane:{axis:ax,coord,origin,u,v,outer,holes,acceptedFaces:acceptedCount},stats:{acceptedFaces:acceptedCount,loops:loops.length}});
  }catch(err){postMessage({type:'error',message:err?.message||String(err)})}
};

function area(p){let a=0;for(let i=0,j=p.length-1;i<p.length;j=i++)a+=p[j][0]*p[i][1]-p[i][0]*p[j][1];return a*.5}
function centroid(p){let x=0,y=0;for(const q of p){x+=q[0];y+=q[1]}return[x/p.length,y/p.length]}
function pointInPoly(pt,p){let inside=false;for(let i=0,j=p.length-1;i<p.length;j=i++){const xi=p[i][0],yi=p[i][1],xj=p[j][0],yj=p[j][1];if(((yi>pt[1])!==(yj>pt[1]))&&(pt[0]<(xj-xi)*(pt[1]-yi)/(yj-yi+1e-20)+xi))inside=!inside}return inside}
function segDist2(p,a,b){const vx=b[0]-a[0],vy=b[1]-a[1],wx=p[0]-a[0],wy=p[1]-a[1],d=vx*vx+vy*vy;if(d<1e-20)return wx*wx+wy*wy;let t=(wx*vx+wy*vy)/d;t=Math.max(0,Math.min(1,t));const dx=p[0]-(a[0]+vx*t),dy=p[1]-(a[1]+vy*t);return dx*dx+dy*dy}
function simplifyClosed(pts,tol){if(pts.length<5)return pts;let out=pts.slice(),changed=true,pass=0,t2=tol*tol;while(changed&&out.length>4&&pass++<10){changed=false;const n=[];for(let i=0;i<out.length;i++){const a=out[(i-1+out.length)%out.length],b=out[i],c=out[(i+1)%out.length];if(segDist2(b,a,c)<t2){changed=true;continue}n.push(b)}out=n}return out}

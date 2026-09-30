import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';

export class Viewer {
  constructor(container,onSelect,onFace){
    this.el=container;this.onSelect=onSelect;this.onFace=onFace;this.objects=[];this.report=null;this.explode=24;this.showPCB=true;this.showLid=true;this.ghost=false;this.showPlanes=true;this.picking=false;
    this.scene=new THREE.Scene();this.scene.background=new THREE.Color('#f4f6f8');
    this.camera=new THREE.PerspectiveCamera(34,1,.1,10000);this.camera.up.set(0,0,1);this.camera.position.set(95,-110,125);
    this.renderer=new THREE.WebGLRenderer({antialias:true,alpha:false});this.renderer.setPixelRatio(Math.min(devicePixelRatio,2));this.renderer.localClippingEnabled=true;container.appendChild(this.renderer.domElement);
    this.controls=new OrbitControls(this.camera,this.renderer.domElement);this.controls.target.set(15,25,5);this.controls.enableDamping=false;this.controls.addEventListener('change',()=>this.render());
    this.scene.add(new THREE.HemisphereLight(0xffffff,0xb6bdc8,2.7));
    for(const [p,intensity]of [[[45,-65,100],2.4],[[-80,40,65],1.5]]){let l=new THREE.DirectionalLight(0xffffff,intensity);l.position.set(...p);this.scene.add(l);}
    this.group=new THREE.Group();this.scene.add(this.group);this.planeGroup=new THREE.Group();this.scene.add(this.planeGroup);this.anchorGroup=new THREE.Group();this.scene.add(this.anchorGroup);
    this.grid=new THREE.GridHelper(300,60,0xb4c0cf,0xdce2e9);this.grid.rotation.x=Math.PI/2;this.grid.position.z=-5.1;this.grid.material.transparent=true;this.grid.material.opacity=.56;this.scene.add(this.grid);
    const axes=new THREE.AxesHelper(12);axes.material.transparent=true;axes.material.opacity=.55;this.scene.add(axes);this.axes=axes;
    this.ray=new THREE.Raycaster();this.pointer=new THREE.Vector2();this.down=null;
    this.renderer.domElement.addEventListener('pointerdown',e=>this.down=[e.clientX,e.clientY]);
    this.renderer.domElement.addEventListener('pointerup',e=>{if(!this.down||Math.hypot(e.clientX-this.down[0],e.clientY-this.down[1])>5||e.button!==0)return;this.pick(e);});
    new ResizeObserver(()=>this.resize()).observe(container);this.resize();
  }
  resize(){const w=this.el.clientWidth,h=this.el.clientHeight;this.camera.aspect=w/h;this.camera.updateProjectionMatrix();this.renderer.setSize(w,h);this.render();}
  render(){this.renderer.render(this.scene,this.camera);}
  clear(group){for(const o of [...group.children]){group.remove(o);o.traverse(x=>{x.geometry?.dispose();if(Array.isArray(x.material))x.material.forEach(m=>m.dispose());else x.material?.dispose();});}}
  update(report,fit=false){
    this.clear(this.group);this.clear(this.planeGroup);this.objects=[];this.report=report;
    const colors={shell:'#c5cedc',lid:'#d8dfeb',pcb:'#4b9188',component:'#607087',reference:'#b58e5f'};
    for(const item of report.meshes){
      const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(item.vertices.flat(),3));g.setIndex(item.triangles.flat());g.computeVertexNormals();
      const mat=new THREE.MeshStandardMaterial({color:item.color||colors[item.kind]||'#8997ab',roughness:.73,metalness:.08,flatShading:true,side:THREE.DoubleSide});
      const m=new THREE.Mesh(g,mat);m.userData=item;m.userData.baseColor=item.color||colors[item.kind];this.group.add(m);this.objects.push(m);
      if(item.kind!=='reference'||item.triangles.length<18000){const edges=new THREE.LineSegments(new THREE.EdgesGeometry(g,28),new THREE.LineBasicMaterial({color:'#3e526b',transparent:true,opacity:item.kind==='component'?.15:.24}));m.add(edges);}
    }
    for(const p of report.planes){
      const basis=p.axes;const m=new THREE.Mesh(new THREE.PlaneGeometry(26,26),new THREE.MeshBasicMaterial({color:'#d5a554',transparent:true,opacity:.13,side:THREE.DoubleSide,depthWrite:false}));
      const matrix=new THREE.Matrix4().set(basis[0][0],basis[0][1],basis[0][2],p.origin[0],basis[1][0],basis[1][1],basis[1][2],p.origin[1],basis[2][0],basis[2][1],basis[2][2],p.origin[2],0,0,0,1);m.applyMatrix4(matrix);m.userData={kind:'plane',id:p.id};
      const edge=new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry),new THREE.LineBasicMaterial({color:'#c0944b',transparent:true,opacity:.65}));m.add(edge);this.planeGroup.add(m);
    }
    this.grid.position.z=report.metrics.bottom-.1;this.axes.position.z=0;this.visibility();if(fit)this.fit();this.select(this.selection);this.render();
  }
  visibility(){for(const m of this.objects){const k=m.userData.kind;m.visible=k==='pcb'||k==='component'?this.showPCB:k==='lid'?this.showLid:k==='reference'?m.userData.visible!==false:true;if(k==='lid')m.position.z=this.explode;if(k==='shell'){m.material.transparent=this.ghost;m.material.opacity=this.ghost?.22:1;m.material.depthWrite=!this.ghost;}}this.planeGroup.visible=this.showPlanes;this.render();}
  select(selection){this.selection=selection;for(const m of this.objects){const hit=selection&&(m.userData.id===selection.id||m.userData.refId===selection.id||m.userData.componentId===selection.id);m.material.color.set(hit?'#89b8fb':m.userData.baseColor);m.material.emissive.set(hit?'#17385e':'#000000');m.material.emissiveIntensity=hit?.12:0;}
    this.clear(this.anchorGroup);const p=[...(this.report?.planes||[]),...(this.report?.features||[])].find(p=>p.id===selection?.id&&p.origin&&p.axes);
    if(p){const b=p.axes,axes=new THREE.AxesHelper(9);axes.material.depthTest=false;axes.renderOrder=10;axes.applyMatrix4(new THREE.Matrix4().set(b[0][0],b[0][1],b[0][2],p.origin[0],b[1][0],b[1][1],b[1][2],p.origin[1],b[2][0],b[2][1],b[2][2],p.origin[2],0,0,0,1));this.anchorGroup.add(axes);}
    this.render();}
  fit(view='iso'){
    const met=this.report?.metrics||{xmin:0,xmax:31,ymin:0,ymax:54,bottom:-5,top:12};
    const center=new THREE.Vector3((met.xmin+met.xmax)/2,(met.ymin+met.ymax)/2,(met.top+met.bottom+(this.showLid?this.explode:0))/2);
    const size=Math.max(met.xmax-met.xmin,met.ymax-met.ymin,met.top-met.bottom+(this.showLid?this.explode:0));const d=size*2.8;
    const directions={iso:[1.15,-1.45,1.55],top:[0,.001,1],front:[0,-1,.08],right:[1,0,.08]};const dir=new THREE.Vector3(...(directions[view]||directions.iso)).normalize();
    this.controls.target.copy(center);this.camera.position.copy(center).addScaledVector(dir,d);this.camera.near=.1;this.camera.far=Math.max(1000,d*15);this.camera.updateProjectionMatrix();this.controls.update();this.render();
  }
  pick(e){
    const rect=this.renderer.domElement.getBoundingClientRect();this.pointer.set((e.clientX-rect.left)/rect.width*2-1,-(e.clientY-rect.top)/rect.height*2+1);this.ray.setFromCamera(this.pointer,this.camera);
    const hits=this.ray.intersectObjects(this.objects.filter(m=>m.visible&&(!this.picking||m.userData.boardAsset||['reference','component'].includes(m.userData.kind))),false);
    if(!hits.length)return;let hit=hits[0];if(this.ghost&&!this.picking)hit=hits.find(h=>h.object.userData.kind!=='shell')||hit;
    const item=hit.object.userData;
    if(this.picking){
      if(item.kind==='reference'||item.boardAsset){
        const face=item.faces.find(f=>hit.faceIndex>=f.start&&hit.faceIndex<f.start+f.count);
        if(!face||face.type!=='PLANE'){this.onFace({error:'Choose a flat STEP face. Mesh references and curved surfaces do not define a planar attachment.'});return;}
        this.onFace({kind:item.boardAsset?'boardFace':'face',ref:item.boardAsset||item.refId,body:item.body,face:{origin:face.origin,normal:face.normal,xDir:face.xDir,body:item.body,id:face.id},plane:'face'});
      }else if(item.kind==='component')this.onFace({kind:'component',ref:item.id,normal:hit.face.normal.toArray(),plane:'face'});
    }else this.onSelect({kind:item.kind,id:item.kind==='reference'?item.refId:item.componentId||item.id});
  }
}

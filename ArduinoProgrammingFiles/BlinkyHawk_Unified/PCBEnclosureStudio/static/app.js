import {Viewer} from './viewer.js';
const $=s=>document.querySelector(s),$$=s=>[...document.querySelectorAll(s)];
const escape=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const clone=p=>structuredClone(p),uid=()=>crypto.randomUUID(),icons=()=>window.lucide?.createIcons({attrs:{'stroke-width':1.6}});
let project=null,report=null,selection={kind:'enclosure',id:'enclosure'},past=[],future=[],revision=0,renderedRevision=-1,running=false,timer,toastTimer,pickTarget=null;
const viewer=new Viewer($('#viewport'),s=>select(s),a=>acceptFace(a));
const featureIcons={window:'rectangle-horizontal',hole:'circle',box:'box',cylinder:'cylinder',standoff:'cylinder',vent:'align-justify',text:'type',sketch:'pentagon'};
const featureNames={window:'Connector window',hole:'Round opening',box:'Rectangular boss',cylinder:'Round boss',standoff:'PCB standoff',vent:'Vent array',text:'Lettering',sketch:'Polygon feature'};
const ico=n=>`<i data-lucide="${n}"></i>`;
function toast(text){$('#toast').textContent=text;$('#toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('#toast').hidden=true,6500);}
async function api(url,options={}){const r=await fetch(url,options);if(!r.ok){let d;try{d=await r.json();}catch{d={detail:r.statusText};}throw Error(typeof d.detail==='string'?d.detail:JSON.stringify(d.detail));}return r;}
const jsonOptions=p=>({method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(p)});
function remember(){if(project){past.push(clone(project));if(past.length>60)past.shift();}future=[];}
function changed(){revision++;$('#save-state').textContent='Saved in this browser';try{localStorage.setItem('pcb-enclosure-studio:v1',JSON.stringify(project));}catch{$('#save-state').textContent='Use Save project to preserve changes';}renderTree();scheduleBuild();}
function edit(fn){if(!project)return;remember();fn();changed();}
function scheduleBuild(){clearTimeout(timer);$('#export').disabled=true;timer=setTimeout(rebuild,220);}
async function rebuild(fit=false){
  if(!project||running)return;running=true;const rev=revision,snapshot=clone(project);$('#busy').hidden=false;$('#status').textContent='Rebuilding enclosure…';
  try{const r=await (await api('/api/build',jsonOptions(snapshot))).json();if(rev===revision){report=r;renderedRevision=rev;viewer.update(r,fit||!viewer.report);renderTree();renderChecks();if(!$('#inspector').contains(document.activeElement))renderInspector();$('#dimensions-label').textContent=`${r.metrics.width.toFixed(1)} × ${r.metrics.length.toFixed(1)} × ${r.metrics.height.toFixed(1)} mm`;$('#build-stats').textContent=`${r.buildSeconds.toFixed(1)} s rebuild · ${project.features.length} features`;
      $('#status').innerHTML=`<span class="status-dot" style="background:${r.errors.length?'#d77878':'#6baf89'}"></span>${r.errors.length?escape(r.errors[0]):'Model up to date · dimensions in mm'}`;$('#export').disabled=!!r.errors.length;}}
  catch(e){if(rev===revision){$('#status').textContent=e.message;$('#export').disabled=true;$('#check-results').innerHTML=`<div class="check-row error">${escape(e.message)}</div>`;toast(e.message);}}
  finally{running=false;$('#busy').hidden=true;if(rev!==revision)rebuild();}
}
function setProject(p,fit=true){project=p;revision++;$('#export').disabled=true;report=null;renderedRevision=-1;selection={kind:'enclosure',id:'enclosure'};$('#welcome').hidden=!!p;$('#project-name').value=p?.name||'Untitled enclosure';renderTree();renderInspector();if(p){try{localStorage.setItem('pcb-enclosure-studio:v1',JSON.stringify(p));}catch{}rebuild(fit);}else{viewer.clear(viewer.group);viewer.clear(viewer.planeGroup);viewer.clear(viewer.anchorGroup);viewer.objects=[];viewer.report=null;viewer.render();localStorage.removeItem('pcb-enclosure-studio:v1');$('#status').textContent='Ready to import a board';$('#build-stats').textContent='Solid modeling · Open CASCADE';$('#check-results').innerHTML='';$('#dimensions-label').textContent='Millimeters · Z up';}}
function select(s){selection=s;if(['shell','lid','pcb'].includes(s.kind))selection={kind:s.kind==='pcb'?'board':'enclosure',id:'enclosure'};viewer.select(s);renderTree();renderInspector();}
function selectedFeature(){return project?.features.find(x=>x.id===selection.id);}
function selectedPlane(){return project?.planes.find(x=>x.id===selection.id);}
function selectedComponent(){return project?.board.components.find(x=>x.id===selection.id);}
function selectedReference(){return project?.references.find(x=>x.id===selection.id);}
function treeItem(id,kind,title,icon,detail='',status=''){return `<button class="tree-item ${selection.id===id?'active':''}" data-select="${escape(id)}" data-kind="${kind}">${ico(icon)}<span class="item-name">${escape(title)}</span>${detail?`<small>${escape(detail)}</small>`:''}${status?`<span class="tree-indicator ${status}"></span>`:''}</button>`;}
function renderTree(){
  viewer.select(selection);
  $('#undo').disabled=!past.length;$('#redo').disabled=!future.length;
  $$('[data-add],#add-plane,#pick-face,#auto-standoffs,#import-reference,#save-project').forEach(b=>b.disabled=!project);
  if(!project){$('#model-tree').innerHTML='<p class="tree-empty">Your PCB, construction planes, and enclosure features will appear here.</p>';return;}
  let h=`<div class="tree-section">BASE GEOMETRY</div>${treeItem('enclosure','enclosure','Enclosure','box','2 parts')}${treeItem('board','board',project.board.name,'circuit-board')}`;
  h+=`<div class="tree-section">FEATURES <span>${project.features.length}</span></div>`;
  if(!project.features.length)h+='<p class="tree-empty">Choose a modeling tool above<br>to create your first feature.</p>';
  for(const f of project.features){const fr=report?.features.find(x=>x.id===f.id);h+=treeItem(f.id,'feature',f.name,featureIcons[f.type]||'box','',f.suppressed?'suppressed':fr?.status||'');}
  if(project.planes.length){h+='<div class="tree-section">CONSTRUCTION</div>';for(const p of project.planes)h+=treeItem(p.id,'plane',p.name,'layers');}
  if(project.references.length){h+='<div class="tree-section">REFERENCE MODELS</div>';for(const r of project.references)h+=treeItem(r.id,'reference',r.name,'package',r.format);}
  h+=`<div class="tree-section">COMPONENTS <span>${project.board.components.length}</span></div><input class="tree-search" id="component-search" placeholder="Find a component…" aria-label="Find a component"><div class="tree-component-list">`;
  for(const c of project.board.components)h+=`<button class="tree-item ${selection.id===c.id?'active':''}" data-select="${escape(c.id)}" data-kind="component" data-search="${escape((c.ref+' '+c.value+' '+c.footprint).toLowerCase())}">${project.board.source==='step'?`<span class="item-name" title="${escape(c.value)}">${escape(c.value)}</span>`:`<span class="ref">${escape(c.ref)}</span><span class="value">${escape(c.value||c.footprint)}</span>`}</button>`;
  h+='</div>';const search=$('#component-search')?.value||'';$('#model-tree').innerHTML=h;$('#component-search').value=search;$('#component-search').addEventListener('input',filterComponents);filterComponents();
  $$('[data-select]').forEach(b=>b.addEventListener('click',()=>select({kind:b.dataset.kind,id:b.dataset.select})));icons();
}
function filterComponents(){const q=$('#component-search')?.value.toLowerCase()||'';$$('[data-search]').forEach(b=>b.hidden=!b.dataset.search.includes(q));}
function renderChecks(){
  if(!report)return;const count=report.errors.length+report.collisions.length+report.warnings.length;$('#check-badge').textContent=count?String(count):'CLEAR';let h='';
  for(const e of report.errors)h+=`<div class="check-row error">${ico('circle-alert')}<span>${escape(e)}</span></div>`;
  for(const c of report.collisions.slice(0,8))h+=`<div class="check-row warning">${ico('triangle-alert')}<span>${escape(c.name)} overlaps ${c.part}<small>${c.volume.toFixed(2)} mm³${c.estimated?' · estimated envelope':''}</small></span></div>`;
  if(report.collisions.length>8)h+=`<div class="check-row warning">${report.collisions.length-8} more overlaps</div>`;
  for(const w of report.warnings.slice(0,4))h+=`<div class="check-row warning">${escape(w)}</div>`;
  if(!count)h=`<div class="check-row good">${ico('circle-check')}<span>No modeled overlaps<small>${project.board.source==='step'?'PCB and actual STEP solids':'PCB and component envelopes'}</small></span></div>`;
  $('#check-results').innerHTML=h;icons();
}
const section=(title,content)=>`<section class="prop-section"><div class="section-heading"><h3>${title}</h3></div>${content}</section>`;
const field=(key,label,value,type='number',extra='')=>`<label>${label}<input data-key="${key}" type="${type}" value="${escape(value)}" ${type==='number'?'step="0.1"':''} ${extra}></label>`;
const selectField=(key,label,value,options)=>`<label>${label}<select data-key="${key}">${options.map(([v,t])=>`<option value="${escape(v)}" ${String(value)===String(v)?'selected':''}>${escape(t)}</option>`).join('')}</select></label>`;
const grid=(s,three=false)=>`<div class="prop-grid ${three?'triple':''}">${s}</div>`;
const toggle=(key,label,value)=>`<label class="toggle"><input data-key="${key}" type="checkbox" ${value?'checked':''}>${label}</label>`;
const vecFields=(key,labels,values)=>grid(labels.map((l,i)=>field(`${key}.${i}`,l,(values||[0,0,0])[i])).join(''),true);
function setDeep(o,path,value){let keys=path.split('.'),last=keys.pop(),cur=o;for(const k of keys){if(cur[k]===undefined)cur[k]={};cur=cur[k];}cur[last]=value;}
function bindInputs(object){$$('#inspector [data-key]').forEach(el=>el.addEventListener('change',()=>{
  let value=el.type==='checkbox'?el.checked:el.type==='number'?Number(el.value):el.value;
  if(el.type==='number'&&(!el.value||!Number.isFinite(value))){toast('Enter a finite dimension.');renderInspector();return;}
  edit(()=>{setDeep(object,el.dataset.key,value);if(el.dataset.key==='anchor.ref'&&object.anchor?.kind==='pad')object.anchor.pad=project.board.components.find(c=>c.id===value)?.pads[0]?.number;});if(el.tagName==='SELECT'||el.type==='checkbox')renderInspector();
}));}
function renderInspector(){
  $('#inspector-title').textContent='Design settings';$('#selection-type').textContent='PARAMETRIC';
  if(!project){$('#inspector').innerHTML='<div class="inspector-empty">Import a board to configure clearances, wall thickness, the lid, and attachment-based features.</div>';return;}
  if(selection.kind==='component'){renderComponent();return;}if(selection.kind==='reference'){renderReference();return;}
  const f=selectedFeature(),p=selectedPlane();if(f||p){renderFeature(f||p,!!p);return;}
  if(selection.kind==='board'&&project.board.source==='step'){renderStepBoard();return;}
  if(selection.kind==='board'){
    $('#inspector-title').textContent='PCB reference';$('#selection-type').textContent='KICAD';
    $('#inspector').innerHTML=section('Board',field('thickness','Board thickness',project.board.thickness)+`<div class="space data-line"><span>Outline</span><strong>${project.board.width.toFixed(2)} × ${project.board.length.toFixed(2)} mm</strong></div><div class="data-line"><span>Components / holes</span><strong>${project.board.components.length} / ${project.board.holes.length}</strong></div>`)+section('Import fidelity',project.board.warnings.map(w=>`<p class="prop-help warning">${escape(w)}</p>`).join(''))+section('PCB revision','<button class="full" id="reimport-board">Reimport PCB revision</button><p class="prop-help">Feature attachments use footprint IDs. Missing components are reported as failed attachments instead of silently moving features.</p>');bindInputs(project.board);$('#reimport-board').onclick=()=>$('#board-file').click();icons();return;
  }
  const c=project.enclosure;
  $('#inspector').innerHTML=section('Enclosure',selectField('shape','Outline style',c.shape,[['rectangle','Rounded rectangle'],['outline','Follow PCB outline']])+`<div class="space">${grid(field('clearance','PCB clearance',c.clearance)+field('wall','Wall thickness',c.wall))}</div>`+`<div class="space">${grid(field('floor','Floor thickness',c.floor)+field('corner','Corner radius',c.corner))}</div>`)+section('Internal space',grid(field('below','Below PCB',c.below)+field('above','Above PCB',c.above))+toggle('autoHeight','Grow height for components',c.autoHeight)+field('topMargin','Component headroom',c.topMargin)+`<p class="prop-help">PCB bottom is Z = 0. ${project.board.source==='step'?'Height uses the actual component solids from STEP.':'Component bodies are envelopes; verify or edit their heights.'}</p>`)+section('Lid & locating lip',grid(field('lid','Lid thickness',c.lid)+field('lip','Lip depth',c.lip))+`<div class="space">${grid(field('lipWidth','Lip width',c.lipWidth)+field('lipClearance','Lip clearance',c.lipClearance))}</div><p class="prop-help">Set lip depth to 0 for a flat lid. Add closure holes or bosses with the modeling tools.</p>`)+section('Assembly',`<div class="data-line"><span>Shell volume</span><strong>${((report?.volumes.shell||0)/1000).toFixed(2)} cm³</strong></div><div class="data-line"><span>Lid volume</span><strong>${((report?.volumes.lid||0)/1000).toFixed(2)} cm³</strong></div>`);bindInputs(c);icons();
}
function renderComponent(){
  const c=selectedComponent();if(!c)return;if(project.board.source==='step'){renderStepComponent(c);return;}$('#inspector-title').textContent=c.ref;$('#selection-type').textContent='PCB COMPONENT';
  $('#inspector').innerHTML=section('Component',`<div class="attachment-chip">${ico('microchip')}${escape(c.value||c.footprint)}</div><p class="prop-help">${escape(c.footprint)}</p>`)+section('Envelope',grid(field('width','Width',c.width)+field('depth','Length',c.depth))+`<div class="space">${field('height','Height above mounting surface',c.height)}</div>`+selectField('side','PCB side',c.side,[['top','Top'],['bottom','Bottom']])+`<p class="prop-help warning">Envelope, not an exact component model. KiCad file imports do not load external 3D libraries.</p>`)+section('Placement',grid(field('xy.0','X',c.xy[0])+field('xy.1','Y',c.xy[1]))+`<div class="space">${field('rotation','Rotation Z (degrees)',c.rotation)}</div>`)+section('Build around this part',`<button id="component-window" class="full">${ico('rectangle-horizontal')}Add a projected window</button><button id="component-plane" class="full space">${ico('layers')}Create attached plane</button>`);bindInputs(c);$('#component-window').onclick=()=>addFeature('window');$('#component-plane').onclick=()=>addPlane();icons();
}
function renderStepBoard(){
  const b=project.board;$('#inspector-title').textContent='PCB assembly';$('#selection-type').textContent='STEP';
  $('#inspector').innerHTML=section('Detailed board',`<div class="attachment-chip">${ico('circuit-board')}${escape(b.name)}</div><div class="space data-line"><span>Board outline</span><strong>${b.width.toFixed(2)} × ${b.length.toFixed(2)} mm</strong></div><div class="data-line"><span>Substrate thickness</span><strong>${b.thickness.toFixed(3)} mm</strong></div><div class="data-line"><span>Solid bodies / groups</span><strong>${b.bodyCount} / ${b.components.length}</strong></div><p class="prop-help">Actual STEP solids are used for display, height, and fit checks. Click a flat face to attach a feature or plane.</p>`)+section('Substrate & orientation',selectField('substrate','PCB substrate body',String(b.substrate),b.substrateOptions.map(o=>[String(o.body),`Body ${o.body+1} · ${o.dimensions.map(d=>d.toFixed(2)).join(' × ')} · ${o.name}`]))+toggle('flip','Flip board side',b.flip)+`<p class="prop-help">The broadest flat body is selected initially. Choose the main PCB, not a daughterboard or shield. Changing this moves the assembly into the new board frame.</p>`)+section('Import fidelity',b.warnings.map(w=>`<p class="prop-help">${escape(w)}</p>`).join(''))+section('PCB revision','<button class="full" id="reimport-board">Reimport PCB revision</button><p class="prop-help">STEP face attachments are tied to this source file. Re-pick them after importing a changed STEP assembly.</p>');
  for(const el of $$('#inspector [data-key]'))el.onchange=async()=>{const choice=$('#inspector [data-key="substrate"]'),flip=$('#inspector [data-key="flip"]');choice.disabled=true;flip.disabled=true;try{const next=await (await api('/api/board/substrate',jsonOptions({asset:b.asset,name:b.name,substrate:Number(choice.value),flip:flip.checked}))).json();edit(()=>{project.board=next.board;renderInspector();});}catch(e){toast(e.message);renderInspector();}};
  $('#reimport-board').onclick=()=>$('#board-file').click();icons();
}
function renderStepComponent(c){
  $('#inspector-title').textContent=c.ref;$('#selection-type').textContent='STEP SOLIDS';
  $('#inspector').innerHTML=section('Component',`<div class="attachment-chip">${ico('microchip')}${escape(c.value)}</div><p class="prop-help">${c.bodies.length} original solid bodies. Geometry and placement come from the imported assembly.</p>`)+section('Measured bounds',`<div class="data-line"><span>Width × length</span><strong>${c.width.toFixed(2)} × ${c.depth.toFixed(2)} mm</strong></div><div class="data-line"><span>Bottom / top Z</span><strong>${c.zMin.toFixed(3)} / ${c.zMax.toFixed(3)}</strong></div><p class="prop-help">Body center and directional attachments use these bounds. Pick a planar face for an exact surface attachment.</p>`)+section('Build around this part',`<button id="component-face" class="full">${ico('mouse-pointer-2')}Pick an exact face</button><button id="component-window" class="full space">${ico('rectangle-horizontal')}Add a projected window</button><button id="component-plane" class="full space">${ico('layers')}Create attached plane</button>`);
  $('#component-face').onclick=startFacePick;$('#component-window').onclick=()=>addFeature('window');$('#component-plane').onclick=()=>addPlane();icons();
}
function renderReference(){
  const r=selectedReference();if(!r)return;$('#inspector-title').textContent='Reference model';$('#selection-type').textContent=r.format;
  $('#inspector').innerHTML=section('Model',field('name','Name',r.name,'text')+toggle('visible','Show in viewport',r.visible)+`<p class="prop-help">${r.planarFaces} planar STEP faces available for attachment. Reference geometry is not included in exported enclosure parts or collision checks.</p>`)+section('Position',vecFields('offset',['X','Y','Z'],r.offset)+`<button id="center-reference" class="full space">Center on PCB / bottom at Z = 0</button>`)+section('Orientation',vecFields('rotation',['X°','Y°','Z°'],r.rotation)+`<p class="prop-help">Reference units are millimeters. STL has no unit declaration.</p>`)+`<div class="prop-actions"><button id="reference-face">${ico('mouse-pointer-2')}Pick face</button><button id="delete-reference" class="danger">Remove</button></div>`;bindInputs(r);
  $('#center-reference').onclick=()=>edit(()=>{const b=r.bounds;const q=project.board;r.offset=[q.width/2-(b[0][0]+b[1][0])/2,q.length/2-(b[0][1]+b[1][1])/2,-b[0][2]];r.rotation=[0,0,0];renderInspector();});$('#reference-face').onclick=startFacePick;
  $('#delete-reference').onclick=()=>edit(()=>{project.references=project.references.filter(x=>x.id!==r.id);selection={kind:'enclosure',id:'enclosure'};renderInspector();});icons();
}
function attachmentForm(f,isPlane){
  const a=f.anchor;let html=selectField('anchor.kind','Attach to',a.kind,[['origin','World origin'],['case','Enclosure surface'],['component','PCB component'],['pad','Component pad'],['hole','PCB hole'],['plane','Construction plane'],['face','Picked reference STEP face'],...(project.board.source==='step'||a.kind==='boardFace'?[['boardFace','Picked PCB STEP face']]:[])]);
  if(a.kind==='component'||a.kind==='pad')html+=`<div class="space">${selectField('anchor.ref','Component',a.ref,project.board.components.map(c=>[c.id,c.ref+' · '+(c.value||c.footprint)]))}</div>`;
  if(a.kind==='component')html+=`<div class="space">${selectField('anchor.point','Component feature',a.point||'center',[['center','Body center'],['origin','Footprint origin'],['top','Top face'],['bottom','Bottom face'],['front','Front face (−local Y)'],['back','Back face (+local Y)'],['left','Left face (−local X)'],['right','Right face (+local X)']])}</div>`;
  if(a.kind==='pad'){const c=project.board.components.find(c=>c.id===a.ref);html+=`<div class="space">${selectField('anchor.pad','Pad number',a.pad,(c?.pads||[]).map(p=>[p.number,'Pad '+p.number]))}</div>`;}
  if(a.kind==='hole')html+=`<div class="space">${selectField('anchor.ref','PCB hole',a.ref,project.board.holes.map(h=>[h.id,h.source+' · Ø'+h.diameter.toFixed(2)]))}</div>`;
  if(a.kind==='case')html+=`<div class="space">${selectField('anchor.point','Surface',a.point||'floor',[['floor','Inside floor'],['lid','Lid outside'],['rim','Shell rim'],['front','Front wall'],['back','Back wall'],['left','Left wall'],['right','Right wall']])}</div>`;
  if(a.kind==='plane')html+=`<div class="space">${selectField('anchor.ref','Plane',a.ref,project.planes.filter(p=>p.id!==f.id).map(p=>[p.id,p.name]))}</div>`;
  if(a.kind==='face'||a.kind==='boardFace')html+=`<p class="prop-help">${a.face?'Planar reference face '+escape(a.face.id):'No face picked yet.'}</p><button id="repick-face" class="full space">Pick a STEP face</button>`;
  html+=`<div class="space">${selectField('anchor.plane','Orientation',a.plane||'face',[['face','Follow attachment frame'],['XY','XY · normal +Z'],['XZ','XZ · normal −Y'],['YZ','YZ · normal +X'],['-XY','XY · normal −Z'],['-XZ','XZ · normal +Y'],['-YZ','YZ · normal −X']])}</div>`;
  html+=`<div class="space">${selectField('anchor.project','Project origin to',a.project||'none',[['none','No projection'],['front','Front wall'],['back','Back wall'],['left','Left wall'],['right','Right wall'],['floor','Inside floor'],['lid','Lid outside']])}</div>`;
  return html;
}
function dimensionForm(f){
  const d=f.dimensions;let h='';
  if(['box','window','vent'].includes(f.type))h+=grid(field('dimensions.width','Width',d.width)+field('dimensions.height','Height',d.height));
  if(['box','window'].includes(f.type))h+=`<div class="space">${field('dimensions.radius','Corner radius',d.radius||0)}</div>`;
  if(['cylinder','hole','standoff'].includes(f.type))h+=field('dimensions.diameter','Diameter',d.diameter);
  if(f.type==='standoff')h+=`<div class="space">${field('dimensions.bore','Screw bore diameter',d.bore)}</div>`;
  if(f.type==='vent')h+=`<div class="space">${grid(field('dimensions.count','Slot count',d.count)+field('dimensions.pitch','Slot spacing',d.pitch))}</div>`;
  if(f.type==='text')h+=field('dimensions.text','Text',d.text,'text')+`<div class="space">${field('dimensions.height','Text size',d.height)}</div>`;
  if(f.type==='sketch')h+=`<label>Polygon points (U, V)<textarea class="sketch-input" id="sketch-points">${escape((d.points||[]).map(p=>p.join(', ')).join('\n'))}</textarea></label><p class="prop-help">One pair per line. Closed automatically; dimensions in the attachment plane.</p>`;
  h+=`<div class="space">${field('dimensions.depth',f.type==='standoff'?'Post height':'Extrusion depth',d.depth)}</div>`;
  if(!['standoff','text'].includes(f.type))h+=`<div class="space">${selectField('extent','Extrusion direction',f.extent,[['symmetric','Symmetric about plane'],['one-sided','Along plane normal']])}</div>`;
  return h;
}
function renderFeature(f,isPlane){
  $('#inspector-title').textContent=isPlane?'Construction plane':'Feature properties';$('#selection-type').textContent=isPlane?'REFERENCE':f.type.toUpperCase();
  let h=section(isPlane?'Plane':'Feature',field('name','Name',f.name,'text')+(!isPlane?`<div class="space">${grid(selectField('operation','Operation',f.operation,[['cut','Remove material'],['add','Add material']])+selectField('target','Part',f.target,[['shell','Shell'],['lid','Lid'],['both','Both']]))}</div>`:''));
  h+=section('Attachment',attachmentForm(f,isPlane))+section('Local position',vecFields('offset',['U','V','N'],f.offset)+`<p class="prop-help">U and V lie in the plane; N follows its normal. Colored axes show the selected attachment: red U, green V, blue N.</p>`)+section('Local rotation',vecFields('rotation',['U°','V°','N°'],f.rotation));
  if(!isPlane)h+=section('Dimensions',dimensionForm(f));
  if(!isPlane)h+=section('Feature state',toggle('suppressed','Suppress this feature',f.suppressed));
  const status=report?.features.find(x=>x.id===f.id);if(status?.message)h+=`<div class="feature-status ${status.status}">${escape(status.message)}</div>`;
  h+=`<div class="prop-actions"><button id="duplicate-feature">Duplicate</button><button id="remove-feature" class="danger">Remove</button></div>`;
  if(!isPlane)h+=`<div class="prop-actions"><button id="move-earlier">↑ Earlier</button><button id="move-later">↓ Later</button></div>`;
  $('#inspector').innerHTML=h;
  // Type changes populate a valid reference before rebuild, rather than retaining an unrelated ID.
  const typeSelect=$('#inspector [data-key="anchor.kind"]');typeSelect.removeAttribute('data-key');typeSelect.onchange=()=>edit(()=>{f.anchor={kind:typeSelect.value,plane:'face',project:'none'};if(['component','pad'].includes(typeSelect.value))f.anchor.ref=project.board.components[0]?.id;if(typeSelect.value==='pad')f.anchor.pad=project.board.components[0]?.pads[0]?.number;if(typeSelect.value==='hole')f.anchor.ref=project.board.holes[0]?.id;if(typeSelect.value==='plane')f.anchor.ref=project.planes.find(p=>p.id!==f.id)?.id;if(typeSelect.value==='case')f.anchor.point='floor';renderInspector();});bindInputs(f);
  if($('#repick-face'))$('#repick-face').onclick=startFacePick;
  if($('#sketch-points'))$('#sketch-points').onchange=e=>{const points=e.target.value.trim().split(/\n/).map(l=>l.split(/[,\s]+/).filter(Boolean).map(Number));if(points.some(p=>p.length!==2||p.some(v=>!Number.isFinite(v)))){toast('Use two numbers per line, for example: 10, 5.');return;}edit(()=>f.dimensions.points=points);};
  $('#remove-feature').onclick=()=>edit(()=>{const key=isPlane?'planes':'features';project[key]=project[key].filter(x=>x.id!==f.id);selection={kind:'enclosure',id:'enclosure'};renderInspector();});
  $('#duplicate-feature').onclick=()=>edit(()=>{const n=clone(f);n.id=uid();n.name+=' copy';project[isPlane?'planes':'features'].push(n);selection={kind:isPlane?'plane':'feature',id:n.id};renderInspector();});
  for(const [id,delta]of [['move-earlier',-1],['move-later',1]])if($('#'+id))$('#'+id).onclick=()=>edit(()=>{const i=project.features.findIndex(x=>x.id===f.id),j=i+delta;if(j>=0&&j<project.features.length)[project.features[i],project.features[j]]=[project.features[j],project.features[i]];});icons();
}
function nearestWall(c){const [x,y]=c.xy,b=project.board;return [['left',x],['right',b.width-x],['front',y],['back',b.length-y]].sort((a,b)=>a[1]-b[1])[0][0];}
const wallPlane={left:'-YZ',right:'YZ',front:'XZ',back:'-XZ'};
function makeFeature(type,anchor=null){
  const c=selectedComponent(),p=selectedPlane(),cut=['window','hole','vent','sketch'].includes(type);let a=anchor;
  if(!a&&p)a={kind:'plane',ref:p.id,plane:'face'};
  if(!a&&c&&cut){const wall=nearestWall(c);a={kind:'component',ref:c.id,point:'center',plane:wallPlane[wall],project:wall};}
  if(!a&&c)a={kind:'component',ref:c.id,point:'bottom',plane:'-XY'};
  if(!a)a={kind:'case',point:type==='text'?'lid':cut?'front':'floor',plane:'face'};
  return {id:uid(),name:featureNames[type]+' '+(project.features.filter(f=>f.type===type).length+1),type,operation:cut?'cut':'add',target:type==='text'?'lid':'shell',anchor:a,offset:[0,0,type==='text'?-.15:0],rotation:[0,0,0],extent:cut?'symmetric':'one-sided',suppressed:false,dimensions:{width:type==='sketch'?10:12,height:type==='text'?4:type==='vent'?1.5:6,depth:type==='standoff'?Math.max(.2,project.enclosure.below):type==='text'?.7:cut?12:5,diameter:type==='standoff'?6:4,bore:2.2,radius:type==='window'?1:0,count:5,pitch:4,text:'BlinkyHawk',points:[[-5,-4],[5,-4],[7,0],[5,4],[-5,4]]}};
}
function addFeature(type,anchor){if(!project)return;edit(()=>{const f=makeFeature(type,anchor);project.features.push(f);selection={kind:'feature',id:f.id};renderInspector();});}
function addPlane(anchor){if(!project)return;const c=selectedComponent(),p=selectedPlane();edit(()=>{const n={id:uid(),name:'Plane '+(project.planes.length+1),anchor:anchor||(c?{kind:'component',ref:c.id,point:'top',plane:'face'}:p?{kind:'plane',ref:p.id,plane:'face'}:{kind:'origin',plane:'XY'}),offset:[0,0,0],rotation:[0,0,0]};project.planes.push(n);selection={kind:'plane',id:n.id};renderInspector();});}
function startFacePick(){if(!project)return;pickTarget=selectedFeature()||selectedPlane();viewer.picking=true;$('#face-hint').hidden=false;$('#pick-face').classList.add('recording');}
function endFacePick(){viewer.picking=false;$('#face-hint').hidden=true;$('#pick-face').classList.remove('recording');pickTarget=null;}
function acceptFace(a){
  if(a.error){toast(a.error);return;}
  if(a.kind==='component'){
    const c=project.board.components.find(c=>c.id===a.ref),theta=c.rotation*Math.PI/180,n=a.normal;
    const local=[Math.cos(theta)*n[0]+Math.sin(theta)*n[1],-Math.sin(theta)*n[0]+Math.cos(theta)*n[1],n[2]];const axis=local.map(Math.abs).indexOf(Math.max(...local.map(Math.abs)));a.point=axis===2?(local[2]>0?'top':'bottom'):axis===0?(local[0]>0?'right':'left'):(local[1]>0?'back':'front');delete a.normal;
  }
  const target=pickTarget;endFacePick();if(target)edit(()=>{target.anchor=a;target.offset=[0,0,0];renderInspector();});else addPlane(a);toast('Planar attachment selected.');
}
async function fileImport(input,url,callback){const file=input.files[0];input.value='';if(!file)return;endFacePick();$('#busy-text').textContent='Importing geometry…';$('#busy').hidden=false;try{const data=new FormData();data.append('file',file);const result=await (await api(url,{method:'POST',body:data})).json();callback(result);}catch(e){toast(e.message);}finally{$('#busy').hidden=!running;$('#busy-text').textContent='Rebuilding solids…';}}
async function download(url,p){const response=await api(url,jsonOptions(p));const blob=await response.blob(),a=document.createElement('a'),u=URL.createObjectURL(blob);a.href=u;a.download=(response.headers.get('Content-Disposition')||'').match(/filename="([^"]+)"/)?.[1]||'enclosure.zip';document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(u),30000);}

$('#import-board').onclick=$('#welcome-import').onclick=()=>$('#board-file').click();
$('#board-file').onchange=e=>fileImport(e.target,'/api/import/board',p=>{remember();if(project){p.features=project.features;p.planes=project.planes;p.references=project.references;p.enclosure=project.enclosure;p.name=project.name;}setProject(p);toast(p.board.source==='step'?'STEP PCB imported with detailed components. Verify the detected substrate in PCB properties.':'PCB imported. Check component heights before exporting.');});
$('#import-reference').onclick=()=>$('#reference-file').click();$('#reference-file').onchange=e=>fileImport(e.target,'/api/import/reference',r=>{edit(()=>project.references.push(r));selection={kind:'reference',id:r.id};renderInspector();toast('Reference imported in its original coordinates. Use Center on PCB or adjust its position.');});
$('#load-step-example').onclick=async()=>{try{setProject(await (await api('/api/example/step')).json());}catch(e){toast(e.message);}};
$('#load-example').onclick=async()=>{try{setProject(await (await api('/api/example')).json());}catch(e){toast(e.message);}};
$('#new-project').onclick=()=>{remember();setProject(null);toast('New workspace. Undo restores the previous project.');};
$('#open-project').onclick=()=>$('#project-file').click();$('#project-file').onchange=e=>fileImport(e.target,'/api/project/open',p=>{remember();setProject(p);toast('Project and embedded reference models opened.');});
$('#save-project').onclick=async()=>{if(!project)return;try{await download('/api/project/save',project);$('#save-state').textContent='Portable project downloaded';}catch(e){toast(e.message);}};
$('#project-name').onchange=e=>{if(project)edit(()=>project.name=e.target.value||'Untitled enclosure');};
$$('[data-add]').forEach(b=>b.onclick=()=>addFeature(b.dataset.add));$('#add-plane').onclick=()=>addPlane();$('#pick-face').onclick=startFacePick;$('#cancel-pick').onclick=endFacePick;
$('#auto-standoffs').onclick=()=>{if(!project)return;const holes=project.board.holes.filter(h=>h.mounting&&!project.features.some(f=>f.type==='standoff'&&f.anchor.kind==='hole'&&f.anchor.ref===h.id));if(!holes.length){toast('No unused mounting holes were identified. You can attach a standoff to any hole manually.');return;}edit(()=>{for(const h of holes){const f=makeFeature('standoff',{kind:'hole',ref:h.id,plane:'XY'});f.name='Standoff · '+h.source;f.dimensions.bore=Math.max(1,h.diameter-.1);f.dimensions.diameter=Math.max(5,h.diameter+3);project.features.push(f);}selection={kind:'feature',id:project.features.at(-1).id};renderInspector();});toast(`${holes.length} standoffs created. Check screw bore dimensions for your hardware.`);};
$('#undo').onclick=()=>{if(!past.length)return;future.push(clone(project));const p=past.pop();setProject(p,false);};$('#redo').onclick=()=>{if(!future.length)return;past.push(clone(project));setProject(future.pop(),false);};
document.addEventListener('keydown',e=>{if(['INPUT','TEXTAREA','SELECT'].includes(document.activeElement.tagName))return;if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='z'){e.preventDefault();(e.shiftKey?$('#redo'):$('#undo')).click();}if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='y'){e.preventDefault();$('#redo').click();}if(e.key==='Escape')endFacePick();});
for(const key of ['iso','top','front','right'])$('#view-'+key).onclick=()=>{$$('.view-mode button').forEach(b=>b.classList.remove('active'));$('#view-'+key).classList.add('active');viewer.fit(key);};$('#fit-view').onclick=()=>viewer.fit();
for(const [id,key]of [['show-pcb','showPCB'],['show-lid','showLid'],['ghost-shell','ghost'],['show-planes','showPlanes']])$('#'+id).onchange=e=>{viewer[key]=e.target.checked;viewer.visibility();};$('#explode').oninput=e=>{viewer.explode=Number(e.target.value);$('#explode-value').textContent=e.target.value;viewer.visibility();};
$('#help').onclick=()=>$('#help-dialog').showModal();$('#export').onclick=()=>{if(!project||running||renderedRevision!==revision)return;$('#export-warning').textContent=report.collisions.length?`${report.collisions.length} modeled overlaps remain. Inspect the fit-check warnings before printing.`:'Model-based check only. Verify solder joints, wiring, cable overmolds, and printer tolerances with a fit print.';$('#export-dialog').showModal();};
$('#download-parts').onclick=async()=>{if(running||renderedRevision!==revision){toast('Wait for the current rebuild before exporting.');return;}const b=$('#download-parts');b.disabled=true;b.textContent='Preparing solids…';try{await download(`/api/export/${$('#export-format').value}/${$('#export-part').value}`,project);$('#export-dialog').close();toast('Parts exported.');}catch(e){toast(e.message);}finally{b.disabled=false;b.innerHTML=ico('download')+'Download';icons();}};
$('#export').disabled=true;
let saved;try{saved=JSON.parse(localStorage.getItem('pcb-enclosure-studio:v1'));}catch{}
if(saved?.version===1&&saved.board){setProject(saved);}else{renderTree();renderInspector();}
icons();

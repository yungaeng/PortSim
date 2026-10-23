import {impactOverlay,mapPosition} from './control-view.mjs';
const esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const xy=p=>Array.isArray(p)&&p.length>=2&&p.slice(0,2).every(Number.isFinite);
const fmt=v=>Number(v.toFixed(3));
const backgroundCache=new WeakMap();
export function validFootprint(r){return xy(r?.position_m)&&xy(r?.size_m)&&r.size_m.every(v=>v>0)&&Number.isFinite(r.yaw_deg??0);}
export function footprintRect(r,attributes=''){
 if(!validFootprint(r))return '';
 const [x,y]=r.position_m,[w,h]=r.size_m;
 return `<rect x="${fmt(x-w/2)}" y="${fmt(y-h/2)}" width="${fmt(w)}" height="${fmt(h)}" transform="rotate(${fmt(r.yaw_deg||0)} ${fmt(x)} ${fmt(y)})" ${attributes}/>`;
}
export function mapViewport(layout,mode='all'){
 if(mode==='gap'){
  const yard=mapViewport(layout,'yard'),road=layout?.central_road_m;
  return yard&&road?.length===4?{...yard,minY:road[1]-140,height:280}:null;
 }
 const regions={all:'bounds_m',yard:'yard_view_m',quay:'quay_view_m',depot:'depot_view_m'},b=layout?.[regions[mode]||'bounds_m'];
 if(!Array.isArray(b)||b.length!==4||!b.every(Number.isFinite)||b[2]<=b[0]||b[3]<=b[1])return null;
 return {minX:b[0],minY:b[1],width:b[2]-b[0],height:b[3]-b[1]};
}
function bayMarkup(b){
 if(!Number.isInteger(b.bay)||!Number.isInteger(b.rows)||b.rows<1||b.rows>20)return '';
 const r=footprintRect(b,`class="yard-slot-outline" data-yard-bay="${b.bay}"`);if(!r)return '';
 const [x,y]=b.position_m,[w,h]=b.size_m;
 let lines='';for(let row=1;row<b.rows;row++)lines+=`M${x-w/2} ${y-h/2+row*h/b.rows}h${w}`;
 if(b.pads===2)lines+=`M${x} ${y-h/2}v${h}`;
 return `<g class="map-item" data-actor="${esc(b.yard_id)}" role="button" tabindex="0" aria-label="${esc(b.yard_id)} Bay ${b.bay+1} 선택"><title>${esc(b.yard_id)} · Bay ${b.bay+1} · ${b.length_ft}ft${b.pads===2?' 두 자리':''}</title>${r}<path d="${lines}" class="yard-slot-lines" pointer-events="none"/></g>`;
}
function background(layout){
 if(backgroundCache.has(layout))return backgroundCache.get(layout);
 let h=`<polygon points="${(layout.boundary_m||[]).filter(xy).map(p=>p.map(fmt).join(',')).join(' ')}" class="terminal-land"/>`;
 // Batch static world rectangles into paths; thousands of painted dashes
 // must not become thousands of new DOM nodes on each telemetry refresh.
 const paths=rs=>rs.filter(validFootprint).map(r=>{
  const [x,y]=r.position_m,[w,h]=r.size_m,c=Math.cos((r.yaw_deg||0)*Math.PI/180),s=Math.sin((r.yaw_deg||0)*Math.PI/180);
  return [[-1,-1],[1,-1],[1,1],[-1,1]].map(([a,b],i)=>`${i?'L':'M'}${fmt(x+a*w*c/2-b*h*s/2)} ${fmt(y+a*w*s/2+b*h*c/2)}`).join('')+'Z';
 }).join('');
 h+=`<path class="terminal-pavement" d="${paths(layout.pavement||[])}" pointer-events="none"/>`;
 h+=`<g class="terminal-markings" pointer-events="none"><path d="${paths((layout.markings||[]).filter(r=>r.material!=='yellow'))}"/><path class="paint-yellow" d="${paths((layout.markings||[]).filter(r=>r.material==='yellow'))}"/></g>`;
 h+='<g class="terminal-buildings" pointer-events="none">'+(layout.buildings||[]).map(r=>footprintRect(r)).join('')+'</g>';
 h+='<g class="terminal-yard-bays">'+(layout.yard_bays||[]).map(bayMarkup).join('')+'</g>';
 for(const b of layout.yard_blocks||[])if(xy(b.label_position_m))h+=`<text x="${b.label_position_m[0]-5}" y="${b.label_position_m[1]+3}" class="yard-block-label" text-anchor="end" pointer-events="none">${String(b.number).padStart(2,'0')}</text>`;
 const road=layout.central_road_m;
 if(road?.length===4)h+=`<text x="${(road[0]+road[2])/2}" y="${road[1]+2.5}" class="central-road-label" text-anchor="middle" pointer-events="none">중앙도로 · ${road[3]}m</text>`;
 backgroundCache.set(layout,h);return h;
}

export function renderTerminalOverview(state,layout,{selected=null,focus=null,mode='all'}={}){
 const controls=`<div class="map-view-controls" aria-label="지도 보기">${[['all','전체'],['yard','야드'],['gap','중앙 공간'],['quay','선석'],['depot','주차장']].map(([key,name])=>`<button class="chip ${key===mode?'selected':''}" data-map-view="${key}" aria-pressed="${key===mode}">${name}</button>`).join('')}</div>`;
 const head=`<div class="panel-head"><div><h2>터미널 오버뷰 <span class="number">ENGINE PLAN</span></h2><p>엔진의 실제 배치 · 장비를 클릭하면 상세 정보가 열립니다.</p></div>${controls}</div>`;
 const viewport=mapViewport(layout,mode);
 if(!viewport||!state?.run_id||layout?.run_id!==state.run_id)return `<section class="panel terminal-overview">${head}<div class="map-unavailable"><strong>엔진 배치 데이터 대기</strong><p>새 빌드로 시뮬레이션을 실행하면 실제 도로·야드 배치를 불러옵니다.</p><small>다른 실행의 배경에 현재 장비 위치를 겹쳐 표시하지 않습니다.</small></div></section>`;
 const actors=state.actors||[];
 const item=(a,body)=>`<g class="map-item ${a.id===selected?'selected':''}" data-actor="${esc(a.id)}" tabindex="0" role="button" aria-label="${esc(a.name)} 선택"><title>${esc(a.name)} · ${esc(a.state)}</title>${body}</g>`;
 let h=background(layout);
 if(selected?.startsWith('yard-'))h+='<g class="yard-selection" pointer-events="none">'+(layout.yard_bays||[]).filter(b=>b.yard_id===selected).map(b=>footprintRect(b)).join('')+'</g>';
 for(const a of actors.filter(a=>a.type==='vessel'&&xy(a.position_m))){
  const [x,y]=a.position_m;h+=item(a,footprintRect({position_m:a.position_m,size_m:layout.vessel_size_m||[45,210]},'class="map-vessel map-selection"')+`<text x="${x}" y="${y}" transform="rotate(-90 ${x} ${y})" class="vessel-map-label" text-anchor="middle">${esc(a.name)}</text>`);
 }
 // Actual cargo footprints, rather than filling every marked empty yard slot.
 for(const a of actors.filter(a=>a.type==='container'&&a.state==='Yard'&&xy(a.position_m)&&xy(a.dimensions_m)))
  h+=item(a,footprintRect({position_m:a.position_m,size_m:a.dimensions_m.slice(0,2),yaw_deg:a.rotation_deg?.[2]||0},'class="map-yard-cargo map-selection"'));
 h+=impactOverlay(state,focus,x=>x,y=>y,selected);
 for(const a of actors.filter(a=>['sts','rmg','agv'].includes(a.type))){
  const p=mapPosition(a);if(!xy(p))continue;const [x,y]=p;
  if(a.type==='agv'){
   const yaw=a.rotation_deg?.[2]||0;
   h+=item(a,`<circle cx="${x}" cy="${y}" r="7" fill="transparent"/>`+footprintRect({position_m:p,size_m:[4.2,14.6],yaw_deg:yaw},`class="map-agv map-selection${a.payload_kg>0?' loaded':''}"`));
  }else{
   const parts=(a.map_parts||[]).map(r=>footprintRect(r)).join('');
   h+=item(a,`<g class="map-crane ${a.type}">${parts||`<circle cx="${x}" cy="${y}" r="3"/>`}</g><circle cx="${x}" cy="${y}" r="7" fill="transparent"/><text x="${x+5}" y="${y-5}" class="crane-map-label">${esc(a.name)}</text>`);
   if(xy(a.head_position_m))h+=`<circle cx="${a.head_position_m[0]}" cy="${a.head_position_m[1]}" r="2.1" fill="${a.fault?'#b83c46':'#116a59'}" pointer-events="none"/>`;
  }
  const e=a.emergency_test;
  if(e?.active&&xy(e.worker_position_m))h+=`<g class="map-test-worker" pointer-events="none"><circle cx="${e.worker_position_m[0]}" cy="${e.worker_position_m[1]}" r="3.5"/><text x="${e.worker_position_m[0]+6}" y="${e.worker_position_m[1]-5}">시험 작업자</text></g>`;
 }
 const {minX,minY,width,height}=viewport,scale=mode==='depot'?25:100;
 h+=`<g class="map-scale" transform="translate(${minX+width-140} ${minY+height-25})" pointer-events="none"><path d="M0 -5v5h${scale}v-5"/><text x="${scale/2}" y="-9" text-anchor="middle">${scale}m</text></g>`;
 return `<section class="panel terminal-overview" data-map-mode="${esc(mode)}">${head}<div class="terminal-map-wrap"><svg class="terminal-world-map" viewBox="${minX} ${minY} ${width} ${height}" preserveAspectRatio="xMidYMid meet" role="img" aria-label="동일 축척의 실제 엔진 터미널 배치 지도"><defs><clipPath id="terminal-plan-clip"><rect x="${minX}" y="${minY}" width="${width}" height="${height}"/></clipPath></defs><g clip-path="url(#terminal-plan-clip)">${h}</g></svg></div><div class="map-caption"><span>X 내륙 → · Y 부두축 ↓ · 가로/세로 동일 축척</span><span>슬롯 구획: 엔진 배치 · 채워진 칸: 실제 적재 화물</span></div><div class="map-plan-key"><span><i class="key-road"></i>실제 포장면</span><span><i class="key-yard"></i>야드 슬롯</span><span><i class="key-agv"></i>AGV 실제 차체</span><span>7열 · 빈 공간 · 6열</span></div></section>`;
}

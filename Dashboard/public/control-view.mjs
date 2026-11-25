import {incidentActions} from './incident-model.mjs';
const esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
export const mapPosition=a=>a?.gantry_position_m||a?.position_m;
import {workerIntrusionControls} from './agv-emergency-view.mjs';
export function controlPanel(a,state,live,pending,message='',distance=20){
 return equipmentControlPanel(a,state,live,pending,message)+
  workerIntrusionControls(a,state,!live||!state?.control_session||!!state.fault||!!pending,distance);
}
function equipmentControlPanel(a,state,live,pending,message=''){
 if(!['sts','rmg','agv'].includes(a?.type))return '<div class="note">STS·RMG·AGV를 선택하면 고장을 제어할 수 있습니다.</div>';
 const failed=a.fault_scope==='equipment',disabled=!live||!state?.control_session||!(state?.incident_control_version>=4)||!!state.fault||!!pending;
 const mode=a.type==='agv'?'':`<label>고장 유형 <select data-fault-mode ${disabled||failed?'disabled':''}><option value="fail_reboot">자동화 카메라 · 재부팅 ${esc(a.sensor_reboot_seconds??30)}초</option><option value="fail_sensor_repair">필수 센서 · 현장 수리 ${esc(a.sensor_technician_seconds??900)}초</option><option value="fail_gantry">갠트리 주행 고장 · 자동 복구</option></select></label>`;
 const remaining=a.sensor_repair_remaining_seconds;
 const repair=failed&&a.type!=='agv'?`<p>${a.gantry_failure?'갠트리 주행 불가 · 정비 중':Number.isFinite(remaining)&&remaining>=0?`자동 복구까지 약 ${Math.ceil(remaining)}초 · 정지 중에는 시간이 흐르지 않습니다.`:'돌발상황 관리에서 복구 상태 확인'}${a.fault_evacuating?' · 독립 안전 제어로 대피 중':''}</p>`:'';
 return `<section class="fault-controls"><h3>장비 고장 · 복구</h3><p>${esc(a.name)} · ${a.type==='agv'?'주행 고장':'크레인 고장 유형 선택'}</p>${mode}${repair}${a.loaded_recovery_active?`<p role="status">${esc(a.loaded_recovery_status)} · ${esc(a.recovery_transferred_count)}/${esc(a.recovery_cargo_count)}개 인계</p>`:""}<div class="control-actions"><button class="button danger" data-command="fail" ${disabled||failed?'disabled':''}>오류 발생</button>${a.type==='agv'&&failed?`<button class="button" data-command="replace_loaded_agv" disabled title="${esc('복구 방식 재설계 중으로 잠겨 있습니다.')}">새 AGV 배정 · R/S 인계 (잠금)</button>`:''}</div><p class="control-message" role="status">${esc(message||(!live?'최근 기록은 제어할 수 없습니다. 언리얼 시뮬레이션을 실행하세요.':state?.fault?'전체 안전 정지 원인을 먼저 해결하세요.':!state?.control_session?'제어 기능이 포함된 엔진으로 다시 실행하세요.':failed?'고장 적용 상태 · 관련 작업 격리':'복구 일정은 실시간 엔진에서 적용합니다.'))}</p><div class="note">${a.type==='agv'?'AGV는 현재 위치·화물·예약을 유지합니다. 새 AGV 배정과 R/S 이동·인계는 현재 잠겨 있습니다. 다른 AGV는 기존 도로 예약·LiDAR로 대기하거나 우회합니다.':'이미 잠기거나 매달린 화물은 유지합니다. 자동화 카메라 고장의 대피는 빈 스프레더·안전 높이·독립 주행 안전계 정상일 때만 허용합니다. 수리 시간에는 센서 재확인 시간이 추가됩니다. 미인양 STS 작업과 RMG 목적지는 도달 범위·적재 조건을 만족할 때 재배정합니다.'}</div></section>`;
}

export function impactModel(state,focus){
 const actors=state?.actors||[],history=state?.recovery_events||[],events=history.filter(e=>!focus||e.cause_actor===focus.cause&&e.sequence>focus.after);
 const byId=new Map(actors.map(a=>[a.id,a])),latest=new Map();
 // Keep history intact; only the latest assignment of each kind can still be pending.
 for(const e of history)if(['sts_reassigned','yard_reassigned'].includes(e.kind)){
  const key=`${e.kind}|${e.cargo}`,old=latest.get(key);
  if(!old||e.sequence>old.sequence)latest.set(key,e);
 }
 const activeEvents=events.filter(e=>{
  if(latest.get(`${e.kind}|${e.cargo}`)!==e)return false;
  const cargo=byId.get(e.cargo),target=byId.get(e.to_actor);
  if(cargo?.type!=='container'||!target)return false;
  if(e.kind==='sts_reassigned')return ['Ship','STS'].includes(cargo.state)&&
   (cargo.assigned_sts==null||target.name===`STS-${String(cargo.assigned_sts).padStart(2,'0')}`);
  return ['Ship','STS','AGV','RMG'].includes(cargo.state)&&
   (cargo.planned_rmg==null||target.name===`RMG-${String(cargo.planned_rmg).padStart(2,'0')}`);
 });
 const related=new Set(activeEvents.map(e=>e.to_actor)),cargos=new Set(activeEvents.map(e=>e.cargo)),failed=new Set();
 for(const a of actors)if(a.incident_pickup_failure||a.incident_placement_failure||a.incident_faults?.length||a.incident_needs_position_fix)related.add(a.id);
 for(const a of actors)if(a.fault_scope==='equipment'){related.add(a.id);failed.add(a.id);}
 for(const a of actors)if(a.fault_evacuating||a.incident_road_waiting||a.incident_detour_active)related.add(a.id);
 for(const a of actors)if(a.type==='agv'&&(failed.has(a.sts)||failed.has(a.rmg)||cargos.has(a.cargo_actor)||
  cargos.has(a.cargo)||a.cargo_items?.some(c=>cargos.has(c.id))||a.equipment_waiting||a.recovery_route_pending))related.add(a.id);
 // Include queues caused by a failed/affected vehicle, including indirect blockers.
 for(let n=0;n<actors.length;n++){
  let changed=false;
  for(const a of actors)if(a.type==='agv'&&!related.has(a.id)&&actors.some(b=>b.type==='agv'&&related.has(b.id)&&Number.isInteger(b.vehicle_id)&&
    [a.fms_blocking_agv,a.lidar_blocking_agv].includes(b.vehicle_id))){related.add(a.id);changed=true;}
  if(!changed)break;
 }
 // Worker causality is exported by the engine with current hold flags; do not
 // extend it using stale blocker IDs from a vehicle that has already resumed.
 for(const a of actors)if(a.emergency_test?.active)related.add(a.id);
 for(const e of state?.worker_impacts||[])if(byId.get(e.cause_actor)?.emergency_test?.active)related.add(e.to_actor);
 return {events,activeEvents,related};
}

export function impactFeed(state,focus,live){
 const {events,related}=impactModel(state,focus),actors=state?.actors||[],name=id=>actors.find(a=>a.id===id)?.name||id;
 const groups=new Map();
 for(const e of events){const key=`${e.kind}|${e.from_actor}|${e.to_actor}`;const old=groups.get(key);groups.set(key,{...e,count:(old?.count||0)+1});}
 const types={loaded_agv_dispatched:'대체 AGV · R/S 출동',loaded_agv_lift:'R/S 화물 인양',loaded_agv_seated:'대체 AGV 안착',loaded_agv_resumed:'대체 AGV 작업 승계',fault:'고장 격리',repair:'수리',evacuation_started:'고장 STS 대피',sts_reassigned:'STS 작업 재배정',yard_reassigned:'야드 목적지 재배정',agv_reassigned:'주차장 출구 대기 AGV 교체',agv_emergency_started:'AGV 돌발시험 시작',agv_emergency_finished:'AGV 돌발 정지 결과',worker_entered:'작업자 돌발 진입',worker_removed:'작업자 제거 · 운송 재개 허용',worker_impact_started:'작업자 영향 발생',worker_impact_cleared:'작업자 영향 추적 종료'};
 Object.assign(types,Object.fromEntries(Object.values(incidentActions).flat()));
 Object.assign(types,{road_rerouted:'도로 차단 우회',road_isolated:'AGV 고장 · 현장 통제',road_waiting:'통제 구역 앞 대기',road_reopened:'복구 확인 · 도로 재개방',incident_pickup_reassigned:'STS 인계 AGV 재배정',standby_released:'고장 AGV의 사전 대기 배정 해제'});types.incident_repaired='돌발상황 복구';
 types.pickup_retry='STS 체결 후퇴·재접근';
 types.sts_pickup_resumed='수리 후 기존 STS 인양 재개';
 Object.assign(types,{weather_changed:'날씨 변경',fail_reboot:'AGV 센서 재부팅',sensor_rebooted:'AGV 센서 복구',misdelivery:'오배송 지시 감지',fail_pickup:'스프레더 체결 실패',fail_placement:'적재 착좌 확인 실패'});
 const road=state?.central_road;
 const roadUsage=road?`<p class="note">중앙도로 실제 주행 · 적재 ${Math.round((road.loaded_east_m||0)+(road.loaded_west_m||0))}m · 빈차 ${Math.round((road.empty_east_m||0)+(road.empty_west_m||0))}m</p>`:'';
 return `<section class="impact-feed"><div class="impact-heading"><strong>고장·돌발상황 영향 · 재배정 기록</strong><span>${live?'엔진 실시간':'최근 실행 기록'} · 영향 대상 ${actors.filter(a=>related.has(a.id)&&['sts','rmg','agv'].includes(a.type)).length}대</span><button class="link" data-impact="all">전체 기록</button></div>${workerImpactList(state,focus)}<div class="impact-events">${[...groups.values()].slice(-12).reverse().map(e=>`<button class="impact-event" data-actor="${esc(e.to_actor)}"><b>${esc(types[e.kind]||e.kind)}${e.count>1?` ${e.count}건`:''}</b><span>${esc(name(e.from_actor))}${e.to_actor!==e.from_actor?` → ${esc(name(e.to_actor))}`:''} · ${Math.floor(e.simulation_seconds)}초${e.reason?` · ${esc(workerImpactReason(e.reason))}`:''}</span></button>`).join('')||'<p>장비 오류 또는 작업자 진입을 실행하면 실제 영향과 이력이 표시됩니다.</p>'}</div>${roadUsage}<small>빨강: 고장·작업자 · 주황: 현재 영향 대상 · 파랑: AGV 잔여 경로 · 주황 점선: 계획 재배정 연결</small></section>`;
}

export const workerImpactReason=r=>({worker_intrusion:'작업자 감지·제동·정지 유지',fms_queue:'FMS 도로 예약 대기',lidar_queue:'LiDAR 앞차 안전정지',sts_handover_wait:'STS 배정 AGV 인계 대기',rmg_reservation_wait:'RMG 인수·출차 예약 유지'}[r]||r);
function currentWorkerImpacts(state,focus){
 const active=new Set((state?.actors||[]).filter(a=>a.emergency_test?.active).map(a=>a.id));
 return (state?.worker_impacts||[]).filter(e=>active.has(e.cause_actor)&&(!focus||focus.cause===e.cause_actor));
}
export function workerImpactDetails(state,a){
 const rows=currentWorkerImpacts(state).filter(e=>e.to_actor===a?.id),byId=new Map((state?.actors||[]).map(v=>[v.id,v]));
 return rows.length?`<div class="note amber worker-impact-detail">${rows.map(e=>`${esc(byId.get(e.cause_actor)?.name||e.cause_actor)} 작업자 상황 → ${esc(workerImpactReason(e.reason))} · ${Math.floor(e.duration_seconds||0)}초${e.from_actor!==e.to_actor?` · 직접 선행 장비 ${esc(byId.get(e.from_actor)?.name||e.from_actor)}`:''}`).join('<br>')}</div>`:'';
}
function workerImpactList(state,focus){
 const rows=currentWorkerImpacts(state,focus),byId=new Map((state?.actors||[]).map(a=>[a.id,a]));if(!rows.length)return '';
 return `<div class="worker-impact-list"><strong>현재 작업자 상황의 실제 영향</strong>${rows.map(e=>{
  const a=byId.get(e.to_actor),from=byId.get(e.from_actor);
  return `<button class="worker-impact-row" data-actor="${esc(e.to_actor)}"><b>${esc(a?.name||e.to_actor)}</b><span>${esc(workerImpactReason(e.reason))}${e.from_actor!==e.to_actor?` ← ${esc(from?.name||e.from_actor)}`:''}</span><small>${a?.type==='agv'?`${Number.isFinite(a.speed_mps)?a.speed_mps.toFixed(2):'—'} m/s · `:''}${Math.floor(e.duration_seconds||0)}초</small></button>`;
 }).join('')}<small>현재 예약·LiDAR·인계 대기만 연결합니다. 예약 중인 RMG는 실제 작업 정지와 구분합니다.</small></div>`;
}

export function impactOverlay(state,focus,X,Y,selected){
 const {activeEvents,related}=impactModel(state,focus),actors=state?.actors||[];
 const points=ps=>ps.filter(p=>Array.isArray(p)&&p.slice(0,2).every(Number.isFinite)).map(p=>`${X(p[0])},${Y(p[1])}`).join(' ');
 let html='';
 for(const c of state?.incident_road_closures||[]){
  if(c.polygon_m?.length===4){
   html+=`<polygon class="incident-road-closure" points="${points(c.polygon_m)}" fill="#e9855b" fill-opacity=".45" stroke="#b33b25" stroke-width="1.5"><title>AGV 고장 · 현장 통제 ${esc(c.control_radius_m)}m</title></polygon>`;
   continue;
  }
  const p=c.min_m,q=c.max_m;if(!p||!q)continue;html+=`<rect class="incident-road-closure" x="${Math.min(X(p[0]),X(q[0]))}" y="${Math.min(Y(p[1]),Y(q[1]))}" width="${Math.max(4,Math.abs(X(q[0])-X(p[0])))}" height="${Math.max(4,Math.abs(Y(q[1])-Y(p[1])))}" fill="#e9855b" stroke="#b33b25" stroke-width="1.5"><title>돌발상황 · 도로 차단</title></rect>`;}
 for(const c of state?.incident_yard_blocks||[]){const p=c.position_m;if(p)html+=`<rect x="${X(p[0])-5}" y="${Y(p[1])-5}" width="10" height="10" fill="#e9855b"><title>목적 슬롯 점유</title></rect>`;}
 for(const e of activeEvents.slice(-40))html+=`<polyline class="plan-connection" points="${points([e.from_position_m,e.to_position_m])}"/>`;
 for(const a of actors.filter(a=>['sts','rmg','agv','rs'].includes(a.type))){
  const p=mapPosition(a);if(!p)continue;
  if(a.fault||a.incident_pickup_failure||a.incident_placement_failure||a.incident_faults?.length)html+=`<circle class="fault-halo" cx="${X(p[0])}" cy="${Y(p[1])}" r="9"/>`;
  else if(related.has(a.id))html+=`<circle class="impact-halo" cx="${X(p[0])}" cy="${Y(p[1])}" r="7"/>`;
  if(related.has(a.id)||a.id===selected||a.type==='rs'&&a.loaded_recovery_active){
   if(['agv','rs'].includes(a.type)&&a.route?.length)html+=`<polyline class="impact-route" points="${points([p,...a.route.map(n=>n.position_m)])}"/>`;
  }
 }
 return `<g class="impact-overlay" pointer-events="none">${html}</g>`;
}

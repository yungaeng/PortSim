import {renderAGVPhysics,renderAGVSensors} from './agv-physics-view.mjs';
import {renderFrictionDetails} from './friction-view.mjs';
const esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const n=(v,d=2)=>Number.isFinite(v)?v.toLocaleString('ko-KR',{maximumFractionDigits:d}):'—';
const row=(k,v)=>`<div class="kv-row"><span>${k}</span><b>${v}</b></div>`;
export function weatherHoldStatus(state){
 const h=state?.weather_hold;
 if(!h)return '기상 운용 상태 미수신';
 return (h.active?'기상 회복 대기':'기상 대기 해제')+(h.active?` · ${esc(h.reason||'회복 조건 확인 중')} · 회복 확인 ${n(h.recovery_elapsed_seconds,1)} / ${n(h.recovery_required_seconds,1)} 시뮬레이션 초`:'')+(state.manual_paused?' · 수동 일시정지 유지':'');
}
export function environmentLive(state,live){
 const e=state?.environment;if(!e)return '<div class="note">현재 엔진의 환경 관측값이 없습니다. 새 빌드에서 실행하면 표시됩니다.</div>';
 return `<div class="detail-subhead">현재 적용된 환경 <span>${live?'LIVE':'저장 기록 · 실시간 아님'}</span></div>`+
  row('기상 운용',weatherHoldStatus(state))+row('강수 / 강설',`${n(e.rain_mm_h)} / ${n(e.snow_mm_h)} mm/h`)+row('실제 바람 / 평균',`${n(e.effective_wind_mps)} / ${n(e.wind_mps)} m/s`)+
  row('수막 / 적설',`${n(e.water_film_mm,3)} / ${n(e.snow_depth_mm,3)} mm`)+row('젖음 / 눈 / 얼음 피복',`${n(e.wetness*100,1)} / ${n(e.snow_coverage*100,1)} / ${n(e.ice_coverage*100,1)} %`)+
  row('수막 / 적설 변화',`${n(e.water_change_mm_h,3)} / ${n(e.snow_change_mm_h,3)} mm/h`)+row('타이어 정지 / 운동 마찰계수',`${n(e.tire_mu_static,3)} / ${n(e.tire_mu_kinetic,3)}`)+row('레일 정지 마찰계수',n(e.rail_mu_static,3))+'<div class="note">환경 관측 전용입니다. 날씨 변경은 언리얼에서 설정합니다.</div>';
}
export function recordLiveSample(history,state,actor){
 const time=state?.simulation_seconds,p=actor?.agv_physics;if(!p||!Number.isFinite(time))return [];
 const last=history.at(-1),key=`${state.run_id}|${actor.id}`;
 if(last&&(last.key!==key||last.time>time))history=[];
 if(history.at(-1)?.time===time)return history;
 return [...history,{key,time,speed:actor.speed_mps,slip:p.max_slip_ratio*100,ltr:p.load_transfer_ratio,mu:actor.friction?.tire_static_mu??state.environment?.tire_mu_static}].slice(-180);
}
function trace(samples,key,label,color){
 const valid=samples.filter(s=>Number.isFinite(s[key]));if(!valid.length)return `<div class="note">${label}: 관측 대기</div>`;
 const t0=valid[0].time,t1=valid.at(-1).time,lo=Math.min(0,...valid.map(s=>s[key])),hi=Math.max(lo+.01,...valid.map(s=>s[key]));
 const points=valid.map(s=>`${(10+580*(s.time-t0)/Math.max(1,t1-t0)).toFixed(2)},${(85-70*(s[key]-lo)/(hi-lo)).toFixed(2)}`).join(' ');
 return `<figure class="live-trace"><figcaption>${label} · 현재 ${n(valid.at(-1)[key],3)} <span>${n(t0,1)}~${n(t1,1)} 시뮬레이션 초</span></figcaption><svg viewBox="0 0 600 100" role="img" aria-label="${label} 실시간 추이"><path d="M10 10V85H590" fill="none" stroke="#ccd9d7"/><polyline points="${points}" fill="none" stroke="${color}" stroke-width="2"/><text x="12" y="14">${n(hi,2)}</text><text x="12" y="98">${n(lo,2)}</text></svg></figure>`;
}
export function livePhysicsResults(state,live,actor,history){
 if(!actor)return '<section class="panel"><div class="form-body">시뮬레이션을 실행하면 현재 AGV가 표시됩니다.</div></section>';
 return `<section class="panel"><div class="panel-head"><div><h2>${esc(actor.name)} · 실시간 물리</h2><p>현재 실행 · ${n(state?.simulation_seconds,2)} 시뮬레이션 초</p></div><span class="live-label">${live?'● LIVE':'○ 저장 기록'}</span></div><div class="form-body">`+
  (!live?'<div class="note amber">엔진 연결이 없어 마지막 값을 표시합니다. 새 관측을 수신하면 실시간 갱신을 재개합니다.</div>':'')+
  trace(history,'speed','차체 속도 (m/s)','#159681')+trace(history,'slip','현재 최대 슬립 (%)','#c88435')+trace(history,'ltr','좌우 하중이동률 LTR','#779bbc')+
  renderFrictionDetails(actor)+renderAGVPhysics(actor)+(actor.agv_physics?renderAGVSensors(actor):'')+'</div></section>';
}

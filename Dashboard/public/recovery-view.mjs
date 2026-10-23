const escape=value=>String(value??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const row=(label,value)=>`<div class="kv-row"><span>${label}</span><b>${escape(value)}</b></div>`;
export function faultDialogTitle(state,live){
 if(!live)return '최근 실행 기록에 장비 오류가 있습니다';
 return state?.fault?'전체 운전이 정지했습니다':'장비 고장 격리 · 다른 작업은 계속 운영합니다';
}
export function renderRecoveryDetails(actor){
 let html='';
 if(actor?.fault_scope==='equipment'){
  html=`<div class="detail-subhead">${actor.gantry_failure?'갠트리 주행 고장':'센서 고장'} · 장비 격리</div>`;
  const handling=actor.retained_load?'잠금·화물 유지 · 수리 대기':actor.recovery_status==='verification_repair_required'?'안착 검증 미완료 · 수리 대기':'미인양 작업 재배정 · 대체 불가 시 대기';
  html+=row('화물 처리',handling);
  html+=row('운영 범위','관련 장비·작업만 격리');
 }
 if(actor?.rail_safety_hold)html+=row('레일 이동','인접 크레인 간격 확보 대기');
 if(actor?.gantry_failure)html+=row('갠트리 고장','주행 불가 · 현재 레일 위치 유지 · 수동 수리 필요');
 if(actor?.sensor_failure){
  const phases={rebooting:'센서 재부팅',technician_repair:'현장 수리',self_check:'센서 재확인',awaiting_safe_stop:'대피 주행 감속 후 재개'};
  const seconds=actor.sensor_repair_remaining_seconds;
  html+=row('센서 복구',Number.isFinite(seconds)&&seconds>=0?`${phases[actor.sensor_repair_phase]||'수리 대기'} · ${Math.ceil(seconds)}초 남음`:'자동 복구 해제 · 수동 수리 대기');
 }
 if(actor?.fault_evacuating)html+=row(actor.sensor_failure?'고장 장비 대피':'수리 후 복귀',`레일 Y ${Number(actor.fault_evacuation_target_y_m).toFixed(1)}m · 추월 없이 저속 이동`);
 if(actor?.yard_admission_suspended)html+=row('야드 접근권','고장 작업의 미사용 접근권 해제 · 복구 후 재확보');
 if(actor?.reassignments>0)html+=row('재배정 횟수',actor.reassignments);
 if(actor?.original_sts>0 && actor?.assigned_sts>0 && actor.original_sts!==actor.assigned_sts)
  html+=row('STS 담당 변경',`STS ${actor.original_sts} → STS ${actor.assigned_sts}`);
 if(actor?.recovery_hold)html+=row('복구 대기',({sts_no_reachable_alternative:'도달 가능한 대체 STS 없음',no_compatible_healthy_yard:'정상 RMG의 적합한 슬롯 없음'})[actor.recovery_hold]||actor.recovery_hold);
 return html;
}

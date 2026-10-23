const esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const n=(v,d=3)=>Number.isFinite(v)?v.toLocaleString('ko-KR',{maximumFractionDigits:d}):'—';
const row=(k,v)=>`<div class="kv-row"><span>${k}</span><b>${v}</b></div>`;
export const emergencyOutcome=v=>({idle:'시험 없음',worker_entered:'작업자 진입 · 감지 대기',detected_reaction_pending:'LiDAR 감지 · 반응 지연',physical_braking:'타이어 물리 급제동 중',stopped_clear:'작업자 앞 안전 정지',worker_contact:'작업자 접촉',cargo_shift_limit:'화물 이동 한계 초과',restraint_overload:'고정장치 용량 초과',wheel_contact_limit:'바퀴 접촉·롤 한계 초과',timeout:'제한 시간 초과',dynamics_invalid:'물리 계산 오류',external_constraint_invalidated:'외부 강제 정지로 시험 무효'}[v]||v||'시험 없음');

export function emergencyControls(a,disabled){
 if(a.type!=='agv')return '';
 const e=a.emergency_test,active=!!e?.active,canStart=!!e&&!disabled&&!active&&!a.fault&&a.stage===3&&a.speed_mps>=1&&!a.fms_hold&&!a.controlled_stop;
 const clearable=!disabled&&active&&e.stopped&&!e.collision&&(e.peak_cargo_shift_m??1)<=.001;
 return `<section class="fault-controls"><h3>AGV 돌발상황 통합 시험</h3><p>젖은 노면 + 편심 40ft + 작업자 진입</p><div class="control-actions"><button class="button" data-command="test_wet_worker" ${canStart?'':'disabled'}>고정 화물 시험</button><button class="button danger" data-command="test_wet_unsecured" ${canStart?'':'disabled'}>미고정 화물 시험</button><button class="button" data-command="clear_worker" ${clearable?'':'disabled'}>안전 정지 시험 해제</button></div><div class="note">40ft 한 개를 싣고 1m/s 이상으로 야드에 이동 중인 직선 구간에서 실행합니다. 엔진이 예약 도로 길이를 추가 확인합니다. 선택 차량의 노면·화물 CoG만 에디터 Emergency Test 설정으로 바뀝니다(기본 젖음 100%, CoG 좌측 0.8m·위 0.5m 편차). 시험 중 경로 예약은 유지됩니다. 안전 정지 후 해제하면 원래 CoG·환경으로 복귀합니다. 충돌·화물 이동·한계 초과 시험은 시뮬레이션 초기화가 필요합니다.</div>${emergencyDetails(a)}</section>`;
}

export function emergencyDetails(a){
 const e=a.emergency_test;if(!e||e.outcome==='idle')return '';
 const p=a.agv_physics||{},vec=v=>Array.isArray(v)?v.map(x=>n(x)).join(' / '):'—';
 return `<div class="detail-subhead">돌발상황 · ${esc(emergencyOutcome(e.outcome))}</div>`+
  row('시험 상태',e.active?'시험 유지 · 운송 보류':'완료 기록')+row('화물 고정',e.cargo_secured?'고정 · 반력 계산':'미고정 · 데크 마찰 접촉')+
  row('경과 / 감지 / 제동 시작',`${n(e.elapsed_s)} / ${e.detected_at_s>=0?n(e.detected_at_s):'—'} / ${e.brake_at_s>=0?n(e.brake_at_s):'—'} s`)+
  row(e.stopped?'완료 제동거리':'현재 제동 이동거리',`${n(e.braking_distance_m)} m`)+row('최소 작업자 이격',`${n(e.minimum_clearance_m)} m`)+
  row('최대 슬립 / |LTR|',`${n(e.peak_slip*100,1)}% / ${n(e.peak_ltr)}`)+
  row('최대 관성력 / 고정장치 반력',`${n(e.peak_inertia_n/1000)} / ${n(e.peak_restraint_n/1000)} kN`)+
  row('화물 이동 XYZ / 최대 이동',`${vec(p.cargo_shift_m)} / ${n(e.peak_cargo_shift_m)} m`)+
  row('화물 상태',p.cargo_unsafe?'이동·고정 한계 초과':p.cargo_sliding?'데크 위 미끄러짐':'접촉 유지')+
  row('지역 시험 노면 젖음 / 결빙',`${n(e.wetness_override*100,0)}% / ${n(e.ice_override*100,0)}%`);
}

export function renderEmergencyReport(report){
 let h='<section class="panel"><div class="panel-head"><div><h2>돌발상황 통합 시험</h2><p>노면 · 편심 화물 · LiDAR 감지 지연 · 급제동 · 데크 접촉</p></div></div><div class="form-body">';
 if(!report?.cases?.length)return h+'<p>통합 시험 보고서가 없습니다. TestAGVDynamics.ps1 실행 후 여기에 결과가 표시됩니다.</p></div></section>';
 h+=`<div class="note ${report.valid?'':'amber'}">${report.valid?'계산·기대 결과 검증 완료':'검증 실패 포함'} · ${esc(report.generated_at)} · 과거 독립 시험 보고서입니다. 접촉·화물 이동은 위험 사례의 결과이며 안전 정지를 의미하지 않습니다.</div><div class="table-scroll"><table><thead><tr><th>사례</th><th>노면 μs/μk</th><th>화물 고정</th><th>제동 이동 m</th><th>최소 이격 m</th><th>화물 이동 m</th><th>최대 |LTR|</th><th>결과</th></tr></thead><tbody>`;
 for(const c of report.cases)h+=`<tr><td>${esc(c.input?.label||c.input?.id)}</td><td>${n(c.mu_static,2)} / ${n(c.mu_kinetic,2)}</td><td>${c.cargo_secured?'고정':'미고정'}</td><td>${n(c.braking_distance_m)}</td><td>${n(c.minimum_clearance_m)}</td><td>${n(c.peak_cargo_shift_m)}</td><td>${n(c.peak_ltr)}</td><td>${esc(emergencyOutcome(c.outcome))}${c.valid?'':' · 검증 실패'}</td></tr>`;
 h+='</tbody></table></div>';
 for(const c of report.cases)h+=`<details><summary>${esc(c.input?.label||c.input?.id)} · 입력·감지·관성력·시간 기록</summary><pre class="raw">${esc(JSON.stringify(c.input,null,2))}</pre>${row('감지 / 제동 시작',`${n(c.detected_at_s)} / ${n(c.brake_at_s)} s`)}${row('최대 관성력 / 고정장치 반력',`${n(c.peak_inertia_n/1000)} / ${n(c.peak_restraint_n/1000)} kN`)}<div class="table-scroll"><table><thead><tr><th>초</th><th>속도 m/s</th><th>장애물 이격 m</th><th>감지/제동</th><th>슬립 %</th><th>좌/우 kN</th><th>화물 이동 m</th><th>데크 힘 kN</th></tr></thead><tbody>${(c.samples||[]).map(s=>`<tr><td>${n(s.time_s,2)}</td><td>${n(s.speed_mps)}</td><td>${n(s.gap_m)}</td><td>${s.detected?'감지':'—'} / ${s.brake?'제동':'—'}</td><td>${n(s.slip*100,1)}</td><td>${n(s.left_kn,1)} / ${n(s.right_kn,1)}</td><td>${n(s.cargo_shift_m)}</td><td>${n(s.deck_force_kn)}</td></tr>`).join('')}</tbody></table></div></details>`;
 return h+'<div class="note">작업자는 갑자기 차로에 진입한 뒤 정지하는 기하학적 시험 장애물입니다. 화물은 평면 병진·정지/운동 마찰·이상적 고정장치로 계산합니다. 화물 회전·낙하·고정장치 파괴 이후와 3D 전복 이후는 해석하지 않습니다. 계수·센서 주기·지연·고정장치 용량은 실측 인증값이 아닌 시험 가정입니다.</div></div></section>';
}

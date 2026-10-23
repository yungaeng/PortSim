const esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const n=(v,d=3)=>Number.isFinite(v)?v.toLocaleString('ko-KR',{maximumFractionDigits:d}):'—';
import {emergencyDetails} from './agv-emergency-view.mjs';
const vec=v=>Array.isArray(v)?v.map(x=>n(x)).join(' / '):'—';
const row=(label,value)=>`<div class="kv-row"><span>${label}</span><b>${value}</b></div>`;
const caseNames={dry_brake:'건조 노면 제동',wet_brake:'젖은 노면 제동',ice_brake:'결빙 노면 제동',center_turn:'중앙 CoG 선회',eccentric_turn:'편심 CoG 선회',high_eccentric_turn:'높은 편심 CoG 선회',controlled_high_turn:'높은 편심 · 안정성 제어'};
export function renderAGVSensors(a){
 const p=a.agv_physics;
 return '<div class="detail-subhead">Wheel / Steering · IMU</div>'+row('차륜 RPM (후좌/후우/중좌/중우/전좌/전우)',(p.wheels||[]).map(w=>n(w.rpm,1)).join(' / '))+
  row('휠 누적거리',`${n(a.wheel_odometry_m)} m`)+row('전륜 평균 조향각',`${n(a.steering_angle_deg)}°`)+row('Transponder 추정 XYZ',`${vec(a.estimated_position_m)} m`)+row('위치 오차',`${n(a.position_error_mm)} mm`)+
  row('IMU specific force X/Y/Z',`${vec(p.imu_specific_force_mps2)} m/s²`)+row('IMU 각속도 roll/pitch/yaw',`${vec(p.imu_gyro_rad_s)} rad/s`)+
  '<div class="detail-subhead">FMS · LiDAR</div>'+row('FMS',a.fms_hold?'예약 대기':'진입 허용')+row('다음 목표 XYZ',`${vec(a.fms_next_node_m)} m`)+
  row('전방 / 후방 장애물',`${a.lidar_front_obstacle_m>=0?n(a.lidar_front_obstacle_m)+' m':'감지 없음'} / ${a.lidar_rear_obstacle_m>=0?n(a.lidar_rear_obstacle_m)+' m':'감지 없음'}`)+
  row('안전거리 / 판정',`${n(a.lidar_safety_distance_m)} m / ${p.physical_emergency_braking?'돌발시험 물리 급제동':a.controlled_stop?'기존 강제 정지':'주행 허용'}`)+
  '<div class="note">휠 엔코더는 바퀴 회전량을 적분하므로 슬립 시 실제 이동거리와 달라집니다. IMU는 노이즈·지연 없는 합성 관측값입니다. 위치추정은 이상적인 Transponder/관성 보정 가정이며 실제 센서 융합 필터나 실장 좌표를 검증한 모델은 아닙니다.</div>';
}
export function renderAGVPhysics(a){
 const p=a?.agv_physics;if(!p)return '<div class="note">AGV 타이어·안정성 데이터가 필요합니다. 새 엔진을 실행하세요.</div>';
 const total=p.left_load_n+p.right_load_n,left=Number.isFinite(total)&&total>0?Math.max(0,Math.min(100,p.left_load_n/total*100)):50;
 let h='<div class="detail-subhead">타이어 · CoG 안정성 <span>6개 등가 접촉점</span></div>';
 h+=row('전체 질량 / 합성 CoG',`${n(p.mass_kg/1000)} t · ${vec(p.combined_cog_m)} m`);
 h+=row('화물 CoG (노면 기준)',`${vec(p.cargo_cog_m)} m`);
 h+=row('차체 속도 X / Y',`${vec(p.velocity_body_mps)} m/s`)+row('가속도 X / Y',`${vec(p.acceleration_body_mps2)} m/s²`);
 h+=row('실제 회전반경',p.turn_radius_m>0?`${n(p.turn_radius_m)} m`:'직진 / 정지')+row('롤 / 피치',`${n(p.roll_deg)}° / ${n(p.pitch_deg)}°`);
 h+=row('최대 현재 / 누적 슬립률',`${n(p.max_slip_ratio*100)}% / ${n(p.peak_slip_ratio*100)}%`);
 h+=`<div class="agv-load-bar" aria-label="좌우 수직하중 비율"><span style="width:${left}%">좌 ${n(p.left_load_n/1000,1)} kN</span><span>우 ${n(p.right_load_n/1000,1)} kN</span></div>`;
 h+=row('좌우 하중이동률 LTR',n(p.load_transfer_ratio))+row('접촉 / 안정성',p.overturn_threshold?'접촉·롤 한계 초과 · 해석 범위 이탈':p.wheel_lift?'바퀴 들림':Math.abs(p.load_transfer_ratio)>.8?'하중 편중 주의':'접촉 유지');
 h+=row('안정성 속도 제어',p.settings?.stability_control?(p.stability_limited?'감속 적용':'활성'):'시험용 해제')+row('현 곡률의 안정성 속도',`${n(p.stability_speed_limit_mps)} m/s`);
 h+=row('IMU specific force X/Y/Z',`${vec(p.imu_specific_force_mps2)} m/s²`)+row('IMU 각속도 roll/pitch/yaw',`${vec(p.imu_gyro_rad_s)} rad/s`);
 h+=p.braking?row('진행 중 제동 시간 / 거리',`${n(p.braking_elapsed_s)} s / ${n(p.braking_distance_m)} m`):row('최근 완료한 실제 제동',p.last_brake_valid?`${n(p.last_braking_seconds)} s / ${n(p.last_braking_distance_m)} m`:'유효한 완료 기록 없음');
 if(p.physical_emergency_braking)h+='<div class="note amber">작업자 감지 후 타이어 접지력으로 급제동 중입니다. 정지 전까지 실제 차량이 계속 이동할 수 있습니다.</div>';
 if(p.legacy_stop_constraint)h+='<div class="note amber">현재 FMS·일반 차량 LiDAR 정지는 기존 강제 정지 제약이며 돌발상황 시험의 물리 급제동과 구분합니다.</div>';
 h+='<div class="table-scroll"><table><thead><tr><th>바퀴군</th><th>RPM</th><th>하중 kN</th><th>토크 kNm</th><th>슬립 %</th></tr></thead><tbody>';
 const names={rear_left:'후좌',rear_right:'후우',middle_left:'중좌',middle_right:'중우',front_left:'전좌',front_right:'전우'};
 for(const w of p.wheels||[])h+=`<tr><td>${esc(names[w.name]||w.name)}${w.sliding?' · 슬립':''}</td><td>${n(w.rpm,1)}</td><td>${n(w.normal_n/1000,1)}</td><td>${n(w.applied_torque_nm/1000,2)}</td><td>${n(w.slip_ratio*100,2)}</td></tr>`;
 h+='</tbody></table></div><div class="note">X 전진 · Y 좌측 · Z 위. CoG 높이는 노면 기준입니다. IMU는 바이어스·노이즈·지연 없는 합성값이며 specific force의 Z에는 중력 g가 포함됩니다. 롤·피치는 등가 스프링 모델 상태이며 차체 메시와 충돌체의 3D 전복 애니메이션은 아닙니다.</div>';
 h+=`<details><summary>적용 가정값 · 전체</summary><pre class="raw">${esc(JSON.stringify(p.settings,null,2))}</pre></details>`;
 return h+emergencyDetails(a);
}

export function renderAGVExperiments(report){
 const title='<div class="panel-head"><div><h2>AGV 노면 · CoG 물리 시험</h2><p>터미널 운송과 분리된 반복 시험 · 동일 타이어/하중 솔버 사용</p></div></div>';
 if(!report?.cases?.length)return `<section class="panel">${title}<div class="form-body"><p>시험 결과가 아직 없습니다.</p><p>AGV_PhysicsBench.json에서 조건을 정하고 TestAGVDynamics.ps1을 실행하세요.</p></div></section>`;
 let h=`<section class="panel">${title}<div class="form-body"><div class="note ${report.valid?'':'amber'}">${report.valid?'시험 보고서 생성 완료':'실패 항목이 포함된 보고서'} · ${esc(report.generated_at)}. 실시간 운송 기록이 아닙니다. 입력 조건과 표본을 펼쳐 확인하세요.</div><div class="table-scroll"><table><thead><tr><th>시험</th><th>μ 정지/운동</th><th>제동거리 m</th><th>제동시간 s</th><th>최대 슬립 %</th><th>최대 |LTR|</th><th>종료</th></tr></thead><tbody>`;
 for(const c of report.cases)h+=`<tr><td>${esc(caseNames[c.input?.id]||c.input?.id)}</td><td>${n(c.mu_static,2)} / ${n(c.mu_kinetic,2)}</td><td>${c.brake_completed?n(c.braking_distance_m):'—'}</td><td>${c.brake_completed?n(c.braking_seconds):'—'}</td><td>${n(c.peak_slip*100,1)}</td><td>${n(c.peak_ltr)}</td><td>${!c.valid?'실패':c.contact_threshold?'접촉 한계 도달':c.brake_completed?'정지 완료':'설정 시간 완료'}</td></tr>`;
 h+='</tbody></table></div>';
 for(const c of report.cases){
  const samples=c.samples||[];
  h+=`<details><summary>${esc(c.input?.id)} · 입력 및 시간별 결과</summary><pre class="raw">${esc(JSON.stringify(c.input,null,2))}</pre><div class="table-scroll"><table><thead><tr><th>초</th><th>속도 m/s</th><th>슬립 %</th><th>횡가속 m/s²</th><th>좌/우 kN</th><th>롤 °</th></tr></thead><tbody>${samples.map(s=>`<tr><td>${n(s.time_s,1)}</td><td>${n(s.speed_mps,2)}</td><td>${n(s.slip_ratio*100,1)}</td><td>${n(s.lateral_mps2,2)}</td><td>${n(s.left_kn,1)} / ${n(s.right_kn,1)}</td><td>${n(s.roll_deg,2)}</td></tr>`).join('')}</tbody></table></div></details>`;
 }
 return h+'<div class="note">차체·타이어·서스펜션은 편집 가능한 가정입니다. 코너 시험은 위험 차이를 관찰하려고 현장 2.5 m/s 회전 제한을 적용하지 않습니다. 한계 초과 시 시험을 끝내며 3D 전복 이후 거동은 계산하지 않습니다. 작업자·급제동·데크 마찰을 연결한 시험은 위 통합 시험 보고서에서 별도로 확인합니다.</div></div></section>';
}

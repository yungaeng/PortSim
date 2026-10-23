const number=(value,unit='')=>Number.isFinite(value)?`${value.toLocaleString('ko-KR',{maximumFractionDigits:3})}${unit}`:'—';
const row=(label,value)=>`<div class="kv-row"><span>${label}</span><b>${value}</b></div>`;
export function renderFrictionDetails(actor){
 const f=actor?.friction;if(!f)return '';
 let html='<div class="detail-subhead">날씨 · 접촉 마찰 <span>가정 계수 적용</span></div>';
 html+=row('젖음 / 적설 / 결빙',`${number(f.wetness*100,'%')} / ${number(f.snow*100,'%')} / ${number(f.ice*100,'%')}`);
 if(Number.isFinite(f.water_film_mm)){
  html+=row('물막 / 쌓인 눈',`${number(f.water_film_mm,' mm')} / ${number(f.snow_depth_mm,' mm')}`);
  html+=row('건조도',number(f.dryness*100,'%'));
  html+=row('노면 수분 상태',f.drying?'배수·건조 중':f.water_change_mm_per_hour>0?'젖는 중':'유지');
  html+=row('물막 / 적설 변화율',`${number(f.water_change_mm_per_hour)} / ${number(f.snow_change_mm_per_hour)} mm/h`);
 }
 const pair=(prefix)=>`${number(f[`${prefix}_static_mu`])} / ${number(f[`${prefix}_kinetic_mu`])}`;
 if(actor.type==='agv'){
  html+=row('타이어 μ 정지 / 운동',pair('tire'))+row('접지 수직하중',number(f.normal_force_n/1000,' kN'));
  html+=row('외풍·공기저항 여유분',number(f.wind_reserve_n,' N'));
  html+=row('제동 감속 한계',number(f.braking_mps2,' m/s²'))+row('코너 속도 상한',number(f.curve_speed_mps,' m/s'));
  html+=row('현재 속도의 계산 제동거리',f.traction_available===false?'접지력 부족':number(f.stopping_distance_m,' m'));
  html+=`<div class="note">${actor.agv_physics?'타이어 토크·결합 슬립·CoG 하중 분배를 실제 주행에 적용합니다. 아래 실제 제동 기록은 완료한 물리 제동만 표시합니다.':'이 기록은 타이어 상세 물리 도입 전 데이터입니다.'} FMS·LiDAR의 기존 즉시 안전정지는 별도 제약이며 돌발 장애물 통합은 다음 단계입니다.</div>`;
 }else{
  html+=row('레일 μ 정지 / 운동',pair('rail'))+row('트롤리 / 갠트리 가속 한계',`${number(f.trolley_acceleration_limit_mps2)} / ${number(f.gantry_acceleration_limit_mps2)} m/s²`);
  html+=row('스프레더 접촉 상태',f.mechanically_locked?'트위스트록 기계 결합':f.seating_active?(f.seating_grip_ok?'안착 마찰 조건 충족':'안착 마찰 부족 · 잠금 대기'):'안착 판정 비활성');
  if(f.seating_active)html+=row('안착 필요 힘 / 마찰 한계',`${number(f.seating_demand_n)} / ${number(f.seating_capacity_n)} N`);
 }
 html+=row('금속 접촉 μ 정지 / 운동',pair('steel'));
 return html+'<div class="note">강수·강설에 따라 물막과 눈이 쌓이고, 그친 뒤 배수·건조·융해가 진행되며 마찰계수가 갱신됩니다. 시간은 시뮬레이션 배속을 따릅니다. 계수와 건조·융해 속도는 조정 가능한 가정이며, 잠긴 컨테이너는 트위스트록으로 유지됩니다.</div>';
}

export const incidentActions={
 agv:[['fail_reboot','센서 오류 · 재부팅'],['fail_sensor_repair','센서 고장 · 현장 수리'],['localization_loss','위치 인식 상실'],['fms_delay','FMS 통신 지연'],['fms_disconnect','FMS 통신 단절'],['road_block','진행 도로 차단'],['fail','장비 자체 오류 · 구동 정지'],['misdelivery','오배송 지시 · 목적지 확인 실패'],['worker_enter','작업자 출현']],
 sts:[['equipment_failure','장비 자체 오류 · 제어기 현장 복구'],['restart_controller','일시 제어기 오류 · 재시작'],['fail_pickup','스프레더 체결 실패'],['fail_release','스프레더 체결 해제 실패'],['fail_hoist','호이스트 구동 고장'],['agv_unavailable','AGV 인계 불가'],['fail_gantry','갠트리 고장'],['fail_reboot','센서 오류 · 재부팅'],['fail_sensor_repair','센서 고장 · 현장 수리']],
 rmg:[['equipment_failure','장비 자체 오류 · 제어기 현장 복구'],['restart_controller','일시 제어기 오류 · 재시작'],['fail_pickup','스프레더 체결 실패'],['fail_release','스프레더 체결 해제 실패'],['fail_gantry','갠트리 고장'],['fail_placement','적재 착좌 확인 실패'],['slot_occupied','목적 슬롯 점유'],['height_mismatch','적재 높이 정보 불일치'],['fail_reboot','센서 오류 · 재부팅'],['fail_sensor_repair','센서 고장 · 현장 수리']],
 environment:[['set_environment','날씨 변경']]
};
export const userClearsIncident=action=>action==='worker_enter'||action==='road_block';
export const weatherFields={rain_mm_h:[0,100,'강우량 (mm/h)'],snow_mm_h:[0,100,'강설량 (mm/h)'],wind_mps:[0,7,'기준 풍속 (m/s)'],fog_m:[0,1000,'안개 시정 (m, 0은 해제)'],ice:[0,1,'결빙 입력 강도 (0~1)']};
export const triggers={immediate:'지정 시간 후',moving:'AGV 이동 중',loaded:'AGV 화물 적재 중',pickup:'크레인 체결 단계',placing:'크레인 하강·적재 단계',release:'화물 착좌 후 잠금 해제 직전',handover:'AGV 인계 단계'};
export function cleanControlRadius(value=1){
 if(!Number.isFinite(value)||value<.5||value>20)throw Error('현장 통제 반경은 0.5~20m입니다.');
 return value;
}
export function cleanWeather(input){
 if(!input||typeof input!=='object'||Array.isArray(input))throw Error('날씨 값을 입력하세요.');
 const out={};for(const [key,value] of Object.entries(input)){
  const field=weatherFields[key];if(!field||!Number.isFinite(value)||value<field[0]||value>field[1])throw Error('날씨 범위를 확인하세요: '+key);
  out[key]=value;
 }
 if(!Object.keys(out).length)throw Error('변경할 날씨를 하나 이상 입력하세요.');return out;
}
export function cleanStep(input,state){
 const type=input.actor_id==='environment'?'environment':state.actors.find(a=>a.id===input.actor_id)?.type;
 if(!incidentActions[type]?.some(([a])=>a===input.action))throw Error('장비에 지원하지 않는 상황입니다.');
 const out={actor_id:input.actor_id,action:input.action,delay:input.delay??0,trigger:input.trigger??'immediate'};
 if(!Number.isFinite(out.delay)||out.delay<0||out.delay>3600)throw Error('발생 지연은 0~3,600초입니다.');
 if(!Object.hasOwn(triggers,out.trigger)||out.trigger!=='immediate'&&!(type==='agv'?['moving','loaded']:['sts','rmg'].includes(type)?['pickup','placing','release','handover']:[]).includes(out.trigger))throw Error('장비에 맞는 발생 조건을 선택하세요.');
 if(out.action==='set_environment')out.weather=cleanWeather(input.weather);
 if(out.action==='worker_enter'){out.worker_distance_m=input.worker_distance_m??20;if(!Number.isFinite(out.worker_distance_m)||out.worker_distance_m<5||out.worker_distance_m>80)throw Error('작업자 거리는 5~80m입니다.');}
 if(type==='agv'&&out.action==='fail')out.control_radius_m=cleanControlRadius(input.control_radius_m);
 return out;
}
export function cleanScenario(input,state){
 if(!Array.isArray(input.steps)||input.steps.length<1||input.steps.length>12)throw Error('상황은 1~12개로 구성하세요.');
 const steps=input.steps.map(s=>cleanStep(s,state));
 // Multiple faults may overlap; repeating the same fault would lose ownership of its recovery timer.
 const seen=new Set();for(const s of steps){
  const kind=['fail_reboot','fail_sensor_repair'].includes(s.action)?'sensor':['equipment_failure','restart_controller'].includes(s.action)?'controller':s.action;
  const key=s.actor_id+'|'+kind;
  if(s.action!=='set_environment'&&seen.has(key))throw Error('같은 장비의 동일 오류는 한 번만 추가하세요.');
  seen.add(key);
 }
 return {name:String(input.name||'실시간 시나리오').slice(0,80),steps};
}

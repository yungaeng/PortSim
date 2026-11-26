const esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const time=n=>Number.isFinite(n)&&n>0;
const count=n=>Number.isInteger(n)&&n>=0;
export function equipmentPerformance(actor){
 const p=actor?.performance;
 if(p?.schema_version!==1)return {utilization:null,crane:null,berth:null};
 const observed=p.observed_seconds,working=p.working_seconds,b=p.berth;
 return {
  utilization:['sts','rmg','agv','rs'].includes(actor.type)&&time(observed)&&Number.isFinite(working)&&working>=0&&working<=observed?working/observed*100:null,
  crane:['sts','rmg'].includes(actor.type)&&time(observed)&&count(p.handled_containers)?p.handled_containers*3600/observed:null,
  berth:actor.type==='vessel'&&time(b?.operation_seconds)&&count(b?.handled_containers)?b.handled_containers*3600/b.operation_seconds:null
 };
}
export function renderEquipmentPerformance(actor,{compact=false}={}){
 const result=equipmentPerformance(actor),p=actor?.performance;
 const cell=(value,title)=>`<td class="performance-cell mono ${value===null?'muted':''}" title="${esc(title)}">${value===null?'—':value.toLocaleString('ko-KR',{minimumFractionDigits:1,maximumFractionDigits:1})}</td>`;
 const utilization=Number.isFinite(p?.observed_seconds)?`실제 작업 ${p.working_seconds?.toFixed(1)??'—'}초 ÷ 관측 운용 ${p.observed_seconds.toFixed(1)}초 × 100. 대기·고장은 작업시간에서 제외합니다.`:'관측 작업시간 기록이 없습니다. 새 엔진 실행부터 측정합니다.';
 const utilizationCell=cell(result.utilization,utilization);
 const crane=['sts','rmg'].includes(actor?.type)?`완료 컨테이너 ${p?.handled_containers??'—'}개 ÷ 관측 운용 시간. 크기에 관계없이 컨테이너 1개를 1개로 셉니다.`:'크레인에 적용하는 지표입니다.';
 const berth=actor?.type==='vessel'&&p?.berth?`선석 ${p.berth.berth_id}: 하역 인계·선박 적재 완료 ${p.berth.handled_containers}개 ÷ 해당 선석 작업시간.`:'선석 생산성은 선박 행에서 확인합니다.';
 const berthCell=cell(result.berth,berth);
 if(compact)return utilizationCell+berthCell;
 return utilizationCell+cell(result.crane,crane)+berthCell;
}

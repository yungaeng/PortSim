export function compareOperation({baselineHours, baselineCount, baselineCranes, automatedSeconds, automatedCount, automatedCranes=3}) {
  const values = [baselineHours, baselineCount, baselineCranes, automatedSeconds, automatedCount, automatedCranes].map(Number);
  if (values.some(v => !Number.isFinite(v) || v <= 0)) return {ok:false, reason:'양수인 작업 시간·컨테이너 수·STS 대수를 입력하세요.'};
  const [hours, count, cranes, seconds, autoCount, autoCranes] = values;
  if (![count, cranes, autoCount, autoCranes].every(Number.isInteger)) return {ok:false, reason:'컨테이너 수와 STS 대수는 정수여야 합니다.'};
  if(count !== autoCount) return {ok:false, reason:`같은 물량끼리 비교해야 합니다. 자동화 대상은 ${autoCount}개입니다.`};
  if(cranes !== autoCranes) return {ok:false, reason:`STS 대수가 다릅니다. 자동화 대상은 ${autoCranes}대입니다.`};
  const baselineSeconds=hours*3600;
  return {ok:true, percent:(baselineSeconds-seconds)/baselineSeconds*100, savedSeconds:baselineSeconds-seconds, baselineSeconds, automatedSeconds:seconds};
}
export function vesselDuration(vessel, boundary) {
  const end = boundary === 'yard' ? vessel?.placed_at : vessel?.unloaded_at;
  const start = vessel?.started_at;
  const done = boundary === 'yard' ? vessel?.placed : vessel?.unloaded;
  if(!vessel || !(vessel.initial_count>0) || done !== vessel.initial_count || !Number.isFinite(start) || start<0 || !Number.isFinite(end) || end<start) return null;
  return end-start;
}
export function formatDuration(seconds) {
  if (!Number.isFinite(seconds) || seconds<0) return '—';
  const s=Math.floor(seconds);
  return `${String(Math.floor(s/3600)).padStart(2,'0')}:${String(Math.floor(s%3600/60)).padStart(2,'0')}:${String(s%60).padStart(2,'0')}`;
}

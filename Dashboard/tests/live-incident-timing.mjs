// Explicit integration runner for a disposable live terminal instance.
import assert from 'node:assert/strict';
import {writeFile} from 'node:fs/promises';
import {createIncidentBaseline,compareIncidentTiming} from '../public/incident-timing.mjs';
const base=process.argv[2]||'http://127.0.0.1:4184',started=Date.now(),deadline=started+600000;
let run,scenario,reference,initialPositions,otherMovement=false;
while(Date.now()<deadline){
 const {state:s,live}=await(await fetch(base+'/api/state')).json();
 if(!live||Date.parse(s?.generated_at||'')<started||s?.incident_timing?.schema_version!==1){await new Promise(r=>setTimeout(r,200));continue;}
 if(s.fault)throw Error(s.fault);
 if(!scenario){
  if((s.incident_timing.cargo||[]).filter(c=>c.completed_at>=0).length<5){await new Promise(r=>setTimeout(r,200));continue;}
  const vehicle=s.actors.find(a=>a.type==='agv'&&a.stage===3&&a.on_deck_count>0&&a.speed_mps>.1);
  if(!vehicle){await new Promise(r=>setTimeout(r,200));continue;}
  reference=createIncidentBaseline(s);run=s.run_id;
  assert.equal(compareIncidentTiming(s,reference).status,'same_run');
  initialPositions=new Map(s.actors.filter(a=>a.type==='agv'&&a.id!==vehicle.id).map(a=>[a.id,JSON.stringify(a.position_m)]));
  const response=await fetch(base+'/api/equipment-command',{method:'POST',headers:{Origin:base,'Content-Type':'application/json'},body:JSON.stringify({run_id:run,control_session:s.control_session,actor_id:'scenario',action:'scenario_start',name:'시간 측정: 적재 AGV FMS 단절과 복구',steps:[{actor_id:vehicle.id,action:'fms_disconnect',trigger:'loaded'}]})});
  const result=await response.json();if(!response.ok)throw Error(result.message);scenario=result.id;console.log('QUEUED',scenario,vehicle.id);
 }
 if(s.run_id!==run)throw Error('Live run changed');
 otherMovement||=s.actors.some(a=>initialPositions.has(a.id)&&JSON.stringify(a.position_m)!==initialPositions.get(a.id));
 const record=s.incident_scenarios.find(r=>r.id===scenario),step=record?.steps[0];
 if(step&&['rejected','timeout','not_exercised'].includes(step.state))throw Error(JSON.stringify(step));
 if(step?.state==='verified'&&step.work_completed_at>=0){
  assert.equal(step.response_observed,true);assert.equal(otherMovement,true);
  assert.equal(step.recovery_seconds,60);
  assert(step.measurement_applied_at>=0&&step.fault_cleared_at>=step.measurement_applied_at+59.9);
  assert(step.work_resumed_at>=step.measurement_applied_at&&step.work_completed_at>=step.work_resumed_at);
  assert(step.affected_actors.length>=1&&step.affected_cargo.length>=1);
  assert.equal(s.incident_timing.comparison_key,reference.comparison_key);
  assert.throws(()=>createIncidentBaseline(s),/정상/);
  const output={run,scenario,step,otherMovement,baseline_prefix:reference,clock_seconds:s.incident_timing.clock_seconds};
  await writeFile(new URL('../../PortSim/Saved/Tests/IncidentTiming_Live.json',import.meta.url),JSON.stringify(output,null,2));
  console.log('PASS: actual fault interval, observed work resumption, final yard completion, affected scope, stable initial key and dirty-reference rejection');process.exit(0);
 }
 await new Promise(r=>setTimeout(r,200));
}
throw Error('Live incident timing integration timed out');

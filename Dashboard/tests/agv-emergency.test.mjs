import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtemp,mkdir,writeFile,readFile,readdir,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {createDashboardServer} from '../server.mjs';
import {emergencyControls,emergencyDetails,renderEmergencyReport} from '../public/agv-emergency-view.mjs';
import {impactModel} from '../public/control-view.mjs';

test('live controls require supported telemetry and moving cargo; unsafe outcomes cannot be released',()=>{
 const a={id:'agv',type:'agv',stage:3,speed_mps:6,emergency_test:{active:false,outcome:'idle'}};
 assert.match(emergencyControls(a,false),/data-command="test_wet_worker" >/);
 assert.match(emergencyControls({...a,emergency_test:null},false),/data-command="test_wet_worker" disabled/);
 for(const b of [{...a,speed_mps:0},{...a,fms_hold:true},{...a,controlled_stop:true},{...a,stage:1}])
  assert.match(emergencyControls(b,false),/data-command="test_wet_worker" disabled/);
 assert.match(emergencyControls(a,true),/data-command="test_wet_worker" disabled/);
 a.emergency_test={active:true,stopped:true,peak_cargo_shift_m:0};
 assert.match(emergencyControls(a,false),/data-command="clear_worker" >/);
 a.emergency_test.peak_cargo_shift_m=.3;
 assert.match(emergencyControls(a,false),/data-command="clear_worker" disabled/);
 const model=impactModel({actors:[a,{id:'queue',type:'agv',fms_blocking_agv:100}],recovery_events:[]});
 assert(model.related.has('agv'));a.emergency_test.active=false;
 assert(!impactModel({actors:[a]}).related.has('agv'));
});

test('reports distinguish expected hazards from safety, retain diagnostics and escape external input',()=>{
 assert.match(renderEmergencyReport(null),/보고서가 없습니다/);
 const report={valid:true,generated_at:'now',cases:[{valid:true,input:{id:'<script>',label:'<img>'},cargo_secured:false,
  outcome:'cargo_shift_limit',braking_distance_m:.5,minimum_clearance_m:5,peak_cargo_shift_m:.301,
  samples:[{time_s:1,speed_mps:5,gap_m:5,detected:true,brake:true,cargo_shift_m:.301}]}]};
 const html=renderEmergencyReport(report);
 assert.match(html,/화물 이동 한계 초과/);assert.match(html,/안전 정지를 의미하지 않습니다/);
 assert.match(html,/&lt;img&gt;/);assert.doesNotMatch(html,/<img>|<script>|undefined|NaN|Infinity/);
 assert.match(emergencyDetails({emergency_test:{outcome:'physical_braking',active:true,cargo_secured:true}}),/타이어 물리 급제동 중/);
 assert.doesNotMatch(emergencyDetails({emergency_test:{outcome:'worker_contact'}}),/NaN|undefined/);
});

test('HTTP publishes only whitelisted emergency commands and serves the separate integrated report',async t=>{
 const repo=await mkdtemp(path.join(tmpdir(),'portsim-emergency-')),dir=path.join(repo,'PortSim/Saved/Dashboard');
 await mkdir(dir,{recursive:true});await mkdir(path.join(repo,'PortSim/Saved/Tests'),{recursive:true});
 const state={schema_version:1,run_id:'test',control_session:'a'.repeat(32),generated_at:new Date().toISOString(),actors:[{id:'agv',type:'agv'},{id:'sts',type:'sts'}]};
 await writeFile(path.join(dir,'state.json'),JSON.stringify(state));
 const server=createDashboardServer({repo});await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
 t.after(async()=>{server.closeAllConnections();await new Promise(resolve=>server.close(resolve));
  if(!path.resolve(repo).startsWith(path.join(tmpdir(),'portsim-emergency-')))throw Error('Unexpected fixture path');await rm(repo,{recursive:true,force:true});});
 const base=`http://127.0.0.1:${server.address().port}`;
 const post=(actor_id,action)=>fetch(base+'/api/equipment-command',{method:'POST',headers:{Origin:base,'Content-Type':'application/json'},
  body:JSON.stringify({run_id:state.run_id,control_session:state.control_session,actor_id,action,brake_demand:999})});
 assert.equal((await post('sts','test_wet_worker')).status,400);
 assert.equal((await post('agv','set_any_parameter')).status,400);
 assert.equal((await post('agv','test_wet_worker')).status,202);
 const [file]=await readdir(path.join(dir,'commands'));const command=JSON.parse(await readFile(path.join(dir,'commands',file),'utf8'));
 assert.equal(command.action,'test_wet_worker');assert.equal(command.brake_demand,undefined);
 assert.deepEqual(await (await fetch(base+'/api/agv-emergencies')).json(),{report:null});
 await writeFile(path.join(repo,'PortSim/Saved/Tests/AGVEmergency_Report.json'),JSON.stringify({valid:true,cases:[]}));
 assert.equal((await (await fetch(base+'/api/agv-emergencies')).json()).report.valid,true);
 assert.equal((await fetch(base+'/agv-emergency-view.mjs')).status,200);
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtemp,mkdir,writeFile,readdir,readFile,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {createDashboardServer} from '../server.mjs';
import {cleanScenario} from '../public/incident-model.mjs';
import {IncidentUI,incidentRunStatus,renderIncidentRuns} from '../public/incident-view.mjs';
import {impactModel,impactOverlay} from '../public/control-view.mjs';
const actors=[{id:'v',type:'agv'},{id:'s',type:'sts'},{id:'r',type:'rmg'}];
test('weather precedes the composer and sensor recovery time is read-only',()=>{
 const ui=new IncidentUI({snapshot:()=>({}),send:()=>{},render:()=>{},notify:()=>{}});
 const html=ui.view({actors,incident_control_supported:true,incident_control_version:4},true);
 assert(html.indexOf('incident-weather-panel')<html.indexOf('incident-grid'));
 assert.match(html,/날씨 먼저 설정/);assert.match(html,/data-incident-weather="rain_mm_h"/);
 assert.doesNotMatch(html,/data-incident-field="(duration|timeout)"/);
 const sensor=renderIncidentRuns({actors:[{id:'v',name:'AGV',type:'agv',incident_sensor_failure:true,fault:'센서 오류',incident_sensor_phase:'rebooting'}],
  incident_scenarios:[{id:'sensor',steps:[{actor_id:'v',action:'fail_reboot',state:'recovering',recovery_seconds:30,recovery_remaining_seconds:12}]}]},true);
 assert.match(sensor,/재부팅 중/);assert.match(sensor,/자동 복구 예정까지 12초/);assert.doesNotMatch(sensor,/data-fault-action="fail_reboot"/);
 for(const actor_id of ['s','r'])assert.equal(cleanScenario({steps:[{actor_id,action:'equipment_failure'}]},{actors}).steps[0].action,'equipment_failure');
});
test('additional operational incidents and release/handover conditions are type checked',()=>{
 for(const action of ['localization_loss','fms_delay','fms_disconnect','road_block'])assert.equal(cleanScenario({steps:[{actor_id:'v',action,trigger:'moving'}]},{actors}).steps[0].action,action);
 for(const action of ['fail_release','fail_hoist','agv_unavailable'])assert.equal(cleanScenario({steps:[{actor_id:'s',action,trigger:action==='fail_release'?'release':'handover'}]},{actors}).steps[0].action,action);
 for(const action of ['slot_occupied','height_mismatch','fail_release'])assert.equal(cleanScenario({steps:[{actor_id:'r',action,trigger:'placing'}]},{actors}).steps[0].action,action);
 assert.throws(()=>cleanScenario({steps:[{actor_id:'s',action:'slot_occupied'}]},{actors}));
 assert.throws(()=>cleanScenario({steps:[{actor_id:'v',action:'fail_release',trigger:'release'}]},{actors}));
 const html=renderIncidentRuns({actors,incident_road_closures:[{min_m:[0,0,0],max_m:[1,1,1]}],incident_scenarios:[{id:'r',name:'정상 운송 중 돌발상황',steps:[{actor_id:'v',action:'fms_disconnect',state:'active',message:'명령 수신 중단'}]}]},true);
 assert.match(html,/FMS 통신 단절/);assert.match(html,/명령 수신 중단/);assert.doesNotMatch(html,/data-fault-action="fms_disconnect"/);
});
test('drive-failure control margin, route highlights and current hold are distinct from history',()=>{
 const fail={actor_id:'v',action:'fail',trigger:'moving'};
 const clean=radius=>cleanScenario({steps:[{...fail,control_radius_m:radius}]},{actors}).steps[0];
 assert.equal(clean(undefined).control_radius_m,1);
 assert.equal(clean(2.5).control_radius_m,2.5);
 for(const radius of [0,21,null,'1',Infinity])assert.throws(()=>clean(radius));
 const zone={kind:'agv_drive_failure',cause_actor:'v',control_radius_m:2.5,polygon_m:[[0,0],[0,3],[2,3],[2,0]]};
 const detouring={id:'w',name:'AGV-02',type:'agv',position_m:[5,0],incident_road_cause:'v',incident_detour_active:true};
 const state={actors:[...actors,detouring],incident_road_closures:[zone],recovery_events:[{kind:'road_rerouted',cause_actor:'v',to_actor:'w',sequence:1}]};
 assert.match(impactOverlay(state,null,x=>x,y=>y,null),/<polygon class="incident-road-closure"/);
 assert.match(renderIncidentRuns(state,true,false),/우회·대체 1대/);
 assert(impactModel(state).related.has('w'));
 detouring.incident_detour_active=false;
 assert(!impactModel(state).related.has('w'));
 assert.equal(impactModel(state).events.length,1);
});
test('combined scenarios validate target types, conditions, ranges and unique fault ownership',()=>{
 const step={actor_id:'v',action:'worker_enter',trigger:'loaded',worker_distance_m:20,delay:0,duration:30};
 const valid={steps:[{actor_id:'environment',action:'set_environment',weather:{rain_mm_h:25}},step,{actor_id:'r',action:'fail_gantry',delay:30,duration:60}]};
 assert.equal(cleanScenario(valid,{actors}).steps.length,3);
 for(const change of [{actor_id:'missing'},{actor_id:'s'},{worker_distance_m:Infinity},{delay:-1},{trigger:'placing'}])assert.throws(()=>cleanScenario({steps:[{...step,...change}]},{actors}));
 assert.throws(()=>cleanScenario({steps:[step,step]},{actors}),/同|동일/);
 assert.throws(()=>cleanScenario({steps:[{actor_id:'environment',action:'set_environment',weather:{rain_mm_h:101}}]},{actors}));
 assert.throws(()=>cleanScenario({steps:[{actor_id:'s',action:'fail_reboot'},{actor_id:'s',action:'fail_sensor_repair'}]},{actors}));
 assert.throws(()=>cleanScenario({steps:[{actor_id:'s',action:'equipment_failure'},{actor_id:'s',action:'restart_controller'}]},{actors}));
});
test('repaired and unobserved scenarios cannot appear as verified; history remains visible',()=>{
 assert.equal(incidentRunStatus({steps:[{state:'awaiting_completion'}]}),'실행 중');
 assert.equal(incidentRunStatus({steps:[{state:'not_exercised'}]}),'확인 필요');
 assert.equal(incidentRunStatus({steps:[{state:'verified'}]}),'검증 통과');
 const state={incident_scenarios:[{id:'x',name:'<unsafe>',steps:[{actor_id:'v',action:'fail',state:'awaiting_completion',message:'restored <tag>'}]}],actors};
 const html=renderIncidentRuns(state,true);assert.match(html,/&lt;unsafe&gt;/);assert.match(html,/restored &lt;tag&gt;/);assert.match(html,/복구 후 작업 완료 대기/);assert.doesNotMatch(html,/실시간 물리|슬립|토크/);
});
test('HTTP strips old recovery settings and reserves manual clearance for field controls',async t=>{
 const repo=await mkdtemp(path.join(tmpdir(),'portsim-incidents-')),directory=path.join(repo,'PortSim/Saved/Dashboard');await mkdir(directory,{recursive:true});
 const state={schema_version:1,run_id:'run',control_session:'a'.repeat(32),generated_at:new Date().toISOString(),actors,incident_control_supported:true,incident_control_version:4};
 await writeFile(path.join(directory,'state.json'),JSON.stringify(state));const server=createDashboardServer({repo});await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const url=`http://127.0.0.1:${server.address().port}`;t.after(async()=>{server.closeAllConnections();await new Promise(r=>server.close(r));await rm(repo,{recursive:true,force:true});});
 const post=body=>fetch(url+'/api/equipment-command',{method:'POST',headers:{Origin:url,'Content-Type':'application/json'},body:JSON.stringify({run_id:state.run_id,control_session:state.control_session,actor_id:'scenario',...body})});
 assert.equal((await post({action:'scenario_start',steps:[{actor_id:'r',action:'worker_enter'}]})).status,400);
 const response=await post({action:'scenario_start',name:'Combination',steps:[{actor_id:'environment',action:'set_environment',weather:{rain_mm_h:25}},{actor_id:'v',action:'fail_reboot',delay:10,duration:0},{actor_id:'r',action:'fail_placement',trigger:'placing',duration:20}]});assert.equal(response.status,202);
 const files=await readdir(path.join(directory,'commands')),command=JSON.parse(await readFile(path.join(directory,'commands',files[0]),'utf8'));assert.equal(command.steps.length,3);assert.equal(command.steps[1].delay,10);
 assert(command.steps.every(step=>!('duration' in step)&&!('timeout' in step)));
 assert.equal((await post({action:'incident_clear',actor_id:'v',fault_action:'fail_reboot'})).status,409);
 assert.equal((await post({action:'incident_clear',actor_id:'v',fault_action:'road_block'})).status,202);
 assert.equal((await post({action:'incident_clear',actor_id:'v',fault_action:'fail_gantry'})).status,400);
});

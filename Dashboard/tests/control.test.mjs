import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtemp,mkdir,writeFile,readFile,readdir,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {createDashboardServer} from '../server.mjs';
import {controlPanel,impactModel,impactOverlay,mapPosition,impactFeed} from '../public/control-view.mjs';

async function fixture(t,stale=false){
 const repo=await mkdtemp(path.join(tmpdir(),'portsim-control-')),directory=path.join(repo,'PortSim/Saved/Dashboard');
 await mkdir(directory,{recursive:true});
 const state={schema_version:1,run_id:'test-run',control_session:'a'.repeat(32),generated_at:new Date(Date.now()-(stale?60000:0)).toISOString(),actors:[{id:'sts',type:'sts'},{id:'rmg',type:'rmg'},{id:'agv',type:'agv'},{id:'cargo',type:'container'}]};
 await writeFile(path.join(directory,'state.json'),JSON.stringify(state));
 const server=createDashboardServer({repo});await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
 const url=`http://127.0.0.1:${server.address().port}`;
 t.after(async()=>{server.closeAllConnections();await new Promise(resolve=>server.close(resolve));if(!path.resolve(repo).startsWith(path.join(tmpdir(),'portsim-control-')))throw Error('Unexpected fixture path');await rm(repo,{recursive:true,force:true});});
 const post=(body={},origin=url,contentType='application/json')=>fetch(url+'/api/equipment-command',{method:'POST',headers:{Origin:origin,'Content-Type':contentType},body:JSON.stringify({run_id:state.run_id,control_session:state.control_session,actor_id:'sts',action:'fail',...body})});
 return {state,directory,post,url};
}
test('HTTP publishes only allowed commands atomically, serializes duplicate submissions',async t=>{
 const f=await fixture(t);
 assert.equal((await f.post({},'http://untrusted.invalid')).status,403);
 assert.equal((await f.post({},f.url,'text/plain')).status,415);
 assert.equal((await f.post({actor_id:'cargo'})).status,400);
 assert.equal((await f.post({run_id:'old-run'})).status,409);
 assert.equal((await f.post({action:'console'})).status,400);
 const responses=await Promise.all([f.post({ignored:'not published'}),f.post()]);
 assert.deepEqual(responses.map(r=>r.status).sort(),[202,409]);
 const files=await readdir(path.join(f.directory,'commands'));assert.equal(files.length,1);assert.match(files[0],/^[a-f0-9]{32}\.json$/);
 const command=JSON.parse(await readFile(path.join(f.directory,'commands',files[0]),'utf8'));
 assert.equal(command.actor_id,'sts');assert.equal(command.action,'fail');assert.equal(command.ignored,undefined);
 assert.equal((await f.post({actor_id:'rmg'})).status,202);assert.equal((await f.post({actor_id:'agv'})).status,202);
});
test('recorded snapshots cannot receive queued controls',async t=>{
 const f=await fixture(t,true);assert.equal((await f.post()).status,409);
});

test('repair modes are whitelisted per equipment and appear with live countdown and road usage',async t=>{
 const f=await fixture(t);
 assert.equal((await f.post({actor_id:'agv',action:'fail_gantry'})).status,400);
 assert.equal((await f.post({action:'fail_reboot'})).status,202);
 assert.equal((await f.post({actor_id:'rmg',action:'fail_sensor_repair'})).status,202);
 const html=controlPanel({type:'sts',name:'STS',fault_scope:'equipment',sensor_repair_remaining_seconds:17},f.state,true,null);
 assert.match(html,/17초/);assert.match(html,/fail_gantry/);assert.match(html,/fail_sensor_repair/);
 assert.match(impactFeed({actors:[],central_road:{loaded_east_m:12,empty_west_m:24}},null,true),/적재 12m · 빈차 24m/);
});
test('controls require live engine, distinguish queue acknowledgement from applied state',()=>{
 const actor={id:'sts',type:'sts',name:'<STS>'},state={control_session:'a'.repeat(32)};
 assert.match(controlPanel(actor,state,false,null),/data-command="fail" disabled/);
 assert.match(controlPanel(actor,state,true,{id:'pending'},'엔진 응답 대기'),/data-command="fail" disabled/);
 assert.match(controlPanel({...actor,fault_scope:'equipment'},state,true,null),/data-command="repair" >/);
 assert.match(controlPanel(actor,state,true,null),/&lt;STS&gt;/);
 assert.match(controlPanel(actor,{...state,fault:'safety'},true,null),/전체 안전 정지/);
});
test('overview follows actual gantry position and traces real affected routes and indirect queues',()=>{
 const state={actors:[{id:'rmg',type:'rmg',name:'RMG',position_m:[1,2,0],gantry_position_m:[30,40,0]},
  {id:'agv',type:'agv',vehicle_id:100,rmg:'rmg',cargo_actor:'cargo',position_m:[2,3,0],route:[{position_m:[5,6,0]}]},
  {id:'cargo',type:'container',state:'AGV'},
  {id:'queue',type:'agv',vehicle_id:101,fms_blocking_agv:100,position_m:[8,9,0]},
  {id:'unrelated',type:'agv',position_m:[20,30,0]}],recovery_events:[{sequence:2,kind:'yard_reassigned',cause_actor:'failed',from_actor:'failed',to_actor:'rmg',cargo:'cargo',from_position_m:[1,2,0],to_position_m:[3,4,0],simulation_seconds:50}]};
 assert.deepEqual(mapPosition(state.actors[0]),[30,40,0]);
 const focus={cause:'failed',after:1},model=impactModel(state,focus);
 assert(model.related.has('queue'));assert(!model.related.has('unrelated'));
 const svg=impactOverlay(state,focus,v=>v,v=>v,null);
 assert.match(svg,/plan-connection/);assert.match(svg,/impact-route/);assert.doesNotMatch(svg,/motion-trail/);
 assert.match(impactFeed(state,focus,true),/야드 목적지 재배정/);
 assert.equal(impactModel(state,{cause:'failed',after:3}).events.length,0);
});

test('completed reassigned work clears map impacts without deleting the recovery history',()=>{
 const state={actors:[
  {id:'failed',type:'sts',position_m:[0,0,0]},
  {id:'sts',type:'sts',name:'STS-01',state:'working',position_m:[1,1,0]},
  {id:'rmg',type:'rmg',name:'RMG-01',state:'working',position_m:[2,2,0]},
  {id:'cargo',type:'container',state:'Ship',assigned_sts:1,planned_rmg:1,reassignments:2},
  {id:'agv',type:'agv',vehicle_id:100,cargo_actor:'cargo',sts:'sts',rmg:'rmg',position_m:[3,3,0],route:[{position_m:[4,4,0]}]},
  {id:'queue',type:'agv',fms_blocking_agv:100,position_m:[5,5,0]},
  {id:'other',type:'agv',sts:'sts',rmg:'rmg',cargo_actor:'unrelated',position_m:[6,6,0]}
 ],recovery_events:[
  {sequence:1,kind:'fault',cause_actor:'failed',from_actor:'failed',to_actor:'failed',simulation_seconds:1},
  {sequence:2,kind:'sts_reassigned',cause_actor:'failed',from_actor:'failed',to_actor:'sts',cargo:'cargo',simulation_seconds:2},
  {sequence:3,kind:'yard_reassigned',cause_actor:'failed',from_actor:'failed',to_actor:'rmg',cargo:'cargo',simulation_seconds:3},
  {sequence:4,kind:'repair',cause_actor:'failed',from_actor:'failed',to_actor:'failed',simulation_seconds:4}
 ]};
 const focus={cause:'failed',after:0},history=JSON.stringify(state.recovery_events),cargo=state.actors[3];
 assert.deepEqual([...impactModel(state,focus).related].sort(),['agv','queue','rmg','sts']);
 cargo.state='STS';assert(impactModel(state,focus).related.has('sts'));
 cargo.state='AGV';assert(!impactModel(state,focus).related.has('sts'));
 assert(impactModel(state,focus).related.has('rmg'));
 cargo.state='RMG';assert(impactModel(state,focus).related.has('rmg'));
 cargo.state='Yard';
 assert.equal(impactModel(state,focus).related.size,0);
 assert.doesNotMatch(impactOverlay(state,focus,v=>v,v=>v,null),/impact-halo|plan-connection|impact-route/);
 const feed=impactFeed(state,focus,true);
 assert.match(feed,/STS 작업 재배정/);assert.match(feed,/야드 목적지 재배정/);assert.match(feed,/영향 대상 0대/);
 assert.equal(JSON.stringify(state.recovery_events),history);assert.equal(cargo.reassignments,2);
 // A currently failed vehicle still affects its queue, independently of finished plans.
 state.actors[4].fault_scope='equipment';state.actors[4].fault='drive failure';
 assert.deepEqual([...impactModel(state,focus).related].sort(),['agv','queue']);
 assert.match(impactOverlay(state,focus,v=>v,v=>v,null),/fault-halo/);
});

test('a recipient stays highlighted until its last pending plan finishes; superseded plans are history only',()=>{
 const state={actors:[
  {id:'rmg1',type:'rmg',name:'RMG-01'}, {id:'rmg2',type:'rmg',name:'RMG-02'},
  {id:'cargo1',type:'container',state:'AGV',planned_rmg:1},
  {id:'cargo2',type:'container',state:'RMG',planned_rmg:1}
 ],recovery_events:[
  {sequence:1,kind:'yard_reassigned',cause_actor:'fault1',to_actor:'rmg1',cargo:'cargo1'},
  {sequence:2,kind:'yard_reassigned',cause_actor:'fault1',to_actor:'rmg1',cargo:'cargo2'}
 ]};
 state.actors[2].state='Yard';assert(impactModel(state).related.has('rmg1'));
 state.actors[3].state='Yard';assert.equal(impactModel(state).related.size,0);
 state.actors[2].state='Ship';state.actors[2].planned_rmg=2;
 state.recovery_events.push({sequence:3,kind:'yard_reassigned',cause_actor:'fault2',to_actor:'rmg2',cargo:'cargo1'});
 assert.deepEqual([...impactModel(state).related],['rmg2']);
 assert.equal(impactModel(state,{cause:'fault1',after:0}).related.size,0);
 assert.equal(impactModel(state,{cause:'fault1',after:0}).events.length,2);
 // A later assignment outside the retained event window must not revive the old recipient.
 state.actors[2].planned_rmg=3;assert.equal(impactModel(state).related.size,0);
});

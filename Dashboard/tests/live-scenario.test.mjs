import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtemp,mkdir,writeFile,readFile,readdir,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {createDashboardServer} from '../server.mjs';
import {environmentLive,recordLiveSample,livePhysicsResults,weatherHoldStatus} from '../public/scenario-view.mjs';

test('weather hold reports engine recovery without inferring resume or hiding manual pause',()=>{
 assert.match(weatherHoldStatus({}),/미수신/);
 const state={manual_paused:true,weather_hold:{active:true,reason:'wind <limit>',recovery_elapsed_seconds:4,recovery_required_seconds:10}};
 assert.match(weatherHoldStatus(state),/wind &lt;limit&gt;/);
 assert.match(weatherHoldStatus(state),/4 \/ 10 시뮬레이션 초/);
 assert.match(weatherHoldStatus(state),/수동 일시정지 유지/);
 state.weather_hold.active=false;
 assert.match(weatherHoldStatus(state),/기상 대기 해제.*수동 일시정지 유지/);
 assert.doesNotMatch(weatherHoldStatus(state),/회복 확인|실행 중/);
});

test('live page renders current values and preserves sample identity across pauses, resets and vehicle selection',()=>{
 const s={run_id:'r',simulation_seconds:10,environment:{wind_mps:3.5,rain_mm_h:25,water_film_mm:.123456,tire_mu_static:.4}};
 const a={id:'a',name:'<AGV>',speed_mps:4,agv_physics:{max_slip_ratio:.12,load_transfer_ratio:.2,wheels:[]}};
 let h=recordLiveSample([],s,a);assert.equal(h.length,1);assert.equal(h[0].slip,12);
 assert.equal(recordLiveSample(h,s,a).length,1);
 for(let i=11;i<211;i++)h=recordLiveSample(h,{...s,simulation_seconds:i},a);
 assert.equal(h.length,180);
 assert.equal(recordLiveSample(h,s,{...a,id:'b'}).length,1);
 assert.equal(recordLiveSample(h,{...s,simulation_seconds:1},a).length,1);
 assert.equal(recordLiveSample(h,{...s,run_id:'new'},a).length,1);
 const html=livePhysicsResults(s,true,a,h);
 assert.match(html,/&lt;AGV&gt;/);assert.match(html,/LIVE/);assert.doesNotMatch(html,/과거 독립 시험|시험 보고서/);
 assert.match(livePhysicsResults(s,false,a,h),/엔진 연결이 없어/);
 assert.doesNotThrow(()=>livePhysicsResults(s,false,{id:'legacy'},[]));
 assert.match(environmentLive(s,true),/관측 전용/);
 assert.match(environmentLive(s,true),/0.123/);
});

test('web cannot modify weather even if a client invents the old capability',async t=>{
 const repo=await mkdtemp(path.join(tmpdir(),'portsim-scenario-')),dir=path.join(repo,'PortSim/Saved/Dashboard');
 await mkdir(dir,{recursive:true});
 const state={schema_version:1,run_id:'live',control_session:'a'.repeat(32),generated_at:new Date().toISOString(),actors:[{id:'agv',type:'agv'}],environment_command_supported:true};
 await writeFile(path.join(dir,'state.json'),JSON.stringify(state));
 const server=createDashboardServer({repo});await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const url=`http://127.0.0.1:${server.address().port}`;
 t.after(async()=>{server.closeAllConnections();await new Promise(r=>server.close(r));if(!path.resolve(repo).startsWith(path.join(tmpdir(),'portsim-scenario-')))throw Error('Unexpected path');await rm(repo,{recursive:true,force:true});});
 for(const actor_id of ['environment','agv'])for(const action of ['set_environment','set_surface']){
  const response=await fetch(url+'/api/equipment-command',{method:'POST',headers:{Origin:url,'Content-Type':'application/json'},body:JSON.stringify({run_id:state.run_id,control_session:state.control_session,actor_id,action,rain_mm_h:50})});
  assert.equal(response.status,400);
 }
});

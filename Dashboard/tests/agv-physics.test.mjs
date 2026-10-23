import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtemp,mkdir,writeFile,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {createDashboardServer} from '../server.mjs';
import {renderAGVPhysics,renderAGVExperiments,renderAGVSensors} from '../public/agv-physics-view.mjs';

test('live tire details distinguish measured braking, constraints and synthetic IMU',()=>{
 const a={agv_physics:{mass_kg:70000,combined_cog_m:[0,.3,2],cargo_cog_m:[0,.7,3.5],left_load_n:400000,right_load_n:286465,
  max_slip_ratio:.12,peak_slip_ratio:.3,load_transfer_ratio:-.16,last_brake_valid:false,legacy_stop_constraint:true,
  settings:{stability_control:true},wheels:[{name:'front_left',rpm:60,normal_n:100000,applied_torque_nm:10000,slip_ratio:.12,sliding:true}]}};
 let html=renderAGVPhysics(a);assert.match(html,/유효한 완료 기록 없음/);assert.match(html,/기존 강제 정지/);assert.match(html,/합성값/);assert.match(html,/전좌 · 슬립/);
 a.agv_physics.last_brake_valid=true;a.agv_physics.last_braking_seconds=2;a.agv_physics.last_braking_distance_m=5;
 assert.match(renderAGVPhysics(a),/2 s \/ 5 m/);
 a.agv_physics.overturn_threshold=true;assert.match(renderAGVPhysics(a),/해석 범위 이탈/);
 assert.doesNotMatch(renderAGVSensors(a),/NaN|undefined|Infinity/);
});
test('bench results show failed/contact-limited runs and escape external text',()=>{
 assert.match(renderAGVExperiments(null),/결과가 아직 없습니다/);
 const report={valid:false,cases:[{valid:true,input:{id:'<script>',kind:'turn'},contact_threshold:true,samples:[]},{valid:false,input:{id:'bad'},samples:[]}]};
 const html=renderAGVExperiments(report);assert.match(html,/접촉 한계 도달/);assert.match(html,/실패/);assert.match(html,/&lt;script&gt;/);
 assert.doesNotMatch(html,/<script>|NaN|Infinity|undefined/);
});
test('report endpoint handles missing, valid and malformed results without inventing data',async t=>{
 const repo=await mkdtemp(path.join(tmpdir(),'portsim-agv-physics-')),dir=path.join(repo,'PortSim/Saved/Tests');
 await mkdir(dir,{recursive:true});const server=createDashboardServer({repo});await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
 t.after(async()=>{server.closeAllConnections();await new Promise(resolve=>server.close(resolve));
  if(!path.resolve(repo).startsWith(path.join(tmpdir(),'portsim-agv-physics-')))throw Error('Unexpected test directory');await rm(repo,{recursive:true,force:true});});
 const base=`http://127.0.0.1:${server.address().port}`;
 assert.deepEqual(await (await fetch(base+'/api/agv-experiments')).json(),{report:null});
 await writeFile(path.join(dir,'AGVDynamics_Report.json'),JSON.stringify({valid:true,cases:[]}));
 assert.equal((await (await fetch(base+'/api/agv-experiments')).json()).report.valid,true);
 await writeFile(path.join(dir,'AGVDynamics_Report.json'),'{incomplete');
 assert.equal((await fetch(base+'/api/agv-experiments')).status,503);
 assert.equal((await fetch(base+'/agv-physics-view.mjs')).status,200);
});

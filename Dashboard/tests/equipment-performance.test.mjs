import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createDashboardServer} from '../server.mjs';
import {equipmentPerformance,renderEquipmentPerformance} from '../public/equipment-performance.mjs';
const actor=(type='sts')=>({type,performance:{schema_version:1,observed_seconds:3600,working_seconds:1800,handled_containers:30,berth:{berth_id:1,handled_containers:80,operation_seconds:7200}}});
test('utilization and crane productivity use gross observation hours',()=>{
 assert.deepEqual(equipmentPerformance(actor()),{utilization:50,crane:30,berth:40});
 const a=actor();a.performance.observed_seconds=7200;assert.deepEqual(equipmentPerformance(a),{utilization:25,crane:15,berth:40});
});
test('only applicable equipment and berth rows receive each metric',()=>{
 assert.deepEqual(equipmentPerformance(actor('agv')),{utilization:50,crane:null,berth:null});
 assert.deepEqual(equipmentPerformance(actor('rmg')),{utilization:50,crane:30,berth:null});
 assert.deepEqual(equipmentPerformance(actor('rs')),{utilization:50,crane:null,berth:null});
 assert.deepEqual(equipmentPerformance(actor('vessel')),{utilization:null,crane:null,berth:40});
 assert.deepEqual(equipmentPerformance(actor('sts')),{utilization:50,crane:30,berth:null});
 assert.deepEqual(equipmentPerformance(actor('container')),{utilization:null,crane:null,berth:null});
});
test('missing and zero duration stay unknown, while an observed idle crane has valid zeros',()=>{
 assert.deepEqual(equipmentPerformance({type:'sts'}),{utilization:null,crane:null,berth:null});
 const a=actor();a.performance.observed_seconds=0;a.performance.berth.operation_seconds=0;
 assert.deepEqual(equipmentPerformance(a),{utilization:null,crane:null,berth:null});
 a.performance.observed_seconds=3600;a.performance.working_seconds=0;a.performance.handled_containers=0;
 assert.deepEqual(equipmentPerformance(a),{utilization:0,crane:0,berth:null});
 a.performance.working_seconds=4000;assert.equal(equipmentPerformance(a).utilization,null);
});
test('rendered cells explain the denominators and escape engine labels',()=>{
 const a=actor();a.performance.berth.berth_id='<script>';
 const html=renderEquipmentPerformance(a),compact=renderEquipmentPerformance(a,{compact:true}),
  vesselCompact=renderEquipmentPerformance(actor('vessel'),{compact:true});
 assert.equal((html.match(/<td /g)||[]).length,3);assert.equal((compact.match(/<td /g)||[]).length,2);
 assert.match(vesselCompact,/40\.0/);
 assert.match(html,/50\.0/);assert.match(html,/30\.0/);assert.match(html,/>—<\/td>/);
 assert.doesNotMatch(html,/선석 1:/);
 assert.match(html,/&lt;script&gt;/);assert.doesNotMatch(html,/<script>/);
});
test('both tables show utilization and vessel-only berth productivity; explorer adds crane productivity',async t=>{
 const app=await readFile(new URL('../public/app.mjs',import.meta.url),'utf8');
 assert.match(app,/작업 \/ 위치<\/th><th class="performance-column" title="실제 작업 시간 ÷ 관측 운용 시간 × 100">가동률 \(%\)<\/th>\$\{performanceColumns\?'<th class="performance-column"[\s\S]*?크레인 생산성[\s\S]*?<\/th>':''\}<th class="performance-column" title="선박별 선석 완료 컨테이너 수 ÷ 해당 선석 작업 시간">선석 생산성/);
 assert.match(app,/renderEquipmentPerformance\(a,\{compact:!performanceColumns\}\)/);
 assert.match(app,/columnCount=performanceColumns\?8:7/);assert.match(app,/colspan="\$\{columnCount\}"/);
 assert.match(app,/b\.dataset\.view==='equipment'&&view!=='equipment'\)filter='all'/);
 assert.match(app,/setInterval\(poll,1000\)/);
 const server=createDashboardServer();await new Promise(r=>server.listen(0,'127.0.0.1',r));
 t.after(()=>server.close());const response=await fetch(`http://127.0.0.1:${server.address().port}/equipment-performance.mjs`);
 assert.equal(response.status,200);assert.match(await response.text(),/export function equipmentPerformance/);
});

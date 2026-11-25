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
 assert.deepEqual(equipmentPerformance(actor('vessel')),{utilization:null,crane:null,berth:40});
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
 const html=renderEquipmentPerformance(a);assert.equal((html.match(/<td /g)||[]).length,3);
 assert.match(html,/50\.0/);assert.match(html,/30\.0/);assert.match(html,/40\.0/);
 assert.match(html,/&lt;script&gt;/);assert.doesNotMatch(html,/<script>/);
});
test('the explorer places three columns directly after task/location and serves its module',async t=>{
 const app=await readFile(new URL('../public/app.mjs',import.meta.url),'utf8');
 assert.match(app,/performanceColumns=view==='equipment'/);
 assert.match(app,/작업 \/ 위치<\/th>\$\{performanceColumns\?[\s\S]*?가동률 \(%\)[\s\S]*?크레인 생산성[\s\S]*?선석 생산성/);
 assert.match(app,/colspan="\$\{performanceColumns\?8:5\}"/);
 const server=createDashboardServer();await new Promise(r=>server.listen(0,'127.0.0.1',r));
 t.after(()=>server.close());const response=await fetch(`http://127.0.0.1:${server.address().port}/equipment-performance.mjs`);
 assert.equal(response.status,200);assert.match(await response.text(),/export function equipmentPerformance/);
});

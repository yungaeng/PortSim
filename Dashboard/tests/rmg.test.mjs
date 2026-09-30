import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

// Exercise rendering with an RMG snapshot before a live UE session is available.
// This is a render-function test, not a substitute for live transport validation.
const settings=JSON.parse(readFileSync(new URL('../../PortSim/Config/RMG_Simulation.json',import.meta.url),'utf8'));
const source=readFileSync(new URL('../public/app.mjs',import.meta.url),'utf8').replace(/^\uFEFF?import [^\n]*\n/,'').replace(/\ninit\(\);\s*$/,'');
function context(){
 const c=vm.createContext({document:{querySelectorAll:()=>[],addEventListener:()=>{}},localStorage:{getItem:()=>null},settingsFixture:settings});
 vm.runInContext(source,c);
 vm.runInContext(`state={actors:[{id:'rmg-1',type:'rmg',name:'RMG-01',stage:5,busy:true,observation:{active:true,valid:true,stack_profile_valid:false,crane_clear:true},pickup:{phase:'complete',corners:[]},applied_crane_profile:{reference:settingsFixture.reference,simulation_assumptions:settingsFixture},mounted_sensors:settingsFixture.sensor_mounts.map(m=>({...m,world_position_m:m.position_m,mount_position_m:m.position_m,forward:[0,0,-1],rays:[],reference:m}))}]};selected='rmg-1';reference={sensors:[{id:'S01',key:'sts-only',name:'STS_ONLY_SENTINEL'}]};`,c);
 return c;
}
test('RMG sensor page uses its own source, mounts and equipment selection',()=>{
 const c=context(),html=vm.runInContext('sensorsPage()',c);
 assert.match(html,/RMG 센서 배치와 탐지/);
 assert.match(html,/RMG-01/);
 assert.match(html,/전방 크레인 충돌 레이저/);
 assert.match(html,/Konecranes/);
 assert.doesNotMatch(html,/STS_ONLY_SENTINEL/);
});
test('RMG inspector shows stack rejection and independent settings',()=>{
 const c=context(),html=vm.runInContext('sensorDetails(current())+variableDetails(current())',c);
 assert.match(html,/미확인 \/ 불일치/);
 assert.match(html,/RMG 적용 사양/);
 assert.match(html,/40000/);
 assert.match(html,/센서 기반 픽업/);
 assert.doesNotMatch(html,/개별 센서 관측 모델 없음/);
});
test('RMG motor details never fall back to the STS drum or gearbox',()=>{
 const c=context();
 vm.runInContext("current().dynamics={wires:[],motor_count:2};settings={dynamics:{drum_radius_m:1.3,gear_ratio:24}};",c);
 const html=vm.runInContext('dynamicsDetails(current())',c);
 assert.match(html,/0\.6 m \/ 30:1/);
 assert.doesNotMatch(html,/1\.3 m \/ 24:1/);
});

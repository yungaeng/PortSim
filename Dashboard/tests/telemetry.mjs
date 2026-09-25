// Run after an actual unified-engine run. This checks real data, not a UI fixture.
import {readFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
const state=JSON.parse(await readFile(new URL('../../PortSim/Saved/Dashboard/state.json',import.meta.url),'utf8'));
assert.equal(state.schema_version,1);assert.equal(state.unified,true);
assert.equal(new Set(state.actors.map(a=>a.id)).size,state.actors.length);
const byType=t=>state.actors.filter(a=>a.type===t);
assert.equal(byType('sts').length,9);assert.equal(byType('agv').length,60);assert.equal(byType('rmg').length,46);assert.equal(byType('container').length,1584);
assert.equal(state.vessels.length,3);assert.equal(state.vessels.reduce((n,v)=>n+v.initial_count,0),1584);
for(const a of state.actors)assert(a.position_m.every(Number.isFinite));
for(const [i,a] of byType('sts').sort((a,b)=>a.name.localeCompare(b.name)).entries()){
 assert.equal(a.name,`STS-${String(i+1).padStart(2,'0')}`);
 assert.equal(a.mounted_sensors.length,14);
 assert.equal(a.components.filter(c=>c.name.startsWith('Sensor_')).length,25);
 for(const mount of a.mounted_sensors){
  for(const instance of mount.instances||[]){
   assert(instance.world_position_m.every(Number.isFinite));assert.equal(instance.rays.length,mount.scan?9:0);
   for(const ray of instance.rays)if(ray.distance_m!==null)assert(ray.distance_m>=mount.minimum-.001&&ray.distance_m<=mount.maximum+.001);
  }
  const component=a.components.find(c=>c.name===`Sensor_${mount.key}`);assert(component);
  for(let i=0;i<3;i++)assert(Math.abs(component.world_position_m[i]-mount.world_position_m[i])<.00001);
  if(mount.frame==='spreader'){
   const spreader=a.components.find(c=>c.name==='Spreader');assert(spreader);
   const distance=Math.hypot(...mount.world_position_m.map((v,i)=>v-spreader.world_position_m[i]));
   assert(Math.abs(distance-Math.hypot(...mount.mount_position_m))<.001,'mount metres must not inherit mesh scale');
  }
 }

 const boom=a.mounted_sensors.find(m=>m.key==='boom_collision_lidar');
 assert.equal(boom.instances.length,2);assert.equal(boom.fov_deg,190);assert.equal(boom.maximum,80);
 assert.equal(boom.reference.specification_status,'VERIFIED_PRODUCT');assert.equal(boom.reference.dgt_installation_confirmed,false);
 const encoder=a.mounted_sensors.find(m=>m.key==='trolley_encoder');assert.equal(encoder.maximum,1700);assert.equal(encoder.mount_position_m[1],.025);assert.equal(encoder.reference.reference_specs.selected_gap_tolerance_m,.01);assert(encoder.value>=0);assert(Math.abs(encoder.value/.0001-Math.round(encoder.value/.0001))<.001);
 assert.equal(a.dynamics.wires.length,4);
 for(const key of ['sway_deg','skew_deg','motor_torque_each_nm','motor_rpm','motor_power_w'])assert(Number.isFinite(a.dynamics[key]));
 for(const w of a.dynamics.wires){assert(w.tension_n>=0);assert(w.top_m.every(Number.isFinite));assert(w.bottom_m.every(Number.isFinite));}
 for(const m of a.mounted_sensors){assert(m.world_position_m.every(Number.isFinite));assert(m.minimum<m.maximum);assert.equal(m.rays.length,m.scan?9:0);for(const ray of m.rays)if(ray.distance_m!==null)assert(ray.distance_m>=m.minimum-.001&&ray.distance_m<=m.maximum+.001);}
 assert.equal(a.hoist_power_w,670000);assert(a.rated_payload_kg>65000);
 assert.equal(a.pickup.corners.length,4);
 for(const c of a.pickup.corners)assert(c.error_m.every(Number.isFinite));
 if(a.busy&&a.carrying&&a.stage>=3)assert.equal(a.pickup.verified,true,'Full hoist requires verified trial lift');
 assert.equal(a.observation.corner_loads_n.length,4);assert.equal(a.observation.locks.length,4);
 if(a.carrying&&a.observation.valid)assert(Math.abs(a.observation.corner_loads_n.reduce((x,y)=>x+y,0)/(9.80665+a.observation.hoist_acceleration_mps2)-a.payload_kg)<1);
}
for(const a of byType('agv')){
 if(a.sts)assert(byType('sts').some(s=>s.id===a.sts));
 if(a.stage>0&&!state.paused&&!state.fault)assert.equal(a.state,'working');
}
for(const a of byType('rmg'))assert.equal(a.observation,undefined);
console.log(`PASS real telemetry: ${state.actors.length} entities, ${state.initial_ship} cargo, 9 STS / 60 AGV / 46 RMG`);

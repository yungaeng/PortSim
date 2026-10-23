// Run alongside TestSite.ps1 only in the final validation phase.
import {writeFile} from 'node:fs/promises';
import {setTimeout as wait} from 'node:timers/promises';
const base='http://127.0.0.1:4173',started=Date.now();
const summary={commands:[],rejections:[],otherTrafficMoved:false};
let session,vehicle,lastTry=0,initialPositions;
const output=new URL('../../PortSim/Saved/Tests/AGVEmergency_WebIntegration.json',import.meta.url);
async function snapshot(){const r=await fetch(base+'/api/state');if(!r.ok)throw Error('State unavailable');return r.json();}
async function command(s,a,action){
 const r=await fetch(base+'/api/equipment-command',{method:'POST',headers:{Origin:base,'Content-Type':'application/json'},
  body:JSON.stringify({run_id:s.run_id,control_session:s.control_session,actor_id:a.id,action})});
 const result=await r.json();if(r.status!==202)throw Error(result.message);
 for(let j=0;j<40;j++){
  await wait(250);const {state:p}=await snapshot();
  if(p.control_session!==session)throw Error('Engine session changed');
  const ack=p.command_results?.find(c=>c.id===result.id);
  if(ack){summary.commands.push({action,actor:a.name,...ack});return {state:p,applied:ack.status==='applied',message:ack.message};}
 }
 throw Error('Native acknowledgement missing');
}
try{
 while(Date.now()-started<15*60*1000){
  const {state:s,live}=await snapshot();
  if(!live||Date.parse(s?.generated_at)<started){await wait(250);continue;}
  if(s.control_session!==session){if(vehicle)throw Error('Controlled run reset');session=s.control_session;}
  if(s.fault)throw Error(s.fault);
  if(!vehicle && Date.now()-lastTry>750){
   const a=s.actors.find(v=>v.type==='agv'&&v.stage===3&&v.on_deck_count===1&&v.cargo_items?.[0]?.container_length_ft===40&&
    v.speed_mps>=2&&!v.fms_hold&&!v.controlled_stop&&!v.emergency_test?.active&&Math.abs(v.agv_physics?.yaw_rate_rad_s||0)<.01&&v.route_remaining_m>60);
   if(a){
    lastTry=Date.now();const result=await command(s,a,'test_wet_worker');
    if(result.applied){
     vehicle=a.id;summary.vehicle=a.name;summary.run=s.run_id;
     initialPositions=new Map(s.actors.filter(v=>v.type==='agv'&&v.id!==a.id).map(v=>[v.id,v.position_m]));
     await writeFile(new URL('../../PortSim/Saved/Tests/AGVEmergency_RuntimeStarted.json',import.meta.url),JSON.stringify(result.state,null,2));
    }else summary.rejections.push(result.message);
   }
  }
  if(vehicle){
   const a=s.actors.find(v=>v.id===vehicle),e=a.emergency_test;
   summary.otherTrafficMoved ||= s.actors.some(v=>initialPositions.has(v.id)&&JSON.stringify(v.position_m)!==JSON.stringify(initialPositions.get(v.id)));
   if(e.finished){
    summary.result=e;
    if(!e.stopped||e.collision||e.invalidated||e.minimum_clearance_m<=0||e.brake_at_s<e.detected_at_s)throw Error('Expected physical safe stop did not complete');
    if(!a.agv_physics.last_brake_valid)throw Error('Tire brake record invalidated');
    await writeFile(new URL('../../PortSim/Saved/Tests/AGVEmergency_RuntimeStopped.json',import.meta.url),JSON.stringify(s,null,2));
    const r=await command(s,a,'clear_worker');if(!r.applied)throw Error(r.message);
    const released=r.state.actors.find(v=>v.id===vehicle);
    if(released.emergency_test.active||!released.emergency_test.stopped)throw Error('Scenario release lost history or remained active');
    if(!summary.otherTrafficMoved)throw Error('No other traffic movement observed');
    summary.valid=true;await writeFile(output,JSON.stringify(summary,null,2));
    console.log(`PASS web → native ${summary.vehicle}: physical safe stop, ${e.braking_distance_m.toFixed(3)}m braking travel, ${e.minimum_clearance_m.toFixed(3)}m worker clearance; released; other traffic moved`);
    process.exit(0);
   }
  }
  await wait(250);
 }
 throw Error('No eligible loaded AGV completed an integrated emergency');
}catch(error){summary.valid=false;summary.error=error.message;await writeFile(output,JSON.stringify(summary,null,2));throw error;}

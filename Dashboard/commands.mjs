import {mkdir,readdir,readFile,writeFile,rename,unlink} from 'node:fs/promises';
import {randomUUID} from 'node:crypto';
import path from 'node:path';

export function equipmentCommands({directory,readState}){
 return async(req,res)=>{
  const reply=(status,message,extra={})=>{res.writeHead(status,{'Content-Type':'application/json; charset=utf-8'});res.end(JSON.stringify({message,...extra}));};
  const origin=req.headers.origin;
  if(origin!==`http://${req.headers.host}` || (req.headers['sec-fetch-site'] && req.headers['sec-fetch-site']!=='same-origin'))return reply(403,'현재 웹사이트에서만 장비를 제어할 수 있습니다.');
  if(!/^application\/json(?:;|$)/i.test(req.headers['content-type']||''))return reply(415,'JSON 명령만 지원합니다.');
  let body='',input;
  for await(const part of req){body+=part;if(Buffer.byteLength(body)>1024)return reply(413,'명령이 너무 큽니다.');}
  try{input=JSON.parse(body);}catch{return reply(400,'잘못된 명령 형식입니다.');}
  const {state,live}=await readState();
  if(!live || !/^[a-f0-9]{32}$/i.test(state?.control_session||''))return reply(409,'제어 가능한 실시간 엔진 연결이 필요합니다.');
  if(input.run_id!==state.run_id || input.control_session!==state.control_session)return reply(409,'실행이 변경됐습니다. 새로고침하세요.');
  if(state.fault)return reply(409,'전체 안전 정지 원인을 먼저 해결하세요.');
  const actor=state.actors.find(a=>a.id===input.actor_id);
  const actions=actor?.type==='agv'?['fail','repair','test_wet_worker','test_wet_unsecured','clear_worker']:['fail','fail_reboot','fail_sensor_repair','fail_gantry','repair'];
  if(!actor || !['sts','rmg','agv'].includes(actor.type) || !actions.includes(input.action))return reply(400,'지원하지 않는 장비 또는 명령입니다.');
  if(actor.fault && actor.fault_scope!=='equipment')return reply(409,'센서 수리로 안전 오류를 해제할 수 없습니다.');
  await mkdir(directory,{recursive:true});
  let pending=0;
  for(const file of await readdir(directory)){
   if(!/^[a-f0-9]{32}\.json$/.test(file))continue;
   try{
    const old=JSON.parse(await readFile(path.join(directory,file),'utf8'));
    if(Date.now()-Date.parse(old.created_at)>10000 || old.control_session!==state.control_session){await unlink(path.join(directory,file));continue;}
    pending++;if(old.actor_id===actor.id)return reply(409,'이 장비의 이전 명령 응답을 기다리고 있습니다.');
   }catch(e){if(e.code!=='ENOENT')pending++;}
  }
  if(pending>=32)return reply(429,'대기 중인 명령이 많습니다. 잠시 후 다시 시도하세요.');
  const id=randomUUID().replaceAll('-','');
  const command={id,run_id:state.run_id,control_session:state.control_session,actor_id:actor.id,action:input.action,created_at:new Date().toISOString()};
  const temporary=path.join(directory,`${id}.tmp`),target=path.join(directory,`${id}.json`);
  await writeFile(temporary,JSON.stringify(command),{flag:'wx'});await rename(temporary,target);
  return reply(202,'엔진 응답 대기',{id,status:'queued'});
 };
}

import http from 'node:http';
import {readFile, stat} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import path from 'node:path';
import {equipmentCommands} from './commands.mjs';

const root = path.dirname(fileURLToPath(import.meta.url));
const defaultRepo = path.dirname(root);
const port = Number(process.env.PORTSIM_DASHBOARD_PORT || 4173);
const files = new Map([
  ['/', ['index.html', 'text/html']],
  ['/style.css', ['style.css', 'text/css']],
  ['/app.mjs', ['app.mjs', 'text/javascript']],
  ['/metrics.mjs', ['metrics.mjs', 'text/javascript']],
  ['/friction-view.mjs', ['friction-view.mjs', 'text/javascript']],
  ['/agv-physics-view.mjs', ['agv-physics-view.mjs', 'text/javascript']],
  ['/agv-emergency-view.mjs', ['agv-emergency-view.mjs', 'text/javascript']],
  ['/recovery-view.mjs', ['recovery-view.mjs', 'text/javascript']],
  ['/control-view.mjs', ['control-view.mjs', 'text/javascript']],
  ['/terminal-map.mjs', ['terminal-map.mjs', 'text/javascript']],
  ['/terminal-map.css', ['terminal-map.css', 'text/css']]
]);
export function createDashboardServer({repo=defaultRepo}={}){
let cache = {mtime: 0, state: null};
let layoutCache={run:null,mtime:0,data:null};
async function readState() {
  const file = path.join(repo, 'PortSim/Saved/Dashboard/state.json');
  try {
    const info = await stat(file);
    if (info.mtimeMs !== cache.mtime) {
      const state = JSON.parse(await readFile(file, 'utf8'));
      if (state.schema_version !== 1 || !Array.isArray(state.actors)) throw new Error('Unsupported snapshot');
      cache = {mtime: info.mtimeMs, state};
    }
  } catch (error) {
    // Keep the last valid snapshot even if the file is momentarily absent/locked.
    // Its timestamp expires the live badge; the next poll retries the read.
  }
  const age = cache.state ? Math.max(0, (Date.now()-Date.parse(cache.state.generated_at))/1000) : null;
  return {state: cache.state, live: age !== null && age < 5, age_seconds: age, server_time: new Date().toISOString()};
}
const control=equipmentCommands({directory:path.join(repo,'PortSim/Saved/Dashboard/commands'),readState});
// Serialize publication, including duplicate/rate checks across browser tabs.
let commands=Promise.resolve();
const server = http.createServer(async (req, res) => {
  res.setHeader('Cache-Control', 'no-store');
  res.setHeader('X-Content-Type-Options', 'nosniff');
  res.setHeader('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'");
  if (!['127.0.0.1', 'localhost'].includes((req.headers.host || '').split(':')[0])) { res.writeHead(403).end(); return; }
  try {
    const url = new URL(req.url, `http://127.0.0.1:${port}`);
    if(req.method==='POST' && url.pathname==='/api/equipment-command'){
      commands=commands.then(()=>control(req,res)).catch(()=>{if(!res.writableEnded)res.writeHead(500).end('Command publication failed');});
      await commands;return;
    }
    if (req.method !== 'GET') { res.writeHead(405).end(); return; }
    if (url.pathname === '/api/health') {
      res.setHeader('Content-Type', 'application/json; charset=utf-8');
      res.end(JSON.stringify({application: 'PortSim Observatory', repository: repo})); return;
    }
    if (url.pathname === '/api/state') {
      res.setHeader('Content-Type', 'application/json; charset=utf-8');
      res.end(JSON.stringify(await readState())); return;
    }
    if(url.pathname==='/api/terminal-layout'){
      res.setHeader('Content-Type','application/json; charset=utf-8');
      const {state}=await readState(),run=state?.run_id;
      if(!/^[A-Za-z0-9_-]{1,100}$/.test(run||'')){res.end(JSON.stringify({layout:null}));return;}
      try{
        const file=path.join(repo,'PortSim/Saved/Dashboard',`layout-${run}.json`),info=await stat(file);
        if(layoutCache.run!==run||layoutCache.mtime!==info.mtimeMs){
          const data=JSON.parse(await readFile(file,'utf8'));
          if(data.schema_version!==1||data.run_id!==run||!Array.isArray(data.yard_bays)||!Array.isArray(data.bounds_m))throw Error('Mismatched layout');
          layoutCache={run,mtime:info.mtimeMs,data};
        }
        res.end(JSON.stringify({layout:layoutCache.data}));
      }catch(error){
        if(error.code==='ENOENT'){res.end(JSON.stringify({layout:null}));return;}
        res.writeHead(503).end(JSON.stringify({error:'현재 실행과 일치하는 배치 데이터를 읽지 못했습니다.'}));
      }
      return;
    }
    if (url.pathname === '/api/agv-experiments' || url.pathname === '/api/agv-emergencies') {
      res.setHeader('Content-Type','application/json; charset=utf-8');
      let report=null;
      try { report=JSON.parse(await readFile(path.join(repo,'PortSim/Saved/Tests',url.pathname==='/api/agv-emergencies'?'AGVEmergency_Report.json':'AGVDynamics_Report.json'),'utf8')); }
      catch(error){ if(error.code!=='ENOENT') {res.writeHead(503).end(JSON.stringify({error:'시험 보고서를 읽을 수 없습니다.'}));return;} }
      res.end(JSON.stringify({report}));return;
    }
    if (url.pathname === '/api/reference') {
      const [reference, settings] = await Promise.all([
        readFile(path.join(repo, 'Document/STS/STS_ReferenceData.json'), 'utf8'),
        readFile(path.join(repo, 'PortSim/Config/STS_Simulation.json'), 'utf8')
      ]);
      res.setHeader('Content-Type', 'application/json; charset=utf-8');
      res.end(JSON.stringify({reference: JSON.parse(reference), settings: JSON.parse(settings)})); return;
    }
    const entry = files.get(url.pathname);
    if (!entry) { res.writeHead(404).end('Not found'); return; }
    res.setHeader('Content-Type', `${entry[1]}; charset=utf-8`);
    res.end(await readFile(path.join(root, 'public', entry[0])));
  } catch { res.writeHead(500).end('Unable to read project data'); }
});
return server;
}
if(process.argv[1] && path.resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const server=createDashboardServer();
 server.listen(port, '127.0.0.1', () => console.log(`PortSim Observatory: http://127.0.0.1:${port}`));
 server.on('error', e => { console.error(`Dashboard: ${e.message}`); process.exitCode=1; });
}

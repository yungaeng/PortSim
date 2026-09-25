import http from 'node:http';
import {readFile, stat} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import path from 'node:path';

const root = path.dirname(fileURLToPath(import.meta.url));
const repo = path.dirname(root);
const port = Number(process.env.PORTSIM_DASHBOARD_PORT || 4173);
const files = new Map([
  ['/', ['index.html', 'text/html']],
  ['/style.css', ['style.css', 'text/css']],
  ['/app.mjs', ['app.mjs', 'text/javascript']],
  ['/metrics.mjs', ['metrics.mjs', 'text/javascript']]
]);
let cache = {mtime: 0, state: null};
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
const server = http.createServer(async (req, res) => {
  res.setHeader('Cache-Control', 'no-store');
  res.setHeader('X-Content-Type-Options', 'nosniff');
  res.setHeader('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'");
  if (!['127.0.0.1', 'localhost'].includes((req.headers.host || '').split(':')[0])) { res.writeHead(403).end(); return; }
  if (req.method !== 'GET') { res.writeHead(405).end(); return; }
  try {
    const url = new URL(req.url, `http://127.0.0.1:${port}`);
    if (url.pathname === '/api/state') {
      res.setHeader('Content-Type', 'application/json; charset=utf-8');
      res.end(JSON.stringify(await readState())); return;
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
server.listen(port, '127.0.0.1', () => console.log(`PortSim Observatory: http://127.0.0.1:${port}`));
server.on('error', e => { console.error(`Dashboard: ${e.message}`); process.exitCode=1; });

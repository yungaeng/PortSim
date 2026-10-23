import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtemp,mkdir,writeFile,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {footprintRect,mapViewport,renderTerminalOverview} from '../public/terminal-map.mjs';
import {createDashboardServer} from '../server.mjs';

const layout=()=>({schema_version:1,run_id:'test-run',bounds_m:[-75,-540,950,660],yard_view_m:[165,-515,660,360],quay_view_m:[-65,-525,178,350],depot_view_m:[30,345,170,650],
 boundary_m:[[-7,-525],[705,-525],[925,645],[-7,645]],pavement:[{position_m:[635,-160],size_m:[22,900],yaw_deg:0}],markings:[],buildings:[],
 yard_bays:[...Array.from({length:7},(_,i)=>i*2),18,20,22,24,26,28].map(bay=>({yard_id:'yard-1',bay,length_ft:bay===0||bay===18?20:40,pads:bay===0||bay===18?2:1,rows:7,position_m:[205+bay*13,-465],size_m:[12.192,21]})),
 yard_blocks:[{id:'yard-1',number:1,label_position_m:[172,-465]}],central_road_m:[145,-160,635,16]});
test('footprints retain actual metre position, dimensions and rotation',()=>{
 assert.equal(footprintRect({position_m:[100,200],size_m:[4.2,14.6],yaw_deg:90}),'<rect x="97.9" y="192.7" width="4.2" height="14.6" transform="rotate(90 100 200)" />');
 assert.equal(footprintRect({position_m:[NaN,2],size_m:[1,2]}),'');
 assert.equal(footprintRect({position_m:[1,2],size_m:[-1,2]}),'');
 const v=mapViewport(layout(),'all');assert.deepEqual(v,{minX:-75,minY:-540,width:1025,height:1200});
 assert.deepEqual(mapViewport(layout(),'gap'),{minX:165,minY:-300,width:495,height:280});
});
test('seven/six yard columns keep their real gap; AGV and remaining route share world coordinates',()=>{
 const s={run_id:'test-run',actors:[{id:'agv',name:'AGV-100',type:'agv',position_m:[635,-160,0],rotation_deg:[0,0,90],route:[{position_m:[643,-164.5,0]}]}]};
 const html=renderTerminalOverview(s,layout(),{selected:'agv'});
 assert.equal((html.match(/data-yard-bay=/g)||[]).length,13);
 assert.doesNotMatch(html,/data-yard-bay="14"|data-yard-bay="16"/);
 assert.match(html,/x="632.9" y="-167.3" width="4.2" height="14.6" transform="rotate\(90 635 -160\)"/);
 assert.match(html,/points="635,-160 643,-164.5"/);assert.match(html,/preserveAspectRatio="xMidYMid meet"/);
 assert.match(html,/중앙도로 · 16m/);assert.match(html,/7열 · 빈 공간 · 6열/);
 const zoomed=renderTerminalOverview(s,layout(),{mode:'yard'});
 assert.match(zoomed,/viewBox="165 -515 495 875"/);
 assert.match(zoomed,/rotate\(90 635 -160\)/,'View changes do not alter vehicle coordinates');
 assert.match(zoomed,/clip-path="url\(#terminal-plan-clip\)"/,'Meet letterboxes cannot reveal geometry outside the selected world region');
});
test('actual yard cargo is separate from empty slot markings; wrong-run backgrounds are never drawn',()=>{
 const s={run_id:'test-run',actors:[{id:'cargo',name:'<Cargo>',type:'container',state:'Yard',position_m:[300,100,1.5],dimensions_m:[2.438,12.192,2.591],rotation_deg:[0,0,90]}]};
 const html=renderTerminalOverview(s,layout());assert.match(html,/map-yard-cargo/);assert.match(html,/&lt;Cargo&gt;/);
 assert.doesNotMatch(renderTerminalOverview({...s,actors:[]},layout()),/class="map-yard-cargo/);
 assert.match(renderTerminalOverview({...s,run_id:'other'},layout()),/엔진 배치 데이터 대기/);
 assert.doesNotMatch(renderTerminalOverview({...s,run_id:'other'},layout()),/map-yard-cargo|<svg/);
});
test('HTTP serves only the layout for the current run and rejects corrupt or mismatched exports',async t=>{
 const repo=await mkdtemp(path.join(tmpdir(),'portsim-map-')),dir=path.join(repo,'PortSim/Saved/Dashboard');
 await mkdir(dir,{recursive:true});await writeFile(path.join(dir,'state.json'),JSON.stringify({schema_version:1,run_id:'test-run',generated_at:new Date().toISOString(),actors:[]}));
 const server=createDashboardServer({repo});await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
 t.after(async()=>{server.closeAllConnections();await new Promise(resolve=>server.close(resolve));if(!path.resolve(repo).startsWith(path.join(tmpdir(),'portsim-map-')))throw Error('Unexpected fixture path');await rm(repo,{recursive:true,force:true});});
 const base=`http://127.0.0.1:${server.address().port}`;
 assert.deepEqual(await (await fetch(base+'/api/terminal-layout')).json(),{layout:null});
 const file=path.join(dir,'layout-test-run.json');await writeFile(file,JSON.stringify(layout()));
 assert.equal((await (await fetch(base+'/api/terminal-layout')).json()).layout.yard_bays.length,13);
 await writeFile(file,JSON.stringify({...layout(),run_id:'old-run'}));assert.equal((await fetch(base+'/api/terminal-layout')).status,503);
 await writeFile(file,'{bad');assert.equal((await fetch(base+'/api/terminal-layout')).status,503);
 assert.equal((await fetch(base+'/terminal-map.mjs')).status,200);assert.equal((await fetch(base+'/terminal-map.css')).status,200);
});

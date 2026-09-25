import test from 'node:test';
import assert from 'node:assert/strict';
import {compareOperation,vesselDuration,formatDuration} from '../public/metrics.mjs';
const base={baselineHours:10,baselineCount:528,baselineCranes:3,automatedSeconds:21600,automatedCount:528,automatedCranes:3};
test('equal workload: 10 hours versus 6 hours saves 40%',()=>{
 const r=compareOperation(base);assert.equal(r.ok,true);assert.equal(r.percent,40);assert.equal(r.savedSeconds,14400);
});
test('a slower automated run reports negative savings',()=>{
 const r=compareOperation({...base,automatedSeconds:43200});assert.equal(r.percent,-20);assert.equal(r.savedSeconds,-7200);
});
test('missing, zero, nonfinite and negative inputs never yield a percentage',()=>{
 for(const value of ['',null,0,-1,NaN,Infinity,'unknown']) assert.equal(compareOperation({...base,baselineHours:value}).ok,false);
 assert.equal(compareOperation({...base,automatedSeconds:null}).ok,false);
});
test('different cargo counts or crane counts are not comparable',()=>{
 assert.equal(compareOperation({...base,baselineCount:18}).ok,false);
 assert.equal(compareOperation({...base,baselineCranes:1}).ok,false);
 assert.equal(compareOperation({...base,baselineCount:528.5,automatedCount:528.5}).ok,false);
});
test('completion uses vessel start to final event, not sum of crane durations',()=>{
 const v={started_at:5,initial_count:528,unloaded:528,placed:528,unloaded_at:36005,placed_at:36125};
 assert.equal(vesselDuration(v,'handover'),36000);assert.equal(vesselDuration(v,'yard'),36120);
 assert.equal(vesselDuration({...v,unloaded:527},'handover'),null);
 assert.equal(vesselDuration({...v,placed_at:-1},'yard'),null);
 assert.equal(vesselDuration({...v,started_at:-1},'handover'),null);
 assert.equal(vesselDuration({...v,initial_count:0,unloaded:0},'handover'),null);
});
test('duration retains more than 24 hours and rejects missing values',()=>{
 assert.equal(formatDuration(90061),'25:01:01');assert.equal(formatDuration(null),'—');assert.equal(formatDuration(-1),'—');
});

const assert=require('node:assert/strict');
const {decision,create}=require('../dds_swarm_fusion/static/track-output-status.js');
const data=()=>({alive:true,available:true,server_time:100,
  state:{revision:1,batch:{revision:1,published_at:99,publish_ok:true,timestamp_ms:1},
    decisions:[{id:5,passed:true,reasons:[]},{id:15,passed:false,reasons:['too_few_points']}]}});
const reason={too_few_points:'有效定位点数不足'};
assert.equal(decision(data(),5).label,'放行');
assert.equal(decision(data(),'5').label,'放行');
assert.equal(decision(data(),15,0,reason).label,'拦截');
assert.match(decision(data(),15,0,reason).title,/有效定位点数不足/);
assert.equal(decision(data(),50).label,'待确认');
assert.equal(decision(null,5).label,'待确认');
assert.equal(decision({...data(),alive:false},5).label,'待确认');
assert.equal(decision({...data(),available:false},5).label,'待确认');
assert.equal(decision(data(),5,10).label,'待确认');
let changed=data();changed.state.revision=2;
assert.equal(decision(changed,5).label,'待确认');
let failed=data();failed.state.batch.publish_ok=false;failed.state.batch.publish_error='DDS failed';
assert.equal(decision(failed,5).label,'发送失败');
assert.equal(decision(failed,15).label,'拦截');
delete failed.state.batch.publish_error;
assert.equal(decision(failed,5).label,'待确认');
let stale=data();stale.state.batch.published_at=80;
assert.equal(decision(stale,5).label,'待确认');
let missingTime=data();delete missingTime.state.batch.published_at;
assert.equal(decision(missingTime,5).label,'待确认');
assert.equal(decision(data(),5).kind,'passed'); // old replay timestamp must not imply stale
const complete=data();complete.state.decisions[0].track_age_s=9.;complete.state.decisions[0].filter_phase='complete';
assert.equal(decision(complete,5).label,'放行');
assert.match(decision(complete,5).title,/筛选期已结束/);
assert.match(decision(complete,5).title,/9.000 秒/);
complete.state.decisions[0].filter_phase='initial';complete.state.decisions[0].track_age_s=8.999;
assert.match(decision(complete,5).title,/前9秒筛选中/);
assert.match(decision(complete,5).title,/8.999 秒/);

// Minimal DOM contract: attributes only; React text and ID ordering untouched.
class Cell {
  constructor(text){this.textContent=text;this.attrs={};}
  getAttribute(k){return this.attrs[k];}
  setAttribute(k,v){this.attrs[k]=v;}
}
const five=new Cell('▲ F5'),fifteen=new Cell('▲ F15'),header=new Cell('航迹');
let cells=[fifteen,five],elapsed=0,observerCallback,observed;
const rail={classList:{add(){}},querySelector:()=>header,querySelectorAll:()=>cells};
const doc={querySelector:s=>s.endsWith('.tlist')?{}:rail,getElementById:()=>({}),
  defaultView:{performance:{now:()=>elapsed},requestAnimationFrame:fn=>fn(),
    MutationObserver:class {constructor(fn){observerCallback=fn;}observe(root,opts){observed=opts;}}}};
const badges=create(doc,reason);assert.equal(badges.active(),true);
assert.equal(five.attrs['data-dds-label'],'待确认');
badges.update(data());
assert.equal(five.attrs['data-dds-label'],'放行');
assert.equal(fifteen.attrs['data-dds-label'],'拦截');
assert.match(fifteen.attrs.title,/有效定位点数不足/);
assert.equal(five.textContent,'▲ F5');
assert.equal(observed.attributes,undefined); // no observer feedback from own writes
cells=[five,fifteen,new Cell('▲ F500')];observerCallback();
assert.equal(cells[2].attrs['data-dds-label'],'待确认');
let reversed=data();reversed.state.decisions[0].passed=false;reversed.state.decisions[0].reasons=['too_few_points'];
badges.update(reversed);assert.equal(five.attrs['data-dds-label'],'拦截');
badges.update(data());assert.equal(five.attrs['data-dds-label'],'放行');
elapsed=11000;badges.sync();assert.equal(five.attrs['data-dds-label'],'待确认');
badges.update(null);assert.equal(fifteen.attrs['data-dds-label'],'待确认');
console.log('PASS: policy state, ID matching, freshness/replay, failures, reorder, React-safe attributes, filter changes, disconnect');

/* Offline DOM-contract/interaction tests; mocked HTTP only, no live DDS. */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const script = fs.readFileSync(path.join(__dirname, '../dds_swarm_fusion/static/output-filter.js'), 'utf8');
const defaults = {enabled:true, points_enabled:true, min_points:2, min_speed_enabled:true,
  min_speed_mps:1, max_speed_enabled:true, max_speed_mps:15, height_enabled:true,
  min_height_m:30, reference_radar_id:'radar_3152'};
const flags = ['points_enabled','min_speed_enabled','max_speed_enabled','height_enabled'];
const clone = x => JSON.parse(JSON.stringify(x));
const drain = async () => { for(let i=0;i<10;i++) await new Promise(resolve=>setImmediate(resolve)); };

function harness(settings=defaults) {
  const elements = new Map();
  class Element {
    constructor(tag) { this.tag=tag; this.attrs={}; this.events={}; this.children=[]; this.value='';
      this.checked=false; this.disabled=false; this.textContent=''; this.open=false;
      this.classList={toggle(){}}; }
    set id(value) {this.attrs.id=value; elements.set(value,this);}
    get id() {return this.attrs.id;}
    set value(value) {this._value=String(value);}
    get value() {return this._value;}
    setAttribute(k,v) {this.attrs[k]=String(v);}
    getAttribute(k) {return this.attrs[k]??null;}
    set innerHTML(value) {
      this.html=value;
      for(const [,tag,attrs] of value.matchAll(/<([a-z]+)\b([^>]*\bid="[^"]+"[^>]*)>/g)) {
        const e=new Element(tag);
        for(const [,key,v] of attrs.matchAll(/([\w-]+)="([^"]*)"/g)) e.setAttribute(key,v);
        e.id=e.attrs.id; e.disabled=/\sdisabled(?:\s|$)/.test(attrs);
        if(tag==='select')e.value='all';
      }
    }
    addEventListener(name,fn) {this.events[name]=fn;}
    append(...items) {this.children.push(...items);}
    replaceChildren(...items) {this.children=items;}
    showModal() {this.open=true;}
    close() {this.open=false;}
    async click() {if(!this.disabled)this.events.click?.();await drain();}
  }
  const state={instance_id:'fixture',revision:0,saved_hash:'hash0',settings:clone(settings),
    saved_settings:clone(settings),config_path:'/fixture/output_filter.yaml',config_error:null,
    filter_window_policy:'track_initial',filter_window_s:9,unsaved:false,decisions:[],batch:null};
  const h={state,requests:[],online:true,mode:'ok',poll:null,queued:null};
  const response=()=>({alive:true,available:true,state:clone(state),token:'test-token'});
  const ack=(command,ok=true)=>{
    if(ok){state.settings=clone(command.settings);state.revision++;
      if(command.save){state.saved_settings=clone(command.settings);state.saved_hash='hash'+state.revision;}}
    state.unsaved=JSON.stringify(state.settings)!==JSON.stringify(state.saved_settings);
    state.last_command={request_id:'request-'+h.requests.length,ok,saved:ok&&command.save,error:ok?null:'permission denied'};
  };
  const fetch=async(url,options={})=>{
    if(!h.online)throw Error('offline');
    if(options.method!=='POST')return {ok:true,json:async()=>response()};
    const command=JSON.parse(options.body);h.requests.push(command);
    if(h.mode==='pending'){h.queued=command;return {ok:true,json:async()=>({...response(),pending:true,request_id:'request-'+h.requests.length})};}
    ack(command,h.mode!=='fail');
    return {ok:h.mode!=='fail',json:async()=>({...response(),result:clone(state.last_command)})};
  };
  h.ack=()=>{ack(h.queued);h.queued=null;};
  const document={body:new Element('body'),createElement:tag=>new Element(tag),getElementById:id=>elements.get(id)};
  const context={document,window:{},fetch,AbortSignal:{timeout:()=>null},setInterval:fn=>{h.poll=fn;},confirm:()=>true};
  vm.runInNewContext(script,context);
  h.el=id=>document.getElementById('of-'+id);
  h.checked=flag=>h.el(flag).getAttribute('aria-checked')==='true';
  h.refresh=async()=>{h.poll();await drain();};
  h.open=()=>h.el('open').click();
  return h;
}

(async()=>{
  const h=harness({...defaults,points_enabled:false,height_enabled:false});
  assert.equal(h.el('points_enabled').disabled,true);
  await h.open();
  assert.equal(h.checked('points_enabled'),false);
  assert.equal(h.checked('min_speed_enabled'),true);
  assert.equal(h.el('points_enabled').getAttribute('role'),'switch');
  assert.match(h.el('state-points_enabled').textContent,/当前生效：关闭.*配置保存：关闭/);
  assert.equal(h.el('min_points').disabled,true);
  await h.el('points_enabled').click();
  assert.equal(h.requests.length,0,'toggle alone must not send or save');
  assert.equal(h.checked('points_enabled'),true);
  assert.equal(h.el('min_points').disabled,false);
  assert.match(h.el('state-points_enabled').textContent,/修改未应用/);
  await h.refresh();
  assert.equal(h.checked('points_enabled'),true,'poll must preserve unapplied choice');
  await h.el('apply').click();
  assert.equal(h.requests[0].save,false);
  assert.equal(h.state.settings.points_enabled,true);
  assert.equal(h.state.saved_settings.points_enabled,false);
  assert.match(h.el('state-points_enabled').textContent,/当前生效：开启.*配置保存：关闭/);
  await h.el('height_enabled').click();
  await h.el('save').click();
  assert.equal(h.requests[1].save,true);
  assert.equal(h.state.saved_settings.height_enabled,true);
  assert.equal(h.state.saved_settings.points_enabled,true);
  assert.match(h.el('message').textContent,/开关和阈值已生效并保存/);
  for(const flag of flags){
    const before=clone(h.state.settings);
    await h.el(flag).click();await h.el('save').click();
    for(const other of flags)assert.equal(h.state.saved_settings[other],other===flag?!before[other]:before[other]);
  }
  h.online=false;await h.refresh();
  for(const flag of flags)assert.equal(h.el(flag).disabled,true);
  assert.match(h.el('state-points_enabled').textContent,/未连接，待确认/);

  const p=harness();await p.open();
  await p.el('points_enabled').click();p.mode='pending';await p.el('save').click();
  assert.equal(p.el('points_enabled').disabled,true);
  assert.equal(p.el('save').disabled,true);
  assert.equal(p.state.settings.points_enabled,true);
  p.ack();await p.refresh();
  assert.equal(p.checked('points_enabled'),false);
  assert.equal(p.el('save').disabled,false);
  assert.match(p.el('message').textContent,/已生效并保存/);

  const f=harness();await f.open();f.mode='fail';
  await f.el('height_enabled').click();await f.el('save').click();
  assert.equal(f.state.settings.height_enabled,true);
  assert.equal(f.state.saved_settings.height_enabled,true);
  assert.equal(f.checked('height_enabled'),false,'retain failed draft for user inspection');
  assert.match(f.el('message').textContent,/permission denied/);
  assert.match(f.el('state-height_enabled').textContent,/当前生效：开启.*修改未应用/);
  await f.el('restore').click();
  assert.equal(f.checked('height_enabled'),true);

  const m=harness({...defaults,enabled:false,points_enabled:false});await m.open();
  assert.equal(m.checked('points_enabled'),false);
  assert.equal(m.checked('height_enabled'),true);
  assert.equal(m.el('min_height_m').disabled,true);
  assert.match(m.el('state-height_enabled').textContent,/总开关关闭/);
  m.el('enabled').checked=true;m.el('enabled').events.change();
  assert.equal(m.el('min_height_m').disabled,false);
  assert.equal(m.el('min_points').disabled,true);

  const b=harness();await b.open();
  await b.el('max_speed_enabled').click();b.el('min_speed_mps').value='20';
  await b.el('save').click();assert.equal(b.requests.length,1);
  await b.el('max_speed_enabled').click();await b.el('save').click();
  assert.equal(b.requests.length,1,'invalid re-enabled min/max must not send');
  assert.match(b.el('message').textContent,/同时开启速度上下限/);
  console.log('PASS: independent switch buttons, startup values, draft/poll, temporary apply, save ACK, pending lock, failed save, offline, master switch, one-sided speed bounds');
})().catch(e=>{console.error(e);process.exitCode=1;});

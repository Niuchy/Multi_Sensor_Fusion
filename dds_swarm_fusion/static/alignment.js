/* Preview-only UI. No DDS publisher and no live fusion control commands. */
'use strict';
const $ = id => document.getElementById(id);
let state = null, result = null, paused = false, dirty = false, busy = false;
let offlineContent = null, playing = null, generation = 0, flashUntil = 0;
let draftRevision = 0;
let defaults = {time_source:'frame', clock_basis:'unknown', time_offset_ms:0,
  coordinate_mode:'auto', station_enu_m:null, azimuth_bias_deg:0,
  elevation_bias_deg:0, range_bias_m:0, position_bias_enu_m:[0,0,0]};
const numericKeys = ['time_offset_ms','azimuth_bias_deg','elevation_bias_deg','range_bias_m'];
const selectKeys = ['time_source','clock_basis','coordinate_mode'];
const vectorKeys = ['station_enu_m','position_bias_enu_m'];
const fmt = (x, digits=2) => x === null || x === undefined ? '—' : Number(x).toFixed(digits);
const vec = xyz => xyz ? xyz.map(x=>fmt(x,2)).join(', ') : '不可用';
const device = () => $('device').value.trim();
const offline = () => $('mode').value === 'offline';
function notice(message, error=false, flash=false) {
  if (!flash && Date.now() < flashUntil) return;
  $('notice').textContent = message; $('notice').classList.toggle('error',error);
  if(flash) flashUntil = Date.now()+6000;
}
async function api(path, data) {
  const opts = {cache:'no-store'};
  if (data !== undefined) {
    opts.method='POST'; opts.headers={'Content-Type':'application/json','X-Alignment-Token':state?.token || ''};
    opts.body=JSON.stringify(data);
  }
  const response=await fetch('/api/alignment'+path,opts);
  if(response.status===404) throw Error('网关尚未加载时空对齐接口。请在方便时手动重启融合服务，再刷新本页。');
  const body=await response.json();
  if(!response.ok) throw Error(typeof body.detail==='string' ? body.detail : JSON.stringify(body.detail));
  return body;
}
function parseVector(text, optional=false) {
  if(optional && !text.trim()) return null;
  const parts=text.trim().split(/[,，\s]+/);
  const values=parts.map(Number);
  if(parts.length!==3 || values.some(x=>!Number.isFinite(x))) throw Error('ENU / 原点坐标需要三个有限数值，用逗号分隔。');
  return values;
}
function params() {
  if(!$('params').reportValidity()) throw Error('请检查参数范围。');
  const p={};
  for(const key of selectKeys) p[key]=$(key).value;
  for(const key of numericKeys) {
    if(!$(key).value.trim()) throw Error('数值参数不能留空。');
    p[key]=Number($(key).value);
  }
  for(const key of vectorKeys) p[key]=parseVector($(key).value,key==='station_enu_m');
  return p;
}
function fillParams(p, isDirty=false) {
  for(const key of [...selectKeys,...numericKeys]) $(key).value=p[key];
  for(const key of vectorKeys) $(key).value=p[key] ? p[key].join(', ') : '';
  dirty=isDirty; draftStatus();
  draftRevision=state?.revision || 0;
}
function draftStatus() { $('draft-status').textContent=dirty?'未保存草稿':'已载入保存值 / 默认值'; }
function historyOptions() {
  const prior=$('revision').value;
  $('revision').replaceChildren(new Option('v0 · 全部默认','0'));
  for(const h of state.history) $('revision').add(new Option(`v${h.revision} · ${h.action} · ${h.note || new Date(h.created_at*1000).toLocaleString()}`,String(h.revision)));
  if([...$('revision').options].some(o=>o.value===prior)) $('revision').value=prior;
  $('version-status').textContent=`服务器当前 v${state.revision} · 保存 / 回滚仅影响预览。\n${state.profile_file}`;
}
function updateState(next) {
  const first=state===null, previousRevision=state?.revision;
  state=next; defaults=next.defaults;
  $('device-options').replaceChildren();
  for(const sid of new Set([...Object.keys(next.devices),...Object.keys(next.profiles)])) $('device-options').append(new Option(sid));
  if(first) {
    const observed=Object.keys(next.devices);
    if(observed.length===1) $('device').value=observed[0];
    fillParams(next.profiles[device()] || defaults);
    fillContext();
  }
  if(first || previousRevision!==next.revision) historyOptions();
  const obs=next.devices[device()];
  $('live-status').textContent=`旁路 ${next.packets} 包 · 缓存 ${next.buffered_records} 条 · 解析异常 ${next.errors} · 已淘汰 ${next.evicted_records} 条${!obs?' · 所选设备未收到':' · 所选设备有接收记录'}`;
  if(next.reader_error) notice('旁路 DDS 订阅异常：'+next.reader_error,true);
  else if(!next.last_poll_at) notice('尚未完成旁路订阅轮询，等待网关。');
  else if(!next.packets) notice('网关已连接，旁路暂未收到 RadarTarget。是否正式输入节点收到数据，请同时查看“接入诊断”。');
  else notice('旁路观测中。原始记录保留；所有参数只用于本页预览和离线比较，不改变正式融合 / DDS 输出。');
  $('updated').textContent='界面更新 '+new Date().toLocaleTimeString();
}
function fillContext() {
  const c=state?.devices[device()]?.context || state?.default_context;
  if(!c) return;
  $('log-origin').value=c.origin ? c.origin.join(', ') : '';
  $('log-station').value=c.station_enu_m ? c.station_enu_m.join(', ') : '';
  $('log-convention').value=c.azimuth_convention || 'north_cw';
}
function stopPlayback() {
  if(playing!==null) clearTimeout(playing);
  playing=null; $('play').textContent='播放';
}
function acceptResult(next) {
  stopPlayback(); result=next;
  const s=next.summary;
  $('count').textContent=`${s.records} / ${next.rows.filter(r=>r.tag===0).length}`;
  $('interval').textContent=`${fmt(s.interval_mean_s,3)} / ${fmt(s.interval_jitter_s,4)} s`;
  $('delta').textContent=`${fmt(s.receive_delta_p50_ms,1)} / ${fmt(s.receive_delta_p95_ms,1)} ms`;
  $('change').textContent=`${fmt(s.position_change_rms_m)} / ${fmt(s.position_change_max_m)} m`;
  $('anomalies').textContent=`重复 ${s.duplicate_observations} · 乱序 ${s.out_of_order_observations} · 坐标异常 ${s.invalid_positions}`;
  const origins=[...new Set(next.rows.map(r=>JSON.stringify(r.context.origin)))];
  const previous=$('origin-filter').value;
  $('origin-filter').replaceChildren();
  for(const key of origins) $('origin-filter').add(new Option(key==='null'?'未知原点':key,key));
  if(origins.includes(previous)) $('origin-filter').value=previous;
  $('cursor').max=String(Math.max(0,next.rows.length-1));
  $('cursor').value=$('cursor').max;
  $('play').disabled=!offline() || !next.rows.length;
  $('file-note').textContent=next.note || '使用采集时的坐标上下文。';
  renderCursor();
}
async function preview() {
  const id=++generation, p=params();
  let next;
  if(offline()) {
    if(offlineContent===null) return;
    const data={content:offlineContent,format:$('file-format').value,device:device(),params:p};
    if(data.format==='receiver_log') data.context={origin:parseVector($('log-origin').value),
      station_enu_m:parseVector($('log-station').value,true),azimuth_convention:$('log-convention').value};
    next=await api('/replay',data);
  } else next=await api('/preview',{device:device(),params:p});
  if(id===generation) acceptResult(next);
}
function details(row) {
  const el=$('record-details'); el.replaceChildren();
  if(!row) { $('record-label').textContent='暂无记录'; $('record-warning').textContent=''; return; }
  $('record-label').textContent=`radar_${row.device_id} / 航迹 ${row.track_id} · ${row.tag?'真实点':'外推点'}`;
  const raw=row.raw_position, corrected=row.corrected_position;
  const items=[['设备 Frame 时间',fmt(row.frame_timestamp_ms,0)+' ms'],['Unit / PXSB 时间',`${fmt(row.unit_timestamp_ms,0)} / ${fmt(row.header_timestamp_ms,0)} ms`],
    ['调试订阅接收时间',fmt(row.received_at_ms,1)+' ms'],['当前预览校正时间',fmt(row.corrected_timestamp_ms,1)+' ms'],
    ['参考原点（经、纬、高）',vec(row.context.origin)],['采集时配置站址 ENU',vec(row.context.station_enu_m)],
    ['配置站址来源',row.context.station_source || '采集 / 用户提供（未核实）'],
    ['方位定义',row.context.azimuth_convention==='north_cw'?'北零、顺时针':'东零、逆时针'],
    ['原始 / 校正坐标来源',`${raw?.source || '不可用'} → ${corrected?.source || '不可用'}`],
    ['原始默认选路 ENU（m）',vec(raw?.xyz)],['校正后 ENU（m）',vec(corrected?.xyz)],
    ['原始 L1 推算 ENU（m）',vec(raw?.l1_xyz)],['原始 L2 转换 ENU（m）',vec(raw?.l2_xyz)],
    ['L1 / L2 三维距离差',fmt(raw?.l1_l2_distance_m)+' m（不是误差真值）'],
    ['设备 L2 经、纬、高',`${fmt(row.raw.lng_deg,7)}, ${fmt(row.raw.lat_deg,7)}, ${fmt(row.raw.alt_m,2)}`],
    ['设备 L1 距离、方位、俯仰',`${fmt(row.raw.range_m)} m, ${fmt(row.raw.azimuth_deg)}°, ${fmt(row.raw.elevation_deg)}°`]];
  for(const [key,value] of items) { const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=key;dd.textContent=value;el.append(dt,dd); }
  $('record-warning').textContent=[...row.errors,...row.warnings].join('；');
}
function renderCursor() {
  const rows=result?.rows || [], cursor=Number($('cursor').value);
  $('cursor-label').textContent=`${rows.length?cursor+1:0} / ${rows.length}`;
  details(rows[cursor]);
  const tbody=$('records');tbody.replaceChildren();
  for(let i=cursor;i>=Math.max(0,cursor-59);i--) {
    const r=rows[i]; if(!r) continue;
    const tr=document.createElement('tr');
    for(const value of [`${r.sequence} / ${r.track_id}`,fmt(r.frame_timestamp_ms,0),fmt(r.corrected_timestamp_ms,0),fmt(r.receive_minus_corrected_ms,1),fmt(r.interval_s,3),`${r.raw_position?.source || '—'} → ${r.corrected_position?.source || '—'}`,r.tag?'真实':'外推']) {
      const td=document.createElement('td');td.textContent=value;tr.append(td);
    }
    tr.onclick=()=>{stopPlayback();$('cursor').value=String(i);renderCursor();}; tbody.append(tr);
  }
  drawPlot();
}
function drawPlot() {
  const canvas=$('plot'), ctx=canvas.getContext('2d'), w=canvas.clientWidth,h=canvas.clientHeight,dpr=window.devicePixelRatio || 1;
  if(!w || !h) return;
  canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);ctx.scale(dpr,dpr);ctx.clearRect(0,0,w,h);
  const origin=$('origin-filter').value, all=(result?.rows || []).filter(r=>JSON.stringify(r.context.origin)===origin);
  const shown=(result?.rows || []).slice(0,Number($('cursor').value)+1).filter(r=>JSON.stringify(r.context.origin)===origin);
  const pts=all.flatMap(r=>[r.raw_position?.xyz,r.corrected_position?.xyz]).filter(Boolean);
  ctx.font='11px system-ui';ctx.fillStyle='#91a6be';
  if(!pts.length) {ctx.textAlign='center';ctx.fillText('暂无可绘制位置 · 请检查数据、参考原点与站址',w/2,h/2);return;}
  const xs=pts.map(p=>p[0]),ys=pts.map(p=>p[1]),xmin=Math.min(...xs),xmax=Math.max(...xs),ymin=Math.min(...ys),ymax=Math.max(...ys);
  const scale=Math.min((w-100)/Math.max(30,xmax-xmin),(h-70)/Math.max(30,ymax-ymin));
  const cx=(xmin+xmax)/2,cy=(ymin+ymax)/2;
  const xy=p=>[w/2+(p[0]-cx)*scale,h/2-(p[1]-cy)*scale];
  ctx.lineWidth=1;ctx.strokeStyle='#203147';
  for(let i=0;i<=4;i++) {
    const x=48+(w-72)*i/4,y=22+(h-57)*i/4;
    ctx.beginPath();ctx.moveTo(x,22);ctx.lineTo(x,h-35);ctx.stroke();
    ctx.beginPath();ctx.moveTo(48,y);ctx.lineTo(w-24,y);ctx.stroke();
    ctx.textAlign='center';ctx.fillText(fmt(cx+(x-w/2)/scale,0),x,h-16);
    ctx.textAlign='right';ctx.fillText(fmt(cy-(y-h/2)/scale,0),44,y+4);
  }
  ctx.textAlign='right';ctx.fillText('E / m',w-6,h-2);ctx.textAlign='left';ctx.fillText('N / m',8,12);
  for(const [key,color] of [['raw_position','#9aabba'],['corrected_position','#55d5c5']]) {
    const groups=new Map();
    for(const r of shown) {
      const id=r.device_id+':'+r.track_id;
      if(!groups.has(id)) groups.set(id,[]);
      groups.get(id).push(r);
    }
    for(const group of groups.values()) {
      ctx.beginPath();let pen=false;ctx.strokeStyle=color;ctx.lineWidth=1.7;
      for(const r of group) {
        if(!r[key]){pen=false;continue;}
        const [x,y]=xy(r[key].xyz);if(pen)ctx.lineTo(x,y);else ctx.moveTo(x,y);pen=true;
      }ctx.stroke();
      for(const r of group) if(r[key]) {
        const [x,y]=xy(r[key].xyz);ctx.beginPath();ctx.arc(x,y,key==='raw_position'?4:2.6,0,Math.PI*2);
        ctx.fillStyle=r.tag?color:'#0b1422';ctx.fill();ctx.strokeStyle=color;ctx.lineWidth=1.2;ctx.stroke();
      }
    }
  }
}
function download(name, obj) {
  const url=URL.createObjectURL(new Blob([JSON.stringify(obj,null,2)],{type:'application/json'}));
  const link=document.createElement('a');link.href=url;link.download=name;link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
}
async function action(fn) {try {await fn();} catch(e) {notice(e.message,true,true);} }
$('params').addEventListener('input',()=>{dirty=true;draftStatus();});
$('params').onsubmit=e=>{e.preventDefault();action(preview);};
$('device').onchange=()=>action(async()=>{generation++;stopPlayback();fillParams(state?.profiles[device()] || defaults);fillContext();await preview();});
$('mode').onchange=()=>action(async()=>{
  generation++;stopPlayback();$('offline-panel').hidden=!offline();$('pause').disabled=offline();
  $('capture').disabled=offline();paused=false;$('pause').textContent='暂停显示';
  if(!offline() || offlineContent!==null) await preview();
  else {result=null;acceptResult({rows:[],summary:{records:0,duplicate_observations:0,out_of_order_observations:0,invalid_positions:0}});}
});
$('pause').onclick=()=>{paused=!paused;$('pause').textContent=paused?'继续显示':'暂停显示';};
$('file-format').onchange=()=>{$('log-context').hidden=$('file-format').value!=='receiver_log';};
$('file').onchange=()=>{
  const file=$('file').files[0]; if(!file) return;
  offlineContent=null;$('file-format').value=/\.json$/i.test(file.name)?'capture':'receiver_log';$('file-format').onchange();
};
$('compare-file').onclick=()=>action(async()=>{
  const file=$('file').files[0];if(!file) throw Error('请先选择采集 JSON 或解码日志。');
  if(file.size>6*1024*1024) throw Error('文件最大 6 MiB（JSON 封装后请求最大 8 MiB）。');
  offlineContent=await file.text();await preview();
  notice('离线对比完成：没有发送 DDS，也没有改变现场融合参数。',false,true);
});
$('save').onclick=()=>action(async()=>{
  if(!state) throw Error('请先连接网关。');
  const next=await api('/save',{device:device(),params:params(),expected_revision:draftRevision,note:$('note').value});
  updateState(next);draftRevision=next.revision;dirty=false;draftStatus();notice(`预览参数已保存为 v${next.revision}。正式融合参数未改变。`,false,true);
});
$('reload').onclick=()=>action(async()=>{updateState(await api(''));fillParams(state.profiles[device()] || defaults);await preview();});
$('reset').onclick=()=>action(async()=>{fillParams(defaults,true);await preview();});
$('rollback').onclick=()=>action(async()=>{
  if(!state) throw Error('请先连接网关。');
  const revision=Number($('revision').value);
  if(!window.confirm(`将全部设备的预览参数集回滚到 v${revision}，并保留现有版本。不会改变正式融合。继续？`)) return;
  updateState(await api('/rollback',{revision,expected_revision:state.revision,note:$('note').value}));
  fillParams(state.profiles[device()] || defaults);await preview();notice(`已回滚并生成 v${state.revision}。`,false,true);
});
$('capture').onclick=()=>action(async()=>download(`radar-capture-${Date.now()}.json`,await api('/capture?device='+encodeURIComponent(device()))));
$('export-result').onclick=()=>action(async()=>{if(!result)throw Error('尚无对比结果。');download(`alignment-comparison-${Date.now()}.json`,{...result,device:device(),exported_at:new Date().toISOString(),profile_revision:state?.revision});});
$('cursor').oninput=()=>{stopPlayback();renderCursor();};
$('origin-filter').onchange=drawPlot;
$('speed').onchange=stopPlayback;
$('play').onclick=()=>{
  if(playing!==null) {stopPlayback();return;}
  const rows=result?.rows;if(!rows?.length || !offline()) return;
  if(Number($('cursor').value)>=rows.length-1) $('cursor').value='0';
  $('play').textContent='暂停回放';renderCursor();
  const next=()=>{
    const i=Number($('cursor').value);
    if(i>=rows.length-1){stopPlayback();return;}
    // Original device report spacing; out-of-order / duplicate data is shown without an invented delay.
    const delay=Math.max(0,rows[i+1].frame_timestamp_ms-rows[i].frame_timestamp_ms)/Number($('speed').value);
    playing=setTimeout(()=>{$('cursor').value=String(i+1);renderCursor();next();},Math.min(delay,2147483647));
  };next();
};
window.addEventListener('resize',drawPlot);
async function tick() {
  if(!busy) {
    busy=true;
    try {updateState(await api(''));if(!offline() && !paused) await preview();}
    catch(e) {notice(e.message,true);}
    finally {busy=false;}
  }
  setTimeout(tick,1500);
}
fillParams(defaults);tick();

(() => {
  const $ = id => document.getElementById(id);
  const esc = value => String(value ?? '—').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const number = (v, digits=1) => typeof v === 'number' && Number.isFinite(v) ? v.toFixed(digits) : '—';
  const pill = (text, cls='') => `<span class="pill ${cls}">${esc(text)}</span>`;
  const empty = (n, text) => `<tr><td colspan="${n}" class="empty">${esc(text)}</td></tr>`;
  const rfPlaceholder = s => s.sensor_id?.startsWith('rf_df_') && s.quality_reasons?.includes('rf_zero_azimuth_placeholder');
  const sampleLabel = s => rfPlaceholder(s) ? '0° 占位 · 无测向量测' : s.extrap_flag===undefined ? '设备报告' : s.extrap_flag ? '真实点' : '外推点';
  let paused = false, lastSnapshot = null;
  $('pause').onclick = () => {paused = !paused; $('pause').textContent = paused ? '继续显示' : '暂停显示';};
  function render(d) {
    const nodes = d.nodes || {}, input = nodes.input || {}, tracking = nodes.tracking || {}, fusion = nodes.fusion || {};
    const topics = input.topics || {}, samples = input.samples || [], history = input.recent_samples || [];
    const packets = Object.values(topics).reduce((sum,t)=>sum+(t.packets||0),0);
    const parsed = Object.values(topics).reduce((sum,t)=>sum+(t.parsed||0),0);
    const errors = Object.values(topics).reduce((sum,t)=>sum+(t.errors||0),0);
    const radio = Object.values(tracking.radar || {});
    const heartbeat = n => n.alive ? pill('心跳正常','ok') : pill(n.heartbeat_at ? '心跳中断' : '未收到心跳','bad');
    const stage = (title,node,count,note) => `<article class="stage"><div class="stage-head"><span>${title}</span>${heartbeat(node)}</div><strong>${esc(count ?? '—')}</strong><p>${esc(note)}</p></article>`;
    $('stages').innerHTML = stage('01 · DDS 接收',input,packets,'报文总数；来自融合输入节点') + stage('02 · 协议解析',input,parsed,`${errors} 次错误 · ${input.raw_batches||0} 个归一化批次`) + stage('03 · 雷达航迹',tracking,radio.reduce((s,t)=>s+(t.total||0),0),`${radio.reduce((s,t)=>s+(t.tentative||0),0)} 条待确认 · 跟踪器重置 ${tracking.reset_count||0} 次`) + stage('04 · 融合输出',fusion,fusion.external_count,`常规输出 ${fusion.fused_count||0} 条；总数包含低置信度滑行`);
    let messages = [];
    if (d.gateway.error) messages.push(`网关 DDS 读取异常：${d.gateway.error}`);
    for (const [name,title] of [['input','输入'],['tracking','跟踪'],['fusion','融合']]) if (!nodes[name]?.alive) messages.push(`${title}节点尚无有效心跳`);
    if (input.alive && !packets) messages.push('输入节点正在运行，但尚未收到设备 DDS 报文。检查设备发布状态、Topic、Domain、组播与网络。');
    if (errors) messages.push('存在报文解析错误，见下方错误列和原始报文头。');
    if (samples.some(s=>s.extrap_flag === 0)) messages.push('雷达报告 tag=0：设备外推点。按低置信度滑行显示与推送；不累计真实量测命中。请与设备端核对标记定义。');
    if (input.last_data_received_at && d.server_time-input.last_data_received_at>4) messages.push(`设备数据已 ${number(d.server_time-input.last_data_received_at)} 秒未更新；界面保留最近收到的记录。`);
    if ((input.dropped_old||0)+(input.duplicates||0)) messages.push(`已丢弃乱序 ${input.dropped_old||0} 条、重复 ${input.duplicates||0} 条。`);
    if (input.rf_zero_azimuth_filtered) messages.push(`射频 0° 占位已过滤 ${input.rf_zero_azimuth_filtered} 条：仅保留接收记录，不画测向线、不参与跟踪与融合；非零方位仍正常处理。`);
    $('notice').textContent = messages.join(' ') || '接收与处理链路有有效心跳。下方数据会随设备报告更新。';
    $('notice').classList.toggle('good', !messages.length);
    $('topics').innerHTML = ['RadarTarget','EOTarget','ESMTarget','AcousticTarget'].map(name=>{
      const t=topics[name]||{}, age=t.last_received_at ? d.server_time-t.last_received_at : null;
      const state=!input.alive ? pill('接收节点离线','bad') : age===null ? pill('未收到数据') : age>4 ? pill('等待 / 数据陈旧','warn') : pill('已收到','ok');
      return `<tr><td>${name}</td><td>${state}</td><td>${t.packets||0} / ${t.parsed||0}</td><td>${number((t.packets||0)/Math.max(1,d.server_time-(input.started_at||d.server_time)),3)}</td><td>${age===null?'—':number(age)+' s'}</td><td class="error">${esc(t.errors||0)}${t.last_error?' · '+esc(t.last_error):''}</td></tr>`;
    }).join('');
    $('input-mode').textContent = `${input.radar_input_mode||'—'} / 方位 ${input.azimuth_convention||'—'}`;
    const stationSources = {reported_lla:'报文设备自身 GPS',configured_lla:'config 经纬高',configured_enu:'config ENU 换算经纬高',unknown:'站址未知'};
    const stationReasons = {valid_device_gps:'报文设备 GPS 有效',protocol_has_no_device_gps:'当前协议无设备自身 GPS 字段',device_gps_missing:'报文缺少设备自身 GPS',device_gps_invalid:'报文设备 GPS 无效（零坐标、非有限数或越界）'};
    if ($('device-stations')) $('device-stations').innerHTML = (input.device_stations||[]).map(s=>{
      const lla=s.station_lla||[],enu=s.station_enu||[],origin=s.station_reference_lla||[];
      return `<tr><td>${esc(s.sensor_id)}</td><td>${pill(stationSources[s.station_source]||'未知',s.station_source==='reported_lla'?'ok':'warn')}</td><td>${number(lla[0],7)} / ${number(lla[1],7)}</td><td>${number(lla[2],2)}</td><td>${enu.map(v=>number(v,2)).join(' / ')||'—'}</td><td>${esc(stationReasons[s.station_reason]||s.station_reason)}<br>${esc(s.station_config||'不使用配置站址')}</td><td>${origin.map((v,i)=>number(v,i<2?7:2)).join(' / ')||'—'}</td><td>${number(s.last_received_at==null?null:d.server_time-s.last_received_at)}</td></tr>`;
    }).join('') || empty(8,'等待设备报文；旧版输入节点可能不提供站址来源，请重启输入服务后查看');
    const eoModes = {position:'设备坐标',ranged:'距离＋角度',bearing:'纯角度',invalid:'未采用'};
    const reasons = {station_unknown:'站址未知',angle_calibration_unconfirmed:'角度标定未确认',
      range_validity_unconfirmed:'测距有效性未确认',position_unchanged_unverified:'坐标长期未变，疑似旧值',
      range_unchanged_unverified:'距离长期未变，疑似旧值',not_normal_tracking_or_fault:'非正常跟踪或故障',
      position_jump:'坐标跳变',update_rate_limited:'高频更新限流',baseline_too_short:'站间基线过短',
      intersection_angle_too_small:'视线夹角过小',intersection_uncertainty_too_large:'预计交汇误差过大',
      rays_do_not_meet:'两条视线不一致',intersection_behind_or_out_of_range:'交点反向或超范围',
      bound_position_rejected:'恢复坐标未通过关联门',accepted:'交汇几何通过'};
    if ($('eo-config')) $('eo-config').textContent = `排序窗口 ${input.reorder_window_ms ?? '—'} ms · ${input.eo_config_path || '等待配置'}`;
    if ($('eo-status')) $('eo-status').innerHTML = samples.filter(s=>s.sensor_id?.startsWith('eo_')).map(s=>{
      const ids=(fusion.tracks||[]).filter(t=>t.source_bindings?.[s.sensor_id]===s.source_track_id).map(t=>t.id);
      return `<tr><td>${esc(s.sensor_id)} / ${esc(s.track_id)}</td><td>${esc(eoModes[s.mode]||'旧版输入')}${s.fresh===false?' · 未作为新量测':''}</td><td>${esc(stationSources[s.station_source]||s.station_source)}</td><td>${esc(ids.join(', ')||'未关联')}</td><td>${esc(s.last_position_ms)} / ${esc(s.last_range_ms)}</td><td>${esc((s.quality_reasons||[]).map(r=>reasons[r]||r).join('；')||'字段检查通过；不等于独立精度验证')}</td></tr>`;
    }).join('') || empty(6,'等待光电报文；纯角度无定位时不会伪造坐标输出');
    if ($('eo-geometry')) $('eo-geometry').textContent = (fusion.geometry_events||[]).slice(-5).map(e=>`源时刻 ${e.timestamp_ms ?? '—'} · ${e.devices.join(' + ')}：${reasons[e.reason]||e.reason}`).join('；') || `方向记录 ${(fusion.direction_tracks||[]).length} 条，双站待确认 ${(fusion.eo_pending||[]).length} 组。交汇需独立站址、有效标定和近同时刻观测。`;
    $('samples').innerHTML = samples.length ? samples.map(s=>{
      const fields = [['源时间戳',s.timestamp_ms],['同目标间隔',number(s.interval_s,3)+' s'],['距本机接收',number(d.server_time-s.received_at)+' s'],['采用距离',number(s.r)+' m'],['报文原始距离',number(s.raw_range_m ?? s.r)+' m'],['原始方位',number(s.raw_azimuth_deg ?? s.azimuth_deg ?? (s.az==null?null:s.az*180/Math.PI),2)+'°'],['东 / 北',number(s.x)+' / '+number(s.y)],['经度 / 纬度',number(s.lng_deg,7)+' / '+number(s.lat_deg,7)],['海拔',number(s.alt_m)+' m'],['报文速度 / 航向',number(s.speed)+' m/s / '+number(s.course_deg)+'°']];
      return `<div class="sample"><div class="sample-title"><span>${esc(s.sensor_id)} · 目标 ${esc(s.track_id)}</span>${pill(sampleLabel(s),rfPlaceholder(s)?'warn':s.extrap_flag===undefined?'':s.extrap_flag?'ok':'warn')}</div><dl>${fields.map(([k,v])=>`<div><dt>${k}</dt><dd>${esc(v)}</dd></div>`).join('')}</dl></div>`;
    }).join('') : '<p class="empty">等待可解析的设备目标数据</p>';
    $('tracking-note').textContent = radio.map(t=>t.reason).filter(Boolean).join(' · ') || '等待雷达输入';
    $('tracks').innerHTML = (fusion.tracks||[]).map(t=>`<tr><td>${esc(t.id)} / ${esc(t.source_track_id)}</td><td>${esc((t.sensor_ids||[]).join(', '))}</td><td>${pill(t.localization_mode==='bearing_geometry'?'双站角度支撑':t.localization_mode==='predicted_position'?'位置预测':t.is_coasting||t.age_ticks>0?'滑行 / 外推':'位置更新',t.localization_mode==='predicted_position'||t.is_coasting?'warn':'ok')}<br>位置龄期 ${number(t.position_age_s)} s</td><td>${number(t.x)} / ${number(t.y)} / ${number(t.z)}</td><td>${number(t.conf,3)}</td><td>${esc(t.report_timestamp_ms)}</td><td>${esc(t.last_measurement_ms ?? '—')}</td></tr>`).join('') || empty(7,'暂无输出；先检查接收、解析与航迹确认状态');
    $('history').innerHTML = [...history].reverse().map(s=>`<tr><td>${new Date(s.received_at*1000).toLocaleTimeString()}</td><td>${esc(s.sensor_id)} / ${esc(s.track_id)}</td><td>${esc(s.timestamp_ms)}</td><td>${number(s.interval_s,3)} s</td><td>${number(s.r)} m / ${number(s.raw_azimuth_deg ?? s.azimuth_deg ?? (s.az==null?null:s.az*180/Math.PI),2)}°</td><td>${esc(sampleLabel(s))}</td></tr>`).join('') || empty(6,'尚未收到设备目标记录');
    $('raw').textContent=JSON.stringify(d,null,2);
    $('updated').textContent=`更新 ${new Date(d.server_time*1000).toLocaleTimeString()}`;
    draw(history.filter(s=>s.sensor_id?.startsWith('radar') && Number.isFinite(s.x) && Number.isFinite(s.y)));
  }
  function draw(points) {
    const canvas=$('scope'), width=canvas.clientWidth, height=260, scale=window.devicePixelRatio||1;
    canvas.width=width*scale; canvas.height=height*scale;
    const c=canvas.getContext('2d'); c.scale(scale,scale); c.clearRect(0,0,width,height);
    c.strokeStyle='#203440'; c.lineWidth=1;
    for(let x=24;x<width;x+=48){c.beginPath();c.moveTo(x,10);c.lineTo(x,height-20);c.stroke();}
    for(let y=20;y<height;y+=48){c.beginPath();c.moveTo(24,y);c.lineTo(width-24,y);c.stroke();}
    c.fillStyle='#8297a4';c.font='11px monospace';c.fillText('N ↑',width-45,24);c.fillText('E →',width-45,height-12);
    if(!points.length){c.fillText('等待带位置的雷达数据',24,135);return;}
    const xs=points.map(p=>p.x),ys=points.map(p=>p.y),minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys);
    const factor=Math.min((width-88)/Math.max(100,maxX-minX),(height-80)/Math.max(100,maxY-minY));
    points.forEach((p,i)=>{const x=width/2+(p.x-(minX+maxX)/2)*factor,y=height/2-(p.y-(minY+maxY)/2)*factor;c.fillStyle=p.extrap_flag===0?'#f2c574':'#71d9c2';c.globalAlpha=.25+.75*(i+1)/points.length;c.beginPath();c.arc(x,y,i===points.length-1?5:2.5,0,Math.PI*2);c.fill();});
    c.globalAlpha=1;c.fillStyle='#8297a4';c.fillText(`中心 E ${number((minX+maxX)/2)} / N ${number((minY+maxY)/2)}`,24,height-12);
  }
  async function poll(){
    try{
      const response=await fetch('/api/diagnostics',{cache:'no-store',signal:AbortSignal.timeout(4000)});
      if(!response.ok) throw new Error(`HTTP ${response.status}`);
      lastSnapshot=await response.json();
      $('connection').textContent=paused?'已暂停显示':'诊断 API 已连接';$('connection').className='pill '+(paused?'warn':'ok');
      if(!paused) render(lastSnapshot);
    }catch(error){$('connection').textContent='诊断 API 连接失败';$('connection').className='pill bad';$('notice').textContent=`诊断接口不可用：${error.message}。已显示的数据可能陈旧。`; $('notice').classList.remove('good');}
    setTimeout(poll,1000);
  }
  window.addEventListener('resize',()=>{if(lastSnapshot&&!paused)render(lastSnapshot);});
  poll();
})();

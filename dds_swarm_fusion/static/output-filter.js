/* Standalone panel: does not alter the existing situation canvas or WebSocket. */
(() => {
  'use strict';
  const rules = [
    ['points_enabled','min_points','最少有效定位点数','个',1,10000,1,20],
    ['min_speed_enabled','min_speed_mps','最低水平速度','m/s',0,1000,.1,50],
    ['max_speed_enabled','max_speed_mps','最高水平速度','m/s',0,1000,.1,50],
    ['height_enabled','min_height_m','最低相对雷达高度','m',0,30000,1,300],
  ];
  const reasons = {
    existing_output_gate:'原有融合输出条件未通过',too_few_points:'有效定位点数不足',
    points_unavailable:'有效点数未知',speed_unavailable:'速度缺失或无效',
    speed_below_min:'低于最低速度',speed_above_max:'超过最高速度',
    radar_station_unavailable:'参考雷达站址未知',height_unavailable:'目标高度未知',
    height_below_min:'低于最低相对高度',dds_capacity:'超过单包 255 个目标上限',
  };
  const trackBadges = window.SbDdsTrackStatus?.create(document, reasons);
  let latest=null, baseline=null, dirty=false, busy=false, pending=null, loading=false;
  const open = document.createElement('button');
  open.id='of-open'; open.textContent='DDS 输出过滤 / 阈值设置';
  const dialog = document.createElement('dialog'); dialog.id='of-dialog';
  dialog.innerHTML=`<header><h2>DDS 输出过滤</h2><button id="of-close" aria-label="关闭阈值面板">关闭</button></header>
  <div class="of-body">
    <p class="of-help">只筛选正式 FusionMultiTarget。全部内部航迹、态势主画面和外部 WebSocket 不删除、不改协议。</p>
    <div id="of-message" role="status" class="of-banner">正在读取融合节点…</div>
    <div id="of-runtime" class="of-help"></div>
    <div id="of-policy" class="of-banner">等待融合节点确认过滤生效时段…</div>
    <label><input id="of-enabled" type="checkbox"> 启用 DDS 输出过滤（关闭后仍保留原有融合输出条件）</label>
    <p class="of-help">下面四项可独立开启或关闭。按钮表示待应用的选择；点击“应用到当前运行”临时生效，或点击“保存配置并应用”同时保存开关和阈值，下次启动从配置读取。</p>
    <div class="of-grid">${rules.map(([flag,key,title,unit,min,max,step,rangeMax])=>`
      <section class="of-rule"><div class="of-rule-head"><span id="of-title-${flag}">${title}</span>
      <button type="button" role="switch" aria-checked="false" aria-labelledby="of-title-${flag} of-switch-text-${flag}" class="of-toggle" id="of-${flag}" disabled><span class="of-toggle-track" aria-hidden="true"></span><span id="of-switch-text-${flag}">待读取</span></button></div>
      <div class="of-rule-state of-help" id="of-state-${flag}" aria-live="polite">等待读取当前生效和已保存状态…</div>
      <div class="of-inputs"><input type="range" id="of-range-${key}" min="${min}" max="${rangeMax}" step="${step}" aria-label="${title}滑块">
      <input type="number" id="of-${key}" min="${min}" max="${max}" step="${step}" aria-label="${title}"><span>${unit}</span></div></section>`).join('')}</div>
    <div class="of-inline"><label for="of-reference">参考雷达</label><input type="text" id="of-reference" maxlength="80" placeholder="radar_3152"></div>
    <p class="of-help">高度＝融合目标 ENU 高度－参考雷达生效站址 ENU 高度；不是绝对海拔。在筛选期内，站址/数值缺失时，启用的对应规则不放行。低速、低空真实无人机也可能在筛选期内被过滤。</p>
    <p class="of-help">点数按本航迹实际接受的新定位时刻累计；重复报文、心跳、预测、单站纯测向不计数，同一时刻多源只计一次。滑块之外的数值可手动输入。</p>
    <div id="of-edit" class="of-help"></div>
    <div class="of-actions"><button id="of-apply" class="of-primary">应用到当前运行</button><button id="of-save">保存配置并应用</button><button id="of-restore">载入已保存值</button><button id="of-refresh">刷新生效值</button></div>
    <div id="of-reference-state" class="of-help"></div>
    <h3>正式 DDS 发布预览</h3><div id="of-counts" class="of-banner">等待输出批次</div>
    <div class="of-inline"><label for="of-view">列表</label><select id="of-view"><option value="all">全部航迹</option><option value="passed">仅 DDS 放行</option><option value="blocked">仅被过滤</option></select><span class="of-help">最多显示最近批次 100 条；此处“放行”不等于下游接收确认。</span></div>
    <div class="of-table"><table><thead><tr><th>航迹</th><th>航迹年龄 / 筛选期</th><th>点数</th><th>速度 m/s</th><th>相对高度 m</th><th>发布状态 / 原因</th></tr></thead><tbody id="of-rows"></tbody></table></div>
  </div>`;
  document.body.append(open,dialog);
  const el=id=>document.getElementById('of-'+id);
  const say=(message,warn=false)=>{el('message').textContent=message;el('message').classList.toggle('of-warn',warn);};
  const number=v=>v===null||v===undefined?'—':Number(v).toFixed(2);
  const ageNumber=v=>Number.isFinite(v)?v.toFixed(3):'—';
  const phase=r=>({initial:'前9秒筛选中',complete:'筛选期已结束',age_unknown:'年龄未知，按筛选期检查',disabled:'总开关已关闭'}[r.filter_phase]||'旧节点：全程过滤');
  const summary=s=>`${s.enabled?'开启':'关闭'}；点数 ${s.points_enabled?'≥'+s.min_points:'不限'}；速度 ${s.min_speed_enabled?s.min_speed_mps:'不限'}～${s.max_speed_enabled?s.max_speed_mps:'不限'} m/s；相对高度 ${s.height_enabled?'≥'+s.min_height_m+' m':'不限'}`;
  const checked=flag=>el(flag).getAttribute('aria-checked')==='true';
  const setChecked=(flag,value)=>el(flag).setAttribute('aria-checked',String(value));
  function editing(){dirty=true;el('edit').textContent='有待应用修改：尚未改变正在发送的 DDS。';}
  function buttons(){
    const ready=latest&&latest.alive&&latest.available;
    const locked=!ready||busy||!!pending;
    for(const name of ['apply','save'])el(name).disabled=!ready||busy||!!pending;
    el('save').disabled=el('save').disabled||!!latest?.state?.config_error;
    el('restore').disabled=!latest?.state?.saved_settings||busy||!!pending;
    el('refresh').disabled=busy;
    el('enabled').disabled=el('reference').disabled=locked;
    for(const [flag,key] of rules){
      const on=checked(flag),s=latest?.state;
      el(flag).disabled=locked;
      el(key).disabled=el('range-'+key).disabled=locked||!el('enabled').checked||!on;
      el('switch-text-'+flag).textContent=s?(on?'过滤开启':'过滤关闭'):'待读取';
      el(flag).title=on?'点击关闭此项过滤；应用或保存后生效':'点击开启此项过滤；应用或保存后生效';
      if(s){
        const active=ready?(s.settings.enabled?(s.settings[flag]?'开启':'关闭'):'关闭（总开关关闭）'):'未连接，待确认';
        const saved=s.config_error?'读取异常':!s.saved_settings?'未知':s.saved_settings[flag]?'开启':'关闭';
        const savedMaster=s.saved_settings&&!s.saved_settings.enabled?'；保存的总开关关闭':'';
        el('state-'+flag).textContent=`当前生效：${active}｜配置保存：${saved}${savedMaster}${on!==s.settings[flag]?'｜开关修改未应用':''}`;
      }
    }
  }
  function fill(s){
    el('enabled').checked=s.enabled; el('reference').value=s.reference_radar_id;
    for(const [flag,key,,,,,,rangeMax]of rules){
      setChecked(flag,s[flag]);el(key).value=s[key];
      el('range-'+key).max=Math.max(rangeMax,s[key]);el('range-'+key).value=s[key];
    }
    buttons();
  }
  function read(){
    const s={enabled:el('enabled').checked,reference_radar_id:el('reference').value.trim()};
    for(const [flag,key,title,,min,max]of rules){
      const field=el(key),v=Number(field.value);
      if(!field.value.trim()||!Number.isFinite(v)||v<min||v>max||(key==='min_points'&&!Number.isInteger(v)))throw Error(`${title}输入无效，允许 ${min}～${max}`);
      s[flag]=checked(flag);s[key]=v;
    }
    if(s.min_speed_enabled&&s.max_speed_enabled&&s.min_speed_mps>s.max_speed_mps)throw Error('同时开启速度上下限时，最低速度不能大于最高速度');
    if(!/^radar(?:_[A-Za-z0-9-]+)?$/.test(s.reference_radar_id))throw Error('请填写有效雷达设备标识，例如 radar_3152');
    return s;
  }
  function rows(){
    const s=latest?.state;if(!s)return;
    const list=(s.decisions||[]).filter(r=>el('view').value==='all'||(r.passed?'passed':'blocked')===el('view').value);
    el('rows').replaceChildren();
    for(const r of list){
      const tr=document.createElement('tr');tr.className=r.passed?'of-pass':'of-blocked';
      for(const value of ['F'+r.id,ageNumber(r.track_age_s)+' s / '+phase(r),r.points??'—',number(r.speed_mps),number(r.height_above_radar_m),r.passed?'放行':r.reasons.map(x=>reasons[x]||x).join('；')]){
        const td=document.createElement('td');td.textContent=value;tr.append(td);
      }el('rows').append(tr);
    }
    const b=s.batch;
    el('counts').textContent=b?`内部 ${b.total} 条 / 原有输出候选 ${b.eligible} 条 / 放行 ${b.passed} 条 / 未发送 ${b.blocked} 条。${b.publish_ok?'发布接口返回成功':'未确认发布成功'}${b.publish_error?'：'+b.publish_error:''}\n数据时间：${b.timestamp_ms?new Date(b.timestamp_ms).toLocaleString():'未知'}；批次参数版本 ${b.revision}${b.revision!==s.revision?'（参数已更新，等待下一输出批次）':''}`:'等待新的输出批次（没有目标报文时仍可调整参数）';
    const ref=s.reference;
    el('reference-state').textContent=ref?`参考雷达 ${ref.sensor_id}：${ref.known?'站址 ENU 高度 '+number(ref.station_z_enu_m)+' m；来源 '+(ref.station_source||'未标明'):'尚无有效站址；筛选期内启用高度条件时不会放行'}。`:'参考雷达站址将在处理数据后显示。';
  }
  function accept(data){
    latest=data;
    trackBadges?.update(data);
    const s=data.state;
    if(s){
      const initialOnly=s.filter_window_policy==='track_initial'&&s.filter_window_s===9;
      el('policy').textContent=initialOnly?'生效时段：点数、最低速度、最高速度、相对雷达高度，仅在融合航迹建立后的前 9 秒检查；满 9 秒后不再因这四项拦截。原有融合输出条件仍保留。低速、低空的非无人机航迹存续满 9 秒后也可能输出。':'当前融合节点仍为旧版全程过滤；新规则需重启融合服务后生效，不能只刷新网页。';
      el('policy').classList.toggle('of-warn',!initialOnly);
      el('runtime').textContent=`运行版本 ${s.revision}｜${s.unsaved?'临时值，未保存':'与保存值一致'}｜${s.config_path}\n当前生效：${summary(s.settings)}`;
      if(!dirty&&!busy&&!pending){fill(s.settings);baseline={instance_id:s.instance_id,revision:s.revision,saved_hash:s.saved_hash};}
      if(pending&&s.last_command?.request_id===pending.id){
        const result=s.last_command;pending=null;
        if(result.ok){dirty=false;baseline={instance_id:s.instance_id,revision:s.revision,saved_hash:s.saved_hash};fill(s.settings);el('edit').textContent='';say(result.saved?'融合节点已确认：开关和阈值已生效并保存，下次启动读取此配置。':'融合节点已确认临时生效，未写入配置文件。');}
        else say('未应用：'+result.error,true);
      }else if(pending&&Date.now()>pending.until){pending=null;say('请求确认超时，不能宣称已生效；请刷新生效值核对后再操作。',true);}
      if(s.config_error)say('磁盘配置异常，当前运行值继续使用；请先检查配置文件：'+s.config_error,true);
      rows();
    }
    if(!data.alive||!data.available)say('融合节点未连接或尚未加载新版功能；显示内容可能陈旧，应用按钮已禁用。',true);
    buttons();
  }
  async function refresh(force=false){
    if(loading)return;loading=true;
    try{
      const response=await fetch('/api/output-filter',{cache:'no-store',signal:AbortSignal.timeout(5000)});
      if(!response.ok)throw Error('HTTP '+response.status);
      if(force){dirty=false;pending=null;baseline=null;el('edit').textContent='';}
      const first=!latest;accept(await response.json());
      if((first||force)&&latest.alive&&latest.available)say('已读取当前开关和阈值。编辑后点击“应用到当前运行”，或“保存配置并应用”。');
    }catch(e){if(latest)latest.alive=false;trackBadges?.update(null);say('阈值接口不可用：'+e.message+'。不会自动更改 DDS。',true);buttons();}
    finally{loading=false;}
  }
  async function apply(save){
    if(busy||pending||!baseline)return;
    try{
      const settings=read();busy=true;buttons();say('等待融合节点确认…');
      const response=await fetch('/api/output-filter/apply',{method:'POST',headers:{'Content-Type':'application/json','X-Output-Filter-Token':latest.token},
        body:JSON.stringify({settings,save,instance_id:baseline.instance_id,expected_revision:baseline.revision,expected_saved_hash:baseline.saved_hash}),signal:AbortSignal.timeout(7000)});
      const data=await response.json();
      if(!response.ok)throw Error(data.result?.error||data.detail||'请求失败');
      pending={id:data.request_id||data.result?.request_id,until:Date.now()+7000};
      accept(data);
      if(data.pending)say('请求已发送，但尚未确认生效；正在等待融合节点回执。',true);
    }catch(e){say('操作未确认成功：'+e.message+'。请刷新生效值后核对。',true);}
    finally{busy=false;buttons();}
  }
  for(const [flag,key,,,,,,rangeMax]of rules){
    el('range-'+key).addEventListener('input',()=>{el(key).value=el('range-'+key).value;editing();});
    el(key).addEventListener('input',()=>{const n=Number(el(key).value);if(Number.isFinite(n)){el('range-'+key).max=Math.max(rangeMax,n);el('range-'+key).value=n;}editing();});
    el(flag).addEventListener('click',()=>{if(!el(flag).disabled){setChecked(flag,!checked(flag));editing();buttons();}});
  }
  el('enabled').addEventListener('change',()=>{editing();buttons();});el('reference').addEventListener('input',editing);
  el('view').addEventListener('change',rows);
  el('apply').addEventListener('click',()=>apply(false));el('save').addEventListener('click',()=>apply(true));
  el('restore').addEventListener('click',()=>{if(latest?.state){fill(latest.state.saved_settings);baseline={instance_id:latest.state.instance_id,revision:latest.state.revision,saved_hash:latest.state.saved_hash};editing();say('已载入磁盘保存值，点击“应用到当前运行”后生效。');}});
  el('refresh').addEventListener('click',()=>{if(!dirty||confirm('放弃尚未应用的编辑，重新读取生效值？'))refresh(true);});
  el('close').addEventListener('click',()=>dialog.close());
  open.addEventListener('click',()=>{dialog.showModal();refresh();});
  setInterval(()=>{
    trackBadges?.sync();
    if(dialog.open||pending||trackBadges?.active())refresh();
  },1000);
  buttons();
})();

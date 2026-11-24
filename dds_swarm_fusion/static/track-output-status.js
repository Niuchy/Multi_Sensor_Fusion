/* Read-only badges for the existing React track list. No DDS/config writes. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.SbDdsTrackStatus = api;
})(typeof globalThis === 'object' ? globalThis : this, function () {
  'use strict';
  const waiting = title => ({kind:'pending', label:'待确认', title});
  function decision(data, id, elapsedSeconds=0, reasons={}) {
    if (!data?.alive || !data.available || !data.state)
      return waiting('DDS 过滤状态未连接或已过期，不能确认是否放行');
    const state=data.state, batch=state.batch;
    if (!batch || batch.revision !== state.revision)
      return waiting('等待当前参数版本的下一批 DDS 过滤结果');
    if (!Number.isFinite(data.server_time) || !Number.isFinite(batch.published_at) ||
        !Number.isFinite(elapsedSeconds) || elapsedSeconds < 0)
      return waiting('尚无有效的 DDS 发布批次时间');
    // Compare server wall times, then use local monotonic elapsed time. Replay
    // measurement timestamps may be historical and must NOT determine staleness.
    const age=data.server_time-batch.published_at+elapsedSeconds;
    if (age < -1 || age > 10)
      return waiting('最近 DDS 结果已超过 10 秒未更新，等待新批次');
    const row=(state.decisions || []).find(r => String(r.id) === String(id));
    if (!row) return waiting('当前 DDS 诊断批次没有此航迹，等待同步；诊断最多列出 100 条');
    const life=Number.isFinite(row.track_age_s)?`航迹年龄 ${row.track_age_s.toFixed(3)} 秒；`:'';
    const phase={initial:'前9秒筛选中；',complete:'前9秒筛选期已结束，点数/速度/高度不再拦截；',
      age_unknown:'年龄未知，暂按筛选期检查；',disabled:'阈值总开关已关闭；'}[row.filter_phase]||'';
    const detail=life+phase;
    if (row.passed === false)
      return {kind:'blocked',label:'拦截',title:detail+'未发送到正式 DDS：'+
        ((row.reasons || []).map(x => reasons[x] || x).join('；') || '未通过输出条件')};
    if (row.passed !== true) return waiting('此航迹缺少明确的 DDS 过滤结论');
    if (batch.publish_error)
      return {kind:'error',label:'发送失败',title:detail+'过滤已通过，但 DDS 发布接口报错：'+batch.publish_error};
    if (batch.publish_ok !== true) return waiting('过滤已通过，尚未确认 DDS 发布接口返回成功');
    return {kind:'passed',label:'放行',title:detail+'通过正式 DDS 输出过滤；发布接口返回成功，不代表下游接收确认'};
  }

  function create(doc, reasons={}) {
    let data=null, received=0, queued=false;
    const win=doc.defaultView;
    const now=() => win.performance.now();
    const assign=(node,name,value) => {
      if (node.getAttribute(name) !== value) node.setAttribute(name,value);
    };
    function sync() {
      const rail=doc.querySelector('.rail.right');
      if (!rail) return;
      rail.classList.add('of-dds-labels');
      const header=rail.querySelector('.tlist-h > span:first-child');
      if (header) assign(header,'title','航迹编号后为正式 DDS 放行状态，每秒同步；悬停编号查看原因');
      for (const cell of rail.querySelectorAll('.trow .tid')) {
        const match=cell.textContent.match(/\bF(\d+)\b/);
        if (!match) continue;
        const state=decision(data,match[1],Math.max(0,(now()-received)/1000),reasons);
        // Attributes/CSS only: never insert foreign child nodes into React's
        // managed spans, and never alter the original track ID or click handlers.
        assign(cell,'data-dds-state',state.kind);
        assign(cell,'data-dds-label',state.label);
        assign(cell,'title',state.title);
        assign(cell,'aria-label',cell.textContent.trim()+' · '+state.label);
      }
    }
    const app=doc.getElementById('root');
    if (app && win.MutationObserver) {
      new win.MutationObserver(() => {
        if (queued) return;
        queued=true;
        win.requestAnimationFrame(() => {queued=false;sync();});
      }).observe(app,{subtree:true,childList:true,characterData:true});
    }
    sync();
    return {
      update(value) {data=value;received=now();sync();},
      active() {return !!doc.querySelector('.rail.right .tlist');},
      sync,
    };
  }
  return {decision,create};
});

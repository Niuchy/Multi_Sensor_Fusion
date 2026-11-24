(() => {
  const link = document.createElement('a');
  link.href = '/debug.html';
  link.textContent = '接入诊断 · DDS / 雷达数据 ↗';
  link.target = '_blank';
  link.rel = 'noopener';
  Object.assign(link.style, {position:'fixed', right:'16px', bottom:'148px',
    zIndex:'40', padding:'9px 13px', border:'1px solid #426978', borderRadius:'5px',
    background:'#11242c', color:'#a9ece3', font:'12px system-ui', textDecoration:'none'});
  document.body.appendChild(link);
})();

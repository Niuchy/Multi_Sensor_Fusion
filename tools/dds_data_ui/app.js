'use strict';
const $ = id => document.getElementById(id);
let state = null, online = false, authenticated = false, pending = false, uploading = false, fileSignature = '', logSignature = '';
let libraryDirectory = null, pickerTarget = null, pickerData = null, pickerSequence = 0;
const filePath = (directory, name) => directory && name ? directory.replace(/\/$/,'')+'/'+name : '';
const downloadURL = (directory, name) => '/api/download?'+new URLSearchParams({directory,name});
const names = {RadarTarget:'雷达',ESMTarget:'侦察',EOTarget:'光电',AcousticTarget:'声学'};
const states = {running:'运行中',stopping:'正在停止',completed:'已完成',stopped:'已停止',failed:'失败'};
const phases = {starting:'启动 / 校验中',quiet_guard:'输入静默检查 · 尚未发送',record_started:'正在接收设备报文',replay_progress:'正在发送 DDS',file_checked:'文件校验完成'};
const active = () => state?.job && ['running','stopping'].includes(state.job.state);
const chosen = id => [...$(id).querySelectorAll('input:checked')].map(el=>el.value);
const options = () => ({topics:chosen('replay-topics'),time_mode:$('time-mode').value,speed:Number($('speed').value),allow_incomplete:$('allow-incomplete').checked});
const ticket = () => state?.preflight && state.preflight.path === filePath(libraryDirectory,$('replay-file').value) && JSON.stringify(state.preflight.options) === JSON.stringify(options()) && Date.now()/1000-state.preflight.created < 1800 ? state.preflight : null;
const bytes = n => n == null ? '—' : n < 1024 ? `${n} B` : n < 1048576 ? `${(n/1024).toFixed(1)} KiB` : `${(n/1048576).toFixed(2)} MiB`;
const timestamp = n => new Date(n*1000).toLocaleString('zh-CN',{hour12:false});
function notice(text,kind='error') { $('notice').textContent=text; $('notice').className=kind; $('notice').hidden=!text; }
async function api(path,body) {
  const response = await fetch(path,{method:body===undefined?'GET':'POST',credentials:'same-origin',cache:'no-store',headers:body===undefined?{}:{'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body),signal:AbortSignal.timeout(15000)});
  const value=await response.json();
  if(!response.ok){if(response.status===401) loggedOut();throw Error(value.error||`HTTP ${response.status}`);}
  return value;
}
function loggedOut(){authenticated=false;online=false;$('workspace').hidden=true;$('login-panel').hidden=false;$('logout').hidden=true;$('connection').textContent='未登录';}
function pathPreviews(){
  $('record-path').textContent=filePath($('record-directory').value,$('record-name').value.trim())||'等待连接';
  $('replay-path').textContent=filePath(libraryDirectory,$('replay-file').value)||'尚未选择';
}
function newName(){const d=new Date();$('record-name').value=`capture_${d.getFullYear()}${String(d.getMonth()+1).padStart(2,'0')}${String(d.getDate()).padStart(2,'0')}_${String(d.getHours()).padStart(2,'0')}${String(d.getMinutes()).padStart(2,'0')}${String(d.getSeconds()).padStart(2,'0')}.jsonl`;pathPreviews();}
function controls(){
  const busy=!!active()||pending||uploading, safe=online&&authenticated;
  $('record').disabled=!safe||busy||!state?.native_available;
  $('check').disabled=!safe||busy||!$('replay-file').value;
  $('replay').disabled=!safe||busy||!state?.native_available||!ticket()||!$('isolated').checked;
  $('stop').disabled=!safe||!active()||state?.job?.state==='stopping';
  $('upload').disabled=!safe||busy;
  $('upload-file').disabled=busy;
  $('refresh').disabled=!safe;
  $('library-browse').disabled=!safe||busy;
  for(const el of $('record-form').querySelectorAll('input,button[type=button]')) el.disabled=busy||!safe;
  for(const el of $('replay-form').querySelectorAll('input,select,button[type=button]')) if(el.id!=='replay') el.disabled=busy||!safe;
}
function previewResult(){
  $('time-hint').textContent=$('time-mode').value==='rebase'?'重定位：只平移已知 PXSB / Unit / Frame 的非零时间戳。位置、速度、ID 不变；非零 CRC 或未知格式会拒绝。':'原时间：所有应用报文字节不变。旧时间戳可能被已有融合状态丢弃。';
  if(Number($('speed').value)!==1) $('time-hint').textContent+=' 倍速只改发包节奏，不缩放报文时间差；不等效实装验证。';
  const t=ticket();$('check-result').className='check-result'+(t?' ok':'');
  if(t){const s=t.summary;$('check-result').textContent=`预检查通过 · ${s.selected_records} 包 · ${s.playback_duration_s.toFixed(3)} 秒\n${Object.entries(s.selected_counts).map(([k,v])=>`${k}: ${v}`).join(' / ')}\n${s.complete?'文件有完成标记':'⚠ 文件不完整，已显式启用恢复'} · ${s.completion_reason||'无结束标记'}\n校验后内容改变会拒绝发送；预检查 30 分钟内有效。`;}
  else if(state?.job?.action==='check'&&state.job.state==='completed'&&state.job.path===filePath(libraryDirectory,$('replay-file').value)&&state.job.summary?.selected_records===0){$('check-result').textContent='校验完成，但所选 Topic 为 0 包，不能回放。请检查录制或选择其他 Topic。';}
  else {$('check-result').textContent='尚无当前文件 / 参数的有效预检查。预检查不会加载 DDS 或发送数据。';}
  pathPreviews();controls();
}
function files(){
  const signature=JSON.stringify([state.files,state.file_total,state.file_error,libraryDirectory,!!active(),state.job?.path,pending,uploading]);if(signature===fileSignature)return;fileSignature=signature;
  const previous=$('replay-file').value;
  $('replay-file').replaceChildren(new Option(state.files.length?'请选择文件':'请先录制或上传 .jsonl 文件',''));
  for(const f of state.files)$('replay-file').add(new Option(f.name,f.name));
  if(state.files.some(f=>f.name===previous))$('replay-file').value=previous;
  $('files-body').replaceChildren();
  for(const file of state.files){
    const row=document.createElement('tr');if(file.name===$('replay-file').value)row.className='selected';
    for(const text of [file.name,bytes(file.bytes),timestamp(file.modified)]){const td=document.createElement('td');td.textContent=text;row.append(td);}
    const actions=document.createElement('td'),select=document.createElement('button');select.textContent='选作回放';select.className='quiet';select.disabled=!!active()||pending||uploading;
    select.onclick=()=>{$('replay-file').value=file.name;$('isolated').checked=false;fileSignature='';files();previewResult();$('replay-title').scrollIntoView({behavior:'smooth',block:'center'});};actions.append(select);
    if(!(active()&&state.job.path===filePath(libraryDirectory,file.name))){const link=document.createElement('a');link.textContent='下载到 Mac';link.href=downloadURL(libraryDirectory,file.name);link.download=file.name;actions.append(link);}
    row.append(actions);$('files-body').append(row);
  }
  if(!state.files.length){const row=document.createElement('tr'),td=document.createElement('td');td.colSpan=4;td.textContent='还没有录制文件。开始录制，或从电脑上传已有录制。';row.append(td);$('files-body').append(row);}
  $('files-note').textContent=state.file_error||`共 ${state.file_total} 个文件${state.file_total>500?'，显示最近 500 个':''}。文件列表不等于完整性校验，请先预检查。`;
}
function eventText(event){
  const rest={...event};delete rest.at;delete rest.event;
  const labels={record_started:'录制开始',record_progress:'录制进度',record_finished:'录制结束',file_checked:'文件校验',quiet_guard:'静默检查',replay_progress:'发送进度',replay_stopped:'回放结束',dry_run:'预检查结束 · 未发包',interrupted:'手动停止',error:'错误',warning:'提醒',console:'进程日志'};
  return `${new Date(event.at*1000).toLocaleTimeString('zh-CN',{hour12:false})}  ${labels[event.event]||event.event}\n${JSON.stringify(rest,null,2)}`;
}
function render(){
  if(!libraryDirectory)libraryDirectory=state.directory;
  if(!$('record-directory').value)$('record-directory').value=state.directory;
  $('directory').textContent=libraryDirectory;$('replay-directory').value=libraryDirectory;$('runtime-note').hidden=state.native_available;
  $('runtime-note').textContent=`当前环境：${state.platform}，未发现可用 Linux DDS 环境。可上传、预检查和下载；录制 / 发送请在 DGX 或 RK3588 启动此工具。`;
  files();const job=state.job;
  if(job){
    $('task-title').textContent=({record:'录制',check:'预检查',replay:'DDS 回放'}[job.action])+' · '+job.file;
    $('task-state').textContent=states[job.state]||job.state;
    $('count-label').textContent=job.action==='record'?'已接收报文':job.action==='replay'?'已发送调用 / 计划包数':'校验通过的所选包数';
    const p=job.progress,total=job.summary?.selected_records;
    $('task-count').textContent=job.action==='record'?(p.records??0):job.action==='replay'?`${p.published_calls??0} / ${total??'—'}`:(total??'—');
    $('task-elapsed').textContent=job.elapsed_s.toFixed(1)+' s';
    $('task-size').textContent=job.action==='record'?bytes(job.bytes??p.bytes):'—';
    $('task-progress').value=job.action==='replay'&&total?(p.published_calls??0)/total*100:0;
    $('task-progress').hidden=job.action!=='replay';
    $('task-detail').textContent=job.error||(`${active()?phases[job.phase]||job.phase:states[job.state]}${job.action==='replay'?' · 发送成功不是接收确认，请核对融合诊断。':job.action==='record'?' · 收包计数约每 2 秒更新；若为 0，请检查设备发布与 DDS 网络。':''}`);
    if(!active()&&job.action==='record'&&p.reason)$('task-detail').textContent+=' · 结束原因：'+({duration:'达到设定时长',size_limit:'达到文件大小上限',interrupted:'收到停止请求',error:'录制异常'}[p.reason]||p.reason);
    $('task-detail').textContent+=' · '+job.path;
    $('topic-counts').replaceChildren();for(const [topic,count] of Object.entries(p.counts||job.summary?.selected_counts||{})){const item=document.createElement('span');item.textContent=`${names[topic]||topic} / ${topic}：${count}`;$('topic-counts').append(item);}
    const signature=JSON.stringify(job.events);if(signature!==logSignature){logSignature=signature;const atEnd=$('logs').scrollHeight-$('logs').scrollTop-$('logs').clientHeight<60;$('logs').textContent=job.events.map(eventText).join('\n\n')||'任务启动中…';if(atEnd)$('logs').scrollTop=$('logs').scrollHeight;}
  }
  const recorded=state.last_recording;
  $('record-download').hidden=!recorded;
  if(recorded){
    $('last-record-path').textContent=recorded.path;
    $('last-download').hidden=!recorded.download_available;
    $('last-download').href=downloadURL(recorded.directory,recorded.file);$('last-download').download=recorded.file;
    $('last-record-note').textContent=(recorded.state==='failed'?'录制异常，文件可能不完整。':'')+(recorded.download_available?'下载到当前打开浏览器的电脑；设备原文件保留。Mac 的具体目录由浏览器下载设置或保存对话框决定。':'文件不存在、正在使用或超出大小上限，暂不可下载。');
  }
  previewResult();
}
async function refresh(){
  const requested=libraryDirectory;
  try{const result=await api('/api/state'+(requested?'?'+new URLSearchParams({directory:requested}):''));if(requested!==libraryDirectory)return;state=result;if(!online&&authenticated)notice('工具连接已恢复，请核对当前任务状态。','good');online=true;authenticated=true;$('workspace').hidden=false;$('login-panel').hidden=true;$('logout').hidden=false;$('connection').textContent='工具已连接';$('connection').className='badge teal';render();}
  catch(e){online=false;controls();if(authenticated){$('connection').textContent='连接中断 · 数据可能陈旧';$('connection').className='badge amber';notice(`无法连接工具：${e.message}。页面不能确认任务是否已停止；请检查工具终端。`);}}
}
async function job(body){pending=true;controls();try{await api('/api/jobs',body);notice(body.action==='record'?'录制任务已启动。':body.action==='check'?'预检查已开始，不发送 DDS。':'回放任务已启动；先校验文件并检查输入静默，再发送 DDS。','good');$('isolated').checked=false;await refresh();}catch(e){notice(e.message);}finally{pending=false;controls();}}
$('login-form').onsubmit=async e=>{e.preventDefault();try{await api('/api/login',{key:$('access-key').value});$('access-key').value='';notice('');await refresh();}catch(err){notice(err.message);}};
$('logout').onclick=async()=>{try{await api('/api/logout',{});loggedOut();notice('已退出登录。正在运行的任务不会因此停止。','good');}catch(e){notice(e.message);}};
$('new-name').onclick=newName;
$('record-name').oninput=pathPreviews;
$('record-form').onsubmit=e=>{e.preventDefault();job({action:'record',file:$('record-name').value.trim(),directory:$('record-directory').value,topics:chosen('record-topics'),duration:Number($('duration').value),max_mib:Number($('max-mib').value)});};
$('replay-form').onsubmit=e=>{e.preventDefault();$('isolated').checked=false;job({action:'check',file:$('replay-file').value,directory:libraryDirectory,...options()});};
for(const el of $('replay-form').querySelectorAll('input,select'))el.addEventListener('change',()=>{if(el.id!=='isolated')$('isolated').checked=false;fileSignature='';if(state)files();previewResult();});
$('replay').onclick=()=>{const t=ticket();if(!t||!$('isolated').checked)return;$('confirm-details').textContent=`文件：${t.path}\n${t.summary.selected_records} 包 / ${t.summary.playback_duration_s.toFixed(3)} 秒 / ${t.options.speed} 倍速 / ${t.options.time_mode==='original'?'保留原时间':'重定位时间'}`;$('confirm-dialog').showModal();};
$('cancel-send').onclick=()=>$('confirm-dialog').close();
$('confirm-send').onclick=()=>{const t=ticket();$('confirm-dialog').close();if(!t||!$('isolated').checked)return;job({action:'replay',file:t.file,directory:t.directory,...options(),preflight_id:t.id,inputs_stopped:true,confirm_send:true});};
$('stop').onclick=async()=>{if(!state?.job)return;try{await api('/api/stop',{job_id:state.job.id});notice('已请求停止本工具当前任务，正在等待安全结束。','good');await refresh();}catch(e){notice(e.message);}};
$('refresh').onclick=refresh;
$('upload').onclick=async()=>{
  const f=$('upload-file').files[0];if(!f)return notice('请先选择 .jsonl 文件。');if(!f.name.endsWith('.jsonl')||f.size>512*1024*1024)return notice('需要本工具 .jsonl 录制文件，最大 512 MiB。');
  const destination=libraryDirectory;
  uploading=true;controls();if(state)files();$('upload-status').textContent='正在上传…';
  try{const result=await new Promise((resolve,reject)=>{const xhr=new XMLHttpRequest();xhr.open('POST','/api/upload?'+new URLSearchParams({name:f.name,directory:destination}));xhr.setRequestHeader('Content-Type','application/octet-stream');xhr.timeout=600000;xhr.upload.onprogress=e=>{if(e.lengthComputable)$('upload-status').textContent=e.loaded===e.total?'上传完成，正在校验…':`上传 ${Math.round(e.loaded/e.total*100)}%`;};xhr.onload=()=>{try{const r=JSON.parse(xhr.responseText);if(xhr.status===401)loggedOut();xhr.status>=200&&xhr.status<300?resolve(r):reject(Error(r.error||'上传失败'));}catch(e){reject(e);}};xhr.onerror=()=>reject(Error('上传连接失败'));xhr.ontimeout=()=>reject(Error('上传超时，请刷新列表确认是否已保存'));xhr.send(f);});
    $('upload-status').textContent='已保存';await refresh();$('replay-file').value=result.file;$('isolated').checked=false;fileSignature='';files();previewResult();notice(`已上传 ${result.file}，${result.summary.file_records} 包${result.summary.complete?'':'；注意：文件不完整，回放需显式启用恢复'}。请按当前回放参数预检查。`,'good');
  }catch(e){$('upload-status').textContent='上传失败';notice(e.message);}finally{uploading=false;controls();if(state)files();}
};
function browserError(text){$('browser-error').textContent=text;$('browser-error').hidden=!text;}
async function browse(directory){
  const sequence=++pickerSequence;pickerData=null;browserError('');$('browser-use').disabled=true;$('browser-up').disabled=true;
  $('browser-list').textContent='正在读取设备目录…';
  try{
    const result=await api('/api/browse?'+new URLSearchParams({directory:directory||state.directory}));
    if(sequence!==pickerSequence||!$('file-browser').open)return;
    pickerData=result;$('browser-address').value=result.directory;
    $('browser-roots').replaceChildren(new Option('跳转到允许的位置…',''));
    for(const root of result.roots)$('browser-roots').add(new Option(root,root));
    $('browser-up').disabled=!result.parent;
    $('browser-use').disabled=pickerTarget==='record'&&!result.writable;
    $('browser-info').textContent=`${result.writable?'可写':'只读'} · 可用 ${bytes(result.free_bytes)} · ${result.folder_total} 个文件夹 / ${result.file_total} 个录制文件${result.folder_total>500||result.file_total>500?'（各最多显示 500 项）':''}`;
    $('browser-list').replaceChildren();
    function row(name,detail,click){const button=document.createElement('button');button.type='button';button.className='browser-entry';const label=document.createElement('span'),info=document.createElement('small');label.textContent=name;info.textContent=detail;button.append(label,info);button.onclick=click;$('browser-list').append(button);}
    for(const folder of result.folders)row('文件夹 / '+folder.name,'进入',()=>browse(folder.path));
    if(pickerTarget!=='record')for(const file of result.files)row(file.name,bytes(file.bytes)+' · 选择',()=>selectFile(result.directory,file.name));
    if(!$('browser-list').children.length){const empty=document.createElement('p');empty.className='browser-empty';empty.textContent=pickerTarget==='record'?'没有子文件夹，可以使用当前目录或新建文件夹。':'当前目录没有可选文件或子文件夹。';$('browser-list').append(empty);}
  }catch(e){if(sequence===pickerSequence){browserError(e.message);$('browser-list').replaceChildren();}}
}
function openBrowser(target){
  pickerTarget=target;$('browser-title').textContent=target==='record'?'选择录制保存目录':target==='replay'?'选择录制文件':'选择文件列表目录';
  $('browser-use').textContent=target==='record'?'保存到此目录':'查看此目录的文件';
  $('browser-folder-name').value='';$('file-browser').showModal();browse(target==='record'?$('record-directory').value:libraryDirectory);
}
async function selectFile(directory,name=''){
  $('file-browser').close();pickerSequence++;libraryDirectory=directory;$('replay-file').value='';$('isolated').checked=false;fileSignature='';
  await refresh();if(state?.file_directory===directory){$('replay-file').value=name;fileSignature='';files();previewResult();}
}
$('record-browse').onclick=()=>openBrowser('record');
$('replay-browse').onclick=()=>openBrowser('replay');
$('library-browse').onclick=()=>openBrowser('library');
$('browser-close').onclick=()=>{$('file-browser').close();pickerSequence++;};
$('browser-roots').onchange=()=>{if($('browser-roots').value)browse($('browser-roots').value);};
$('browser-go').onsubmit=e=>{e.preventDefault();browse($('browser-address').value.trim());};
$('browser-up').onclick=()=>{if(pickerData?.parent)browse(pickerData.parent);};
$('browser-use').onclick=()=>{
  if(!pickerData)return;
  if(pickerTarget==='record'){$('record-directory').value=pickerData.directory;$('file-browser').close();pickerSequence++;pathPreviews();}
  else selectFile(pickerData.directory);
};
$('browser-mkdir').onsubmit=async e=>{e.preventDefault();if(!pickerData)return;const button=e.currentTarget.querySelector('button');button.disabled=true;try{const result=await api('/api/mkdir',{directory:pickerData.directory,name:$('browser-folder-name').value});$('browser-folder-name').value='';await browse(result.directory);}catch(err){browserError(err.message);}finally{button.disabled=false;}};
newName();refresh();
(async function poll(){await new Promise(r=>setTimeout(r,1000));if(authenticated)await refresh();poll();})();

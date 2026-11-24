// Dependency-free DOM smoke test; no browser connection or DDS publication.
const fs = require('fs'), vm = require('vm'), assert = require('assert'), path = require('path');
const root = path.join(__dirname, '..', 'dds_swarm_fusion', 'static');
const html = fs.readFileSync(path.join(root, 'debug.html'), 'utf8');
const source = fs.readFileSync(path.join(root, 'debug.js'), 'utf8');
async function render(snapshot) {
  const elements = {};
  for (const m of html.matchAll(/id="([^"]+)"/g)) elements[m[1]] = {
    textContent:'',innerHTML:'',className:'',clientWidth:600,
    classList:{toggle(){},remove(){}},
    getContext:()=>new Proxy({}, {get:()=>()=>{}})
  };
  const context = {document:{getElementById:id=>elements[id]},
    window:{devicePixelRatio:1,addEventListener(){}},
    fetch:async()=>({ok:true,json:async()=>snapshot}),
    setTimeout(){}, AbortSignal, console};
  vm.runInNewContext(source,context);
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(elements.connection.textContent,'诊断 API 已连接');
  return elements;
}
(async()=>{
  let e = await render({nodes:{},gateway:{},server_time:1000});
  assert(e['eo-status'].innerHTML.includes('等待光电报文'));
  assert(e['device-stations'].innerHTML.includes('等待设备报文'));
  e = await render({gateway:{},server_time:1000,nodes:{
    input:{samples:[{sensor_id:'eo_201',track_id:7,source_track_id:7,mode:'bearing',
      station_source:'configured_enu',quality_reasons:['range_validity_unconfirmed','<script>'],fresh:true}]},
    fusion:{tracks:[{id:4,source_bindings:{eo_201:7},localization_mode:'predicted_position',position_age_s:3.2}],
      geometry_events:[{devices:['eo_201','eo_202'],reason:'baseline_too_short'}]}
  }});
  assert(e['eo-status'].innerHTML.includes('纯角度'));
  assert(e['eo-status'].innerHTML.includes('测距有效性未确认'));
  assert(e['eo-status'].innerHTML.includes('&lt;script&gt;'));
  assert(e['eo-status'].innerHTML.includes('<td>4</td>'));
  assert(e['eo-geometry'].textContent.includes('站间基线过短'));
  assert(e.tracks.innerHTML.includes('位置预测'));
  e = await render({gateway:{},server_time:1000,nodes:{input:{device_stations:[
    {sensor_id:'eo_1112',station_source:'reported_lla',station_reason:'valid_device_gps',
     station_lla:[120.1,30.2,23.45],station_enu:[1,2,3],station_reference_lla:[120,30,0],last_received_at:999},
    {sensor_id:'rf_df_1113',station_source:'configured_enu',station_reason:'protocol_has_no_device_gps',
     station_lla:[119.999,30,8],station_config:'config/<script>',last_received_at:999}
  ]}}});
  assert(e['device-stations'].innerHTML.includes('报文设备自身 GPS'));
  assert(e['device-stations'].innerHTML.includes('120.1000000 / 30.2000000'));
  assert(e['device-stations'].innerHTML.includes('23.45'));
  assert(e['device-stations'].innerHTML.includes('config ENU 换算经纬高'));
  assert(e['device-stations'].innerHTML.includes('当前协议无设备自身 GPS 字段'));
  assert(e['device-stations'].innerHTML.includes('config/&lt;script&gt;'));
  const rfZero = {sensor_id:'rf_df_4001',track_id:0,received_at:999,
    raw_azimuth_deg:0,az:null,angle_valid:false,fresh:false,
    quality_reasons:['rf_zero_azimuth_placeholder']};
  e = await render({gateway:{},server_time:1000,nodes:{input:{
    rf_zero_azimuth_filtered:3301,samples:[rfZero],recent_samples:[rfZero]
  }}});
  assert(e.notice.textContent.includes('射频 0° 占位已过滤 3301 条'));
  assert(e.samples.innerHTML.includes('0° 占位 · 无测向量测'));
  assert(e.samples.innerHTML.includes('0.00°'));
  assert(e.history.innerHTML.includes('0° 占位 · 无测向量测'));
  e = await render({gateway:{},server_time:1000,nodes:{input:{
    samples:[{sensor_id:'rf_df_4001',track_id:231,az:1,raw_azimuth_deg:57.3,angle_valid:true}]
  }}});
  assert(!e.samples.innerHTML.includes('0° 占位'));
  assert(e.samples.innerHTML.includes('57.30°'));
  console.log('debug UI smoke: OK (empty, EO, bindings, quality, geometry, device GPS sources, RF placeholders, escaping)');
})().catch(error=>{console.error(error);process.exitCode=1;});

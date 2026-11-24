// Optional real-browser smoke test. Uses a separately started LOCAL test UI;
// never clicks publish. Requires playwright / playwright-core with Chrome.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const assert=require('node:assert/strict');
const path=require('node:path');
const os=require('node:os');
const url=process.env.FIELD_SIM_TEST_URL||'http://127.0.0.1:18011/';
const key=process.env.FIELD_SIM_TEST_KEY;
if(!key)throw Error('Set FIELD_SIM_TEST_KEY to the temporary test server key');
if(!['127.0.0.1','localhost'].includes(new URL(url).hostname))throw Error('Browser smoke test is restricted to a local temporary server');
(async()=>{
 const browser=await chromium.launch({headless:true,...(process.env.CHROME_PATH?{executablePath:process.env.CHROME_PATH}:{})});
 try{
  const context=await browser.newContext({viewport:{width:1560,height:1100},locale:'zh-CN'});
  const page=await context.newPage(),errors=[];page.on('pageerror',e=>errors.push(String(e)));
  await page.goto(url);await page.locator('#accessKey').fill(key);await page.locator('#loginForm button').click();
  await page.locator('#eo1Id').waitFor();
  await page.locator('#recipe').selectOption('dual-bearing');await page.locator('#duration').fill('15');
  await page.locator('#targetCount').selectOption('3');
  await page.locator('#eo1Target').selectOption('2');await page.locator('#eo2Target').selectOption('2');
  assert.equal(await page.locator('#eoCards .device').count(),2);
  assert(await page.locator('#configForm').evaluate(form=>form.checkValidity()));
  await page.locator('#generate').click();
  await page.waitForFunction(()=>document.querySelector('#planned').textContent!=='—'&&!document.querySelector('#evaluate').disabled);
  const first=await page.locator('#runs').inputValue();
  for(const id of ['71112','71113'])assert((await page.locator('#devicePackets').textContent()).includes(id));
  await page.locator('#evaluate').click();await page.locator('#evaluation').waitFor({state:'visible'});
  assert.match(await page.locator('#evaluation').textContent(),/通过/);
  assert.match(await page.locator('#evaluation').textContent(),/综合 F1/);
  assert.equal(await page.locator('#evaluation tbody tr').count(),3);
  assert.match(await page.locator('#evaluation tbody tr').first().textContent(),/0.0%/);
  await page.locator('#preflight').click();
  await page.waitForFunction(()=>document.querySelector('#phase').textContent==='已完成'&&!document.querySelector('#preflight').disabled);
  assert(await page.locator('#publish').isDisabled());
  await page.locator('#eo2X').fill('450');assert(await page.locator('#preflight').isDisabled());
  await page.locator('#removeEO').click();assert.equal(await page.locator('#eoCards .device').count(),1);
  await page.locator('#targetCount').selectOption('1');assert.equal(await page.locator('#eo1Target').inputValue(),'1');
  await page.locator('#generate').click();
  await page.waitForFunction(old=>document.querySelector('#runs').value&&document.querySelector('#runs').value!==old&&!document.querySelector('#preflight').disabled,first);
  assert(!(await page.locator('#devicePackets').textContent()).includes('71113'));
  await page.locator('#runs').selectOption(first);
  await page.waitForFunction(old=>document.querySelector('#runs').value===old&&document.querySelector('#eo2Id'),first);
  await page.waitForTimeout(2800);assert.equal(await page.locator('#runs').inputValue(),first);
  assert.equal(await page.locator('#targetCount').inputValue(),'3');assert.equal(await page.locator('#eo2Target').inputValue(),'2');
  await page.screenshot({path:path.join(os.tmpdir(),'field-sim-ui-desktop.png'),fullPage:true});
  await page.setViewportSize({width:390,height:844});
  await page.screenshot({path:path.join(os.tmpdir(),'field-sim-ui-mobile.png'),fullPage:true});
  assert(await page.evaluate(()=>document.body.scrollWidth<=innerWidth),'horizontal overflow');
  // Static files may update before an operator restarts the HTTP process.
  // Refuse generation/sending, but retain stop for that process's owned job.
  await page.route('**/api/state',route=>route.fulfill({json:{api_version:1,job:{id:'owned-mock',state:'running'}}}));
  await page.waitForFunction(()=>document.querySelector('#notice').textContent.includes('重新运行 field_sim_web.sh'));
  assert(await page.locator('#generate').isDisabled());assert(await page.locator('#publish').isDisabled());
  assert(!(await page.locator('#stop').isDisabled()));let stopped=false;
  await page.route('**/api/stop',route=>{assert.equal(route.request().postDataJSON().job_id,'owned-mock');stopped=true;return route.fulfill({json:{ok:true}});});
  await page.locator('#stop').click();await page.waitForTimeout(200);assert(stopped);
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({browser_smoke_pass:true,dds_packets_sent:0,page_errors:errors}));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});

/* Mock serial data; exported artifacts are software-test fixtures, not measurements. */
const {chromium}=require('playwright');
const {pathToFileURL}=require('url'),fs=require('fs'),path=require('path'),assert=require('assert');
(async()=>{
 const out=process.argv[2];fs.mkdirSync(out,{recursive:true});
 const browser=await chromium.launch({channel:'msedge',headless:true});
 try{
 const context=await browser.newContext({viewport:{width:1440,height:1100}}),page=await context.newPage(),errors=[];
 page.on('pageerror',e=>errors.push(e.message));
 await page.addInitScript(()=>{
  window.sent=[];window.ambiguous=false;window.noCoordinates=false;window.liveInput=true;
  const crc=s=>{let c=65535;for(const ch of s){c^=ch.charCodeAt(0)<<8;for(let i=0;i<8;i++)c=((c&32768)?(c<<1)^0x1021:c<<1)&65535;}return c;};
  let controller,timer,tick=0,frame=0,n=0;const enc=new TextEncoder();
  const fake={async open(){
   this.readable=new ReadableStream({start(c){controller=c;}});
   this.writable=new WritableStream({write(b){sent.push(new TextDecoder().decode(b));}});
   timer=setInterval(()=>{
    if(!liveInput)return;tick+=100;n++;if(n%2)frame++;
    const pose=[2850,800,0],obj=noCoordinates?[1,-1,null,0]:[1,600,200,0],objects=ambiguous?[obj,obj]:[obj];
    const a=['V7',tick,1,100,0,0,0,frame,50,0,1,0,objects,50,[frame,50,1,1,[]],[0,0,0,0,0],[pose,1,pose,objects.map(()=>null),pose,[0,0,0]],[null,0,null,null,null],[1,0,3700,1]];
    const s=JSON.stringify(a);controller.enqueue(enc.encode(s+'*'+crc(s).toString(16).padStart(4,'0')+'\n'));
   },100);
  },async close(){clearInterval(timer);}};
  Object.defineProperty(navigator,'serial',{value:{requestPort:async()=>fake}});
 });
 await page.goto(pathToFileURL(path.resolve('tools/vision_xy.html')).href);
 await page.locator('#connect').click();await page.waitForTimeout(350);
 await page.locator('#sample-open').click();assert(await page.locator('#sample-start').evaluate(el=>el.getBoundingClientRect().top<innerHeight));
 await page.locator('#sample-start').click();assert((await page.locator('#sample-progress').innerText()).includes('测量原点'));
 await page.locator('#sample-reference').fill('SOFTWARE TEST ONLY: ground mark -> facing contact edge');
 await page.locator('#sample-seconds').fill('5');await page.locator('#sample-start').click();
 await page.waitForTimeout(1300);assert(await page.locator('#remote-key1').isDisabled());assert(await page.locator('#remote-key4').isEnabled());
 await page.locator('#field-hit').click({position:{x:100,y:100}});assert.equal((await page.evaluate(()=>sent)).length,0);
 await page.locator('#remote-key4').click();assert((await page.evaluate(()=>sent.at(-1))).includes(',K,4*'));
 await page.evaluate(()=>ambiguous=true);await page.waitForTimeout(500);
 await page.evaluate(()=>{ambiguous=false;noCoordinates=true;});await page.waitForTimeout(450);
 await page.evaluate(()=>noCoordinates=false);await page.waitForTimeout(4200);
 assert(await page.locator('#sample-start').isEnabled());
 assert((await page.locator('#sample-results').innerText()).includes('10.00 / 0.00'));
 assert((await page.locator('#sample-results').innerText()).includes('2.00 / 0.00'));
 assert((await page.locator('#sample-progress').innerText()).includes('重复视觉帧'));
 assert((await page.locator('#sample-progress').innerText()).includes('多个目标'));
 assert((await page.locator('#sample-progress').innerText()).includes('缺少距离'));
 assert.equal((await page.evaluate(()=>sent)).length,1); // only the explicitly clicked KEY4
 let download=page.waitForEvent('download');await page.locator('#sample-export').click();let file=await download;await file.saveAs(path.join(out,'synthetic_samples.json'));
 const data=JSON.parse(fs.readFileSync(path.join(out,'synthetic_samples.json'),'utf8'));
 assert.equal(data.groups.length,1);const g=data.groups[0];assert(g.samples.length>5);assert.equal(new Set(g.samples.map(s=>s.frame)).size,g.samples.length);
 assert.equal(g.summary.range_cm.bias,10);assert.equal(g.status,'完成');
 download=page.waitForEvent('download');await page.locator('#sample-csv').click();file=await download;await file.saveAs(path.join(out,'synthetic_samples.csv'));
 assert.equal(fs.readFileSync(path.join(out,'synthetic_samples.csv'),'utf8').trim().split('\r\n').length,g.samples.length+1);
 await page.locator('#sample-panel').screenshot({path:path.join(out,'sampling_desktop.png')});
 await page.reload();assert((await page.locator('#sample-storage').innerText()).includes('恢复 1 组'));
 assert((await page.locator('#sample-results').innerText()).includes('10.00 / 0.00'));assert(await page.locator('#sample-start').isDisabled());
 await page.locator('#demo').click();await page.waitForTimeout(400);assert(await page.locator('#sample-start').isDisabled());assert.equal(await page.locator('#sample-results tr').count(),1);
 await page.locator('#demo').click();await page.locator('#connect').click();await page.waitForTimeout(350);
 await page.locator('#sample-reference').fill('SOFTWARE TEST ONLY: reconnect');await page.locator('#sample-start').click();await page.waitForTimeout(1300);
 await page.locator('#connect').click();assert((await page.locator('#sample-results').innerText()).includes('已结束'));
 assert.equal(await page.locator('#sample-results tr').count(),2);
 await page.setViewportSize({width:390,height:844});await page.locator('#sample-panel').scrollIntoViewIfNeeded();await page.waitForTimeout(100);
 assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));
 await page.locator('#sample-panel').screenshot({path:path.join(out,'sampling_mobile.png')});
 await page.locator('#sample-undo').click();assert.equal(await page.locator('#sample-results tr').count(),1);
 assert.deepEqual(errors,[]);
 console.log('PASS: sampling UI real-mode gating, no sampling-generated writes, motion-click isolation with explicit KEY4 retained, timer, unique frames, raw errors, rejected-frame counters, JSON/CSV export, persisted cache, demo isolation, disconnect ends group and mobile layout');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1);});

const {chromium}=require('playwright');
const {pathToFileURL}=require('url'),path=require('path'),assert=require('assert');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true});
 const page=await browser.newPage({viewport:{width:1440,height:1100}}),errors=[];
 page.on('pageerror',e=>errors.push(e.message));
 await page.addInitScript(()=>{
  const crc=s=>{let c=65535;for(const ch of s){c^=ch.charCodeAt(0)<<8;for(let i=0;i<8;i++)c=((c&32768)?(c<<1)^0x1021:c<<1)&65535;}return c;};
  let c,timer,tick=0,observations=0,seen=0;const h=[[],[]],enc=new TextEncoder();
  window.sent=[];window.visibleObjects=true;window.memoryOn=true;window.mapEpoch=1;window.mapCount=2;
  const emit=p=>{const s=JSON.stringify(p);c.enqueue(enc.encode(s+'*'+crc(s).toString(16).padStart(4,'0')+'\n'));};
  const telemetry=()=>{
   tick+=150;
   const visible=window.visibleObjects&&window.mapCount>0;
   if(visible){seen=tick-50;observations++;for(let i=0;i<2;i++){h[i].push([seen,1200+i*450+observations%3,1800+i*250,1]);if(h[i].length>8)h[i].shift();}}
   const objects=visible?[[1,1000,0,0],[2,1200,2000,0]]:[];
   const points=visible?[[1200,1800,1,90],[1650,2050,2,100]]:[];
   emit(['V7',tick,1,100,0,0,0,tick/150,50,visible?0:1,1,0,objects,50,[tick/150,50,1,1,[]],[0,0,0,0,0],[[2850,800,0],visible?1:0,visible?[2850,800,0]:null,points,[2850,800,0],[0,0,0]],[null,0,null,null,null],[1,0,3700,1]]);
   if(window.memoryOn){
    if(!window.mapCount){emit(['M1',tick,window.mapEpoch,0,0,0,null]);return;}
    for(let i=0;i<2;i++)emit(['M1',tick,window.mapEpoch,observations+1,2,0,[i+1,i+1,1200+i*450,1800+i*250,90,seen,seen,observations,visible?1:0,0,observations,h[i]]]);
   }
  };
  const fake={readable:null,writable:null,async open(){
   this.readable=new ReadableStream({start(controller){c=controller;}});
   this.writable=new WritableStream({write(bytes){const text=new TextDecoder().decode(bytes);window.sent.push(text);if(text!=='MAP\n')throw Error('History UI sent a motion command');}});
   timer=setInterval(telemetry,150);
  },async close(){clearInterval(timer);}};
  Object.defineProperty(navigator,'serial',{value:{requestPort:async()=>fake}});
 });
 await page.goto(pathToFileURL(path.resolve('tools/vision_xy.html')).href);
 await page.locator('#connect').click();await page.waitForTimeout(1500);
 assert((await page.locator('#map-sync').innerText()).includes('2 / 2'));
 assert.equal(await page.locator('.field-target').count(),2);
 assert.equal(await page.locator('#map-history-body tr').count(),8);
 assert((await page.locator('#map-detail').innerText()).includes('已观测'));
 assert.equal(await page.locator('#map-select option').count(),2);
 assert((await page.evaluate(()=>sent)).every(s=>s==='MAP\n'));
 const before=await page.evaluate(()=>sent.length);
 await page.locator('#map-select').selectOption('2');await page.locator('#field-circles').check();await page.waitForTimeout(180);
 assert((await page.locator('#map-detail').innerText()).includes('#2 核心物资'));
 assert.equal(await page.locator('.match-circle').count(),2);assert.equal(await page.locator('.object-history').count(),1);
 assert.equal(await page.evaluate(()=>sent.length),before); // viewing history cannot move the vehicle
 await page.locator('#field-layout').scrollIntoViewIfNeeded();await page.screenshot({path:path.join(process.argv[2],'map_desktop.png')});
 await page.locator('[aria-label="物块地图历史"]').screenshot({path:path.join(process.argv[2],'map_history.png')});
 await page.evaluate(()=>{visibleObjects=false;});await page.waitForTimeout(1900);
 assert.equal(await page.locator('.field-target').count(),2);
 assert((await page.locator('#map-detail').innerText()).includes('历史位置'));
 assert.equal(await page.locator('.live-observation').count(),0);
 await page.evaluate(()=>{memoryOn=false;});await page.waitForTimeout(1800);
 assert.equal(await page.locator('.field-target').count(),2);
 await page.locator('#map-sync-button').click();await page.waitForTimeout(80);assert((await page.evaluate(()=>sent.at(-1)))==='MAP\n');
 await page.locator('#connect').click();await page.waitForTimeout(120);
 assert.equal(await page.locator('.field-target').count(),2);
 assert((await page.locator('#map-sync').innerText()).includes('断开'));
 const downloadPromise=page.waitForEvent('download');await page.locator('#map-export').click();const download=await downloadPromise;await download.saveAs(path.join(process.argv[2],'map_export.json'));
 await page.evaluate(()=>{visibleObjects=true;memoryOn=true;});await page.locator('#connect').click();await page.waitForTimeout(500);
 assert.equal(await page.locator('.field-target').count(),2);
 await page.setViewportSize({width:390,height:844});await page.locator('[aria-label="物块地图历史"]').scrollIntoViewIfNeeded();await page.waitForTimeout(100);
 assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));
 await page.screenshot({path:path.join(process.argv[2],'map_mobile.png')});
 await page.evaluate(()=>{mapEpoch++;mapCount=0;});await page.waitForTimeout(400);
 assert.equal(await page.locator('.field-target').count(),0);assert.equal(await page.locator('#map-history-body tr').count(),0);
 assert((await page.locator('#map-sync').innerText()).includes('0 / 0'));
 await page.locator('#connect').click();assert.deepEqual(errors,[]);
 await browser.close();console.log('PASS: browser map/history, category colours, selection without motion, circles/history controls, empty/stale/serial-disconnect retention, resync, JSON export, map reset and mobile layout');
})().catch(e=>{console.error(e);process.exit(1);});

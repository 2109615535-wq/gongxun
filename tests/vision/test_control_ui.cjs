const {chromium}=require('playwright');
const {pathToFileURL}=require('url');const path=require('path'),assert=require('assert');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true});
 const page=await browser.newPage({viewport:{width:1440,height:1200}});const errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.addInitScript(()=>{
  const crc=s=>{let c=65535;for(const ch of s){c^=ch.charCodeAt(0)<<8;for(let i=0;i<8;i++)c=((c&32768)?(c<<1)^0x1021:c<<1)&65535;}return c;};
  let controller,timer,tick=0,step=0,ctrl=[null,0,null,null,null];
  const enc=new TextEncoder();window.sent=[];window.oldFirmware=false;window.dropConfirm=false;window.pauseTelemetry=false;window.imuState=[1,0,3700,1];window.robotYaw=9000;
  const emit=s=>controller.enqueue(enc.encode(s+'*'+crc(s).toString(16).padStart(4,'0')+'\n'));
  const telemetry=()=>{if(window.pauseTelemetry)return;tick+=150;const p=['V7',tick,1,20,0,0,0,tick/150,40,0,1,0,[],50,[tick/150,40,1,1,[]],[step,0,0,0,0],[[2850,800,window.robotYaw],1,[2850,800,0],[],[2850,800,0],[0,0,0]],ctrl,window.imuState];if(window.oldFirmware){p[0]='V5';p.pop();p.pop();}emit(JSON.stringify(p));};
  const fake={readable:null,writable:null,async open(){this.readable=new ReadableStream({start(c){controller=c;}});this.writable=new WritableStream({write(bytes){const text=new TextDecoder().decode(bytes);window.sent.push(text);assertFrame(text);const a=text.split('*')[0].split(','),id=Number(a[1]);let status=0;if(a[2]==='G'){step=50;if(!window.dropConfirm)ctrl=[id,0,Number(a[3]),Number(a[4]),Number(a[5])/10];}else{const k=Number(a[3]);step=k===1?1:k===4?0:step;status=k===2||k===3?1:0;if(!window.dropConfirm)ctrl=[id,status,null,null,null];}if(!window.dropConfirm)emit('A1,'+id+','+status+','+step);telemetry();}});timer=setInterval(telemetry,150);},async close(){clearInterval(timer);}};
  function assertFrame(s){const [body,tail]=s.trim().split('*');if(crc(body)!==parseInt(tail,16))throw Error('write CRC mismatch');}
  Object.defineProperty(navigator,'serial',{value:{requestPort:async()=>fake}});
 });
 await page.goto(pathToFileURL(path.resolve('tools/vision_xy.html')).href);
 assert(await page.locator('#remote-key1').isDisabled());
 await page.locator('#demo').click();await page.waitForTimeout(180);assert(await page.locator('#remote-key4').isDisabled());
 await page.locator('#demo').click();await page.locator('#connect').click();await page.waitForTimeout(350);
 assert(await page.locator('#remote-key1').isEnabled());
 assert((await page.locator('#field-pose').innerText()).includes('IMU 朝向') && (await page.locator('#field-pose').innerText()).includes('90.0°'));
 assert((await page.locator('#field-imu').innerText()).includes('IMU：正常'));
 await page.evaluate(()=>{imuState=[1,401,3700,0];});await page.waitForTimeout(250);
 assert((await page.locator('#field-imu').innerText()).includes('过期'));
 await page.evaluate(()=>{imuState=[0,null,null,0];});await page.waitForTimeout(250);
 assert((await page.locator('#field-imu').innerText()).includes('等待首帧'));
 await page.evaluate(()=>{imuState=[1,0,3700,1];});await page.waitForTimeout(250);
 async function mapClick(x,y){const xy=await page.locator('#field-svg').evaluate((el,{x,y})=>{const p=el.createSVGPoint();p.x=30+x;p.y=330-y;const q=p.matrixTransform(el.getScreenCTM());return{x:q.x,y:q.y};},{x,y});await page.mouse.click(xy.x,xy.y);await page.waitForTimeout(220);}
 await page.locator('#field-svg').scrollIntoViewIfNeeded();await mapClick(200,180);
 const sentGoal=(await page.evaluate(()=>sent.at(-1))).split('*')[0].split(',');assert.equal(sentGoal[2],'G');assert(Math.abs(Number(sentGoal[3])-2000)<10 && Math.abs(Number(sentGoal[4])-1800)<10);assert.equal(sentGoal[5],'500');
 assert((await page.locator('#control-status').innerText()).includes('主控已确认'));
 assert.equal(await page.locator('#goto-marker').count(),1);assert((await page.locator('#nav-target').innerText()).includes((Number(sentGoal[3])/10).toFixed(1)));
 // Hold mouse down across multiple complete map redraws: one command per release.
 const stableHit=await page.locator('#field-hit').elementHandle();
 for(const hold of [180,350]){
   const before=await page.evaluate(()=>sent.length);
   const point=await page.locator('#field-svg').evaluate(el=>{const p=el.createSVGPoint();p.x=200;p.y=150;const q=p.matrixTransform(el.getScreenCTM());return{x:q.x,y:q.y};});
   await page.mouse.move(point.x,point.y);await page.mouse.down();await page.waitForTimeout(hold);await page.mouse.up();await page.waitForTimeout(180);
   assert.equal(await page.evaluate(()=>sent.length),before+1);
   assert(await stableHit.evaluate(el=>el===document.getElementById('field-hit')));
 }
 await page.screenshot({path:process.argv[2]+'/desktop.png',fullPage:false});
 for(const k of [2,3,1,4]){await page.locator('#remote-key'+k).click();await page.waitForTimeout(80);assert((await page.evaluate(()=>sent.at(-1))).includes(',K,'+k+'*'));}
 assert((await page.locator('#control-status').innerText()).includes('主控已确认'));
 const beforeStopQueue=await page.evaluate(()=>sent.length);
 await page.evaluate(()=>{const svg=document.getElementById('field-svg'),p=svg.createSVGPoint();p.x=150;p.y=150;const q=p.matrixTransform(svg.getScreenCTM());for(let i=0;i<2;i++)svg.dispatchEvent(new MouseEvent('click',{clientX:q.x,clientY:q.y,bubbles:true}));document.getElementById('remote-key4').click();});
 await page.waitForTimeout(180);const queued=await page.evaluate(n=>sent.slice(n),beforeStopQueue);assert.equal(queued.length,1);assert(queued[0].includes(',K,4*'));
 await page.evaluate(()=>{oldFirmware=true;});await page.waitForTimeout(300);assert(await page.locator('#remote-key1').isDisabled());assert(await page.locator('#remote-key4').isEnabled());
 await page.evaluate(()=>{oldFirmware=false;});await page.waitForTimeout(300);
 await page.evaluate(()=>{pauseTelemetry=true;});await page.waitForTimeout(1150);assert(await page.locator('#remote-key1').isDisabled());assert(await page.locator('#remote-key4').isEnabled());
 await page.evaluate(()=>{pauseTelemetry=false;dropConfirm=true;});await page.waitForTimeout(250);
 await page.locator('#remote-key2').click();const count=await page.evaluate(()=>sent.length);await page.waitForTimeout(2200);assert((await page.locator('#control-status').innerText()).includes('未收到确认'));assert.equal(await page.evaluate(()=>sent.length),count);
 await page.evaluate(()=>{dropConfirm=false;});
 await page.setViewportSize({width:390,height:844});await page.locator('#field-svg').scrollIntoViewIfNeeded();await mapClick(100,100);
 const mobileGoal=(await page.evaluate(()=>sent.at(-1))).split('*')[0].split(',');assert.equal(mobileGoal[2],'G');assert(Math.abs(Number(mobileGoal[3])-1000)<15 && Math.abs(Number(mobileGoal[4])-1000)<15);
 assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));
 await page.locator('.remote-controls').scrollIntoViewIfNeeded();await page.screenshot({path:process.argv[2]+'/mobile.png',fullPage:false});
 await page.locator('#connect').click();assert(await page.locator('#remote-key4').isDisabled());
 assert.deepEqual(errors,[]);await browser.close();console.log('PASS: real browser mocked serial, desktop/mobile field clicks, all keys, ACK/target display, old/stale firmware gating, no timeout retries, disconnect, no JS errors');
})().catch(e=>{console.error(e);process.exit(1);});

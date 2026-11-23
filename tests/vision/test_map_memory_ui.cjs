const {chromium}=require('playwright');
const {pathToFileURL}=require('url'),path=require('path'),assert=require('assert');
if(!process.argv.includes('--uart5'))(async()=>{
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

if(process.argv.includes('--uart5'))(async()=>{
 const fs=require('node:fs');
 const browser=await chromium.launch({channel:'msedge',headless:true});
 try{
  const page=await browser.newPage(),errors=[];
  page.on('pageerror',error=>errors.push(error.message));
  await page.addInitScript(()=>{
   const le=(value,n)=>Array.from({length:n},(_,i)=>(value>>>(8*i))&255);
   const u16=value=>le(value,2),u32=value=>le(value,4),pose=(x,y,yaw)=>[...u32(x),...u32(y),...u32(yaw)];
   const crc16=array=>array.reduce((crc,byte)=>{
    crc^=byte<<8;
    for(let i=0;i<8;i++)crc=((crc<<1)^((crc&0x8000)?0x1021:0))&0xffff;
    return crc;
   },65535);
   function cobs(body){
    const output=[0];let slot=0,code=1;
    for(const byte of body){
     if(byte)output.push(byte);
     if(!byte||++code===255){output[slot]=code;slot=output.push(0)-1;code=1;}
    }
    output[slot]=code;return [...output,0];
   }
   let stream,timer,seq=65532,tick=0xfffffee0,epoch=1,gen=0,step=30,reason=0,held=0,frame=90;
   let cameraEnabled=true,clock=0,trackCount=48,rev=7,closed=false;
   const sent=window.sent=[];
   const track=id=>[...u32(epoch),...u32(rev),trackCount,...u32(0),...u32(id),...u32(rev),1,1,
    ...u32(1200+id*20),...u32(1800),...u16(90),...u32(tick-60),...u32(tick-60),...u32(8)];
   const body=(type,payload,at=tick,corrupt=false)=>{
    const raw=[1,type,...u16(seq++),...u16(payload.length),...u32(at),...payload];
    return cobs([...raw,...u16(crc16(raw)^(corrupt?0x2000:0))]);
   };
   const emit=(type,payload,at=tick,corrupt=false)=>{
    if(!closed)stream.enqueue(Uint8Array.from(body(type,payload,at,corrupt)));
   };
   function live(){
    tick=(tick+100)>>>0;
    const cameras=cameraEnabled&&clock++%2===0;
    const live=[step,reason,held,0,...u16(step===0?50:120),cameras?31:7,...pose(2850,695,-9000),...u32(145279),...u16(36000)];
    if(cameras){
     const observed=[...u32(frame++),...u32(40),0,7,3,...pose(2850,690,-9000)];
     const upper=[1,22,...u16(0),...u32(-65535),...u32(65535),...u32(1202),...u32(1801),...u32(1),...u16(90),
      2,3,...u16(65535),...u16(-6000),1,4,...u32(20),...u32(300)];
     const lower=[...u32(frame+20),...u32(45),0,3,1,1,7,6];
     live.push(...observed,...upper,...lower);
    }
    const bytes=body(1,live);
    if(clock<=1){for(const byte of bytes)stream.enqueue(Uint8Array.of(byte));}
    else stream.enqueue(Uint8Array.from(bytes));
   }
   function sync(){
    const syncId=++gen;
    const head=(phase,index)=>[...u16(syncId),phase,index,trackCount,...u32(epoch)];
    emit(11,head(0,0));
    const first=trackCount>0?body(11,[...head(1,0),...track(1)]):[];
    if(first.length)stream.enqueue(Uint8Array.from(first));
    setTimeout(()=>{
     if(closed||gen!==syncId)return;
     const combined=[];
     for(let i=1;i<trackCount;i++){
      const part=body(11,[...head(1,i),...track(i+1)]);
      if(syncId!==1||i!==16)combined.push(...part);
     }
     combined.push(...body(11,head(2,trackCount)));
     stream.enqueue(Uint8Array.from(combined));
    },180);
   }
   window.stopResume=()=>{
    emit(2,[30,6],tick-75);emit(2,[30,0],tick-25);
    emit(2,[30,4],tick-10,true); // corrupt payload must never alter the page.
   };
   window.onlyHeartbeats=()=>{cameraEnabled=false;};
   window.restart=()=>{
    tick=0;seq=0;step=0;reason=0;epoch=1;trackCount=0;rev=0;
    live();
   };
   const fake={async open(){
    this.readable=new ReadableStream({start(controller){stream=controller;}});
    this.writable=new WritableStream({write(bytes){
      const text=new TextDecoder().decode(bytes);sent.push(text);
      if(text==='GET\n'){
       emit(5,[...u32(epoch),...pose(2850,695,0),...pose(0,0,0),...u16(15)]);
       emit(9,[0,0]);
      }else if(text==='MAP\n')sync();
      else if(text.startsWith('HIST,')){
       const match=/^HIST,(\d+),(\d+)\*([0-9A-F]{4})\r\n$/.exec(text);
       if(!match||+match[1]!==epoch||crc16([...text.split('*')[0]].map(ch=>ch.charCodeAt(0)))!==parseInt(match[3],16))throw Error('Invalid history command');
       const id=+match[2];
       const history=Array.from({length:8},(_,i)=>[...u32(tick-60-50*(7-i)),...u32(1200+i),...u32(1800),1]).flat();
       emit(4,[...track(id),8,...history]);
      }else if(text.startsWith('C1,')){
       const [,id,kind,key]=text.split(/[,*]/);
       if(kind!=='K'||key!=='4')throw Error('Unexpected motion command');
       step=0;reason=1;emit(8,[...u32(+id),0,step]);emit(2,[step,reason]);
      }else throw Error('Unknown command');
    }});
    closed=false;timer=setInterval(live,100);live();
   },async close(){closed=true;clearInterval(timer);}};
   Object.defineProperty(navigator,'serial',{value:{requestPort:async()=>fake}});
  });
  await page.goto(pathToFileURL(path.resolve('tools/vision_xy.html')).href);
  await page.locator('#connect').click();
  await page.waitForFunction(()=>document.querySelector('#source-detail').textContent.includes('二进制'));
  assert.equal(await page.locator('#age-value').innerText(),'40 ms');
  await page.waitForFunction(()=>document.querySelector('#field-extrinsic').textContent.includes('Y 0.0 cm'));
  await page.waitForFunction(()=>document.querySelector('#grip-angle').textContent.includes('120° · 闭合指令'));
  await page.evaluate(()=>stopResume());
  await page.locator('#remote-key4').click();
  await page.waitForFunction(()=>document.querySelector('#control-status').textContent.includes('已确认'),null,{timeout:1800});
  await page.waitForFunction(()=>document.querySelector('#grip-angle').textContent.includes('50° · 松开指令'));
  await page.waitForFunction(()=>document.querySelector('#map-sync').textContent.includes('已同步 48 / 48'),null,{timeout:6000});
  assert((await page.evaluate(()=>sent.filter(x=>x==='MAP\n').length))>=2);
  assert.equal(await page.locator('.field-target').count(),48);
  assert(await page.locator('#metrics').innerText().then(x=>x.includes('页面 CRC 1')));
  assert.equal(await page.locator('#field-imu').innerText().then(x=>x.includes('360.00°')),true);
  assert.equal(await page.locator('#map-history-body tr').count(),0);
  await page.locator('#map-select').selectOption('48');
  await page.waitForFunction(()=>document.querySelectorAll('#map-history-body tr').length===8);
  const mapDownload=page.waitForEvent('download',{timeout:22000});
  await page.locator('#map-export').click();
  const mapPath=await (await mapDownload).path(),map=JSON.parse(fs.readFileSync(mapPath,'utf8'));
  assert.equal(map.complete,true);assert.equal(map.records.length,48);
  assert(map.records.every(p=>p.history.length===8));assert.deepEqual(map.pending_history,[]);
  const csvDownload=page.waitForEvent('download');
  await page.locator('#task-export').click();
  const csv=fs.readFileSync(await (await csvDownload).path(),'utf8');
  assert(csv.includes('"6","发现危险物，停车等待，移除后继续"'));assert(csv.includes('"恢复"'));
  const liveDownload=page.waitForEvent('download');
  await page.locator('#export').click();
  const log=fs.readFileSync(await (await liveDownload).path(),'utf8').trim().split('\n').map(JSON.parse);
  const heartbeat=log.find(l=>l.data?.type==='vision_state'),vision=log.find(l=>l.data?.source==='upper').data;
  assert.equal(heartbeat.wire_format,'uart5-bin-v1');assert.equal(heartbeat.wire_bytes,39);
  assert.equal(heartbeat.data.field.observed,null);assert.deepEqual(heartbeat.data.objects,[]);
  assert.equal(vision.objects[0].raw_x_mm,-65535);assert.equal(vision.objects[0].d_mm,-1);
  assert.equal(vision.objects[1].xy_derived,true);assert.equal(vision.lower.objects[0].object_inside,1);
  assert.equal(vision.imu.age_ms,145279);
  assert(log.some(row=>row.ack&&row.ack.ack_wait_ms>=0&&row.ack.ack_wait_ms<1800));
  await page.evaluate(()=>onlyHeartbeats());
  await page.waitForFunction(()=>!document.querySelector('.live-observation'),null,{timeout:1500});
  assert.equal(await page.locator('#age-value').innerText(),'40 ms');
  assert.equal(await page.locator('.field-target').count(),48);
  await page.evaluate(()=>restart());
  await page.waitForFunction(()=>document.querySelector('#map-sync').textContent.includes('已同步 0 / 0'),null,{timeout:4000});
  assert.equal(await page.locator('#age-value').innerText(),'—');
  assert.equal(await page.locator('.field-target').count(),0);
  await page.locator('#connect').click();assert.deepEqual(errors,[]);
  console.log('PASS: UART5 binary browser stream, split and coalesced frames, CRC drop, age and wrap, late events, KEY4 during sync, missing item repair, 48 tracks with 8 histories and JSONL V9, MCU restart');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1);});

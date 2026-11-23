const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');

const htmlPath=path.resolve(__dirname,'../../tools/vision_xy.html');
const source=fs.readFileSync(htmlPath,'utf8').match(/<script>([\s\S]*?)<\/script>/)[1];
const moduleStub={exports:{}};
vm.runInNewContext(source,{module:moduleStub});
const V=moduleStub.exports;

function bytes(format,values){
  const body=Buffer.alloc(format.length*4);
  let offset=0;
  [...format].forEach((type,i)=>{
    const size={B:1,H:2,h:2,I:4,i:4,f:4}[type];
    const write={B:'writeUInt8',H:'writeUInt16LE',h:'writeInt16LE',I:'writeUInt32LE',i:'writeInt32LE',f:'writeFloatLE'}[type];
    body[write](values[i],offset);offset+=size;
  });
  return body.subarray(0,offset);
}
let streamSequence=0;
function wire(type,tick,payload){
  const header=bytes('BBHHI',[1,type,streamSequence++&0xffff,payload.length,tick]);
  const body=Buffer.concat([header,payload]);
  let crc=0xffff;
  for(const b of body){
    crc^=b<<8;
    for(let i=0;i<8;i++)crc=((crc&0x8000)?(crc<<1)^0x1021:crc<<1)&0xffff;
  }
  const encoded=[0];
  let mark=0,length=1;
  for(const byte of Buffer.concat([body,bytes('H',[crc])])){
    if(byte===0){
      encoded[mark]=length;mark=encoded.length;encoded.push(0);length=1;
    }else{
      encoded.push(byte);
      if(++length===255){encoded[mark]=255;mark=encoded.length;encoded.push(0);length=1;}
    }
  }
  encoded[mark]=length;encoded.push(0);
  return Buffer.from(encoded);
}
const records=[
  bytes('IIIIIIII'+'f'.repeat(19)+'I',[1,0,17,8,3,150,800,200,165,18,165,68,160,72,180,0,0,180,180,15,2,2,2,40,0,0,0,2026112301]),
  bytes('IIIIIIffff',[0,1,1,1,0,0,5,-4,4,0]),
  bytes('IIhhhhhhhhhhhBB',[22,125,0,500,0,18000,1600,720,30,-20,1610,710,0,31,1]),
  bytes('IIIIIIIIf',[3,190,230,23,24,2,22,0,6]),
  bytes('IIHHBBBB'+'h'.repeat(6)+'H'+'h'.repeat(9),[22,230,10,90,3,1,3,1,1640,690,18000,-10,10,18000,35900,10,200,-30,100,0,180,-25,80,0]),
  bytes('IIhhhhiiii',[230,200,200,201,202,203,-2000,4000,6000,8000]),
  bytes('I'.repeat(14)+'i'.repeat(4)+'f'.repeat(7),[1,6,17,1,0,100,300,295,22,12,1,3,50,1,20,22,23,24,165,68,180,0,0,1,6])
];
function meta(boot_id=11,session_id=4,sequence=records.length,flags=2,dropped=0,start=100,end=300){
  return wire(12,end,bytes('IIIIIIHBB',[boot_id,session_id,sequence,dropped,start,end,sequence-dropped,flags,1]));
}
function chunk(from,to,indexes,boot_id=11,session_id=4){
  const items=indexes.map(seq=>{
    const tick=[100,105,155,230,245,250,300][seq],buf=records[seq];
    return Buffer.concat([bytes('IIBB',[seq,tick,seq+1,buf.length]),buf]);
  });
  return wire(13,310,Buffer.concat([bytes('IIIIB',[boot_id,session_id,from,to,indexes.length]),...items]));
}
const packet=wireBytes=>V.decodeBinary(wireBytes).data;
const readRows=(runs,run)=>V.key2Jsonl(runs,run).trimEnd().split('\n').map(JSON.parse);

function test(){
  const runs=new V.Key2Runs();
  assert.equal(runs.meta(packet(meta(0,0,0,0))),null);
  const run=runs.meta(packet(meta(11,4,4,1)));
  const first=packet(chunk(0,2,[0,1]));runs.chunk(first);
  assert.equal(runs.nextRead(run),2);
  runs.chunk(first);assert.equal(run.records.size,2);
  assert.equal(runs.nextRead(run),2); // A missing wireless response doesn't advance the repair cursor.
  const resumed=new V.Key2Runs();resumed.restore([runs.save(run)]);
  const restored=resumed.meta(packet(meta()));assert.equal(resumed.nextRead(restored),2);
  resumed.chunk(packet(chunk(2,6,[2,3,4,5])));
  assert.equal(resumed.summary(restored).missing_records,1);
  resumed.chunk(packet(chunk(6,7,[6])));
  const rows=readRows(resumed,restored),summary=rows.at(-1);
  assert.equal(summary.complete,true);
  assert.deepEqual([rows[3].before_x,rows[3].before_y,rows[3].after_x,rows[3].after_y,rows[3].age_ms],[161,71,164,69,30]);
  assert.deepEqual([rows[5].dx,rows[5].dy,rows[5].target_right,rows[5].applied_right],[1,-1,-3,-2.5]);
  assert.equal(rows[6].wheels[0].encoder_delta,-2000);
  assert.equal(rows[6].wheels[3].target,203);
  assert.equal(rows[5].imu_yaw,359);
  assert.equal(rows[0].firmware_version,2026112301);
  assert(summary.odometry.length&&summary.corrected.length);
  assert.equal(summary.timeline[0].duration_ms,195);
  assert.equal(rows.at(-2).outputs_zero,1);
  const oldKey=restored.key;
  const next=resumed.meta(packet(meta(11,5,1,1,0,800,800)));
  assert.equal(resumed.runs.get(oldKey).records.size,7);
  resumed.meta(packet(meta(33,1,1,1,0,900,900)));
  assert.match(resumed.summary(next).unavailable_reason,/重启/);
  assert.equal(resumed.runs.size,3);
  assert.equal(resumed.runs.get(oldKey).unavailable,'');
  const overflow=new V.Key2Runs(),out=overflow.meta(packet(meta(11,4,7,2,2)));
  overflow.chunk(packet(chunk(0,7,[0,3,4,5,6])));
  const gapRows=readRows(overflow,out),gap=gapRows.at(-1);
  assert.equal(gap.complete,false);
  assert.equal(gap.mcu_unavailable_records,2);
  assert.deepEqual(gap.missing_ranges,[{from_seq:1,to_seq_exclusive:3}]);
  assert.equal(gapRows.at(-2).type,'END');
  assert.equal(overflow.nextRead(out),7); // A first-class MCU gap is not endlessly retried.

  const long=new V.Key2Runs(),startTick=0xffffffd8;
  const many=long.meta(packet(meta(111,1,2301,2,0,startTick,2260)));
  const samples=Array.from({length:2301},(_,seq)=>({seq,tick:(startTick+seq)>>>0,type:'NAV',valid:1,
    x:165,y:65,yaw:180,odom_x:0,odom_y:3,odom_yaw:180}));
  samples[0]={...packet(chunk(0,1,[0])).records[0],tick:startTick};
  samples[2300]={seq:2300,tick:(startTick+2300)>>>0,type:'END',x:165,y:68,yaw:180,outcome:1};
  for(let i=0;i<samples.length;i+=15){const portion=samples.slice(i,i+15);long.chunk({boot_id:111,session_id:1,request_start:i,scan_next:i+portion.length,records:portion});}
  const longRows=readRows(long,many);
  assert.equal(longRows.length,2303);
  assert.equal(longRows.at(-2).relative_ms,2300);
  assert.equal(longRows.at(-1).total_ms,2300);
  assert.equal(longRows.at(-1).complete,true);
  assert.throws(()=>packet(chunk(1,2,[0])),/序号/);
  console.log('PASS: KEY2 binary records, out-of-order rejection, repair and deduplication, persistent run model, reboot separation, overflow holes, 2301 rows across tick wrap');
}

function option(name){const i=process.argv.indexOf(name);return i<0?null:process.argv[i+1];}
function firmwareFile(filename){
  const runs=new V.Key2Runs();
  let count=0;
  for(const [number,line] of fs.readFileSync(filename,'utf8').split(/\r?\n/).entries()){
    const match=/^WIRE\s+\S+\s+([0-9a-f\s]+)$/i.exec(line);
    if(!match)continue;
    let parsed;
    try{parsed=V.decodeBinary(Buffer.from(match[1].replace(/\s/g,''),'hex'));}
    catch(error){throw new Error('Actual firmware fixture line '+(number+1)+': '+error.message);}
    if(parsed.type===12){runs.meta(parsed.data);count++;}
    if(parsed.type===13){runs.chunk(parsed.data);count++;}
  }
  const run=[...runs.runs.values()].reverse().find(item=>runs.summary(item).complete);
  assert(run,'expected a complete session in actual firmware fixture');
  assert(count>=2,'expected actual META and CHUNK packets');
  const rows=readRows(runs,run),types=rows.map(r=>r.type);
  for(const type of ['START','STEP','OBS','FIX','NAV','WHEEL','END'])assert(types.includes(type),'missing actual '+type);
  assert.equal(rows[0].firmware_version,2026112304);
  assert.equal(rows[1].obs_max_ms,200);
  assert.equal(rows[1].max_fix,0);
  assert.equal(rows.at(-2).outputs_zero,1);
  const example=option('--example');
  if(example)fs.writeFileSync(example,V.key2Jsonl(runs,run),'utf8');
  console.log('PASS: actual firmware KEY2 packets -> browser decoder -> complete JSONL ('+rows.at(-1).records+' records)');
}

async function browserTest(){
  const {chromium}=require('playwright');
  const {pathToFileURL}=require('node:url');
  const browser=await chromium.launch({channel:'msedge',headless:true});
  try{
    const page=await browser.newPage({acceptDownloads:true}),errors=[];
    page.on('pageerror',error=>errors.push(error.message));
    const browserRows=records.map(r=>[...r]);
    browserRows[0][0]=2;
    await page.addInitScript(({rows})=>{
      const little=(n,len)=>Array.from({length:len},(_,i)=>(n>>>(8*i))&255);
      const u16=n=>little(n,2),u32=n=>little(n,4),pose=(x,y,yaw)=>[...u32(x),...u32(y),...u32(yaw)];
      let seq=10,tick=10000,stream,timer,boot=0,session=0,end=false,connected=false;
      let dropStart=true,roverState=0;
      const sent=window.motionCommands=[],requested=window.retryRequests=[],nonces=window.bootNonces=[];
      function emit(type,payload){
        if(!connected)return;
        const raw=[1,type,...u16(seq++),...u16(payload.length),...u32(tick),...payload];
        let crc=65535;
        for(const byte of raw){crc^=byte<<8;for(let i=0;i<8;i++)crc=((crc&32768)?(crc<<1)^0x1021:crc<<1)&65535;}
        const encoded=[0];let slot=0,length=1;
        for(const byte of raw.concat(u16(crc))){
          if(byte)encoded.push(byte);
          if(byte===0||++length===255){encoded[slot]=length;slot=encoded.push(0)-1;length=1;}
        }
        encoded[slot]=length;encoded.push(0);stream.enqueue(Uint8Array.from(encoded));
      }
      function emitMeta(){
        const count=end?7:session?2:0;
        emit(12,[...u32(boot),...u32(session),...u32(count),...u32(0),...u32(100),...u32(end?300:0),...u16(count),end?2:session?1:0,1]);
      }
      function emitLive(){
        tick+=100;
        emit(1,[roverState,0,0,0,...u16(50),7,...pose(1610,710,18000),...u32(12),...u16(18000)]);
      }
      window.key4Physically=()=>{end=true;roverState=0;emitMeta();};
      const fake={async open(){
        connected=true;this.readable=new ReadableStream({start(controller){stream=controller;}});
        this.writable=new WritableStream({write(bytes){
          const request=new TextDecoder().decode(bytes).trim();
          if(request==='GET'){
            emit(5,[...u32(17),...pose(1610,710,18000),...pose(0,0,0),...u16(0x20)]);
            emit(9,[0,0]);emitMeta();
          }else if(request.startsWith('KRUN?,')){
            const nonce=Number(request.split(',')[1]);nonces.push(nonce);
            boot||=nonce;emitMeta();
          }else if(request.startsWith('KREAD,')){
            const [,b,s,start]=request.split(',').map(Number);requested.push(start);
            if(b!==boot||s!==session){emitMeta();return;}
            if(dropStart){dropStart=false;return;}
            const stop=Math.min(end?7:2,start+2),out=[...u32(boot),...u32(session),...u32(start),...u32(stop),stop-start];
            for(let i=start;i<stop;i++)out.push(...u32(i),...u32([100,105,155,230,245,250,300][i]),i+1,rows[i].length,...rows[i]);
            emit(13,out);
          }else if(request.includes(',K,2*')){
            sent.push(request);session=1;roverState=80;end=false;emitMeta();
            emit(8,[...u32(Number(request.split(',')[1])),0,80]);
          }
        }});
        emitLive();timer=setInterval(emitLive,100);
      },async close(){clearInterval(timer);connected=false;}};
      Object.defineProperty(navigator,'serial',{value:{requestPort:async()=>fake}});
    },{rows:browserRows});
    await page.goto(pathToFileURL(htmlPath).href);
    await page.locator('#connect').click();
    await page.locator('#key2-state').filter({hasText:'等待 KEY2 启动'}).waitFor();
    await page.locator('#remote-key2').click();
    await page.locator('#key2-detail').filter({hasText:'已收 2 / 2 条'}).waitFor();
    assert.equal(await page.evaluate(()=>motionCommands.length),1);
    assert.deepEqual((await page.evaluate(()=>retryRequests)).slice(0,2),[0,0]);
    const oldNonce=await page.evaluate(()=>bootNonces[0]);
    await page.locator('#connect').click();
    await page.locator('#key2-state').filter({hasText:'等待重新连接'}).waitFor();
    await page.locator('#connect').click();
    await page.locator('#key2-state').filter({hasText:'记录中'}).waitFor();
    assert((await page.evaluate(()=>bootNonces)).some(nonce=>nonce!==oldNonce));
    const pending=page.waitForEvent('download');
    await page.evaluate(()=>key4Physically());
    const download=await pending;
    const jsonl=fs.readFileSync(await download.path(),'utf8').trimEnd().split('\n').map(JSON.parse);
    assert.equal(jsonl.at(-1).complete,true);
    assert.equal(jsonl.at(-1).records,7);
    assert.equal(jsonl[0].boot_id,oldNonce);
    assert.equal(jsonl.find(r=>r.type==='START').source,2);
    assert.match(download.suggestedFilename(),/^KEY2-物资区对正-.+\.jsonl$/);
    await page.waitForTimeout(1100); // Allow the page's normal periodic IndexedDB write.
    await page.reload();
    await page.locator('#key2-state').filter({hasText:'已收齐'}).waitFor();
    assert.equal(await page.locator('#key2-export').isEnabled(),true);
    assert.equal(await page.locator('#key2-steps li').count(),1);
    assert.equal(await page.locator('#key2-trails polyline').count(),2);
    assert.deepEqual(errors,[]);
    console.log('PASS: Edge mock UART KEY2, wireless retry, reconnect, physical KEY4, automated JSONL download, IndexedDB reload, stage list and two trajectories');
  } finally { await browser.close(); }
}

test();
const input=option('--firmware');
if(input)firmwareFile(input);
if(process.argv.includes('--browser'))browserTest().catch(error=>{console.error(error);process.exitCode=1;});

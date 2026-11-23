const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const html=fs.readFileSync(path.join(__dirname,'../../tools/vision_xy.html'),'utf8');
const source=html.match(/<script>([\s\S]*?)<\/script>/)[1];
const sandbox={module:{exports:{}},console};vm.createContext(sandbox);vm.runInContext(source,sandbox);
const V=sandbox.module.exports;
const raw=fs.readFileSync(process.argv[2],'utf8');
const p=V.parse(raw);
assert.equal(p.type,'vision_xy');assert.equal(p.count,3);
let d=V.derive(p,0,0,true);
assert.equal(d.points.length,2);assert.equal(d.rows.length,3);
assert.equal(d.points[0].x_cm,25.1);assert.equal(d.points[1].x_cm,-25);
assert(!d.points[0].control);assert(!d.rows[2].plottable);
assert.equal(V.derive(p,361,0,true).points.length,0);
assert.equal(V.derive(p,0,361,true).points.length,0);
assert.equal(V.derive(p,0,0,false).points.length,0);
const good=JSON.parse(JSON.stringify(p));good.ready=good.available=1;good.danger=0;good.objects[0].valid=1;
assert(V.derive(good,0,0,true).points[0].control);
assert(!V.derive(good,110,0,true).points[0].control);
const bad=JSON.parse(JSON.stringify(p));bad.objects[2].valid=1;assert.throws(()=>V.parse(JSON.stringify(bad)));
bad.objects[2].valid=0;bad.count=4;assert.throws(()=>V.parse(JSON.stringify(bad)));
const received=[];let errors=0;const lines=new V.Lines(s=>received.push(V.parse(s)),()=>errors++);
const packet=raw+'\r\n';for(const c of packet)lines.push(c);lines.push(packet+packet);
assert.equal(received.length,3);lines.push('x'.repeat(8200)+'\n'+packet);
assert.equal(errors,1);assert.equal(received.length,4);
const b={xmin:-60,xmax:60,ymin:-20,ymax:100};
const zero=V.point(0,0,b,900,540),right=V.point(10,0,b,900,540),forward=V.point(0,10,b,900,540);
assert(right.x>zero.x);assert(forward.y<zero.y);
assert(Math.abs((right.x-zero.x)-(zero.y-forward.y))<1e-8);
assert(source.includes('getWriter('));
console.log('PASS: actual C telemetry -> browser parser, XY axis orientation/equal scale, estimates, unknown coordinates, expiry, split/merged/oversize lines, bidirectional control page');

const rv3=JSON.parse(JSON.stringify(p));rv3.protocol='vision_simple_v3';rv3.ack=rv3.ready=0;rv3.danger=1;rv3.available=1;rv3.objects[0].valid=1;
assert.equal(V.derive(rv3,0,0,true).points.length,2);
assert(V.derive(rv3,0,0,true).points[0].control);
rv3.protocol='rescue_json_v2';assert.equal(V.derive(rv3,0,0,true).points.length,0);
rv3.protocol='unknown';assert.throws(()=>V.parse(JSON.stringify(rv3)));
console.log('PASS: RV3 displays without fake ACK/ready; JSON still requires ACK; unknown protocol rejected');

assert.equal(p.wire_version,8);
assert(p.control && p.control.id===null && p.control.target===null);
assert.equal(V.crc16('123456789'),0x29b1);
assert.throws(()=>V.parse(raw.slice(0,80)+raw.slice(100)),/CRC/);
assert.throws(()=>V.parse(raw.replace('500','501')),/CRC/);
assert.throws(()=>V.parse(raw.slice(0,raw.lastIndexOf('*'))),/CRC/);
function framed(o){const s=JSON.stringify(o);return s+'*'+V.crc16(s).toString(16).padStart(4,'0');}
const actualArray=JSON.parse(raw.slice(0,raw.lastIndexOf('*')));
assert.equal(actualArray.length,19);assert.equal(actualArray[0],'V8');
const nav=JSON.parse(JSON.stringify(actualArray));nav[15][0]=50;nav[17]=[123,0,2000,1800,50];
assert.equal(V.parse(framed(nav)).control.target.x,200);
nav[17][2]=3001;assert.throws(()=>V.parse(framed(nav)));
const array=actualArray.slice(0,17);array[0]='V5';
array[12]=array[12].map(o=>o.slice(0,4));
array[14]=[...array[14].slice(0,4),array[14][4].map(o=>o.slice(0,2))];
assert.equal(array.length,17);assert.equal(array[0],'V5');
const compact={type:'vision_xy',version:2,t:array[1],l:array[2],r:array[3],b:array[4],u:array[5],d:array[6],f:array[7],a:array[8],s:array[9],v:array[10],q:array[11],o:array[12],g:array[13],n:array[14],m:array[15]};
const invalid=JSON.parse(JSON.stringify(compact));invalid.o[0][2]=19000;assert.throws(()=>V.parse(framed(invalid)));
const timed=[];const splitter=new V.Lines((s,ms)=>timed.push([s,ms]),()=>assert.fail());
splitter.push('abc',100);splitter.push('def\n',135);assert.deepEqual(timed,[['abcdef',35]]);
const recovered=[];let damaged=0;const recover=new V.Lines(s=>{try{recovered.push(V.parse(s));}catch(e){damaged++;}},()=>damaged++);
recover.push(raw.slice(0,90)+raw.slice(130)+'\n'+raw+'\n');assert.equal(damaged,1);assert.equal(recovered.length,1);
console.log('PASS: compact telemetry CRC detects deletion/corruption/missing checksum, stream recovers, assembly age tracked');
const history=new V.PositionHistory();
const observed=JSON.parse(JSON.stringify(p));observed.age_ms=218;observed.objects[0].valid=1;
history.accept(observed,1000,50);
const at=(time,current=observed)=>history.display(V.derive(current,time-1000,50,true),time,true);
assert(!at(1000).historical && at(1000).points.length===2);
let held=at(1200);assert(held.historical && !held.fresh && held.age===468 && held.points.length===2);
assert(held.points.every(o=>!o.control));
assert(at(2231).historical);assert.equal(at(2232).points.length,0);
// Repeated IDs cannot restart the history lifetime, even after expiry.
history.accept({...observed,age_ms:0},2300,0);assert.equal(at(2300).points.length,0);
const next={...observed,frame:observed.frame+1,age_ms:20};history.accept(next,2400,10);
assert(history.display(V.derive(next,400,10,true),2800,true).historical);
const stale={...next,status:2,fresh:0,available:0,count:0,objects:[]};history.accept(stale,2750,10);
assert(history.display(V.derive(stale,0,10,true),2750,true).historical);
const empty={...stale,status:1,fresh:1,available:1};history.accept(empty,2800,10);
assert.equal(history.display(V.derive(stale,0,10,true),2800,true).points.length,0);
for(const cleared of [{...next,status:3},{...next,status:4},{...next,link:0},{...next,objects:[],count:0}]){
 history.accept({...next,frame:next.frame+1},3000,10);history.accept(cleared,3010,10);
 assert.equal(history.display(V.derive(stale,0,10,true),3400,true).points.length,0);
}
history.accept(next,4000,10);assert.equal(history.display(V.derive(next,400,10,false),4400,false).points.length,0);
assert.equal(history.display(V.derive(stale,0,10,true),4401,true).points.length,0);
console.log('PASS: history stays visibly stale, duplicate frames cannot renew it, 1.5s expiry, empty/fault/disconnect clear immediately');

assert(!V.ageExpired(null));assert(!V.ageExpired(399));assert(!V.ageExpired(400));assert(V.ageExpired(401));
const boundary={...p,age_ms:400};assert(V.derive(boundary,0,0,true).fresh);assert(!V.derive(boundary,1,0,true).fresh);
assert(!V.derive({...p,age_ms:null},5000,0,true).expired);
const waiting=new V.PositionHistory();waiting.accept({...p,age_ms:200},0,0);
const unavailable={...p,status:2,fresh:0,objects:[],count:0,age_ms:350};waiting.accept(unavailable,150,0);
const pending=waiting.display(V.derive(unavailable,0,0,true),150,true);
assert(pending.historical && !pending.expired && pending.age===350);
assert(waiting.display(V.derive(unavailable,0,0,true),201,true).expired);
console.log('PASS: displayed age 400ms is not expired, 401ms is expired, unknown age is not timeout, early upstream stale is waiting-update');

// No theoretical wire budget or extra margin; observed assembly is added once.
assert(source.includes("wireMs=mode==='demo'?0:Math.max(0,assemblyMs)"));
assert(!source.includes('assemblyMs+6'));assert(!source.includes('10000/115200'));
const measured={...p,age_ms:218};
assert.equal(V.derive(measured,20,35,true).age,273);
assert.equal(V.derive(measured,20,35,true).age,273);
assert.equal(V.derive(measured,21,35,true).age,274);
splitter.push('whole\n',200);assert.equal(timed[1][1],0);
console.log('PASS: measured assembly only, no artificial margin, repeated age calculations do not accumulate');

assert.equal(p.grip,120);assert.equal(p.lower.objects.length,2);assert.equal(p.lower.objects[0].in_roi,1);
assert.equal(p.lower.objects[0].zone_kind,6);assert.equal(p.lower.objects[0].object_inside,1);
assert.equal(V.deriveLower(p,109,0,true).rows[0].object_inside,1);
assert.equal(V.deriveLower(p,110,0,true).rows[0].object_inside,undefined);
for(const mutate of [a=>a[12][0][4]=-65536,a=>a[14][4][0][2]=4,a=>a[14][4][0][3]=2]){
 const a=JSON.parse(JSON.stringify(actualArray));mutate(a);assert.throws(()=>V.parse(framed(a)));
}
assert.equal(V.deriveLower(p,0,0,true).roi,1);
assert(V.deriveLower({...p,fresh:0,objects:[],count:0},0,0,true).fresh);
assert(V.deriveLower({...p,lower:{...p.lower,age_ms:400}},0,0,true).fresh);
assert(!V.deriveLower({...p,lower:{...p.lower,age_ms:400}},1,0,true).fresh);
assert.equal(V.deriveLower(p,401,0,true).rows.length,0);
assert.equal(V.deriveLower(p,0,0,false).roi,0);
assert.equal(V.deriveLower({...p,lower:{...p.lower,fresh:0,status:3}},0,0,true).roi,0);
const old={...compact};delete old.n;delete old.g;delete old.m;assert.equal(V.parse(framed(old)).lower,null);
for(const change of [q=>q.n[4][0][1]=2,q=>q.g=140,q=>q.n[3]=0,q=>q.n[4].push([9,1])]){
 const q=JSON.parse(JSON.stringify(compact));change(q);assert.throws(()=>V.parse(framed(q)));
}
console.log('PASS: actual lower telemetry, ROI independent of upper, 400/401ms expiry, disconnect, legacy telemetry and invalid fields');

const legacy=V.parse(framed(compact));assert.equal(legacy.wire_version,2);
const legacyExpected=JSON.parse(JSON.stringify({...p,field:null,control:null,imu:null,wire_version:2,protocol:'vision_simple_v2',fault:'rv2_observation'}));
legacyExpected.objects.forEach(o=>delete o.track_id);
Object.assign(legacyExpected.objects[0],{x_cm:25,y_cm:43.3});
delete legacyExpected.lower.objects[0].zone_kind;delete legacyExpected.lower.objects[0].object_inside;
assert.deepEqual(legacyExpected,legacy);
assert.equal(V.parse(framed(['V3',...array.slice(1,15)])).mission,null);
assert.throws(()=>V.parse(framed([...array,0])));
const corrupted=[...array];corrupted[14]=null;assert.throws(()=>V.parse(framed(corrupted)));
console.log('PASS: V3 preserves all V2 fields and rejects malformed arrays; sample '+Buffer.byteLength(raw.trim())+' bytes vs '+Buffer.byteLength(framed(compact))+' bytes');

assert.equal(p.test,0);assert.equal(p.mission.step,0);
const active=[...array];active[15]=[30,3,2,0];
assert.deepEqual(JSON.parse(JSON.stringify(V.parse(framed(active)).mission)),{step:30,reason:3,inside:2,outside:0,held_kind:null});
for(const m of [[2,0,0,0],[30,25,0,0],[30,0,4,0],[30,0,0]]){const q=[...array];q[15]=m;assert.throws(()=>V.parse(framed(q)));}
console.log('PASS: task firmware/state/reason/ROI confirmation telemetry, no false stationary flag');

for(const reason of [16,17]) {const q=[...array];q[15]=[30,reason,0,0];assert.equal(V.parse(framed(q)).mission.reason,reason);}

assert.equal(p.mission.held_kind,0);
for(const held of [0,1,2,3]) {const q=[...array];q[15]=[99,0,0,3,held];assert.equal(V.parse(framed(q)).mission.held_kind,held);}
for(const held of [-1,10,1.5,null]) {const q=[...array];q[15]=[99,0,0,3,held];assert.throws(()=>V.parse(framed(q)));}
console.log('PASS: held cargo survives ROI=0 in telemetry; old task frames retain unknown cargo, malformed cargo rejected');

assert(p.field.aligned);assert.equal(p.field.robot.x,285);assert.equal(p.field.origin.y,69.5);assert.equal(p.field.robot.y,p.field.origin.y+5);
assert.equal(p.field.observed.y,p.field.origin.y+3);assert.equal(p.field.points[0].x,310.1);assert.equal(p.field.points[2],null);
assert.equal(p.field.extrinsic.y,0); // Actual firmware V8 payload, decoded in cm.
const expectedFieldY=p.field.observed.y+p.objects[0].y_cm;
assert(Math.abs(p.field.points[0].y-expectedFieldY)<=0.100001); // Integer mm on the wire.
console.log('PASS: firmware V8 reports extrinsic.y=0 and projects the unshifted raw observation');
for(const mutate of [a=>a[16][1]=0,a=>a[16][2]=null,a=>a[16][3].pop(),a=>a[16][0][0]=NaN]){const a=JSON.parse(JSON.stringify(array));mutate(a);assert.throws(()=>V.parse(framed(a)));}
const fh=new V.FieldHistory();fh.accept(p,1000,20);
let fd=fh.display(p,1340,340,true);assert(!fd.points[0].historical);
fd=fh.display(p,1341,341,true);assert(fd.points[0].historical);
const moved=JSON.parse(JSON.stringify(p));moved.tick++;moved.field.robot.y+=20;fh.accept(moved,1350,20);
fd=fh.display(moved,1400,50,true);assert.equal(fd.points[0].y,p.field.points[0].y);assert.equal(fh.entry.now,1000);
assert.equal(fh.display(moved,2440,1090,true).points.length,0);
fh.accept({...p,tick:p.tick+2,frame:p.frame+1},3000,0);assert(fh.entry);
fh.accept({...p,tick:p.tick+3,status:1},3010,0);assert.equal(fh.entry,null);
fh.clear();assert.equal(fh.trail.length,0);
console.log('PASS: actual MCU field coordinates, aligned pose, unknowns, malformed map rejection, fixed historical coordinates, 400ms expiry, no duplicate refresh, clear/reset');

assert.equal(p.field.points[0].track_id,0);assert.equal(p.field.points[1].track_id,0);assert.equal(p.objects[0].track_id,0);assert.equal(p.field.points[0].radius,6.5);
assert.equal(V.targetStyle(1).fill,'#59dc82');assert.equal(V.targetStyle(2).fill,'#111111');assert.equal(V.targetStyle(3).fill,'#ff9b47');assert.equal(V.targetStyle(4).fill,'#69cfff');assert.notEqual(V.targetStyle(2).fill,V.targetStyle(2).stroke);
assert.equal(V.targetLabel({id:1,track_id:1}),'#1');assert.equal(V.targetLabel(p.field.points[0]),'序1');
const v4=JSON.parse(JSON.stringify(array));v4[0]='V4';v4[16][3]=v4[16][3].map(v=>v?v.slice(0,2):null);
const oldMap=V.parse(framed(v4));assert.equal(oldMap.wire_version,4);assert.equal(oldMap.field.points[0].radius,null);assert.equal(oldMap.field.points[0].track_id,0);
for(const mutate of [a=>a[16][3][0][2]=-1,a=>a[16][3][0][3]=-1,a=>{a[16][3][0][2]=1;a[16][3][1][2]=1;},a=>a[16][3][0].pop()]){const a=JSON.parse(JSON.stringify(array));mutate(a);assert.throws(()=>V.parse(framed(a)));}
console.log('PASS: V8 unconfirmed IDs and measurement radii, category colours and black outline, invalid/duplicate confirmed IDs rejected, old V4 retained');

for (const step of [60,61,62,63,64,65,66,67,68,69,70,71,72,73,74]) {
 const reason=step===67?18:step===69?20:0;
 const a=actualArray.slice();a[15]=[step,reason,0,0,step<=64||step===67||step===68||step===70?1:0];
 const mission=V.parse(framed(a)).mission;
 assert.equal(mission.step,step);
 assert.equal(mission.reason,reason);
}
for (const reason of [21,22,23,24]) {
 const a=actualArray.slice();a[15]=[reason===24?71:69,reason,0,0,0];
 assert.equal(V.parse(framed(a)).mission.reason,reason);
}
console.log('PASS: return-material stages, dock interruption, X sample diagnostics and centre-scan states round-trip');

const click=V.fieldClick(230,150);assert.equal(click.x,200);assert.equal(click.y,180);
assert.equal(V.fieldClick(20,150),null);assert.equal(V.fieldClick(330,30).y,300);
assert(V.command(123,'G',[200,180,50]).startsWith('C1,123,G,2000,1800,500*'));
assert(V.command(124,'K',[4]).startsWith('C1,124,K,4*'));
assert.throws(()=>V.command(1,'G',[-1,20,50]));assert.throws(()=>V.command(1,'K',[5]));
const ack='A1,123,0,50';assert.equal(V.ack(ack+'*'+V.crc16(ack).toString(16).padStart(4,'0')).step,50);
assert.throws(()=>V.ack(ack+'*0000'));
console.log('PASS: V6 control feedback, click coordinates, command encoding and CRC acknowledgement validation');

assert(p.imu.ready && p.imu.heading_valid);assert.equal(p.imu.raw_yaw,25);
assert(V.imuStatus(p,400,0,true).fresh);assert(!V.imuStatus(p,401,0,true).fresh);
assert(!V.imuStatus(p,0,401,true).fresh);assert(!V.imuStatus(p,0,0,false).fresh);
const v6=[...array,actualArray[17]];v6[0]='V6';assert.equal(V.parse(framed(v6)).imu,null);
const unready=JSON.parse(JSON.stringify(actualArray));unready[18]=[0,null,null,0];assert(!V.imuStatus(V.parse(framed(unready)),0,0,true).ready);
for(const data of [[1,null,0,1],[0,0,0,0],[1,0,36001,1],[1,0,0,2]]){unready[18]=data;assert.throws(()=>V.parse(framed(unready)));}
console.log('PASS: V7 IMU telemetry, readiness/age/raw yaw, 400ms boundary, disconnect and V6 compatibility');

const taskLog=new V.TaskHistory();
const report=(tick,step,reason=0)=>taskLog.accept(tick,{step,reason});
report(100,64);report(250,65);report(250,69); // ignore duplicate timestamp
report(400,65);report(550,69);report(700,71,24);
report(850,72,24);report(1000,73);report(1150,74);
report(1300,30);report(1450,0,1);
assert.deepEqual(Array.from(taskLog.entries,r=>[r.step,r.elapsedMs,r.result]),[
 [64,150,'成功'],[65,300,'成功'],[69,150,'失败'],[71,150,'成功'],
 [72,150,'成功'],[73,150,'失败'],[30,150,'失败']
]);
assert(taskLog.csv().includes('"65","后退 50 cm","0.300","成功",""'));
assert(taskLog.csv().includes('X 校准超时，沿用现有 X'));
assert(taskLog.csv().includes('KEY4 停止'));
const missed=new V.TaskHistory();
missed.accept(1700,{step:68,reason:0});missed.accept(1850,{step:65,reason:0});
missed.accept(2000,{step:65,reason:0});missed.accept(3200,{step:69,reason:20});
assert.equal(missed.entries[0].result,'未确认');
assert.equal(missed.entries[1].elapsedMs,150);
assert(missed.csv().includes('执行中'));
missed.disconnect();assert.equal(missed.entries[2].detail,'串口断开');
const wrapLog=new V.TaskHistory();
wrapLog.accept(0xffffff00,{step:68,reason:0});wrapLog.accept(0x20,{step:67,reason:6});
assert.equal(wrapLog.entries[0].elapsedMs,288);
assert.equal(wrapLog.entries[0].detail,'发现危险物');
wrapLog.accept(1000,{step:65,reason:0});wrapLog.accept(1,{step:0,reason:0});
assert.equal(wrapLog.entries[1].detail,'主控计时重置');
console.log('PASS: browser-only task history CSV, success/failure, X timeout, KEY4, missed steps, disconnect, MCU tick wrap/reset');

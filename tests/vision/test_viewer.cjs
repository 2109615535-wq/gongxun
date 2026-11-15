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
assert.equal(d.points[0].x_cm,25);assert.equal(d.points[1].x_cm,-25);
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
assert(!source.includes('getWriter('));assert(!source.includes('port.write('));
console.log('PASS: actual C telemetry -> browser parser, XY axis orientation/equal scale, estimates, unknown coordinates, expiry, split/merged/oversize lines, read-only page');

const rv2=JSON.parse(JSON.stringify(p));rv2.protocol='vision_simple_v2';rv2.ack=rv2.ready=0;rv2.danger=1;rv2.available=1;rv2.objects[0].valid=1;
assert.equal(V.derive(rv2,0,0,true).points.length,2);
assert(V.derive(rv2,0,0,true).points[0].control);
rv2.protocol='rescue_json_v2';assert.equal(V.derive(rv2,0,0,true).points.length,0);
rv2.protocol='unknown';assert.throws(()=>V.parse(JSON.stringify(rv2)));
console.log('PASS: RV2 displays without fake ACK/ready; JSON still requires ACK; unknown protocol rejected');

assert.equal(p.wire_version,3);
assert.equal(V.crc16('123456789'),0x29b1);
assert.throws(()=>V.parse(raw.slice(0,80)+raw.slice(100)),/CRC/);
assert.throws(()=>V.parse(raw.replace('500','501')),/CRC/);
assert.throws(()=>V.parse(raw.slice(0,raw.lastIndexOf('*'))),/CRC/);
function framed(o){const s=JSON.stringify(o);return s+'*'+V.crc16(s).toString(16).padStart(4,'0');}
const array=JSON.parse(raw.slice(0,raw.lastIndexOf('*')));
assert.equal(array.length,16);assert.equal(array[0],'V3');
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

assert.equal(p.grip,100);assert.equal(p.lower.objects.length,2);assert.equal(p.lower.objects[0].in_roi,1);
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
assert.deepEqual({...p,wire_version:2},legacy);
assert.equal(V.parse(framed(array.slice(0,-1))).mission,null);
assert.throws(()=>V.parse(framed([...array,0])));
const corrupted=[...array];corrupted[14]=null;assert.throws(()=>V.parse(framed(corrupted)));
console.log('PASS: V3 preserves all V2 fields and rejects malformed arrays; sample '+Buffer.byteLength(raw.trim())+' bytes vs '+Buffer.byteLength(framed(compact))+' bytes');

assert.equal(p.test,0);assert.equal(p.mission.step,0);
const active=[...array];active[15]=[30,3,2,0];
assert.deepEqual(JSON.parse(JSON.stringify(V.parse(framed(active)).mission)),{step:30,reason:3,inside:2,outside:0,held_kind:null});
for(const m of [[2,0,0,0],[30,18,0,0],[30,0,4,0],[30,0,0]]){const q=[...array];q[15]=m;assert.throws(()=>V.parse(framed(q)));}
console.log('PASS: task firmware/state/reason/ROI confirmation telemetry, no false stationary flag');

for(const reason of [16,17]) {const q=[...array];q[15]=[30,reason,0,0];assert.equal(V.parse(framed(q)).mission.reason,reason);}

assert.equal(p.mission.held_kind,0);
for(const held of [0,1,2,3]) {const q=[...array];q[15]=[99,0,0,3,held];assert.equal(V.parse(framed(q)).mission.held_kind,held);}
for(const held of [-1,10,1.5,null]) {const q=[...array];q[15]=[99,0,0,3,held];assert.throws(()=>V.parse(framed(q)));}
console.log('PASS: held cargo survives ROI=0 in telemetry; old task frames retain unknown cargo, malformed cargo rejected');

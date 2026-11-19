const fs=require('fs'),vm=require('vm'),path=require('path'),assert=require('assert');
const source=fs.readFileSync(path.join(__dirname,'../../tools/vision_xy.html'),'utf8').match(/<script>([\s\S]*?)<\/script>/)[1];
const sandbox={module:{exports:{}},console};vm.createContext(sandbox);vm.runInContext(source,sandbox);const V=sandbox.module.exports;
const raw=fs.readFileSync(process.argv[2],'utf8').split(/\r?\n/).filter(s=>s.startsWith('["M1",'));
assert.equal(raw.length,3);const packets=raw.map(V.parseMemory),m=new V.MapMemory();
const frame=a=>{const b=JSON.stringify(a);return b+'*'+V.crc16(b).toString(16).padStart(4,'0');};
assert.equal(packets[0].track.history.length,8);assert.equal(packets[0].track.observations,13);
m.accept(packets[0],1000,20);const original=m.display(1000,true)[0];assert(!original.historical);
assert.equal(m.display(1000+400-original.age,true)[0].historical,false);
assert.equal(m.display(1001+400-original.age,true)[0].historical,true);
assert.equal(m.display(70000,true).length,1);assert.equal(m.display(70000,false).length,1);
m.accept(packets[1],2000,20);assert(m.display(2000,true)[0].historical);
assert.equal(m.display(2000,true)[0].x,original.x);assert.equal(m.display(2000,true)[0].history.length,8);
m.accept(packets[2],12000,20);assert(m.display(12000,true)[0].age>10000); // retransmit cannot renew age
assert.equal(m.display(12000,true)[0].observations,13);
// Empty/fault V7 observations never delete authoritative map entries.
m.observe({tick:packets[2].tick+10,status:1,field:null});assert.equal(m.entries.size,1);
m.observe({tick:packets[2].tick+20,status:3,field:null});assert.equal(m.entries.size,1);
// Reset epoch clears old records, while an ordinary uint32 wrap does not.
const reset=V.parseMemory(frame(['M1',packets[2].tick+30,packets[2].epoch+1,0,0,0,null]));
assert(m.accept(reset,13000,0));assert.equal(m.entries.size,0);
m.accept({...packets[0],tick:0xfffffff0},14000,0);m.observe({tick:20,field:null});assert.equal(m.entries.size,1);
assert(m.accept({...packets[0],tick:0},15000,0));assert.equal(m.entries.size,1);
// Reject corrupted CRC, missing checksum, malformed/oversized history and future timestamps.
assert.throws(()=>V.parseMemory(raw[0].replace('M1','M2')));assert.throws(()=>V.parseMemory(raw[0].split('*')[0]));
for(const mutate of [p=>p[4]=49,p=>p[6][0]=0,p=>p[6][8]=2,p=>p[6][11].push(p[6][11][0]),p=>p[6][11][0][0]=p[1]+1,p=>p[6][11][0][1]='bad']){
 const a=JSON.parse(raw[0].split('*')[0]);mutate(a);assert.throws(()=>V.parseMemory(frame(a)));
}
console.log('PASS: actual MCU M1 -> page, 8 raw samples, 400/401ms age, long absence/empty/fault/disconnect retention, retransmit age, reset epoch, tick wrap and strict malformed packet rejection');

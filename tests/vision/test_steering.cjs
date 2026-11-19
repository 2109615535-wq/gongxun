const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const html=fs.readFileSync(path.join(__dirname,'../../tools/vision_xy.html'),'utf8');
const box={module:{exports:{}},console};vm.createContext(box);vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],box);
const V=box.module.exports,frame=p=>{const s=JSON.stringify(p);return s+'*'+V.crc16(s).toString(16).padStart(4,'0');};
if(process.argv[2]){
 const s=fs.readFileSync(process.argv[2],'utf8').split(/\r?\n/).find(l=>l.startsWith('["S1",'));
 const p=V.parseSteer(s);assert.equal(p.source,1);assert.equal(p.target,0);assert.equal(p.current,-10);
 assert.equal(p.error,10);assert.equal(p.output,30);assert.equal(p.p,25);assert.equal(p.i,5);
 assert.equal(p.kp,2.5);assert.equal(p.ki,.5);assert.equal(p.kd,.12);
}
const good=['S1',1000,2,20,0,-.5,.5,1,.8,.2,0,1.6,.4,0];
assert.equal(V.parseSteer(frame(good)).current,-.5);
assert.throws(()=>V.parseSteer(frame(good).replace('1000','1001')));
assert.throws(()=>V.parseSteer(JSON.stringify(good)));
for(const mutate of [p=>p.pop(),p=>p[2]=3,p=>p[3]=null,p=>p[5]=null,p=>p[11]=-1]){
 const p=good.slice();mutate(p);assert.throws(()=>V.parseSteer(frame(p)));
}
const idle=good.slice();idle[2]=0;idle[3]=null;assert.equal(V.parseSteer(frame(idle)).source,0);
assert(V.command(1,'P',[1.35,.1,.02]).startsWith('C1,1,P,13500,1000,200*'));
assert(V.command(2,'T',[1]).startsWith('C1,2,T,1*'));
for(const args of [[-1,0,0],[Infinity,0,0],[NaN,0,0],[1,2]])assert.throws(()=>V.command(1,'P',args));
assert.throws(()=>V.command(1,'T',[2]));
console.log('PASS: actual C S1 -> browser values/signs/components/gains, CRC and strict malformed input rejection; fixed-point gain commands');

/* Synthetic observations only: no hardware and no calibration results. */
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(path.join(__dirname,'../../tools/vision_xy.html'),'utf8');
const sandbox={module:{exports:{}}};vm.createContext(sandbox);vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],sandbox);
const V=sandbox.module.exports,plain=v=>JSON.parse(JSON.stringify(v));
const close=(a,b)=>assert(Math.abs(a-b)<1e-9,`${a} != ${b}`);
const config={kind:1,x:0,y:50,seconds:20,reference:'wheel-centre ground projection -> visible contact edge'};
function packet(frame,tick=frame*150,objects=[[1,600,200,0]],age=50){
 const pose=[2850,800,0];
 const a=['V7',tick,1,100,0,0,0,frame,age,objects.length?0:1,1,0,objects,50,[frame,50,1,1,[]],[0,0,0,0,0],[pose,1,pose,objects.map(()=>null),pose,[0,0,0]],[null,0,null,null,null],[1,0,3700,1]];
 const s=JSON.stringify(a);return V.parse(s+'*'+V.crc16(s).toString(16).padStart(4,'0'));
}
const c=new V.ErrorSampling();
assert.throws(()=>c.start({...config,reference:''},0,packet(1)));
assert.throws(()=>c.start({...config,y:0},0,packet(1)));
assert.throws(()=>c.start(config,0,{...packet(1),mission:{step:30}}));
c.start(config,0,packet(1));
assert(!c.accept(packet(2),500)); // settling, not a sample
assert(!c.accept(packet(2),1100)); // duplicate cannot become a new observation
assert(c.accept(packet(3),1200)); // source valid=0 estimates are intentional
assert(!c.accept(packet(3),1300));
assert(!c.accept(packet(4,600,[[1,500,0,0],[1,600,0,0]]),1400));
assert(!c.accept(packet(5,750,[[2,500,0,0]]),1500));
assert(!c.accept(packet(6,900,[[1,-1,null,0]]),1600));
assert(c.accept(packet(7,1050,[[1,620,400,0]],400),1700,0));
assert(!c.accept(packet(8,1200,[[1,600,0,0]],400),1800,1));
assert(c.accept(packet(9,1350,[[3,100,0,0],[1,580,0,0]],100),1900,10)); // frame-index changes do not change identity
assert.equal(c.active.samples.length,3);
const s=V.sampleSummary(c.active);
close(s.range_cm.bias,10);close(s.range_cm.sd,2);
close(s.angle_deg.bias,2);close(s.angle_deg.sd,2);
close(s.range_cm.rmse,Math.sqrt((100+144+64)/3));
close(s.covariance_cm_rad[0][1],4*Math.PI/180);
close(s.covariance_cm_rad[1][1],(2*Math.PI/180)**2);
assert.equal(s.range_cm.p95_abs,12);assert.equal(c.active.samples[0].valid,0);
assert.equal(c.active.skipped['所选颜色有多个目标'],1);
assert.equal(c.active.skipped['未识别所选颜色'],1);
c.poll(21000);assert.equal(c.active,null);assert.equal(c.groups[0].status,'完成');
const restored=new V.ErrorSampling();restored.restore(plain(c.dataset()));
assert.equal(restored.groups[0].samples.length,3);assert.equal(restored.csv().trim().split('\r\n').length,4);
assert.throws(()=>restored.restore({type:'wrong'}));
const malformed=plain(c.dataset());malformed.groups[0].samples[0].d_mm=null;assert.throws(()=>restored.restore(malformed));
close(V.wrapAngle(359),-1);close(V.wrapAngle(-359),1);
assert.equal(V.errorStats([]),null);assert.equal(V.errorStats([4]).sd,null);
const one={...config,samples:[{d_mm:1000,angle:-179}]};
one.x=Math.sin(179*Math.PI/180)*100;one.y=Math.cos(179*Math.PI/180)*100;
close(V.sampleSummary(one).angle_deg.bias,2); // wrapped innovation, not -358 degrees
const roll=new V.ErrorSampling();roll.start(config,0,packet(0xfffffffe,0xfffffff0));
assert(roll.accept(packet(0xffffffff,50),1100));assert(roll.accept(packet(0,200),1250));
roll.accept(packet(1,0),1400);assert(!roll.active);assert(roll.groups[0].status.includes('时间回退'));
const frameReset=new V.ErrorSampling();frameReset.start(config,0,packet(100));frameReset.accept(packet(1,16000),1200);
assert(!frameReset.active);assert(frameReset.groups[0].status.includes('帧号回退'));
const motion=new V.ErrorSampling();motion.start(config,0,packet(1));motion.accept({...packet(2),mission:{step:30}},1200);
assert(!motion.active);assert.equal(motion.groups[0].samples.length,0);
const incomplete=new V.ErrorSampling();incomplete.start(config,0,packet(1));incomplete.accept(packet(2),1200);
restored.restore(plain(incomplete.dataset()));assert(!restored.active);assert(restored.groups[0].status.includes('未完成'));
incomplete.stop();assert(!incomplete.accept(packet(3),1400));
console.log('PASS: raw V7 estimates, new-frame-only sampling, ambiguity/missing/stale rejection, exact 400 ms boundary, unbiased sample SD, bias/RMSE/percentiles/cross-covariance, angle wrap, MCU/frame reset and wrap, task movement stop, incomplete cache restoration, CSV export');

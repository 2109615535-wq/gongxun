const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const html=fs.readFileSync(path.join(__dirname,'../../tools/vision_xy.html'),'utf8');
const box={module:{exports:{}},console};vm.createContext(box);vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],box);
const V=box.module.exports,frame=p=>{const s=JSON.stringify(p);return s+'*'+V.crc16(s).toString(16).padStart(4,'0');};
if (process.argv[2] === '--firmware') { checkFirmwareVectors(process.argv[3]); return; }
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

function checkFirmwareVectors(vectorPath) {
  const vision = V;
  const vectors = new Map();
  for (const line of fs.readFileSync(vectorPath, 'ascii').trim().split(/\r?\n/)) {
    const [prefix, tag, hex] = line.split(' ');
    assert.equal(prefix, 'WIRE');
    assert.match(hex, /^(?:[0-9a-f]{2})+$/);
    if (!vectors.has(tag)) vectors.set(tag, Buffer.from(hex, 'hex'));
  }

  function read(tag, type) {
    assert(vectors.has(tag), 'missing C vector ' + tag);
    const bytes = vectors.get(tag);
    const packet = vision.decodeBinary(bytes);
    assert.equal(packet.type, type, tag);
    assert.equal(packet.wire_bytes, bytes.length, tag);
    return packet;
  }

  const event = read('event', 2);
  assert.equal(vectors.get('event').toString('hex'), '04010201020203e80301051e048ccf00');
  assert.equal(event.seq, 1);
  assert.equal(event.tick, 1000);
  assert.equal(event.data.step, 30);
  assert.equal(event.data.reason, 4);

  const expected = vectors.get('event');
  const damaged = Buffer.from(expected);
  damaged[14] ^= 1;
  assert.throws(() => vision.decodeBinary(damaged), error => error.code === 'crc');
  const wrongLength = Buffer.from(expected);
  wrongLength[5] = 3;
  assert.throws(() => vision.decodeBinary(wrongLength), /长度/);
  const missingByte = Buffer.concat([expected.subarray(0, 8), expected.subarray(9)]);
  const decoded = [], errors = [];
  const stream = new vision.BinaryStream(value => decoded.push(value), error => errors.push(error.code));
  for (const byte of expected) stream.push(Uint8Array.of(byte));
  stream.push(Buffer.concat([damaged, missingByte, expected]));
  stream.push(Buffer.concat([Buffer.alloc(480, 1), expected, expected]));
  assert.equal(decoded.length, 3); // The oversized packet consumes one delimiter before recovery.
  assert.deepEqual(errors, ['crc', 'format', 'oversize']);

  const live = read('live', 1).data;
  assert.equal(live.wire_version, 9);
  assert.equal(live.grip, 120);
  assert.equal(live.wire_format, 'uart5-bin-v1');
  assert.equal(live.source, 'upper');
  assert.equal(live.frame, 42);
  assert.equal(live.age_ms, 40);
  assert.equal(live.field.robot.y, 74.5);
  assert.equal(live.field.observed.y, 72.5);
  assert(live.field.aligned);
  assert(live.objects[0].position_present);
  assert.equal(live.objects[0].valid, 0);
  assert.equal(live.objects[0].raw_x_mm, 251);
  assert.equal(live.objects[0].raw_y_mm, 432);
  assert.equal(live.objects[0].x_cm, 25.1);
  assert.equal(live.field.points[0].track_id, 0);
  assert.equal(live.field.points[0].radius, 6.5);
  assert(live.objects[1].xy_derived);
  assert(!live.objects[1].position_present);
  assert.equal(live.objects[1].angle, -30);
  assert.equal(live.objects[2].x_cm, null);
  assert.equal(live.lower.objects[0].zone_kind, 6);
  assert.equal(live.lower.objects[0].in_roi, 1);
  assert.equal(live.lower.objects[0].object_inside, 1);

  const full = read('full', 1).data;
  assert.equal(full.objects.length, 12);
  assert.equal(full.lower.objects.length, 12);
  assert.equal(full.objects[0].angle, 0);
  assert.equal(full.objects[0].angle_present, 1);
  assert.equal(full.objects[0].d_mm, 65535);
  assert.equal(full.objects[0].raw_x_mm, -65535);
  assert.equal(full.objects[0].raw_y_mm, 65535);
  assert.equal(full.lower.objects[11].zone_kind, 8);
  assert.equal(full.lower.objects[11].object_inside, 1);

  const heartbeat = read('heartbeat', 1);
  assert.equal(heartbeat.wire_bytes, 39);
  assert.equal(heartbeat.data.source, 'state');
  assert(!heartbeat.data.upper_present && !heartbeat.data.lower_present);
  assert.equal(heartbeat.data.frame, null);
  assert.equal(heartbeat.data.imu.raw_yaw, 360);
  assert.equal(heartbeat.data.imu.age_ms, 145279);

  const info = read('info', 5).data;
  assert.equal(info.extrinsic.y, 0);
  assert.equal(info.origin.x, 285);
  const history = read('history', 4).data;
  assert.equal(history.count, 48);
  assert.equal(history.track.track_id, 48);
  assert.equal(history.track.history.length, 8);
  assert(history.track.history.every(sample => sample.accepted));
  const sync = read('sync_item', 11).data;
  assert.equal(sync.phase, 1);
  assert.equal(sync.index, 0);
  assert.equal(sync.total, 48);
  assert.equal(sync.track.track.track_id, 1);
  assert.equal(sync.track.track.history.length, 0);
  const stats = read('stats', 6).data;
  assert.equal(stats.count, 48);
  assert(stats.tx > 0 && stats.tx_high_water > 0);
  assert.equal(read('track', 3).data.track.history.length, 0);
  assert.equal(read('ack', 8).data.id, 857);
  assert.equal(read('ack', 8).data.status, 0);
  assert.equal(read('ack', 8).data.step, 0);
  const lost = read('lost', 2);
  const restore = read('restore', 2);
  assert.equal(lost.data.step, restore.data.step);
  assert.equal(lost.data.reason, 4);
  assert.equal(restore.data.reason, 0);
  assert.equal(restore.tick - lost.tick, 1);
  const steer = read('steer', 7).data;
  assert.equal(steer.kp, 2.5);
  assert.equal(steer.ki, 0.5);
  assert(Math.abs(steer.kd - 0.12) < 1e-7);
  console.log('PASS: C encoder to page decoder, exact EVENT, single-byte/concatenated/split startup, CRC, bad length, missing byte, oversize recovery, optional XY, full arrays, INFO/TRACK/HISTORY/SYNC/STATS/STEER/ACK');
}

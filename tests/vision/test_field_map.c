#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../../../project/Core/Src/field_map.c"

VisionResult vision;
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
static VisionObservation o;
static void observation(uint32_t now, uint32_t age, int distance, float angle)
{
    static uint32_t frame;
    memset(&o, 0, sizeof(o));
    o.updated_ms=now; o.age_ms=age; o.frame_id=++frame;
    o.fresh=1; o.status=VISION_OBS_OK; o.count=1;
    o.objects[0].kind=VISION_GREEN;
    o.objects[0].distance_mm=distance;
    o.objects[0].angle=angle; o.objects[0].angle_present=1;
}
static void near(float a, float b) { assert(fabsf(a-b)<0.002f); }
static void pair_frame(uint32_t now, float x0, float x1, float distance, unsigned kind, unsigned count)
{
    unsigned i;
    memset(&o,0,sizeof(o));o.count=count;
    field_map.observed.tick=now;
    for(i=0;i<count;i++) {
        memset(&field_map.points[i],0,sizeof(FieldPoint));
        field_map.points[i].present=1;field_map.points[i].x=i?x1:x0;
        field_map.points[i].y=100;
        o.objects[i].kind=(VisionKind)kind;o.objects[i].distance_mm=(int32_t)(distance*10);
    }
    associate(&o);
}
static void tracking_tests(void)
{
    uint32_t a,b;
    unsigned i;
    field_map_reset(0);
    pair_frame(100,0,10,100,VISION_GREEN,2);
    a=field_map.points[0].track_id;b=field_map.points[1].track_id;
    assert(a && b && a!=b);near(field_map.points[0].radius,9);
    pair_frame(200,7,0,100,VISION_GREEN,2);
    assert(field_map.points[0].track_id==b && field_map.points[1].track_id==a);
    near(field_map.points[0].x,7); /* raw observation, no smoothing */
    pair_frame(300,0,0,100,VISION_BLACK,1);
    assert(field_map.points[0].track_id!=a && field_map.points[0].track_id!=b);
    pair_frame(2200,0,0,100,VISION_GREEN,1);assert(field_map.points[0].track_id==a);
    pair_frame(120001,0,0,100,VISION_GREEN,1);assert(field_map.points[0].track_id==a);
    field_map_reset(0);pair_frame(10,0,0,50,VISION_GREEN,1);a=field_map.points[0].track_id;
    near(field_map.points[0].radius,6.5f);
    pair_frame(20,10,0,50,VISION_GREEN,1);assert(field_map.points[0].track_id!=a);
    field_map_reset(0);pair_frame(10,0,0,200,VISION_GREEN,1);a=field_map.points[0].track_id;
    near(field_map.points[0].radius,14);
    pair_frame(20,10,0,50,VISION_GREEN,1);assert(field_map.points[0].track_id==a); /* max(old,new) */
    field_map_reset(0);pair_frame(10,0,1,50,VISION_GREEN,2);
    assert(field_map.points[0].track_id!=field_map.points[1].track_id); /* no same-frame merge */
    pair_frame(20,0.2f,0.3f,50,VISION_GREEN,2);
    assert(field_map.points[0].track_id!=field_map.points[1].track_id); /* overlapping circles */
    field_map_reset(0);
    for(i=0;i<FIELD_TRACK_CAPACITY+1;i++)pair_frame(i+1,i*20.0f,0,50,VISION_GREEN,1);
    assert(field_map.points[0].track_id==0 && field_map.track_count==48 && field_map.unmapped_count==1);
    assert(field_tracks[0].id==1 && field_tracks[47].id==48); /* Never evict. */
    field_map_reset(0xfffffff0U);pair_frame(0xfffffff0U,0,0,50,VISION_GREEN,1);
    a=field_map.points[0].track_id;pair_frame(100,1,0,50,VISION_GREEN,1);
    assert(field_map.points[0].track_id==a);
    /* Actual update: a repeated camera frame must not refresh last observation time. */
    field_map_reset(1000);field_map_record(1100,0,0,0);
    observation(1100,0,500,0);o.objects[0].kind=VISION_GREEN;field_map_update(&o,1100);
    a=field_map.points[0].track_id;assert(field_tracks[0].seen_ms==1100);
    o.updated_ms=1200;o.age_ms=100;field_map_update(&o,1200);
    assert(field_map.points[0].track_id==a && field_tracks[0].seen_ms==1100);
    o.frame_id++;o.status=VISION_OBS_EMPTY;o.count=0;field_map_update(&o,1250);
    field_map_record(1300,0,0,0);observation(1300,0,500,0);o.objects[0].kind=VISION_GREEN;
    field_map_update(&o,1300);assert(field_map.points[0].track_id==a);
    puts("PASS: stable IDs under reordered detections, same-class global nearest one-to-one pairing, overlapping circles, no same-frame merge, range-dependent gates, whole-round reacquisition/no expiry, no smoothing, no ID recycling, duplicate timestamps and wrap");
}
static void memory_tests(void)
{
    unsigned i;uint32_t rev,epoch;
    FieldTrack *t;
    field_map_reset(0);epoch=field_map.epoch;
    for(i=0;i<12;++i)pair_frame(10+i*20,(float)(i%3)*0.1f,0,100,VISION_GREEN,1);
    t=&field_tracks[0];assert(t->observations==12 && t->history_count==8);
    assert(t->history[t->history_next].tick==90);
    near(t->x,0.2f); /* Raw accepted position, not an average. */
    rev=t->revision;
    tick=5000;vision.upper=o;vision.upper.fresh=0;field_map_process();
    assert(t->id==1 && t->observations==12 && !t->visible && t->revision>rev);
    pair_frame(100000,0,0,100,VISION_GREEN,1);assert(t->id==1 && t->observations==13);
    /* In-gate displacement: keep the old accepted position until supported. */
    pair_frame(100020,6,0,100,VISION_GREEN,1);
    assert(t->review && t->pending_count==1);near(t->x,0);
    assert(!t->history[(t->history_next+7)%8].accepted);
    pair_frame(100040,6.2f,0,100,VISION_GREEN,1);assert(t->pending_count==2);near(t->x,0);
    pair_frame(100060,6.1f,0,100,VISION_GREEN,1);
    near(t->x,6.1f);assert(!t->review && !t->pending_count && t->position_ms==100060);
    pair_frame(100080,0,0,100,VISION_GREEN,1);assert(t->review);
    pair_frame(100100,0,0,100,VISION_GREEN,0);assert(!t->visible && !t->pending_count);
    pair_frame(100120,0,0,100,VISION_GREEN,1);assert(t->pending_count==1);near(t->x,6.1f);
    pair_frame(100140,6,0,100,VISION_GREEN,1);assert(!t->review);near(t->x,6);
    pair_frame(100160,60,0,100,VISION_GREEN,1);assert(field_map.track_count==2 && field_tracks[0].id==1 && field_tracks[1].id==2);
    field_map_gap();assert(field_map.track_count==2 && !t->visible);
    field_map_reset(100180);assert(!field_map.track_count && field_map.epoch==epoch+1 && !field_tracks[0].id);
    /* Scheduler updates map without any telemetry call; duplicate polls count once. */
    field_map_record(100200,0,0,0);observation(100200,0,1000,0);vision.upper=o;tick=100200;
    field_map_process();rev=field_map.revision;assert(field_map.track_count==1);
    for(i=0;i<20;++i){tick+=5;vision.upper.updated_ms=tick;vision.upper.age_ms=tick-100200;field_map_process();}
    assert(field_map.revision==rev && field_tracks[0].observations==1 && field_tracks[0].seen_ms==100200);
    tick=100600;field_map_process();assert(field_tracks[0].visible);
    tick=100601;field_map_process();assert(!field_tracks[0].visible && field_map.track_count==1);
    field_map_gap();assert(field_map.track_count==1);
    field_map_record(100620,0,0,0);tick=100620;vision.upper.updated_ms=tick;vision.upper.age_ms=420;
    field_map_process();assert(field_tracks[0].observations==1);
    puts("PASS: independent map update, persistent records through expiry/gaps, bounded chronological raw history, new-frame-only counts, three-frame jump acceptance, missing-frame reset, out-of-gate separate ID, reset epoch and full-capacity retention");
}
int main(void)
{
    unsigned i;
    FieldPose p;
    tracking_tests();
    memory_tests();
    field_map_reset(1000);
    near(field_map.robot.x,285); near(field_map.robot.y,80);
    field_map_record(990,0,0,0); assert(field_map.robot.tick==1000);
    /* Moving 50cm/s: observation at 1100, received at 1300. */
    field_map_record(1200,0,10,0); field_map_record(1400,0,20,0);
    observation(1300,200,500,0); field_map_update(&o,1350);
    assert(field_map.aligned); near(field_map.observed.y,85);
    near(field_map.points[0].x,285); near(field_map.points[0].y,135);
    /* Updated age + timestamp from VSP_Get still represents the same instant. */
    o.updated_ms=1350; o.age_ms=250; field_map_update(&o,1360);
    near(field_map.points[0].y,135);
    observation(1400,0,350,0); field_map_update(&o,1400);
    near(field_map.points[0].y,135); /* same static block after advancing */
    field_map_record(1600,0,20,90*DEG_RAD);
    observation(1600,0,350,-90); field_map_update(&o,1600);
    near(field_map.points[0].x,285); near(field_map.points[0].y,135);
    /* Interpolation follows continuous yaw across the display's 360-degree wrap. */
    field_map_reset(2000); field_map_record(2100,0,0,350*DEG_RAD);
    field_map_record(2300,0,0,370*DEG_RAD);
    assert(pose_at(2200,&p)); near(p.yaw,360*DEG_RAD);
    observation(2300,100,1000,0); field_map_update(&o,2300);
    near(field_map.points[0].x,285); near(field_map.points[0].y,180);
    field_map_update(&o,2600); assert(field_map.aligned); /* 400ms valid */
    field_map_update(&o,2601); assert(!field_map.aligned && !field_map.count);
    observation(2400,0,100,0); field_map_update(&o,2400);
    assert(!field_map.aligned); /* future of history: wait, do not extrapolate */
    field_map_record(2400,0,0,0); field_map_update(&o,2400); assert(field_map.aligned);
    observation(2400,0,-1,0); field_map_update(&o,2400); assert(!field_map.points[0].present);
    observation(2400,0,100,0); o.objects[0].angle_present=0;
    field_map_update(&o,2400); assert(!field_map.points[0].present);
    observation(2400,0,100,0); o.status=VISION_OBS_EMPTY; o.count=0;
    field_map_update(&o,2400); assert(!field_map.aligned && !field_map.count);
    observation(2400,UINT32_MAX,100,0); field_map_update(&o,2400); assert(!field_map.aligned);
    field_map_reset(3000);
    observation(3000,1,100,0); field_map_update(&o,3000); assert(!field_map.aligned);
    for(i=1;i<=80;i++)field_map_record(3000+i*20,0,(float)i,0);
    assert(!pose_at(3000,&p)); assert(pose_at(4500,&p)); near(p.y,155);
    field_map_reset(0xfffffff0U); field_map_record(24,0,20,0);
    observation(24,20,100,0); field_map_update(&o,24);
    assert(field_map.aligned); near(field_map.points[0].y,100);
    puts("PASS: field origin, manual-motion coordinates, observation-time interpolation, static target during translation/rotation, duplicate age, yaw/tick wrap, history boundaries, unknown coordinates, reset and 400/401ms expiry");
    return 0;
}

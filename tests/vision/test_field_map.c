#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../../../project/Core/Src/field_map.c"

VisionResult vision;
static uint32_t tick, frame;
uint32_t HAL_GetTick(void) { return tick; }
static VisionObservation o;
static void observation(uint32_t now, uint32_t age, int distance, float angle)
{
    memset(&o, 0, sizeof(o));
    o.updated_ms=now; o.age_ms=age; o.frame_id=++frame;
    o.fresh=1; o.status=VISION_OBS_OK; o.count=1;
    o.objects[0].kind=VISION_GREEN;
    o.objects[0].distance_mm=distance;
    o.objects[0].angle=angle; o.objects[0].angle_present=1;
}
static void observation_xy(uint32_t now, uint32_t age, int y_mm)
{
    observation(now,age,y_mm,0);
    o.objects[0].position_present=1;
    o.objects[0].y_mm=y_mm;
    o.objects[0].angle_present=0;
}
static void near(float a, float b) { assert(fabsf(a-b)<0.002f); }
static void pair_frame(uint32_t now, float x0, float x1, float distance, unsigned kind, unsigned count)
{
    unsigned i;
    memset(&o,0,sizeof(o));o.count=count;o.frame_id=++frame;
    field_map.observed.tick=now;
    for(i=0;i<count;i++) {
        memset(&field_map.points[i],0,sizeof(FieldPoint));
        field_map.points[i].present=1;field_map.points[i].x=i?x1:x0;
        field_map.points[i].y=100;
        o.objects[i].kind=(VisionKind)kind;o.objects[i].distance_mm=(int32_t)(distance*10);
        o.objects[i].position_present=1;
    }
    associate(&o);
}
static void seed_pair(float x0, float x1)
{
    pair_frame(100,x0,x1,100,VISION_GREEN,2);
    pair_frame(200,x0,x1,100,VISION_GREEN,2);
    assert(!field_map.track_count);
    pair_frame(300,x0,x1,100,VISION_GREEN,2);
    assert(field_map.track_count==2);
    assert(field_map.points[0].track_id==1 && field_map.points[1].track_id==2);
}
static void tracking_tests(void)
{
    unsigned i,j;
    field_map_reset(0);
    seed_pair(0,9);
    /* Nearest-first would take (1,0), leaving -8 unable to use the track at 9. */
    pair_frame(400,1,-8,100,VISION_GREEN,2);
    assert(field_map.points[0].track_id==2 && field_map.points[1].track_id==1);
    near(field_map.points[0].x,1); /* The current point stays raw. */
    near(field_tracks[1].x,7);
    near(field_tracks[0].x,-2);

    field_map_reset(0);
    seed_pair(0,6);
    /* Both assignments have two pairs; nearest-first has squared cost 65 vs 29. */
    pair_frame(400,1,-2,100,VISION_GREEN,2);
    assert(field_map.points[0].track_id==2 && field_map.points[1].track_id==1);
    pair_frame(500,0,0,100,VISION_BLACK,1);
    assert(!field_map.points[0].track_id && field_map.track_count==2);
    pair_frame(600,0,0,100,VISION_BLACK,1);
    pair_frame(700,0,0,100,VISION_BLACK,1);
    assert(field_map.points[0].track_id==3 && field_map.track_count==3);
    pair_frame(120001,0,0,100,VISION_GREEN,1);
    assert(field_map.points[0].track_id==1); /* Confirmed objects survive long gaps. */

    field_map_reset(0);
    /* The candidate stage must also prioritize the maximum number of matches. */
    pair_frame(100,0,9,100,VISION_GREEN,2);
    pair_frame(200,1,-8,100,VISION_GREEN,2);
    assert(!field_map.track_count);
    pair_frame(300,1,-8,100,VISION_GREEN,2);
    assert(field_map.track_count==2);
    near(field_tracks[field_map.points[0].track_id-1U].x,11.0f/3.0f);
    near(field_tracks[field_map.points[1].track_id-1U].x,-16.0f/3.0f);

    field_map_reset(0);
    seed_pair(0,1); /* Overlapping gates cannot merge same-frame objects. */
    pair_frame(400,0.2f,0.3f,50,VISION_GREEN,2);
    assert(field_map.points[0].track_id && field_map.points[1].track_id &&
           field_map.points[0].track_id!=field_map.points[1].track_id);
    near(field_map.points[0].radius,6.5f);

    field_map_reset(0);
    for(i=0;i<=FIELD_TRACK_CAPACITY;++i)
        for(j=0;j<3;++j)
            pair_frame(1+i*3+j,i*20.0f,0,50,VISION_GREEN,1);
    assert(!field_map.points[0].track_id && field_map.track_count==48 && field_map.unmapped_count==1);
    assert(field_tracks[0].id==1 && field_tracks[47].id==48); /* Never evict. */
    puts("PASS: global maximum-cardinality/minimum-cost assignments for confirmed tracks and candidates, class gates, no same-frame merge, whole-round retention and bounded capacity");
}
static void confirmation_tests(void)
{
    unsigned i;
    field_map_reset(1000);
    observation_xy(1000,0,500);field_map_update(&o,1000);
    o.updated_ms=1050;o.age_ms=50;field_map_update(&o,1050);
    assert(!field_map.track_count && !field_map.points[0].track_id);
    field_map_record(1100,0,0,0);observation_xy(1100,0,500);field_map_update(&o,1100);
    assert(!field_map.track_count && !field_map.points[0].track_id);
    observation(1110,0,500,0);o.status=VISION_OBS_EMPTY;o.count=0;field_map_update(&o,1110);
    for(i=0;i<2;++i) {
        uint32_t now=1200+i*100;
        field_map_record(now,0,0,0);observation_xy(now,0,500);field_map_update(&o,now);
        assert(!field_map.track_count);
    }
    observation_xy(1310,0,500);
    apply_frame(&o,NULL); /* A distinct frame supplied without an aligned pose. */
    for(i=0;i<3;++i) {
        uint32_t now=1400+i*100;
        field_map_record(now,0,0,0);frame+=7;observation_xy(now,0,500);
        field_map_update(&o,now);field_map_update(&o,now);
        assert(field_map.track_count==(i==2));
    }
    assert(field_map.points[0].track_id==1 && field_tracks[0].observations==1);
    assert(field_tracks[0].history_count==1 && field_tracks[0].seen_ms==1600);
    puts("PASS: exactly three distinct supported frames confirm an ID, duplicate polls never count, sequence gaps are allowed, empty/unaligned frames interrupt confirmation");
}
static void confirmation_gap_tests(void)
{
    enum { SAME_EXPIRED, NEW_FAULT, POLAR_ONLY };
    unsigned gap, i;
    for(gap=SAME_EXPIRED;gap<=POLAR_ONLY;++gap) {
        const unsigned frames_to_confirm=gap==SAME_EXPIRED?1U:FIELD_CONFIRM_FRAMES;
        field_map_reset(1000);
        observation_xy(1000,0,500);field_map_update(&o,1000);
        field_map_record(1100,0,0,0);observation_xy(1100,0,500);field_map_update(&o,1100);
        assert(field_map.count==1 && !field_map.track_count);
        switch(gap) {
        case SAME_EXPIRED:
            field_map_update(&o,1500);assert(field_map.aligned); /* Exactly 400 ms. */
            field_map_update(&o,1501);assert(!field_map.aligned && !field_map.count);
            break; /* The consumed frame expires without breaking the sequence. */
        case NEW_FAULT:
            observation_xy(1501,0,500);o.status=VISION_OBS_FAULT;field_map_update(&o,1501);
            assert(!field_map.aligned && !field_map.count);
            break; /* A genuinely new invalid frame breaks confirmation. */
        case POLAR_ONLY:
            field_map_record(1501,0,0,0);observation(1501,0,500,0);field_map_update(&o,1501);
            assert(field_map.aligned && field_map.points[0].present && !field_map.points[0].track_id);
            break; /* A new polar-only frame displays but cannot confirm a candidate. */
        }
        assert(!field_map.track_count);
        for(i=0;i<frames_to_confirm;++i) {
            uint32_t now=1600+i*100;
            field_map_record(now,0,0,0);observation_xy(now,0,500);field_map_update(&o,now);
            assert(field_map.track_count==(i+1U==frames_to_confirm));
        }
        assert(field_map.points[0].track_id==1 && field_tracks[0].observations==1);
    }
    puts("PASS: ageing a consumed frame hides the live observation, while a new invalid or polar-only frame breaks candidate confirmation");
}
static void memory_tests(void)
{
    unsigned i;
    uint32_t rev,epoch;
    FieldTrack *t;
    field_map_reset(0);epoch=field_map.epoch;
    for(i=0;i<3;++i)pair_frame(10+i*10,0,0,100,VISION_GREEN,1);
    t=&field_tracks[0];assert(t->observations==1 && t->history_count==1);
    pair_frame(40,2,0,100,VISION_GREEN,1);
    near(t->x,0.5f);
    near(t->history[(t->history_next+7)%8].x,2.0f);
    assert(t->history[(t->history_next+7)%8].accepted && !t->review);
    for(i=0;i<10;++i)pair_frame(50+i*10,2,0,100,VISION_GREEN,1);
    assert(t->observations==12 && t->history_count==8);
    assert(t->history[t->history_next].tick==70);
    rev=t->revision;
    tick=5000;vision.upper=o;vision.upper.fresh=0;field_map_process();
    assert(t->id==1 && t->observations==12 && !t->visible && t->revision>rev);
    pair_frame(100000,2,0,100,VISION_GREEN,1);
    assert(t->id==1 && t->observations==13);
    field_map_gap();assert(field_map.track_count==1 && !t->visible);
    field_map_reset(100180);assert(!field_map.track_count && field_map.epoch==epoch+1 && !t->id);
    puts("PASS: KF mean differs from raw samples, bounded chronological raw history, long absence preserves confirmed map, reset clears map and epoch");
}
static void covariance_tests(void)
{
    const float measurement_variance=81.0f/5.991464547f;
    const float initial=measurement_variance/3.0f;
    unsigned i;
    field_map_reset(1000);
    for(i=0;i<3;++i) {
        uint32_t now=1100+i*100;
        field_map_record(now,0,0,0);observation_xy(now,0,1000);field_map_update(&o,now);
    }
    assert(field_map.track_count==1);
    near(field_tracks[0].variance,initial);

    field_map_record(1400,0,0,3.0f*DEG_RAD);
    observation(1400,0,1000,0);o.status=VISION_OBS_EMPTY;o.count=0;field_map_update(&o,1400);
    near(field_tracks[0].variance,initial); /* Pure rotation at fixed wheel centre. */

    field_map_record(1500,3,0,3.0f*DEG_RAD);
    field_map_record(1600,3,4,3.0f*DEG_RAD);
    observation(1600,0,1000,0);o.status=VISION_OBS_EMPTY;o.count=0;
    field_map_update(&o,1600);field_map_update(&o,1600);
    near(field_tracks[0].variance,initial+7.0f*0.09f);

    field_map_correct_pose(123,456,30.0f*DEG_RAD);
    field_map_record(1700,3,4,3.0f*DEG_RAD);
    observation(1700,0,1000,0);o.status=VISION_OBS_EMPTY;o.count=0;field_map_update(&o,1700);
    near(field_tracks[0].variance,initial+7.0f*0.09f);
    near(field_tracks[0].x,FIELD_START_X_CM);
    near(field_tracks[0].y,FIELD_START_Y_CM+100.0f);
    puts("PASS: position variance grows by 0.09 cm^2 per centimetre of wheel-centre translation, neither pure rotation nor pose correction adds travelled distance");
}
static void zero_extrinsic_tests(void)
{
    const FieldPose poses[] = {
        {1100, 100.0f, 100.0f, 0.0f},
        {2100, 70.0f, 130.0f, 90.0f*DEG_RAD},
        {3100, 70.0f, 130.0f, 90.0f*DEG_RAD},
    };
    unsigned direct, i;
    for (direct=0; direct<2; ++direct) {
        field_map_reset(1000);
        field_map_correct_pose(0,0,0);
        for (i=0; i<sizeof(poses)/sizeof(poses[0]); ++i) {
            const FieldPose *p=&poses[i];
            const uint32_t now=p->tick+100U;
            field_map_record(p->tick,p->x,p->y,p->yaw);
            /* Delivery occurs after the vehicle has moved and turned again. */
            field_map_record(now,p->x+1.0f,p->y+0.5f,p->yaw+5.0f*DEG_RAD);
            if (direct) observation_xy(now,100U,300);
            else observation(now,100U,300,0);
            field_map_update(&o,now);
            assert(field_map.aligned && field_map.count==1 && field_map.points[0].present);
            assert(field_map.observed.tick==p->tick);
            near(field_map.points[0].x,100.0f);
            near(field_map.points[0].y,130.0f);
            assert(field_map.points[0].track_id==(direct && i==2));
        }
        assert(field_map.track_count==direct && field_tracks[0].observations==direct);
        if (direct) {
            const float variance=field_tracks[0].variance;
            /* A polar-only observation near the existing track must leave the KF untouched. */
            field_map_record(3300,71.0f,130.5f,95.0f*DEG_RAD);
            observation(3300,0,300,0);field_map_update(&o,3300);
            assert(field_map.points[0].present && !field_map.points[0].track_id);
            assert(field_tracks[0].observations==1 && !field_tracks[0].visible);
            near(field_tracks[0].x,100.0f);near(field_tracks[0].y,130.0f);
            near(field_tracks[0].variance,variance);
        }
    }
    puts("PASS: zero extrinsic XY and polar display at (100,130) cm across delayed poses; only original XY confirms or updates the KF");
}

static void near_object_tests(void)
{
    unsigned i;
    FieldTrack *track=&field_tracks[0];

    field_map_reset(1000);
    tick=1100;
    field_map_record(tick,0,0,0);
    observation_xy(tick,0,120);
    o.count=6;
    for (i=0; i<4; ++i) {
        o.objects[i]=o.objects[0];
        o.objects[i].kind=(VisionKind)(VISION_GREEN+i);
    }
    o.objects[2].distance_mm=-1;
    o.objects[2].x_mm=72;
    o.objects[2].y_mm=96; /* XY-only radial distance is exactly 120 mm. */
    o.objects[3].distance_mm=119;
    o.objects[4]=o.objects[0];
    o.objects[4].distance_mm=o.objects[4].y_mm=121;
    o.objects[5]=o.objects[0];
    o.objects[5].kind=VISION_BLUE_MATERIAL_ZONE;
    vision.upper=o;
    field_map_process();
    assert(!memcmp(&vision.upper,&o,sizeof(o)));
    assert(field_map.aligned && field_map.count==6);
    for (i=0; i<4; ++i) assert(!field_map.points[i].present);
    assert(field_map.points[4].present && field_map.points[5].present);

    field_map_reset(1100);
    for (i=0; i<FIELD_CONFIRM_FRAMES; ++i) {
        tick=1200+i*10;
        field_map_record(tick,0,0,0);
        observation_xy(tick,0,121);
        field_map_update(&o,tick);
    }
    assert(field_map.track_count==1 && track->id==1 && track->observations==1);
    ++tick;
    field_map_record(tick,0,0,0);
    observation_xy(tick,0,120);
    field_map_update(&o,tick);
    assert(!field_map.points[0].present && !field_map.points[0].track_id);
    assert(field_map.track_count==1 && track->id==1 && !track->visible);
    assert(track->observations==1 && track->history_count==1);
    near(track->x,FIELD_START_X_CM);
    near(track->y,FIELD_START_Y_CM+12.1f);

    observation(tick,0,120,0);
    field_map_update(&o,tick);
    assert(!field_map.points[0].present);
    observation(tick,0,121,0);
    field_map_update(&o,tick);
    assert(field_map.points[0].present);
    observation_xy(tick,0,121);
    field_map_update(&o,tick);
    assert(field_map.points[0].track_id==1 && track->observations==2);
    puts("PASS: 12 cm inclusive upper-object filter, XY and polar inputs, safety zone and raw frame preserved, existing track position and ID retained");
}

int main(void)
{
    unsigned i;
    FieldPose p;
    const float fixed_y=FIELD_START_Y_CM+55.0f;
    const float turn=atan2f(3.0f,4.0f);
    const float wheel_y_after_translation=FIELD_START_Y_CM+20.0f;
    tracking_tests();
    confirmation_tests();
    confirmation_gap_tests();
    memory_tests();
    covariance_tests();
    zero_extrinsic_tests();
    near_object_tests();
    field_map_reset(1000);
    near(field_map.robot.x,285); near(field_map.robot.y,FIELD_START_Y_CM);
    field_map_record(990,0,0,0); assert(field_map.robot.tick==1000);
    /* Moving 50cm/s: observation at 1100, received at 1300. */
    field_map_record(1200,0,10,0); field_map_record(1400,0,20,0);
    /* Infer each camera reading from one fixed field point, 55 cm ahead of start. */
    observation_xy(1300,200,(int)lroundf((fixed_y-(FIELD_START_Y_CM+5))*10));
    field_map_update(&o,1350);
    assert(field_map.aligned); near(field_map.observed.y,FIELD_START_Y_CM+5);
    near(field_map.points[0].x,285); near(field_map.points[0].y,fixed_y);
    assert(!field_map.points[0].track_id);
    /* Updated age + timestamp from VSP_Get still represents the same instant. */
    o.updated_ms=1350; o.age_ms=250; field_map_update(&o,1360);
    near(field_map.points[0].y,fixed_y);
    observation_xy(1400,0,(int)lroundf((fixed_y-wheel_y_after_translation)*10));
    field_map_update(&o,1400);
    near(field_map.points[0].y,fixed_y);
    /* The wheel centre stays fixed while the camera turns. Invert the
     * rotation to obtain raw camera XY for the same static field point. */
    field_map_record(1600,0,20,turn);
    observation(1600,0,-1,0);
    o.objects[0].position_present=1;
    o.objects[0].x_mm=(int32_t)lroundf(-(fixed_y-wheel_y_after_translation)*sinf(turn)*10);
    o.objects[0].y_mm=(int32_t)lroundf((fixed_y-wheel_y_after_translation)*cosf(turn)*10);
    field_map_update(&o,1600);
    near(field_map.points[0].x,285); near(field_map.points[0].y,fixed_y);
    assert(field_map.points[0].track_id==1 && field_map.track_count==1 && field_tracks[0].observations==1);
    /* Interpolation follows continuous yaw across the display's 360-degree wrap. */
    field_map_reset(2000); field_map_record(2100,0,0,350*DEG_RAD);
    field_map_record(2300,0,0,370*DEG_RAD);
    assert(field_map_pose_at(2200,&p)); near(p.yaw,360*DEG_RAD);
    observation(2300,100,1000,0); field_map_update(&o,2300);
    near(field_map.points[0].x,285); near(field_map.points[0].y,FIELD_START_Y_CM+100);
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
    assert(!field_map_pose_at(3000,&p)); assert(field_map_pose_at(4500,&p)); near(p.y,FIELD_START_Y_CM+75);
    field_map_reset(0xfffffff0U); field_map_record(24,0,20,0);
    observation(24,20,200,0); field_map_update(&o,24);
    assert(field_map.aligned); near(field_map.points[0].y,FIELD_START_Y_CM+30);
    puts("PASS: field origin, manual-motion coordinates, observation-time interpolation, static target during translation/rotation, duplicate age, yaw/tick wrap, history boundaries, unknown coordinates, reset and 400/401ms expiry");
    return 0;
}

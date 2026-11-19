#include "field_map.h"
#include <math.h>
#include <string.h>

#define HISTORY_SIZE 64U
#define DEG_RAD (3.14159265358979323846f / 180.0f)
FieldMap field_map;
static FieldPose history[HISTORY_SIZE];
static unsigned next_pose, pose_count;
static uint8_t processed;
static uint32_t processed_frame, processed_tick;
FieldTrack field_tracks[FIELD_TRACK_CAPACITY];
static uint32_t next_id;

static float match_radius(float distance)
{
    return FIELD_MATCH_BASE_CM + FIELD_MATCH_DISTANCE_SCALE * distance;
}
static void track_changed(FieldTrack *t)
{
    t->revision = ++field_map.revision;
}
static void hide_tracks(void)
{
    unsigned j;
    for(j=0;j<FIELD_TRACK_CAPACITY;++j) {
        FieldTrack *t=&field_tracks[j];
        if(t->id && (t->visible || t->pending_count)) {
            t->visible=0; t->pending_count=0; track_changed(t);
        }
    }
}
static void remember(FieldTrack *t, const FieldPoint *p, float distance)
{
    FieldSample *sample=&t->history[t->history_next];
    float dx=p->x-t->x, dy=p->y-t->y;
    float radius=match_radius(distance>t->distance?distance:t->distance)*0.5f;
    uint8_t accept=1;
    /* Keep the raw samples. A large in-gate jump needs three consecutive
     * observations near the candidate; never average/freeze away the evidence. */
    if(t->observations && dx*dx+dy*dy>radius*radius) {
        dx=p->x-t->pending_x; dy=p->y-t->pending_y;
        if(!t->pending_count || dx*dx+dy*dy>radius*radius) {
            t->pending_x=p->x; t->pending_y=p->y; t->pending_count=0;
        }
        ++t->pending_count;
        accept=t->pending_count>=FIELD_JUMP_CONFIRM_FRAMES;
    }
    t->seen_ms=field_map.observed.tick;
    if(t->observations!=UINT32_MAX)++t->observations;
    t->visible=1; t->review=!accept;
    if(accept) {
        t->x=p->x; t->y=p->y; t->distance=distance;
        t->position_ms=t->seen_ms; t->pending_count=0;
    }
    sample->tick=t->seen_ms; sample->x=p->x; sample->y=p->y; sample->accepted=accept;
    t->history_next=(uint8_t)((t->history_next+1U)%FIELD_TRACK_HISTORY);
    if(t->history_count<FIELD_TRACK_HISTORY)++t->history_count;
    track_changed(t);
}
static void associate(const VisionObservation *o)
{
    uint8_t used[FIELD_TRACK_CAPACITY] = {0};
    int assignments[12];
    unsigned i,j;
    for(i=0;i<o->count;++i)assignments[i]=-1;
    /* Same-class global nearest pair, one observation and one track per pair.
     * Missing observations NEVER expire or recycle a map identity. */
    for(;;) {
        int bi=-1,bj=-1;
        float best=0;
        for(i=0;i<o->count;++i) {
            const FieldPoint *p=&field_map.points[i];
            float distance=o->objects[i].distance_mm*0.1f;
            if(!p->present || assignments[i]>=0)continue;
            for(j=0;j<FIELD_TRACK_CAPACITY;++j) {
                const FieldTrack *t=&field_tracks[j];
                float dx,dy,d2,radius;
                if(!t->id || used[j] || t->kind!=(uint8_t)o->objects[i].kind)continue;
                if((int32_t)(field_map.observed.tick-t->seen_ms)<0)continue;
                dx=p->x-t->x;dy=p->y-t->y;d2=dx*dx+dy*dy;
                radius=match_radius(distance>t->distance?distance:t->distance);
                if(d2<=radius*radius && (bi<0 || d2<best ||
                    (d2==best && t->id<field_tracks[bj].id))) {
                    best=d2;bi=(int)i;bj=(int)j;
                }
            }
        }
        if(bi<0)break;
        assignments[bi]=bj;used[bj]=1;
    }
    for(i=0;i<o->count;++i) {
        FieldPoint *p=&field_map.points[i];
        int slot=assignments[i];
        FieldTrack *t;
        if(!p->present)continue;
        if(slot<0) {
            for(j=0;j<FIELD_TRACK_CAPACITY;++j)
                if(!field_tracks[j].id) {slot=(int)j;break;}
            if(slot<0 || !next_id) {
                if(field_map.unmapped_count!=UINT32_MAX)++field_map.unmapped_count;
                continue; /* Full: report it, never evict a remembered object. */
            }
            field_tracks[slot].id=next_id++;
            field_tracks[slot].kind=(uint8_t)o->objects[i].kind;
            ++field_map.track_count;
        }
        used[slot]=1;t=&field_tracks[slot];
        remember(t,p,o->objects[i].distance_mm*0.1f);
        p->track_id=t->id;
        p->radius=match_radius(o->objects[i].distance_mm*0.1f);
    }
    for(j=0;j<FIELD_TRACK_CAPACITY;++j) {
        FieldTrack *t=&field_tracks[j];
        if(t->id && !used[j] && (t->visible || t->pending_count)) {
            t->visible=0;t->pending_count=0;track_changed(t);
        }
    }
}

void field_map_record(uint32_t tick, float x, float y, float yaw)
{
    float a = FIELD_START_YAW_DEG * DEG_RAD;
    FieldPose p;
    p.tick = tick;
    p.x = FIELD_START_X_CM + x * cosf(a) + y * sinf(a);
    p.y = FIELD_START_Y_CM - x * sinf(a) + y * cosf(a);
    p.yaw = yaw + a; /* Keep continuous angle for interpolation across 360 degrees. */
    if (pose_count && (int32_t)(tick - field_map.robot.tick) < 0) return;
    field_map.robot = p;
    field_map.heading_valid = 1;
    if (pose_count && history[(next_pose + HISTORY_SIZE - 1U) % HISTORY_SIZE].tick == tick) {
        history[(next_pose + HISTORY_SIZE - 1U) % HISTORY_SIZE] = p;
        return;
    }
    history[next_pose] = p;
    next_pose = (next_pose + 1U) % HISTORY_SIZE;
    if (pose_count < HISTORY_SIZE) ++pose_count;
}

void field_map_reset(uint32_t now)
{
    uint32_t epoch=field_map.epoch+1U;
    memset(&field_map, 0, sizeof(field_map));
    field_map.epoch=epoch?epoch:1U;
    next_pose = pose_count = 0;
    processed = 0;
    memset(field_tracks, 0, sizeof(field_tracks));
    next_id = 1;
    field_map_record(now, 0, 0, 0);
}

void field_map_gap(void)
{
    next_pose = pose_count = 0;
    hide_tracks();
    field_map.aligned = field_map.count = field_map.heading_valid = 0;
}

static int pose_at(uint32_t tick, FieldPose *out)
{
    unsigned i, first;
    if (!pose_count) return 0;
    first = (next_pose + HISTORY_SIZE - pose_count) % HISTORY_SIZE;
    for (i = 0; i < pose_count; ++i) {
        const FieldPose *a = &history[(first + i) % HISTORY_SIZE];
        if (a->tick == tick) { *out = *a; return 1; }
        if (i + 1U < pose_count) {
            const FieldPose *b = &history[(first + i + 1U) % HISTORY_SIZE];
            uint32_t span = b->tick - a->tick, offset = tick - a->tick;
            /* Unsigned differences also handle the HAL tick wrap. */
            if (span && offset < span) {
                float t = (float)offset / (float)span;
                out->tick = tick;
                out->x = a->x + (b->x - a->x) * t;
                out->y = a->y + (b->y - a->y) * t;
                out->yaw = a->yaw + (b->yaw - a->yaw) * t;
                return 1;
            }
        }
    }
    return 0; /* No extrapolation or silent substitution of the current pose. */
}

void field_map_update(const VisionObservation *o, uint32_t now)
{
    uint32_t wait=now-o->updated_ms, observed_tick=o->updated_ms-o->age_ms;
    unsigned i;
    if(!field_map.heading_valid || !o->fresh ||
       (o->status!=VISION_OBS_OK && o->status!=VISION_OBS_EMPTY) ||
       o->age_ms>400U || wait>400U-o->age_ms || o->count>12U) {
        field_map.aligned=field_map.count=0; hide_tracks(); return;
    }
    /* VSP refreshes age/updated_ms on every poll, even for the same frame.
     * Never count those polls as new observations or refresh map timestamps. */
    if(processed && processed_frame==o->frame_id) return;
    if(processed && (int32_t)(observed_tick-processed_tick)<0)return;
    field_map.aligned=field_map.count=0;
    if(o->status==VISION_OBS_EMPTY) {
        hide_tracks();
        field_map.frame=o->frame_id;processed_frame=o->frame_id;
        processed_tick=observed_tick;processed=1;return;
    }
    /* age includes receive/queue time once, but not camera exposure/buffering.
     * Wait for bracketing odometry; never silently use the current pose. */
    if(!pose_at(observed_tick,&field_map.observed))return;
    field_map.frame=processed_frame=o->frame_id;
    field_map.count=o->count;field_map.aligned=processed=1;processed_tick=observed_tick;
    for(i=0;i<o->count;++i) {
        const VisionObject *v=&o->objects[i];
        FieldPoint *p=&field_map.points[i];
        float a,r,x,y,c,s;
        memset(p,0,sizeof(*p));
        if(v->distance_mm<0 || !v->angle_present ||
           !(v->angle>=-180.0f && v->angle<=180.0f))continue;
        a=(v->angle+FIELD_VISION_YAW_DEG)*DEG_RAD;
        r=v->distance_mm*0.1f;
        x=FIELD_VISION_X_CM+r*sinf(a);y=FIELD_VISION_Y_CM+r*cosf(a);
        c=cosf(field_map.observed.yaw);s=sinf(field_map.observed.yaw);
        p->x=field_map.observed.x+x*c+y*s;
        p->y=field_map.observed.y-x*s+y*c;p->present=1;
    }
    associate(o);
}
void field_map_process(void)
{
    field_map_update(&vision.upper,HAL_GetTick());
}

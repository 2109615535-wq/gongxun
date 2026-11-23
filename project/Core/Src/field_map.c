#include "field_map.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define HISTORY_SIZE 64U
#define MATCH_COLUMN_CAPACITY (FIELD_TRACK_CAPACITY + VISION_MAX_OBJECTS)
#define MATCH_MISSED_COST ((VISION_MAX_OBJECTS+1U)*FIELD_ASSOCIATION_GATE)
#define MATCH_FORBIDDEN_COST (2.0f*MATCH_MISSED_COST)
#define DEG_RAD (3.14159265358979323846f / 180.0f)
FieldMap field_map;
static FieldPose history[HISTORY_SIZE];
static unsigned next_pose, pose_count;
static uint8_t processed, awaiting_pose;
static uint32_t processed_frame, processed_tick, awaiting_frame;
FieldTrack field_tracks[FIELD_TRACK_CAPACITY];
static uint32_t next_id;
static float origin_x = FIELD_START_X_CM, origin_y = FIELD_START_Y_CM;
static float origin_yaw = FIELD_START_YAW_DEG * DEG_RAD;
static float translation_x, translation_y;
static float wheel_travel_cm, odometry_travel_cm;

typedef struct {
    float x, y, variance;
    uint8_t kind, hits;
} Candidate;

static Candidate candidates[VISION_MAX_OBJECTS];
static unsigned candidate_count;

/* Fixed workspace reused in both matching passes by the map scheduler. */
static struct {
    float cost[VISION_MAX_OBJECTS][MATCH_COLUMN_CAPACITY];
    float row_price[VISION_MAX_OBJECTS+1U], column_price[MATCH_COLUMN_CAPACITY+1U];
    float slack[MATCH_COLUMN_CAPACITY+1U];
    uint8_t row_for_column[MATCH_COLUMN_CAPACITY+1U], parent[MATCH_COLUMN_CAPACITY+1U];
    uint8_t used[MATCH_COLUMN_CAPACITY+1U];
} matching;

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
    for (j=0; j<field_map.track_count; ++j) {
        FieldTrack *t=&field_tracks[j];
        if (t->visible) {
            t->visible=0;
            track_changed(t);
        }
    }
}

static void hide_observation(void)
{
    field_map.aligned=field_map.count=0;
    hide_tracks();
}

static void clear_observation(void)
{
    candidate_count=0;
    hide_observation();
}

static float range_cm(const VisionObject *object)
{
    if (object->distance_mm>=0) return object->distance_mm*0.1f;
    return hypotf((float)object->x_mm,(float)object->y_mm)*0.1f;
}

static void predict_tracks(void)
{
    const float added_variance=FIELD_MOTION_VARIANCE_PER_CM*wheel_travel_cm;
    unsigned i;
    wheel_travel_cm=0;
    for (i=0; i<field_map.track_count; ++i) field_tracks[i].variance+=added_variance;
    for (i=0; i<candidate_count; ++i) candidates[i].variance+=added_variance;
}

static void update_position(float *x, float *y, float *variance,
                            const FieldPoint *point, float measurement_variance)
{
    float gain=*variance/(*variance+measurement_variance);
    *x+=gain*(point->x-*x);
    *y+=gain*(point->y-*y);
    *variance*=(1.0f-gain);
}

static void remember(FieldTrack *t, const FieldPoint *p, float distance)
{
    FieldSample *sample=&t->history[t->history_next];
    t->position_ms=t->seen_ms=field_map.observed.tick;
    if (t->observations!=UINT32_MAX) ++t->observations;
    t->visible=1;
    t->distance=distance;
    sample->tick=t->seen_ms;
    sample->x=p->x;
    sample->y=p->y;
    sample->accepted=1;
    t->history_next=(uint8_t)((t->history_next+1U)%FIELD_TRACK_HISTORY);
    if (t->history_count<FIELD_TRACK_HISTORY) ++t->history_count;
    track_changed(t);
}

/* Rectangular Hungarian assignment. Each row can use any dummy column;
 * missing a match costs more than all feasible pair costs combined. */
static void solve_matching(unsigned rows, unsigned columns)
{
    unsigned i, j, col, next_col, row, previous;
    float delta, reduced;
    memset(matching.row_price,0,sizeof(matching.row_price));
    memset(matching.column_price,0,sizeof(matching.column_price));
    memset(matching.row_for_column,0,sizeof(matching.row_for_column));
    for (i=1; i<=rows; ++i) {
        matching.row_for_column[0]=(uint8_t)i;
        memset(matching.used,0,sizeof(matching.used));
        for (j=1; j<=columns; ++j) matching.slack[j]=FLT_MAX;
        col=0;
        do {
            matching.used[col]=1;
            row=matching.row_for_column[col];
            delta=FLT_MAX;
            next_col=0;
            for (j=1; j<=columns; ++j) {
                if (matching.used[j]) continue;
                reduced=matching.cost[row-1U][j-1U]-matching.row_price[row]-matching.column_price[j];
                if (reduced<matching.slack[j]) {
                    matching.slack[j]=reduced;
                    matching.parent[j]=(uint8_t)col;
                }
                if (matching.slack[j]<delta) {
                    delta=matching.slack[j];
                    next_col=j;
                }
            }
            for (j=0; j<=columns; ++j) {
                if (matching.used[j]) {
                    matching.row_price[matching.row_for_column[j]]+=delta;
                    matching.column_price[j]-=delta;
                } else {
                    matching.slack[j]-=delta;
                }
            }
            col=next_col;
        } while (matching.row_for_column[col]);

        do {
            previous=matching.parent[col];
            matching.row_for_column[col]=matching.row_for_column[previous];
            col=previous;
        } while (col);
    }
}

static void match_points(const VisionObservation *o, const float *variances,
                         const int *confirmed, int *assignments)
{
    unsigned i, j, rows=0;
    unsigned tracks=confirmed?candidate_count:field_map.track_count;
    unsigned row_to_point[VISION_MAX_OBJECTS];
    if (!tracks) return;
    for (i=0; i<o->count; ++i) {
        const FieldPoint *p=&field_map.points[i];
        if (!p->present || !o->objects[i].position_present || (confirmed && confirmed[i]>=0)) continue;
        row_to_point[rows]=i;
        for (j=0; j<tracks; ++j) {
            float x, y, variance, dx, dy, distance_squared;
            uint8_t kind;
            if (confirmed) {
                const Candidate *c=&candidates[j];
                x=c->x; y=c->y; variance=c->variance; kind=c->kind;
            } else {
                const FieldTrack *t=&field_tracks[j];
                x=t->x; y=t->y; variance=t->variance; kind=t->kind;
            }
            matching.cost[rows][j]=MATCH_FORBIDDEN_COST;
            if (kind!=(uint8_t)o->objects[i].kind) continue;
            dx=p->x-x; dy=p->y-y;
            distance_squared=(dx*dx+dy*dy)/(variance+variances[i]);
            if (distance_squared<=FIELD_ASSOCIATION_GATE)
                matching.cost[rows][j]=distance_squared;
        }
        ++rows;
    }
    if (!rows) return;
    for (i=0; i<rows; ++i)
        for (j=tracks; j<tracks+rows; ++j) matching.cost[i][j]=MATCH_MISSED_COST;

    solve_matching(rows,tracks+rows);
    for (j=1; j<=tracks; ++j) {
        unsigned row=matching.row_for_column[j];
        if (row && matching.cost[row-1U][j-1U]<=FIELD_ASSOCIATION_GATE)
            assignments[row_to_point[row-1U]]=(int)(j-1U);
    }
}

static void associate(const VisionObservation *o)
{
    uint8_t used[FIELD_TRACK_CAPACITY]={0};
    int confirmed[VISION_MAX_OBJECTS], tentative[VISION_MAX_OBJECTS];
    float distances[VISION_MAX_OBJECTS], variances[VISION_MAX_OBJECTS];
    Candidate next_candidates[VISION_MAX_OBJECTS];
    unsigned i, j, next_count=0;
    for (i=0; i<o->count; ++i) {
        FieldPoint *p=&field_map.points[i];
        confirmed[i]=tentative[i]=-1;
        if (!p->present) continue;
        distances[i]=range_cm(&o->objects[i]);
        p->radius=match_radius(distances[i]); /* Measurement reference radius for V8. */
        variances[i]=p->radius*p->radius/FIELD_ASSOCIATION_GATE;
        p->track_id=0;
    }
    match_points(o,variances,NULL,confirmed);
    match_points(o,variances,confirmed,tentative);

    for (i=0; i<o->count; ++i) {
        FieldPoint *p=&field_map.points[i];
        FieldTrack *t;
        unsigned slot;
        if (!p->present || !o->objects[i].position_present) continue;
        if (confirmed[i]>=0) {
            slot=(unsigned)confirmed[i];
            t=&field_tracks[slot];
            update_position(&t->x,&t->y,&t->variance,p,variances[i]);
        } else {
            Candidate c={p->x,p->y,variances[i],(uint8_t)o->objects[i].kind,1};
            if (tentative[i]>=0) {
                c=candidates[tentative[i]];
                update_position(&c.x,&c.y,&c.variance,p,variances[i]);
                ++c.hits;
            }
            if (c.hits<FIELD_CONFIRM_FRAMES) {
                next_candidates[next_count++]=c;
                continue;
            }
            if (field_map.track_count>=FIELD_TRACK_CAPACITY || !next_id) {
                if (field_map.unmapped_count!=UINT32_MAX) ++field_map.unmapped_count;
                continue;
            }
            slot=field_map.track_count++;
            t=&field_tracks[slot];
            t->id=next_id++;
            t->kind=c.kind;
            t->x=c.x; t->y=c.y; t->variance=c.variance;
        }
        used[slot]=1;
        p->track_id=t->id;
        remember(t,p,distances[i]);
    }
    candidate_count=next_count;
    memcpy(candidates,next_candidates,next_count*sizeof(candidates[0]));
    for (j=0; j<field_map.track_count; ++j) {
        FieldTrack *t=&field_tracks[j];
        if (!used[j] && t->visible) {
            t->visible=0;
            track_changed(t);
        }
    }
}

static void save_pose(FieldPose p)
{
    if (pose_count && (int32_t)(p.tick - field_map.robot.tick) < 0) return;
    field_map.robot = p;
    field_map.robot.x += translation_x;
    field_map.robot.y += translation_y;
    field_map.heading_valid = 1;
    if (pose_count && history[(next_pose + HISTORY_SIZE - 1U) % HISTORY_SIZE].tick == p.tick) {
        history[(next_pose + HISTORY_SIZE - 1U) % HISTORY_SIZE] = p;
        return;
    }
    history[next_pose] = p;
    next_pose = (next_pose + 1U) % HISTORY_SIZE;
    if (pose_count < HISTORY_SIZE) ++pose_count;
}

void field_map_record(uint32_t tick, float x, float y, float yaw)
{
    float s = sinf(origin_yaw), c = cosf(origin_yaw);
    FieldPose p;
    p.tick = tick;
    p.x = origin_x + x * c + y * s;
    p.y = origin_y - x * s + y * c;
    p.yaw = yaw + origin_yaw; /* Continuous angle for interpolation. */
    /* Only physical wheel-centre translation contributes. A pose correction
     * uses save_pose directly and never adds the coordinate jump. */
    if (pose_count && (int32_t)(tick-field_map.robot.tick)>=0) {
        float travel = hypotf(p.x-(field_map.robot.x-translation_x),
                              p.y-(field_map.robot.y-translation_y));
        wheel_travel_cm += travel;
        odometry_travel_cm += travel;
    }
    save_pose(p);
}

void field_map_correct_pose(float x, float y, float yaw)
{
    FieldPose p = field_map.robot;
    float dx = origin_x + translation_x - p.x, dy = origin_y + translation_y - p.y;
    float delta = atan2f(sinf(yaw - p.yaw), cosf(yaw - p.yaw));
    float s = sinf(delta), c = cosf(delta);
    origin_x = x + dx * c + dy * s;
    origin_y = y - dx * s + dy * c;
    origin_yaw += delta;
    translation_x = translation_y = 0.0f;
    p.x = x;
    p.y = y;
    p.yaw += delta;
    /* Do not interpolate a camera frame across a manual pose correction. */
    field_map_gap();
    save_pose(p);
}

void field_map_translate(float dx, float dy)
{
    /* Keep the interpolation history in the unshifted odometry frame.
     * Every lookup applies the current translation, even for a delayed frame
     * captured before the previous correction reached the controller. */
    translation_x += dx;
    translation_y += dy;
    field_map.robot.x += dx;
    field_map.robot.y += dy;
}

float field_map_travel_cm(void)
{
    return odometry_travel_cm;
}

void field_map_reset(uint32_t now)
{
    uint32_t epoch=field_map.epoch+1U;
    memset(&field_map, 0, sizeof(field_map));
    field_map.epoch=epoch?epoch:1U;
    next_pose = pose_count = 0;
    processed = awaiting_pose = 0;
    candidate_count=0;
    wheel_travel_cm=odometry_travel_cm=0;
    memset(field_tracks, 0, sizeof(field_tracks));
    next_id = 1;
    origin_x = FIELD_START_X_CM;
    origin_y = FIELD_START_Y_CM;
    origin_yaw = FIELD_START_YAW_DEG * DEG_RAD;
    translation_x = translation_y = 0.0f;
    field_map_record(now, 0, 0, 0);
}

void field_map_gap(void)
{
    next_pose = pose_count = 0;
    awaiting_pose=0;
    clear_observation();
    field_map.heading_valid=0;
}

int field_map_pose_at(uint32_t tick, FieldPose *out)
{
    unsigned i, first;
    if (!pose_count) return 0;
    first = (next_pose + HISTORY_SIZE - pose_count) % HISTORY_SIZE;
    for (i = 0; i < pose_count; ++i) {
        const FieldPose *a = &history[(first + i) % HISTORY_SIZE];
        if (a->tick == tick) {
            *out = *a;
            out->x += translation_x;
            out->y += translation_y;
            return 1;
        }
        if (i + 1U < pose_count) {
            const FieldPose *b = &history[(first + i + 1U) % HISTORY_SIZE];
            uint32_t span = b->tick - a->tick, offset = tick - a->tick;
            /* Unsigned differences also handle the HAL tick wrap. */
            if (span && offset < span) {
                float t = (float)offset / (float)span;
                out->tick = tick;
                out->x = translation_x + a->x + (b->x - a->x) * t;
                out->y = translation_y + a->y + (b->y - a->y) * t;
                out->yaw = a->yaw + (b->yaw - a->yaw) * t;
                return 1;
            }
        }
    }
    return 0; /* No extrapolation or silent substitution of the current pose. */
}

/* Apply one distinct frame, using its already aligned pose when available. */
static void apply_frame(const VisionObservation *o, const FieldPose *pose)
{
    unsigned i;
    predict_tracks();
    field_map.frame=o->frame_id;
    if (!pose || o->status==VISION_OBS_EMPTY) {
        clear_observation();
        return;
    }
    field_map.observed=*pose;
    field_map.count=o->count;
    field_map.aligned=1;
    for(i=0;i<o->count;++i) {
        const VisionObject *v=&o->objects[i];
        FieldPoint *p=&field_map.points[i];
        float x,y,c,s;
        memset(p,0,sizeof(*p));
        if (v->kind>=VISION_GREEN && v->kind<=VISION_BLUE && range_cm(v)<=12.0f)
            continue;
        if(v->position_present) {
            const float a=FIELD_VISION_YAW_DEG*DEG_RAD;
            float right=v->x_mm*0.1f, forward=v->y_mm*0.1f;
            x=FIELD_VISION_X_CM+right*cosf(a)+forward*sinf(a);
            y=FIELD_VISION_Y_CM-right*sinf(a)+forward*cosf(a);
        } else {
            float a,r;
            if(v->distance_mm<0 || !v->angle_present ||
               !(v->angle>=-180.0f && v->angle<=180.0f))continue;
            a=(v->angle+FIELD_VISION_YAW_DEG)*DEG_RAD;
            r=v->distance_mm*0.1f;
            x=FIELD_VISION_X_CM+r*sinf(a);y=FIELD_VISION_Y_CM+r*cosf(a);
        }
        c=cosf(field_map.observed.yaw);s=sinf(field_map.observed.yaw);
        p->x=field_map.observed.x+x*c+y*s;
        p->y=field_map.observed.y-x*s+y*c;p->present=1;
    }
    associate(o);
}
void field_map_update(const VisionObservation *o, uint32_t now)
{
    uint32_t wait=now-o->updated_ms, observed_tick=o->updated_ms-o->age_ms;
    FieldPose pose;
    const FieldPose *aligned_pose=NULL;
    if (!field_map.heading_valid || !o->fresh ||
        (o->status!=VISION_OBS_OK && o->status!=VISION_OBS_EMPTY) ||
        o->age_ms>400U || wait>400U-o->age_ms || o->count>VISION_MAX_OBJECTS) {
        awaiting_pose=0;
        if (processed && processed_frame==o->frame_id) hide_observation();
        else clear_observation();
        return;
    }
    /* Polling the same vision result must never add another hit. */
    if (processed && processed_frame==o->frame_id) return;
    if (processed && (int32_t)(observed_tick-processed_tick)<0) {
        awaiting_pose=0;
        clear_observation();
        return;
    }
    if (awaiting_pose && awaiting_frame!=o->frame_id)
        clear_observation(); /* A frame that could not be aligned was superseded. */
    if (o->status==VISION_OBS_OK) {
        if (field_map_pose_at(observed_tick,&pose)) {
            aligned_pose=&pose;
        } else if ((int32_t)(observed_tick-field_map.robot.tick)>0) {
            /* Wait until wheel odometry brackets this frame; never extrapolate. */
            awaiting_pose=1;
            awaiting_frame=o->frame_id;
            field_map.aligned=field_map.count=0;
            return;
        }
    }
    awaiting_pose=0;
    processed=1;
    processed_frame=o->frame_id;
    processed_tick=observed_tick;
    apply_frame(o,aligned_pose);
}

void field_map_process(void)
{
    field_map_update(&vision.upper,HAL_GetTick());
}

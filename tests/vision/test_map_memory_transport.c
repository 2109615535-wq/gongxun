#define main existing_firmware_tests
#include "test_static.c"
#undef main

#ifdef UART5_BANDWIDTH_REPLAY

/* Feed captured values to the production codec and to the actual UART5 sender.
 * The policy replay advances TX at 115200 8N1 and uses the firmware's gap. */
typedef struct {
    unsigned char type;
    uint32_t tick, rx, bad, uart_errors, drops;
    U5_Pose origin, extrinsic;
    uint8_t has_command, command_status, has_target, history_count;
    uint32_t command_number;
    uint16_t target_x, target_y;
    float target_speed;
    U5_Live live;
    U5_Track track;
    FieldSample history[FIELD_TRACK_HISTORY];
    int32_t history_x[FIELD_TRACK_HISTORY], history_y[FIELD_TRACK_HISTORY];
} Capture;

static uint32_t read_number(FILE *f)
{
    char word[24], *end;
    uint32_t result;
    assert(fscanf(f,"%23s",word)==1);
    if(word[0]=='-') result=(uint32_t)strtol(word,&end,10);
    else result=(uint32_t)strtoul(word,&end,10);
    assert(!*end);
    return result;
}

static void read_pose(FILE *f, U5_Pose *pose)
{
    pose->x_mm=(int32_t)read_number(f);
    pose->y_mm=(int32_t)read_number(f);
    pose->yaw_cdeg=(int32_t)read_number(f);
}

static void read_obs(FILE *f, U5_Observation *obs)
{
    obs->frame=read_number(f);
    obs->age=read_number(f);
    obs->status=(uint8_t)read_number(f);
    obs->flags=(uint8_t)read_number(f);
    obs->count=(uint8_t)read_number(f);
    assert(obs->count<=U5_MAX_OBJECTS);
}

static int capture_read(FILE *f, Capture *row)
{
    unsigned i;
    char type;
    if(fscanf(f," %c",&type)!=1) return 0;
    memset(row,0,sizeof(*row));
    row->type=(unsigned char)type;
    row->tick=read_number(f);
    if(type=='V') {
        U5_Live *l=&row->live;
        l->step=(uint8_t)read_number(f);
        l->reason=(uint8_t)read_number(f);
        l->held_kind=(uint8_t)read_number(f);
        l->roi_counts=(uint8_t)read_number(f);
        l->servo=(int16_t)read_number(f);
        l->flags=(uint8_t)read_number(f);
        read_pose(f,&l->robot);
        l->imu_age=read_number(f);
        l->imu_yaw=(uint16_t)read_number(f);
        read_pose(f,&row->origin);
        read_pose(f,&row->extrinsic);
        row->rx=read_number(f);
        row->bad=read_number(f);
        row->uart_errors=read_number(f);
        row->drops=read_number(f);
        row->has_command=(uint8_t)read_number(f);
        row->command_number=read_number(f);
        row->command_status=(uint8_t)read_number(f);
        row->has_target=(uint8_t)read_number(f);
        row->target_x=(uint16_t)read_number(f);
        row->target_y=(uint16_t)read_number(f);
        row->target_speed=read_number(f)*0.1f;
        read_obs(f,&l->upper);
        read_pose(f,&l->observed);
        for(i=0;i<l->upper.count;++i) {
            U5_UpperObject *o=&l->upper_objects[i];
            o->kind=(uint8_t)read_number(f);
            o->flags=(uint8_t)read_number(f);
            o->distance=(uint16_t)read_number(f);
            o->angle=(int16_t)read_number(f);
            o->raw_x=(int32_t)read_number(f);
            o->raw_y=(int32_t)read_number(f);
            o->field_x=(int32_t)read_number(f);
            o->field_y=(int32_t)read_number(f);
            o->track_id=read_number(f);
            o->radius=(uint16_t)read_number(f);
        }
        read_obs(f,&l->lower);
        for(i=0;i<l->lower.count;++i) {
            U5_LowerObject *o=&l->lower_objects[i];
            o->kind=(uint8_t)read_number(f);
            o->flags=(uint8_t)read_number(f);
            o->zone=(uint8_t)read_number(f);
        }
    } else {
        U5_Track *t=&row->track;
        assert(type=='M');
        t->epoch=read_number(f);
        t->map_rev=read_number(f);
        t->count=(uint8_t)read_number(f);
        t->unmapped=read_number(f);
        t->id=read_number(f);
        if(t->id) {
            t->revision=read_number(f);
            t->kind=(uint8_t)read_number(f);
            t->flags=(uint8_t)read_number(f);
            t->x_mm=(int32_t)read_number(f);
            t->y_mm=(int32_t)read_number(f);
            t->radius=(uint16_t)read_number(f);
            t->seen=read_number(f);
            t->position=read_number(f);
            t->observations=read_number(f);
            row->history_count=(uint8_t)read_number(f);
            assert(row->history_count<=FIELD_TRACK_HISTORY);
            for(i=0;i<row->history_count;++i) {
                row->history[i].tick=read_number(f);
                row->history_x[i]=(int32_t)read_number(f);
                row->history_y[i]=(int32_t)read_number(f);
                row->history[i].x=row->history_x[i]*0.1f;
                row->history[i].y=row->history_y[i]*0.1f;
                row->history[i].accepted=(uint8_t)read_number(f);
            }
        }
    }
    return 1;
}

static void hex(const uint8_t *bytes, unsigned length)
{
    unsigned i;
    assert(length!=0);
    for(i=0;i<length;++i) printf("%02X",bytes[i]);
    putchar('\n');
}

static void same_emit(U5_Type type, uint32_t now, const U5_Payload *p)
{
    static uint16_t seq;
    uint8_t out[U5_MAX_WIRE];
    uint16_t length=U5_Frame(out,type,seq++,now,p);
    hex(out,length);
}

static void same_sample(const Capture *c)
{
    U5_Payload p;
    unsigned i;
    if(c->type=='M') {
        if(!c->track.id) {
            p.length=0; U5_U16(&p,1); U5_U8(&p,2);
            U5_U8(&p,0); U5_U8(&p,0); U5_U32(&p,c->track.epoch);
            same_emit(U5_MAP_SYNC,c->tick,&p);
            return;
        }
        p.length=0; U5_TrackPayload(&p,&c->track);
        U5_U8(&p,c->history_count);
        for(i=0;i<c->history_count;++i) {
            U5_U32(&p,c->history[i].tick);
            U5_U32(&p,(uint32_t)c->history_x[i]);
            U5_U32(&p,(uint32_t)c->history_y[i]);
            U5_U8(&p,c->history[i].accepted);
        }
        same_emit(U5_HISTORY,c->tick,&p);
        return;
    }
    p.length=0; U5_LivePayload(&p,&c->live); same_emit(U5_LIVE,c->tick,&p);
    /* Include state separated from LIVE before comparing full-content rates. */
    p.length=0; U5_U32(&p,1); U5_PutPose(&p,&c->origin);
    U5_PutPose(&p,&c->extrinsic); U5_U16(&p,0); same_emit(U5_INFO,c->tick,&p);
    p.length=0;
    U5_U8(&p,c->has_command);
    if(c->has_command) { U5_U32(&p,c->command_number); U5_U8(&p,c->command_status); }
    U5_U8(&p,c->has_target);
    if(c->has_target) { U5_U16(&p,c->target_x); U5_U16(&p,c->target_y); U5_F32(&p,c->target_speed); }
    same_emit(U5_CONTROL,c->tick,&p);
    p.length=0; U5_U32(&p,1); U5_U32(&p,0); U5_U32(&p,0); U5_U8(&p,0);
    U5_U32(&p,c->rx); U5_U32(&p,c->bad); U5_U32(&p,c->uart_errors); U5_U32(&p,c->drops);
    for(i=0;i<4;++i) U5_U32(&p,0);
    U5_U16(&p,0); same_emit(U5_STATS,c->tick,&p);
}

static void restore_pose(FieldPose *dst, const U5_Pose *src, uint32_t now)
{
    dst->tick=now;
    dst->x=src->x_mm*0.1f;
    dst->y=src->y_mm*0.1f;
    dst->yaw=src->yaw_cdeg*(DEG_RAD*0.01f);
}

static void restore_obs(VisionObservation *dst,const U5_Observation *src,uint32_t now)
{
    memset(dst,0,sizeof(*dst));
    dst->frame_id=src->frame;
    dst->age_ms=src->age;
    dst->updated_ms=now;
    dst->fresh=(src->flags&U5_FRESH)!=0;
    dst->available=(src->flags&U5_AVAILABLE)!=0;
    dst->status=(VisionObservationStatus)src->status;
    dst->count=src->count;
}

static void restore_live(const Capture *row)
{
    static const char *const faults[]={"rv3_observation","waiting_rv3","invalid_packet",
        "uart_error","vision_timeout","upper_unavailable"};
    const U5_Live *l=&row->live;
    unsigned i, fault=l->flags>>5;
    assert(fault<sizeof(faults)/sizeof(faults[0]));
    strcpy(vision.fault_code,faults[fault]);
    vision.connected=(l->flags&U5_LINK)!=0;
    vision.rx_packets=row->rx;
    vision.bad_packets=row->bad;
    vision.uart_errors=row->uart_errors;
    step=l->step;
    task_info.reason=l->reason;
    task_info.held_kind=l->held_kind;
    task_info.inside_frames=l->roi_counts&15U;
    task_info.outside_frames=l->roi_counts>>4;
    servo_angle=l->servo;
    restore_pose(&field_map.robot,&l->robot,row->tick);
    field_map.heading_valid=(l->flags&U5_HEADING)!=0;
    imu.ready=(l->flags&U5_IMU_READY)!=0;
    imu.tick=row->tick-l->imu_age;
    imu.raw_yaw=l->imu_yaw*0.01f;
    restore_obs(&vision.upper,&l->upper,row->tick);
    field_map.aligned=(l->upper.flags&U5_ALIGNED)!=0;
    field_map.frame=l->upper.frame;
    field_map.count=l->upper.count;
    restore_pose(&field_map.observed,&l->observed,row->tick-l->upper.age);
    for(i=0;i<l->upper.count;++i) {
        VisionObject *v=&vision.upper.objects[i];
        const U5_UpperObject *o=&l->upper_objects[i];
        FieldPoint *point=&field_map.points[i];
        v->kind=(VisionKind)o->kind;
        v->distance_mm=(o->flags&U5_DISTANCE)?o->distance:-1;
        v->angle=o->angle*0.01f;
        v->angle_present=(o->flags&U5_ANGLE)!=0;
        v->position_present=(o->flags&U5_RAW_XY)!=0;
        v->x_mm=o->raw_x;
        v->y_mm=o->raw_y;
        v->angle_valid=v->distance_valid=(o->flags&U5_VALID)!=0;
        point->present=(o->flags&U5_FIELD_XY)!=0;
        point->track_id=o->track_id;
        point->x=o->field_x*0.1f;
        point->y=o->field_y*0.1f;
        point->radius=o->radius*0.1f;
    }
    restore_obs(&vision.lower,&l->lower,row->tick);
    for(i=0;i<l->lower.count;++i) {
        VisionObject *v=&vision.lower.objects[i];
        const U5_LowerObject *o=&l->lower_objects[i];
        v->kind=(VisionKind)o->kind;
        v->in_roi=(o->flags&U5_IN_ROI)!=0;
        v->inside_valid=(o->flags&U5_HAS_ZONE)!=0;
        v->object_inside=(o->flags&U5_INSIDE)!=0;
        v->zone_kind=(VisionKind)o->zone;
    }
    faraway_task_observe();
}

static void restore_track(const Capture *row)
{
    const U5_Track *t=&row->track;
    FieldTrack *dst=NULL;
    unsigned i;
    if(field_map.epoch!=t->epoch) {
        field_map.track_count=0;
        field_map.revision=0;
        field_map.epoch=t->epoch;
    }
    if(t->map_rev>field_map.revision) field_map.revision=t->map_rev;
    field_map.unmapped_count=t->unmapped;
    if(!t->id) return;
    for(i=0;i<field_map.track_count;++i)
        if(field_tracks[i].id==t->id) { dst=&field_tracks[i]; break; }
    if(!dst) {
        assert(field_map.track_count<FIELD_TRACK_CAPACITY);
        dst=&field_tracks[field_map.track_count++];
    } else if((int32_t)(t->revision-dst->revision)<0) return;
    dst->id=t->id;
    dst->revision=t->revision;
    dst->kind=t->kind;
    dst->visible=t->flags&1U;
    dst->review=(t->flags&2U)!=0;
    dst->x=t->x_mm*0.1f;
    dst->y=t->y_mm*0.1f;
    dst->distance=(t->radius*0.1f-FIELD_MATCH_BASE_CM)/FIELD_MATCH_DISTANCE_SCALE;
    dst->seen_ms=t->seen;
    dst->position_ms=t->position;
    dst->observations=t->observations;
    dst->history_count=row->history_count;
    dst->history_next=(uint8_t)(row->history_count%FIELD_TRACK_HISTORY);
    memcpy(dst->history,row->history,sizeof(dst->history));
}

static uint8_t sent_frame[U5_MAX_WIRE];
static unsigned sent_size;
static uint32_t transmit_done, transmit_start;
static unsigned expected_history, history_replies;

static void pump_uart(uint32_t window_start)
{
    unsigned i;
    if(!pending) return;
    if(!transmit_done) {
        transmit_done=tick+(pending_size*10U+114U)/115U;
        if(!sent_size) transmit_start=tick;
    }
    if(tick!=transmit_done) return;
    for(i=0;i<pending_size;++i) {
        const uint8_t byte=pending[i];
        assert(sent_size<sizeof(sent_frame));
        sent_frame[sent_size++]=byte;
        if(byte) continue;
        assert(sent_size>=14U && sent_frame[1]==U5_VERSION);
        if(sent_frame[2]==U5_HISTORY) ++history_replies;
        if((int32_t)(transmit_start-window_start)>=0) {
            printf("%lu %lu ",(unsigned long)(transmit_start-window_start),
                   (unsigned long)(tick-window_start));
            hex(sent_frame,sent_size);
        }
        sent_size=0;
    }
    pending=NULL;
    last_complete=tick;
    transmit_done=0;
    HAL_UART_TxCpltCallback(&huart5);
}

static void send_text(const char *text)
{
    const char *p;
    for(p=text;*p;++p) {
        rx_byte=(uint8_t)*p;
        faraway_uart_rx_complete(&huart5);
    }
}

static void send_crc_command(const char *body)
{
    char msg[90];
    const unsigned crc=VSP_Crc16((const uint8_t *)body,(uint16_t)strlen(body));
    assert(snprintf(msg,sizeof(msg),"%s*%04X\r\n",body,crc)>0);
    send_text(msg);
}

static void run_window(FILE *input,uint32_t begin,uint32_t duration,const char *mode)
{
    const uint32_t end=begin+duration;
    const uint32_t request_at=begin+1000U;
    uint32_t next_request=request_at, session_start;
    unsigned query=0;
    Capture row;
    int more=capture_read(input,&row);
    assert(more);
    session_start=row.tick;
    tick=session_start;
    field_map_reset(tick);
    vision.upper.age_ms=vision.lower.age_ms=UINT32_MAX;
    assert(faraway_init());
    for(tick=session_start;tick<=end+100U;++tick) {
        while(more && row.tick<=tick && row.tick<end) {
            if(row.type=='V') restore_live(&row);
            else restore_track(&row);
            more=capture_read(input,&row);
        }
        pump_uart(begin);
        if(tick>=end) continue; /* Allow an in-progress frame to complete. */
        if(tick==request_at && !strcmp(mode,"sync")) send_text("MAP\r\n");
        if(tick==request_at && !strcmp(mode,"control")) send_crc_command("C1,1,K,2");
        if(tick>=next_request && !strcmp(mode,"steer")) {
            char body[40];
            (void)snprintf(body,sizeof(body),"C1,%u,T,1",++query);
            send_crc_command(body);
            next_request+=4000U;
        }
        if(tick>=request_at && !strcmp(mode,"history") && history_replies==expected_history &&
                expected_history<1U && field_map.track_count) {
            char body[52];
            (void)snprintf(body,sizeof(body),"HIST,%lu,%lu",(unsigned long)field_map.epoch,
                           (unsigned long)field_tracks[0].id);
            ++expected_history;
            send_crc_command(body);
        }
        if(tick>=request_at && !strcmp(mode,"export") && history_replies==expected_history &&
                expected_history<field_map.track_count) {
            char body[52];
            (void)snprintf(body,sizeof(body),"HIST,%lu,%lu",(unsigned long)field_map.epoch,
                           (unsigned long)field_tracks[expected_history].id);
            ++expected_history;
            send_crc_command(body);
        }
        faraway_process();
        if((tick-session_start)%5U==0U) faraway_report();
        /* The call that starts a chunk happens above. Capture its actual start. */
        pump_uart(begin);
    }
    assert(history_replies==expected_history);
}

int main(int argc,char **argv)
{
    FILE *input;
    Capture row;
    if(argc!=3 && argc!=6) {
        fputs("usage: benchmark same CAPTURE | benchmark window CAPTURE BEGIN DURATION MODE\n",stderr);
        return EXIT_FAILURE;
    }
    input=fopen(argv[2],"r");
    assert(input);
    if(argc==3 && !strcmp(argv[1],"same"))
        while(capture_read(input,&row)) same_sample(&row);
    else {
        assert(argc==6 && !strcmp(argv[1],"window"));
        run_window(input,(uint32_t)strtoul(argv[3],NULL,10),(uint32_t)strtoul(argv[4],NULL,10),argv[5]);
    }
    assert(!ferror(input));
    fclose(input);
    return EXIT_SUCCESS;
}

#else


static void checksummed(const char *body, char *text, unsigned size)
{
    snprintf(text,size,"%s*%04X\r\n",body,VSP_Crc16((const uint8_t *)body,(uint16_t)strlen(body)));
}

static void send_text(const char *text)
{
    while (*text) {
        rx_byte=(uint8_t)*text++;
        faraway_uart_rx_complete(&huart5);
    }
    faraway_process();
}

static void advance_uart(unsigned elapsed)
{
    reset_wire();
    tick+=elapsed;
    faraway_report();
    drain();
}

static void load_camera_frame(unsigned frame_id)
{
    field_map_record(tick,0,0,0);
    cameras(500,(float)(frame_id%3),0,0);
    vision.upper.frame_id=frame_id;
    vision.upper.objects[0].position_present=1;
    vision.upper.objects[0].x_mm=0;
    vision.upper.objects[0].y_mm=500;
    field_map_process();
}

static void check_vectors(void)
{
    const uint8_t expected[]={4,1,2,1,2,2,3,0xe8,3,1,5,0x1e,4,0x8c,0xcf,0};
    uint8_t bytes[U5_MAX_WIRE];
    uint16_t length;
    U5_Payload payload={0,{0}};
    assert(VSP_Crc16((const uint8_t *)"123456789",9)==0x29b1);
    U5_U8(&payload,30);
    U5_U8(&payload,4);
    length=U5_Frame(bytes,U5_EVENT,1,1000,&payload);
    assert(length==sizeof(expected) && !memcmp(bytes,expected,sizeof(expected)));
    wire_vector("event",bytes,length);
    payload.length=U5_MAX_PAYLOAD+1U;
    assert(!U5_Frame(bytes,U5_EVENT,1,1000,&payload));
}

static void full_sync(void)
{
    unsigned i,offset,items=0,started=0,ended=0,stopped_with_sync=0,status=0,have_seq=0;
    uint16_t sync_id=0,next_seq=0;
    uint32_t last_status=0,max_gap=0;
    char text[100];
    WireFrame rx;
    checksummed("C1,857,K,4",text,sizeof(text));
    reset_wire();
    command("MAP\r\n");
    for(i=0;i<4000 && !ended;++i) {
        for(offset=0;wire_next(wire,wire_size,&offset,&rx);) {
            const uint8_t *p=rx.payload;
            assert(!have_seq || rx.seq==next_seq);
            have_seq=1;
            next_seq=(uint16_t)(rx.seq+1U);
            if(rx.type==WIRE_MAP_SYNC) {
                assert(rx.length>=9 && p[4]==48 && wire_u32(p+5)==field_map.epoch);
                if(p[2]==0) {
                    assert(!started && rx.length==9);
                    started=1;sync_id=wire_u16(p);
                } else if(p[2]==1) {
                    assert(started && wire_u16(p)==sync_id && p[3]==items && rx.length==54);
                    assert(wire_u32(p+9)==field_map.epoch && wire_u32(p+22)==items+1U);
                    ++items;
                    if(items==1) wire_vector("sync_item",wire+offset-rx.encoded_size,rx.encoded_size);
                } else {
                    assert(p[2]==2 && started && wire_u16(p)==sync_id && rx.length==9 && items==48);
                    ended=1;
                }
            } else if(rx.type==WIRE_LIVE) {
                if(last_status && rx.tick-last_status>max_gap) max_gap=rx.tick-last_status;
                last_status=rx.tick;
                ++status;
            }
        }
        if(ended) break;
        reset_wire();
        tick+=5;
        faraway_report();
        if(items>=3 && !stopped_with_sync && pending) {
            unsigned index=0;
            uint32_t began=tick;
            assert(task_goto_start(150,150,50));
            assert(step==50);
            send_text(text);
            assert(step==0); /* Stop while the UART still has a frame in flight. */
            stopped();
            drain();
            assert(wire_next(wire,wire_size,&index,&rx));
            assert(wire_next(wire,wire_size,&index,&rx));
            assert(rx.type==WIRE_ACK && wire_u32(rx.payload)==857 && rx.payload[4]==0);
            assert(tick-began<120);
            wire_vector("ack",wire+index-rx.encoded_size,rx.encoded_size);
            stopped_with_sync=1;
        } else drain();
    }
    assert(ended && items==48 && stopped_with_sync && status>10 && max_gap<160);
    printf("PASS: 48-item indexed sync and timely KEY4/ACK during sync, contiguous sequence, LIVE max gap %lu ms\n",(unsigned long)max_gap);
}

static void history_and_events(void)
{
    char body[80],text[100];
    unsigned i,offset,seen=0;
    uint32_t lost_at,restored_at;
    WireFrame rx;
    reset_wire();
    snprintf(body,sizeof(body),"HIST,%lu,48",(unsigned long)field_map.epoch);
    checksummed(body,text,sizeof(text));
    command(text);
    for(i=0;!wire_find(wire,wire_size,WIRE_HISTORY,&rx) && i<100;i++) advance_uart(10);
    assert(i<100 && rx.length==150 && wire_u32(rx.payload+13)==48 && rx.payload[45]==8);
    assert(rx.payload[46+12]==1 && rx.payload[46+7*13+12]==1);
    for(offset=0;wire_next(wire,wire_size,&offset,&rx);)
        if(rx.type==WIRE_HISTORY) wire_vector("history",wire+offset-rx.encoded_size,rx.encoded_size);
    assert(command_id==857); /* Read-only history does not consume command IDs. */

    reset_wire();
    step=30;
    lost_at=++tick;task_info.reason=TASK_TARGET_LOST;faraway_task_observe();
    restored_at=++tick;task_info.reason=TASK_OK;faraway_task_observe();
    faraway_report();drain();
    for(i=0;i<100 && seen<2;++i) {
        for(offset=0;wire_next(wire,wire_size,&offset,&rx);) {
            if(rx.type!=WIRE_EVENT || rx.payload[0]!=30) continue;
            assert(rx.length==2);
            if(seen==0) assert(rx.payload[1]==TASK_TARGET_LOST && rx.tick==lost_at);
            else assert(rx.payload[1]==TASK_OK && rx.tick==restored_at);
            wire_vector(seen?"restore":"lost",wire+offset-rx.encoded_size,rx.encoded_size);
            ++seen;
        }
        if(seen<2) advance_uart(5);
    }
    assert(seen==2 && !motor_drive_enabled);
    puts("PASS: independent HIST query with all 8 samples, and distinct loss/recovery events within one heartbeat");
}

static void check_map_deltas(void)
{
    unsigned i,offset,deltas=0,stats=0;
    uint8_t seen[2]={0,0};
    WireFrame rx;
    field_tracks[0].revision=++field_map.revision;
    field_tracks[47].revision=++field_map.revision;
    for(i=0;i<250;++i) {
        advance_uart(10);
        for(offset=0;wire_next(wire,wire_size,&offset,&rx);) {
            if(rx.type==WIRE_TRACK) {
                uint32_t id=wire_u32(rx.payload+13);
                assert(rx.length==45 && rx.encoded_size==59);
                assert(id==1 || id==48);
                assert(!seen[id==48]++);
                if(!deltas) wire_vector("track",wire+offset-rx.encoded_size,rx.encoded_size);
                ++deltas;
            }
            if(rx.type==WIRE_STATS) {
                assert(rx.length==47 && rx.encoded_size==61 && rx.payload[12]==48);
                assert(wire_u32(rx.payload+29)>0 && wire_u16(rx.payload+45)>0);
                if(!stats) wire_vector("stats",wire+offset-rx.encoded_size,rx.encoded_size);
                ++stats;
            }
        }
    }
    assert(deltas==2 && stats>=2);
    puts("PASS: only changed tracks sent, unchanged entries stay quiet, completed UART bytes and queue high-water reported");
}

int main(void)
{
    unsigned i,offset;
    FieldTrack sample;
    WireFrame info;
    check_vectors();
    Motor_Init();
    tick=1000;
    field_map_reset(tick);
    for(i=1;i<=14;++i){tick+=20;load_camera_frame(i);}
    assert(!opened && field_map.track_count==1 && field_tracks[0].history_count==8);
    assert(field_tracks[0].observations==12 && !motor_drive_enabled && !enabled_calls);
    sample=field_tracks[0];
    field_map.track_count=48;
    for(i=0;i<48;++i) {
        field_tracks[i]=sample;
        field_tracks[i].id=i+1;
        field_tracks[i].revision=i+1;
    }
    field_map.revision=48;
    assert(faraway_init());
    command("GET\r\n");
    for(i=0;!wire_find(wire,wire_size,WIRE_INFO,&info) && i<100;i++) advance_uart(10);
    assert(i<100 && info.length==30 && wire_u32(info.payload+20)==0);
    for(offset=0;wire_next(wire,wire_size,&offset,&info);)
        if(info.type==WIRE_INFO) wire_vector("info",wire+offset-info.encoded_size,info.encoded_size);
    full_sync();
    history_and_events();
    check_map_deltas();
    return 0;
}
#endif

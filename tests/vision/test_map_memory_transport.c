#define main existing_firmware_tests
#include "test_static.c"
#undef main

static void map_frame(unsigned frame_id)
{
    field_map_record(tick,0,0,0);
    cameras(500,(float)(frame_id%3),0,0);
    vision.upper.frame_id=frame_id;
    field_map_process();
}
static unsigned packet_id(void)
{
    unsigned long at,epoch,rev,count,missed,id;
    assert(sscanf(wire,"[\"M1\",%lu,%lu,%lu,%lu,%lu,[%lu",&at,&epoch,&rev,&count,&missed,&id)==6);
    assert(wire_size<640);
    assert(VSP_Crc16((const uint8_t *)wire,(uint16_t)(strchr(wire,'*')-wire))==strtoul(strchr(wire,'*')+1,NULL,16));
    return (unsigned)id;
}
int main(void)
{
    unsigned i,n,seen[FIELD_TRACK_CAPACITY]={0},maps=0,status=0,max_packet=0;
    uint32_t previous_revision,previous_observations,last_status=0,max_gap=0;
    FieldTrack original;
    Motor_Init();tick=1000;field_map_reset(tick);
    for(i=1;i<=12;++i){tick+=20;map_frame(i);}
    assert(!opened && field_map.track_count==1 && field_tracks[0].history_count==8);
    assert(field_tracks[0].observations==12 && !motor_drive_enabled && !enabled_calls);
    assert(faraway_init());
    faraway_send("BUSY\n");assert(tx_head!=tx_tail);
    previous_observations=field_tracks[0].observations;
    tick+=20;map_frame(13);faraway_vision_report();
    assert(field_tracks[0].observations==previous_observations+1 && g_faraway_tx_drop_count==1);
    reset_wire();faraway_map_report(tick);drain();assert(packet_id()==1);puts(wire);
    assert(field_tracks[0].observations==13);
    previous_revision=field_tracks[0].revision;
    reset_wire();tick+=1000;vision.upper.fresh=0;field_map_process();
    assert(field_tracks[0].id==1 && !field_tracks[0].visible && field_tracks[0].revision>previous_revision);
    faraway_map_report(tick);drain();assert(packet_id()==1);puts(wire);
    reset_wire();tick+=10000;field_map_process();faraway_map_report(tick);drain();puts(wire);
    assert(field_map.track_count==1 && field_tracks[0].observations==13);
    /* Per-track revisions and a separate repair cursor prevent a busy target
     * from starving forgotten/lost packets for all the quiet map records. */
    original=field_tracks[0];field_map_reset(tick);field_map.track_count=48;
    for(i=0;i<48;++i){field_tracks[i]=original;field_tracks[i].id=i+1;field_tracks[i].revision=i+1;}
    field_map.revision=48;
    for(i=0;i<48;++i){reset_wire();tick+=300;faraway_map_report(tick);drain();seen[packet_id()-1]++;}
    for(i=0;i<48;++i)assert(seen[i]==1);
    memset(seen,0,sizeof(seen));
    for(i=0;i<48*4;++i){reset_wire();tick+=300;field_tracks[47].revision=++field_map.revision;faraway_map_report(tick);drain();seen[packet_id()-1]++;}
    for(i=0;i<48;++i)assert(seen[i]>=1);
    command("MAP\n");assert(map_sync_remaining==48 && map_repair_cursor==0);
    reset_wire();tick+=300;faraway_map_report(tick);drain();assert(packet_id()==1);
    /* Normal scheduling: bounded map packets interleave with live status.
     * No new movement side effects, even during resync of a full map. */
    vision.upper.fresh=1;vision.upper.status=VISION_OBS_OK;vision.upper.count=12;
    for(i=0;i<12;++i){
        VisionObject *v=&vision.upper.objects[i];
        memset(v,0,sizeof(*v));v->kind=VISION_BLUE_MATERIAL_ZONE;
        v->distance_mm=65535;v->angle=-135;v->angle_present=1;
        field_map.points[i].present=1;field_map.points[i].track_id=i+1;
        field_map.points[i].x=-4349;field_map.points[i].y=-4554;field_map.points[i].radius=331.7f;
    }
    vision.lower=vision.upper;field_map.count=12;field_map.aligned=1;
    field_map.frame=vision.upper.frame_id;
    for(i=0;i<100;++i){
        reset_wire();tick+=50;vision.upper.updated_ms=tick;vision.upper.age_ms=0;vision.lower.updated_ms=tick;vision.lower.age_ms=0;
        faraway_report();drain();
        if(!wire_size)continue;
        if(wire_size>max_packet)max_packet=wire_size;
        if(strstr(wire,"[\"M1\",")){++maps;packet_id();}
        if(strstr(wire,"[\"V7\",")){
            if(last_status && tick-last_status>max_gap)max_gap=tick-last_status;
            last_status=tick;++status;
        }
        assert(!motor_drive_enabled && !enabled_calls && step==0);
    }
    assert(maps>=10 && status>=20 && max_gap<400);
    /* Max-width timestamps + all eight history samples fit the bounded packet. */
    reset_wire();field_map_reset(0xffffff00U);tick=0xfffffff0U;tx_next_ms=tick;field_map.track_count=1;
    field_tracks[0]=original;field_tracks[0].id=1;field_tracks[0].revision=field_map.revision=UINT32_MAX;
    field_tracks[0].observations=UINT32_MAX;field_tracks[0].seen_ms=field_tracks[0].position_ms=tick;
    for(n=0;n<8;++n){field_tracks[0].history[n].tick=tick-7+n;field_tracks[0].history[n].x=-1000000;field_tracks[0].history[n].y=1000000;}
    field_tracks[0].history_next=0;field_tracks[0].history_count=8;
    faraway_map_report(tick);drain();assert(packet_id()==1);
    printf("PASS: map without telemetry/open UART, busy TX independence, actual M1 CRC/history, bounded packets (%u max scheduled bytes), full snapshot, repair fairness, status gap %lu ms, no motor actions\n",max_packet,(unsigned long)max_gap);
    return 0;
}

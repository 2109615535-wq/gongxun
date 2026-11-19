/* Actual PID, UART and task code; fake HAL only, never opens hardware. */
#define main existing_firmware_tests
#include "test_static.c"
#undef main

static void tune_command(unsigned id, const char *payload)
{
    char body[80], text[96];
    snprintf(body,sizeof(body),"C1,%u,%s",id,payload);
    snprintf(text,sizeof(text),"%s*%04X\n",body,VSP_Crc16((const uint8_t *)body,(uint16_t)strlen(body)));
    command(text);
}

int main(void)
{
    unsigned before, n, status_packets=0, steer_packets=0, max_status_gap=0;
    uint32_t last_status=0;
    PID_t local;
    Motor_Init(); tick=1000;
    memset(&local,0,sizeof(local));
    local.kp=3; local.ki=.5f; local.kd=2;
    local.output_max=123; local.error_int_max=25;
    local.error_int=7;local.output=9;
    PID_Init(&local);
    assert(local.kp==3 && local.ki==.5f && local.kd==2);
    assert(local.output_max==123 && local.error_int_max==25 && local.error_int==0 && local.output==0);
    assert(steering_set_pid(2,.1f,0));
    task_prepare();enter_step(30);
    assert(pid_steer.error_int_max==FLT_MAX);
    follow_target(10,1,60,50,1);motor_process();
    assert(fabsf(pid_steer.output-21)<.001f && pid_steer.error_int==10);
    assert(steer_state.source==1 && steer_state.p==20 && fabsf(steer_state.i-1)<.001f);
    follow_target(10,1,60,50,1);assert(pid_steer.error_int==10); /* duplicate frame */
    follow_target(10,2,60,50,1);assert(fabsf(pid_steer.output-22)<.001f && pid_steer.error_int==20);
    drive_stop();assert(pid_steer.error_int==0 && pid_steer.error_int_max==FLT_MAX);
    follow_target(-.5f,3,60,50,1);assert(pid_steer.output==0 && pid_steer.error_now==-.5f && pid_steer.error_int==0);
    tick+=20;follow_target(1,4,60,50,1);assert(pid_steer.output==0);
    { uint32_t sample_tick=steer_state.tick;tick+=20;follow_target(1,4,60,50,1);assert(steer_state.tick==sample_tick); }
    follow_target(-1.01f,5,60,50,1);assert(fabsf(pid_steer.output+2.121f)<.001f);
    task_stop();task_start();move();
    assert(pid_steer.kp==2 && pid_steer.ki==.1f && pid_steer.error_int_max==FLT_MAX);
    assert(!steering_set_pid(-1,0,0) && !steering_set_pid(NAN,0,0));
    assert(faraway_init());reset_wire();before=enabled_calls;
    tune_command(1,"P,25000,5000,1200");
    assert(step==10 && enabled_calls==before && !motor_drive_enabled);
    assert(pid_steer.kp==2.5f && pid_steer.ki==.5f && fabsf(pid_steer.kd-.12f)<.00001f);
    assert(strstr(wire,"A1,1,0,10*"));
    follow_target(10,5,60,50,1);motor_process();assert(fabsf(pid_steer.output-30)<.001f);
    reset_wire();tune_command(1,"P,25000,5000,1200");
    assert(pid_steer.error_int==10); /* duplicate ACK cannot clear history */
    reset_wire();tune_command(0,"P,0,0,0");assert(pid_steer.kp==2.5f && command_status==0);
    assert(strstr(wire,"A1,0,4,"));
    reset_wire();command("C1,2,P,10000,-1,0*0000\n");assert(pid_steer.kp==2.5f);
    reset_wire();faraway_steer_report(tick);drain();
    assert(strstr(wire,"[\"S1\","));
    assert(VSP_Crc16((const uint8_t *)wire,(uint16_t)(strchr(wire,'*')-wire))==strtoul(strchr(wire,'*')+1,NULL,16));
    puts(wire); /* shared with the browser's pure parser test */
    reset_wire();tune_command(2,"T,1");assert(steer_stream);
    before=enabled_calls;tick=steer_stream_tick+6001;faraway_report();drain();
    assert(!steer_stream && enabled_calls==before); /* lease expires without motion */
    reset_wire();tune_command(3,"T,1");assert(steer_stream);
    for(n=0;n<45;n++) {
        reset_wire();tick+=50;faraway_report();drain();
        if(strstr(wire,"[\"V7\",")) {
            if(last_status && tick-last_status>max_status_gap)max_status_gap=tick-last_status;
            last_status=tick;++status_packets;
        }
        if(strstr(wire,"[\"S1\","))++steer_packets;
    }
    assert(status_packets>=5 && steer_packets>=5 && max_status_gap<500);
    printf("PASS: subscribed S1/status scheduling, %u steering/%u status packets, max status gap %ums; no queued telemetry backlog\n",steer_packets,status_packets,max_status_gap);
    tune_command(4,"T,0");assert(!steer_stream);
    tune_command(5,"K,4");stopped();assert(pid_steer.kp==2.5f && pid_steer.error_int_max==FLT_MAX);
    field_map_reset(tick);task_goto_start(285,180,50);task_goto(285,180,50);
    assert(pid_steer.kp==2.5f && pid_steer.ki==.5f && pid_steer.error_int_max==FLT_MAX);
    task_emergency_stop();
    puts("PASS: PID reset preserves configuration; I accumulates once per sample; live CRC gain updates persist across stop/KEY1/goto, do not start motion, deduplicate; optional S1 CRC and lease expiry");
    return 0;
}

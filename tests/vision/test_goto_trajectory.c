
/* Actual firmware controllers plus HAL stubs; no serial port or hardware access. */
#define main firmware_regression_main
#include "test_static.c"
#undef main

static unsigned trajectory_count, trajectory_failed, overshoot_count, arrived_count;
static float worst_side, worst_error, worst_ratio;
static FILE *trajectory_csv;

static void trajectory_case(const char *name, float x0, float y0, float yaw0,
                            float tx, float ty, float speed, int plant, int jitter,
                            int retarget, int save_trace)
{
    float x=x0,y=y0,yaw=yaw0,actual[4]={0},previous[4],step_seconds;
    float direct=hypotf(tx-x0,ty-y0),ux=(tx-x0)/direct,uy=(ty-y0)/direct;
    float heading=atan2f(sinf(atan2f(ux,uy)-yaw0),cosf(atan2f(ux,uy)-yaw0));
    float path=0,side=0,turn_total=0,backtrack=0,old_distance=direct,elapsed=0;
    float peak_change=0,peak_wheel=0,final_error,ratio,px=x,py=y,pyaw=yaw;
    const float wheel_limit=900.0f*3.1415926f*wheel_diameter*ENCODER_SPEED_AT_PWM1000/(40810.0f*1000.0f);
    unsigned i,j,dt_ms;int failed=0,switched=0;
    float stop_error=-1,tracking_backtrack=0,stop_allowance;
    TaskResult result=TASK_RUNNING;
    task_emergency_stop();field_map_reset(tick);
    for(i=0;i<2000 && (result!=TASK_DONE || wheel_peak(actual)>.01f);i++) {
        float fw,ri,w,mid,d,dx,dy;
        dt_ms=jitter?(i%2?30:10):20;step_seconds=dt_ms*.001f;
        /* Model 0: ideal wheels. 1: 40ms actuator lag.
         * 2: same lag + one sample of pose delay + 0.05deg yaw/0.05cm pose noise. */
        field_map.robot.x=plant==2?px+.05f*sinf(i*1.7f):x;
        field_map.robot.y=plant==2?py+.05f*cosf(i*1.3f):y;
        field_map.robot.yaw=plant==2?pyaw+.05f*3.1415926f/180*sinf(i*2.1f):yaw;
        field_map.robot.tick=tick;
        px=x;py=y;pyaw=yaw;
        if(retarget && !switched && elapsed>=1.0f) {
            tx=60;ty=190;switched=1;
            /* A new click cancels old motion, resets the PID and starts the new target. */
            memcpy(previous,wheel_applied,sizeof(previous));
            assert(task_goto_start(tx,ty,speed));
            assert(wheel_peak(wheel_applied)==0);
        }
        result=task_goto(tx,ty,speed);
        if(result==TASK_FAILED){failed=1;break;}
        if(result==TASK_DONE) {
            if(stop_error<0)stop_error=hypotf(tx-field_map.robot.x,ty-field_map.robot.y);
            if(body_motion.target_forward!=0 || body_motion.target_right!=0 || body_motion.target_turn!=0)failed=1;
            if(motor_drive_enabled || pid_steer.output!=0)failed=1;
        }
        memcpy(previous,wheel_applied,sizeof(previous));
        tick+=dt_ms;motor_process();elapsed+=step_seconds;
        for(j=0;j<4;j++) {
            float change=fabsf(wheel_applied[j]-previous[j])/step_seconds;
            if(change>peak_change)peak_change=change;
            if(fabsf(wheel_applied[j])>peak_wheel)peak_wheel=fabsf(wheel_applied[j]);
            actual[j]+=(wheel_applied[j]-actual[j])*(plant?step_seconds/(.04f+step_seconds):1);
            if(htim1.compare[j]>7560)failed=1;
        }
        fw=(actual[0]+actual[1]+actual[2]+actual[3])*.25f*1.028666f;
        ri=(-actual[0]+actual[1]-actual[2]+actual[3])*.25f*.952092f;
        w=(-actual[0]+actual[1]+actual[2]-actual[3])/(2*(wheel_X_length+wheel_Y_length));
        mid=yaw+w*step_seconds*.5f;
        x+=(ri*cosf(mid)+fw*sinf(mid))*step_seconds;
        y+=(-ri*sinf(mid)+fw*cosf(mid))*step_seconds;yaw+=w*step_seconds;
        path+=hypotf(fw,ri)*step_seconds;turn_total+=fabsf(w)*step_seconds*180/3.1415926f;
        dx=x-x0;dy=y-y0;d=fabsf(dx*uy-dy*ux);if(d>side)side=d;
        d=hypotf(tx-x,ty-y);if(d>old_distance && !switched){backtrack+=d-old_distance;if(result!=TASK_DONE)tracking_backtrack+=d-old_distance;}old_distance=d;
        if(trajectory_csv && save_trace)fprintf(trajectory_csv,"%s,%d,%.3f,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            name,plant,elapsed,x,y,yaw*180/3.1415926f,tx,ty,speed,body_motion.turn,pid_steer.error_int,peak_change);
    }
    final_error=hypotf(tx-x,ty-y);ratio=path/direct;
    if(result==TASK_DONE)++arrived_count;
    /* Verify control arrival separately from coast after the requested immediate
     * stop. No-ramp/constant-speed mode cannot promise the old physical 4cm
     * result at 100cm/s with 40ms motor lag and one sample of delayed position.
     * Bound coast by the model tau + pose delay + one sample, never alter speed. */
    stop_allowance=GOTO_POSITION_TOLERANCE_CM+speed*((plant?.04f:0)+(plant==2?.03f:0)+.03f)+.1f;
    if(stop_error>=0 && final_error>4.001f)++overshoot_count;
    if(result!=TASK_DONE || i>=2000 || stop_error<0 || stop_error>2.001f ||
       final_error>stop_allowance || peak_wheel>wheel_limit+.001f)failed=1;
    if(!retarget && (path>direct*1.15f+3+(plant?.04f*speed:0) || side>5 || turn_total>fabsf(heading)*180/3.1415926f+35 ||
                    tracking_backtrack>3 || elapsed>6+2*direct/speed))failed=1;
    if(retarget && (elapsed>15 || path>400 || turn_total>360))failed=1;
    if(!isfinite(path) || !isfinite(turn_total))failed=1;
    ++trajectory_count;if(failed)++trajectory_failed;
    if(!retarget){if(side>worst_side)worst_side=side;if(ratio>worst_ratio)worst_ratio=ratio;}
    if(final_error>worst_error)worst_error=final_error;
    if(failed || save_trace || final_error>4.001f)printf("%s %s plant=%d speed=%.0f distance=%.2f time=%.2f path=%.2f side=%.2f back=%.2f turn=%.2f error=%.2f accel=%.3f\n",
        failed?"FAIL":"PASS",name,plant,speed,direct,elapsed,path,side,backtrack,turn_total,final_error,peak_change);
}

int main(int argc,char **argv)
{
    int plant,speed_index,distance_index,angle;
    float speeds[]={20,50,100},distances[]={10,80,150};
    char name[80];
    Motor_Init();
    if(argc>1){trajectory_csv=fopen(argv[1],"w");assert(trajectory_csv);fprintf(trajectory_csv,"case,plant,time,x,y,yaw,target_x,target_y,speed,turn,integral,max_wheel_accel\n");}
    for(plant=0;plant<3;plant++) {
        trajectory_case("field_center",285,80,0,150,150,50,plant,0,0,1);
        trajectory_case("screenshot_goal",285,80,0,175,145,50,plant,0,0,1);
        trajectory_case("rear",285,80,0,285,0,50,plant,0,0,1);
        trajectory_case("near",150,150,0,153,150,50,plant,0,0,1);
        trajectory_case("yaw_wrap",150,150,359*3.1415926f/180,150,250,50,plant,1,0,1);
        trajectory_case("unwrapped_yaw",150,150,721*3.1415926f/180,60,190,50,plant,1,0,1);
        trajectory_case("retarget",150,150,0,240,200,50,plant,0,1,1);
        for(speed_index=0;speed_index<3;speed_index++)for(distance_index=0;distance_index<3;distance_index++)for(angle=-180;angle<180;angle+=30) {
            float rad=angle*3.1415926f/180,dist=distances[distance_index];
            snprintf(name,sizeof(name),"grid_s%d_d%d_a%d",(int)speeds[speed_index],(int)dist,angle);
            trajectory_case(name,150,150,0,150+dist*sinf(rad),150+dist*cosf(rad),speeds[speed_index],plant,0,0,0);
        }
    }
    if(trajectory_csv)fclose(trajectory_csv);
    task_emergency_stop();stopped();
    printf("TRAJECTORY: %u cases, %u failures; max side %.3fcm, max final error %.3fcm, max path/direct %.3f\n",trajectory_count,trajectory_failed,worst_side,worst_error,worst_ratio);
    printf("ARRIVAL: %u reached, %u not reached within simulated 40s\n",arrived_count,trajectory_count-arrived_count);
    printf("COAST: %u cases exceed 4cm final error after arrival stop; no endpoint deceleration, controller arrival verified separately\n",overshoot_count);
    return trajectory_failed?1:0;
}

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "vision.h"
#include "motor.h"
#include "../../../project/Core/Src/vision_simple.c"
#include "../../../project/Core/Src/pid.c"
#include "../../../project/Core/Src/motor.c"
#include "../../../project/Core/Src/faraway.c"
#include "../../../project/Core/Src/servo.c"
#include "../../../project/Core/Src/key.c"
#include "../../../project/Core/Src/decode_move.c"

TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart5 = { UART5 };
VisionResult vision;
TIM_HandleTypeDef htim9;
static uint16_t pressed_pin;
Encoder_t encoder1, encoder2, encoder3, encoder4;

static unsigned enabled_calls;
static char wire[8192];
static unsigned wire_size;
static uint32_t tick, last_complete;
static uint8_t *pending;
static uint16_t pending_size;
static unsigned transfers;
uint32_t HAL_GetTick(void) { return tick; }
uint32_t scheduler_get_tick(void) { return tick; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port,uint16_t pin) { (void)port;return (pin&pressed_pin)?GPIO_PIN_RESET:GPIO_PIN_SET; }
void Error_Handler(void) { assert(0); }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{ if (port == GPIOE && state == GPIO_PIN_SET) ++enabled_calls; (void)pin; }
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t c) { (void)h; (void)c; return HAL_OK; }
void encoder_process(void) { }
int64_t encoder_get_count(const Encoder_t *e) { return e->count; }
int32_t encoder_get_speed(const Encoder_t *e) { return e->speed; }
void vision_uart_tx_complete(UART_HandleTypeDef *h) { (void)h; }
int HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ (void)h; (void)p; (void)n; return HAL_OK; }
int HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ assert(h->Instance==UART5); assert(n<=64 && !pending); if(transfers)assert((uint32_t)(tick-last_complete)>=5); pending=p;pending_size=n;++transfers;return HAL_OK; }
int HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
int HAL_UART_AbortTransmit(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
static void drain(void) {
    unsigned guard=0;
    while(tx_head!=tx_tail || tx_length){
        assert(++guard<1000);
        if(pending){
            assert(wire_size+pending_size<sizeof(wire));
            memcpy(wire+wire_size,pending,pending_size);wire_size+=pending_size;wire[wire_size]=0;
            tick+=(pending_size*10U+114U)/115U;
            pending=NULL;last_complete=tick;HAL_UART_TxCpltCallback(&huart5);
            assert(!pending); /* callback must leave a quiet gap */
            tick+=4;faraway_process();assert(!pending);
            tick+=1;
        } else tick+=5;
        faraway_process();
    }
}
static void reset_wire(void) { drain(); wire_size=0; wire[0]=0; }
static void command(const char *text)
{
    while (*text) { rx_byte = (uint8_t)*text++; faraway_uart_rx_complete(&huart5); }
    faraway_process(); drain();
}

static uint32_t simulation_frame, simulation_age;
static void cameras(int distance, float angle, int valid, int roi)
{
    unsigned i;
    vision.connected=1;
    for(i=0;i<2;i++) {
        VisionObservation *o=i?&vision.lower:&vision.upper;
        memset(o,0,sizeof(*o));o->fresh=o->available=1;o->updated_ms=tick;
        o->frame_id=++simulation_frame;o->status=VISION_OBS_EMPTY;o->age_ms=simulation_age;
        if((!i&&distance>0)||(i&&roi>=0)) {
            o->status=VISION_OBS_OK;o->count=1;o->objects[0].kind=VISION_GREEN;
            o->objects[0].distance_mm=i?-1:distance;o->objects[0].angle=angle;
            o->objects[0].angle_present=!i;
            o->objects[0].distance_valid=o->objects[0].angle_valid=valid&&!i;
            o->objects[0].in_roi=i&&roi==1;
        }
    }
}
static void frame(int distance,float angle,int valid,int roi)
{
    tick+=20;cameras(distance,angle,valid,roi);task_observe_lower();move();motor_process();
}
static void stopped(void)
{
    unsigned i;assert(!motor_drive_enabled);
    for(i=0;i<4;i++)assert(htim1.compare[i]==0);
}
static void key_press(uint16_t pin)
{
    pressed_pin=pin;Key_Update();Key_Update();pressed_pin=0;Key_Update();Key_Update();
}
static void wheel_limit_tests(void)
{
    float base[4], scaled[4], peak=0, scale, rotation, factor, expected1,expected2;
    unsigned i;
    task_stop();
    Motor_SetDriveEnabled(1);
    /* 360deg/s is passed through, with 50cm/s still present, below wheel maximum. */
    for(i=0;i<84;i++) {tick+=20;drive(50,360);}
    rotation=(wheel_X_length+wheel_Y_length)*0.5f*360.0f*3.1415926f/180.0f;
    factor=40810.0f*1000.0f/(3.1415926f*wheel_diameter*(float)ENCODER_SPEED_AT_PWM1000);
    expected1=(50.0f/1.028666f-rotation)*factor;
    expected2=(50.0f/1.028666f+rotation)*factor;
    assert(fabsf(pid_motor1.target-expected1)<1.1f && fabsf(pid_motor2.target-expected2)<1.1f);
    drive(50,400);assert(pid_motor2.target>expected2); /* no 360 degree ceiling either */
    drive(50,0);assert(fabsf(pid_motor1.target-50.0f/1.028666f*factor)<1.1f); /* at steady forward speed */
    body_move(50,20,180);
    base[0]=pid_motor1.target;base[1]=pid_motor2.target;base[2]=pid_motor3.target;base[3]=pid_motor4.target;
    /* Independently derive unrounded wheel values before requesting an overload. */
    rotation=(wheel_X_length+wheel_Y_length)*0.5f*180.0f*3.1415926f/180.0f;
    base[0]=(50.0f/1.028666f-20.0f/0.952092f-rotation)*factor;
    base[1]=(50.0f/1.028666f+20.0f/0.952092f+rotation)*factor;
    base[2]=(50.0f/1.028666f-20.0f/0.952092f+rotation)*factor;
    base[3]=(50.0f/1.028666f+20.0f/0.952092f-rotation)*factor;
    for(i=0;i<4;i++)if(fabsf(base[i])>peak)peak=fabsf(base[i]);
    body_move(500,200,1800);
    scaled[0]=pid_motor1.target;scaled[1]=pid_motor2.target;scaled[2]=pid_motor3.target;scaled[3]=pid_motor4.target;
    scale=900.0f/peak;
    for(i=0;i<4;i++)assert(fabsf(scaled[i])<=900 && fabsf(scaled[i]-base[i]*scale)<1.1f);
    body_move(-500,-200,-1800);
    assert(fabsf(pid_motor1.target+scaled[0])<1.1f && fabsf(pid_motor2.target+scaled[1])<1.1f);
    body_move(0,0,0);assert(pid_motor1.target==0 && pid_motor2.target==0);
    MotorSetSpeed1(5000);MotorSetSpeed2(-5000);
    assert(pid_motor1.target==900 && pid_motor2.target==-900);
    assert(pid_motor1.output_max==900 && pid_motor4.output_max==900);
    MotorSetPWM(1800,-900,450,-225);
    assert(pid_motor1.output==900 && pid_motor2.output==-450 && pid_motor3.output==225 && pid_motor4.output==-112.5f);
    motor_process();assert(htim1.compare[TIM_CHANNEL_4]==7560 && htim1.compare[TIM_CHANNEL_3]==3780);
    MotorSetPWM(900,-900,900,-900);motor_process();
    for(i=0;i<4;i++)assert(htim1.compare[i]==7560); /* denominator remains 1000, not 900 */
    /* Final output path still enforces 90% even if a controller produces excess. */
    pid_motor1.output=2000;pid_motor2.output=-2000;motor_process();
    assert(htim1.compare[TIM_CHANNEL_4]==7560 && htim1.compare[TIM_CHANNEL_3]==7560);
    MotorSetPWM(INT32_MIN,INT32_MAX,0,0);motor_process();
    assert(htim1.compare[TIM_CHANNEL_4]<=7560 && htim1.compare[TIM_CHANNEL_3]<=7560);
    task_stop();stopped();
    puts("PASS: steady 50cm/s + unrestricted 360/400deg/s, four-wheel proportional saturation, +/-900 targets, 90% PID/PWM bound and denominator, immediate stop");
}
static void forward_acceleration_tests(void)
{
    unsigned i;
    float previous, rotation_target;
    uint32_t saved_tick=tick;
    drive_stop();
    for(i=1;i<=84;i++) {
        previous=drive_forward;tick+=20;drive(50,0);
        assert(drive_forward-previous<=0.6001f);
        assert(fabsf(drive_forward-(i<84?0.6f*i:50.0f))<0.001f);
    }
    assert(drive_forward==50.0f);
    /* 50 is a setpoint, not a cap: a larger request is still reached. */
    for(i=0;i<84;i++) {tick+=20;drive(100,0);}
    assert(drive_forward==100.0f && pid_motor1.target>300);
    drive(50,0);assert(drive_forward==100.0f); /* no elapsed time, no linear change */
    tick+=20;drive(50,0);assert(fabsf(drive_forward-99.4f)<0.001f);
    drive_stop();stopped();assert(drive_forward==0);
    body_move(0,0,400);rotation_target=pid_motor2.target;
    drive(0,400);assert(pid_motor2.target==rotation_target); /* no angular ramp/cap */
    drive_stop();tick+=7;drive(50,0);assert(fabsf(drive_forward-0.21f)<0.0001f);
    tick+=33;drive(50,0);assert(fabsf(drive_forward-1.2f)<0.0001f);
    tick=0xfffffff0U;drive_stop();tick+=20;drive(50,0);
    assert(fabsf(drive_forward-0.6f)<0.0001f);
    task_stop();stopped();assert(drive_forward==0);
    tick=saved_tick;drive_stop();
    puts("PASS: forward acceleration 30cm/s2, 50/100cm/s reachable, unrestricted immediate turn, irregular/duplicate/wrapped timing and immediate stop reset");
}
static void steering_pid_tests(void)
{
    task_stop();task_prepare();simulation_age=0;
    cameras(600,10,0,0);target_angle=10;
    assert(steering_pid()==40); /* P only on acquisition, no D kick */
    cameras(600,8,0,0);target_angle=8;
    assert(steering_pid()==28); /* 4*8 + 2*(8-10), damps approaching zero */
    assert(pid_steer.error_last==10 && pid_steer.error_now==8);
    tick+=20;assert(steering_pid()==28 && pid_steer.error_last==10); /* same frame holds D */
    cameras(600,8,0,0);assert(steering_pid()==32); /* new steady frame */
    cameras(600,12,0,0);target_angle=12;assert(steering_pid()==56); /* growing error */
    cameras(600,-8,0,0);target_angle=-8;assert(steering_pid()==-72);
    cameras(600,2,0,0);target_angle=2;assert(steering_pid()==0 && !steer_seen);
    cameras(600,5,0,0);target_angle=5;assert(steering_pid()==20);
    drive_stop();cameras(600,-15,0,0);target_angle=-15;
    assert(steering_pid()==-60); /* reacquisition discards old derivative history */
    cameras(600,80,0,0);target_angle=80;assert(steering_pid()==510); /* no 360 speed cap */
    assert(pid_steer.error_int==0 && pid_steer.ki==0);
    task_stop();assert(!steer_seen && pid_steer.output==0);stopped();
    puts("PASS: actual pid.c steering PD, damping, growing/crossing errors, new-frame-only updates, deadband reset, reacquisition without kick, no added turn cap");
}
static void mission_tests(void)
{
    unsigned i;
    float forward_target, right, left;
    wheel_limit_tests();
    forward_acceleration_tests();
    steering_pid_tests();
    task_stop();Servo_SetAngle(50,0);memset(&vision,0,sizeof(vision));
    simulation_age=285;
    frame(600,15,0,1);assert(step==0 && servo_angle==50 && task_info.held_kind==VISION_NONE);stopped();
    key_press(key1_Pin);move();assert(step==10);
    frame(600,15,0,0);assert(step==30);stopped();
    frame(600,0,0,0);assert(motor_drive_enabled && pid_motor1.target==pid_motor2.target);
    assert(fabsf(drive_forward-0.6f)<0.0001f);
    for(i=0;i<83;i++)frame(600,0,0,0);
    assert(drive_forward==50.0f);
    forward_target=pid_motor1.target;
    frame(200,0,0,0);assert(pid_motor1.target==forward_target);
    frame(50,0,0,0);assert(pid_motor1.target==forward_target); /* no near slowdown */
    frame(600,15,0,0);assert(step==30 && motor_drive_enabled);
    assert(pid_motor1.target>0 && pid_motor2.target>pid_motor1.target);
    assert(fabsf((pid_motor1.target+pid_motor2.target)/2-forward_target)<=1.0f);
    right=pid_motor2.target;left=pid_motor1.target;
    frame(600,-15,0,0); /* D reacts to the crossing; next steady frame is symmetric. */
    frame(600,-15,0,0);frame(600,15,0,0);frame(600,15,0,0);
    right=pid_motor2.target;left=pid_motor1.target;
    frame(600,-15,0,0);frame(600,-15,0,0);
    assert(pid_motor1.target==right && pid_motor2.target==left);
    frame(600,2,0,0);assert(pid_motor1.target==forward_target && pid_motor2.target==forward_target);
    frame(600,30,0,0);right=pid_motor2.target;left=pid_motor1.target;
    frame(600,80,0,0);assert(pid_motor1.target<left && pid_motor2.target>right); /* no body turn cap */
    frame(0,0,0,0);assert(step==30 && task_info.reason==TASK_TARGET_LOST);stopped();
    tick+=60000;frame(0,0,0,0);assert(step==30);stopped();
    frame(600,15,0,0);assert(step==30 && motor_drive_enabled);
    assert(fabsf(drive_forward-0.6f)<0.0001f);

    /* A single lower ROI frame, even with no upper target, closes immediately. */
    frame(0,0,0,1);assert(step==40 && servo_angle==100 && task_info.held_kind==VISION_GREEN);stopped();
    assert(drive_forward==0);
    frame(0,0,0,0);assert(servo_angle==100 && task_info.held_kind==VISION_GREEN);
    for(i=0;i<30;i++)frame(0,0,0,0);
    assert(step==99 && servo_angle==100 && task_info.held_kind==VISION_GREEN);stopped();
    reset_wire();tick+=151;faraway_report();drain();
    assert(strstr(wire,"[99,0,0,3,1]]*")); /* actual MCU telemetry retains cargo when ROI=0 */
    tick+=60000;vision.connected=0;vision.uart_errors++;move();
    assert(step==99 && servo_angle==100 && task_info.held_kind==VISION_GREEN);stopped();
    cameras(600,0,0,0);vision.lower.objects[0].kind=VISION_BLUE;move();
    assert(servo_angle==100 && task_info.held_kind==VISION_GREEN);stopped();
    key_press(key1_Pin);assert(step==99 && servo_angle==100); /* no dropping on restart */
    key_press(key4_Pin);assert(step==0 && servo_angle==50 && task_info.held_kind==VISION_NONE);stopped();
    simulation_age=0;

    /* ROI event cannot be lost when multiple packets arrive before move(). */
    key_press(key1_Pin);move();frame(600,0,0,0);assert(step==30);
    cameras(200,0,0,1);task_observe_lower();assert(pending_kind==VISION_GREEN);
    cameras(0,0,0,0);task_observe_lower();assert(task_info.inside_frames==0);
    move();assert(step==40 && servo_angle==100 && task_info.held_kind==VISION_GREEN);stopped();
    tick+=401;vision.connected=0;vision.uart_errors++;move();assert(step==40 && servo_angle==100);
    tick+=100;move();assert(step==99 && servo_angle==100);stopped();
    key_press(key4_Pin);assert(task_info.held_kind==VISION_NONE && pending_kind==VISION_NONE);

    /* A block already at the claw need not reappear in the upper camera. */
    key_press(key1_Pin);move();frame(0,0,0,1);
    assert(step==40 && servo_angle==100 && task_info.held_kind==VISION_GREEN);stopped();
    key_press(key4_Pin);
    /* Stop also cancels an accepted event before the actuator command. */
    key_press(key1_Pin);move();cameras(200,0,0,1);task_observe_lower();assert(pending_kind==VISION_GREEN);
    key_press(key4_Pin);move();assert(step==0 && servo_angle==50 && pending_kind==VISION_NONE);stopped();

    /* Nearest ordinary block, excluding closer nonordinary objects. */
    key_press(key1_Pin);move();cameras(900,30,0,0);vision.upper.count=3;
    vision.upper.objects[1]=vision.upper.objects[0];vision.upper.objects[1].distance_mm=600;
    vision.upper.objects[1].angle=-15;vision.upper.objects[2]=vision.upper.objects[0];
    vision.upper.objects[2].kind=VISION_BLACK;vision.upper.objects[2].distance_mm=100;
    move();assert(step==30 && target_angle==-15);
    /* Lower absence prevents driving but automatically recovers. */
    cameras(600,0,0,0);vision.lower.fresh=0;move();assert(step==30 && task_info.reason==TASK_LOWER_WAIT);stopped();
    frame(600,0,0,0);assert(motor_drive_enabled);
    cameras(600,0,0,0);vision.upper.objects[0].angle_present=0;move();stopped();
    cameras(600,0,0,0);vision.upper.objects[0].distance_mm=-1;move();stopped();
    cameras(600,0,0,0);vision.upper.objects[0].angle=(float)(HUGE_VAL-HUGE_VAL);move();stopped();
    frame(600,0,0,0);assert(motor_drive_enabled);
    vision.upper.age_ms=400;move();assert(motor_drive_enabled);
    vision.upper.age_ms=401;move();assert(task_info.reason==TASK_UPPER_WAIT);stopped();
    frame(600,0,0,0);assert(motor_drive_enabled);
    vision.uart_errors++;move();assert(step==30 && task_info.reason==TASK_UART_ERROR);stopped();
    frame(600,0,0,0);assert(motor_drive_enabled);
    /* Ambiguous/nonordinary/hazard ROI must not be recorded as held cargo. */
    cameras(600,0,0,1);vision.lower.count=2;vision.lower.objects[1]=vision.lower.objects[0];move();
    assert(step==30 && task_info.reason==TASK_AMBIGUOUS && pending_kind==VISION_NONE);stopped();
    cameras(600,0,0,1);vision.lower.objects[0].kind=VISION_BLACK;move();
    assert(pending_kind==VISION_NONE && servo_angle==50);stopped();
    cameras(600,0,0,1);vision.lower.objects[0].kind=VISION_BLUE;move();
    assert(task_info.reason==TASK_HAZARD && task_info.held_kind==VISION_NONE);stopped();
    frame(600,0,0,0);assert(motor_drive_enabled);
    key_press(key4_Pin);key_press(key1_Pin|key4_Pin);assert(step==0);stopped();
    Motor_SetDriveEnabled(1);MotorSetPWM(1000,-1000,1000,-1000);motor_process();
    for(i=0;i<4;i++)assert(htim1.compare[i]==7560);
    key_press(key4_Pin);stopped();motor_process();stopped();
    puts("PASS: 50cm/s forward with uncapped proportional steering/deadband, loss recovery, first ROI grip, queued event latch, cargo held across occlusion/faults, explicit KEY4 release");
}

int main(void)
{
    unsigned i, max_wire;
    Servo_Init(); assert(Servo_GetAngle()==50 && htim9.compare[0]==1056);
    Servo_SetAngle(100,0); assert(Servo_GetAngle()==100 && htim9.compare[0]==1611);
    pressed_pin=key2_Pin;Key_Update();Key_Update();assert(Servo_GetAngle()==100);
    Servo_SetAngle(50,0); assert(Servo_GetAngle()==50);
    pressed_pin=0;Key_Update();Key_Update();
    Motor_Init(); assert(enabled_calls == 0);
    MotorEnable1(1); MotorEnable2(1); MotorEnable3(1); MotorEnable4(1);
    MotorSetSpeed1(1000); MotorSetSpeed2(-1000); MotorSetSpeed3(500); MotorSetSpeed4(-500);
    motor_process(); assert(enabled_calls == 0);
    for (i=0; i<4; ++i) assert(htim1.compare[i] == 0);
    MotorSetPWM(1000,-1000,500,-500); motor_process();
    for (i=0; i<4; ++i) assert(htim1.compare[i] == 0);
    assert(faraway_init());
    command("RUN,1\nENABLE,1\nTARGET,900\nPID,2,1,0\n");
    assert(strstr(wire,"ERR,TASK_CONTROL") && !strstr(wire,"OK,"));
    assert(enabled_calls == 0);
    reset_wire();
    vision.connected=1; vision.acknowledged=vision.vision_ready=0; vision.danger_flag=1;
    strcpy(vision.fault_code,"rv2_observation");
    vision.upper.available=vision.upper.fresh=1; vision.upper.frame_id=42; vision.upper.age_ms=40;
    vision.upper.count=3;
    for(i=0;i<3;++i) {
        VisionObject *o=&vision.upper.objects[i];
        o->target_id=1+2*i; o->kind=VISION_GREEN; o->distance_mm=500;
        o->angle=i==0?30.0f:i==1?-30.0f:0.0f; o->angle_present=i<2;
    }
    vision.lower=vision.upper;vision.lower.updated_ms=tick;vision.lower.count=2;vision.lower.objects[0].in_roi=1;
    servo_angle=100;
    vision.upper.updated_ms=tick; faraway_report(); drain(); puts(wire);
    assert(strstr(wire,"[1,500,3000,0]") && strstr(wire,"[1,500,-3000,0]"));
    assert(strstr(wire,"[1,500,null,0]"));
    reset_wire(); vision.upper.fresh=0; tick+=200; vision.upper.updated_ms=tick; faraway_report(); drain();
    assert(strstr(wire,",0,0,[],"));
    /* Full-size telemetry remains bounded; stale/raw coordinates never gain valid. */
    reset_wire(); vision.upper.fresh=1;vision.upper.count=12;
    for(i=0;i<12;++i) {
        VisionObject *o=&vision.upper.objects[i]; memset(o,0,sizeof(*o));
        o->target_id=UINT32_MAX; o->kind=VISION_BLUE_MATERIAL_ZONE;
        o->distance_mm=65535; o->angle=-135; o->angle_present=1;
    }
    tick+=200; vision.upper.updated_ms=tick; vision.lower=vision.upper; faraway_report(); drain();
    assert(wire_size < 768 && strstr(wire,"[8,65535,-13500,0]") && g_faraway_tx_drop_count==0);
    max_wire=wire_size;
    for(i=0;i<40;i++){
        reset_wire();tick+=200;vision.upper.updated_ms=tick;vision.lower=vision.upper;faraway_report();drain();
        assert(strstr(wire,"*"));
        assert(VSP_Crc16((const uint8_t *)wire,(uint16_t)(strchr(wire,'*')-wire))==(unsigned)strtoul(strchr(wire,'*')+1,NULL,16));
        { const char *item=wire;unsigned count=0;while((item=strstr(item,"[8,65535,-13500,0]"))!=NULL){++count;++item;}assert(count==12); }
    }
    reset_wire();tick+=200;vision.upper.updated_ms=tick;faraway_report();
    faraway_vision_report();assert(g_faraway_tx_drop_count==1);drain();
    tick=UINT32_MAX-2;tx_next_ms=tick;reset_wire();faraway_vision_report();drain();
    {
        uint32_t report_at;
        reset_wire();tick+=200;report_at=tick;faraway_report();drain();assert(wire_size>0);
        reset_wire();tick=report_at+149;faraway_report();assert(!pending && wire_size==0);
        tick=report_at+150;faraway_report();drain();assert(wire_size>0);
    }
    reset_wire();
    vision.upper.age_ms=218;vision.upper.updated_ms=tick-182;vision.upper.fresh=1;
    faraway_vision_report();drain();assert(strstr(wire,",42,400,0,1,0,["));
    reset_wire();vision.upper.updated_ms=tick-183;
    faraway_vision_report();drain();assert(strstr(wire,",42,401,2,0,0,[],"));
    reset_wire();vision.upper.age_ms=UINT32_MAX;
    faraway_vision_report();drain();assert(strstr(wire,",42,null,2,0,0,[],"));
    reset_wire();vision.lower.frame_id=123;vision.lower.age_ms=218;vision.lower.updated_ms=tick-182;
    faraway_vision_report();drain();assert(strstr(wire,"[123,400,0,1,"));
    reset_wire();vision.lower.updated_ms=tick-183;
    faraway_vision_report();drain();assert(strstr(wire,"[123,401,2,0,[]]"));
    printf("PASS: motor EN/PWM interlocks, rejected remote motion, compact CRC telemetry, NA, expiry, 64-byte chunks/5ms gaps, ring wrap, backlog skip, time rollover, 150ms reporting, 12+12 objects (%u bytes), servo 50/100 PWM and key override blocked\n",max_wire);
    mission_tests();
    return 0;
}

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "vision.h"
#include "motor.h"
#include "../../../project/Core/Src/vision_simple.c"
#include "../../../project/Core/Src/pid.c"
#include "../../../project/Core/Src/motor.c"
#include "../../../project/Core/Src/field_map.c"
#include "../../../project/Core/Src/imu.c"
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
volatile uint32_t g_hdz601_attitude_count, g_hdz601_attitude_tick;
static hdz601_attitude_t fake_attitude;
static int fake_imu_ready;
bool hdz601_get(hdz601_attitude_t *p) { *p=fake_attitude;return fake_imu_ready!=0; }
static void imu_sample(float degrees) {
    fake_imu_ready=1;fake_attitude.yaw=degrees;
    ++g_hdz601_attitude_count;g_hdz601_attitude_tick=tick;imu_process();
}
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
static void stopping_requested(void)
{
    assert(body_motion.target_forward==0 && body_motion.target_right==0 && body_motion.target_turn==0);
}
static void key_press(uint16_t pin)
{
    pressed_pin=pin;Key_Update();Key_Update();pressed_pin=0;Key_Update();Key_Update();
}

static void advance_motors(unsigned cycles)
{
    while(cycles--) { tick+=20; motor_process(); }
}
static void wheel_limit_tests(void)
{
    float base[4], peak=0, scale, rotation, factor, expected1,expected2;
    unsigned i;
    task_emergency_stop();Motor_SetDriveEnabled(1);
    drive(50,360);advance_motors(100);
    rotation=(wheel_X_length+wheel_Y_length)*0.5f*360.0f*3.1415926f/180.0f;
    factor=40810.0f*1000.0f/(3.1415926f*wheel_diameter*(float)ENCODER_SPEED_AT_PWM1000);
    expected1=(50.0f/1.028666f-rotation)*factor;
    expected2=(50.0f/1.028666f+rotation)*factor;
    assert(fabsf(pid_motor1.target-expected1)<.001f && fabsf(pid_motor2.target-expected2)<.001f);
    drive(50,400);advance_motors(100);assert(pid_motor2.target>expected2);
    drive(50,0);advance_motors(100);assert(fabsf(pid_motor1.target-50.0f/1.028666f*factor)<.001f);
    rotation=(wheel_X_length+wheel_Y_length)*0.5f*180.0f*3.1415926f/180.0f;
    base[0]=(50.0f/1.028666f-20.0f/0.952092f-rotation)*factor;
    base[1]=(50.0f/1.028666f+20.0f/0.952092f+rotation)*factor;
    base[2]=(50.0f/1.028666f-20.0f/0.952092f+rotation)*factor;
    base[3]=(50.0f/1.028666f+20.0f/0.952092f-rotation)*factor;
    for(i=0;i<4;i++)if(fabsf(base[i])>peak)peak=fabsf(base[i]);
    body_move(500,200,1800);advance_motors(250);scale=900.0f/peak;
    for(i=0;i<4;i++)assert(fabsf(wheel_applied[i]*factor)<=900.001f && fabsf(wheel_applied[i]*factor-base[i]*scale)<.001f);
    body_move(-500,-200,-1800);advance_motors(250);
    for(i=0;i<4;i++)assert(fabsf(wheel_applied[i]*factor+base[i]*scale)<.001f);
    body_stop();advance_motors(250);assert(pid_motor1.target==0 && pid_motor2.target==0);
    MotorSetSpeed1(5000);MotorSetSpeed2(-2500);advance_motors(250);
    assert(fabsf(pid_motor1.target-900)<.001f && fabsf(pid_motor2.target+450)<.001f);
    assert(pid_motor1.output_max==900 && pid_motor4.output_max==900);
    MotorSetPWM(1800,-900,450,-225);
    assert(pid_motor1.output==900 && pid_motor2.output==-450 && pid_motor3.output==225 && pid_motor4.output==-112.5f);
    motor_process();assert(htim1.compare[TIM_CHANNEL_4]==7560 && htim1.compare[TIM_CHANNEL_3]==3780);
    MotorSetPWM(900,-900,900,-900);motor_process();
    for(i=0;i<4;i++)assert(htim1.compare[i]==7560);
    pid_motor1.output=2000;pid_motor2.output=-2000;motor_process();
    assert(htim1.compare[TIM_CHANNEL_4]==7560 && htim1.compare[TIM_CHANNEL_3]==7560);
    MotorSetPWM(INT32_MIN,INT32_MAX,0,0);motor_process();
    assert(htim1.compare[TIM_CHANNEL_4]<=7560 && htim1.compare[TIM_CHANNEL_3]<=7560);
    task_emergency_stop();stopped();
    puts("PASS: unrestricted body/turn commands, four-wheel common 90% saturation, legacy speed API, 90% PID/PWM output and denominator, immediate stop");
}


static void no_acceleration_tests(void)
{
    float target[4]={-60,80,-20,40};unsigned i;
    task_emergency_stop();Motor_SetDriveEnabled(1);
    Motor_SetWheelSpeedCmS(-60,80,-20,40);motor_process();
    for(i=0;i<4;i++)assert(wheel_applied[i]==target[i]); /* no elapsed-time ramp */
    Motor_SetWheelSpeedCmS(60,-80,20,-40);tick+=7;motor_process();
    for(i=0;i<4;i++)assert(wheel_applied[i]==-target[i]); /* reversal in one cycle */
    body_stop();motor_process();
    for(i=0;i<4;i++)assert(wheel_applied[i]==0);
    body_move(50,0,360);tick+=20;motor_process();
    assert(fabsf(body_motion.forward-50)<.001f && fabsf(body_motion.turn-360)<.001f);
    assert(Motor_StopTime()==0 && body_brake_distance()==0);
    MotorSetSpeed1(123);tick+=20;motor_process();assert(fabsf(pid_motor1.target-123)<.001f);
    MotorSetPWM(0,0,0,0);encoder1.speed=100;
    MotorSetSpeed1(0);MotorSetSpeed2(0);MotorSetSpeed3(0);MotorSetSpeed4(0);motor_process();
    for(i=0;i<4;i++)assert(wheel_applied[i]==0);
    encoder1.speed=0;task_emergency_stop();stopped();
    tick=UINT32_MAX-9;Motor_SetDriveEnabled(1);body_move(100,0,0);tick+=20;motor_process();
    assert(fabsf(body_motion.forward-100)<.001f);
    task_emergency_stop();tick=100000;motor_process();stopped();
    puts("PASS: no software acceleration/deceleration, immediate target/reversal/zero, legacy setters, PWM transition, tick wrap; 90% retained");
}

static void steering_pid_tests(void)
{
    task_emergency_stop();task_prepare();simulation_age=0;
    /* Exercise fixed PD gains independently of the user-tuned task gains. */
    assert(pid_steer.kp==TASK_STEER_KP && pid_steer.ki==TASK_STEER_KI && pid_steer.kd==TASK_STEER_KD);
    pid_steer.kp=4;pid_steer.ki=0;pid_steer.kd=2;
    cameras(600,10,0,0);target_angle=10;
    assert(steering_pid(target_angle, vision.upper.frame_id)==40); /* P only on acquisition, no D kick */
    cameras(600,8,0,0);target_angle=8;
    assert(steering_pid(target_angle, vision.upper.frame_id)==28); /* 4*8 + 2*(8-10), damps approaching zero */
    assert(pid_steer.error_last==10 && pid_steer.error_now==8);
    tick+=20;assert(steering_pid(target_angle, vision.upper.frame_id)==28 && pid_steer.error_last==10); /* same frame holds D */
    cameras(600,8,0,0);assert(steering_pid(target_angle, vision.upper.frame_id)==32); /* new steady frame */
    cameras(600,12,0,0);target_angle=12;assert(steering_pid(target_angle, vision.upper.frame_id)==56); /* growing error */
    cameras(600,-8,0,0);target_angle=-8;assert(steering_pid(target_angle, vision.upper.frame_id)==-72);
    cameras(600,2,0,0);target_angle=2;assert(steering_pid(target_angle, vision.upper.frame_id)==28 && steer_seen);
    /* Inclusive +/-1deg deadband; raw error stays visible, no D kick on exit. */
    cameras(600,.5f,0,0);target_angle=.5f;assert(steering_pid(target_angle, vision.upper.frame_id)==0);
    cameras(600,1,0,0);target_angle=1;assert(steering_pid(target_angle, vision.upper.frame_id)==0);
    cameras(600,-1,0,0);target_angle=-1;assert(steering_pid(target_angle, vision.upper.frame_id)==0);
    cameras(600,0,0,0);target_angle=0;assert(steering_pid(target_angle, vision.upper.frame_id)==0);
    cameras(600,1.01f,0,0);target_angle=1.01f;assert(fabsf(steering_pid(target_angle, vision.upper.frame_id)-4.04f)<.001f);
    cameras(600,5,0,0);target_angle=5;assert(fabsf(steering_pid(target_angle, vision.upper.frame_id)-27.98f)<.001f);
    drive_stop();cameras(600,-15,0,0);target_angle=-15;
    assert(steering_pid(target_angle, vision.upper.frame_id)==-60); /* reacquisition discards old derivative history */
    cameras(600,80,0,0);target_angle=80;assert(steering_pid(target_angle, vision.upper.frame_id)==510); /* no 360 speed cap */
    assert(pid_steer.error_int==0 && pid_steer.ki==0);
    task_emergency_stop();assert(!steer_seen && pid_steer.output==0);stopped();
    assert(steering_set_pid(TASK_STEER_KP,TASK_STEER_KI,TASK_STEER_KD));
    puts("PASS: actual pid.c steering PD, damping, growing/crossing errors, new-frame-only updates, inclusive +/-1deg deadband, measured error retained, no derivative kick on exit, reacquisition without kick, no added turn cap");
}

static void pursuit_stop_tests(void)
{
    task_emergency_stop();task_start();move();frame(600,0,0,0);frame(600,0,0,0);
    assert(fabsf(body_motion.forward-50)<.001f);
    frame(0,0,0,0);stopped();stopping_requested();assert(body_motion.forward==0);
    frame(600,0,0,0);assert(fabsf(body_motion.forward-50)<.001f);
    frame(0,0,0,1);assert(step==40 && servo_angle==100 && body_motion.forward==0);stopped();
    tick+=500;move();assert(step==99 && task_info.held_kind==VISION_GREEN);
    task_emergency_stop();Motor_SetDriveEnabled(1);body_move(50,0,90);motor_process();
    task_stop();stopped();assert(body_motion.forward==0 && body_motion.turn==0);
    Motor_SetDriveEnabled(1);body_move(50,0,90);motor_process();key_press(key4_Pin);stopped();
    puts("PASS: yesterday pursuit stopping behavior, loss/reacquisition without ramp, first ROI grip and cargo latch, KEY4 immediate stop");
}

static void goto_tests(void)
{
    float output,integral;VisionResult saved;
    task_emergency_stop();field_map_reset(tick);saved=vision;
    assert(task_goto(285,180,50)==TASK_RUNNING);
    assert(body_motion.target_forward==50 && body_motion.target_right==0 && body_motion.target_turn==0);
    motor_process();assert(fabsf(body_motion.forward-50)<.001f);
    assert(memcmp(&saved,&vision,sizeof(vision))==0); /* virtual target never overwrites camera */
    field_map.robot.yaw=.2f;field_map.robot.tick+=20;task_goto(285,180,50);
    assert(body_motion.target_forward==50 && body_motion.target_turn<0);
    /* Same error stream through point mapping and actual visual pursuit. */
    output=body_motion.target_turn;integral=pid_steer.error_int;
    task_emergency_stop();task_prepare();drive_stop();
    cameras(1000,-.2f*180/3.1415926f,0,0);task_approach();
    assert(fabsf(body_motion.target_turn-output)<.001f && fabsf(pid_steer.error_int-integral)<.001f);
    task_emergency_stop();field_map_reset(tick);field_map.robot.yaw=359.5f*3.1415926f/180;
    task_goto(285,180,50);assert(body_motion.target_forward==50 && body_motion.target_turn==0);
    task_emergency_stop();field_map_reset(tick);
    task_goto(285,280,120);motor_process();assert(fabsf(body_motion.forward-120)<.01f);
    task_emergency_stop();field_map_reset(tick);task_goto(285,92,50);
    assert(body_motion.target_forward==50); /* no endpoint slowdown */
    task_goto(285,82,50);assert(goto_arrived);stopped();
    assert(task_goto(285,180,0)==TASK_FAILED);stopped();
    assert(task_goto((float)(HUGE_VAL-HUGE_VAL),180,50)==TASK_FAILED);
    task_emergency_stop();field_map_reset(tick);task_goto(185,80,50);
    assert(body_motion.target_forward==50 && body_motion.target_turn<0);
    field_map.robot.yaw=-3.1415926f/2;field_map.robot.tick+=20;task_goto(185,80,50);
    assert(body_motion.target_forward==50 && fabsf(body_motion.target_turn-pid_steer.ki*pid_steer.error_int)<.001f);
    field_map.heading_valid=0;task_goto(185,80,50);stopped();
    task_emergency_stop();stopped();
    puts("PASS: point/body target mapping, shared pursuit PID output, forward from first cycle, no camera dependency/write, no speed cap or endpoint slowdown, pose loss stops");
}

static void goto_latch_pid_tests(void)
{
    unsigned i;float output,integral;
    task_emergency_stop();field_map_reset(tick);
    assert(task_goto_start(200,180,50));task_goto(200,180,50);
    assert(pid_steer.kp==TASK_STEER_KP && pid_steer.ki==TASK_STEER_KI && pid_steer.kd==TASK_STEER_KD);
    output=pid_steer.output;integral=pid_steer.error_int;
    task_goto(200,180,50);assert(pid_steer.output==output && fabsf(pid_steer.error_int-integral)<.001f);
    field_map.robot.x=200;field_map.robot.y=179;
    assert(task_goto(200,180,50)==TASK_DONE && goto_arrived);stopped();
    field_map.robot.x=210;field_map.robot.y=190;field_map.robot.yaw=3.1415926f;
    for(i=0;i<50;i++) {
        field_map.robot.tick=++tick;
        assert(task_goto(200,180,50)==TASK_DONE);stopped();stopping_requested();
    }
    assert(task_goto_start(200,180,50) && !goto_arrived);move();assert(step==50 && !goto_arrived);
    field_map.robot.x=200;field_map.robot.y=180;move();assert(goto_arrived && step==51);
    assert(task_goto(150,100,50)==TASK_RUNNING && !goto_arrived);
    task_emergency_stop();assert(!goto_active && !goto_arrived);stopped();
    puts("PASS: shared PID new-pose-only updates, arrival latch, same-point restart, changed destination and KEY4 cancellation");
}

static void control_command_tests(void)
{
    char text[100], body[80];unsigned i;
    task_emergency_stop();reset_wire();command_seen=0;
    strcpy(body,"C1,100,G,2000,1800,500");
    snprintf(text,sizeof(text),"%s*%04X\n",body,VSP_Crc16((uint8_t*)body,(uint16_t)strlen(body)));
    command(text);assert(step==50 && goto_target.valid && goto_target.x==200 && goto_target.y==180 && goto_target.speed==50);
    assert(strstr(wire,"A1,100,0,50*"));
    field_map_reset(tick);move();tick+=20;motor_process();assert(motor_drive_enabled);
    for(i=0;i<25;i++){tick+=20;move();motor_process();}
    command(text);assert(step==50); /* no duplicate reset */
    strcpy(body,"C1,101,K,4");snprintf(text,sizeof(text),"%s*%04X\n",body,VSP_Crc16((uint8_t*)body,(uint16_t)strlen(body)));
    command(text);stopped();assert(step==0 && !goto_target.valid);
    strcpy(body,"C1,100,G,2000,1800,500");snprintf(text,sizeof(text),"%s*%04X\n",body,VSP_Crc16((uint8_t*)body,(uint16_t)strlen(body)));
    command(text);assert(step==0);stopped();assert(strstr(wire,"A1,100,4,0*"));
    command("C1,102,G,2000,1800,500*0000\n");assert(step==0);stopped();
    strcpy(body,"C1,102,K,2");snprintf(text,sizeof(text),"%s*%04X\n",body,VSP_Crc16((uint8_t*)body,(uint16_t)strlen(body)));
    command(text);assert(step==0 && command_status==1);
    strcpy(body,"C1,103,K,3");snprintf(text,sizeof(text),"%s*%04X\n",body,VSP_Crc16((uint8_t*)body,(uint16_t)strlen(body)));
    command(text);assert(step==0 && command_status==1);
    pressed_pin=key4_Pin;
    strcpy(body,"C1,104,G,2000,1800,500");snprintf(text,sizeof(text),"%s*%04X\n",body,VSP_Crc16((uint8_t*)body,(uint16_t)strlen(body)));
    command(text);assert(step==0 && command_status==2);pressed_pin=0;
    strcpy(body,"C1,105,G,3001,1800,500");snprintf(text,sizeof(text),"%s*%04X\n",body,VSP_Crc16((uint8_t*)body,(uint16_t)strlen(body)));
    command(text);assert(step==0 && command_status==3);
    strcpy(body,"C1,106,K,1");snprintf(text,sizeof(text),"%s*%04X\n",body,VSP_Crc16((uint8_t*)body,(uint16_t)strlen(body)));
    command(text);assert(step==1 && command_status==0);
    strcpy(body,"C1,90,K,4");snprintf(text,sizeof(text),"%s*%04X\n",body,VSP_Crc16((uint8_t*)body,(uint16_t)strlen(body)));
    command(text);assert(step==0);stopped(); /* Even an old STOP always stops. */
    task_info.held_kind=VISION_GREEN;servo_angle=100;
    assert(task_goto_start(200,180,50));move();assert(step==50 && servo_angle==100 && task_info.held_kind==VISION_GREEN);
    field_map.robot.x=200;field_map.robot.y=180;body_motion.forward=body_motion.right=body_motion.turn=0;
    move();assert(step==51);move();stopping_requested();
    task_emergency_stop();command_seen=0;reset_wire();
    puts("PASS: UART5 checksummed goto/KEY1-4, duplicate/stale motion rejection, old KEY4 still stops, physical KEY4 priority, unbound KEY2/3, cargo preserved and case50/51 arrival");
}
static void mission_tests(void)
{
    unsigned i;
    float forward_target, right, left;
    wheel_limit_tests();
    no_acceleration_tests();
    steering_pid_tests();
    pursuit_stop_tests();
    goto_tests();
    goto_latch_pid_tests();
    control_command_tests();
    task_emergency_stop();Servo_SetAngle(50,0);memset(&vision,0,sizeof(vision));
    simulation_age=285;
    frame(600,15,0,1);assert(step==0 && servo_angle==50 && task_info.held_kind==VISION_NONE);stopping_requested();
    key_press(key1_Pin);move();assert(step==10);
    frame(600,15,0,0);assert(step==30);stopping_requested();
    frame(600,0,0,0);assert(motor_drive_enabled && pid_motor1.target==pid_motor2.target);
    assert(fabsf(body_motion.forward-50)<.001f);
    for(i=0;i<83;i++)frame(600,0,0,0);
    assert(fabsf(body_motion.forward-50)<.001f);
    forward_target=pid_motor1.target;
    frame(200,0,0,0);assert(pid_motor1.target==forward_target);
    frame(50,0,0,0);assert(pid_motor1.target==forward_target); /* no near slowdown */
    frame(600,15,0,0);assert(step==30 && motor_drive_enabled && body_motion.target_turn>0);
    right=body_motion.target_turn;
    frame(600,-15,0,0);assert(body_motion.target_turn<0);
    left=body_motion.target_turn;assert(fabsf(left-(-15*TASK_STEER_KP-30*TASK_STEER_KD))<.001f);
    frame(600,2,0,0);assert(fabsf(body_motion.target_turn-(2*TASK_STEER_KP+17*TASK_STEER_KD))<.001f && body_motion.target_forward==50);
    frame(600,30,0,0);right=body_motion.target_turn;
    frame(600,80,0,0);assert(body_motion.target_turn>right); /* pursuit PID unchanged */
    frame(0,0,0,0);assert(step==30 && task_info.reason==TASK_TARGET_LOST);stopping_requested();
    tick+=60000;frame(0,0,0,0);assert(step==30);stopping_requested();
    frame(600,15,0,0);assert(step==30 && motor_drive_enabled);
    assert(fabsf(body_motion.forward-50)<.001f);

    /* A single lower ROI frame, even with no upper target, closes immediately. */
    frame(0,0,0,1);assert(step==40 && servo_angle==100 && task_info.held_kind==VISION_GREEN);stopping_requested();
    assert(body_motion.forward==0);
    frame(0,0,0,0);assert(servo_angle==100 && task_info.held_kind==VISION_GREEN);
    for(i=0;i<30;i++)frame(0,0,0,0);
    assert(step==99 && servo_angle==100 && task_info.held_kind==VISION_GREEN);stopping_requested();
    reset_wire();tick+=151;faraway_vision_report();drain();
    assert(strstr(wire,"[99,0,0,3,1],")); /* actual MCU telemetry retains cargo when ROI=0 */
    tick+=60000;vision.connected=0;vision.uart_errors++;move();
    assert(step==99 && servo_angle==100 && task_info.held_kind==VISION_GREEN);stopping_requested();
    cameras(600,0,0,0);vision.lower.objects[0].kind=VISION_BLUE;move();
    assert(servo_angle==100 && task_info.held_kind==VISION_GREEN);stopping_requested();
    key_press(key1_Pin);assert(step==99 && servo_angle==100); /* no dropping on restart */
    key_press(key4_Pin);assert(step==0 && servo_angle==50 && task_info.held_kind==VISION_NONE);stopping_requested();
    simulation_age=0;

    /* ROI event cannot be lost when multiple packets arrive before move(). */
    key_press(key1_Pin);move();frame(600,0,0,0);assert(step==30);
    cameras(200,0,0,1);task_observe_lower();assert(pending_kind==VISION_GREEN);
    cameras(0,0,0,0);task_observe_lower();assert(task_info.inside_frames==0);
    move();assert(step==40 && servo_angle==100 && task_info.held_kind==VISION_GREEN);stopping_requested();
    tick+=401;vision.connected=0;vision.uart_errors++;move();assert(step==40 && servo_angle==100);
    tick+=100;move();assert(step==99 && servo_angle==100);stopping_requested();
    key_press(key4_Pin);assert(task_info.held_kind==VISION_NONE && pending_kind==VISION_NONE);

    /* A block already at the claw need not reappear in the upper camera. */
    key_press(key1_Pin);move();frame(0,0,0,1);
    assert(step==40 && servo_angle==100 && task_info.held_kind==VISION_GREEN);stopping_requested();
    key_press(key4_Pin);
    /* Stop also cancels an accepted event before the actuator command. */
    key_press(key1_Pin);move();cameras(200,0,0,1);task_observe_lower();assert(pending_kind==VISION_GREEN);
    key_press(key4_Pin);move();assert(step==0 && servo_angle==50 && pending_kind==VISION_NONE);stopping_requested();

    /* Nearest ordinary block, excluding closer nonordinary objects. */
    key_press(key1_Pin);move();cameras(900,30,0,0);vision.upper.count=3;
    vision.upper.objects[1]=vision.upper.objects[0];vision.upper.objects[1].distance_mm=600;
    vision.upper.objects[1].angle=-15;vision.upper.objects[2]=vision.upper.objects[0];
    vision.upper.objects[2].kind=VISION_BLACK;vision.upper.objects[2].distance_mm=100;
    move();assert(step==30 && target_angle==-15);
    /* Lower absence prevents driving but automatically recovers. */
    cameras(600,0,0,0);vision.lower.fresh=0;move();assert(step==30 && task_info.reason==TASK_LOWER_WAIT);stopping_requested();
    frame(600,0,0,0);assert(motor_drive_enabled);
    cameras(600,0,0,0);vision.upper.objects[0].angle_present=0;move();stopping_requested();
    cameras(600,0,0,0);vision.upper.objects[0].distance_mm=-1;move();stopping_requested();
    cameras(600,0,0,0);vision.upper.objects[0].angle=(float)(HUGE_VAL-HUGE_VAL);move();stopping_requested();
    frame(600,0,0,0);assert(motor_drive_enabled);
    vision.upper.age_ms=400;move();assert(motor_drive_enabled);
    vision.upper.age_ms=401;move();assert(task_info.reason==TASK_UPPER_WAIT);stopping_requested();
    frame(600,0,0,0);assert(motor_drive_enabled);
    vision.uart_errors++;move();assert(step==30 && task_info.reason==TASK_UART_ERROR);stopping_requested();
    frame(600,0,0,0);assert(motor_drive_enabled);
    /* Ambiguous/nonordinary/hazard ROI must not be recorded as held cargo. */
    cameras(600,0,0,1);vision.lower.count=2;vision.lower.objects[1]=vision.lower.objects[0];move();
    assert(step==30 && task_info.reason==TASK_AMBIGUOUS && pending_kind==VISION_NONE);stopping_requested();
    cameras(600,0,0,1);vision.lower.objects[0].kind=VISION_BLACK;move();
    assert(pending_kind==VISION_NONE && servo_angle==50);stopping_requested();
    cameras(600,0,0,1);vision.lower.objects[0].kind=VISION_BLUE;move();
    assert(task_info.reason==TASK_HAZARD && task_info.held_kind==VISION_NONE);stopping_requested();
    frame(600,0,0,0);assert(motor_drive_enabled);
    key_press(key4_Pin);key_press(key1_Pin|key4_Pin);assert(step==0);stopping_requested();
    Motor_SetDriveEnabled(1);MotorSetPWM(1000,-1000,1000,-1000);motor_process();
    for(i=0;i<4;i++)assert(htim1.compare[i]==7560);
    key_press(key4_Pin);stopping_requested();motor_process();stopping_requested();
    puts("PASS: 50cm/s forward with uncapped proportional steering with +/-1deg deadband, loss recovery, first ROI grip, queued event latch, cargo held across occlusion/faults, explicit KEY4 release");
}

static void imu_odometry_tests(void)
{
    float x, y, distance=wheel_diameter*3.1415926f*1.028666f;
    uint32_t encoder_tick;
    FieldPose p;
    task_emergency_stop();tick=1000000;imu_sample(30);position_encoder_init();
    /* Wheel-only yaw must no longer rotate the pose. */
    encoder1.count-=4081;encoder2.count+=4081;encoder3.count+=4081;encoder4.count-=4081;
    tick+=20;imu_sample(30);encoder1.last_tick=tick;position_encoder();
    assert(fabsf(angle_body)<.0001f && fabsf(x_body)<.0001f && fabsf(y_body)<.0001f);
    /* IMU-only turn changes heading, without inventing displacement. */
    tick+=20;imu_sample(120);encoder1.last_tick=tick;position_encoder();
    assert(fabsf(angle_body-3.1415926f/2)<.0001f);
    x=field_map.robot.x;y=field_map.robot.y;
    /* Encoder sample is older than the latest IMU sample (scheduler order). */
    tick+=20;imu_sample(120);encoder_tick=tick;
    encoder1.count+=40810;encoder2.count+=40810;encoder3.count+=40810;encoder4.count+=40810;
    tick+=20;imu_sample(165);encoder1.last_tick=encoder_tick;position_encoder();
    assert(fabsf(angle_body-3.1415926f/2)<.0001f);
    assert(fabsf(field_map.robot.x-x-distance)<.001f && fabsf(field_map.robot.y-y)<.001f);
    assert(field_map.robot.tick==encoder_tick && !motor_drive_enabled);
    /* Mixed translation/turn uses the mean of the two aligned headings. */
    position_encoder_init();tick+=20;imu_sample(255);encoder1.last_tick=tick;
    encoder1.count+=40810;encoder2.count+=40810;encoder3.count+=40810;encoder4.count+=40810;
    position_encoder();assert(fabsf(x_body-distance*.70710678f)<.001f && fabsf(y_body-distance*.70710678f)<.001f);
    tick+=401;encoder1.last_tick=tick;position_encoder();assert(!field_map.heading_valid);
    assert(!pose_at(tick-200,&p));assert(task_goto(150,150,50)==TASK_RUNNING);stopping_requested();
    tick+=20;imu_sample(255);encoder1.last_tick=tick;position_encoder();assert(field_map.heading_valid);
    assert(!pose_at(tick-200,&p)); /* Never bridge missing IMU interval. */
    fake_imu_ready=0;position_encoder_init();assert(!imu.ready && !field_map.heading_valid);
    assert(task_goto(150,150,50)==TASK_RUNNING);stopping_requested();
    task_emergency_stop();
    puts("PASS: IMU-only heading, encoder-only translation, delayed sample alignment, midpoint turn integration, startup wait, dropout clears pose history and coordinate goto stops; no motor auto-start");
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
    /* Manual wheel rotation updates the field pose while drive remains disabled. */
    imu_sample(25);position_encoder_init();
    encoder1.count=encoder2.count=encoder3.count=encoder4.count=40810;
    tick+=20;encoder1.last_tick=encoder2.last_tick=encoder3.last_tick=encoder4.last_tick=tick;
    position_encoder();motor_process();
    assert(step==0 && !motor_drive_enabled && enabled_calls==0);
    assert(fabsf(field_map.robot.x-FIELD_START_X_CM)<0.001f);
    assert(fabsf(field_map.robot.y-FIELD_START_Y_CM-wheel_diameter*3.1415926f*1.028666f)<0.001f);
    assert(field_map.robot.tick==tick);
    position_encoder_init();
    puts("PASS: manual encoder movement updates field pose without enabling motors or starting the task");
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
    field_map_reset(tick-100);field_map_record(tick,0,5,0);
    imu_sample(25);vision.upper.updated_ms=tick; field_map_process();faraway_vision_report(); drain(); puts(wire);
    assert(strstr(wire,"[1,500,3000,0]") && strstr(wire,"[1,500,-3000,0]"));
    assert(strstr(wire,"[1,500,null,0]"));
    reset_wire(); vision.upper.fresh=0; tick+=200; vision.upper.updated_ms=tick; field_map_process();faraway_vision_report(); drain();
    assert(strstr(wire,",0,0,[],"));
    /* Full-size telemetry remains bounded; stale/raw coordinates never gain valid. */
    reset_wire(); vision.upper.fresh=1;vision.upper.count=12;
    for(i=0;i<12;++i) {
        VisionObject *o=&vision.upper.objects[i]; memset(o,0,sizeof(*o));
        o->target_id=UINT32_MAX; o->kind=VISION_BLUE_MATERIAL_ZONE;
        o->distance_mm=65535; o->angle=-135; o->angle_present=1;
    }
    tick+=200; field_map_reset(tick-100);field_map_record(tick,0,0,0);vision.upper.updated_ms=tick; vision.lower=vision.upper;field_map_process(); faraway_vision_report(); drain();
    assert(wire_size < 1280 && strstr(wire,"[8,65535,-13500,0]") && g_faraway_tx_drop_count==0);
    assert(field_map.aligned && field_map.count==12 && field_map.points[11].present);
    max_wire=wire_size;
    for(i=0;i<40;i++){
        reset_wire();tick+=200;vision.upper.updated_ms=tick;vision.lower=vision.upper;field_map_process();faraway_vision_report();drain();
        assert(strstr(wire,"*"));
        assert(VSP_Crc16((const uint8_t *)wire,(uint16_t)(strchr(wire,'*')-wire))==(unsigned)strtoul(strchr(wire,'*')+1,NULL,16));
        { const char *item=wire;unsigned count=0;while((item=strstr(item,"[8,65535,-13500,0]"))!=NULL){++count;++item;}assert(count==12); }
    }
    reset_wire();tick+=200;vision.upper.updated_ms=tick;faraway_vision_report();
    faraway_vision_report();assert(g_faraway_tx_drop_count==1);drain();
    tick=UINT32_MAX-2;tx_next_ms=tick;reset_wire();faraway_vision_report();drain();
    {
        uint32_t report_at;
        reset_wire();tick+=200;report_at=tick;map_started=1;map_last=tick;faraway_report();drain();assert(wire_size>0);
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
    imu_odometry_tests();
    return 0;
}

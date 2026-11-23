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
#include "../../../project/Core/Src/hui_ku.c"

enum {
    WIRE_LIVE=1, WIRE_EVENT, WIRE_TRACK, WIRE_HISTORY, WIRE_INFO, WIRE_STATS,
    WIRE_STEER, WIRE_ACK, WIRE_CONTROL, WIRE_ERROR, WIRE_MAP_SYNC
};

typedef struct {
    uint8_t bytes[460];
    unsigned type, seq, length, encoded_size;
    uint32_t tick;
    const uint8_t *payload;
} WireFrame;

static inline uint16_t wire_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1]<<8));
}

static inline uint32_t wire_u32(const uint8_t *p)
{
    return (uint32_t)wire_u16(p) | ((uint32_t)wire_u16(p+2)<<16);
}

static inline float wire_f32(const uint8_t *p)
{
    uint32_t bits=wire_u32(p);
    float value;
    memcpy(&value,&bits,sizeof(value));
    return value;
}

/* Independent receiver for firmware tests. Frames are captured only after
 * all async UART chunks complete. CRC uses the production CCITT primitive. */
static inline int wire_next(const void *data, unsigned size, unsigned *offset, WireFrame *result)
{
    const uint8_t *input=data;
    unsigned begin=*offset, end=begin, i, used=0;
    if (begin==size) return 0;
    for (; end<size && input[end]; ++end) {}
    assert(end<size && end>begin);
    for (i=begin; i<end;) {
        unsigned code=input[i++], copy=code-1U;
        assert(code && i+copy<=end && used+copy<=sizeof(result->bytes));
        memcpy(result->bytes+used,input+i,copy);
        i+=copy;
        used+=copy;
        if (code<255 && i<end) {
            assert(used<sizeof(result->bytes));
            result->bytes[used++]=0;
        }
    }
    assert(used>=12 && result->bytes[0]==1);
    result->type=result->bytes[1];
    result->seq=wire_u16(result->bytes+2);
    result->length=wire_u16(result->bytes+4);
    result->tick=wire_u32(result->bytes+6);
    result->payload=result->bytes+10;
    result->encoded_size=end-begin+1U;
    assert(result->length<=448 && result->length+12U==used);
    assert(VSP_Crc16(result->bytes,(uint16_t)(used-2U))==wire_u16(result->bytes+used-2U));
    *offset=end+1U;
    return 1;
}

static inline int wire_find(const void *data, unsigned size, unsigned type, WireFrame *result)
{
    unsigned offset=0;
    while (wire_next(data,size,&offset,result))
        if (result->type==type) return 1;
    return 0;
}

static inline void wire_vector(const char *name, const void *data, unsigned size)
{
    const uint8_t *bytes=data;
    unsigned i;
    printf("WIRE %s ",name);
    for (i=0; i<size; ++i) printf("%02x",bytes[i]);
    putchar('\n');
}

TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart5 = { UART5 };
VisionResult vision;
TIM_HandleTypeDef htim9;
static uint16_t pressed_pin;
Encoder_t encoder1, encoder2, encoder3, encoder4;

static unsigned enabled_calls;
static uint16_t motor_en_pins;
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
{
    if (port != GPIOE) return;
    if (state == GPIO_PIN_SET) {
        ++enabled_calls;
        motor_en_pins |= pin;
    } else {
        motor_en_pins &= (uint16_t)~pin;
    }
}
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

static void expect_ack(uint32_t id, uint8_t status, uint8_t task)
{
    WireFrame ack;
    unsigned offset=0;
    while (wire_next(wire,wire_size,&offset,&ack)) {
        if (ack.type!=WIRE_ACK) continue;
        assert(ack.length==6);
        if (wire_u32(ack.payload)==id && ack.payload[4]==status && ack.payload[5]==task) return;
    }
    assert(!"expected command acknowledgement");
}

static unsigned capture_live(WireFrame *live, bool observations)
{
    unsigned offset=0, size=encode_live(tick,observations);
    assert(size && wire_next(packet,size,&offset,live) && offset==size);
    assert(live->type==WIRE_LIVE && live->tick==tick);
    return size;
}

/* Skip one upper observation, locating the independent lower-camera header. */
static const uint8_t *lower_header(const WireFrame *live)
{
    const uint8_t *upper=live->payload+25;
    const uint8_t *item=upper+11+((upper[9]&4)?12:0);
    unsigned i;
    assert((live->payload[6]&0x18)==0x18);
    for (i=0; i<upper[10]; ++i) {
        unsigned flags=item[1];
        item+=2+((flags&1)?2:0)+((flags&2)?2:0)+((flags&4)?8:0)+((flags&16)?14:0);
    }
    assert(item+11<=live->payload+live->length);
    return item;
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
    unsigned i;assert(!motor_drive_enabled && !motor_en_pins);
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
    frame(0,0,0,1);assert(step==40 && servo_angle==120 && body_motion.forward==0);stopped();
    tick+=500;move();assert(step==RETURN_BACK && task_info.held_kind==VISION_NONE);
    task_emergency_stop();Motor_SetDriveEnabled(1);body_move(50,0,90);motor_process();
    task_stop();stopped();assert(body_motion.forward==0 && body_motion.turn==0);
    Motor_SetDriveEnabled(1);body_move(50,0,90);motor_process();key_press(key4_Pin);stopped();
    puts("PASS: pursuit stopping behavior, loss/reacquisition without ramp, single-frame grip and pending verification, KEY4 immediate stop");
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
    task_emergency_stop();field_map_reset(tick);task_goto(285,FIELD_START_Y_CM+12,50);
    assert(body_motion.target_forward==50); /* no endpoint slowdown */
    task_goto(285,FIELD_START_Y_CM+2,50);assert(goto_arrived);stopped();
    assert(task_goto(285,180,0)==TASK_FAILED);stopped();
    assert(task_goto((float)(HUGE_VAL-HUGE_VAL),180,50)==TASK_FAILED);
    task_emergency_stop();field_map_reset(tick);task_goto(185,FIELD_START_Y_CM,50);
    assert(body_motion.target_forward==50 && body_motion.target_turn<0);
    field_map.robot.yaw=-3.1415926f/2;field_map.robot.tick+=20;task_goto(185,FIELD_START_Y_CM,50);
    assert(body_motion.target_forward==50 && fabsf(body_motion.target_turn-pid_steer.ki*pid_steer.error_int)<.001f);
    field_map.heading_valid=0;task_goto(185,FIELD_START_Y_CM,50);stopped();
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
    expect_ack(100,0,50);
    field_map_reset(tick);move();tick+=20;motor_process();assert(motor_drive_enabled);
    for(i=0;i<25;i++){tick+=20;move();motor_process();}
    command(text);assert(step==50); /* no duplicate reset */
    strcpy(body,"C1,101,K,4");snprintf(text,sizeof(text),"%s*%04X\n",body,VSP_Crc16((uint8_t*)body,(uint16_t)strlen(body)));
    command(text);stopped();assert(step==0 && !goto_target.valid);
    strcpy(body,"C1,100,G,2000,1800,500");snprintf(text,sizeof(text),"%s*%04X\n",body,VSP_Crc16((uint8_t*)body,(uint16_t)strlen(body)));
    command(text);assert(step==0);stopped();expect_ack(100,4,0);
    command("C1,102,G,2000,1800,500*0000\n");assert(step==0);stopped();
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
    task_info.held_kind=VISION_GREEN;servo_angle=120;
    assert(task_goto_start(200,180,50));move();assert(step==50 && servo_angle==120 && task_info.held_kind==VISION_GREEN);
    field_map.robot.x=200;field_map.robot.y=180;body_motion.forward=body_motion.right=body_motion.turn=0;
    move();assert(step==51);move();stopping_requested();
    task_emergency_stop();command_seen=0;reset_wire();
    puts("PASS: UART5 checksummed goto/KEY1-4, duplicate/stale motion rejection, old KEY4 still stops, physical KEY4 priority, unbound KEY3, cargo preserved and case50/51 arrival");
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

    /* One positive frame still closes the claw if odometry becomes unavailable. */
    field_map.heading_valid=0;
    frame(0,0,0,1);
    assert(step==40 && servo_angle==120 && task_info.held_kind==VISION_NONE);stopping_requested();
    for(i=0;i<30;i++)frame(0,0,0,0);
    assert(step==RETURN_BACK && servo_angle==120 && task_info.held_kind==VISION_NONE);stopping_requested();
    {
        WireFrame live;
        capture_live(&live,true);
        assert(live.payload[0]==60 && live.payload[1]==18 && live.payload[2]==0 && live.payload[3]==0);
    } /* Closed without a confirmed load, and no reverse votes yet. */
    tick+=60000;vision.connected=0;vision.uart_errors++;move();
    assert(step==RETURN_BACK && servo_angle==120 && task_info.held_kind==VISION_NONE);stopping_requested();
    key_press(key1_Pin);assert(step==RETURN_BACK && servo_angle==120);
    key_press(key4_Pin);assert(step==0 && servo_angle==50 && task_info.held_kind==VISION_NONE);stopping_requested();
    simulation_age=0;

    /* A complete window may arrive before move(). One positive outweighs two misses. */
    key_press(key1_Pin);move();frame(600,0,0,0);assert(step==30);
    cameras(200,0,0,1);task_observe_lower();
    cameras(0,0,0,0);task_observe_lower();
    cameras(200,0,0,0);task_observe_lower();
    assert(task_info.inside_frames==1 && task_info.outside_frames==2);
    move();assert(step==40 && servo_angle==120 && task_info.held_kind==VISION_NONE);stopping_requested();
    tick+=401;vision.connected=0;vision.uart_errors++;move();assert(step==40 && servo_angle==120);
    tick+=100;move();assert(step==RETURN_BACK && servo_angle==120);stopping_requested();
    key_press(key4_Pin);assert(task_info.held_kind==VISION_NONE && pending_kind==VISION_NONE);

    /* A block at the claw need not reappear in the upper camera. */
    key_press(key1_Pin);move();frame(0,0,0,1);
    assert(step==40 && servo_angle==120 && task_info.held_kind==VISION_NONE);stopping_requested();
    key_press(key4_Pin);
    /* Stop cancels votes collected before the actuator command. */
    key_press(key1_Pin);move();cameras(200,0,0,1);task_observe_lower();
    cameras(200,0,0,1);task_observe_lower();assert(task_info.inside_frames==2);
    key_press(key4_Pin);move();assert(step==0 && servo_angle==50 && task_info.inside_frames==0);stopping_requested();

    /* Nearest ordinary block, even with a closer blue block in the upper frame. */
    key_press(key1_Pin);move();cameras(900,30,0,0);vision.upper.count=3;
    vision.upper.objects[1]=vision.upper.objects[0];vision.upper.objects[1].distance_mm=600;
    vision.upper.objects[1].angle=-15;vision.upper.objects[2]=vision.upper.objects[0];
    vision.upper.objects[2].kind=VISION_BLUE;vision.upper.objects[2].distance_mm=100;
    move();assert(step==30 && target_angle==-15);
    cameras(600,0,0,0);
    vision.upper.count=2;vision.upper.objects[1]=vision.upper.objects[0];
    vision.upper.objects[1].kind=VISION_BLUE;
    vision.lower.objects[0].kind=VISION_BLUE;
    move();assert(step==30 && task_info.reason==TASK_OK && motor_drive_enabled);
    assert(body_motion.target_forward==50.0f);
    /* Unknown objects follow the existing stop rule. */
    vision.upper.objects[1].kind=VISION_UNKNOWN;
    move();assert(step==30 && task_info.reason==TASK_HAZARD);stopped();
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
    cameras(600,0,0,1);vision.lower.count=2;
    vision.lower.objects[1]=vision.lower.objects[0];vision.lower.objects[1].kind=VISION_BLUE;
    move();
    assert(step==30 && task_info.reason==TASK_HAZARD && task_info.held_kind==VISION_NONE);
    assert(pending_kind==VISION_NONE && servo_angle==50);stopped();
    frame(600,0,0,0);assert(motor_drive_enabled);
    key_press(key4_Pin);key_press(key1_Pin|key4_Pin);assert(step==0);stopping_requested();
    Motor_SetDriveEnabled(1);MotorSetPWM(1000,-1000,1000,-1000);motor_process();
    for(i=0;i<4;i++)assert(htim1.compare[i]==7560);
    key_press(key4_Pin);stopping_requested();motor_process();stopping_requested();
    puts("PASS: 50cm/s forward with uncapped proportional steering with +/-1deg deadband, loss recovery, ROI voting across queued frames, unknown grip state with invalid odometry, explicit KEY4 release");
}

static void safety_zone_tests(void)
{
    task_stop(); task_start(); move();
    cameras(200,15,1,1);
    vision.upper.objects[0].in_safety_zone = 1;
    vision.lower.objects[0].in_safety_zone = 1;
    move();
    assert(step==10 && task_info.reason==TASK_TARGET_LOST && servo_angle==50);
    stopped();

    cameras(200,15,1,0);
    vision.upper.count = 2;
    vision.upper.objects[1] = vision.upper.objects[0];
    vision.upper.objects[1].distance_mm = 600;
    vision.upper.objects[1].angle = -15;
    vision.upper.objects[0].in_safety_zone = 1;
    move();
    assert(step==30 && target_angle==-15 && target_distance_cm==60.0f);

    cameras(600,-15,1,0);
    vision.lower.objects[0].in_safety_zone = 1; /* Outside the claw, so driving may continue. */
    move(); assert(motor_drive_enabled && body_motion.target_forward==50.0f);
    cameras(600,-15,1,0);
    vision.upper.objects[0].in_safety_zone = 1;
    move();
    assert(step==30 && task_info.reason==TASK_TARGET_LOST);
    stopped();

    /* Two new lower frames arrive before the next motion cycle. */
    cameras(600,-15,1,1);
    task_observe_lower();
    cameras(600,-15,1,1);task_observe_lower();assert(task_info.inside_frames==2);
    cameras(600,-15,1,1);
    vision.lower.count = 2;
    vision.lower.objects[1] = vision.lower.objects[0];
    vision.lower.objects[0].in_safety_zone = 1;
    task_observe_lower(); assert(task_info.inside_frames==0);
    move();
    assert(step==30 && task_info.reason==TASK_AMBIGUOUS && servo_angle==50);
    stopped();

    cameras(600,-15,1,1);
    vision.lower.objects[0].kind = VISION_BLUE;
    vision.lower.objects[0].in_safety_zone = 1;
    move();
    assert(step==30 && task_info.reason==TASK_HAZARD && servo_angle==50);
    stopped();

    frame(600,-15,1,1);frame(600,-15,1,1);
    assert(step==40 && servo_angle==120 && task_info.held_kind==VISION_NONE);
    cameras(600,-15,1,1);
    vision.lower.objects[0].in_safety_zone = 1;
    move();
    assert(step==40 && servo_angle==120 && pending_kind==VISION_GREEN);
    task_stop();
    puts("PASS: protected blocks skipped in search and approach, lower ROI cancels pending grip, blue warning preserved, outside target closes at 120 degrees");
}

static void grip_verification_tests(void)
{
    task_stop();task_start();move();
    field_map.heading_valid=1;field_map.robot.y=100;field_map.robot.yaw=0;
    frame(400,0,1,0);
    assert(step==30 && servo_angle==50);
    frame(400,0,1,1);
    assert(step==40 && servo_angle==120 && task_info.held_kind==VISION_NONE);
    tick+=500;move();
    assert(step==RETURN_BACK && !return_reverse_started && task_info.inside_frames==0);
    move();
    assert(return_reverse_started && body_motion.target_forward<0);

    /* Receive a delayed frame captured before reverse began. */
    tick+=20;cameras(400,0,1,0);vision.lower.age_ms=21;move();
    assert(task_info.outside_frames==0 && step==RETURN_BACK);
    frame(400,0,1,0);assert(step==RETURN_BACK && task_info.outside_frames==1);
    move();move(); /* Repeated polls of one empty frame cannot trigger a retry. */
    assert(step==RETURN_BACK && task_info.outside_frames==1);
    frame(400,0,1,0);
    assert(step==10 && task_info.reason==TASK_GRIP_LOST && servo_angle==50);
    assert(!task_info.inside_frames && !task_info.outside_frames && !task_info.held_kind);
    stopped();

    frame(400,0,1,1);assert(step==40);
    tick+=500;move();move();
    frame(400,0,1,1);
    tick+=401;
    field_map.robot.y=90;move();
    assert(step==RETURN_BACK && task_info.held_kind==VISION_NONE && servo_angle==120 &&
           task_info.reason==TASK_LOWER_WAIT && !task_info.inside_frames && !task_info.outside_frames);
    stopped(); /* Reaching 10 cm does not confirm an expired positive frame. */
    frame(400,0,1,1);
    assert(step==RETURN_FACE && task_info.held_kind==VISION_GREEN);

    frame(400,0,1,0);assert(step==RETURN_FACE && servo_angle==120);
    cameras(400,0,1,1);vision.lower.objects[0].kind=VISION_BLACK;move();
    assert(step==RETURN_FACE && task_info.reason==TASK_AMBIGUOUS && task_info.held_kind==VISION_GREEN);
    stopped();
    frame(400,0,1,1);assert(motor_drive_enabled);
    frame(400,0,1,0);assert(step==RETURN_FACE && motor_drive_enabled);
    frame(400,0,1,0);
    assert(step==RETURN_FACE && motor_drive_enabled &&
           task_info.inside_frames==1 && task_info.outside_frames==2);
    frame(400,0,1,0);
    assert(step==10 && task_info.reason==TASK_GRIP_LOST && servo_angle==50 && !task_info.held_kind);
    stopped();task_stop();
    puts("PASS: one positive in the last three fresh ROI frames, two empty votes without a positive retry, frame deduplication, reject pre-reverse samples, 10cm verification and monitoring until release");
}

/* Ideal pose integration from applied wheel commands, before the rigid field transform. */
static TaskTarget replay_target(uint32_t id, TaskTargetState state, int eligible)
{
    TaskTarget target;
    assert(task_target_get(field_map.epoch,id,&target));
    assert(target.state==state && target.candidate==eligible);
    return target;
}

static void print_target_replay(TaskTarget target, const char *note)
{
    static const char *const states[]={"available","locked","carrying","delivered","review"};
    static const char *const reasons[]={
        "mapped","selected","replaced","grip_verified","grip_failed","interrupted",
        "possible_duplicate","rechecked","release_pending","release_unverified","delivery_verified"
    };
    printf("TARGET t=%lu epoch=%lu id=%lu state=%s visible=%u candidate=%u "
           "evidence_frame=%lu evidence_ms=%lu related=%lu x=%.1f y=%.1f reason=%s note=%s\n",
           (unsigned long)tick,(unsigned long)target.epoch,(unsigned long)target.track_id,
           states[target.state],target.visible,target.candidate,(unsigned long)target.evidence_frame,
           (unsigned long)target.evidence_tick,(unsigned long)target.related_id,target.x,target.y,
           reasons[target.reason],note);
}

static void replay_green(int x_mm, int y_mm, uint8_t protected)
{
    VisionObject *object=&vision.upper.objects[vision.upper.count++];
    memset(object,0,sizeof(*object));
    vision.upper.status=VISION_OBS_OK;
    object->kind=VISION_GREEN;
    object->x_mm=x_mm;
    object->y_mm=y_mm;
    object->distance_mm=(int32_t)lroundf(hypotf((float)x_mm,(float)y_mm));
    object->angle=atan2f((float)x_mm,(float)y_mm)*180.0f/RETURN_PI;
    object->angle_present=object->position_present=1;
    object->in_safety_zone=protected;
}

static void task_target_replay_tests(void)
{
    unsigned i;
    uint32_t epoch, evidence;
    TaskTarget a,b,alias;

    task_emergency_stop();
    tick=1000;simulation_frame=0;simulation_age=0;
    field_map_reset(tick);field_map_correct_pose(100,100,0);
    epoch=field_map.epoch;
    /* A and B grow through the real KF with distinct co-observed IDs. */
    for (i=0;i<12;++i) {
        tick+=20;field_map_record(tick,0,0,0);
        cameras(0,0,0,0);replay_green(0,240,0);replay_green(120,240,0);
        field_map_process();move();
    }
    a=replay_target(1,TASK_TARGET_AVAILABLE,1);
    b=replay_target(2,TASK_TARGET_AVAILABLE,1);
    assert(a.reason==TARGET_MAPPED && b.reason==TARGET_MAPPED && b.visible);
    print_target_replay(a,"A_confirmed_by_KF");
    evidence=b.evidence_frame;

    task_start();move();
    tick+=450;field_map_record(tick,0,0,0);
    cameras(0,0,0,0);replay_green(0,240,0);
    field_map_process();move();move();
    assert(step==30 && active_target==1);
    a=replay_target(1,TASK_TARGET_LOCKED,0);
    b=replay_target(2,TASK_TARGET_AVAILABLE,1);
    assert(a.reason==TARGET_SELECTED && !b.visible && b.evidence_frame==evidence);
    print_target_replay(a,"selected_from_fresh_visible_view");
    print_target_replay(b,"B_out_of_view_last_position_retained_for_recheck");

    /* At 12 cm the raw detection still feeds pursuit, while the map point is absent. */
    tick+=20;field_map_record(tick,0,12,0);
    cameras(0,0,0,0);replay_green(0,120,0);
    field_map_process();move();
    assert(!field_map.points[0].present && active_target==1 && step==30);
    tick+=20;cameras(0,0,0,1);move();
    assert(step==40 && servo_angle==120 && active_target==1);
    tick+=500;move();move();
    assert(step==RETURN_BACK && return_reverse_started);
    tick+=20;field_map_record(tick,0,1,0);
    cameras(0,0,0,1);move();
    assert(step==RETURN_FACE && task_info.held_kind==VISION_GREEN);
    a=replay_target(1,TASK_TARGET_CARRYING,0);
    assert(a.reason==TARGET_GRIP_VERIFIED && a.evidence_frame==vision.lower.frame_id);
    print_target_replay(a,"positive_lower_frame_after_10cm_reverse");

    /* Supply a pose at the dock, then execute the existing turn/contact/release cases. */
    field_map_correct_pose(return_zone->centre_x,return_zone->contact_y,return_zone->yaw);
    move();move();move();
    assert(step==RETURN_CONTACT);
    tick+=20;cameras(0,0,0,1);move();
    tick+=RETURN_CONTACT_MS;
    encoder1.last_tick=tick;cameras(0,0,0,1);move();
    assert(step==RETURN_RELEASE);
    tick+=20;cameras(0,0,0,1);
    vision.lower.objects[0].zone_kind=return_zone->kind;
    vision.lower.objects[0].inside_valid=vision.lower.objects[0].object_inside=1;
    move();
    a=replay_target(1,TASK_TARGET_REVIEW,0);
    assert(a.reason==TARGET_RELEASE_PENDING && servo_angle==50);
    print_target_replay(a,"open_command_after_own_zone_entry_waiting_for_empty_claw");
    move();
    tick+=20;cameras(0,0,0,-1);vision.lower.age_ms=21;move();
    assert(replay_target(1,TASK_TARGET_REVIEW,0).reason==TARGET_RELEASE_PENDING);
    tick+=20;cameras(0,0,0,-1);move();
    a=replay_target(1,TASK_TARGET_DELIVERED,0);
    assert(a.reason==TARGET_DELIVERY_VERIFIED && a.evidence_frame==vision.lower.frame_id);
    print_target_replay(a,"new_empty_lower_frame_after_confirmed_own_zone_entry");

    task_emergency_stop();task_start();move();
    field_map_correct_pose(100,100,0);
    tick+=20;field_map_record(tick,0,1,0);
    cameras(0,0,0,0);replay_green(0,240,0);
    field_map_process();move();
    assert(field_map.points[0].track_id==1 && step==10 && !active_target);
    a=replay_target(1,TASK_TARGET_DELIVERED,0);
    assert(a.visible);
    print_target_replay(a,"A_visible_again_excluded_from_selection");

    /* A's displaced observation forms ID 3 while distinct B stays co-visible.
     * ID 3 is geographically closer to B than A, so a circular veto would fail. */
    enter_step(0);
    for (i=0;i<FIELD_CONFIRM_FRAMES;++i) {
        tick+=20;field_map_record(tick,0,1,0);
        cameras(0,0,0,0);replay_green(80,240,0);replay_green(120,240,0);
        field_map_process();move();
    }
    assert(field_map.points[0].track_id==3 && field_map.points[1].track_id==2);
    alias=replay_target(3,TASK_TARGET_REVIEW,0);
    b=replay_target(2,TASK_TARGET_AVAILABLE,1);
    assert(alias.reason==TARGET_DUPLICATE && alias.related_id==1 &&
           alias.evidence_frame==vision.upper.frame_id && b.visible);
    print_target_replay(alias,"new_nearby_ID_without_co_observed_A");
    print_target_replay(b,"B_co_observed_with_ID_3_remains_eligible");
    task_start();move();move();
    assert(step==30 && active_target==2);
    b=replay_target(2,TASK_TARGET_LOCKED,0);
    print_target_replay(b,"picked_B_while_closer_duplicate_excluded");

    tick+=20;cameras(0,0,0,1);move();
    assert(step==40 && active_target==2);
    tick+=500;move();move();
    assert(step==RETURN_BACK && return_reverse_started);
    tick+=20;field_map_correct_pose(100,89,0);
    cameras(0,0,0,1);move();
    assert(step==RETURN_FACE && replay_target(2,TASK_TARGET_CARRYING,0).reason==TARGET_GRIP_VERIFIED);
    enter_step(RETURN_RELEASE);
    tick+=20;cameras(0,0,0,1);
    vision.lower.objects[0].zone_kind=return_zone->kind==VISION_RED_MATERIAL_ZONE ?
                                     VISION_BLUE_MATERIAL_ZONE : VISION_RED_MATERIAL_ZONE;
    vision.lower.objects[0].inside_valid=vision.lower.objects[0].object_inside=1;
    move();
    tick+=20;cameras(0,0,0,-1);move();
    b=replay_target(2,TASK_TARGET_REVIEW,0);
    assert(b.reason==TARGET_RELEASE_UNVERIFIED);
    print_target_replay(b,"wrong_zone_entry_plus_empty_claw_does_not_confirm_delivery");

    task_emergency_stop();
    field_map_correct_pose(100,100,0);
    tick+=20;field_map_record(tick,0,1,0);
    cameras(0,0,0,0);replay_green(0,240,0);replay_green(80,240,0);replay_green(120,240,0);
    field_map_process();move();
    alias=replay_target(3,TASK_TARGET_AVAILABLE,1);
    assert(alias.reason==TARGET_RECHECKED && !alias.related_id && alias.visible);
    assert(replay_target(1,TASK_TARGET_DELIVERED,0).visible);
    print_target_replay(alias,"distinct_A_and_ID_3_co_observed_review_cleared");

    field_map_reset(tick);field_map_correct_pose(100,100,0);
    assert(!task_target_get(epoch,1,&a));
    for (i=0;i<FIELD_CONFIRM_FRAMES;++i) {
        tick+=20;field_map_record(tick,0,0,0);
        cameras(0,0,0,0);replay_green(0,240,1);
        field_map_process();move();
    }
    a=replay_target(1,TASK_TARGET_AVAILABLE,0);
    assert(a.epoch!=epoch && a.in_safety_zone && a.reason==TARGET_MAPPED);
    print_target_replay(a,"new_epoch_old_delivery_expired_current_safety_evidence_excludes_pick");
    tick+=20;field_map_record(tick,0,0,0);
    cameras(0,0,0,0);replay_green(0,240,0);
    field_map_process();move();
    a=replay_target(1,TASK_TARGET_AVAILABLE,1);
    print_target_replay(a,"new_fresh_outside_zone_observation");
    puts("PASS: epoch-keyed target lifecycle, replayed perception/mission events, duplicate isolated from distinct neighbour");
}

static void integrate_return_pose(void)
{
    const float dt=0.02f;
    float yaw, sin_yaw, cos_yaw;
    motor_process();
    if (!motor_drive_enabled) return;
    yaw=angle_body+0.5f*dt*body_motion.turn*RETURN_PI/180.0f;
    sin_yaw=sinf(yaw);
    cos_yaw=cosf(yaw);
    x_body+=dt*(body_motion.right*cos_yaw+body_motion.forward*sin_yaw);
    y_body+=dt*(-body_motion.right*sin_yaw+body_motion.forward*cos_yaw);
    angle_body+=dt*body_motion.turn*RETURN_PI/180.0f;
    field_map_record(tick,x_body,y_body,angle_body);
}

static void return_zone_frame(float world_x, uint32_t observed)
{
    cameras(300,0,0,task_info.held_kind==VISION_GREEN);
    vision.upper.objects[0].kind=return_zone->kind;
    vision.upper.objects[0].position_present=1;
    field_map.aligned=1;field_map.frame=vision.upper.frame_id;field_map.count=1;
    field_map.points[0].present=1;field_map.points[0].x=world_x;
    field_map.observed.tick=observed;
}

static void return_material_tests(void)
{
    const float direction=return_zone->kind==VISION_RED_MATERIAL_ZONE ? 1.0f : -1.0f;
    const float start_x=return_zone->centre_x+40.0f;
    const float approach_y=return_zone->contact_y-65.0f*direction;
    float contact_x, release_x, release_y, release_yaw, travelled, measured_zone_x;
    unsigned i;
    uint8_t last=RETURN_BACK, saw_correction=0;

    task_emergency_stop();
    field_map_reset(tick);
    x_body=y_body=0;angle_body=RETURN_PI*0.5f;
    field_map_record(tick,x_body,y_body,angle_body);
    field_map_correct_pose(start_x,approach_y,angle_body);
    task_start();move();
    frame(0,0,0,1);frame(0,0,0,1);
    tick+=500;move();
    assert(step==RETURN_BACK && servo_angle==120 && task_info.held_kind==VISION_NONE);
    stopped();
    field_map.heading_valid=0;move();
    assert(task_info.reason==TASK_RETURN_POSE_WAIT && !return_reverse_started);
    stopped();
    field_map.heading_valid=1;
    for(i=0;i<1200 && step!=RETURN_CONTACT;++i) {
        tick+=20;
        if(step==RETURN_APPROACH) return_zone_frame(return_zone->centre_x+5.0f,tick);
        else cameras(0,0,0,1);
        move();
        if(step==last && motor_drive_enabled) {
            const float yaw=field_map.robot.yaw;
            const float vx=body_motion.target_right*cosf(yaw)+body_motion.target_forward*sinf(yaw);
            const float vy=-body_motion.target_right*sinf(yaw)+body_motion.target_forward*cosf(yaw);
            if(step==RETURN_BACK) assert(body_motion.target_forward<0 && fabsf(vy)<.01f);
            if(step==RETURN_FACE) assert(body_motion.target_forward==0 && body_motion.target_right==0);
            if(step==RETURN_STRAFE) assert(vx<0 && fabsf(vy)<.01f && body_motion.target_turn==0);
            if(step==RETURN_APPROACH) {
                assert(fabsf(vy*direction-30.0f)<.01f && body_motion.target_turn==0);
                if(!saw_correction) {
                    assert(vx>0);
                    saw_correction=1;
                    angle_body+=0.02f; /* IMU drift must not trigger rotation while translating. */
                }
            }
        }
        if(step!=last) {
            if(step==RETURN_FACE) {
                assert(task_info.held_kind==VISION_GREEN);
                assert(start_x-field_map.robot.x>=10 && start_x-field_map.robot.x<10.6f);
                assert(fabsf(field_map.robot.y-approach_y)<.01f);
            }
            if(step==RETURN_STRAFE) assert(fabsf(return_heading_error(return_zone->yaw))<=2.0f);
            if(step==RETURN_APPROACH) assert(fabsf(field_map.robot.x-return_zone->centre_x)<=2);
            last=step;
        }
        integrate_return_pose();
    }
    assert(saw_correction && step==RETURN_CONTACT && servo_angle==120);
    travelled=(return_zone->contact_y-field_map.robot.y)*direction;
    stopped();
    assert(travelled<=13.0f && travelled>12.3f);
    contact_x=field_map.robot.x;
    tick+=20;move();motor_process();
    assert(return_push_active && !motor_pwm_mode);
    assert(motor_en_pins==(motor3_En_Pin | motor4_En_Pin));
    assert(pid_motor1.target==0 && pid_motor2.target==0);
    assert(fabsf(pid_motor3.target-300.0f)<.001f && fabsf(pid_motor4.target-300.0f)<.001f);
    assert(htim1.compare[TIM_CHANNEL_2]>0 && htim1.compare[TIM_CHANNEL_1]>0);
    /* Rolling front wheels stay disabled; only rear encoder feedback drives them forward. */
    encoder1.speed=100;encoder2.speed=100;encoder3.speed=300;encoder4.speed=300;
    motor_process();
    assert(motor_en_pins==(motor3_En_Pin | motor4_En_Pin));
    assert(fabsf(pid_motor3.error_now)<.001f && fabsf(pid_motor4.error_now)<.001f);
    assert(pid_motor3.output>0 && pid_motor3.output<200);
    assert(pid_motor4.output>0 && pid_motor4.output<200);
    encoder1.speed=encoder2.speed=encoder3.speed=encoder4.speed=0;
    tick+=350;cameras(0,0,0,1);task_observe_lower();
    tick+=350;cameras(0,0,0,1);task_observe_lower();
    tick+=299;cameras(0,0,0,1);move();assert(step==RETURN_CONTACT && servo_angle==120);
    angle_body+=0.03f; /* Last raw IMU reading during contact. */
    ++tick;encoder1.last_tick=tick;move();stopped();
    assert(step==RETURN_RELEASE && servo_angle==120 && !return_push_active);
    assert(fabsf(field_map.robot.x-contact_x)<.001f && field_map.robot.y==return_zone->contact_y);
    assert(fabsf(return_heading_error(return_zone->yaw))<.001f && !field_map.aligned);
    ++tick;field_map_record(tick,x_body,y_body,angle_body);
    assert(fabsf(field_map.robot.y-return_zone->contact_y)<.001f);
    cameras(0,0,0,0);move();stopped();
    assert(servo_angle==50 && task_info.held_kind==VISION_NONE);
    tick+=499;move();assert(step==RETURN_RELEASE);stopped();
    ++tick;move();assert(step==RETURN_RETREAT);stopped();

    release_x=field_map.robot.x;release_y=field_map.robot.y;release_yaw=field_map.robot.yaw;
    for(i=0;i<150 && step==RETURN_RETREAT;++i) {
        tick+=20;move();integrate_return_pose();
    }
    travelled=(release_x-field_map.robot.x)*sinf(release_yaw)+
              (release_y-field_map.robot.y)*cosf(release_yaw);
    assert(step==RETURN_CALIBRATE_X && travelled>=50 && travelled<50.6f);
    stopped();

    measured_zone_x=return_zone->centre_x+6.0f;
    release_x=field_map.robot.x;release_y=field_map.robot.y;
    tick=return_sample_after;
    return_zone_frame(measured_zone_x,return_sample_after-1U);move();
    assert(!return_sample_count && task_info.reason==TASK_RETURN_X_ALIGN);
    return_zone_frame(measured_zone_x,tick);move();move();
    assert(return_sample_count==1 && step==RETURN_CALIBRATE_X);
    for(i=1;i<=3;++i) {
        tick+=20;return_zone_frame(measured_zone_x+7.0f*i,tick);move();
    }
    assert(return_sample_count==4);
    tick+=20;return_zone_frame(measured_zone_x,tick);vision.upper.age_ms=401;move();
    assert(return_sample_count==4 && task_info.reason==TASK_UPPER_WAIT);
    tick+=20;cameras(0,0,0,0);move();
    assert(return_sample_count==4 && task_info.reason==TASK_RETURN_X_NO_ZONE);
    for(i=1;i<=9;++i) {
        tick+=20;field_map_record(tick,x_body,y_body,angle_body);
        return_zone_frame(measured_zone_x+4.0f*i/9.0f,tick);move();
        if(i<9) {
            assert(step==RETURN_CALIBRATE_X && return_sample_count==i+4U);
            assert(task_info.reason==(i<6?TASK_RETURN_X_WAIT:TASK_RETURN_X_SPREAD));
        }
    }
    assert(step==RETURN_CENTER_FACE && fabsf(field_map.robot.x-(release_x-8.0f))<.01f);
    assert(fabsf(field_map.robot.y-release_y)<.001f);
    tick+=20;field_map_record(tick,x_body,y_body,angle_body);
    assert(fabsf(field_map.robot.x-(release_x-8.0f))<.01f);
    stopped();

    for(i=0;i<400 && step==RETURN_CENTER_FACE;++i) {
        tick+=20;cameras(600,10,0,0);move();
        if(step==RETURN_CENTER_FACE) {
            assert(body_motion.target_forward==0 && body_motion.target_right==0);
            assert(body_motion.target_turn!=0);
        }
        integrate_return_pose();
    }
    assert(step==RETURN_CENTER_DRIVE && fabsf(field_map.robot.x-(release_x-8.0f))<.01f);
    assert(fabsf(field_map.robot.y-release_y)<.001f);
    assert(fabsf(return_heading_error(atan2f(150.0f-field_map.robot.x,
                                           150.0f-field_map.robot.y)))<=5.0f);
    for(i=0;i<400 && step==RETURN_CENTER_DRIVE;++i) {
        tick+=20;cameras(600,10,0,0);
        vision.upper.objects[0].kind=VISION_BLUE;
        vision.lower.objects[0].kind=VISION_BLUE;
        move();
        if(step==RETURN_CENTER_DRIVE) {
            assert(body_motion.target_forward>0 && body_motion.target_turn==0);
            assert(fabsf(hypotf(body_motion.target_forward,body_motion.target_right)-30.0f)<.01f);
        }
        integrate_return_pose();
    }
    assert(step==RETURN_SCAN);
    assert(hypotf(field_map.robot.x-150.0f,field_map.robot.y-150.0f)<=2.0f);
    stopped();

    tick+=20;cameras(400,5,0,0);vision.upper.age_ms=401;move();
    assert(step==RETURN_SCAN && task_info.reason==TASK_UPPER_WAIT);
    stopped();
    tick+=20;cameras(400,5,0,1);
    vision.upper.objects[0].kind=VISION_BLUE;
    vision.lower.objects[0].kind=VISION_BLUE;
    vision.lower.age_ms=400;
    move();assert(step==RETURN_SCAN && task_info.reason==TASK_HAZARD);stopped();
    ++tick;move();assert(step==RETURN_SCAN && motor_drive_enabled); /* lower ROI observation expired */
    for(i=0;i<500 && step==RETURN_SCAN;++i) {
        tick+=20;cameras(400,5,0,0);
        vision.upper.objects[0].kind=VISION_BLUE;
        vision.lower.objects[0].kind=VISION_BLUE;
        move();
        if(step==RETURN_SCAN) {
            assert(field_map.robot.yaw-return_scan_start_yaw<2.0f*RETURN_PI);
            assert(body_motion.target_forward==0 && body_motion.target_right==0);
            assert(body_motion.target_turn==45.0f);
        }
        integrate_return_pose();
    }
    assert(step==RETURN_DONE);
    travelled=field_map.robot.yaw-return_scan_start_yaw;
    assert(travelled>=2.0f*RETURN_PI && travelled<2.0f*RETURN_PI+.02f);
    stopped();
    frame(400,5,0,0);assert(step==RETURN_DONE);stopped();
    task_start();assert(step==1);

    /* Re-enter scanning with a new yaw baseline, detect, then hand off to normal pursuit. */
    enter_step(RETURN_CENTER_DRIVE);
    frame(0,0,0,0);assert(step==RETURN_SCAN && return_scan_start_yaw==field_map.robot.yaw);
    frame(0,0,0,0);integrate_return_pose();
    assert(step==RETURN_SCAN && motor_drive_enabled);
    key_press(key4_Pin);assert(step==0);stopped();
    enter_step(RETURN_CENTER_DRIVE);
    frame(0,0,0,0);assert(step==RETURN_SCAN && return_scan_start_yaw==field_map.robot.yaw);
    frame(0,0,0,0);integrate_return_pose();
    cameras(250,5,1,0);
    vision.upper.objects[0].in_safety_zone = 1;
    move();assert(step==RETURN_SCAN && body_motion.target_turn==45.0f);
    frame(400,5,0,0);assert(step==30);stopped();
    frame(400,5,0,0);assert(body_motion.target_forward==50.0f);
    frame(400,5,0,1);frame(400,5,0,1);assert(step==40 && servo_angle==120);
    tick+=500;move();assert(step==RETURN_BACK && task_info.held_kind==VISION_NONE);

    task_emergency_stop();stopped();
    enter_step(RETURN_CONTACT);servo_angle=120;task_info.held_kind=VISION_GREEN;
    cameras(0,0,0,1);task_observe_lower();
    frame(0,0,0,1);assert(return_push_active && motor_drive_enabled);
    field_map.heading_valid=0;tick+=20;move();stopped();
    assert(step==RETURN_CONTACT_ABORT && servo_angle==120 && !return_push_active);
    task_emergency_stop();stopped();
    field_map.heading_valid=1;
    enter_step(RETURN_CONTACT);servo_angle=120;task_info.held_kind=VISION_GREEN;
    cameras(400,0,0,1);task_observe_lower();
    cameras(400,0,0,1);
    vision.upper.objects[0].kind=VISION_BLUE;
    vision.lower.count=2;vision.lower.objects[1].kind=VISION_BLUE;
    move();motor_process();
    assert(return_push_active && motor_en_pins==(motor3_En_Pin | motor4_En_Pin));
    tick+=20;vision.lower.frame_id=++simulation_frame;vision.lower.updated_ms=tick;
    vision.lower.objects[1].in_roi=1;move();
    assert(step==RETURN_CONTACT_ABORT && task_info.reason==TASK_HAZARD && !return_push_active);
    assert(servo_angle==120 && task_info.held_kind==VISION_GREEN);stopped();

    enter_step(RETURN_CONTACT);
    frame(0,0,0,1);
    assert(return_push_active && motor_en_pins==(motor3_En_Pin | motor4_En_Pin));
    key_press(key4_Pin);assert(step==0 && !return_push_active);stopped();
    move();motor_process();stopped();

    /* Missing zone: stay stationary through the sampling budget, then use existing X. */
    field_map_correct_pose(return_zone->centre_x+7.0f,release_y,return_zone->yaw);
    release_x=field_map.robot.x;
    release_y=field_map.robot.y;
    release_yaw=field_map.robot.yaw;
    enter_step(RETURN_CALIBRATE_X);
    assert(!return_sample_count && !return_x_skipped);
    tick=return_sample_after;
    return_zone_frame(measured_zone_x,tick);
    vision.upper.objects[0].position_present=0;
    move();
    assert(step==RETURN_CALIBRATE_X && task_info.reason==TASK_RETURN_X_NO_ZONE);
    tick+=2999;
    cameras(0,0,0,0);move();
    assert(step==RETURN_CALIBRATE_X && !return_x_skipped);stopped();
    ++tick;move();
    assert(step==RETURN_CENTER_FACE && return_x_skipped && task_info.reason==TASK_RETURN_X_SKIPPED);
    assert(field_map.robot.x==release_x && field_map.robot.y==release_y && field_map.robot.yaw==release_yaw);
    stopped();
    ++tick;move();
    assert(step==RETURN_CENTER_FACE && motor_drive_enabled && task_info.reason==TASK_RETURN_X_SKIPPED);
    task_emergency_stop();stopped();
    puts("PASS: X calibration accepts ten distinct frames spanning exactly 4cm across losses and outliers; 3s timeout preserves pose and resumes centre return");
    printf("PASS: %s return, 13cm contact trigger, rear-only 30 percent speed PID for 1s, release and 50cm retreat, ten-frame X correction, centre drive, one-turn stop, scan-to-pursuit and repeat return, upper/outside blue ignored, lower-ROI blue stops scan and aborts dock, KEY4\n",
           direction>0?"red":"blue");
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
    assert(!field_map_pose_at(tick-200,&p));assert(task_goto(150,150,50)==TASK_RUNNING);stopping_requested();
    tick+=20;imu_sample(255);encoder1.last_tick=tick;position_encoder();assert(field_map.heading_valid);
    assert(!field_map_pose_at(tick-200,&p)); /* Never bridge missing IMU interval. */
    /* Rear wheels can slip against the wall without changing the integrated pose. */
    x=x_body;y=y_body;encoder_tick=field_map.robot.tick;
    return_push_active=1;
    encoder3.count+=40810;encoder4.count+=40810;
    tick+=20;imu_sample(255);encoder1.last_tick=tick;position_encoder();
    assert(x_body==x && y_body==y && field_map.robot.tick==encoder_tick);
    assert(last_dis3==motordistance3 && last_dis4==motordistance4);
    return_push_active=0;
    tick+=20;imu_sample(255);encoder1.last_tick=tick;position_encoder();
    assert(x_body==x && y_body==y && field_map.robot.tick==tick);
    puts("PASS: wall-push encoder slip is discarded, not replayed on the next pose update");
    fake_imu_ready=0;position_encoder_init();assert(!imu.ready && !field_map.heading_valid);
    assert(task_goto(150,150,50)==TASK_RUNNING);stopping_requested();
    task_emergency_stop();
    puts("PASS: IMU-only heading, encoder-only translation, delayed sample alignment, midpoint turn integration, startup wait, dropout clears pose history and coordinate goto stops; no motor auto-start");
}

int main(void)
{
    unsigned i, max_wire;
    WireFrame live;
    const uint8_t *upper, *lower, *item;
    Servo_Init(); assert(Servo_GetAngle()==50 && htim9.compare[0]==1056);
    Servo_SetAngle(120,0); assert(Servo_GetAngle()==120 && htim9.compare[0]==1833);
    pressed_pin=key3_Pin;Key_Update();Key_Update();assert(Servo_GetAngle()==120);
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
    assert(wire_find(wire,wire_size,WIRE_ERROR,&live) && live.length==1 && live.payload[0]==1);
    assert(!wire_find(wire,wire_size,WIRE_ACK,&live));
    assert(enabled_calls == 0);
    reset_wire();
    vision.connected=1; vision.acknowledged=vision.vision_ready=0; vision.danger_flag=1;
    strcpy(vision.fault_code,"rv3_observation");
    vision.upper.available=vision.upper.fresh=1; vision.upper.frame_id=42; vision.upper.age_ms=40;
    vision.upper.count=3;
    for(i=0;i<3;++i) {
        VisionObject *o=&vision.upper.objects[i];
        o->target_id=1+2*i; o->kind=VISION_GREEN; o->distance_mm=500;
        o->angle=i==0?30.0f:i==1?-30.0f:0.0f; o->angle_present=i<2;
    }
    vision.upper.objects[0].position_present=1;
    vision.upper.objects[0].x_mm=251;vision.upper.objects[0].y_mm=432;
    vision.lower=vision.upper;vision.lower.updated_ms=tick;vision.lower.count=2;vision.lower.objects[0].in_roi=1;
    vision.lower.objects[0].zone_kind=VISION_RED_MATERIAL_ZONE;
    vision.lower.objects[0].inside_valid=vision.lower.objects[0].object_inside=1;
    servo_angle=120;
    field_map_reset(tick-100);field_map_record(tick,0,5,0);
    imu_sample(25);vision.upper.updated_ms=tick;field_map_process();
    max_wire=capture_live(&live,true);
    wire_vector("live",packet,max_wire);
    assert(wire_u32(live.payload+11)==745 && wire_u16(live.payload+23)==2500);
    upper=live.payload+25;
    assert(wire_u32(upper)==42 && wire_u32(upper+4)==40 && upper[9]==7 && upper[10]==3);
    item=upper+23;
    assert(item[0]==1 && item[1]==0x17 && wire_u16(item+2)==500 && wire_u16(item+4)==3000);
    assert(wire_u32(item+6)==251 && wire_u32(item+10)==432 && wire_u32(item+22)==0);
    item+=28;
    assert(item[1]==0x13 && (int16_t)wire_u16(item+4)==-3000);
    item+=20;
    assert(item[1]==1 && wire_u16(item+2)==500);
    lower=lower_header(&live);
    assert(lower[10]==2 && lower[11]==1 && lower[12]==7 && lower[13]==6);
    vision.upper.fresh=0;tick+=200;vision.upper.updated_ms=tick;field_map_process();
    capture_live(&live,true);
    assert(!(live.payload[25+9]&5) && live.payload[25+10]==0);
    /* Full-size telemetry remains bounded; stale/raw coordinates never gain valid. */
    reset_wire(); vision.upper.fresh=1;vision.upper.count=12;
    for(i=0;i<12;++i) {
        VisionObject *o=&vision.upper.objects[i]; memset(o,0,sizeof(*o));
        o->target_id=UINT32_MAX; o->kind=VISION_BLUE_MATERIAL_ZONE;
        o->distance_mm=65535; o->angle=i==0?0:-135; o->angle_present=1;
        o->position_present=1; o->x_mm=-65535; o->y_mm=65535;
    }
    tick+=200; field_map_reset(tick-100);field_map_record(tick,0,0,0);
    vision.upper.updated_ms=tick; vision.lower=vision.upper;
    for(i=0;i<12;++i) {
        VisionObject *o=&vision.lower.objects[i];
        o->kind=VISION_BLACK; o->in_roi=1; o->zone_kind=VISION_BLUE_MATERIAL_ZONE;
        o->inside_valid=o->object_inside=1;
    }
    field_map_process();max_wire=capture_live(&live,true);
    assert(live.length==431 && max_wire>=445 && max_wire<=447);
    upper=live.payload+25;
    assert(upper[10]==12);
    item=upper+23;
    assert(item[0]==8 && item[1]==0x17 && wire_u16(item+2)==65535 && wire_u16(item+4)==0);
    assert((int32_t)wire_u32(item+6)==-65535 && wire_u32(item+10)==65535);
    lower=lower_header(&live);
    assert(lower[10]==12 && lower[11]==2 && lower[12]==7 && lower[13]==8);
    wire_vector("full",packet,max_wire);
    assert(field_map.aligned && field_map.count==12 && field_map.points[11].present);
    reset_wire();
    vision.upper.age_ms=218;vision.upper.updated_ms=tick-182;vision.upper.fresh=1;
    capture_live(&live,true);upper=live.payload+25;
    assert(wire_u32(upper+4)==400 && upper[8]==VISION_OBS_OK && (upper[9]&1));
    vision.upper.updated_ms=tick-183;
    capture_live(&live,true);upper=live.payload+25;
    assert(wire_u32(upper+4)==401 && upper[8]==VISION_OBS_STALE && !(upper[9]&1) && !upper[10]);
    vision.upper.age_ms=UINT32_MAX;
    capture_live(&live,true);upper=live.payload+25;
    assert(wire_u32(upper+4)==UINT32_MAX && !(upper[9]&1));
    vision.lower.frame_id=123;vision.lower.age_ms=218;vision.lower.updated_ms=tick-182;
    capture_live(&live,true);lower=lower_header(&live);
    assert(wire_u32(lower)==123 && wire_u32(lower+4)==400 && lower[8]==VISION_OBS_OK && (lower[9]&1));
    assert(lower[12]==1); /* ROI survives, the older warehouse decision does not. */
    vision.lower.updated_ms=tick-183;
    capture_live(&live,true);lower=lower_header(&live);
    assert(wire_u32(lower+4)==401 && lower[8]==VISION_OBS_STALE && !(lower[9]&1) && !lower[10]);
    imu_sample(360);tick+=145279;
    assert(capture_live(&live,false)==39 && live.length==25 && !(live.payload[6]&0x18));
    assert(wire_u32(live.payload+19)==145279 && wire_u16(live.payload+23)==36000);
    wire_vector("heartbeat",packet,39);
    fake_imu_ready=0;imu_init();
    printf("PASS: motor and servo interlocks, rejected legacy movement, binary CRC, state-only 39 B, 12+12 objects (%u B), 0 degree, 65535 mm, 360 degree IMU and age 145279 ms, independent 400/401 ms freshness and unknown age\n",max_wire);
    mission_tests();
    safety_zone_tests();
    grip_verification_tests();
    return_material_tests();
    imu_odometry_tests();
    task_target_replay_tests();
    return 0;
}

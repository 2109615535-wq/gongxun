#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../../project/Core/Src/vision_simple.c"
#include "../../../project/Core/Src/vision.c"
UART_HandleTypeDef huart6 = { USART6 };
static uint32_t tick;
static unsigned lower_notifications;
static uint32_t notified_frame, previous_notified_frame;
void task_observe_lower(void) {
    if(vision.lower.frame_id!=notified_frame) {
        previous_notified_frame=notified_frame;notified_frame=vision.lower.frame_id;++lower_notifications;
    }
}
static uint8_t *hal_rx;
uint32_t HAL_GetTick(void) { return tick; }
int HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ assert(h == &huart6 && n == 1); hal_rx = p; return HAL_OK; }
int HAL_UART_AbortReceive(UART_HandleTypeDef *h) { assert(h == &huart6); return HAL_OK; }
int HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ (void)h; (void)p; (void)n; assert(0); return HAL_ERROR; }
static void feed_wire(const char *s, int process_now)
{
    while (*s) { *hal_rx = (uint8_t)*s++; vision_uart_rx_complete(&huart6); }
    if (process_now) vision_process();
}
static void send_body(const char *body, int process_now)
{
    char frame[VSP_MAX_LINE + 3u];
    snprintf(frame,sizeof(frame),"%s*%04X\r\n",body,VSP_Crc16((const uint8_t *)body,(uint16_t)strlen(body)));
    feed_wire(frame,process_now);
}
static void feed_body(const char *body) { send_body(body,1); }
int main(int argc, char **argv)
{
    unsigned i;
    assert(vision_init());
    assert(!vision_set_task(NULL));
    tick=100;
    feed_wire("RV2,upper,42,86,ok,1,green,450,1,0,30.00*CBDC\n",1);
    assert(vision.connected && vision.rx_packets==1 && vision.upper.fresh);
    assert(!vision.acknowledged && !vision.vision_ready && !vision.tx_packets);
    assert(vision.upper.count==1 && vision.upper.objects[0].kind==VISION_GREEN);
    assert(vision.upper.objects[0].angle==30 && vision.upper.objects[0].distance_valid);
    tick=164;vision_process();assert(!vision.upper.objects[0].distance_valid);
    tick=200;feed_wire("RV2,upper,42,86,ok,1,green,450,1,0,30.00*CBDC\n",1);
    assert(vision.ignored_packets==1);
    tick=414;vision_process();assert(vision.upper.fresh && vision.upper.age_ms==400);
    tick=415;vision_process();assert(!vision.upper.fresh && !vision.upper.count && vision.upper.age_ms==401);
    tick=432;vision_process();assert(vision.upper.age_ms==418);
    feed_body("RV2,lower,1,0,ok,1,black,-1,0,1,NA");
    assert(vision.lower.fresh && !vision.upper.fresh);
    assert(vision.lower.objects[0].kind==VISION_BLACK && !vision.lower.objects[0].angle_present);
    feed_body("RV2,upper,43,0,ok,3,orange,300,0,0,-30.00,blue,400,0,0,0.00,green,500,0,0,NA");
    assert(vision.upper.count==3 && vision.upper.objects[0].kind==VISION_ORANGE);
    assert(vision.upper.objects[1].kind==VISION_BLUE && vision.upper.objects[1].angle_present);
    assert(!vision.upper.objects[2].angle_present && !vision.upper.objects[0].distance_valid);
    feed_wire("RV2,upper,44,0,empty,0*0000\n",1);assert(vision.bad_packets==1);
    feed_body("RV2,upper,44,0,empty,0");assert(vision.upper.fresh && !vision.upper.count);
    feed_body("RV2,upper,45,-1,fault,0");assert(!vision.upper.fresh);
    for(i=0;i<600;i++){*hal_rx='x';vision_uart_rx_complete(&huart6);}
    feed_wire("\n",1);assert(vision.bad_packets==2);
    feed_body("RV2,upper,46,0,ok,1,green,500,1,0,0.00");assert(vision.upper.fresh);
    vision_uart_error(&huart6);vision_process();assert(!vision.upper.fresh && vision.uart_errors==1);
    feed_wire("discard\n",1);feed_body("RV2,upper,47,0,empty,0");assert(vision.upper.fresh);
    feed_body("RV2,upper,48,0,ok,1,green,500,1,0,0.00");
    for(i=0;i<RX_SIZE+10;i++){*hal_rx='x';vision_uart_rx_complete(&huart6);}
    vision_process();assert(!vision.upper.fresh && vision.uart_errors==2);
    feed_wire("\n",1);
    if(argc>1){
        FILE *f=fopen(argv[1],"rb");char buffer[700];assert(f);
        tick+=10;
        while(fgets(buffer,sizeof(buffer),f)) feed_wire(buffer,1);
        fclose(f);
        assert(vision.upper.count==12 && vision.upper.objects[0].kind==VISION_GREEN);
        assert(vision.upper.objects[1].kind==VISION_BLACK);
        assert(vision.upper.objects[2].kind==VISION_ORANGE);
        assert(vision.upper.objects[3].kind==VISION_BLUE);
        assert(vision.upper.objects[11].angle_present && !vision.tx_packets);
    }
    if (argc > 2) {
        char buffer[VSP_MAX_LINE + 2u];
        FILE *f = fopen(argv[2], "rb");
        assert(f && fgets(buffer, sizeof(buffer), f));
        fclose(f);
        assert(strlen(buffer) > 512u);
        feed_wire(buffer, 1);
        assert(vision.upper.count == 12 && vision.upper.frame_id == 1001);
        for (i = 0; i < vision.upper.count; ++i) {
            assert(vision.upper.objects[i].kind == (i % 2 ? VISION_BLUE_CASUALTY_ZONE : VISION_GREEN));
            assert(vision.upper.objects[i].in_safety_zone == (i % 2 == 0));
        }
    }
    /* UART byte arrival + main-loop queue wait are counted once. */
    tick=10000;
    {
        const char *body="RV2,upper,9000,100,ok,1,green,450,1,0,30.00";
        char frame[200];
        snprintf(frame,sizeof(frame),"%s*%04X",body,VSP_Crc16((const uint8_t *)body,(uint16_t)strlen(body)));
        feed_wire(frame,0);tick+=20;feed_wire("\n",0);tick+=10;vision_process();
        assert(vision.upper.age_ms==130 && vision.upper.updated_ms==tick);
        vision_process();assert(vision.upper.age_ms==130);
        tick+=10;vision_process();assert(vision.upper.age_ms==140);
    }
    feed_body("RV2,upper,9001,218,ok,1,green,450,0,0,30.00");
    tick+=100;vision_process();assert(vision.upper.fresh && vision.upper.age_ms==318);
    tick+=82;vision_process();assert(vision.upper.fresh && vision.upper.age_ms==400);
    tick++;vision_process();assert(!vision.upper.fresh && vision.upper.age_ms==401);
    feed_body("RV2,upper,9002,400,empty,0");assert(vision.upper.fresh && vision.upper.age_ms==400);
    feed_body("RV2,upper,9003,320,stale,0");assert(!vision.upper.fresh && vision.upper.age_ms==320);
    tick+=10;vision_process();assert(vision.upper.age_ms==330);
    feed_body("RV2,upper,9004,-1,fault,0");tick+=10;vision_process();assert(vision.upper.age_ms==UINT32_MAX);
    tick=UINT32_MAX-10;feed_body("RV2,upper,9005,100,empty,0");
    tick=9;vision_process();assert(vision.upper.fresh && vision.upper.age_ms==120);
    receiver.cameras[VSP_UPPER].observation.age_ms=INT32_MAX-2;
    vision_process();assert(vision.upper.age_ms==INT32_MAX && !vision.upper.fresh);
    tick=20000;
    send_body("RV2,lower,100,0,ok,1,green,-1,0,1,NA",0);
    send_body("RV2,lower,101,0,empty,0",0);
    vision_process();assert(notified_frame==101 && previous_notified_frame==100 && lower_notifications>=2);
    puts("PASS: every queued lower frame notifies task input; receiver never controls servo");
    tick+=1001;vision_process();assert(!vision.connected && !vision.upper.count);
    puts("PASS: actual RV2 parser/UART6, no task TX or fake ACK, kind mapping, NA/estimates, age/duplicate isolation, CRC, overflow and UART recovery");
    {
        uint32_t bad = vision.bad_packets;
        feed_body("RV4,lower,102,0,ok,1,green,-1,0,1,NA,NA,NA,NA,0,0,1");
        assert(vision.lower.fresh && vision.lower.objects[0].in_safety_zone && vision.lower.objects[0].in_roi);
        tick += 150;
        vision_process();
        assert(vision.lower.objects[0].in_safety_zone); /* Geometry survives distance validity expiry. */
        feed_body("RV4,lower,103,0,ok,1,green,-1,0,1,NA,NA,NA,NA,0,0,2");
        feed_body("RV4,lower,103,0,ok,1,blue_casualty_zone,-1,0,0,NA,NA,NA,NA,0,0,1");
        feed_body("RV4,lower,103,0,ok,1,green,-1,0,1,NA,NA,0,NA,0,0,1");
        feed_body("RV4,lower,103,0,ok,1,green,-1,0,1,NA,NA,NA,NA,0,0");
        assert(vision.bad_packets == bad + 4 && vision.lower.frame_id == 102);
        feed_body("RV3,lower,103,0,ok,1,green,-1,0,1,NA,NA,NA,red_material_zone,1,1");
        assert(vision.lower.frame_id == 103 && !vision.lower.objects[0].in_safety_zone);
        assert(vision.lower.objects[0].in_roi);
    }
    puts("PASS: RV4 safety-zone bit, malformed fields and RV3 compatibility");
    return 0;
}

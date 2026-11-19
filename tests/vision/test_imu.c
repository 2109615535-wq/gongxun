/* Actual HDZ601 receiver + IMU conversion; no hardware or motor commands. */
#define TEST_HDZ
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../../project/Core/Src/hdz601.c"
#include "../../../project/Core/Src/imu.c"
UART_HandleTypeDef huart2={USART2,1};
static uint32_t test_tick;
uint32_t HAL_GetTick(void){return test_tick;}
int HAL_UART_Init(UART_HandleTypeDef *h){(void)h;return HAL_OK;}
int HAL_UART_DeInit(UART_HandleTypeDef *h){(void)h;return HAL_OK;}
int HAL_UART_Transmit(UART_HandleTypeDef *h,uint8_t *p,uint16_t n,uint32_t t){(void)h;(void)p;(void)n;(void)t;return HAL_OK;}
int HAL_UART_Receive_IT(UART_HandleTypeDef *h,uint8_t *p,uint16_t n){(void)h;(void)p;(void)n;return HAL_OK;}
void dl_delay(uint32_t ms){test_tick+=ms;}
void OLED_ShowFloatNum2(int16_t x,int16_t y,double n,uint8_t a,uint8_t b){(void)x;(void)y;(void)n;(void)a;(void)b;}
void vision_uart_rx_complete(UART_HandleTypeDef *h){(void)h;}
void vision_uart_error(UART_HandleTypeDef *h){(void)h;}
void faraway_uart_rx_complete(UART_HandleTypeDef *h){(void)h;}
void faraway_uart_error(UART_HandleTypeDef *h){(void)h;}
static void enqueue(uint8_t byte){hdz601_uart2_rx_byte=byte;HAL_UART_RxCpltCallback(&huart2);}
static void packet(unsigned cmd,const uint8_t *data,unsigned n,int corrupt){
 unsigned i;uint8_t crc=(uint8_t)(0x60+cmd+n);
 enqueue(0xAA);enqueue(0x55);enqueue(0x60);enqueue((uint8_t)cmd);enqueue((uint8_t)n);
 for(i=0;i<n;i++){enqueue(data[i]);crc=(uint8_t)(crc+data[i]);}enqueue((uint8_t)(crc+corrupt));
}
static void attitude(unsigned yaw){uint8_t p[6]={(uint8_t)yaw,(uint8_t)(yaw>>8),0,0,0,0};packet(1,p,6,0);hdz601_process();imu_process();}
static void degrees(float actual,float expected){assert(fabsf(actual/IMU_DEG_RAD-expected)<.001f);}
int main(void){
 float yaw;uint32_t stamp,count;uint8_t payload[18]={0},ack=0x0A;
 hdz601_init();assert(!imu_init());assert(!imu_at(test_tick,&yaw));
 test_tick=3000;attitude(35900);assert(imu.ready);degrees(angle_imu,0);
 test_tick=3020;attitude(100);degrees(angle_imu,2);assert(imu_at(3010,&yaw));degrees(yaw,1);
 assert(!imu_at(2999,&yaw));
 test_tick=3040;attitude(35900);degrees(angle_imu,0);assert(imu_at(3030,&yaw));degrees(yaw,1);
 stamp=imu.tick;count=imu.frame;
 test_tick+=5;packet(0xF0,&ack,1,0);hdz601_process();imu_process();assert(imu.frame==count&&imu.tick==stamp);
 packet(1,payload,6,1);hdz601_process();imu_process();assert(imu.frame==count&&g_hdz601_checksum_error_count==1);
 packet(1,payload,2,0);hdz601_process();imu_process();assert(imu.frame==count&&g_hdz601_frame_error_count==1);
 test_tick=stamp+400;assert(imu_fresh(test_tick)&&imu_at(test_tick,&yaw));
 test_tick++;imu_process();assert(!imu_fresh(test_tick)&&!imu_at(test_tick,&yaw));
 /* Receive timestamp survives delayed main-loop parsing; ACK never renews it. */
 test_tick=4000;payload[0]=0x28;payload[1]=0x23;packet(1,payload,6,0);
 test_tick=4050;hdz601_process();imu_process();assert(imu.tick==4000);degrees(angle_imu,91);assert(!imu_at(3500,&yaw));
 assert(imu_init());degrees(angle_imu,0);
 /* Full layout has yaw last. 90 -> 180 is a positive quarter turn. */
 payload[16]=0x50;payload[17]=0x46;test_tick=4070;packet(1,payload,18,0);hdz601_process();imu_process();degrees(angle_imu,90);
 /* Reset stale cache waits for a genuinely new fresh frame. */
 test_tick=5000;assert(!imu_init());attitude(27000);degrees(angle_imu,0);
 /* Ring replacement and HAL tick rollover. */
 for(count=0;count<100;count++){test_tick+=10;attitude(27000+count);}
 assert(imu_count==IMU_HISTORY_SIZE);assert(!imu_at(test_tick-700,&yaw));
 test_tick=0xfffffff0U;attitude(35900);assert(imu_init());test_tick=4;attitude(100);
 degrees(angle_imu,2);assert(imu_at(0xfffffffaU,&yaw));degrees(yaw,1);
 puts("PASS: actual HDZ601 UART timestamp, both layouts, ACK/bad-frame isolation, relative zero, CW sign, 359/1 unwrap, encoder-time interpolation, no future substitution, stale400/401, reset/recovery and tick/ring wrap");
 return 0;
}

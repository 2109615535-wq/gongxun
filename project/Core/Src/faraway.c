#include "faraway.h"
#include "vision.h"
#include "key.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "vision_simple.h"
#include "motor.h"
#include "decode_move.h"
#include "field_map.h"
#include "imu.h"
#include <math.h>

#define FARAWAY_RX_SIZE       512U
#define FARAWAY_RX_MASK       (FARAWAY_RX_SIZE - 1U)
#define FARAWAY_TX_SIZE       4096U
#define FARAWAY_TX_CHUNK 64U
#define FARAWAY_TX_GAP_MS 5U
#define FARAWAY_TX_MASK       (FARAWAY_TX_SIZE - 1U)
#define FARAWAY_LINE_SIZE     96U
#define FARAWAY_PROCESS_LIMIT 128U

extern UART_HandleTypeDef huart5;
extern TIM_HandleTypeDef htim1;
extern PID_t pid_motor1;
extern PID_t pid_motor2;
extern PID_t pid_motor3;
extern PID_t pid_motor4;

static uint8_t rx_byte;
static uint8_t rx_buffer[FARAWAY_RX_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool rx_fault;
static uint8_t tx_buffer[FARAWAY_TX_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint16_t tx_length;
static volatile bool opened;
static volatile uint32_t tx_next_ms;
static char line[FARAWAY_LINE_SIZE];
static uint16_t line_length;
static bool discard_line;
static uint8_t command_seen, command_status;
static uint32_t command_id;
static uint16_t command_crc;
static uint8_t steer_stream;
static uint32_t steer_stream_tick, steer_last;

volatile uint32_t g_faraway_rx_overflow_count;
volatile uint32_t g_faraway_uart_error_count;
volatile uint32_t g_faraway_tx_drop_count;

/* Called with interrupts masked, or from the UART ISR. The queued storage
 * remains valid until HAL reports that this contiguous part has completed. */
static void faraway_tx_start(void)
{
    uint16_t length;

    if (!opened || tx_length != 0U || tx_tail == tx_head) {
        return;
    }
    length = tx_head > tx_tail ? (uint16_t)(tx_head - tx_tail)
                              : (uint16_t)(FARAWAY_TX_SIZE - tx_tail);
    if ((int32_t)(HAL_GetTick() - tx_next_ms) < 0) return;
    if (length > FARAWAY_TX_CHUNK) length = FARAWAY_TX_CHUNK;
    tx_length = length;
    if (HAL_UART_Transmit_IT(&huart5, &tx_buffer[tx_tail], length) != HAL_OK) {
        tx_length = 0U;
        g_faraway_uart_error_count++;
    }
}

static void faraway_send(const char *text)
{
    uint32_t irq;
    uint16_t length;
    uint16_t free_bytes;
    uint16_t i;
    uint16_t next_head;

    if (!opened) {
        return;
    }
    length = (uint16_t)strlen(text);
    irq = __get_PRIMASK();
    __disable_irq();
    free_bytes = (uint16_t)((tx_tail + FARAWAY_TX_SIZE - tx_head - 1U)
                            & FARAWAY_TX_MASK);
    if (length > free_bytes) {
        g_faraway_tx_drop_count++;
        __set_PRIMASK(irq);
        return;
    }
    next_head = tx_head;
    __set_PRIMASK(irq);
    /* Sole producer is the main loop. Keep RX interrupts enabled while copying
     * a long report; ISR sees the old head until the complete frame is ready. */
    for (i = 0U; i < length; i++) {
        tx_buffer[next_head] = (uint8_t)text[i];
        next_head = (uint16_t)((next_head + 1U) & FARAWAY_TX_MASK);
    }
    irq = __get_PRIMASK();
    __disable_irq();
    __DMB();
    tx_head = next_head;
    faraway_tx_start();
    __set_PRIMASK(irq);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        vision_uart_tx_complete(huart);
        return;
    }
    if (huart->Instance == UART5 && opened) {
        tx_tail = (uint16_t)((tx_tail + tx_length) & FARAWAY_TX_MASK);
        tx_length = 0U;
        tx_next_ms = HAL_GetTick() + FARAWAY_TX_GAP_MS;
    }
}

bool faraway_init(void)
{
    if (opened) {
        return true;
    }
    if (huart5.Instance != UART5) {
        return false;
    }
    rx_head = 0U;
    rx_tail = 0U;
    rx_fault = false;
    tx_head = 0U;
    tx_tail = 0U;
    tx_length = 0U;
    line_length = 0U;
    discard_line = false;
    command_seen = command_status = 0; command_id = 0;
    steer_stream = 0; steer_stream_tick = steer_last = 0;
    g_faraway_rx_overflow_count = 0U;
    g_faraway_uart_error_count = 0U;
    g_faraway_tx_drop_count = 0U;
    tx_next_ms = HAL_GetTick();
    opened = true;
    __HAL_UART_CLEAR_OREFLAG(&huart5);
    if (HAL_UART_Receive_IT(&huart5, &rx_byte, 1U) != HAL_OK) {
        opened = false;
        g_faraway_uart_error_count++;
        return false;
    }
    return true;
}

void faraway_uart_rx_complete(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (huart->Instance != UART5 || !opened) {
        return;
    }
    next = (uint16_t)((rx_head + 1U) & FARAWAY_RX_MASK);
    if (next == rx_tail) {
        rx_fault = true;
        g_faraway_rx_overflow_count++;
    } else {
        rx_buffer[rx_head] = rx_byte;
        rx_head = next;
    }
    if (HAL_UART_Receive_IT(huart, &rx_byte, 1U) != HAL_OK) {
        rx_fault = true;
        g_faraway_uart_error_count++;
    }
}

void faraway_uart_error(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART5 || !opened) {
        return;
    }
    g_faraway_uart_error_count++;
    rx_fault = true;
    (void)HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    if (HAL_UART_Receive_IT(huart, &rx_byte, 1U) != HAL_OK) {
        g_faraway_uart_error_count++;
    }
}

static char *faraway_trim(char *text)
{
    char *end;

    while (*text == ' ' || *text == '\t') {
        text++;
    }
    end = text + strlen(text);
    while (end > text && (end[-1] == ' ' || end[-1] == '\t')) {
        *--end = '\0';
    }
    return text;
}

/* Compact v2: [kind, distance_mm, angle_centidegrees|null, valid].
 * One outstanding snapshot; CRC detects missing/corrupted wireless bytes. */
static unsigned telemetry_fault(void)
{
    if (!strcmp(vision.fault_code, "waiting_rv2")) return 1;
    if (!strcmp(vision.fault_code, "invalid_packet")) return 2;
    if (!strcmp(vision.fault_code, "uart_error")) return 3;
    if (!strcmp(vision.fault_code, "vision_timeout")) return 4;
    if (!strcmp(vision.fault_code, "upper_unavailable")) return 5;
    return 0;
}
static void faraway_vision_report(void)
{
    static char report[1280];
    const VisionObservation *obs = &vision.upper;
    uint32_t now = HAL_GetTick(), elapsed = now - obs->updated_ms;
    uint32_t total_age = obs->age_ms;
    unsigned fresh, aligned, status = (unsigned)obs->status, i, count;
    int used, written;
    char age[24], angle[16];
    uint16_t crc;
    if (tx_head != tx_tail) { ++g_faraway_tx_drop_count; return; }
    if (total_age != UINT32_MAX)
        total_age = elapsed > UINT32_MAX - 1U - total_age ? UINT32_MAX - 1U : total_age + elapsed;
    fresh = obs->fresh && total_age != UINT32_MAX && total_age <= VSP_TIMEOUT_MS;
    count = fresh ? obs->count : 0;
    aligned = fresh && field_map.aligned && field_map.frame == obs->frame_id;
    if (!fresh && obs->fresh) status = VISION_OBS_STALE;
    if (total_age == UINT32_MAX) strcpy(age, "null");
    else (void)snprintf(age, sizeof(age), "%lu", (unsigned long)total_age);
    /* V3 array: tag,tick,link,rx,bad,uart,drops,upper_frame,age,status,
     * fresh,fault,upper_objects,servo_angle,lower_observation. */
    used = snprintf(report, sizeof(report),
        "[\"V7\",%lu,%u,%lu,%lu,%lu,%lu,%lu,%s,%u,%u,%u,[",
        (unsigned long)now, vision.connected,
        (unsigned long)vision.rx_packets, (unsigned long)vision.bad_packets,
        (unsigned long)vision.uart_errors, (unsigned long)g_faraway_tx_drop_count,
        (unsigned long)obs->frame_id, age, status, fresh,
        telemetry_fault());
    if (used < 0 || (unsigned)used >= sizeof(report)) goto overflow;
    for (i = 0; i < count; ++i) {
        const VisionObject *o = &obs->objects[i];
        unsigned valid = obs->available && total_age < 150U && o->distance_mm >= 0 &&
                         o->angle_present && o->distance_valid && o->angle_valid;
        if (o->angle_present) {
            long a = (long)(o->angle * 100.0f + (o->angle < 0 ? -0.5f : 0.5f));
            (void)snprintf(angle, sizeof(angle), "%ld", a);
        } else strcpy(angle, "null");
        written = snprintf(report + used, sizeof(report) - (unsigned)used,
                           "%s[%u,%ld,%s,%u]", i ? "," : "", (unsigned)o->kind,
                           (long)o->distance_mm, angle, valid);
        if (written < 0 || (unsigned)written >= sizeof(report) - (unsigned)used) goto overflow;
        used += written;
    }
    /* Lower camera stays independent of upper freshness. Compact entries are
     * [kind,in_roi]; coordinates are deliberately not required for ROI testing. */
    {
        const VisionObservation *lower = &vision.lower;
        uint32_t lower_age = lower->age_ms, wait = now - lower->updated_ms;
        unsigned lower_fresh, lower_status = (unsigned)lower->status;
        if (lower_age != UINT32_MAX)
            lower_age = wait > UINT32_MAX - 1U - lower_age ? UINT32_MAX - 1U : lower_age + wait;
        lower_fresh = lower->fresh && lower_age != UINT32_MAX && lower_age <= VSP_TIMEOUT_MS;
        if (!lower_fresh && lower->fresh) lower_status = VISION_OBS_STALE;
        if (lower_age == UINT32_MAX) strcpy(age, "null");
        else (void)snprintf(age, sizeof(age), "%lu", (unsigned long)lower_age);
        written = snprintf(report + used, sizeof(report) - (unsigned)used,
                           "],%d,[%lu,%s,%u,%u,[", (int)servo_angle,
                           (unsigned long)lower->frame_id, age, lower_status, lower_fresh);
        if (written < 0 || (unsigned)written >= sizeof(report) - (unsigned)used) goto overflow;
        used += written;
        for (i = 0; lower_fresh && i < lower->count; ++i) {
            written = snprintf(report + used, sizeof(report) - (unsigned)used,
                               "%s[%u,%u]", i ? "," : "", (unsigned)lower->objects[i].kind,
                               lower->objects[i].in_roi);
            if (written < 0 || (unsigned)written >= sizeof(report) - (unsigned)used) goto overflow;
            used += written;
        }
    }
    if ((unsigned)used + 10U > sizeof(report)) goto overflow;
    written = snprintf(report + used, sizeof(report) - (unsigned)used,
                       "]],[%u,%u,%u,%u,%u]", step, task_info.reason,
                       task_info.inside_frames, task_info.outside_frames, task_info.held_kind);
    if (written < 0 || (unsigned)written + 8U >= sizeof(report) - (unsigned)used) goto overflow;
    used += written;
    /* V5 appends [robot pose,aligned,observation pose|null,world XY list,
     * start pose,vision extrinsics]. Positions mm; angles centidegrees. */
    written = snprintf(report + used, sizeof(report) - (unsigned)used,
        ",[[%ld,%ld,%ld],%u,",
        (long)(field_map.robot.x * 10), (long)(field_map.robot.y * 10),
        (long)(fmodf(field_map.robot.yaw * 18000.0f / 3.14159265358979323846f, 36000.0f)),
        (unsigned)aligned);
    if (written < 0 || (unsigned)written >= sizeof(report) - (unsigned)used) goto overflow;
    used += written;
    if (aligned)
        written = snprintf(report + used, sizeof(report) - (unsigned)used, "[%ld,%ld,%ld],[",
            (long)(field_map.observed.x * 10), (long)(field_map.observed.y * 10),
            (long)(fmodf(field_map.observed.yaw * 18000.0f / 3.14159265358979323846f, 36000.0f)));
    else written = snprintf(report + used, sizeof(report) - (unsigned)used, "null,[");
    if (written < 0 || (unsigned)written >= sizeof(report) - (unsigned)used) goto overflow;
    used += written;
    for (i = 0; i < count; ++i) {
        const FieldPoint *p = &field_map.points[i];
        if (aligned && i < field_map.count && p->present)
            written = snprintf(report + used, sizeof(report) - (unsigned)used,
                "%s[%ld,%ld,%lu,%ld]", i ? "," : "", (long)(p->x * 10), (long)(p->y * 10),
                (unsigned long)p->track_id, (long)(p->radius * 10));
        else written = snprintf(report + used, sizeof(report) - (unsigned)used, "%snull", i ? "," : "");
        if (written < 0 || (unsigned)written >= sizeof(report) - (unsigned)used) goto overflow;
        used += written;
    }
    written = snprintf(report + used, sizeof(report) - (unsigned)used,
        "],[%ld,%ld,%ld],[%ld,%ld,%ld]]",
        (long)(FIELD_START_X_CM * 10), (long)(FIELD_START_Y_CM * 10),
        (long)(FIELD_START_YAW_DEG * 100), (long)(FIELD_VISION_X_CM * 10),
        (long)(FIELD_VISION_Y_CM * 10), (long)(FIELD_VISION_YAW_DEG * 100));
    if (written < 0 || (unsigned)written + 8U >= sizeof(report) - (unsigned)used) goto overflow;
    used += written;
    {
        char id[16];
        if (command_seen) snprintf(id, sizeof(id), "%lu", (unsigned long)command_id);
        else strcpy(id, "null");
        if (goto_target.valid)
            written = snprintf(report + used, sizeof(report)-(unsigned)used,
                ",[%s,%u,%ld,%ld,%.1f]", id, command_status,
                (long)(goto_target.x*10), (long)(goto_target.y*10), (double)goto_target.speed);
        else written = snprintf(report + used, sizeof(report)-(unsigned)used,
                ",[%s,%u,null,null,null]", id, command_status);
        if (written < 0 || (unsigned)written+8U >= sizeof(report)-(unsigned)used) goto overflow;
        used += written;
    }
    /* V7: IMU ready, age, raw yaw (centidegrees), aligned heading available. */
    if (imu.ready)
        written = snprintf(report + used, sizeof(report)-(unsigned)used,
            ",[%u,%lu,%ld,%u]]", 1U, (unsigned long)(now-imu.tick),
            (long)(imu.raw_yaw*100.0f+0.5f), (unsigned)field_map.heading_valid);
    else written = snprintf(report + used, sizeof(report)-(unsigned)used, ",[0,null,null,0]]");
    if (written < 0 || (unsigned)written+8U >= sizeof(report)-(unsigned)used) goto overflow;
    used += written;
    crc = VSP_Crc16((const uint8_t *)report, (uint16_t)used);
    (void)snprintf(report + used, sizeof(report) - (unsigned)used, "*%04X\r\n", crc);
    faraway_send(report);
    return;
overflow:
    ++g_faraway_tx_drop_count;
}

/* One bounded map record per 300 ms; changed records first, with periodic
 * repair copies. MAP requests a round-robin snapshot after page reconnect.
 * These are telemetry only; map creation never runs here. */
static uint32_t map_sent[FIELD_TRACK_CAPACITY], map_epoch, map_last;
static unsigned map_cursor, map_repair_cursor, map_packets, map_sync_remaining;
static uint8_t map_started;
static void faraway_map_report(uint32_t now)
{
    char report[640];
    int slot=-1, used, n;
    unsigned i,j,repair=0;
    const FieldTrack *t=NULL;
    if(map_epoch!=field_map.epoch) {
        memset(map_sent,0,sizeof(map_sent));map_epoch=field_map.epoch;
        map_cursor=map_repair_cursor=map_packets=0;map_sync_remaining=field_map.track_count;
    }
    if(!map_sync_remaining && map_packets%4U!=3U) {
        for(i=0;i<FIELD_TRACK_CAPACITY;++i) {
            j=(map_cursor+i)%FIELD_TRACK_CAPACITY;
            if(field_tracks[j].id && field_tracks[j].revision!=map_sent[j]) {slot=(int)j;break;}
        }
    }
    if(slot<0) {
        repair=1;
        for(i=0;i<FIELD_TRACK_CAPACITY;++i) {
            j=(map_repair_cursor+i)%FIELD_TRACK_CAPACITY;
            if(field_tracks[j].id) {slot=(int)j;break;}
        }
    }
    used=snprintf(report,sizeof(report),"[\"M1\",%lu,%lu,%lu,%u,%lu,",
        (unsigned long)now,(unsigned long)field_map.epoch,(unsigned long)field_map.revision,
        (unsigned)field_map.track_count,(unsigned long)field_map.unmapped_count);
    if(used<0 || (unsigned)used>=sizeof(report))goto overflow;
    if(slot>=0) {
        t=&field_tracks[slot];
        n=snprintf(report+used,sizeof(report)-(unsigned)used,
            "[%lu,%u,%ld,%ld,%ld,%lu,%lu,%lu,%u,%u,%lu,[",
            (unsigned long)t->id,(unsigned)t->kind,(long)(t->x*10),(long)(t->y*10),
            (long)((FIELD_MATCH_BASE_CM+FIELD_MATCH_DISTANCE_SCALE*t->distance)*10),
            (unsigned long)t->seen_ms,(unsigned long)t->position_ms,(unsigned long)t->observations,
            (unsigned)t->visible,(unsigned)t->review,(unsigned long)t->revision);
        if(n<0 || (unsigned)n>=sizeof(report)-(unsigned)used)goto overflow;
        used+=n;
        for(i=0;i<t->history_count;++i) {
            const FieldSample *h=&t->history[(t->history_next+FIELD_TRACK_HISTORY-t->history_count+i)%FIELD_TRACK_HISTORY];
            n=snprintf(report+used,sizeof(report)-(unsigned)used,"%s[%lu,%ld,%ld,%u]",
                i?",":"",(unsigned long)h->tick,(long)(h->x*10),(long)(h->y*10),(unsigned)h->accepted);
            if(n<0 || (unsigned)n>=sizeof(report)-(unsigned)used)goto overflow;
            used+=n;
        }
        n=snprintf(report+used,sizeof(report)-(unsigned)used,"]]]");
    } else n=snprintf(report+used,sizeof(report)-(unsigned)used,"null]");
    if(n<0 || (unsigned)n+8U>=sizeof(report)-(unsigned)used)goto overflow;
    used+=n;
    snprintf(report+used,sizeof(report)-(unsigned)used,"*%04X\r\n",
        VSP_Crc16((const uint8_t *)report,(uint16_t)used));
    faraway_send(report);
    if(t) {map_sent[slot]=t->revision;map_cursor=((unsigned)slot+1U)%FIELD_TRACK_CAPACITY;
        if(repair)map_repair_cursor=map_cursor;}
    if(map_sync_remaining)--map_sync_remaining;
    ++map_packets;map_last=now;map_started=1;
    return;
overflow:
    ++g_faraway_tx_drop_count;
}
/* Optional 5Hz compact steering snapshots; only while tuning page is open.
 * A 6-second lease stops the extra traffic after page loss/disconnection. */
static void faraway_steer_report(uint32_t now)
{
    char report[512], age[16];
    int n;
    if (steer_state.source) snprintf(age, sizeof(age), "%lu", (unsigned long)(now-steer_state.tick));
    else strcpy(age, "null");
    n = snprintf(report, sizeof(report),
        "[\"S1\",%lu,%u,%s,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g]",
        (unsigned long)now, (unsigned)steer_state.source, age,
        (double)pid_steer.target, (double)pid_steer.current, (double)pid_steer.error_now,
        (double)pid_steer.output, (double)steer_state.p, (double)steer_state.i,
        (double)steer_state.d, (double)pid_steer.kp, (double)pid_steer.ki, (double)pid_steer.kd);
    if (n < 0 || (unsigned)n+8U >= sizeof(report)) { ++g_faraway_tx_drop_count; return; }
    snprintf(report+n, sizeof(report)-(unsigned)n, "*%04X\r\n",
        VSP_Crc16((const uint8_t *)report, (uint16_t)n));
    faraway_send(report); steer_last = now;
}
void faraway_report(void)
{
    static uint32_t last_report;
    static bool reported;
    uint32_t now=HAL_GetTick();
    if(!opened)return;
    if(steer_stream && now-steer_stream_tick>6000U)steer_stream=0;
    if(steer_stream && tx_head==tx_tail && reported && now-last_report<150U && now-steer_last>=200U) {
        faraway_steer_report(now);return;
    }
    /* No extra queued backlog. A map packet can delay a status report once;
     * the following scheduler call sends the latest status, never an old queue. */
    if(tx_head==tx_tail && reported &&
       (map_started?now-map_last>=300U:now-last_report>=50U)) {
        faraway_map_report(now);return;
    }
    if(reported && now-last_report<150U)return;
    last_report=now;reported=true;
    faraway_vision_report();
}

/* Checksummed, sequenced control lines. No implicit retries or legacy motion commands. */
static uint8_t command_uint(const char *s, uint32_t *value)
{
    uint32_t n = 0, digit;
    if (!*s) return 0;
    while (*s) {
        if (*s < '0' || *s > '9') return 0;
        digit = (uint32_t)(*s++ - '0');
        if (n > (UINT32_MAX-digit)/10U) return 0;
        n = n*10U+digit;
    }
    *value = n; return 1;
}
static void command_ack(uint32_t id, unsigned status)
{
    char reply[80]; int n;
    n = snprintf(reply, sizeof(reply), "A1,%lu,%u,%u", (unsigned long)id, status, step);
    snprintf(reply+n, sizeof(reply)-(unsigned)n, "*%04X\r\n",
             VSP_Crc16((const uint8_t *)reply, (uint16_t)n));
    faraway_send(reply);
}
static void faraway_command(char *command)
{
    char *star, *parts[6], *p;
    uint32_t id, args[3];
    unsigned count=1, i, crc=0, digit, status=3;
    uint16_t actual;
    uint8_t stop=0;
    command=faraway_trim(command);
    if (!strcmp(command,"MAP")) { map_sync_remaining=field_map.track_count;map_repair_cursor=0;return; }
    if (!strcmp(command,"GET")) { faraway_vision_report(); return; }
    if (strncmp(command,"C1,",3)) { faraway_send("ERR,TASK_CONTROL\r\n"); return; }
    star=strrchr(command,'*');
    if (!star || strlen(star+1)!=4) { faraway_send("ERR,CONTROL_CRC\r\n"); return; }
    for(i=1;i<=4;i++) {
        if(star[i]>='0'&&star[i]<='9')digit=(unsigned)(star[i]-'0');
        else if(star[i]>='A'&&star[i]<='F')digit=(unsigned)(star[i]-'A'+10);
        else if(star[i]>='a'&&star[i]<='f')digit=(unsigned)(star[i]-'a'+10);
        else {faraway_send("ERR,CONTROL_CRC\r\n");return;}
        crc=crc*16U+digit;
    }
    actual=VSP_Crc16((const uint8_t *)command,(uint16_t)(star-command));
    if(actual!=crc) {faraway_send("ERR,CONTROL_CRC\r\n");return;}
    *star=0;parts[0]=command;
    for(p=command;*p;p++) if(*p==',') {
        *p=0;if(count>=6){faraway_send("ERR,CONTROL_FORMAT\r\n");return;}
        parts[count++]=p+1;
    }
    if(count<4 || !command_uint(parts[1],&id)) {faraway_send("ERR,CONTROL_FORMAT\r\n");return;}
    for(i=3;i<count;i++) if(!command_uint(parts[i],&args[i-3])){command_ack(id,3);return;}
    stop=count==4 && !strcmp(parts[2],"K") && args[0]==4;
    /* KEY4 is never blocked by sequence ordering. Older movement is rejected. */
    if(command_seen && (int32_t)(id-command_id)<=0) {
        if(stop) {Key_Press(KEY_4);command_ack(id,0);return;}
        command_ack(id,id==command_id && actual==command_crc ? command_status : 4);return;
    }
    if(count==4 && !strcmp(parts[2],"K") && args[0]>=1 && args[0]<=4)
        status=Key_Press((uint8_t)(1U<<(args[0]-1U)));
    else if(count==6 && !strcmp(parts[2],"G") && args[0]<=3000 && args[1]<=3000 && args[2]>0) {
        if(Key_Read()&KEY_4)status=2;
        else status=task_goto_start(args[0]*0.1f,args[1]*0.1f,args[2]*0.1f)?0:3;
    }
    else if(count==6 && !strcmp(parts[2],"P"))
        status=steering_set_pid(args[0]*0.0001f,args[1]*0.0001f,args[2]*0.0001f)?0:3;
    else if(count==4 && !strcmp(parts[2],"T") && args[0]<=1) {
        if(args[0] && !steer_stream)steer_last=HAL_GetTick()-200U;
        steer_stream=(uint8_t)args[0];steer_stream_tick=HAL_GetTick();status=0;
    }
    command_seen=1;command_id=id;command_crc=actual;command_status=(uint8_t)status;
    command_ack(id,status);
}

void faraway_process(void)
{
    uint16_t i;
    uint8_t byte;
    uint32_t irq;

    if (!opened) {
        return;
    }
    for (i = 0U; i < FARAWAY_PROCESS_LIMIT; i++) {
        irq = __get_PRIMASK();
        __disable_irq();
        if (rx_fault) {
            rx_tail = rx_head;
            rx_fault = false;
            line_length = 0U;
            discard_line = true;
            __set_PRIMASK(irq);
            faraway_send("ERR,RX_LOST\r\n");
            break;
        }
        if (rx_tail == rx_head) {
            faraway_tx_start();
            __set_PRIMASK(irq);
            break;
        }
        byte = rx_buffer[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) & FARAWAY_RX_MASK);
        __set_PRIMASK(irq);

        if (byte == '\r' || byte == '\n') {
            if (!discard_line && line_length != 0U) {
                line[line_length] = '\0';
                faraway_command(line);
            }
            line_length = 0U;
            discard_line = false;
        } else if (!discard_line) {
            if (line_length >= FARAWAY_LINE_SIZE - 1U ||
                (byte < 0x20U && byte != '\t') || byte > 0x7EU) {
                line_length = 0U;
                discard_line = true;
                faraway_send("ERR,LINE\r\n");
            } else {
                line[line_length++] = (char)byte;
            }
        }
    }
}

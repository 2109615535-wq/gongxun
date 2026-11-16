#ifndef VISION_SIMPLE_H
#define VISION_SIMPLE_H

/* 本文件定义主控接收数据的结构和调用接口，具体解析过程在vision_simple.c。
 * 对照Python阅读时，VSP_Object对应SimpleObject，VSP_Observation对应SimpleObservation。
 * 接入顺序为VSP_Init初始化、VSP_Feed投递字节、VSP_Get读取最新结果。 */
#include <stdint.h>

#define VSP_MAX_OBJECTS 12u  /* 每路每帧最多12个物体，u表示无符号整数常量。 */
#define VSP_MAX_LINE 512u    /* 一行协议数据的字节数上限，换行符另算。 */
#define VSP_TIMEOUT_MS 300u  /* 发送侧年龄加接收后经过的时间达到300ms时失效。 */

/* enum枚举给整数起可读的名称，上视为0，下视为1。 */
typedef enum { VSP_UPPER = 0, VSP_LOWER = 1 } VSP_Source;
/* 内部枚举的顺序与vision_simple.c的names数组对应，串口上传输英文名称。 */
typedef enum {
    /* 有目标、空场、过期、故障、目标数量超限、尚无数据。 */
    VSP_GREEN = 1, VSP_ORANGE, VSP_BLUE, VSP_BLACK,
    VSP_RED_CASUALTY_ZONE, VSP_RED_MATERIAL_ZONE,
    VSP_BLUE_CASUALTY_ZONE, VSP_BLUE_MATERIAL_ZONE
} VSP_Kind;
typedef enum {
    VSP_OK = 0, VSP_EMPTY, VSP_STALE, VSP_FAULT, VSP_OVERFLOW, VSP_NO_DATA
} VSP_Status;

typedef struct {
    VSP_Kind kind;          /* 物体类别，用VSP_GREEN等枚举名判断。 */
    int32_t distance_mm;    /* 地面径向距离，单位毫米，-1表示距离未知。 */
    uint8_t distance_valid; /* 1表示距离有效，0时数值可能只是未验证估算。 */
    uint8_t in_roi;         /* 1表示在下视夹爪区域内，夹持成功需另外判断。 */
    float angle;           /* RV2地面方位角，单位度，右正左负；无方位时为0。 */
    uint8_t angle_valid;    /* 有angle且距离/位置验证与150ms新鲜度均满足时为1。 */
    uint8_t angle_present;  /* RV2是否提供角度数值，供显示估计值；NA和RV1为0。 */
} VSP_Object;

/* 一路相机的一次观测，程序只遍历count个objects条目。 */
typedef struct {
    uint32_t frame_id;      /* uint32_t表示32位无符号整数，可用于记录帧编号。 */
    int32_t age_ms;         /* 结果年龄，单位毫秒，未知为-1。 */
    VSP_Status status;      /* 当前结果的状态。 */
    uint8_t count;          /* 本帧物体数量。 */
    VSP_Object objects[VSP_MAX_OBJECTS];
} VSP_Observation;

/* 每路相机的接收历史，用于识别重复旧帧和计算结果是否过期。 */
typedef struct {
    VSP_Observation observation;
    uint32_t received_ms;            /* 最近一次接受正常观测的主控本地时间。 */
    uint32_t last_frame_id;          /* 最近接受的正常图像编号，故障帧保留此记录。 */
    uint32_t last_frame_received_ms; /* 配合编号判断短时倒序和发送端重启。 */
    uint8_t seen_frame;             /* 是否曾收到正常图像。 */
    uint8_t available;              /* 是否已保存正常结果，读取时还会检查超时。 */
} VSP_Channel;

typedef struct {
    char line[VSP_MAX_LINE + 1u]; /* 暂存一行，额外1字节供C字符串结束标记使用。 */
    uint16_t length;              /* 已经收到的字节数。 */
    uint8_t dropping;             /* 异常长行丢弃到换行，再恢复接收。 */
    VSP_Channel cameras[2];       /* 分别保存上视和下视，独立计算有效期。 */
    uint32_t format_errors;       /* 格式或CRC错误次数。 */
    uint32_t duplicates;          /* 重复或短时倒序帧次数。 */
} VSP_Receiver;

/* 这些函数统一在同一个主循环或任务中调用。
 * UART中断/DMA先使用现有驱动将字节和到达时间放入队列，主循环再取出处理。
 * received_ms填写字节实际到达主控的时间，确保排队等待也计入结果年龄。 */
/* 开始接收前调用一次，receiver通常指向一个静态VSP_Receiver变量。 */
void VSP_Init(VSP_Receiver *receiver);
/* 每取出一个接收字节调用一次，遇到换行符后自动解析完整的一帧。 */
void VSP_Feed(VSP_Receiver *receiver, uint8_t byte, uint32_t received_ms);
/* 周期调用，即使没有新字节也需要调用以检查断流超时。
 * 返回1表示新鲜的OK/EMPTY观测，返回0时output->count为0，status说明原因。
 * source选择上视或下视，now_ms一般填写当前HAL_GetTick()。 */
uint8_t VSP_Get(const VSP_Receiver *receiver, VSP_Source source,
                uint32_t now_ms, VSP_Observation *output);
/* 与Python端相同的CRC16算法，校验收到的一行是否损坏。 */
uint16_t VSP_Crc16(const uint8_t *data, uint16_t length);

#endif

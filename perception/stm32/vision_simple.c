#include "vision_simple.h"
#include <string.h>

/* 从入口阅读本文件时，可按VSP_Feed -> accept -> parse -> VSP_Get的顺序。
 * Feed收集字节，parse拆解一行，accept保存合法的新观测，Get提供带超时检查的结果。
 * static修饰的函数和名称表供本文件内部使用。 */
static const char *const names[] = {
    "green", "orange", "blue", "black", "red_casualty_zone", "red_material_zone",
    "blue_casualty_zone", "blue_material_zone"
};
static const char *const statuses[] = {"ok", "empty", "stale", "fault", "overflow"};

uint16_t VSP_Crc16(const uint8_t *data, uint16_t length) {
    /* 逐字节、逐位计算CCITT-FALSE校验，参数与Python端保持一致。 */
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t bit;
    for (i = 0; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc & 0x8000u) ? ((uint32_t)crc << 1) ^ 0x1021u : (uint32_t)crc << 1);
    }
    return crc;
}

void VSP_Init(VSP_Receiver *r) {
    unsigned i;
    /* memset将整个接收器清零，再把两路初始状态设为尚无数据。 */
    memset(r, 0, sizeof(*r));
    for (i = 0; i < 2; ++i) {
        r->cameras[i].observation.status = VSP_NO_DATA;
        r->cameras[i].observation.age_ms = -1;
    }
}

static int number(const char *text, uint32_t maximum, uint32_t *value) {
    /* 将十进制文字转换成整数，成功返回1，并通过value指针交回结果。 */
    uint32_t n = 0;
    if (!*text) return 0;
    for (; *text; ++text) {
        uint32_t digit;
        if (*text < '0' || *text > '9') return 0;
        digit = (uint32_t)(*text - '0');
        /* 在乘10和相加之前检查上限，防止超长数字导致整数溢出。 */
        if (digit > maximum || n > (maximum - digit) / 10u) return 0;
        n = n * 10u + digit;
    }
    *value = n;
    return 1;
}

static int optional_number(const char *text, int32_t *value) {
    /* 距离和年龄允许特殊值-1，其余值使用相同的整数检查。 */
    uint32_t n;
    if (!strcmp(text, "-1")) { *value = -1; return 1; }
    if (!number(text, 65535u, &n)) return 0;
    *value = (int32_t)n;
    return 1;
}

static int hex_digit(char c) {
    /* 把CRC中的一个十六进制字符转换为0～15的整数，非法字符返回-1。 */
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static int angle_number(const char *text, float *value, uint8_t *present) {
    uint32_t whole = 0, fraction = 0;
    unsigned decimals = 0;
    int negative = 0;
    *value = 0;
    *present = 0;
    if (!strcmp(text, "NA")) return 1;
    if (*text == '-') { negative = 1; ++text; }
    if (*text < '0' || *text > '9') return 0;
    while (*text >= '0' && *text <= '9') {
        whole = whole * 10u + (uint32_t)(*text++ - '0');
        if (whole > 180u) return 0;
    }
    if (*text == '.') {
        ++text;
        while (*text >= '0' && *text <= '9') {
            if (++decimals > 2u) return 0;
            fraction = fraction * 10u + (uint32_t)(*text++ - '0');
        }
        if (!decimals) return 0;
        if (decimals == 1u) fraction *= 10u;
    }
    if (*text || (whole == 180u && fraction)) return 0;
    *value = (float)(whole * 100u + fraction) / 100.f;
    if (negative) *value = -*value;
    *present = 1;
    return 1;
}

static int lookup(const char *value, const char *const *list, unsigned count) {
    /* 在名称表中寻找字符串，返回数组下标，找不到时返回-1。strcmp相等时返回0。 */
    unsigned i;
    for (i = 0; i < count; ++i) if (!strcmp(value, list[i])) return (int)i;
    return -1;
}

static int parse(char *line, VSP_Source *source, VSP_Observation *o) {
    /* source和o是输出参数，校验全部通过后，调用方才会保存解析结果。 */
    char *star = strchr(line, '*');
    char *tokens[6u + 5u * VSP_MAX_OBJECTS];
    char *p;
    unsigned count = 0, i, item_fields;
    uint16_t checksum = 0;
    uint32_t n;
    int status;
    /* 先验证星号后4个字符及CRC，确保正文完整，再拆分正文。 */
    if (!star || strlen(star + 1) != 4) return 0;
    for (i = 1; i <= 4; ++i) {
        int digit = hex_digit(star[i]);
        if (digit < 0) return 0;
        checksum = (uint16_t)((checksum << 4) | digit);
    }
    if (VSP_Crc16((const uint8_t *)line, (uint16_t)(star - line)) != checksum) return 0;
    *star = '\0';
    /* 用字符串结束符替换逗号，每个tokens条目指向一个独立字段。 */
    tokens[count++] = line;
    for (p = line; *p; ++p) {
        if (*p == ',') {
            if (count >= sizeof(tokens) / sizeof(tokens[0])) return 0;
            *p = '\0';
            tokens[count++] = p + 1;
        }
    }
    /* 前6项依次为协议标识、相机来源、图像编号、年龄、状态和物体数。 */
    if (count < 6) return 0;
    if (!strcmp(tokens[0], "RV1")) item_fields = 4u;
    else if (!strcmp(tokens[0], "RV2")) item_fields = 5u;
    else return 0;
    if (!strcmp(tokens[1], "upper")) *source = VSP_UPPER;
    else if (!strcmp(tokens[1], "lower")) *source = VSP_LOWER;
    else return 0;
    memset(o, 0, sizeof(*o));
    if (!number(tokens[2], UINT32_MAX, &o->frame_id) || !optional_number(tokens[3], &o->age_ms)) return 0;
    status = lookup(tokens[4], statuses, 5);
    if (status < 0 || !number(tokens[5], VSP_MAX_OBJECTS, &n) || count != 6u + item_fields * n) return 0;
    o->status = (VSP_Status)status;
    o->count = (uint8_t)n;
    if (o->status == VSP_OK || o->status == VSP_EMPTY) {
        if (o->age_ms < 0 || o->age_ms >= (int32_t)VSP_TIMEOUT_MS) return 0;
        if ((o->status == VSP_OK) != (o->count > 0)) return 0;
    } else if (o->count) return 0;
    /* RV1每物体4项，RV2追加第5项angle；严格按标识校验字段数。 */
    for (i = 0; i < o->count; ++i) {
        unsigned offset = 6u + i * item_fields;
        VSP_Object *obj = &o->objects[i];
        int kind = lookup(tokens[offset], names, 8);
        if (kind < 0 || !optional_number(tokens[offset+1], &obj->distance_mm)) return 0;
        obj->kind = (VSP_Kind)(kind + 1);
        if (!number(tokens[offset+2], 1u, &n)) return 0;
        obj->distance_valid = (uint8_t)n;
        if (!number(tokens[offset+3], 1u, &n)) return 0;
        obj->in_roi = (uint8_t)n;
        if (item_fields == 5u) {
            uint8_t present;
            if (!angle_number(tokens[offset+4], &obj->angle, &present)) return 0;
            obj->angle_valid = (uint8_t)(present && obj->distance_valid);
        }
        if ((obj->distance_valid && obj->distance_mm < 0) || (*source == VSP_UPPER && obj->in_roi)) return 0;
    }
    return 1;
}

static void accept(VSP_Receiver *r, uint32_t now) {
    /* 本函数仅在收齐一行后调用，上视与下视各自保存编号和有效期。 */
    VSP_Source source;
    VSP_Observation o;
    VSP_Channel *channel;
    uint32_t delta;
    if (!parse(r->line, &source, &o)) { ++r->format_errors; return; }
    channel = &r->cameras[source];
    if (o.status != VSP_OK && o.status != VSP_EMPTY) {
        /* 故障结果立即撤销该路可用性，正常图像编号继续留作去重依据。 */
        channel->observation = o;
        channel->available = 0;
        return;
    }
    /* 无符号减法能够处理编号从4294967295回到0的情况。
     * 差值为0表示重复，差值超过半个u32范围时按倒序判断。
     * 超时后允许不同编号重新建立序列，以便发送程序重启后恢复。 */
    delta = o.frame_id - channel->last_frame_id;
    if (channel->seen_frame && (delta == 0 ||
            (delta >= 0x80000000u && now - channel->last_frame_received_ms < VSP_TIMEOUT_MS))) {
        ++r->duplicates;
        return;
    }
    channel->observation = o;
    channel->received_ms = now;
    channel->last_frame_id = o.frame_id;
    channel->last_frame_received_ms = now;
    channel->seen_frame = 1;
    channel->available = 1;
}

void VSP_Feed(VSP_Receiver *r, uint8_t byte, uint32_t received_ms) {
    /* 一次只接收一个字节，尚未收到换行时保存在line数组中。 */
    if (byte == '\n') {
        if (!r->dropping && r->length) {
            if (r->line[r->length - 1u] == '\r') --r->length;
            r->line[r->length] = '\0';
            accept(r, received_ms);
        }
        r->length = 0;
        r->dropping = 0;
    } else if (!r->dropping) {
        /* 遇到超长行或非法字符，丢弃本行剩余内容，在下一个换行处恢复。 */
        if ((r->length >= VSP_MAX_LINE && !(r->length == VSP_MAX_LINE && byte == '\r')) || byte == 0 || byte > 127u) {
            r->length = 0;
            r->dropping = 1;
            ++r->format_errors;
        } else r->line[r->length++] = (char)byte;
    }
}

uint8_t VSP_Get(const VSP_Receiver *r, VSP_Source source, uint32_t now, VSP_Observation *output) {
    /* 先复制结果，再根据当前时间判断是否还可使用。调用方读取output即可。 */
    const VSP_Channel *channel;
    uint32_t elapsed;
    memset(output, 0, sizeof(*output));
    output->status = VSP_NO_DATA;
    output->age_ms = -1;
    if (source != VSP_UPPER && source != VSP_LOWER) return 0;
    channel = &r->cameras[source];
    *output = channel->observation;
    if (!channel->available) { output->count = 0; return 0; }
    /* 结果总年龄等于发送侧记录的年龄加主控收到之后经过的时间。
     * 采用本地时间差计算，香橙派与主控无需对齐各自的系统时钟。 */
    elapsed = now - channel->received_ms;
    if (elapsed >= VSP_TIMEOUT_MS || (uint32_t)output->age_ms >= VSP_TIMEOUT_MS - elapsed) {
        output->status = VSP_STALE;
        output->count = 0;
        return 0;
    }
    output->age_ms += (int32_t)elapsed;
    if (output->age_ms >= 150) {
        /* 150ms是距离有效性窗口，300ms是整条观测的新鲜度窗口。 */
        unsigned i;
        for (i = 0; i < output->count; ++i) {
            output->objects[i].distance_valid = 0;
            output->objects[i].angle_valid = 0;
        }
    }
    return 1;
}

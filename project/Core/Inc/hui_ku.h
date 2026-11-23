#ifndef HUI_KU_H
#define HUI_KU_H

#include <stdint.h>

#define HUI_KU_TASK_STEP 80U

enum { HUI_KEY_PHYSICAL = 1, HUI_KEY_REMOTE };
enum { HUI_START_OK = 0, HUI_START_KEY4_HELD = 2, HUI_START_BUSY };

typedef enum {
    HUI_IDLE, HUI_SEEK, HUI_VISUAL, HUI_COAST, HUI_REACQUIRE,
    HUI_CONFIRM, HUI_COMPLETE, HUI_PROTECT, HUI_STOPPED
} HuiKuStage;

/* blocked is HUI_START_OK, HUI_START_BUSY, or HUI_START_KEY4_HELD. */
uint8_t hui_ku_start(uint8_t source, uint8_t blocked);
/* Main-loop notification for each newly accepted USART6 upper frame. */
void hui_ku_observe_upper(void);
void hui_ku_update(uint8_t hazard);
/* Call only after the existing immediate-stop path has cleared the drive. */
void hui_ku_end(uint8_t source);
uint8_t hui_ku_controls(void);
HuiKuStage hui_ku_stage(void);

#endif

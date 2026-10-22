#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*scheduler_task_func_t)(void);

#define SCHEDULER_MAX_TASKS (16U)  //调度器最大任务数16

void scheduler_init(void);
bool scheduler_add_task(scheduler_task_func_t task_func, uint32_t period_ms, uint8_t priority);
bool scheduler_delete_task(scheduler_task_func_t task_func);
bool scheduler_set_priority(scheduler_task_func_t task_func, uint8_t priority);
void scheduler_run(void);
uint32_t scheduler_get_tick(void); //主动获取时刻
void scheduler_tick(void); //由中断驱动的tick步进
void dl_delay(uint32_t delay_ms); //阻塞式调度器延时

#ifdef __cplusplus
}
#endif

#endif /* __SCHEDULER_H */

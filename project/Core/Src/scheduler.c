#include "scheduler.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"

typedef struct
{
    scheduler_task_func_t task_func;
    uint32_t period_ms;
    uint32_t last_run;
    uint8_t priority;
} scheduler_task_t;

static volatile uint32_t tick;
static scheduler_task_t scheduler_tasks[SCHEDULER_MAX_TASKS];
static uint8_t scheduler_task_count;

void scheduler_init(void)
{
    uint8_t i;

    tick = 0U;
    scheduler_task_count = 0U;
    for (i = 0U; i < SCHEDULER_MAX_TASKS; i++)
    {
        scheduler_tasks[i].task_func = NULL;
        scheduler_tasks[i].period_ms = 0U;
        scheduler_tasks[i].last_run = 0U;
        scheduler_tasks[i].priority = 0U;
    }
}

bool scheduler_set_priority(scheduler_task_func_t task_func, uint8_t priority)
{
    uint8_t i;
    uint8_t pos;
    scheduler_task_t task;

    if ((task_func == NULL) || (priority > 3U))
    {
        return false;
    }
    
    for (i = 0U; i < scheduler_task_count; i++)
    {
        if (scheduler_tasks[i].task_func == task_func)
        {
            if (scheduler_tasks[i].priority == priority)
            {
                return true;
            }

            task = scheduler_tasks[i];
            task.priority = priority;
            pos = i;

            if (priority < scheduler_tasks[i].priority)
            {
                while ((pos > 0U) &&
                       (scheduler_tasks[pos - 1U].priority >= priority))
                {
                    scheduler_tasks[pos] = scheduler_tasks[pos - 1U];
                    pos--;
                }
            }
            else
            {
                while ((pos + 1U < scheduler_task_count) &&
                       (scheduler_tasks[pos + 1U].priority < priority))
                {
                    scheduler_tasks[pos] = scheduler_tasks[pos + 1U];
                    pos++;
                }
            }

            scheduler_tasks[pos] = task;
            return true;
        }
    }

    return false;
}

bool scheduler_add_task(scheduler_task_func_t task_func, uint32_t period_ms, uint8_t priority)
{
    uint8_t i;
    uint8_t pos;

    if ((task_func == NULL) || (period_ms == 0U) || (priority > 3U) ||
        (scheduler_task_count >= SCHEDULER_MAX_TASKS))
    {
        return false;
    }

    pos = 0U;
    while ((pos < scheduler_task_count) &&
           (scheduler_tasks[pos].priority < priority))
    {
        pos++;
    }

    for (i = scheduler_task_count; i > pos; i--)
    {
        scheduler_tasks[i] = scheduler_tasks[i - 1U];
    }

    scheduler_tasks[pos].task_func = task_func;
    scheduler_tasks[pos].period_ms = period_ms;
    scheduler_tasks[pos].last_run = tick;
    scheduler_tasks[pos].priority = priority;
    scheduler_task_count++;
    return true;
}

bool scheduler_delete_task(scheduler_task_func_t task_func)
{
    uint8_t i;
    uint8_t j;

    if (task_func == NULL)
    {
        return false;
    }

    for (i = 0U; i < scheduler_task_count; i++)
    {
        if (scheduler_tasks[i].task_func == task_func)
        {
            for (j = i; j + 1U < scheduler_task_count; j++)
            {
                scheduler_tasks[j] = scheduler_tasks[j + 1U];
            }

            scheduler_task_count--;
            scheduler_tasks[scheduler_task_count].task_func = NULL;
            scheduler_tasks[scheduler_task_count].period_ms = 0U;
            scheduler_tasks[scheduler_task_count].last_run = 0U;
            scheduler_tasks[scheduler_task_count].priority = 0U;
            return true;
        }
    }
    return false;
}

void scheduler_run(void)
{
    uint32_t now = tick;
    uint8_t i;

    for (i = 0U; i < scheduler_task_count; i++)
    {
        if ((uint32_t)(now - scheduler_tasks[i].last_run) >=
            scheduler_tasks[i].period_ms)
        {
            scheduler_tasks[i].last_run = now;
            scheduler_tasks[i].task_func();
        }
    }
}

uint32_t scheduler_get_tick(void)
{
    return tick;
}

void scheduler_tick(void)
{
    tick++;
}

void dl_delay(uint32_t delay_ms)
{
    uint32_t start_tick = tick;

    while ((uint32_t)(tick - start_tick) < delay_ms)
    {
    }
}

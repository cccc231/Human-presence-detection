#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "csi_processor.h"

/* 状态定义 */
typedef enum {
    BED_STATE_INIT_CALIBRATING = 0,
    BED_STATE_EMPTY,
    BED_STATE_MAYBE_LYING,
    BED_STATE_LYING,
    BED_STATE_MAYBE_EMPTY,
    BED_STATE_COOLDOWN,
} bed_state_t;

/* 检测事件 */
typedef enum {
    BED_EVENT_NONE = 0,
    BED_EVENT_LYING_CONFIRMED,
    BED_EVENT_GETUP_CONFIRMED,
} bed_event_t;

/* 快照数据（从 csi_processor 传入） */
typedef struct {
    float amplitudes[MAX_SUBCARRIERS];
    uint16_t num_sub;
    int64_t timestamp;
} bed_snapshot_t;

/* 运行时状态 */
typedef struct {
    bed_state_t state;
    bed_state_t confirmed_state;
    bed_state_t cooldown_target;

    /* EMA */
    float slow_ema;
    float fast_ema;

    /* baseline */
    float baseline_mean;
    float baseline_sub[MAX_SUBCARRIERS];
    uint16_t baseline_sub_count;

    /* direction: -1 或 +1 */
    int direction;

    /* CUSUM */
    float cusum_to_lying;
    float cusum_to_empty;

    /* 计数器 */
    int enter_counter;
    int exit_counter;
    int candidate_timer;
    int cooldown_counter;
    int init_count;

    /* 统计 */
    int lying_confirm_count;
    int getup_confirm_count;
    int maybe_lying_count;
    int maybe_empty_count;
} bed_detector_state_t;

extern QueueHandle_t bed_snapshot_queue;

void bed_detector_init(void);
const char *bed_state_name(bed_state_t s);

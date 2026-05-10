#include "bed_detector.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "bed_det";

QueueHandle_t bed_snapshot_queue = NULL;

/* 默认参数（被 bed_params.h 覆盖） */
#ifndef BED_EMA_ALPHA_SLOW
#define BED_EMA_ALPHA_SLOW         0.010f
#endif
#ifndef BED_EMA_ALPHA_FAST
#define BED_EMA_ALPHA_FAST         0.100f
#endif
#ifndef BED_BASELINE_ALPHA
#define BED_BASELINE_ALPHA         0.0005f
#endif
#ifndef BED_OCC_ENTER_TH
#define BED_OCC_ENTER_TH           0.180f
#endif
#ifndef BED_OCC_EXIT_TH
#define BED_OCC_EXIT_TH            0.100f
#endif
#ifndef BED_CHANGE_TH
#define BED_CHANGE_TH              0.030f
#endif
#ifndef BED_CUSUM_DELTA
#define BED_CUSUM_DELTA            1.500f
#endif
#ifndef BED_CUSUM_H
#define BED_CUSUM_H                12.0f
#endif
#ifndef BED_ENTER_STABLE_COUNT
#define BED_ENTER_STABLE_COUNT     240
#endif
#ifndef BED_EXIT_STABLE_COUNT
#define BED_EXIT_STABLE_COUNT      180
#endif
#ifndef BED_CANDIDATE_TIMEOUT
#define BED_CANDIDATE_TIMEOUT      720
#endif
#ifndef BED_COOLDOWN_COUNT
#define BED_COOLDOWN_COUNT         240
#endif
#ifndef BED_INIT_SAMPLES
#define BED_INIT_SAMPLES           500
#endif
#ifndef BED_MOTION_WINDOW
#define BED_MOTION_WINDOW          100
#endif
#ifndef BED_MOTION_QUIET_TH
#define BED_MOTION_QUIET_TH        5.0f
#endif

/* debug 日志间隔（包数）= 0.5秒 @120Hz */
#define BED_DEBUG_INTERVAL         60

static bed_detector_state_t s_state;

const char *bed_state_name(bed_state_t s)
{
    switch (s) {
        case BED_STATE_INIT_CALIBRATING: return "INIT";
        case BED_STATE_EMPTY:            return "EMPTY";
        case BED_STATE_MAYBE_LYING:      return "MAYBE_LYING";
        case BED_STATE_LYING:            return "LYING";
        case BED_STATE_MAYBE_EMPTY:      return "MAYBE_EMPTY";
        case BED_STATE_COOLDOWN:         return "COOLDOWN";
        default:                         return "UNKNOWN";
    }
}

static float compute_mean_amp(const bed_snapshot_t *snap)
{
    float sum = 0;
    for (int i = 0; i < snap->num_sub; i++) {
        sum += snap->amplitudes[i];
    }
    return sum / snap->num_sub;
}

/* 滑动窗口方差（简化: 用最近 WINDOW 个 mean_amp 的方差） */
static float s_motion_buf[BED_MOTION_WINDOW];
static int s_motion_idx = 0;
static int s_motion_count = 0;

static float update_motion_score(float mean_amp)
{
    s_motion_buf[s_motion_idx] = mean_amp;
    s_motion_idx = (s_motion_idx + 1) % BED_MOTION_WINDOW;
    if (s_motion_count < BED_MOTION_WINDOW) {
        s_motion_count++;
        return 0.0f;
    }

    float sum = 0, sum2 = 0;
    for (int i = 0; i < BED_MOTION_WINDOW; i++) {
        sum += s_motion_buf[i];
        sum2 += s_motion_buf[i] * s_motion_buf[i];
    }
    float n = (float)BED_MOTION_WINDOW;
    float mean = sum / n;
    return (sum2 / n) - (mean * mean);
}

static void emit_event(bed_event_t event)
{
    if (event == BED_EVENT_LYING_CONFIRMED) {
        s_state.lying_confirm_count++;
        ESP_LOGI(TAG, "========== BED: LYING_CONFIRMED (#%d) ==========",
                 s_state.lying_confirm_count);
    } else if (event == BED_EVENT_GETUP_CONFIRMED) {
        s_state.getup_confirm_count++;
        ESP_LOGI(TAG, "========== BED: GETUP_CONFIRMED (#%d) ==========",
                 s_state.getup_confirm_count);
    }
}

static void bed_process_packet(const bed_snapshot_t *snap)
{
    float mean_amp = compute_mean_amp(snap);
    bed_detector_state_t *s = &s_state;

    /* --- 滤波 --- */
    s->slow_ema = BED_EMA_ALPHA_SLOW * mean_amp
                  + (1.0f - BED_EMA_ALPHA_SLOW) * s->slow_ema;
    s->fast_ema = BED_EMA_ALPHA_FAST * mean_amp
                  + (1.0f - BED_EMA_ALPHA_FAST) * s->fast_ema;

    /* --- 特征 --- */
    float occupancy_score = 0.0f;
    float signed_innovation = 0.0f;
    float change_score = 0.0f;

    if (s->baseline_mean > 0) {
        float raw_occ = (s->slow_ema - s->baseline_mean) / s->baseline_mean;
        if (s->direction != 0) {
            occupancy_score = s->direction * raw_occ;
        } else {
            occupancy_score = fabsf(raw_occ);
        }

        float innovation = mean_amp - s->slow_ema;
        if (s->direction != 0) {
            signed_innovation = s->direction * innovation;
        } else {
            signed_innovation = innovation;
        }

        float fast_slow_diff = s->fast_ema - s->slow_ema;
        if (s->direction != 0) {
            change_score = s->direction * fast_slow_diff / s->baseline_mean;
        }
    }

    /* CUSUM */
    s->cusum_to_lying = fmaxf(0.0f,
        s->cusum_to_lying + signed_innovation - BED_CUSUM_DELTA);
    s->cusum_to_empty = fmaxf(0.0f,
        s->cusum_to_empty - signed_innovation - BED_CUSUM_DELTA);

    /* motion */
    float motion_score = update_motion_score(mean_amp);

    /* --- 候选条件 --- */
    bool lying_candidate = (occupancy_score > BED_OCC_ENTER_TH)
                        || (s->cusum_to_lying > BED_CUSUM_H)
                        || (change_score > BED_CHANGE_TH);

    bool empty_candidate = (occupancy_score < BED_OCC_EXIT_TH)
                        || (s->cusum_to_empty > BED_CUSUM_H)
                        || (change_score < -BED_CHANGE_TH);

    bool quiet = (motion_score < BED_MOTION_QUIET_TH);

    /* --- 状态机 --- */
    switch (s->state) {

    case BED_STATE_INIT_CALIBRATING:
        s->init_count++;
        if (s->init_count == 1) {
            s->slow_ema = mean_amp;
            s->fast_ema = mean_amp;
            s->baseline_mean = mean_amp;
        } else {
            /* 累加 baseline */
            s->baseline_mean = s->baseline_mean
                + (mean_amp - s->baseline_mean) / s->init_count;
        }
        /* 累加子载波 baseline */
        if (s->init_count == 1) {
            memcpy(s->baseline_sub, snap->amplitudes,
                   snap->num_sub * sizeof(float));
            s->baseline_sub_count = snap->num_sub;
        } else {
            float alpha = 1.0f / s->init_count;
            for (int i = 0; i < snap->num_sub; i++) {
                s->baseline_sub[i] += alpha * (snap->amplitudes[i] - s->baseline_sub[i]);
            }
        }
        if (s->init_count >= BED_INIT_SAMPLES) {
            ESP_LOGI(TAG, "Calibration done: baseline=%.2f, std est needed",
                     s->baseline_mean);
            s->state = BED_STATE_EMPTY;
            s->direction = 0;  /* 未确定，等第一次确认时确定 */
        }
        break;

    case BED_STATE_EMPTY:
        /* 慢速更新 baseline */
        if (occupancy_score < BED_OCC_EXIT_TH && quiet
            && s->cusum_to_lying < BED_CUSUM_H * 0.5f) {
            s->baseline_mean = s->baseline_mean * (1.0f - BED_BASELINE_ALPHA)
                             + mean_amp * BED_BASELINE_ALPHA;
        }

        if (lying_candidate) {
            s->state = BED_STATE_MAYBE_LYING;
            s->enter_counter = 0;
            s->candidate_timer = 0;
            s->maybe_lying_count++;
            ESP_LOGI(TAG, "EMPTY -> MAYBE_LYING (occ=%.3f, cusum_L=%.1f)",
                     occupancy_score, s->cusum_to_lying);
        }
        break;

    case BED_STATE_MAYBE_LYING:
        s->candidate_timer++;
        if (occupancy_score > BED_OCC_ENTER_TH) {
            s->enter_counter++;
        } else {
            s->enter_counter = (s->enter_counter > 0) ? s->enter_counter - 1 : 0;
        }

        if (s->enter_counter >= BED_ENTER_STABLE_COUNT) {
            /* 确认躺下 */
            s->confirmed_state = BED_STATE_LYING;
            if (s->direction == 0) {
                s->direction = (s->slow_ema < s->baseline_mean) ? -1 : 1;
                ESP_LOGI(TAG, "Direction determined: %d", s->direction);
            }
            emit_event(BED_EVENT_LYING_CONFIRMED);
            s->state = BED_STATE_COOLDOWN;
            s->cooldown_target = BED_STATE_LYING;
            s->cooldown_counter = BED_COOLDOWN_COUNT;
            s->cusum_to_lying = 0;

        } else if (occupancy_score < BED_OCC_EXIT_TH) {
            s->state = BED_STATE_EMPTY;
            ESP_LOGI(TAG, "MAYBE_LYING -> EMPTY (rejected, occ=%.3f)",
                     occupancy_score);

        } else if (s->candidate_timer > BED_CANDIDATE_TIMEOUT) {
            s->state = BED_STATE_EMPTY;
            ESP_LOGI(TAG, "MAYBE_LYING -> EMPTY (timeout)");
        }
        break;

    case BED_STATE_LYING:
        if (empty_candidate) {
            s->state = BED_STATE_MAYBE_EMPTY;
            s->exit_counter = 0;
            s->candidate_timer = 0;
            s->maybe_empty_count++;
            ESP_LOGI(TAG, "LYING -> MAYBE_EMPTY (occ=%.3f, cusum_E=%.1f)",
                     occupancy_score, s->cusum_to_empty);
        }
        break;

    case BED_STATE_MAYBE_EMPTY:
        s->candidate_timer++;
        if (occupancy_score < BED_OCC_EXIT_TH) {
            s->exit_counter++;
        } else {
            s->exit_counter = (s->exit_counter > 0) ? s->exit_counter - 1 : 0;
        }

        if (s->exit_counter >= BED_EXIT_STABLE_COUNT) {
            /* 确认起床 */
            s->confirmed_state = BED_STATE_EMPTY;
            emit_event(BED_EVENT_GETUP_CONFIRMED);
            s->state = BED_STATE_COOLDOWN;
            s->cooldown_target = BED_STATE_EMPTY;
            s->cooldown_counter = BED_COOLDOWN_COUNT;
            s->cusum_to_empty = 0;

        } else if (occupancy_score > BED_OCC_ENTER_TH) {
            s->state = BED_STATE_LYING;
            ESP_LOGI(TAG, "MAYBE_EMPTY -> LYING (rejected, occ=%.3f)",
                     occupancy_score);

        } else if (s->candidate_timer > BED_CANDIDATE_TIMEOUT) {
            s->state = BED_STATE_LYING;
            ESP_LOGI(TAG, "MAYBE_EMPTY -> LYING (timeout)");
        }
        break;

    case BED_STATE_COOLDOWN:
        s->cooldown_counter--;
        if (s->cooldown_counter <= 0) {
            s->state = s->cooldown_target;
            ESP_LOGI(TAG, "COOLDOWN -> %s", bed_state_name(s->state));
        }
        break;
    }
}

static void bed_detector_task(void *arg)
{
    bed_snapshot_t snap;
    int debug_count = 0;

    ESP_LOGI(TAG, "Bed detector task started");
    ESP_LOGI(TAG, "  EMA slow=%.4f fast=%.4f", BED_EMA_ALPHA_SLOW, BED_EMA_ALPHA_FAST);
    ESP_LOGI(TAG, "  OCC enter=%.4f exit=%.4f", BED_OCC_ENTER_TH, BED_OCC_EXIT_TH);
    ESP_LOGI(TAG, "  CUSUM delta=%.4f H=%.1f", BED_CUSUM_DELTA, BED_CUSUM_H);
    ESP_LOGI(TAG, "  Stable enter=%d exit=%d", BED_ENTER_STABLE_COUNT, BED_EXIT_STABLE_COUNT);

    while (1) {
        if (xQueueReceive(bed_snapshot_queue, &snap, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        bed_process_packet(&snap);

        /* debug 日志: 每 BED_DEBUG_INTERVAL 包输出一次 */
        debug_count++;
        if (debug_count >= BED_DEBUG_INTERVAL) {
            debug_count = 0;
            float mean_amp = compute_mean_amp(&snap);
            float occ = 0;
            if (s_state.baseline_mean > 0 && s_state.direction != 0) {
                float raw = (s_state.slow_ema - s_state.baseline_mean) / s_state.baseline_mean;
                occ = s_state.direction * raw;
            }
            ESP_LOGI(TAG, "[%s] amp=%.1f slow=%.1f base=%.1f occ=%.3f "
                     "cusum_L=%.1f E=%.1f ent=%d ext=%d",
                     bed_state_name(s_state.state),
                     mean_amp, s_state.slow_ema, s_state.baseline_mean,
                     occ, s_state.cusum_to_lying, s_state.cusum_to_empty,
                     s_state.enter_counter, s_state.exit_counter);
        }
    }
}

void bed_detector_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_state.state = BED_STATE_INIT_CALIBRATING;
    s_state.confirmed_state = BED_STATE_EMPTY;

    bed_snapshot_queue = xQueueCreate(CSI_QUEUE_SIZE, sizeof(bed_snapshot_t));
    configASSERT(bed_snapshot_queue);

    xTaskCreatePinnedToCore(bed_detector_task, "bed_det", 8192, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Bed detector initialized (state machine mode)");
}

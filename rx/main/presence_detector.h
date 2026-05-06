#pragma once

#include "csi_processor.h"

#define WINDOW_SIZE         100
#define INIT_SAMPLES        500
#define THRESHOLD_HIGH      1.5f
#define THRESHOLD_LOW       1.2f
#define AGG_WINDOW_MS       1000
#define PRESENCE_RATIO      0.4f

typedef enum {
    PRESENCE_UNKNOWN = -1,
    PRESENCE_EMPTY = 0,
    PRESENCE_DETECTED = 1,
} presence_state_t;

void presence_detector_init(void);

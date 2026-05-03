#pragma once

#include "csi_processor.h"

#define WINDOW_SIZE         100
#define THRESHOLD_HIGH      1.5f
#define THRESHOLD_LOW       1.2f
#define DEBOUNCE_COUNT      3

typedef enum {
    PRESENCE_UNKNOWN = -1,
    PRESENCE_EMPTY = 0,
    PRESENCE_DETECTED = 1,
} presence_state_t;

void presence_detector_init(void);

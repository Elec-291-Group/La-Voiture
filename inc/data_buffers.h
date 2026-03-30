#ifndef DATA_BUFFERS_H
#define DATA_BUFFERS_H

#include "ir_rx.h"

#define PATH_MAX_WAYPOINTS  32
#define IMU_HISTORY_SIZE    32

typedef struct {
    int x_cm;
    int y_cm;
} Waypoint_t;

typedef struct {
    unsigned char reg_index;
    unsigned int value;
    unsigned char addr;
} IMU_Sample_t;

void PathBuffer_reset(void);
unsigned char PathBuffer_begin(unsigned char expected_count);
unsigned char PathBuffer_store(unsigned char index, int x_cm, int y_cm);
unsigned char PathBuffer_commit(void);
unsigned char PathBuffer_is_loaded(void);
unsigned char PathBuffer_get_count(void);
unsigned char PathBuffer_get(unsigned char index, Waypoint_t *out);

void IMUBuffer_reset(void);
unsigned char IMUBuffer_push_frame(const IR_Frame_t *frame);

#endif

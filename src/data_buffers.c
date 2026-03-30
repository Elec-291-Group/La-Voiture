#include "data_buffers.h"
#include "config.h"

#define IMU_HISTORY_MASK  (IMU_HISTORY_SIZE - 1)


static volatile Waypoint_t xdata path_active[PATH_MAX_WAYPOINTS];
static volatile Waypoint_t xdata path_staging[PATH_MAX_WAYPOINTS];
static volatile unsigned char path_active_count = 0;
static volatile unsigned char path_expected_count = 0;
static volatile unsigned char path_write_mask[(PATH_MAX_WAYPOINTS + 7) / 8];
static volatile bit path_loaded = 0;
static volatile bit path_receiving = 0;

static IMU_Sample_t xdata imu_history[IMU_HISTORY_SIZE];
static volatile unsigned char imu_head = 0;
static unsigned char imu_tail = 0;

static void PathBuffer_clear_write_mask(void)
{
    unsigned char i;
    for (i = 0; i < sizeof(path_write_mask); i++)
        path_write_mask[i] = 0;
}

static void PathBuffer_mark_written(unsigned char index)
{
    path_write_mask[index >> 3] |= (1u << (index & 0x07u));
}

static unsigned char PathBuffer_is_written(unsigned char index)
{
    return (path_write_mask[index >> 3] & (1u << (index & 0x07u))) ? 1u : 0u;
}

void PathBuffer_reset(void)
{
    path_active_count = 0;
    path_expected_count = 0;
    path_loaded = 0;
    path_receiving = 0;
    PathBuffer_clear_write_mask();
}

unsigned char PathBuffer_begin(unsigned char expected_count)
{
    if ((expected_count == 0) || (expected_count > PATH_MAX_WAYPOINTS))
        return 0;

    path_expected_count = expected_count;
    path_receiving = 1;
    path_loaded = 0;
    PathBuffer_clear_write_mask();
    return 1;
}

unsigned char PathBuffer_store(unsigned char index, int x_cm, int y_cm)
{
    if (!path_receiving)
        return 0;
    if (index >= path_expected_count)
        return 0;

    path_staging[index].x_cm = x_cm;
    path_staging[index].y_cm = y_cm;
    PathBuffer_mark_written(index);
    return 1;
}

unsigned char PathBuffer_commit(void)
{
    unsigned char i;

    if (!path_receiving)
        return 0;

    for (i = 0; i < path_expected_count; i++) {
        if (!PathBuffer_is_written(i))
            return 0;
    }

    for (i = 0; i < path_expected_count; i++)
    {
        path_active[i].x_cm = path_staging[i].x_cm;
        path_active[i].y_cm = path_staging[i].y_cm;
    }

    path_active_count = path_expected_count;
    path_receiving = 0;
    path_loaded = 1;
    return 1;
}

unsigned char PathBuffer_is_loaded(void)
{
    return path_loaded ? 1u : 0u;
}

unsigned char PathBuffer_get_count(void)
{
    return path_active_count;
}

unsigned char PathBuffer_get(unsigned char index, Waypoint_t *out)
{
    if ((index >= path_active_count) || (out == 0))
        return 0;

    out->x_cm = path_active[index].x_cm;
    out->y_cm = path_active[index].y_cm;
    return 1;
}

void IMUBuffer_reset(void)
{
    imu_head = 0;
    imu_tail = 0;
}

unsigned char IMUBuffer_push_frame(const IR_Frame_t *frame)
{
    unsigned char next;

    if (frame == 0)
        return 0;
    if ((frame->cmd < IMU_CMD_BASE) || (frame->cmd >= IMU_CMD_BASE + IMU_REG_COUNT))
        return 0;

    imu_history[imu_head].reg_index = (unsigned char)(frame->cmd - IMU_CMD_BASE);
    imu_history[imu_head].value = frame->val;
    imu_history[imu_head].addr = frame->addr;
    next = (imu_head + 1) & IMU_HISTORY_MASK;
    imu_head = next;
    if (imu_head == imu_tail)
        imu_tail = (imu_tail + 1) & IMU_HISTORY_MASK;

    return 1;
}

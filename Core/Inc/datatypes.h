// IMU Registers
#define LSM6DS33_ADDR_HIGH        0x6B
#define LSM6DS33_ADDR_LOW         0x6A

#define LSM6DS33_WHO_AM_I_REG     0x0F
#define LSM6DS33_CTRL1_XL         0x10
#define LSM6DS33_CTRL2_G          0x11
#define LSM6DS33_CTRL3_C          0x12

#define LSM6DS33_OUTX_L_G         0x22
#define LSM6DS33_OUTX_L_XL        0x28

#define LSM6DS33_WHO_AM_I_VAL     0x69

// IMU Values
#define GYRO_SENS_DPS_PER_LSB     0.00875f

#define GYRO_CAL_SAMPLES          200
#define RAD_TO_DEG                57.2957795f

// Vehicle Control 
// needed to be tuned !!!
#define KP 0.022f // how much the car turns to fix itself ======= used to be 0.015 ======
#define KS 0.02f // how much the car reduces speed when turning
#define KF 0.020f // decrease speed before approaching the intersection

#define PB 95 // base power in field tracking mode. ======= used to be 75 ========
#define STOP_VF 1300
#define LEFT_TURN_DEGREE -80
#define RIGHT_TURN_DEGREE 86
#define TURNING_SPEED 100 // speed when turning at intersections. ====== used to be 65 ======
#define INTERSECTION_COMPENSATION_TIME 300
#define INTERSECTION_TURN_SETTLE_TIME 200
#define INTERSECTION_REARM_TIME_MS 1200

//For sensor
#define VL53L0X_I2C_ADDR 0x52 

// ms it takes for the car to rotate one full circle at 60% power
#define IMU_UPDATE_PERIOD_MS      20u
#define POSE_STREAM_PERIOD_MS     100u
#define PATH_MAX_WAYPOINTS        32u
#define PATH_REACHED_TOLERANCE_CM 8.0f
#define DRIVE_SPEED_SCALE_CM_S    0.1f
#define PATH_FORWARD_SPEED        60
#define PATH_APPROACH_SPEED       35
#define PATH_STEER_GAIN           1.0f
#define PATH_MAX_STEER_CMD        55
#define MOTOR_TEST_SPEED          40
#define IR_JOYSTICK_CENTER_X      165u
#define IR_JOYSTICK_CENTER_Y      170u
#define GUIDEWIRE_FRONT_THRESHOLD_MV    1000u
#define GUIDEWIRE_BALANCE_TOLERANCE_MV   150u
#define GUIDEWIRE_LOCK_SAMPLES_REQUIRED    5u
#define GUIDEWIRE_SAMPLE_PERIOD_MS        20u
#define PATH_CMD_BASE                      7u  /* cmd = waypoint_index + PATH_CMD_BASE (7..38) */
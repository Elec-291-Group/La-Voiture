#include <string.h>
#include "bluetooth.h"
#include "config.h"
#include "uart.h"
#include "ir_rx.h"
#include "lcd_fsm.h"
#include "data_buffers.h"
#include "timer.h"

// ---- Internal state ----
static bit bluetooth_stream_enabled = 0;
static char xdata bluetooth_cmd_buf[24];
static char xdata bluetooth_parse_buf[24];
static IR_Frame_t xdata bluetooth_rx_frame;
static char xdata uart_digits[5];

static int parse_signed_int(const char *text, unsigned char *ok)
{
	int sign = 1;
	int value = 0;
	char c;

	*ok = 0;
	if ((text == 0) || (*text == '\0'))
		return 0;

	if (*text == '-') {
		sign = -1;
		text++;
	} else if (*text == '+') {
		text++;
	}

	if (*text == '\0')
		return 0;

	while ((c = *text++) != '\0') {
		if ((c < '0') || (c > '9'))
			return 0;
		value = (value * 10) + (c - '0');
	}

	*ok = 1;
	return sign * value;
}

static void Bluetooth_send_rx_cmd_echo(const char *cmd_text)
{
	UART1_send_string("RX_CMD,");
	UART1_send_string(cmd_text);
	UART1_send_string("\r\n");
}

static void UART1_send_uint(unsigned int value)
{
	unsigned char count = 0;

	if (value == 0) {
		UART1_send_char('0');
		return;
	}

	while ((value > 0) && (count < sizeof(uart_digits))) {
		uart_digits[count++] = (char)('0' + (value % 10u));
		value /= 10u;
	}

	while (count > 0)
		UART1_send_char(uart_digits[--count]);
}

static void Bluetooth_stream_imu_frame(const IR_Frame_t *frame)
{
	UART1_send_string("imu,");
	UART1_send_uint((unsigned int)(frame->cmd - IMU_CMD_BASE));
	UART1_send_char(',');
	UART1_send_uint(frame->val);
	UART1_send_string("\r\n");
}

extern signed char left_power;
extern signed char right_power;

static void Bluetooth_stream_power(unsigned char which, signed char power)
{
	UART1_send_string("pwr,");
	UART1_send_char(which ? '1' : '0');
	UART1_send_char(',');
	if (power < 0) {
		UART1_send_char('-');
		UART1_send_uint((unsigned int)(unsigned char)(-power));
	} else {
		UART1_send_uint((unsigned int)(unsigned char)power);
	}
	UART1_send_string("\r\n");
}

static void Bluetooth_handle_path_command(const char *cmd_text)
{
	char *path_token;
	char *path_arg1;
	char *path_arg2;
	char *path_arg3;
	unsigned char path_ok1;
	unsigned char path_ok2;
	unsigned char path_ok3;
	int path_value1;
	int path_value2;
	int path_value3;

	strcpy(bluetooth_parse_buf, cmd_text);
	path_token = strtok(bluetooth_parse_buf, ",");
	if (path_token == 0)
		return;

	if (strcmp(path_token, "PATH_BEGIN") == 0)
	{
		path_arg1 = strtok(0, ",");
		if (path_arg1 == 0)
		{
			UART1_send_string("PATH_ACK,ERROR\r\n");
			return;
		}

		path_value1 = parse_signed_int(path_arg1, &path_ok1);
		if (!path_ok1 || !PathBuffer_begin((unsigned char)path_value1))
		{
			UART1_send_string("PATH_ACK,ERROR\r\n");
			return;
		}

		UART1_send_string("PATH_ACK,BEGIN\r\n");
		return;
	}

	if (strcmp(path_token, "WPT") == 0)
	{
		path_arg1 = strtok(0, ",");
		path_arg2 = strtok(0, ",");
		path_arg3 = strtok(0, ",");
		if ((path_arg1 == 0) || (path_arg2 == 0) || (path_arg3 == 0))
			return;

		path_value1 = parse_signed_int(path_arg1, &path_ok1);
		path_value2 = parse_signed_int(path_arg2, &path_ok2);
		path_value3 = parse_signed_int(path_arg3, &path_ok3);
		if (!path_ok1 || !path_ok2 || !path_ok3)
			return;

		if (PathBuffer_store((unsigned char)path_value1, path_value2, path_value3))
			Bluetooth_send_rx_cmd_echo(cmd_text);
		return;
	}

	if (strcmp(path_token, "PATH_END") == 0)
	{
		if (PathBuffer_commit()) {
			UART1_send_string("PATH_ACK,LOADED\r\n");
			lcd_state = LCD_S15;
		} else {
			UART1_send_string("PATH_ACK,ERROR\r\n");
		}
	}
}

/* Send left+right power over BLE every 100 ms for the given duration. */
#define DBG_STEP_MS 100u
static void debug_send_power(signed char lp, signed char rp, unsigned int duration_ms)
{
    unsigned int elapsed;
    left_power  = lp;
    right_power = rp;
    for (elapsed = 0; elapsed < duration_ms; elapsed += DBG_STEP_MS) {
        Bluetooth_stream_power(0, lp);
        Bluetooth_stream_power(1, rp);
        waitms(DBG_STEP_MS);
    }
}

/*
 * Bluetooth_debug_drive_sequence
 * Simulates over BLE:
 *   1. Straight forward at 80% for 2 s
 *   2. Spin left  90° at 60% (~521 ms)
 *   3. Spin right 90° at 60% (~521 ms)
 *   4. Stop
 *
 * Timing derivation (Solarbotics GM4, wheelbase=10.6 cm, r=3.3 cm):
 *   max_speed = 77 RPM * 2π * 3.3 / 60 ≈ 26.6 cm/s
 *   ω at 60%  = 2 * (0.6 * 26.6) / 10.6 ≈ 3.01 rad/s
 *   t_90°     = (π/2) / 3.01 ≈ 521 ms
 */
void Bluetooth_debug_drive_sequence(void)
{
    debug_send_power( 80,  80, 2000u); /* straight forward */
    debug_send_power(-60,  60,  521u); /* spin left  90°   */
    debug_send_power( 60, -60,  521u); /* spin right 90°   */
    debug_send_power(  0,   0,  200u); /* stop             */
}

void Bluetooth_handle_commands(void)
{
    static unsigned char cmd_len = 0;

    while (UART1_available())
    {
        char c = UART1_read();

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            bluetooth_cmd_buf[cmd_len] = '\0';

            if (strcmp(bluetooth_cmd_buf, "STREAM_ON") == 0)
            {
                bluetooth_stream_enabled = 1;
                UART1_send_string("OK,STREAM_ON\r\n");
            }
            else if (strcmp(bluetooth_cmd_buf, "STREAM_OFF") == 0)
            {
                bluetooth_stream_enabled = 0;
                UART1_send_string("OK,STREAM_OFF\r\n");
            }
            else if (strcmp(bluetooth_cmd_buf, "STATUS") == 0)
            {
                UART1_send_string(bluetooth_stream_enabled ? "STATUS,STREAM_ON\r\n"
                                                           : "STATUS,STREAM_OFF\r\n");
            }
            else if ((strncmp(bluetooth_cmd_buf, "PATH_BEGIN,", 11) == 0) ||
                     (strncmp(bluetooth_cmd_buf, "WPT,", 4) == 0) ||
                     (strcmp(bluetooth_cmd_buf, "PATH_END") == 0))
            {
                Bluetooth_handle_path_command(bluetooth_cmd_buf);
            }
            else if (cmd_len > 0)
            {
                UART1_send_string("ERR,UNKNOWN_CMD\r\n");
            }

            cmd_len = 0;
            continue;
        }

        if (cmd_len < (sizeof(bluetooth_cmd_buf) - 1))
            bluetooth_cmd_buf[cmd_len++] = c;
        else
            cmd_len = 0;
    }
}

/* Timer3 runs at 6 MHz (16-bit, overflows every ~10.923 ms).
 * 46 overflows ≈ 503 ms — close enough to 500 ms. */
#define IMU_STREAM_OVF_PERIOD  46u

void Bluetooth_forward_imu(void)
{
    static unsigned int  imu_last_t3  = 0;
    static unsigned char imu_ovf_cnt  = 0;
    unsigned int  t3_now;
    unsigned char ri;
    IR_Frame_t    periodic_frame;

    /* Drain incoming IR frames */
    while (IR_RX_get(&bluetooth_rx_frame))
    {
        if (bluetooth_rx_frame.addr != IR_ADDR2)
            continue;

        IR_RX_decode_command(&bluetooth_rx_frame);
        IMUBuffer_push_frame(&bluetooth_rx_frame);
    }

    /* Count Timer3 overflows for a non-blocking 500 ms tick */
    t3_now = IR_RX_read_t3();
    if (t3_now < imu_last_t3)
        imu_ovf_cnt++;
    imu_last_t3 = t3_now;

    if (imu_ovf_cnt < IMU_STREAM_OVF_PERIOD)
        return;
    imu_ovf_cnt = 0;

    /* Send all imu_regs[] over BLE every ~500 ms */
    if (!bluetooth_stream_enabled)
        return;

    for (ri = 0; ri < IMU_REG_COUNT; ri++)
    {
        periodic_frame.cmd  = (unsigned char)(IMU_CMD_BASE + ri);
        periodic_frame.val  = imu_regs[ri];
        periodic_frame.addr = IR_ADDR2;
        Bluetooth_stream_imu_frame(&periodic_frame);
    }
    Bluetooth_stream_power(0, left_power);
    Bluetooth_stream_power(1, right_power);
}

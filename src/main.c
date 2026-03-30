#include <stdlib.h>
#include <EFM8LB1.h>
#include "bootloader.h"
#include "timer.h"
#include "ir_tx.h"
#include "ir_rx.h"
#include "config.h"
#include "uart.h"
#include "lcd.h"
#include "lcd_fsm.h"
#include "data_buffers.h"
#include "bluetooth.h"

unsigned char crossing_action   = 0;
static unsigned char intersection_num = 0;
static bit           crossing_updated = 0;
signed char left_power  = 0;
signed char right_power = 0;

void InitPinADC (unsigned char portno, unsigned char pinno)
{
	unsigned char mask;
	
	mask=1<<pinno;

	SFRPAGE = 0x00;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
	 return ((ADC_at_Pin(pin)*VDD)/0b_0011_1111_1111_1111);
}

// Map a voltage (0 to VDD) to a byte (0 to 255)
unsigned char volt_to_byte(float v)
{
	int val = (int)(v / VDD * 255.0f);
	if (val < 0)   val = 0;
	if (val > 255) val = 255;
	return (unsigned char)val;
}

static void uart0_send_hex_byte(unsigned char v)
{
    unsigned char n;
    n = (v >> 4) & 0x0Fu;
    UART0_send_char((char)(n < 10u ? '0' + n : 'A' + n - 10u));
    n = v & 0x0Fu;
    UART0_send_char((char)(n < 10u ? '0' + n : 'A' + n - 10u));
}

static void uart0_send_uint(unsigned int v)
{
    char buf[5];
    unsigned char i = 0;

    if (v == 0u) {
        UART0_send_char('0');
        return;
    }

    while ((v > 0u) && (i < sizeof(buf))) {
        buf[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    }

    while (i > 0u) {
        UART0_send_char(buf[--i]);
    }
}

static void IR_TX_debug_print(unsigned char cmd, unsigned int val)
{
    UART0_send_string("TX cmd=");
    uart0_send_hex_byte(cmd);
    UART0_send_string(" val=0x");
    uart0_send_hex_byte((unsigned char)(val >> 8));
    uart0_send_hex_byte((unsigned char)(val & 0xFFu));
    UART0_send_string("\r\n");
}

void IR_debug(void)
{
    IR_Frame_t f;
    send_ir_packet((uint8_t)IR_CMD_START, (uint16_t)0x0000, (uint8_t)IR_ADDR1);
    while (fsm_state == FSM_IDLE);
    while (fsm_state != FSM_IDLE);
    IR_TX_debug_print(IR_CMD_START, 0x0000);
    while (IR_RX_get(&f)) {
        if (f.addr == IR_ADDR2) {
            UART0_send_string("cmd=");
            uart0_send_hex_byte(f.cmd);
            UART0_send_string(" val=0x");
            uart0_send_hex_byte((unsigned char)(f.val >> 8));
            uart0_send_hex_byte((unsigned char)(f.val & 0xFFu));
            UART0_send_string(" addr=0x");
            uart0_send_hex_byte(f.addr);
            UART0_send_string("\r\n");
        }
    }
}

static void IR_RX_debug_print(const IR_Frame_t *frame)
{
    UART0_send_string("RX cmd=");
    uart0_send_hex_byte(frame->cmd);
    UART0_send_string(" val=0x");
    uart0_send_hex_byte((unsigned char)(frame->val >> 8));
    uart0_send_hex_byte((unsigned char)(frame->val & 0xFFu));
    UART0_send_string(" addr=0x");
    uart0_send_hex_byte(frame->addr);
    UART0_send_string("\r\n");
}

static IR_Frame_t xdata dbg_frame;
static bit dbg_frame_ready = 0;

void IR_RX_decode_command(const IR_Frame_t *frame)
{
    dbg_frame.cmd  = frame->cmd;
    dbg_frame.val  = frame->val;
    dbg_frame.addr = frame->addr;
    dbg_frame_ready = 1;

    switch (frame->cmd)
    {
        case IR_RX_CMD_DATA_RECEIVED:
            break;

        case IR_RX_CMD_CROSSING_ACTION:
            crossing_action = (unsigned char)(frame->val & 0xFF);
            if (lcd_state == LCD_S5) {
                intersection_num++;
                crossing_updated = 1;
            }
            break;

        case IR_RX_CMD_LEFT_POWER:
        {
            unsigned char byte1 = (frame->val >> 8) & 0xFF;
            unsigned char byte2 = frame->val & 0xFF;
            left_power = (byte1 > 0) ? (signed char)byte1 : -((signed char)byte2);
            break;
        }

        case IR_RX_CMD_RIGHT_POWER:
        {
            unsigned char byte1 = (frame->val >> 8) & 0xFF;
            unsigned char byte2 = frame->val & 0xFF;
            right_power = (byte1 > 0) ? (signed char)byte1 : -((signed char)byte2);
            break;
        }

        default:
            break;
    }
}

static bit pb_start_latch    = 1;
static bit pb_pause_latch    = 1;
static bit pb_reset_latch    = 1;
static bit pbtxcmd_s13_latch = 1;
static bit pbstart_s13_latch = 1;
static bit pbtxcmd_s14_latch = 1;
static bit pbstart_s14_latch = 1;
static bit pbtxcmd_s15_latch = 1;
static bit pbstart_s15_latch = 1;
static bit pbsw_s17_latch    = 1;
static bit pbtxcmd_s17_latch = 1;
static bit pb0_s7_latch      = 1;
static bit pb0_s8_latch      = 1;
static lcd_state_t s7_resume_state = LCD_S0; // which running state S7 should resume to

static void handle_pb_start(void)
{
    if (PB_START == 0) {
        if (pb_start_latch && fsm_state == FSM_IDLE) {
            send_ir_packet(IR_CMD_START, 0x0000, IR_ADDR);
            while (fsm_state == FSM_IDLE);
            while (fsm_state != FSM_IDLE);
            IR_TX_debug_print(IR_CMD_START, 0x0000);
            pb_start_latch = 0;
        }
    } else {
        pb_start_latch = 1;
    }
}

static void handle_pb_pause(void)
{
    if (PB_PAUSE == 0) {
        if (pb_pause_latch && fsm_state == FSM_IDLE) {
            send_ir_packet(IR_CMD_PAUSE, 0x0000, IR_ADDR);
            while (fsm_state == FSM_IDLE);
            while (fsm_state != FSM_IDLE);
            IR_TX_debug_print(IR_CMD_PAUSE, 0x0000);
            if (lcd_state == LCD_S5  || lcd_state == LCD_S6 ||
                lcd_state == LCD_S9  || lcd_state == LCD_S11) {
                s7_resume_state = lcd_state;
                LCD_FSM_pause(lcd_state);
            }
            pb_pause_latch = 0;
        }
    } else {
        pb_pause_latch = 1;
    }
}

static void handle_pb_reset(void)
{
    if (PB_RESET == 0) {
        if (pb_reset_latch && fsm_state == FSM_IDLE) {
            send_ir_packet(IR_CMD_RESET, 0x0000, IR_ADDR);
            while (fsm_state == FSM_IDLE);
            while (fsm_state != FSM_IDLE);
            IR_TX_debug_print(IR_CMD_RESET, 0x0000);
            lcd_state = LCD_S0;
            pb_reset_latch = 0;
        }
    } else {
        pb_reset_latch = 1;
    }
}

static void send_path_waypoints(void)
{
    unsigned char count = PathBuffer_get_count();
    unsigned char i;
    Waypoint_t wpt;

    for (i = 0; i < count; i++) {
        if (!PathBuffer_get(i, &wpt))
            continue;
        send_ir_packet(
            (uint8_t)(IR_CMD_PATH_WPT_BASE + i),
            (uint16_t)(((uint16_t)(wpt.x_cm & 0xFF) << 8) | (uint16_t)(wpt.y_cm & 0xFF)),
            IR_ADDR
        );
        while (fsm_state == FSM_IDLE);
        while (fsm_state != FSM_IDLE);
    }
}

static void handle_s13_buttons(void)
{
    if (lcd_state != LCD_S13) { pbtxcmd_s13_latch = 1; pbstart_s13_latch = 1; return; }

    /* PB_TXCMD: send MODE + PATH to car */
    if (PB_TXCMD == 0) {
        if (pbtxcmd_s13_latch && fsm_state == FSM_IDLE) {
            send_ir_packet(IR_CMD_MODE, active_mode, IR_ADDR);
            IR_TX_debug_print(IR_CMD_MODE, active_mode);
            while (fsm_state == FSM_IDLE);
            while (fsm_state != FSM_IDLE);
            send_ir_packet(IR_CMD_PATH, active_path + 1, IR_ADDR);
            IR_TX_debug_print(IR_CMD_PATH, active_path + 1);
            while (fsm_state == FSM_IDLE);
            while (fsm_state != FSM_IDLE);
            pbtxcmd_s13_latch = 0;
        }
    } else {
        pbtxcmd_s13_latch = 1;
    }

    /* PB_START: send start command and go to S5 */
    if (PB_START == 0) {
        if (pbstart_s13_latch && fsm_state == FSM_IDLE) {
            if (active_path == 3) {
                manual_int_idx = 0;
                manual_dir     = 0;
                lcd_state = LCD_S17;
            } else {
                send_ir_packet(IR_CMD_START, 0x0000, IR_ADDR);
                while (fsm_state == FSM_IDLE);
                while (fsm_state != FSM_IDLE);
                IR_TX_debug_print(IR_CMD_START, 0x0000);
                intersection_num = 0;
                crossing_updated = 0;
                lcd_state = LCD_S5;
            }
            pbstart_s13_latch = 0;
        }
    } else {
        pbstart_s13_latch = 1;
    }
}

static void handle_s14_buttons(void)
{
    if (lcd_state != LCD_S14) { pbtxcmd_s14_latch = 1; pbstart_s14_latch = 1; return; }

    /* PB_TXCMD: send MODE to car */
    if (PB_TXCMD == 0) {
        if (pbtxcmd_s14_latch && fsm_state == FSM_IDLE) {
            send_ir_packet(IR_CMD_MODE, active_mode, IR_ADDR);
            IR_TX_debug_print(IR_CMD_MODE, active_mode);
            while (fsm_state == FSM_IDLE);
            while (fsm_state != FSM_IDLE);
            pbtxcmd_s14_latch = 0;
        }
    } else {
        pbtxcmd_s14_latch = 1;
    }

    /* PB_START: send start command and go to S6 */
    if (PB_START == 0) {
        if (pbstart_s14_latch && fsm_state == FSM_IDLE) {
            send_ir_packet(IR_CMD_START, 0x0000, IR_ADDR);
            while (fsm_state == FSM_IDLE);
            while (fsm_state != FSM_IDLE);
            IR_TX_debug_print(IR_CMD_START, 0x0000);
            lcd_state = LCD_S6;
            pbstart_s14_latch = 0;
        }
    } else {
        pbstart_s14_latch = 1;
    }
}

static void handle_s15_buttons(void)
{
    if (lcd_state != LCD_S15) { pbtxcmd_s15_latch = 1; pbstart_s15_latch = 1; return; }

    /* PB_TXCMD: send MODE (pathfind) + all waypoints + zero_yaw via IR */
    if (PB_TXCMD == 0) {
        if (pbtxcmd_s15_latch && fsm_state == FSM_IDLE) {
            send_ir_packet(IR_CMD_MODE, 0x0002, IR_ADDR);
            IR_TX_debug_print(IR_CMD_MODE, 0x0002);
            while (fsm_state == FSM_IDLE);
            while (fsm_state != FSM_IDLE);
            send_path_waypoints();
            while (fsm_state != FSM_IDLE);
            send_ir_packet(IR_CMD_ZERO_YAW, 0x0012, IR_ADDR);
            while (fsm_state == FSM_IDLE);
            while (fsm_state != FSM_IDLE);
            IR_TX_debug_print(IR_CMD_ZERO_YAW, 0x0012);
            pbtxcmd_s15_latch = 0;
        }
    } else {
        pbtxcmd_s15_latch = 1;
    }

    /* PB_START: send start command and go to S11 */
    if (PB_START == 0) {
        if (pbstart_s15_latch && fsm_state == FSM_IDLE) {
            send_ir_packet(IR_CMD_START, 0x0000, IR_ADDR);
            while (fsm_state == FSM_IDLE);
            while (fsm_state != FSM_IDLE);
            IR_TX_debug_print(IR_CMD_START, 0x0000);
            lcd_state = LCD_S11;
            pbstart_s15_latch = 0;
        }
    } else {
        pbstart_s15_latch = 1;
    }
}

static void handle_s17_buttons(void)
{
    if (lcd_state != LCD_S17) { pbsw_s17_latch = 1; pbtxcmd_s17_latch = 1; return; }

    /* PB_TXCMD: send current intersection decision and advance */
    if (PB_TXCMD == 0) {
        if (pbtxcmd_s17_latch && fsm_state == FSM_IDLE && manual_int_idx < 8) {
            send_ir_packet(
                (uint8_t)IR_CMD_CROSSING_DECISION,
                (uint16_t)(((uint16_t)manual_int_idx << 8) | (uint16_t)manual_dir),
                IR_ADDR
            );
            IR_TX_debug_print(IR_CMD_CROSSING_DECISION, (uint16_t)(((uint16_t)manual_int_idx << 8) | (uint16_t)manual_dir));
            while (fsm_state == FSM_IDLE);
            while (fsm_state != FSM_IDLE);
            LCD_FSM_s17_advance(); // increments manual_int_idx, resets manual_dir, triggers redraw
            pbtxcmd_s17_latch = 0;
        }
    } else {
        pbtxcmd_s17_latch = 1;
    }

    /* JoyStick_SW: all done, send START and begin running */
    if (JoyStick_SW == 0) {
        if (pbsw_s17_latch && fsm_state == FSM_IDLE) {
            send_ir_packet(IR_CMD_START, 0x0000, IR_ADDR);
            IR_TX_debug_print(IR_CMD_START, 0x0000);
            while (fsm_state == FSM_IDLE);
            while (fsm_state != FSM_IDLE);
            lcd_state = LCD_S5;
            pbsw_s17_latch = 0;
        }
    } else {
        pbsw_s17_latch = 1;
    }
}

static void handle_s7_buttons(void)
{
    if (lcd_state != LCD_S7) { pb0_s7_latch = 1; return; }

    /* JoyStick_SW: resume — send IR start and return to previous running state */
    if (JoyStick_SW == 0) {
        if (pb0_s7_latch && fsm_state == FSM_IDLE) {
            send_ir_packet(IR_CMD_START, 0xFF, IR_ADDR);
            IR_TX_debug_print(IR_CMD_START, 0xFF);
            while (fsm_state == FSM_IDLE);
            while (fsm_state != FSM_IDLE);
            lcd_state = s7_resume_state;
            pb0_s7_latch = 0;
        }
    } else {
        pb0_s7_latch = 1;
    }
}

static void handle_s8_buttons(void)
{
    if (lcd_state != LCD_S8) { pb0_s8_latch = 1; return; }

    /* JoyStick_SW: go to S9 (no IR — pathfind uses BT path via S15) */
    if (JoyStick_SW == 0) {
        if (pb0_s8_latch) {
            active_mode = selected_mode;
            active_path = selected_path;
            lcd_state = LCD_S9;
            pb0_s8_latch = 0;
        }
    } else {
        pb0_s8_latch = 1;
    }
}

// ---- Crossing display (Auto/Field mode S5) ----
static void update_crossing_display(void)
{
    static xdata char line1[17];
    unsigned char path_num = active_path + 1; // 0-indexed → 1-3
    unsigned char i = 0;

    line1[i++] = 'P';
    line1[i++] = (char)('0' + path_num);
    line1[i++] = ' ';
    line1[i++] = 'I'; line1[i++] = 'n'; line1[i++] = 't'; line1[i++] = ':';
    if (intersection_num >= 10)
        line1[i++] = (char)('0' + intersection_num / 10);
    line1[i++] = (char)('0' + intersection_num % 10);
    while (i < 16) line1[i++] = ' ';
    line1[16] = '\0';
    LCDprint(line1, 1, 0);

    switch (crossing_action) {
        case 0: LCDprint("Forward         ", 2, 0); break;
        case 1: LCDprint("Left            ", 2, 0); break;
        case 2: LCDprint("Right           ", 2, 0); break;
        case 3: LCDprint("Stop            ", 2, 0); break;
        default: break;
    }
}

// ---- Main ----
void main (){
	static xdata float joystick_x;
	static xdata float joystick_y;
	unsigned char x_byte, y_byte;

	init_pin_input();
	TIMER0_Init();
	TIMER2_Init();
	UART0_init();
	UART1_init();
	IR_RX_init();
	PathBuffer_reset();
	IMUBuffer_reset();

	InitPinADC(1, 4); // Configure Joystick_Y as analog input
	InitPinADC(1, 5); // Configure Joystick_X as analog input
	InitADC();

	LCD_4BIT();
	LCD_FSM_init();

	UART0_send_string("start\r\n");
	while(1){
        // poBluetooth_debug_drive_sequencewer debug 
       // Bluetooth_debug_drive_sequence();

        
		Bluetooth_handle_commands();
		Bluetooth_forward_imu();
		joystick_x = Volts_at_Pin(JOYSTICK_X);
        UART0_send_string("joystick_x=");
        uart0_send_uint((unsigned int)(joystick_x * 1000.0f));
        UART0_send_string("mV\r\n");
        
		joystick_y = Volts_at_Pin(JOYSTICK_Y);

		x_byte = volt_to_byte(joystick_x);
		y_byte = volt_to_byte(joystick_y);

		LCD_FSM_update(x_byte, y_byte);

		handle_pb_start();
		handle_pb_pause();
		handle_pb_reset();
		handle_s7_buttons();
		handle_s8_buttons();
		handle_s13_buttons();
		handle_s14_buttons();
		handle_s15_buttons();
		handle_s17_buttons();

		// Update intersection display in auto mode
		if (lcd_state == LCD_S5 && crossing_updated) {
			crossing_updated = 0;
			update_crossing_display();
		}

		// Debug: print any addr=0x7 frame received
        /*
		if (dbg_frame_ready) {
			dbg_frame_ready = 0;
			IR_RX_debug_print(&dbg_frame);
		}
        */
		// Debug: print imu_regs[] to UART0 whenever any register changes
		{
			static unsigned int xdata imu_prev[IMU_REG_COUNT];
			static bit imu_prev_init = 0;
			unsigned char ri;
			bit imu_changed = 0;

			if (!imu_prev_init) {
				for (ri = 0; ri < IMU_REG_COUNT; ri++)
					imu_prev[ri] = imu_regs[ri];
				imu_prev_init = 1;
			}

			for (ri = 0; ri < IMU_REG_COUNT; ri++) {
				if (imu_regs[ri] != imu_prev[ri]) {
					imu_changed = 1;
					break;
				}
			}

			if (imu_changed) {
				UART0_send_string("IMU:");
				for (ri = 0; ri < IMU_REG_COUNT; ri++) {
					UART0_send_char(' ');
					uart0_send_hex_byte((unsigned char)(imu_regs[ri] >> 8));
					uart0_send_hex_byte((unsigned char)(imu_regs[ri] & 0xFFu));
					imu_prev[ri] = imu_regs[ri];
				}
				UART0_send_string("\r\n");
			}
		}

		// Send joystick only in remote mode
		if (lcd_state == LCD_S6 && fsm_state == FSM_IDLE) {
			send_ir_packet(IR_CMD_JOYSTICK_X, x_byte, IR_ADDR);
			while (fsm_state == FSM_IDLE);
			while (fsm_state != FSM_IDLE);

			send_ir_packet(IR_CMD_JOYSTICK_Y, y_byte, IR_ADDR);
			while (fsm_state == FSM_IDLE);
			while (fsm_state != FSM_IDLE);
		}
		
	}
}

#include "lcd_fsm.h"
#include "lcd.h"
#include "config.h"

// ---- Joystick thresholds ----
#define JOY_HIGH     200
#define JOY_LOW       50
#define JOY_MID_LO   130
#define JOY_MID_HI   180

// ---- Public state ----
lcd_state_t   lcd_state    = LCD_S0;
unsigned char selected_mode = 0;   // 0=auto, 1=remote, 2=pathfind
unsigned char selected_path = 0;   // 0=path1, 1=path2, 2=path3, 3=manual
unsigned char active_mode   = 0;   // locked in when running starts: 0=auto, 1=remote, 2=pathfind
unsigned char active_path   = 0;   // locked in when running starts: 0=path1, 1=path2, 2=path3, 3=manual
unsigned char manual_int_idx = 0;  // current intersection being configured in S17 (0-7)
unsigned char manual_dir     = 0;  // currently selected direction in S17

// ---- Internal ----
static lcd_state_t prev_state = LCD_NUM_STATES; // force redraw on first call
// s7_from_s11 removed — main.c now tracks resume state and handles S7 resume
static bit s17_needs_redraw = 0; // set by LCD_FSM_s17_advance to trigger redraw

// Joystick latches: must return to center before next trigger
static bit joy_x_ready = 1;
static bit joy_y_ready = 1;

// Button latches: must release before next press registers
static bit pb0_ready = 1;
static bit pb1_ready = 1;
// ---- Redraw current state to LCD ----
static void redraw(void)
{
    static xdata char s17_line1[17];
    unsigned char i;
    unsigned char n;

    switch (lcd_state)
    {
        case LCD_S0:
            LCDprint("Welcome!        ", 1, 0);
            LCDprint("Press PB0       ", 2, 0);
            break;

        case LCD_S1:
            LCDprint("Choose Mode:    ", 1, 0);
            if (selected_mode == 0)
                LCDprint(">Fld  Rem  Path ", 2, 0);
            else if (selected_mode == 1)
                LCDprint(" Fld >Rem  Path ", 2, 0);
            else
                LCDprint(" Fld  Rem >Path ", 2, 0);
            break;

        case LCD_S2:
            LCDprint("Choose Path:    ", 1, 0);
            if      (selected_path == 0) LCDprint(">P1  P2  P3  Man", 2, 0);
            else if (selected_path == 1) LCDprint(" P1 >P2  P3  Man", 2, 0);
            else if (selected_path == 2) LCDprint(" P1  P2 >P3  Man", 2, 0);
            else                         LCDprint(" P1  P2  P3 >Man", 2, 0);
            break;

        case LCD_S3:
            LCDprint("Ready (Field)   ", 1, 0);
            LCDprint("Press PB0       ", 2, 0);
            break;

        case LCD_S4:
            LCDprint("Ready (Remote)  ", 1, 0);
            LCDprint("Press PB0       ", 2, 0);
            break;

        case LCD_S5:
            LCDprint("Running (Field) ", 1, 0);
            LCDprint("PB_PAUSE:Pause  ", 2, 0);
            break;

        case LCD_S6:
            LCDprint("Running (Remote)", 1, 0);
            LCDprint("                ", 2, 0);
            break;

        case LCD_S7:
            LCDprint("Paused          ", 1, 0);
            LCDprint("PB0:Go RST:Reset", 2, 0);
            break;

        case LCD_S8:
            LCDprint("Path Tracking   ", 1, 0);
            LCDprint("Press PB0       ", 2, 0);
            break;

        case LCD_S9:
            LCDprint("Path test mode  ", 1, 0);
            LCDprint("Wait PATH_END   ", 2, 0);
            break;

        case LCD_S11:
            LCDprint("Running (Path)  ", 1, 0);
            LCDprint("                ", 2, 0);
            break;

        case LCD_S13:
            LCDprint("Ready to IR TX  ", 1, 0);
            LCDprint("Press PB_TX     ", 2, 0);
            break;

        case LCD_S14:
            LCDprint("Ready to IR TX  ", 1, 0);
            LCDprint("Press PB_TX     ", 2, 0);
            break;

        case LCD_S15:
            LCDprint("Ready to IR TX  ", 1, 0);
            LCDprint("Press PB_TX     ", 2, 0);
            break;

        case LCD_S17:
            if (manual_int_idx >= 8) {
                LCDprint("Man All Sent    ", 1, 0);
                LCDprint("SW to Start     ", 2, 0);
            } else {
                i = 0;
                s17_line1[i++] = 'M'; s17_line1[i++] = 'a'; s17_line1[i++] = 'n';
                s17_line1[i++] = ' ';
                s17_line1[i++] = 'I'; s17_line1[i++] = 'n'; s17_line1[i++] = 't'; s17_line1[i++] = ':';
                n = manual_int_idx + 1;
                if (n >= 10) s17_line1[i++] = (char)('0' + n / 10);
                s17_line1[i++] = (char)('0' + n % 10);
                while (i < 16) s17_line1[i++] = ' ';
                s17_line1[16] = '\0';
                LCDprint(s17_line1, 1, 0);
                switch (manual_dir) {
                    case 0: LCDprint(">Forward        ", 2, 0); break;
                    case 1: LCDprint(">Left           ", 2, 0); break;
                    case 2: LCDprint(">Right          ", 2, 0); break;
                    case 3: LCDprint(">Stop           ", 2, 0); break;
                    default: break;
                }
            }
            break;

        default:
            break;
    }
}

// ---- Pause helper (called from main.c) ----
void LCD_FSM_pause(lcd_state_t from_state)
{
    (void)from_state; // resume state tracked by main.c
    lcd_state = LCD_S7;
}

// ---- Manual intersection advance (called from main.c after sending IR) ----
void LCD_FSM_s17_advance(void)
{
    if (manual_int_idx < 8)
        manual_int_idx++;
    manual_dir = 0;
    s17_needs_redraw = 1;
}

// ---- Init ----
void LCD_FSM_init(void)
{
    lcd_state     = LCD_S0;
    prev_state    = LCD_NUM_STATES;
    selected_mode  = 0;
    selected_path  = 0;
    active_mode    = 0;
    active_path    = 0;
    manual_int_idx = 0;
    manual_dir     = 0;
    joy_x_ready    = 1;
    joy_y_ready    = 1;
    pb0_ready      = 1;
    pb1_ready      = 1;
}

// ---- Update — call every main loop iteration ----
void LCD_FSM_update(unsigned char x_byte, unsigned char y_byte)
{
    unsigned char pb0_pressed = 0;
    unsigned char pb1_pressed = 0;
    unsigned char joy_up      = 0;
    unsigned char joy_down    = 0;
    unsigned char joy_right   = 0;
    unsigned char joy_left    = 0;
    unsigned char need_redraw = 0;

    // ---- Read buttons (active low, latch until release) ----
    if (JoyStick_SW == 0) {
        if (pb0_ready) { pb0_pressed = 1; pb0_ready = 0; }
    } else {
        pb0_ready = 1;
    }

    if (PB_PAUSE == 0) {
        if (pb1_ready) { pb1_pressed = 1; pb1_ready = 0; }
    } else {
        pb1_ready = 1;
    }

    // ---- Read joystick (latch until returns to center) ----
    if (x_byte > JOY_HIGH && joy_x_ready) { joy_up   = 1; joy_x_ready = 0; }
    if (x_byte < JOY_LOW  && joy_x_ready) { joy_down = 1; joy_x_ready = 0; }
    if (x_byte >= JOY_MID_LO && x_byte <= JOY_MID_HI) joy_x_ready = 1;

    if (y_byte > JOY_HIGH && joy_y_ready) { joy_right = 1; joy_y_ready = 0; }
    if (y_byte < JOY_LOW  && joy_y_ready) { joy_left  = 1; joy_y_ready = 0; }
    if (y_byte >= JOY_MID_LO && y_byte <= JOY_MID_HI) joy_y_ready = 1;

    // ---- FSM transitions ----
    switch (lcd_state)
    {
        case LCD_S0:
            if (pb0_pressed) lcd_state = LCD_S1;
            break;

        case LCD_S1:
            if (joy_right && selected_mode < 2) { selected_mode++; need_redraw = 1; }
            if (joy_left  && selected_mode > 0) { selected_mode--; need_redraw = 1; }
            if (pb0_pressed) {
                if      (selected_mode == 0) lcd_state = LCD_S2;
                else if (selected_mode == 1) lcd_state = LCD_S4;
                else                         lcd_state = LCD_S8;
            }
            break;

        case LCD_S2:
            if (joy_right && selected_path < 3) { selected_path++; need_redraw = 1; }
            if (joy_left  && selected_path > 0) { selected_path--; need_redraw = 1; }
            if (pb0_pressed) lcd_state = LCD_S3;
            if (joy_up)      lcd_state = LCD_S1;
            break;

        case LCD_S3:
            if (pb0_pressed) { active_mode = selected_mode; active_path = selected_path; lcd_state = LCD_S13; }
            if (joy_up)      lcd_state = LCD_S2;
            break;

        case LCD_S4:
            if (pb0_pressed) { active_mode = selected_mode; active_path = selected_path; lcd_state = LCD_S14; }
            if (joy_up)      lcd_state = LCD_S1;
            break;

        case LCD_S5:
            break;

        case LCD_S6:
            break;

        case LCD_S8:
            /* pb0 start handled by handle_s8_buttons() in main.c */
            if (joy_up)      lcd_state = LCD_S1;
            break;

        case LCD_S7:
            /* pb0 resume handled by handle_s7_buttons() in main.c */
            if (pb1_pressed) {
                selected_mode = 0;
                selected_path = 0;
                lcd_state = LCD_S0;
            }
            break;

        case LCD_S9:
            break;

        case LCD_S11:
            break;

        case LCD_S17:
            if (manual_int_idx < 8) {
                if (joy_right) { manual_dir = (manual_dir < 3) ? manual_dir + 1 : 0; need_redraw = 1; }
                if (joy_left)  { manual_dir = (manual_dir > 0) ? manual_dir - 1 : 3; need_redraw = 1; }
            }
            break;

        default:
            break;
    }

    // ---- Redraw ----
    if (lcd_state != prev_state) {
        redraw();
        prev_state = lcd_state;
        s17_needs_redraw = 0;
    } else if (need_redraw || s17_needs_redraw) {
        redraw();
        s17_needs_redraw = 0;
    }

}

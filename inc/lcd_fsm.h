#ifndef LCD_FSM_H
#define LCD_FSM_H

typedef enum {
    LCD_S0 = 0,  // Welcome
    LCD_S1,      // Choose Mode (Auto/Remote/Pathfind)
    LCD_S2,      // Choose Path (Auto only)
    LCD_S3,      // Ready (Auto)
    LCD_S4,      // Ready (Remote)
    LCD_S5,      // Running (Auto)
    LCD_S6,      // Running (Remote)
    LCD_S7,      // Pause
    LCD_S8,      // Ready (Pathfind)
    LCD_S9,      // Running (Pathfind)
    LCD_S11,     // Running (Path via BT)
    LCD_S13,     // Ready to IR TX (Field/Auto) — PB_TXCMD: path+mode, PB_START: start→S5, ir start cmd
    LCD_S14,     // Ready to IR TX (Remote) — PB_TXCMD: mode, PB_START: start→S6, ir start cmd
    LCD_S15,     // Ready to IR TX (BT path) — PB_TXCMD: mode, waypoints, PB_START: start→S11, ir start cmd
    LCD_S17,     // Manual intersection setup (Auto/Manual submode)
    LCD_NUM_STATES
} lcd_state_t;

extern lcd_state_t    lcd_state;
extern unsigned char  selected_mode;   // 0=auto, 1=remote, 2=pathfind
extern unsigned char  selected_path;   // 0=path1, 1=path2, 2=path3, 3=manual
extern unsigned char  active_mode;     // locked in at start: 0=auto, 1=remote, 2=pathfind
extern unsigned char  active_path;     // locked in at start: 0=path1, 1=path2, 2=path3, 3=manual
extern unsigned char  manual_int_idx;  // current intersection being configured (0-7)
extern unsigned char  manual_dir;      // currently selected direction (0=fwd,1=left,2=right,3=stop)

void LCD_FSM_init(void);
void LCD_FSM_update(unsigned char x_byte, unsigned char y_byte);
void LCD_FSM_pause(lcd_state_t from_state);
void LCD_FSM_s17_advance(void); /* advance to next intersection and redraw */

#endif

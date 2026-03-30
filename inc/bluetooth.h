#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "ir_rx.h"

void IR_RX_decode_command(const IR_Frame_t *frame);
void Bluetooth_handle_commands(void);
void Bluetooth_forward_imu(void);
void Bluetooth_debug_drive_sequence(void);

#endif

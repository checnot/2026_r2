#ifndef IR_ACTION_H
#define IR_ACTION_H

#include <stdint.h>
#include "H7_F1_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IR_ACTION_CMD_NONE             0U
#define IR_ACTION_CMD_DOCKING_SUCCESS  1U

/* Required IR packet from F103 to H7. Change these 3 bytes if protocol changes. */
#define IR_ACTION_DOCKING_B0           0x1CU
#define IR_ACTION_DOCKING_B1           0x2FU
#define IR_ACTION_DOCKING_B2           0x33U

/* Debug variables for Keil Watch. */
extern volatile uint32_t ir_action_poll_count;
extern volatile uint32_t ir_action_rx_count;
extern volatile uint32_t ir_action_no_new_count;
extern volatile uint32_t ir_action_unknown_count;
extern volatile uint32_t ir_action_docking_success_set_count;
extern volatile uint8_t  ir_action_last_cmd;
extern volatile uint8_t  ir_action_last_ch;
extern volatile uint8_t  ir_action_last_seq;
extern volatile uint8_t  ir_action_rx_frame[H7_F1_SPI_IR_FRAME_LEN];

/* Call this at the original RTOS IR polling position. */
uint8_t IR_Action_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* IR_ACTION_H */

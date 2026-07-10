#ifndef IR_ACTION_H
#define IR_ACTION_H

#include <stdint.h>
#include "H7_F1_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IR_ACTION_CMD_NONE             0U
#define IR_ACTION_CMD_PACKET1          1U
#define IR_ACTION_CMD_PACKET2          2U
#define IR_ACTION_CMD_PACKET3          3U
#define IR_ACTION_CMD_PACKET4          4U
#define IR_ACTION_CMD_PACKET5          5U
#define IR_ACTION_CMD_PACKET6          6U
#define IR_ACTION_CMD_MAX              IR_ACTION_CMD_PACKET6

/* Keep the old name for compatibility. Packet1 used to trigger DOCKING_SUCCESS. */
#define IR_ACTION_CMD_DOCKING_SUCCESS  IR_ACTION_CMD_PACKET1

/* Six IR packets from sender side. */
#define IR_ACTION_PACKET1_B0           0x1CU
#define IR_ACTION_PACKET1_B1           0x2FU
#define IR_ACTION_PACKET1_B2           0x33U

#define IR_ACTION_PACKET2_B0           0x01U
#define IR_ACTION_PACKET2_B1           0x02U
#define IR_ACTION_PACKET2_B2           0x03U

#define IR_ACTION_PACKET3_B0           0xA5U
#define IR_ACTION_PACKET3_B1           0x5AU
#define IR_ACTION_PACKET3_B2           0x66U

#define IR_ACTION_PACKET4_B0           0x3CU
#define IR_ACTION_PACKET4_B1           0xC3U
#define IR_ACTION_PACKET4_B2           0x96U

#define IR_ACTION_PACKET5_B0           0x87U
#define IR_ACTION_PACKET5_B1           0x78U
#define IR_ACTION_PACKET5_B2           0xB4U

#define IR_ACTION_PACKET6_B0           0xD2U
#define IR_ACTION_PACKET6_B1           0x2DU
#define IR_ACTION_PACKET6_B2           0xE1U

/* Debug variables for Keil Watch. */
extern volatile uint32_t ir_action_poll_count;
extern volatile uint32_t ir_action_rx_count;
extern volatile uint32_t ir_action_no_new_count;
extern volatile uint32_t ir_action_unknown_count;
extern volatile uint32_t ir_action_packet_count[IR_ACTION_CMD_MAX + 1U];
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


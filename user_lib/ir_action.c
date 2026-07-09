#include "ir_action.h"
#include "BT_customized.h"

volatile uint32_t ir_action_poll_count = 0;
volatile uint32_t ir_action_rx_count = 0;
volatile uint32_t ir_action_no_new_count = 0;
volatile uint32_t ir_action_unknown_count = 0;
volatile uint32_t ir_action_docking_success_set_count = 0;
volatile uint8_t  ir_action_last_cmd = IR_ACTION_CMD_NONE;
volatile uint8_t  ir_action_last_ch = 0;
volatile uint8_t  ir_action_last_seq = 0;
volatile uint8_t  ir_action_rx_frame[H7_F1_SPI_IR_FRAME_LEN] = {0};

static void IR_Action_ClearRxFrame(void)
{
    ir_action_rx_frame[0] = 0U;
    ir_action_rx_frame[1] = 0U;
    ir_action_rx_frame[2] = 0U;
}

static uint8_t IR_Action_IsDockingSuccessFrame(const uint8_t frame[H7_F1_SPI_IR_FRAME_LEN])
{
    return (frame[0] == IR_ACTION_DOCKING_B0 &&
            frame[1] == IR_ACTION_DOCKING_B1 &&
            frame[2] == IR_ACTION_DOCKING_B2) ? 1U : 0U;
}

uint8_t IR_Action_Poll(void)
{
    uint8_t frame[H7_F1_SPI_IR_FRAME_LEN];
    uint8_t cmd = IR_ACTION_CMD_NONE;

    ir_action_poll_count++;

    /* H7_F1_SPI_Task() returns 1 only when a new IR frame is received. */
    if (!H7_F1_SPI_Task())
    {
        IR_Action_ClearRxFrame();
        ir_action_no_new_count++;
        ir_action_last_cmd = IR_ACTION_CMD_NONE;
        return IR_ACTION_CMD_NONE;
    }

    H7_F1_SPI_GetIrFrame(frame);

    ir_action_rx_count++;
    ir_action_last_ch = H7_F1_SPI_GetIrChannel();
    ir_action_last_seq = H7_F1_SPI_GetIrSeq();
    ir_action_rx_frame[0] = frame[0];
    ir_action_rx_frame[1] = frame[1];
    ir_action_rx_frame[2] = frame[2];

    if (IR_Action_IsDockingSuccessFrame(frame))
    {
        /* Tell the existing behavior tree that docking is complete.
         * The behavior tree will execute the original code:
         *   clamp.catch_flag = 0;
         *   osDelay(800);
         *   clamp.rotate_flag = 0;
         *   chassis_set_lock(0);
         *   MC_bb.mc_mission_state = WAIT_MF_INSTRUCTION;
         */
        MC_bb.DOCKING_SUCCESS = 1U;
        ir_action_docking_success_set_count++;
        cmd = IR_ACTION_CMD_DOCKING_SUCCESS;
    }
    else
    {
        ir_action_unknown_count++;
    }

    ir_action_last_cmd = cmd;
    H7_F1_SPI_ClearNewIrFlag();

    return cmd;
}


#include "ir_action.h"
#include "BT_customized.h"

volatile uint32_t ir_action_poll_count = 0;
volatile uint32_t ir_action_rx_count = 0;
volatile uint32_t ir_action_no_new_count = 0;
volatile uint32_t ir_action_unknown_count = 0;
volatile uint32_t ir_action_packet_count[IR_ACTION_CMD_MAX + 1U] = {0};
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

static uint8_t IR_Action_MatchFrame(const uint8_t frame[H7_F1_SPI_IR_FRAME_LEN])
{
    if (frame[0] == IR_ACTION_PACKET1_B0 &&
        frame[1] == IR_ACTION_PACKET1_B1 &&
        frame[2] == IR_ACTION_PACKET1_B2)
    {
        return IR_ACTION_CMD_PACKET1;
    }

    if (frame[0] == IR_ACTION_PACKET2_B0 &&
        frame[1] == IR_ACTION_PACKET2_B1 &&
        frame[2] == IR_ACTION_PACKET2_B2)
    {
        return IR_ACTION_CMD_PACKET2;
    }

    if (frame[0] == IR_ACTION_PACKET3_B0 &&
        frame[1] == IR_ACTION_PACKET3_B1 &&
        frame[2] == IR_ACTION_PACKET3_B2)
    {
        return IR_ACTION_CMD_PACKET3;
    }

    if (frame[0] == IR_ACTION_PACKET4_B0 &&
        frame[1] == IR_ACTION_PACKET4_B1 &&
        frame[2] == IR_ACTION_PACKET4_B2)
    {
        return IR_ACTION_CMD_PACKET4;
    }

    if (frame[0] == IR_ACTION_PACKET5_B0 &&
        frame[1] == IR_ACTION_PACKET5_B1 &&
        frame[2] == IR_ACTION_PACKET5_B2)
    {
        return IR_ACTION_CMD_PACKET5;
    }

    if (frame[0] == IR_ACTION_PACKET6_B0 &&
        frame[1] == IR_ACTION_PACKET6_B1 &&
        frame[2] == IR_ACTION_PACKET6_B2)
    {
        return IR_ACTION_CMD_PACKET6;
    }

    return IR_ACTION_CMD_NONE;
}

static void IR_Action_OnPacket1(void)
{
    /* 1 号红外保持原用途：PS 对杆成功信号，不参与 JGCB。 */
    // MC_bb.DOCKING_SUCCESS = 1U;
    PS_bb.dock_success = 1;
}

static void IR_Action_OnPacket2(void)
{
    /* 2 号红外：R2 在 CZ 等待点收到后，进入上 R1 台阶流程。 */
    JGCB_bb.r1_platform_cmd = 1U;
}

static void IR_Action_OnPacket3(void)
{
    /* 3 号红外：R2 已在 R1 平台上时，仅关气泵释放手中 KFS。 */
    JGCB_bb.r1_place_cmd = 1U;
}

static void IR_Action_OnPacket4(void)
{
    /*
     * 4 号红外：九宫格 X 最小列。
     * JGCB 内部列号 3 对应 B_CHASSIS_SUDUKU_3_X(当前三列中的最小 X)。
     */
    JGCB_bb.ir_column_cmd = 3U;
}

static void IR_Action_OnPacket5(void)
{
    /* 5 号红外：九宫格中间列。 */
    JGCB_bb.ir_column_cmd = 2U;
}

static void IR_Action_OnPacket6(void)
{
    /*
     * 6 号红外：九宫格 X 最大列。
     * JGCB 内部列号 1 对应 B_CHASSIS_SUDUKU_1_X(当前三列中的最大 X)。
     */
    JGCB_bb.ir_column_cmd = 1U;
}

static void IR_Action_Dispatch(uint8_t cmd)
{
    switch (cmd)
    {
    case IR_ACTION_CMD_PACKET1:
        IR_Action_OnPacket1();
        break;

    case IR_ACTION_CMD_PACKET2:
        IR_Action_OnPacket2();
        break;

    case IR_ACTION_CMD_PACKET3:
        IR_Action_OnPacket3();
        break;

    case IR_ACTION_CMD_PACKET4:
        IR_Action_OnPacket4();
        break;

    case IR_ACTION_CMD_PACKET5:
        IR_Action_OnPacket5();
        break;

    case IR_ACTION_CMD_PACKET6:
        IR_Action_OnPacket6();
        break;

    default:
        break;
    }
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

    cmd = IR_Action_MatchFrame(frame);
    if (cmd != IR_ACTION_CMD_NONE)
    {
        ir_action_packet_count[cmd]++;
        IR_Action_Dispatch(cmd);
    }
    else
    {
        ir_action_unknown_count++;
    }

    ir_action_last_cmd = cmd;
    H7_F1_SPI_ClearNewIrFlag();

    return cmd;
}




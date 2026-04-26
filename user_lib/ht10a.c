#include "ht10a.h"

int16_t _channels[16];
uint8_t ht10a_buffer[30];				//遥控器原始数据

REMOTE_STU HT10A;//在ht10a.c中定义的原始遥控器状态结构体
REMOTE_STU HT10A_last = {0}; //在freertos.c中定义的遥控器last状态

/*================================================= 客制化判断逻辑 ========================================================*/
#include "clamp.h"
#include "chassis.h"
#include "robotic_arm.h"
#include "trajectory.h"

//遥控器控制的目标位置
float x0_target_remote = 0.15f; //初始目标位置，避免机械臂一开机就飞出去
float y0_target_remote = 0.15f; //初始目标位置，避免机械臂一开机就飞出去
float theta0_target_remote = 0.0f;
float effector_pitch_target_remote = -PI/2.0f; //初始目标位置，避免末端执行器一开机就飞出去
float effector_rotate_target_remote = 0.0f; //初始目标位置，避免末端执行器一开机就飞出去

uint8_t air_pump_flag_remote = 0; //遥控器控制的气泵状态，0为不控制，1为吸

/**
 * @brief 用户自定义判断函数，根据遥控器输入的状态进行相应的操作，一般直接插入ht10a_process中调用，切换项目时可注销
*/
void customized_judgment(void)
{

  // ====== 以下为适配应用层 Task 轮询方案(1ms) 的新代码架构 ======
  // 注意：如果想彻底解耦，建议把下面这部分代码剪切到 freertos.c 的 cmd_task 中运行
  // 并且配合末尾的 HT10A_last = HT10A; 进行状态同步。不要在 ht10a_process() 中调用它。
  
//============= 测试用，合并branch时删除 =============

//  if((HT10A.switch_a == 1 && HT10A_last.switch_a == 0) || (HT10A.switch_a == -1 && HT10A_last.switch_a == 0))
// {
//   float base_pos;
//   // 确定新路点的位置基准：
//   if(debug_traj_queue_a1.count > 0) {
//     // 队列里还有未执行的点，基于队列最后一个点延伸
//     int last_idx = (debug_traj_queue_a1.head - 1 + TRAJ_MAX_WAYPOINTS) % TRAJ_MAX_WAYPOINTS;
//     base_pos = debug_traj_queue_a1.points[last_idx];
//   } else {
//     // 队列已空，基于当前发生器正在追赶的 target_pos 延伸
//     base_pos = debug_traj_a1.target_pos;
//   } 
  
//   // 根据拨动方向注入不同目标：以每次增加 PI/2.0f (即90度) 为例
//   if(HT10A.switch_a == 1) {
//       Traj_Path_Queue_AddPoint(&debug_traj_queue_a1, base_pos + PI/2.0f); // 向上推入队基准点 + PI/2
//   } else if(HT10A.switch_a == -1) {
//       Traj_Path_Queue_AddPoint(&debug_traj_queue_a1, base_pos - PI/2.0f); // 向下拉入队基准点 - PI/2
//   }
// }

//============= 测试用，合并branch时删除 =============


  //底盘控制部分
  if(HT10A.switch_d == 0)
  {
    //前半车身升降部分
    if(HT10A_last.switch_b == -1 && HT10A.switch_b == 1)      {chassis_r2.lift_front = 1;}    
    else if(HT10A_last.switch_b == 1 && HT10A.switch_b == -1) {chassis_r2.lift_front = 0;}
    //后半车身升降部分
    if(HT10A_last.switch_c == -1 && HT10A.switch_c == 1)      {chassis_r2.lift_back = 1;}    
    else if(HT10A_last.switch_c == 1 && HT10A.switch_c == -1) {chassis_r2.lift_back = 0;}
  }
  //夹爪控制部分
  if(HT10A.switch_d == -1)
  {
    //夹爪抓取/松开
    if(HT10A_last.switch_b == -1 && HT10A.switch_b == 1)      {clamp.catch_flag = 1;}    
    else if(HT10A_last.switch_b == 1 && HT10A.switch_b == -1) {clamp.catch_flag = 0;}
    //夹爪旋转
    if(HT10A_last.knob_a < 1000 && HT10A.knob_a >= 1000)      {clamp.rotate_flag = 1;}
    else if(HT10A_last.knob_a > -1000 && HT10A.knob_a <= -1000) {clamp.rotate_flag = 0;}
    //夹爪伸缩
    if(HT10A_last.switch_c == -1 && HT10A.switch_c == 1)      {clamp.retractable_flag = 1;}    
    else if(HT10A_last.switch_c == 1 && HT10A.switch_c == -1) {clamp.retractable_flag = 0;}
  }
  //机械臂控制部分
  if(HT10A.switch_d == 1)
  {
    if(abs_int16(HT10A.right_x) > 500)
    {
      if(HT10A.right_x > 0) x0_target_remote += 0.00020f; // 右拨杆右移，增加x
      else                x0_target_remote -= 0.00020f; // 右拨杆左移，减少x
    }
    if(abs_int16(HT10A.right_y) > 500)
    {
      if(HT10A.right_y > 0) y0_target_remote += 0.00020f; // 右拨杆前移，增加y
      else                y0_target_remote -= 0.00020f; // 右拨杆后移，减少y
    }
    if(abs_int16(HT10A.left_x) > 500)
    {
      if(HT10A.left_x > 0) theta0_target_remote += PI/4000.0f; // 左拨杆右移，增加theta
      else                theta0_target_remote -= PI/4000.0f; // 左拨杆左移，减少theta
    }
    if(HT10A_last.switch_b != HT10A.switch_b)
    {
      if(HT10A.switch_b == 1) air_pump_flag_remote = 1; // B开吸
      else                    air_pump_flag_remote = 0; // B关吸
    }
    effector_pitch_target_remote = -PI/2.0f + (float)(HT10A.knob_a+8000)/16000.0f*PI/2.0f; // 俯仰角范围 [-PI/2, 0]
    effector_rotate_target_remote = (float)(HT10A.knob_b+8000) / 16000.0f * PI; // 旋转角度范围 [0, PI]
  
  }

}
/*================================================= 客制化判断逻辑 ========================================================*/

/**
 * 接受HT10A的数据并进行处理
 * @param buffer用于接收原始数据
 * @param _channels用于接收处理过后的数据
*/
void HT10A_process(uint8_t buffer[30])
{
	
  HT10A.last_status.knob_a_last = HT10A.knob_a;
  HT10A.last_status.knob_b_last = HT10A.knob_b;
  HT10A.last_status.left_x_last = HT10A.left_x;
  HT10A.last_status.left_y_last = HT10A.left_y;
  HT10A.last_status.right_x_last = HT10A.right_x;
  HT10A.last_status.right_y_last = HT10A.right_y;
  HT10A.last_status.switch_a_last = HT10A.switch_a;
  HT10A.last_status.switch_b_last = HT10A.switch_b;
  HT10A.last_status.switch_c_last = HT10A.switch_c;
  HT10A.last_status.switch_d_last = HT10A.switch_d;

  if (buffer[0] != 0x0F || buffer[24] != 0x00)
  return; // 数据包头尾正确则开始处理

  _channels[0] = (buffer[1] | ((buffer[2] & 0x07) << 8)) & 0x07FF;

  // 通道1: bits 11-21 (byte2低5位 + byte3高6位)
  _channels[1] = ((buffer[2] >> 3) | (buffer[3] << 5)) & 0x07FF;
  // 通道2: bits 22-32 (byte3低2位 + byte4全8位 + byte5高1位)
  _channels[2] = ((buffer[3] >> 6) | (buffer[4] << 2) | ((buffer[5] & 0x01) << 10)) & 0x07FF;
  // 通道3: bits 33-43 (byte5低7位 + byte6高4位)
  _channels[3] = ((buffer[5] >> 1) | (buffer[6] << 7)) & 0x07FF;
  // 通道4: bits 44-54 (byte6低4位 + byte7高7位)
  _channels[4] = ((buffer[6] >> 4) | (buffer[7] << 4)) & 0x07FF;
  // 通道5: bits 55-65 (byte7低1位 + byte8全8位 + byte9高2位)
  _channels[5] = ((buffer[7] >> 7) | (buffer[8] << 1) | ((buffer[9] & 0x03) << 9)) & 0x07FF;
  // 通道6: bits 66-76 (byte9低6位 + byte10高5位)
  _channels[6] = ((buffer[9] >> 2) | (buffer[10] << 6)) & 0x07FF;
  // 通道7: bits 77-87 (byte10低3位 + byte11全8位)
  _channels[7] = ((buffer[10] >> 5) | (buffer[11] << 3)) & 0x07FF;
  // 通道8: bits 88-98 (byte12低8位 + byte13高3位)
  _channels[8] = (buffer[12] | ((buffer[13] & 0x07) << 8)) & 0x07FF;
  // 通道9: bits 99-109 (byte13低5位 + byte14高6位)
  _channels[9] = ((buffer[13] >> 3) | (buffer[14] << 5)) & 0x07FF;
  // 通道10: bits 110-120 (byte14低2位 + byte15全8位 + byte16高1位)
  _channels[10] = ((buffer[14] >> 6) | (buffer[15] << 2) | ((buffer[16] & 0x01) << 10)) & 0x07FF;
  // 通道11: bits 121-131 (byte16低7位 + byte17高4位)
  _channels[11] = ((buffer[16] >> 1) | (buffer[17] << 7)) & 0x07FF;
  // 通道12: bits 132-142 (byte17低4位 + byte18高7位)
  _channels[12] = ((buffer[17] >> 4) | (buffer[18] << 4)) & 0x07FF;
  // 通道13: bits 143-153 (byte18低1位 + byte19全8位 + byte20高2位)
  _channels[13] = ((buffer[18] >> 7) | (buffer[19] << 1) | ((buffer[20] & 0x03) << 9)) & 0x07FF;
  // 通道14: bits 154-164 (byte20低6位 + byte21高5位)
  _channels[14] = ((buffer[20] >> 2) | (buffer[21] << 6)) & 0x07FF;
  // 通道15: bits 165-175 (byte21低3位 + byte22全8位)
  _channels[15] = ((buffer[21] >> 5) | (buffer[22] << 3)) & 0x07FF;

  //	摇杆[-4000,4000]
  HT10A.left_x = (_channels[3] - 992) * 5;
  HT10A.left_y = (_channels[2] - 992) * 5;
  HT10A.right_x = (_channels[0] - 992) * 5;		//9.18更新负号
  HT10A.right_y = (_channels[1] - 992) * 5;

  //	开关-1,0,1
  HT10A.switch_a = (_channels[4] - 992) / 800;
  HT10A.switch_b = (_channels[5] - 992) / 800;
  HT10A.switch_c = (_channels[6] - 992) / 800;
  HT10A.switch_d = (_channels[7] - 992) / 800;

  //	旋钮[-8000,8000]
  HT10A.knob_a = (_channels[8] - 992) * 10;
  HT10A.knob_b = (_channels[9] - 992) * 10;

}

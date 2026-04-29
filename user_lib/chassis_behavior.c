#include "chassis_behavior.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "route.h"
#include "radar.h"

/*********************************************************************************
 * 关于这个文件的说明：
 * 把底盘需要做的一些动作放在这里。
 * 比如说直线路径跟踪，上坡，上下台阶等等。
 * 
 * 关于直线路径
 * 直线路径跟踪，我在2026/4/22 - 2026/4/23这两天琢磨了一下，改了一些不知是否重要的参数。
 * 最后我得出了一个结论，激光雷达确实可以当码盘用，没毛！
 * 那么阻碍底盘稳稳停在目标点的最重要的参数是什么呢？其实是减速区比例。这个比例可以给到0.4。
 * 如果这个比例过小，底盘就会过晚减速，导致最后急停，然后往复徘徊，无法稳稳停在目标点。
 * 
 * 然后我还要琢磨写一个上下台阶的连贯动作。
 * 这个动作可以分为几个步骤：
 * 1.底盘开到台阶前面。这里需要考虑两个参数，一个是需要上还是下。（这里先以上台阶为例）
 *   第二个参数是方向角。对于上台阶，方向是面对台阶。对于下台阶，方向是背对台阶。
 *   根据方向角参数，可以知道往x方向还是y方向开，坐标值是加还是减。
 *   0°是x方向加，180°是x方向减。90°是y方向加，270°是y方向减。
 * 2.升高底盘。在第一代R2中，采用的是气缸抬升，所以控制气缸电磁阀打开即可。
 * 3.底盘往前移动一点距离，让支撑轮子上台阶。这里改变的坐标由第1步确定。然后把两个前轮收起。
 * 4.底盘再往前移动一段距离，到后轮到台阶边缘。
 * 5.把两个后轮收起来。往前移动，让后轮登上台阶。这里后轮是有可能打滑上不去的。留给二代！
 * 
 * 以上就是自动上台阶需要调的步骤。下台阶就是这个流程反过来。
 *********************************************************************************/
void Chassis_move_xy(float distance)
{
    int8_t direction[2]; //感觉我像个弱智

    if(Radar_Pos.radar_yaw >= -10 && Radar_Pos.radar_yaw < 10) { //说明面对x正方向
        direction[0] = 1;
        direction[1] = 0;
    }
    else if(Radar_Pos.radar_yaw >= 170 || Radar_Pos.radar_yaw < -170) { //说明面对x负方向
        direction[0] = -1;
        direction[1] = 0;
    }
    else if(Radar_Pos.radar_yaw >= 80 && Radar_Pos.radar_yaw < 100) { //说明面对y正方向
        direction[0] = 0;
        direction[1] = 1;
    }
    else if(Radar_Pos.radar_yaw >= -100 && Radar_Pos.radar_yaw < -80) { //说明面对y负方向
        direction[0] = 0;
        direction[1] = -1;
    }

    if(direction[0] == 1 && direction[1] == 0) { //说明往x正方向开
        Chassis_Line_Route(Radar_Pos.radar_x + distance, Radar_Pos.radar_y, Radar_Pos.radar_yaw, 500, 500, 0, 0, 0, 0.01f, 0.4f, 70, 10, 15);
    }
    else if(direction[0] == -1 && direction[1] == 0) { //说明往x负方向开
        Chassis_Line_Route(Radar_Pos.radar_x - distance, Radar_Pos.radar_y, Radar_Pos.radar_yaw, 500, 500, 0, 0, 0, 0.01f, 0.4f, 70, 10, 15);
    }
    else if(direction[0] == 0 && direction[1] == 1) { //说明往y正方向开
        Chassis_Line_Route(Radar_Pos.radar_x, Radar_Pos.radar_y + distance, Radar_Pos.radar_yaw, 500, 500, 0, 0, 0, 0.01f, 0.4f, 70, 10, 15);
    }
    else if(direction[0] == 0 && direction[1] == -1) { //说明往y负方向开
        Chassis_Line_Route(Radar_Pos.radar_x, Radar_Pos.radar_y - distance, Radar_Pos.radar_yaw, 500, 500, 0, 0, 0, 0.01f, 0.4f, 70, 10, 15);
    }
}

void Chassis_step_up(void)
{
    chassis_r2.lift_front = 1; //底盘的前两轮升高
    chassis_r2.lift_back = 1; //底盘的后两轮升高
    osDelay(2000); //等待升高完成，时间根据实际情况调整
    Chassis_move_xy(100.0f);
    chassis_r2.lift_front = 0; //底盘的前两轮收回
    osDelay(2000); //等待收回完成，时间根据实际情况调整
    Chassis_move_xy(350.0f);
    chassis_r2.lift_back = 0; //底盘的后两轮收回
    osDelay(2000); //等待收回完成，时间根据实际情况调整
    Chassis_move_xy(200.0f);
    // finish
}

void Chassis_step_off()
{
    Chassis_move_xy(-200.0f); //先往后退一点距离，确保重心在台阶内
    chassis_r2.lift_back = 1; //底盘的后两轮升高
    osDelay(2000); //等待升高完成，时间根据实际情况调整
    Chassis_move_xy(-350.0f); //往后退，让后轮离开台阶
    chassis_r2.lift_front = 1; //底盘的前两轮升高
    osDelay(2000); //等待升高完成，时间根据实际情况调整
    Chassis_move_xy(-150.0f); //往后退，让前轮离开台阶
    chassis_r2.lift_back = 0; //底盘的后两轮收回
    chassis_r2.lift_front = 0; //底盘的前两轮收回
    osDelay(2000); //等待收回完成，时间根据实际情况调整
    // finish
}

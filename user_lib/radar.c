#include "radar.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**Livox雷达的数据回传处理**/

RADAR_POSITION Radar_Pos;

uint8_t Rec_Data[65];

void Data_process_String(char *str)
{
    // 1. 寻找帧头 "heu"
    char *header_ptr = strstr(str, "heu");
    
    if (header_ptr != NULL)
    {
        float temp_a, temp_b, temp_c, temp_d;
        
        // 2. 尝试解析四个浮点数
        if (sscanf(header_ptr + 3, "%f,%f,%f,%f", &temp_a, &temp_b, &temp_c, &temp_d) >= 4)
        {
            Radar_Pos.radar_x = temp_a;
            Radar_Pos.radar_y = temp_b;
            Radar_Pos.radar_z = temp_c;
            Radar_Pos.radar_yaw = temp_d;
        }
    }
}

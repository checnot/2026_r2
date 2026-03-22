#include "hi14.h"


#include "string.h"

/* HiPNUC protocol constants */
#define CHSYNC1                 (0x5A)              /* CHAOHE message sync code 1 */
#define CHSYNC2                 (0xA5)              /* CHAOHE message sync code 2 */
#define CH_HDR_SIZE             (0x06)              /* CHAOHE protocol header size */


/* new HiPNUC standard packet */
#define HIPNUC_ID_HI91        (0x91)
#define HIPNUC_ID_HI92        (0x92)
#define HIPNUC_ID_HI81        (0x81)

#ifndef D2R
#define D2R (0.0174532925199433F)
#endif

#ifndef R2D
#define R2D (57.2957795130823F)
#endif

#ifndef GRAVITY
#define GRAVITY (9.8F)
#endif


/**
 * @brief    Calculate HiPNUC CRC16
 *
 * @param    inital is initial value
 * @param    buf    is input buffer pointer
 * @param    len    is length of the buffer
 */
static void hipnuc_crc16(uint16_t *inital, const uint8_t *buf, uint32_t len)
{
    uint32_t crc = *inital;
    uint32_t j;
    for (j=0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = buf[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *inital = crc;
}

/* common type conversion */
#define I2(p) (*((int16_t *)(p)))
static uint16_t U2(uint8_t *p)
{
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}

/**
 * @brief 解析帧的负载并输入到数据部分(Parse the payload of a frame and feed into data section)
*/
static int parse_data(hipnuc_raw_t *raw)
{
    int ofs = 0;
    uint8_t *p = &raw->buf[CH_HDR_SIZE];
    
    /* ignore all previous data */
    raw->hi91.tag = 0;
    // raw->hi81.tag = 0;
    // raw->hi92.tag = 0;

    while (ofs < raw->len)
    {
        switch (p[ofs])
        {
        case HIPNUC_ID_HI91:
            memcpy(&raw->hi91, p + ofs, sizeof(hi91_t));
            ofs += sizeof(hi91_t);
            break;
        // case HIPNUC_ID_HI81:
        //     memcpy(&raw->hi81, p + ofs, sizeof(hi81_t));
        //     ofs += sizeof(hi81_t);
        //     break;
        // case HIPNUC_ID_HI92:
        //     memcpy(&raw->hi92, p + ofs, sizeof(hi92_t));
        //     ofs += sizeof(hi92_t);
        //     break;
        default:
            ofs++;
            break;
        }
    }
    return 1;
}

static int decode_hipnuc(hipnuc_raw_t *raw)
{
    uint16_t crc = 0;

    /* checksum */
    hipnuc_crc16(&crc, raw->buf, (CH_HDR_SIZE-2));
    hipnuc_crc16(&crc, raw->buf + CH_HDR_SIZE, raw->len);
    if (crc != U2(raw->buf + (CH_HDR_SIZE-2)))
    {
        // NL_TRACE("ch checksum error: frame:0x%X calcuate:0x%X, len:%d\n", U2(raw->buf + 4), crc, raw->len);
        return -1;
    }

    return parse_data(raw);
}

/* sync code */
static int sync_hipnuc(uint8_t *buf, uint8_t data)
{
    buf[0] = buf[1];
    buf[1] = data;
    return buf[0] == CHSYNC1 && buf[1] == CHSYNC2;
}

/**
 * @brief     HiPNUC decoder input, read one byte at a time.
 *
 * @param    raw is the decoder struct.
 * @param    data is the one byte read from stream.
 * @return   >0: decoder received a frame successfully, else: receiver did not receive a frame successfully.
 */
int hipnuc_input(hipnuc_raw_t *raw, uint8_t data)
{
    /* synchronize frame */
    if (raw->nbyte == 0)
    {
        if (!sync_hipnuc(raw->buf, data))
            return 0;
        raw->nbyte = 2;
        return 0;
    }

    raw->buf[raw->nbyte++] = data;

    if (raw->nbyte == CH_HDR_SIZE)
    {
        if ((raw->len = U2(raw->buf + 2)) > (HIPNUC_MAX_RAW_SIZE - CH_HDR_SIZE))
        {
            // NL_TRACE("ch length error: len=%d\n",raw->len);
            raw->nbyte = 0;
            return -1;
        }
    }

    if (raw->nbyte < CH_HDR_SIZE || raw->nbyte < (raw->len + CH_HDR_SIZE))
    {
        return 0;
    }

    raw->nbyte = 0;

    return decode_hipnuc(raw);
}

/*****************************************************/
uint8_t new_data_flag = 0;
uint16_t uart_rx_index = 0;
uint8_t dma_rx_buf[UART_RX_BUF_SIZE];       //陀螺仪接收第一缓冲区
uint8_t uart_rx_buf[UART_RX_BUF_SIZE];      //陀螺仪接收第二缓冲区
hipnuc_raw_t hipnuc_raw = {0};
/*****************************************************/

/**
 * @brief 在串口中断中调用的hi14数据处理函数
*/
void process_data(void)
{

    for (uint16_t i = 0; i < uart_rx_index; i++)
    {
        hipnuc_input(&hipnuc_raw, uart_rx_buf[i]);
    }
    uart_rx_index = 0; // Reset buffer index after processing
}

/**
 * @brief System initialization
 * 
 * Initializes delay, NVIC priority, USART1 for debug output, USART2 for IMU data reception, 
 * and optionally DMA for USART2 reception. Also initializes the IMU instance.
 */
void app_init(void)
{
    /* Initialize instance */
    memset(&hipnuc_raw, 0, sizeof(hipnuc_raw_t));
    new_data_flag = 0;
}

/*============================== user code ==============================*/

float world_vel_x = 0.0f;//根据加速度计和四元数推算的世界坐标系下的机器人x轴速度，单位为m/s
float world_vel_y = 0.0f;//根据加速度计和四元数推算的世界坐标系下的机器人y轴速度，单位为m/s
float world_vel_z = 0.0f;//根据加速度计和四元数推算的世界坐标系下的机器人z轴速度，单位为m/s

/**
 * @brief 使用加速度计和四元数推算世界坐标系下的机器人三轴速度
 * @param dt 两帧数据之间的时间变化量(秒)
 */
void calculate_robot_velocity(float dt)
{
    // 获取四元数 (w, x, y, z)
    float w = hipnuc_raw.hi91.quat[0];
    float x = hipnuc_raw.hi91.quat[1];
    float y = hipnuc_raw.hi91.quat[2];
    float z = hipnuc_raw.hi91.quat[3];

    // 获取机体坐标系下的加速度 (通常HiPNUC返回的加速度单位为g)
    float acc_x = hipnuc_raw.hi91.acc[0];
    float acc_y = hipnuc_raw.hi91.acc[1];
    float acc_z = hipnuc_raw.hi91.acc[2];

    // 将加速度从机体坐标系转换到世界坐标系
    float world_acc_x = (1.0f - 2.0f * y * y - 2.0f * z * z) * acc_x + 
                        (2.0f * x * y - 2.0f * w * z) * acc_y + 
                        (2.0f * x * z + 2.0f * w * y) * acc_z;
                        
    float world_acc_y = (2.0f * x * y + 2.0f * w * z) * acc_x + 
                        (1.0f - 2.0f * x * x - 2.0f * z * z) * acc_y + 
                        (2.0f * y * z - 2.0f * w * x) * acc_z;
                        
    float world_acc_z = (2.0f * x * z - 2.0f * w * y) * acc_x + 
                        (2.0f * y * z + 2.0f * w * x) * acc_y + 
                        (1.0f - 2.0f * x * x - 2.0f * y * y) * acc_z;

    // 去除重力加速度 (假设Z轴向上)
    world_acc_z -= GRAVITY;

    // 对加速度进行积分，更新三轴速度
    world_vel_x += world_acc_x * dt;
    world_vel_y += world_acc_y * dt;
    world_vel_z += world_acc_z * dt;
}


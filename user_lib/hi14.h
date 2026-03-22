#ifndef _HI14_H_
#define _HI14_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* HiPNUC protocol constants */
#define HIPNUC_MAX_RAW_SIZE     (256)   /* Maximum size of raw message buffer */
#define UART_RX_BUF_SIZE        (1024)
#define LOG_STRING_SIZE         (1024)

/**
 * Packet 0x91: IMU data (floating point)
 */
typedef struct __attribute__((__packed__))
{
    uint8_t         tag;            /* Data packet tag, if tag = 0x00, means that this packet is null */
    uint16_t        main_status;    /* reserved */
    int8_t          temp;           /* Temperature */
    float           air_pressure;   /* Pressure */
    uint32_t        system_time;    /* Timestamp */
    float           acc[3];         /* Accelerometer data (x, y, z) */
    float           gyr[3];         /* Gyroscope data (x, y, z) */
    float           mag[3];         /* Magnetometer data (x, y, z) */
    float           roll;           /* Roll angle */
    float           pitch;          /* Pitch angle */
    float           yaw;            /* Yaw angle */
    float           quat[4];        /* Quaternion (w, x, y, z) */
} hi91_t;

/**
 * HiPNUC raw data structure
 */
typedef struct
{
    int nbyte;                          /* Number of bytes in message buffer */ 
    int len;                            /* Message length (bytes) */
    uint8_t buf[HIPNUC_MAX_RAW_SIZE];   /* Message raw buffer */
    hi91_t hi91;                        /* Decoded 0x91 packet data */
    // hi92_t hi92;                        /* Decoded 0x92 packet data */
    // hi81_t hi81;                        /* Decoded 0x81 packet data */
} hipnuc_raw_t;

int hipnuc_input(hipnuc_raw_t *raw, uint8_t data);
void process_data(void);
void app_init(void);

/*****************************************************/
extern uint8_t new_data_flag;
extern uint16_t uart_rx_index;
extern uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
extern uint8_t dma_rx_buf[UART_RX_BUF_SIZE];
extern hipnuc_raw_t hipnuc_raw;
/*****************************************************/

#ifdef __cplusplus
}
#endif

#endif


#ifndef __BT_CORE_H
#define __BT_CORE_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================================================
 * 1. 行为树核心基础定义
 * ================================================== */

/**
 * @brief BT节点状态枚举
*/
typedef enum {
    BT_SUCCESS,// BT成功
    BT_FAILURE,// BT失败
    BT_RUNNING // BT运行中
} BT_Status;

/**
 * @brief BT节点函数指针类型，所有BT节点函数都应符合此原型
*/
typedef BT_Status (*BT_NodeFunc)(void);

/* ==================================================
 * 2. 核心控制节点原型
 * ================================================== */

BT_Status BT_Sequence(const BT_NodeFunc* nodes, uint8_t count);
BT_Status BT_Sequence_Memory(const BT_NodeFunc* nodes, uint8_t count, uint8_t* memory_index);

BT_Status BT_Selector(const BT_NodeFunc* nodes, uint8_t count);
BT_Status BT_Selector_Memory(const BT_NodeFunc* nodes, uint8_t count, uint8_t* memory_index);

BT_Status BT_Parallel(const BT_NodeFunc* nodes, uint8_t count);


/* ==================================================
 * 3. 核心装饰节点原型 (无状态)
 * ================================================== */

BT_Status BT_Invert(BT_NodeFunc node);

BT_Status BT_ForceSuccess(BT_NodeFunc node);

#endif // __BT_CORE_H

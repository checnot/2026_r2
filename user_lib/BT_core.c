#include "BT_core.h"

/* ==================================================
 *      控制节点实现
 * ================================================== */

/**
 * @brief BT顺序节点 (Reactive Sequence)
 * @return 被tick后从0子节点开始，遇到 FAILURE 或 RUNNING 立即返回该状态，全部成功返回 SUCCESS
 */
BT_Status BT_Sequence_Reactive(const BT_NodeFunc* nodes, uint8_t count) 
{
    for (uint8_t i = 0; i < count; i++) {
        BT_Status status = nodes[i]();
        if (status != BT_SUCCESS) {
            return status;
        }
    }
    return BT_SUCCESS;
}

/**
 * @brief 带记忆的BT顺序节点 (Memory Sequence)
 * @param memory_index 供节点保存当前进度状态的外部变量指针
 */
BT_Status BT_Sequence_Memory(const BT_NodeFunc* nodes, uint8_t count, uint8_t* memory_index) 
{
    for (; *memory_index < count; ) {
        BT_Status status = nodes[*memory_index]();
        
        if (status == BT_SUCCESS) {
            (*memory_index)++; // 这个节点做完了，指向下一个
        } 
        else if (status == BT_RUNNING) {
            return BT_RUNNING; // 停在当前的 index 上，下个Tick直接从这里开始
        } 
        else {
            *memory_index = 0; // 遇到失败，重置记忆
            return BT_FAILURE;
        }
    }
    *memory_index = 0; // 整个序列全部成功完成，重置记忆
    return BT_SUCCESS;
}

/**
 * @brief BT选择节点 (Selector)
 * @return 遇到 SUCCESS 或 RUNNING 立即返回该状态，全部失败才返回 FAILURE
 */
BT_Status BT_Selector(const BT_NodeFunc* nodes, uint8_t count) 
{
    for (uint8_t i = 0; i < count; i++) {
        BT_Status status = nodes[i]();
        if (status != BT_FAILURE) {
            return status;
        }
    }
    return BT_FAILURE;
}

/**
 * @brief 带记忆的BT选择节点 (Memory Selector)
 * @param memory_index 供节点保存当前进度状态的外部变量指针
*/
BT_Status BT_Selector_Memory(const BT_NodeFunc* nodes, uint8_t count, uint8_t* memory_index) 
{
    for (; *memory_index < count; (*memory_index)++) {
        BT_Status status = nodes[*memory_index]();
        
        if (status == BT_RUNNING) {
            return BT_RUNNING; // 记住这个执行中的 Index，下个 Tick 直接跑它
        } 
        else if (status == BT_SUCCESS) {
            *memory_index = 0; // 成功了，重置
            return BT_SUCCESS;
        }
        // Failure 时继续 for 循环，查下一个
    }
    *memory_index = 0; // 全部失败，重置
    return BT_FAILURE;
}

/**
 * @brief BT并行节点 (Parallel)
 * @return 只要有一个失败就返回 FAILURE；必须全部成功才返回 SUCCESS；否则返回 RUNNING。
 */
BT_Status BT_Parallel(const BT_NodeFunc* nodes, uint8_t count) 
{
    uint8_t success_count = 0;
    
    for (uint8_t i = 0; i < count; i++) {
        BT_Status status = nodes[i]();
        // 严格模式：一处失败，整体失败
        if (status == BT_FAILURE) {
            return BT_FAILURE; 
        } else if (status == BT_SUCCESS) {
            success_count++;
        }
    }
    
    // 全部成功才返回成功
    if (success_count == count) {
        return BT_SUCCESS; 
    }
    
    // 还有节点没有执行完
    return BT_RUNNING; 
}


/* ==================================================
 *      装饰节点实现
 * ================================================== */

/**
 * @brief BT取反节点 (Inverter), 将子节点的 SUCCESS 转为 FAILURE，FAILURE 转为 SUCCESS，RUNNING 不变。
 */
BT_Status BT_Invert(BT_NodeFunc node) 
{
    BT_Status status = node();
    
    if (status == BT_SUCCESS) {
        return BT_FAILURE;
    } else if (status == BT_FAILURE) {
        return BT_SUCCESS;
    }
    
    return BT_RUNNING; 
}

/**
 * @brief BT强制成功节点 (Force Success), 只要子节点执行完成就返回 SUCCESS；如果子节点还在运行，则返回 RUNNING。
 */
BT_Status BT_ForceSuccess(BT_NodeFunc node) 
{
    BT_Status status = node();
    
    // 还没做完的事继续做
    if (status == BT_RUNNING) {
        return BT_RUNNING; 
    }
    
    // 不管是 FAILURE 还是 SUCCESS，只要结束了全当 SUCCESS
    return BT_SUCCESS; 
}

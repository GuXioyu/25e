#ifndef __LINE_H
#define __LINE_H

#include "stm32f4xx_hal.h"

/* ==================== 类型定义 ==================== */

/**
 * 赛道模式（单帧检测结果）
 * 基于 8 路数字量（0/1）的几何特征进行分类
 */
typedef enum {
    MODE_NORMAL,      // 正常单线（中间有黑线，两侧白）
    MODE_ALL_WHITE,   // 全部白（丢线或虚线间隙）
    MODE_ALL_BLACK,   // 全部黑（十字路口横线或起跑线）
    MODE_LEFT_LOST,   // 左侧两路全白，右侧有黑 → 车需右急转
    MODE_RIGHT_LOST,  // 右侧两路全白，左侧有黑 → 车需左急转
    MODE_TWO_LINES,   // 出现两段分离的黑线簇（Y 型分支）
    MODE_UNKNOWN      // 未定义（保留）
} TrackMode;

/**
 * 小车控制状态
 * 决定当前的转向和速度策略
 */
typedef enum {
    STATE_NORMAL,       // PID 巡线
    STATE_STRAIGHT,     // 强制直行（十字路口）
    STATE_HARD_LEFT,    // 强制左满舵（右直角弯）
    STATE_HARD_RIGHT,   // 强制右满舵（左直角弯）
    STATE_FOLLOW_LEFT,  // Y 分支跟随左侧线
    STATE_FOLLOW_RIGHT, // Y 分支跟随右侧线
    STATE_STOP          // 停车（严重丢线）
} CarState;

/* ==================== 函数声明 ==================== */

void Line_Timer(void);           // 定时器分频（1ms 时基，产生 10ms 标志）
void Line(void);                 // 主调度函数，每 10ms 由外部循环调用
void Line_Read_GraySensor(void); // 读取 8 路数字量和模拟量
void Line_Get_TrackMode(void);   // 模式识别（数字量 → TrackMode）
void Line_Get_CarState(void);    // 状态转移（含持续时间滤波）
void Line_Car_Control(void);     // 根据状态执行舵机/电机控制

#endif


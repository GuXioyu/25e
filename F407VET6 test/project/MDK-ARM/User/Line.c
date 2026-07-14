#include "Line.H"
#include "GraySensor.h"
#include "My_Uart.h"
#include "usart.h"

/* ==================== 全局变量 ==================== */

uint8_t line_timer_flag;          // 10ms 定时标志，由 Line_Timer 置 1，Line 清 0

uint8_t line_gray;                // 当前全局灰度阈值（由 GraySensor 提供，备用）
uint8_t line_gray_digital[8];     // 8 路数字量：0 = 白（未压线），1 = 黑（压线）
uint16_t line_gray_analog[8];     // 8 路灰度模拟量：0（最黑）~ 4095（最白）

TrackMode current_mode = MODE_NORMAL;   // 当前识别的赛道模式
CarState  current_state = STATE_NORMAL; // 当前小车的控制状态

/* ==================== 定时器分频 ==================== */

/**
 * 由 1ms 定时中断调用，产生 10ms 标志位
 * 同时维护 100ms 基准（保留）
 */
void Line_Timer(void)
{
    static uint8_t count_1ms, count_10ms, count_100ms;
    count_1ms++;
    count_10ms++;
    count_100ms++;

    if (count_10ms >= 10U) {
        count_10ms = 0U;
        line_timer_flag = 1;      // 通知主循环：10ms 到了
    }
}

/* ==================== 主调度 ==================== */

/**
 * 每 10ms 调用一次，依次执行：
 * 传感器读取 → 模式识别 → 状态转移 → 运动控制
 */
void Line(void)
{
    if (line_timer_flag) {
        line_timer_flag = 0;
        Line_Read_GraySensor();
        Line_Get_TrackMode();
        Line_Get_CarState();
        Line_Car_Control();
    }
}

/* ==================== 传感器读取 ==================== */

/**
 * 通过串口触发一次灰度传感器查询，并更新全局数组
 * 注意：假定查询后数据立即就绪（或中断已更新缓存）
 */
void Line_Read_GraySensor(void)
{
    line_gray = GraySensor_GetThreshold();   // 获取动态阈值（备用）
    GraySensor_SendQuery(&huart4);           // 触发 8 路传感器采样

    for (int i = 0; i < 8; i++) {
        line_gray_analog[i]  = GraySensor_GetAnalog(i);   // 模拟量
        line_gray_digital[i] = GraySensor_GetDigital(i);  // 数字量（阈值比较后）
    }
}

/* ==================== 模式识别（单帧） ==================== */

/**
 * 根据当前 line_gray_digital 数组判断赛道模式
 * 结果存入全局变量 current_mode
 * 每 10ms 执行一次，无内部计数器
 */
void Line_Get_TrackMode(void)
{
    int black_cnt = 0;                        // 统计黑点个数
    for (int i = 0; i < 8; i++) {
        if (line_gray_digital[i]) black_cnt++;
    }

    // --- 全白或全黑 ---
    if (black_cnt == 0) {
        current_mode = MODE_ALL_WHITE;        // 完全没有黑线
        return;
    }
    if (black_cnt == 8) {
        current_mode = MODE_ALL_BLACK;        // 全部压在黑线上
        return;
    }

    // --- 检测连续黑线段数（判断双线 / Y 分支） ---
    int segments = 0, in_seg = 0;
    for (int i = 0; i < 8; i++) {
        if (line_gray_digital[i] && !in_seg) {
            segments++;
            in_seg = 1;
        } else if (!line_gray_digital[i]) {
            in_seg = 0;
        }
    }
    if (segments >= 2) {
        current_mode = MODE_TWO_LINES;        // 至少两个独立黑区
        return;
    }

    // --- 单侧丢线检测（直角弯特征） ---
    int left_edge_white  = (line_gray_digital[0] == 0 && line_gray_digital[1] == 0);
    int right_edge_white = (line_gray_digital[6] == 0 && line_gray_digital[7] == 0);

    if (left_edge_white && !right_edge_white && black_cnt >= 3) {
        current_mode = MODE_LEFT_LOST;        // 线在右侧，车应右转
    } else if (right_edge_white && !left_edge_white && black_cnt >= 3) {
        current_mode = MODE_RIGHT_LOST;       // 线在左侧，车应左转
    } else {
        current_mode = MODE_NORMAL;           // 正常单线
    }
}

/* ==================== 状态转移（含持续时间滤波） ==================== */

/**
 * 根据连续的模式持续时间和当前状态，决定小车的下一控制状态
 * 所有计数器均为静态局部变量，函数外不可见
 * 每 10ms 调用一次
 */
void Line_Get_CarState(void)
{
    // --- 持续时间计数器（静态，跨调用保持） ---
    static uint16_t white_count      = 0;    // 全白持续周期
    static uint16_t black_count      = 0;    // 全黑持续周期
    static uint16_t edge_lost_count  = 0;    // 单侧丢线持续周期
    static uint16_t state_count      = 0;    // 当前状态已持续周期

    // --- 更新特征持续时间 ---
    if (current_mode == MODE_ALL_WHITE) {
        white_count++;
    } else {
        white_count = 0;
    }

    if (current_mode == MODE_ALL_BLACK) {
        black_count++;
    } else {
        black_count = 0;
    }

    if (current_mode == MODE_LEFT_LOST || current_mode == MODE_RIGHT_LOST) {
        edge_lost_count++;
    } else {
        edge_lost_count = 0;
    }

    state_count++;   // 本状态持续周期 +1

    // --- 状态转移逻辑 ---
    switch (current_state) {

        case STATE_NORMAL:
            // 十字路口：全黑持续 ≥ 2 周期（20ms）
            if (current_mode == MODE_ALL_BLACK && black_count >= 2) {
                current_state = STATE_STRAIGHT;
            }
            // 左直角弯：左侧丢线 ≥ 1 周期（10ms）
            else if (current_mode == MODE_LEFT_LOST && edge_lost_count >= 1) {
                current_state = STATE_HARD_RIGHT;
            }
            // 右直角弯：右侧丢线 ≥ 1 周期
            else if (current_mode == MODE_RIGHT_LOST && edge_lost_count >= 1) {
                current_state = STATE_HARD_LEFT;
            }
            // Y 分支：立即触发（双线模式非常可靠）
            else if (current_mode == MODE_TWO_LINES) {
                current_state = STATE_FOLLOW_LEFT;   // 默认走左分支
            }
            // 真丢线：全白持续 ≥ 5 周期（50ms）
            else if (current_mode == MODE_ALL_WHITE && white_count >= 5) {
                current_state = STATE_STOP;
            }
            // 其他情况（包括短时全白、单帧噪声）保持 NORMAL
            break;

        case STATE_STRAIGHT:
            // 退出条件：重新见到正常单线 或 超时 300ms (30 周期)
            if (current_mode == MODE_NORMAL || state_count >= 30) {
                current_state = STATE_NORMAL;
            }
            break;

        case STATE_HARD_LEFT:
        case STATE_HARD_RIGHT:
            // 退出条件：找回黑线 或 超时 500ms (50 周期)
            if (current_mode == MODE_NORMAL || state_count >= 50) {
                current_state = STATE_NORMAL;
            }
            break;

        case STATE_FOLLOW_LEFT:
        case STATE_FOLLOW_RIGHT:
            // Y 分支结束：恢复单线 或 超时 1000ms (100 周期)
            if (current_mode == MODE_NORMAL || state_count >= 100) {
                current_state = STATE_NORMAL;
            }
            break;

        case STATE_STOP:
            // 停车后不再自动恢复，可在此加入外部按键复位等
            break;
    }

    // 状态切换时重置 state_count
    static CarState prev_state = STATE_NORMAL;
    if (current_state != prev_state) {
        state_count = 0;
        prev_state = current_state;
    }
}

/* ==================== 运动控制 ==================== */

/**
 * 根据当前控制状态（current_state）执行具体的舵机/电机指令
 * 本函数仅提供框架，内部的具体函数（如 PID_Compute）需自行实现
 */
void Line_Car_Control(void)
{
    switch (current_state) {

        case STATE_NORMAL:
            // 正常 PID 巡线：
            // 1. 根据 line_gray_analog 或 line_gray_digital 计算线位置偏差
            // 2. 偏差送入 PID，输出转向量 steering
            // 3. 设定基础速度
            // 例：
            // float error = compute_line_position(line_gray_digital) - CENTER;
            // float steering = PID_Compute(error);
            // set_steering(steering);
            // set_speed(BASE_SPEED);
            break;

        case STATE_STRAIGHT:
            // 强制直行（十字路口）：
            // 舵机归零，速度可稍降以防打滑
            // set_steering(0.0f);
            // set_speed(STRAIGHT_SPEED);
            break;

        case STATE_HARD_LEFT:
            // 左满舵（过右直角）：
            // 转向打满左极限，减速过弯
            // set_steering(MAX_LEFT);
            // set_speed(TURN_SPEED);
            break;

        case STATE_HARD_RIGHT:
            // 右满舵（过左直角）：
            // 转向打满右极限，减速过弯
            // set_steering(MAX_RIGHT);
            // set_speed(TURN_SPEED);
            break;

        case STATE_FOLLOW_LEFT:
            // Y 分支跟随左线：
            // 仅用左半侧传感器计算线位置（屏蔽右侧）
            // 使用 PID 或特殊跟随逻辑
            // set_steering(PID_Left(...));
            // set_speed(BRANCH_SPEED);
            break;

        case STATE_FOLLOW_RIGHT:
            // Y 分支跟随右线：
            // 仅用右半侧传感器计算线位置
            // set_steering(PID_Right(...));
            // set_speed(BRANCH_SPEED);
            break;

        case STATE_STOP:
            // 停车：
            // 转向归零，速度为零，可关闭电机使能
            // set_steering(0.0f);
            // set_speed(0.0f);
            break;
    }
}

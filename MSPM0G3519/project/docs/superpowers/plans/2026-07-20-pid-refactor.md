# PID 模块重构 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 重构位置式 PID 模块，使 P、I、D 项使用独立模式字段并消除实例间运行状态串扰。

**Architecture:** `PID_t` 同时保存 PID 参数和单个控制器的运行状态，`PID_Update()` 在一个确定的更新顺序中计算误差、P 项、I 项、D 项和最终输出。模式常量使用互斥数值而非可叠加位掩码：P 项只有禁用/启用，I、D 项各在禁用和三种策略中选择一种。

**Tech Stack:** C11 风格 C 源码、MSPM0G3519 MDK 工程、`math.h` 的 `fabsf()`。

---

### Task 1: 重定义 PID 配置与实例状态

**Files:**
- Modify: `C:\Users\Chen\Desktop\Github\25e\MSPM0G3519\project\code\PID.h:8-49`
- Test: `C:\Users\Chen\Desktop\Github\25e\MSPM0G3519\project\mdk\SeekFree_MSPM0G3519_Device_Library.uvprojx`

- [ ] **Step 1: 确认现有宏和字段不能表达互斥模式**

  在 `PID.h` 中记录现状：`PID_DIFMODE_IMPERFECT` 的值为 `0x03U`，与普通和微分先行位掩码重叠；`intmode` 可把多种积分策略同时置位。这一检查的预期结论是旧接口不满足“模式三选一”。

- [ ] **Step 2: 改写模式常量和结构体字段**

  将原有 `intmode`、`difmode` 及其宏替换为以下定义；宏、字段行和函数声明均使用 UTF-8 中文说明：

  ```c
  #define PID_PMODE_DISABLE          (0U) /* 禁用比例项 */
  #define PID_PMODE_ENABLE           (1U) /* 启用比例项 */
  #define PID_IMODE_DISABLE          (0U) /* 禁用积分项 */
  #define PID_IMODE_NORMAL           (1U) /* 普通积分 */
  #define PID_IMODE_VAR_NONLINEAR    (2U) /* 变速非线性积分 */
  #define PID_IMODE_SEPARATION       (3U) /* 积分分离 */
  #define PID_DMODE_DISABLE          (0U) /* 禁用微分项 */
  #define PID_DMODE_NORMAL           (1U) /* 普通微分 */
  #define PID_DMODE_FRONT            (2U) /* 微分先行 */
  #define PID_DMODE_IMPERFECT        (3U) /* 不完全微分 */

  typedef struct
  {
      float kp;                       /* 比例系数 */
      float ki;                       /* 积分系数 */
      float kd;                       /* 微分系数 */
      float target;                   /* 目标值 */
      float actual_current;           /* 当前测量值 */
      float actual_past;              /* 上次测量值 */
      float out;                      /* 本次输出值 */
      float error_current;            /* 当前误差 */
      float error_past;               /* 上次误差 */
      float error_int;                /* 误差积分值 */
      float dif_out;                  /* 微分项历史输出 */
      uint8_t mode_p;                 /* 比例项模式 */
      uint8_t mode_i;                 /* 积分项模式 */
      uint8_t mode_d;                 /* 微分项模式 */
      float intk_nonline;             /* 非线性积分系数 */
      float int_sep_threshold;        /* 积分分离阈值 */
      float error_intmax;             /* 积分上限 */
      float error_intmin;             /* 积分下限 */
      float dif_filter;               /* 微分滤波系数，范围 0 至 1 */
      float out_offset;               /* 非零输出偏置 */
      float outmax;                   /* 输出上限 */
      float outmin;                   /* 输出下限 */
      float out_deadzone;             /* 输入误差死区 */
  } PID_t;

  //更新一次 PID 控制器输出
  void PID_Update(PID_t *p);
  ```

- [ ] **Step 3: 编译检查头文件接口**

  在 MDK 中重新编译包含 `PID.h` 的目标，预期结果是此阶段可能因未同步 `PID.c` 和外部旧字段引用而失败；失败信息必须只用于确认后续修改点，不能修改 `Gimbal.c`、`Task.c` 或 `main.c`。

- [ ] **Step 4: 提交头文件修改**

  ```powershell
  git add MSPM0G3519/project/code/PID.h
  git commit -m "refactor: define PID term modes"
  ```

### Task 2: 以每实例状态重写 PID 更新流程

**Files:**
- Modify: `C:\Users\Chen\Desktop\Github\25e\MSPM0G3519\project\code\PID.c:1-99`
- Test: `C:\Users\Chen\Desktop\Github\25e\MSPM0G3519\project\mdk\SeekFree_MSPM0G3519_Device_Library.uvprojx`

- [ ] **Step 1: 在调试器中准备三个 PID 实例场景**

  使用以下三个零初始化实例和输入，记录 `out`、`error_int`、`dif_out`：

  ```c
  PID_t POnly = { .kp = 2.0f, .target = 10.0f, .actual_current = 7.0f,
                  .mode_p = PID_PMODE_ENABLE, .outmax = 100.0f, .outmin = -100.0f };
  PID_t INormal = { .ki = 1.0f, .target = 10.0f, .actual_current = 0.0f,
                    .mode_i = PID_IMODE_NORMAL, .error_intmax = 3.0f,
                    .error_intmin = -3.0f, .outmax = 100.0f, .outmin = -100.0f };
  PID_t DFront = { .kd = 1.0f, .actual_current = 4.0f, .actual_past = 1.0f,
                   .mode_d = PID_DMODE_FRONT, .outmax = 100.0f, .outmin = -100.0f };
  ```

  预期第一次更新后分别为 `POnly.out == 6.0f`、`INormal.error_int == 3.0f` 且 `INormal.out == 3.0f`、`DFront.out == -3.0f`。该检查在实现前用于定义目标行为。

- [ ] **Step 2: 删除共享临时变量并按模式计算 P、I、D 项**

  用以下完整更新逻辑替换 `PID.c` 中的文件级 `intC`、`difout` 与旧 `PID_Update()`；所有功能段保留中文注释，函数定义行右侧写明用途：

  ```c
  #include "PID.h"

  /**
   * @brief 更新一次位置式 PID 控制输出
   * @param p PID 控制器实例指针
   * @return 无
   */
  void PID_Update(PID_t *p) //更新一次 PID 控制器输出
  {
      float ProportionalOut = 0.0f; //保存本次比例项输出
      float IntegralOut = 0.0f; //保存本次积分项输出

      /* 更新本次控制所需的误差历史 */
      p->error_past = p->error_current; //保存上次误差
      p->error_current = p->target - p->actual_current; //计算当前误差

      /* 按比例模式计算比例项 */
      if (p->mode_p == PID_PMODE_ENABLE)
      {
          ProportionalOut = p->kp * p->error_current; //生成比例项输出
      }

      /* 按积分模式累积误差并始终执行积分限幅 */
      if ((p->mode_i == PID_IMODE_DISABLE) || (p->ki == 0.0f))
      {
          p->error_int = 0.0f; //禁用积分或积分系数为零时清除积分状态
      }
      else if (p->mode_i == PID_IMODE_NORMAL)
      {
          p->error_int += p->error_current; //按普通积分累积误差
      }
      else if (p->mode_i == PID_IMODE_VAR_NONLINEAR)
      {
          p->error_int += p->error_current / (p->intk_nonline * fabsf(p->error_current) + 1.0f); //按非线性系数累积误差
      }
      else if (p->mode_i == PID_IMODE_SEPARATION)
      {
          if (fabsf(p->error_current) > p->int_sep_threshold)
          {
              p->error_int = 0.0f; //误差超过阈值时清除积分状态
          }
          else
          {
              p->error_int += p->error_current; //误差在阈值内时累积积分
          }
      }
      else
      {
          p->error_int = 0.0f; //无效积分模式时禁止保留积分状态
      }
      if (p->error_int > p->error_intmax)
      {
          p->error_int = p->error_intmax; //限制积分上限
      }
      if (p->error_int < p->error_intmin)
      {
          p->error_int = p->error_intmin; //限制积分下限
      }
      IntegralOut = p->ki * p->error_int; //生成积分项输出

      /* 按微分模式生成每实例微分输出 */
      if ((p->mode_d == PID_DMODE_DISABLE) || (p->kd == 0.0f))
      {
          p->dif_out = 0.0f; //禁用微分或微分系数为零时清除微分状态
      }
      else if (p->mode_d == PID_DMODE_NORMAL)
      {
          p->dif_out = p->kd * (p->error_current - p->error_past); //按误差变化计算普通微分
      }
      else if (p->mode_d == PID_DMODE_FRONT)
      {
          p->dif_out = -p->kd * (p->actual_current - p->actual_past); //按测量值变化计算微分先行
      }
      else if (p->mode_d == PID_DMODE_IMPERFECT)
      {
          p->dif_out = (1.0f - p->dif_filter) * p->kd * (p->error_current - p->error_past)
                     + p->dif_filter * p->dif_out; //使用历史微分输出进行一阶滤波
      }
      else
      {
          p->dif_out = 0.0f; //无效微分模式时禁止微分输出
      }
      p->actual_past = p->actual_current; //保存当前测量值供下一次微分使用

      /* 合成输出并保留原有执行器处理能力 */
      p->out = ProportionalOut + IntegralOut + p->dif_out; //合成三项控制输出
      if (p->out > 0.0f)
      {
          p->out += p->out_offset; //为正向非零输出补偿偏置
      }
      else if (p->out < 0.0f)
      {
          p->out -= p->out_offset; //为反向非零输出补偿偏置
      }
      if (p->out > p->outmax)
      {
          p->out = p->outmax; //限制输出上限
      }
      if (p->out < p->outmin)
      {
          p->out = p->outmin; //限制输出下限
      }
      if (fabsf(p->error_current) < p->out_deadzone)
      {
          p->out = 0.0f; //误差进入死区时停止输出
      }
  }
  ```

- [ ] **Step 3: 重新运行三个 PID 实例场景**

  在 MDK 调试器中按 Step 1 的输入分别调用 `PID_Update()`。预期为 `6.0f`、积分钳位后的 `3.0f`、`-3.0f`，并且修改 `DFront.dif_out` 不会改变 `POnly` 或 `INormal` 的任何状态。

- [ ] **Step 4: 验证互斥与禁用路径**

  分别设置 `mode_p`、`mode_i`、`mode_d` 为 `PID_*MODE_DISABLE` 后调用 `PID_Update()`，预期每个被禁用项不参与 `out`；将 `mode_i` 或 `mode_d` 设置为 `4U`，预期对应历史状态清零且不产生输出。设置积分初值为 `10.0f`、上下限为 `±3.0f`，预期一次更新后积分值为 `3.0f`。

- [ ] **Step 5: 编译 PID 所在工程**

  在 MDK 中打开工程并执行 Rebuild。预期是 `PID.c` 自身无编译错误；外部模块仍引用旧 `intmode`、`difmode` 或旧模式宏时，记录这些报错并停止，不修改这些调用文件，因为它们不在本次范围内。

- [ ] **Step 6: 提交源文件修改**

  ```powershell
  git add MSPM0G3519/project/code/PID.c
  git commit -m "refactor: isolate PID control state"
  ```

### Task 3: 更新任务记录并交付验证结果

**Files:**
- Modify: `C:\Users\Chen\Desktop\Github\25e\MSPM0G3519\project\docs\tasks\2026-07-20-pid-refactor.md`
- Test: `C:\Users\Chen\Desktop\Github\25e\MSPM0G3519\project\docs\tasks\2026-07-20-pid-refactor.md`

- [ ] **Step 1: 记录实际验证结果**

  将“验证方法”替换为实际使用的 MDK 工程名称、编译命令或界面操作、三个实例场景的观察值，以及是否存在外部旧接口编译错误。无法构建时，明确写入原因、剩余风险和上述调试器人工验证步骤。

- [ ] **Step 2: 核查文档与已确认范围一致**

  读取任务文档，确认它仍明确写有“不修改外部调用模块”“积分限幅始终启用”“不提供 `PID_Reset()`”。预期不包含“待定”“TODO”或与设计相矛盾的描述。

- [ ] **Step 3: 提交任务记录**

  ```powershell
  git add MSPM0G3519/project/docs/tasks/2026-07-20-pid-refactor.md
  git commit -m "docs: record PID refactor verification"
  ```

# PID 前馈项扩展 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为位置式 PID 增加由 `mode_f` 控制的独立前馈项。

**Architecture:** 在 `PID_t` 中增加前馈模式和系数，不增加前馈历史状态。`PID_Update()` 在误差更新后计算 `kf × (error_current - error_past)`，与 P、I、D 项合成后复用既有输出处理链。

**Tech Stack:** C11 风格 C、MSPM0G3519 MDK 工程、GCC 主机临时测试。

---

### Task 1: 用测试定义前馈模式行为

**Files:**
- Create: `C:\Users\Chen\AppData\Local\Temp\pid_feedforward_test.c`
- Test: `C:\Users\Chen\AppData\Local\Temp\pid_feedforward_test.exe`

- [ ] **Step 1: 写入失败的前馈接口测试**

  创建以下临时测试；当前头文件不含 `mode_f`、`kf` 和 `PID_FMODE_ENABLE`，预期无法编译：

  ```c
  #include <assert.h>
  #include <math.h>
  #include "PID.h"

  static void PID_TestClose(float Actual, float Expected) //检查浮点结果是否符合预期
  {
      assert(fabsf(Actual - Expected) < 0.0001f); //限制浮点误差
  }

  int main(void) //执行前馈项临时测试
  {
      PID_t Feedforward = {0}; //准备前馈测试控制器

      Feedforward.target = 5.0f; //设置目标值
      Feedforward.actual_current = 2.0f; //设置当前测量值
      Feedforward.error_current = 1.0f; //设置上次误差
      Feedforward.kf = 2.0f; //设置前馈系数
      Feedforward.mode_f = PID_FMODE_ENABLE; //启用前馈项
      Feedforward.outmax = 100.0f; //设置输出上限
      Feedforward.outmin = -100.0f; //设置输出下限
      PID_Update(&Feedforward); //计算前馈输出
      PID_TestClose(Feedforward.out, 4.0f); //验证前馈输出

      Feedforward.mode_f = PID_FMODE_DISABLE; //禁用前馈项
      PID_Update(&Feedforward); //重新计算输出
      PID_TestClose(Feedforward.out, 0.0f); //验证禁用前馈项不产生输出

      Feedforward.mode_p = PID_PMODE_ENABLE; //启用比例项
      Feedforward.kp = 1.0f; //设置比例系数
      Feedforward.mode_f = PID_FMODE_ENABLE; //重新启用前馈项
      Feedforward.error_current = 1.0f; //恢复上次误差
      PID_Update(&Feedforward); //计算比例和前馈合成输出
      PID_TestClose(Feedforward.out, 7.0f); //验证 P 与 F 项共同参与输出

      return 0; //返回测试成功状态
  }
  ```

- [ ] **Step 2: 运行测试并确认其因新接口缺失失败**

  ```powershell
  gcc -std=c11 -Wall -Wextra -Werror -I ..\code C:\Users\Chen\AppData\Local\Temp\pid_feedforward_test.c ..\code\PID.c -lm -o C:\Users\Chen\AppData\Local\Temp\pid_feedforward_test.exe
  ```

  预期：退出码非 0，错误指出 `PID_t` 缺少 `mode_f`、`kf` 或 `PID_FMODE_ENABLE` 未定义。

### Task 2: 在 PID 文件对中实现前馈项

**Files:**
- Modify: `C:\Users\Chen\Desktop\Github\25e\MSPM0G3519\project\code\PID.h:18-51`
- Modify: `C:\Users\Chen\Desktop\Github\25e\MSPM0G3519\project\code\PID.c:9-99`
- Test: `C:\Users\Chen\AppData\Local\Temp\pid_feedforward_test.exe`

- [ ] **Step 1: 在 `PID.h` 增加前馈模式和配置字段**

  在微分模式宏后增加：

  ```c
  #define PID_FMODE_DISABLE          (0U) /* 禁用前馈项 */
  #define PID_FMODE_ENABLE           (1U) /* 启用前馈项 */
  ```

  在 `mode_d` 后、积分配置字段前增加：

  ```c
  uint8_t mode_f; //前馈项模式
  float kf; //前馈系数
  ```

- [ ] **Step 2: 在 `PID.c` 计算并合成前馈输出**

  在局部输出变量区域新增：

  ```c
  float FeedforwardOut = 0.0f; //保存本次前馈项输出
  ```

  在比例项计算段后、积分项计算段前新增：

  ```c
  /* 按前馈模式计算前馈项 */
  if (p->mode_f == PID_FMODE_ENABLE)
  {
      FeedforwardOut = p->kf * (p->error_current - p->error_past); //按误差变化生成前馈项输出
  }
  ```

  将最终输出合成语句改为：

  ```c
  p->out = ProportionalOut + IntegralOut + p->dif_out + FeedforwardOut; //合成四项控制输出
  ```

  保留当前工作区中 PID 文件已有的用户注释和空白改动，不改变 P、I、D、输出偏置、限幅或死区逻辑。

- [ ] **Step 3: 重新编译并运行前馈测试**

  ```powershell
  gcc -std=c11 -Wall -Wextra -Werror -I ..\code C:\Users\Chen\AppData\Local\Temp\pid_feedforward_test.c ..\code\PID.c -lm -o C:\Users\Chen\AppData\Local\Temp\pid_feedforward_test.exe
  C:\Users\Chen\AppData\Local\Temp\pid_feedforward_test.exe
  ```

  预期：退出码为 0；前馈启用结果为 `4.0f`，禁用结果为 `0.0f`，P/F 合成结果为 `7.0f`。

- [ ] **Step 4: 提交 PID 文件对**

  ```powershell
  git add MSPM0G3519/project/code/PID.h MSPM0G3519/project/code/PID.c
  git commit -m "feat: add PID feedforward term"
  ```

### Task 3: 记录验证并清理临时测试

**Files:**
- Modify: `C:\Users\Chen\Desktop\Github\25e\MSPM0G3519\project\docs\tasks\2026-07-20-pid-feedforward.md`

- [ ] **Step 1: 更新任务文档的实际验证结果**

  写入 GCC 测试命令及其结果，明确 MDK 命令行工具不可用时未执行工程级构建，并给出在 `mdk/SeekFree_MSPM0G3519_Device_Library.uvprojx` 中单步检查前馈启用、禁用与输出合成的人工方法。

- [ ] **Step 2: 删除临时测试源码与可执行文件**

  删除 `C:\Users\Chen\AppData\Local\Temp\pid_feedforward_test.c` 和 `C:\Users\Chen\AppData\Local\Temp\pid_feedforward_test.exe`，确认它们不进入 Git 工作区。

- [ ] **Step 3: 提交任务记录**

  ```powershell
  git add MSPM0G3519/project/docs/tasks/2026-07-20-pid-feedforward.md
  git commit -m "docs: record PID feedforward verification"
  ```

# Motor Compare Initial Zero Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 `Motor_CompareLastValue()` 将未记录过的通道视为历史值 0，从而将首次非零输入识别为变化。

**Architecture:** 保持既有函数签名及调用方不变。函数先比较本次输入与每通道的静态历史值，再更新历史值；静态数组的 C 语言零初始化提供首次比较基准。

**Tech Stack:** C11、Keil MDK 工程、MSPM0 固件。

---

### Task 1: 验证现有行为会丢弃首次非零输入

**Files:**
- Modify: `C:/Users/Chen/Desktop/Github/25e/MSPM0G3519/project/code/Motor.c:151-170`
- Modify: `C:/Users/Chen/Desktop/Github/25e/MSPM0G3519/project/code/Motor.h:138-141`
- Verify: MDK 工程的现有构建目标

- [x] **Step 1: 在调试器中设定调用序列作为失败用例**

在 `Motor_CompareLastValue()` 的入口观察通道 2，并依次以 `0`、`20`、`20`、`-20` 调用。当前实现首次 `20` 返回 0，不能满足预期序列 `0, 1, 0, 1`。

- [x] **Step 2: 记录失败证据**

预期当前失败点：首次输入 `20` 进入 `Motor_LastCompareValid[channel] == 0U` 分支并返回 0。

### Task 2: 修改比较函数与公共说明

**Files:**
- Modify: `C:/Users/Chen/Desktop/Github/25e/MSPM0G3519/project/code/Motor.c:141-170`
- Modify: `C:/Users/Chen/Desktop/Github/25e/MSPM0G3519/project/code/Motor.h:138-141`

- [x] **Step 1: 删除首次调用特殊分支**

将函数主体改为先比较后更新：

```c
result = (value != Motor_LastCompareValue[channel]) ? 1U : 0U;
Motor_LastCompareValue[channel] = value;
return result;
```

保留通道越界直接返回 0 的分支。删除 `Motor_LastCompareValid` 的定义及所有使用点，避免保留不再使用的状态。

- [x] **Step 2: 更新函数说明**

将声明与定义的说明统一为：首次调用按历史值 0 比较；本次值不同返回 1，相同返回 0；完成比较后保存本次值。

### Task 3: 验证修改与构建

**Files:**
- Verify: `C:/Users/Chen/Desktop/Github/25e/MSPM0G3519/project/code/Motor.c`
- Verify: `C:/Users/Chen/Desktop/Github/25e/MSPM0G3519/project/code/Motor.h`

- [x] **Step 1: 重复调用序列验证**

在调试器中执行 `0, 20, 20, -20`，确认返回值为 `0, 1, 0, 1`。另以无效通道调用，确认返回 0 且有效通道的历史值未变化。

- [ ] **Step 2: 构建 MDK 工程（当前环境缺少 Keil 命令行工具，未执行）**

运行项目已有的非交互式构建命令；若环境没有 Keil 命令行构建工具，记录未执行原因，并以 MDK 打开工程、Rebuild 后零错误作为人工验证方法。

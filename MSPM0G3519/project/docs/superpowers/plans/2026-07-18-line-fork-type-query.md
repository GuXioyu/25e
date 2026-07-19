# Line Fork Type Query Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在循迹模块中增加只读岔道类型查询接口，供外部判断当前是否为左直角岔道。

**Architecture:** `Line.h` 暴露以 `LineForkType_t` 为输入的查询函数。`Line.c` 仅读取内部 `line_fork_type` 并返回匹配结果，不写入岔道状态，因此不会影响现有识别、左转完成或速度控制流程。

**Tech Stack:** MSPM0G3519 C 固件、Keil MDK 工程。

---

### Task 1: 增加岔道类型只读查询接口

**Files:**
- Modify: `code/Line.h`（在 `Line_GetFlag()` 声明后）
- Modify: `code/Line.c`（在 `Line_GetFlag()` 定义后）
- Test: MDK 工程构建

- [ ] **Step 1: 编写接口行为检查用例**

当前工程没有可独立运行的 `Line` 单元测试入口，使用以下静态检查代替：

```powershell
rg -n -A 14 'uint8_t Line_GetForkType\(LineForkType_t Type\)' code/Line.c
```

预期：实现尚不存在，命令无匹配输出。

- [ ] **Step 2: 在 `code/Line.h` 声明接口**

在 `Line_GetFlag()` 声明后增加：

```c
/**
 * @brief 查询当前岔道类型是否与指定类型一致
 * @param Type 待查询的岔道类型
 * @return 1 表示当前类型匹配；0 表示当前类型不匹配
 */
uint8_t Line_GetForkType(LineForkType_t Type); //查询当前岔道类型是否匹配
```

- [ ] **Step 3: 在 `code/Line.c` 实现只读比较**

在 `Line_GetFlag()` 定义后增加：

```c
/**
 * @brief 查询当前岔道类型是否与指定类型一致
 * @param Type 待查询的岔道类型
 * @return 1 表示当前类型匹配；0 表示当前类型不匹配
 */
uint8_t Line_GetForkType(LineForkType_t Type) //查询当前岔道类型是否匹配
{
    //仅比较当前岔道类型，避免查询操作改变岔道控制状态
    if (line_fork_type == Type)
    {
        return 1U; //通知调用方当前岔道类型匹配
    }

    return 0U; //通知调用方当前岔道类型不匹配
}
```

- [ ] **Step 4: 运行静态行为检查**

```powershell
rg -n -A 14 'uint8_t Line_GetForkType\(LineForkType_t Type\)' code/Line.c
```

预期：函数仅比较 `line_fork_type == Type` 并返回 `1U` 或 `0U`，没有 `line_fork_type =` 赋值。

- [ ] **Step 5: 构建 MDK 工程**

```powershell
& 'C:\Keil_v5\UV4\UV4.exe' -b 'mdk\SeekFree_MSPM0G3519_Device_Library.uvprojx'
```

预期：工程构建完成且无编译错误；若本机未安装 Keil 或命令不可用，记录原因并进行声明/定义一致性的静态检查。

- [ ] **Step 6: 复核改动范围**

```powershell
git diff --check -- code/Line.c code/Line.h docs/tasks/2026-07-18-line-fork-type-query.md
git diff -- code/Line.c code/Line.h
```

预期：仅增加 `Line_GetForkType()` 的声明和只读实现；不修改现有岔道状态机逻辑。


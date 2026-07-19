# Cam 注释补全与逻辑修复实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复 Cam 模块编译和目标坐标读取逻辑，补齐中文注释，并保留无数据通路的激光扩展接口。

**Architecture:** Cam 模块继续从 UART4 的完整帧缓存中解析 `[cam,x,y]`。模块只维护目标坐标及其新数据标志；`CAM_LASER` 仍是公开类型值，但在未定义激光帧格式前明确返回失败且不改变目标数据状态。

**Tech Stack:** C11、Keil MDK/ARMCLANG、逐飞 UART 兼容层。

---

## 文件结构

- 修改：`code/Cam.h`，公开 Cam 模块类型常量和函数声明，并补齐中文注释。
- 修改：`code/Cam.c`，实现 UART4 坐标帧解析、目标数据读取及参数校验。
- 不新增自动化测试文件：工程没有主机测试框架，且本次仅允许操作 `Cam.c/.h` 一对代码文件。以现有 Keil 构建错误作为 RED 验证，以修复后的完整构建和静态输入输出核对作为 GREEN 验证。

### Task 1: 记录现有编译失败（RED）

**Files:**

- Test: `mdk/Objects/SeekFree_MSPM0G3519_Device_Library.build_log.htm`

- [x] **Step 1: 构建未修改工程并记录 Cam.c 的失败原因**

Run:

```powershell
& 'D:\Academy\Keil\UV4\UV4.exe' -b '.\SeekFree_MSPM0G3519_Device_Library.uvprojx'
```

Expected: 构建失败，日志包含 `Cam.c(40): error: function definition is not allowed here` 和 `Cam.c(53): error: expected '}'`；该失败证明测试对象是 Cam.c 缺失的闭合花括号。

### Task 2: 补齐 Cam 公共接口

**Files:**

- Modify: `code/Cam.h`

- [x] **Step 1: 声明类型常量与两个公共函数，并写中文接口注释**

```c
#define CAM_TARGERT (1U)  //目标坐标数据类型
#define CAM_LASER   (2U)  //激光坐标预留数据类型

//解析 UART4 接收的目标坐标帧
void Cam_GetXY(void);

//读取指定类型的最新坐标
uint8_t Cam_LoadXY(uint16 *X, uint16 *Y, uint8_t Type);
```

Expected: 头文件可以让调用方按当前接口调用 `Cam_GetXY()` 与 `Cam_LoadXY()`；`CAM_LASER` 仍可供未来代码使用。

### Task 3: 修复 Cam 数据解析和读取逻辑

**Files:**

- Modify: `code/Cam.c`

- [x] **Step 1: 以严格整数解析函数替换浮点转换，并在范围无效时拒绝提交坐标**

```c
static uint8_t Cam_ParseCoordinate(const char *Text, uint16 *Coordinate)  //解析单个无符号坐标
{
    char *End;                                                           //数值后的首个字符位置
    unsigned long Value;                                                 //转换后的坐标数值

    if ((Text == NULL) || (Coordinate == NULL))                          //拒绝空输入或空输出地址
    {
        return 0U;                                                       //通知坐标无效
    }

    Value = strtoul(Text, &End, 10);                                     //按十进制转换坐标文本
    if ((*Text == '\0') || (*End != '\0') || (Value > UINT16_MAX))       //拒绝空文本、非整数和超范围值
    {
        return 0U;                                                       //保持原有坐标不变
    }

    *Coordinate = (uint16)Value;                                        //输出已校验的坐标
    return 1U;                                                           //通知转换成功
}
```

- [x] **Step 2: 仅在 UART4 已收到完整帧时解析 `[cam,x,y]`，并只在两个坐标均有效时提交目标数据**

```c
if (Serial_GetRxFlag(&CAM_HUART) == 0U)                                 //未收到完整帧时不访问接收缓存
{
    return;                                                              //结束本次轮询
}

Tag = strtok((char *)Serial_GetRxPacket(&CAM_HUART), ",");             //分离帧标签
if ((Tag == NULL) || (strcmp(Tag, "cam") != 0))                          //拒绝非视觉坐标帧
{
    return;                                                              //保持已保存坐标不变
}

ValueX = strtok(NULL, ", ");                                            //读取 X 坐标字段
ValueY = strtok(NULL, ", ");                                            //读取 Y 坐标字段
if ((Cam_ParseCoordinate(ValueX, &TargetX) == 0U) ||                     //校验 X 坐标
    (Cam_ParseCoordinate(ValueY, &TargetY) == 0U))                       //校验 Y 坐标
{
    return;                                                              //拒绝不完整或非法坐标帧
}

Cam_TargetX = TargetX;                                                   //提交完整帧的目标 X 坐标
Cam_TargetY = TargetY;                                                   //提交完整帧的目标 Y 坐标
Cam_TargetDataFlag = 1U;                                                 //通知目标坐标已更新
```

- [x] **Step 3: 按数据类型读取坐标，确保激光预留接口不会消耗目标数据**

```c
if ((X == NULL) || (Y == NULL))                                         //拒绝空输出地址
{
    return 0U;                                                           //避免解引用空指针
}

if (Type == CAM_TARGERT)                                                 //处理当前已实现的目标坐标类型
{
    if (Cam_TargetDataFlag == 0U)                                        //检查是否存在未读取的目标坐标
    {
        return 0U;                                                       //通知调用方没有新目标数据
    }

    *X = Cam_TargetX;                                                    //复制目标 X 坐标
    *Y = Cam_TargetY;                                                    //复制目标 Y 坐标
    Cam_TargetDataFlag = 0U;                                             //在成功读取后消费目标数据
    return 1U;                                                           //通知坐标读取成功
}

if (Type == CAM_LASER)                                                   //保留未来激光坐标扩展入口
{
    return 0U;                                                           //当前没有激光数据通路
}

return 0U;                                                               //拒绝未知坐标类型
```

Expected: 源文件只有完整闭合的函数定义；`Cam_GetXY()` 不再反向读取接收标志；`Cam_LoadXY()` 在所有路径返回确定值。

### Task 4: 构建与行为复核（GREEN）

**Files:**

- Verify: `code/Cam.c`
- Verify: `code/Cam.h`

- [x] **Step 1: 构建 Keil 工程（Cam.c 通过；完整链接被既有 Gimbal 错误阻断）**

Run:

```powershell
& 'D:\Academy\Keil\UV4\UV4.exe' -b '.\SeekFree_MSPM0G3519_Device_Library.uvprojx'
```

Expected: 进程退出码为 `0`，构建日志不含 `Cam.c` 的 `error:` 或 `warning:`。

- [x] **Step 2: 静态核对已确认的数据流**

```powershell
rg -n 'Serial_GetRxFlag|CAM_TARGERT|CAM_LASER|Cam_TargetDataFlag|return 0U|return 1U' '..\code\Cam.c' '..\code\Cam.h'
```

Expected: `Serial_GetRxFlag()` 的无数据分支立即返回；目标读取成功后才清除目标标志；`CAM_LASER` 返回 `0U`，不存在激光坐标变量。

- [x] **Step 3: 检查最终差异范围**

Run:

```powershell
git diff --check -- '..\code\Cam.c' '..\code\Cam.h'
git diff --no-index -- /dev/null '..\code\Cam.c'
git diff --no-index -- /dev/null '..\code\Cam.h'
```

Expected: `git diff --check` 没有输出；人工确认仅有已批准的 Cam.c/Cam.h 逻辑与注释改动。

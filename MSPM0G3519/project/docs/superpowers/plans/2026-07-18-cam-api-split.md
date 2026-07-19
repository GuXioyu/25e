# Cam 接口拆分实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 Cam 的新数据标志读取与单轴坐标读取拆分为独立接口。

**Architecture:** `Cam_ReadXY()` 保持接收并保存坐标的职责。`Cam_GetFlag()` 是唯一消费新数据标志的函数；`Cam_GetXY(Axis, Type)` 只读取已保存的单个坐标，因此坐标读取不影响帧通知状态。

**Tech Stack:** C11、Keil MDK/ARMCLANG、逐飞 UART 兼容层。

---

### Task 1: 记录旧接口状态（RED）

**Files:**

- Test: `code/Cam.c`
- Test: `code/Cam.h`

- [x] **Step 1: 验证新接口尚未声明和实现**

Run:

```powershell
rg -n 'Cam_GetFlag|Cam_GetXY\(uint8_t Axis, uint8_t Type\)' '..\code\Cam.c' '..\code\Cam.h'
```

Expected: 命令没有匹配；当前 `Cam_GetXY` 仍带两个坐标输出指针并消费 `cam_flag`。

### Task 2: 拆分 Cam 公共接口

**Files:**

- Modify: `code/Cam.h`
- Modify: `code/Cam.c`

- [x] **Step 1: 在头文件声明拆分后的两个接口**

```c
//读取并清除一次新坐标标志
uint8_t Cam_GetFlag(void);

//读取指定类型与轴的已保存坐标
uint16_t Cam_GetXY(uint8_t Axis, uint8_t Type);
```

- [x] **Step 2: 新增唯一消费新数据标志的函数**

```c
uint8_t Cam_GetFlag(void)    //读取并清除一次新坐标标志
{
    if (cam_flag == 0)       //没有新坐标时通知读取失败
    {
        return 0;
    }

    cam_flag = 0;            //读取成功后消费本次新数据通知
    return 1;                //通知调用方存在新坐标
}
```

- [x] **Step 3: 将 Cam_GetXY 改为纯坐标读取函数**

```c
uint16_t Cam_GetXY(uint8_t Axis, uint8_t Type)    //读取指定类型与轴的已保存坐标
{
    if (Type == CAM_TARGERT)                      //选择目标坐标数据
    {
        if (Axis == 1)                             //选择 X 轴坐标
        {
            return cam_target_x;
        }
        if (Axis == 2)                             //选择 Y 轴坐标
        {
            return cam_target_y;
        }
    }
    else if (Type == CAM_LASER)                    //选择激光坐标预留数据
    {
        if (Axis == 1)                             //选择 X 轴坐标
        {
            return cam_laser_x;
        }
        if (Axis == 2)                             //选择 Y 轴坐标
        {
            return cam_laser_y;
        }
    }

    return 0;                                      //无效类型或轴返回零坐标
}
```

Expected: `Cam_GetXY()` 中不出现 `cam_flag`；只有 `Cam_GetFlag()` 清零该标志。

### Task 3: 静态核对拆分结果

**Files:**

- Verify: `code/Cam.c`
- Verify: `code/Cam.h`

- [x] **Step 1: 核对函数声明、实现和标志位写入位置**

Run:

```powershell
rg -n 'Cam_GetFlag|Cam_GetXY|cam_flag = 0|cam_flag = 1' '..\code\Cam.c' '..\code\Cam.h'
```

Expected: `cam_flag = 0` 只位于 `Cam_GetFlag()`；`cam_flag = 1` 保留在 `Cam_ReadXY()`；`Cam_GetXY` 的声明和定义均为两个 `uint8_t` 参数并返回 `uint16_t`。

- [x] **Step 2: 检查差异格式和已知构建限制**

Run:

```powershell
git diff --check -- '..\code\Cam.c' '..\code\Cam.h'
rg -n 'Cam_GetXY\(&gimbal_x, &gimbal_y, CAM_TARGERT\)' '..\code\Gimbal.c'
```

Expected: 差异格式检查没有输出；Gimbal.c 仍存在旧接口调用。用户已确认本次仅修改 Cam 文件，因此不构建整个工程。

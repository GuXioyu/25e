# Motor Set Zero Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `Motor` API that persists the current motor position as the EMM single-turn home zero point.

**Architecture:** Keep `Motor` as the application-facing adapter. Add one null-safe public wrapper in `Motor.c` that delegates to the existing EMM command with its save flag fixed at `1U`; the wrapper does not modify `Motor_t` cached motion state and does not trigger physical movement.

**Tech Stack:** C, MSPM0G3519 firmware, Keil MDK, existing `Emm_V5` serial command layer.

---

### Task 1: Validate And Add The Motor Set-Zero API

**Files:**
- Modify: `MSPM0G3519/project/code/Motor.h:101-119`
- Modify: `MSPM0G3519/project/code/Motor.c:163-179`
- Test: `MSPM0G3519/project/mdk/SeekFree_MSPM0G3519_Device_Library.uvprojx` build

- [ ] **Step 1: Verify the intended public API is absent**

Run:

```powershell
rg -n "void Motor_SetZero\(Motor_t \*motor\);" ..\code\Motor.h
```

Expected: no matching declaration, confirming that the requested API is not yet available.

- [ ] **Step 2: Add the public declaration**

Insert after `Motor_Stop()` in `Motor.h`:

```c
// 将当前位置设为单圈回零零点并保存
void Motor_SetZero(Motor_t *motor);
```

- [ ] **Step 3: Implement the minimal persistent set-zero wrapper**

Insert after `Motor_Stop()` in `Motor.c`:

```c
/**
 * @brief 将电机当前位置设为单圈回零零点并保存。
 * @param motor 电机对象。
 * @return 无。
 */
void Motor_SetZero(Motor_t *motor) /* 设置并保存电机单圈回零零点 */
{
	if (motor == NULL)
	{
		return;
	}

	Emm_V5_Origin_Set_O(motor->addr, 1U); /* 将当前位置设为零点并保存至驱动器 */
}
```

Do not change `motor->mode`, `motor->target_speed`, or `motor->target_angle`.

- [ ] **Step 4: Verify the declaration and implementation are present**

Run:

```powershell
rg -n "Motor_SetZero|Emm_V5_Origin_Set_O\(motor->addr, 1U\)" ..\code\Motor.h ..\code\Motor.c
```

Expected: one declaration in `Motor.h`, one definition in `Motor.c`, and one `Emm_V5_Origin_Set_O(motor->addr, 1U)` call.

- [ ] **Step 5: Build the Keil project**

Run:

```powershell
$uv4 = Get-Command UV4.exe -ErrorAction SilentlyContinue
if ($null -eq $uv4) { throw "UV4.exe 未在 PATH 中，需在 Keil MDK 中打开并编译 SeekFree_MSPM0G3519_Device_Library.uvprojx" }
& $uv4.Source -b .\SeekFree_MSPM0G3519_Device_Library.uvprojx
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
```

Expected: build completes with exit code `0` and no new compile or link errors.

- [ ] **Step 6: Perform the hardware persistence check**

Call `Motor_SetZero(&motor)` while the motor is stationary, power-cycle the EMM driver, then query or command the motor using the existing position workflow.

Expected: the location that was current when `Motor_SetZero()` was called remains the driver’s single-turn zero point after restart, and the call itself produces no movement.

- [ ] **Step 7: Commit the scoped code and task document**

```powershell
git add ..\code\Motor.c ..\code\Motor.h ..\docs\tasks\2026-07-18-motor-set-zero.md
git commit -m "feat: add motor set zero API"
```

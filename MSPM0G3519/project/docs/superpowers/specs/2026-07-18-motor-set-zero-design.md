# Motor Set Zero Design

## 目标

在 `Motor` 模块中提供将电机当前位置设为单圈回零零点的公开接口，并默认保存至 EMM 驱动器。

## 涉及模块

- `code/Motor.h`：声明上层设零接口。
- `code/Motor.c`：实现参数检查及底层调用。
- `code/Emm_V5.h`：复用现有 `Emm_V5_Origin_Set_O()`，不修改该模块。

## 设计

新增 `Motor_SetZero(Motor_t *motor)`。当 `motor` 为 `NULL` 时直接返回；否则调用
`Emm_V5_Origin_Set_O(motor->addr, 1U)`，将当前位置定义为单圈回零零点并写入驱动器非易失存储。

该接口不触发物理回零运动，不修改 `Motor_t` 的模式、目标速度或目标角度缓存。

## 验证

通过工程编译确认新增声明、实现及现有 `Emm_V5` 接口可正确链接；在实机上调用接口后断电重启，确认当前位置仍为已保存零点。

## 已确认决定

- 使用 EMM 的 `Emm_V5_Origin_Set_O()`。
- 保存标志固定为 `1U`。
- 仅实现当前位置设零，不实现触发机械回零。

## 待澄清问题

无。

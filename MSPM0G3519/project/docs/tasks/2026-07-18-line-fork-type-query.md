# 循迹岔道类型查询

## 目标需求

在 `Line` 模块中增加岔道类型查询函数。输入 `LineForkType_t` 岔道类型，当前类型匹配时返回 `1`，不匹配时返回 `0`；查询不得改变当前岔道类型。

## 涉及模块

- `code/Line.h`
- `code/Line.c`

## 实现思路

新增 `uint8_t Line_GetForkType(LineForkType_t Type)`。函数仅读取模块内部的 `line_fork_type`，使用相等比较产生 `1` 或 `0`，不写入任何岔道状态变量。

## 执行步骤

1. 在 `Line.h` 增加函数声明与中文说明。
2. 在 `Line.c` 增加函数定义，比较输入类型和当前岔道类型。
3. 检查函数未修改 `line_fork_type`，并构建 MDK 工程。

## 验证方法

- 调用 `Line_GetForkType(LINE_FORK_TYPE_LEFT_RIGHT_ANGLE)` 时，当前已锁定左直角岔道返回 `1`。
- 当前不是左直角岔道时，以上调用返回 `0`。
- 检查查询前后 `line_fork_type` 不发生改变。

## 已确认决定

- 采用通用输入参数接口，而不是仅判断左转的专用接口。
- 函数返回 `1` 或 `0`。
- 函数为只读查询，不能改变岔道类型。

## 待澄清问题

- 无。

# 循迹岔道类型查询设计

## 目标

在 `Line` 模块中增加一个只读查询接口。调用方传入岔道类型；当该类型与当前已识别的岔道类型相同时返回 `1`，否则返回 `0`。

## 接口与行为

接口为 `uint8_t Line_GetForkType(LineForkType_t Type)`。

- `Type` 为 `LINE_FORK_TYPE_LEFT_RIGHT_ANGLE` 且当前正在左直角岔道转弯时，返回 `1`。
- 当前类型与 `Type` 不匹配时返回 `0`。
- 函数仅比较 `line_fork_type`，不修改该变量，也不影响岔道识别、转弯完成或速度输出。

## 涉及模块

- `code/Line.h`：声明对外查询接口并说明返回值。
- `code/Line.c`：实现只读比较逻辑。

## 验证

- 静态检查函数仅包含比较与返回，不出现对 `line_fork_type` 的赋值。
- 构建 MDK 工程，确认新增声明与定义可编译。


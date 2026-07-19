# 25e Visual Aiming Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `/userdata/rdkstudio/25e` 落实基础瞄准和发挥前两项所需的视觉几何、距离融合、激光跟踪和固定快照输出，并保持M0与画圆题不变。

**Architecture:** RDK负责靶框几何、PnP距离、STP23L融合、预测坐标和激光检测；M0负责四个X42S电机、题目状态、激光开关和视觉外环。视觉快照每个视觉帧输出，距离/PnP按帧号降频，透视矩阵在同帧复用。

**Tech Stack:** Python 3, OpenCV, NumPy, Horizon BPU runtime, pyserial, 640×360 BGR camera frames.

---

## 目标文件边界

- 修改：`/userdata/rdkstudio/25e/main.py`
- 修改：`/userdata/rdkstudio/25e/target_utils.py`
- 修改：`/userdata/rdkstudio/25e/distance/distance.py`
- 修改：`/userdata/rdkstudio/25e/stp23l.py`
- 修改：`/userdata/rdkstudio/25e/my_uart.py`
- 测试：板端已有 `distance/tests/test_distance.py`，新增纯数学测试时放入同一测试目录；不启动真实设备。
- 文档：`D:\GitHub\25e\2025电赛E题_视觉瞄准解题思路与落实方案.md` 与 `D:\GitHub\25e\docs\tasks\2026-07-19-rdk-visual-aiming.md`。

## Task 1: 纯几何接口

**Files:** `target_utils.py`, `distance/distance.py`, `distance/tests/test_distance.py`

- [ ] 将 `calc_diagonal_center(corners)` 改为两条对角线交点，输入无效、平行或越界时返回 `None`；保留原函数名，避免破坏 `main.py` 调用。
- [ ] 在 `distance/distance.py` 增加按运行图像尺寸缩放相机内参的私有函数；物理靶点固定为 `(-13.5,-9,0)、(13.5,-9,0)、(13.5,9,0)、(-13.5,9,0)` cm。
- [ ] 增加 `estimate_target_pose(corners, image_width, image_height, distance_config=config)`：调用 `cv2.solvePnP`，返回有限、正向的 `rvec/tvec/distance_cm`；失败返回 `None`。
- [ ] 将距离配置的白色内框物理尺寸统一为27.0cm×18.0cm；只在显示/串口边界把距离格式化为一位小数，内部保留浮点精度。
- [ ] 增加 `predict_laser_axis(distance_cm, distance_config=config)`：使用 `set_lenth_cm=3.0`、`u0`、`v0`、`fy` 计算预测像素；公式为 `x=u0`、`y=v0+fy*set_lenth_cm/distance_cm`，并限制结果在图像范围内。
- [ ] 为交点、PnP正视/斜视、无效角点和预测公式增加不依赖相机/BPU的单元测试。

## Task 2: 激光归一化记忆

**Files:** `target_utils.py`

- [ ] 在 `LaserTracker` 中新增 `last_normalized_point`，其值为靶面归一化坐标 `(s,t)`，范围为0到1。
- [ ] 检测到原图激光点后，用当前靶框透视矩阵转换为 `(s,t)`；下一帧用新靶框把 `(s,t)`投影到快检图，作为局部窗口中心。
- [ ] 保留 `last_output_point` 作为对外最后有效原图像素；检测失败时返回它，不让历史值阻止完整靶面兜底。
- [ ] 目标丢失或靶框重置时清除归一化状态；激光单帧丢失时不清除靶框锁定。
- [ ] 保留现有120×95快检、连续失败门限、240×190兜底和冷却机制，禁止将完整兜底改成每帧运行。
- [ ] 增加归一化点在靶框平移、缩放和梯形透视变化后的重投影测试。

## Task 3: STP23L可重新锁定与融合

**Files:** `stp23l.py`, `main.py`, `distance/distance.py`

- [ ] 将STP23L状态拆为原始候选、稳定值、更新时间和新层级候选；保持现有串口解析帧格式不变。
- [ ] 使用中位数/连续样本抑制单帧异常；距离突变候选连续3帧互相接近时允许替换旧稳定值，不能永久拒绝新距离。
- [ ] 在主流程保存最近一次PnP距离和STP距离；仅当STP新鲜、STP轴投影点在靶框内、连续稳定且与PnP相差不超过15.0cm时采用STP，否则采用PnP。
- [ ] 目标未锁定、距离无效或融合失败时保持最后有效融合距离；首次无有效值输出0.0。
- [ ] 为15.0cm边界、超过边界、背景到靶面重新锁定和超时样本增加纯逻辑测试。

## Task 4: 主流程按帧调度和预测

**Files:** `main.py`

- [ ] 在 `FixedConfig` 增加 `set_lenth_cm = 3.0`、`pnp_frame_interval = 3`、`target_width_cm = 27.0`、`target_height_cm = 18.0` 及安装后标定的 `u0/v0` 字段；默认 `u0/v0`先使用当前相机内参主点，实机标定后替换。
- [ ] 扩展共享视觉状态：当前靶心、当前激光、最后有效中心、最后有效激光、最后有效预测点、最后有效距离、当前帧号和上次PnP帧号。
- [ ] 在 `user_after_inference` 内每帧执行内框和靶心；仅在首帧锁定、重捕获、帧号间隔达到3或中心/尺寸变化阈值触发时执行PnP和融合。
- [ ] 同一帧只生成一次靶框透视矩阵，并把它传给激光检测和归一化记忆路径；不对整帧做去畸变重映射。
- [ ] 生成固定快照字段：`center_x/y`、`nowlaser_x/y`、`predict_x/y`、`dis`；本帧无效时保持对应最后有效值，首次为0。
- [ ] 保持基础题激光关闭时也能发送靶心和预测数据；不要再要求中心和激光同时有效才发布。

## Task 5: 每视觉帧UART输出

**Files:** `my_uart.py`, `main.py`

- [ ] 将共享状态改为带递增视觉帧号的最新快照；UART线程只发送尚未发送的最新帧。
- [ ] 每帧按现有M0格式发送：`[cam,center_x,center_y,nowlaser_x,nowlaser_y,predict_x,predict_y,dis]`。
- [ ] 在RDK终端以`coordinate_print_interval=0.20s`限频打印`[laser,x,y,],[center,x,y]`，避免逐帧标准输出拖慢视觉线程。
- [ ] 像素字段发送四舍五入整数，`dis`发送一位小数；不增加校验、序号或有效位。
- [ ] UART发送失败不得阻塞视觉线程；队列积压时丢弃旧快照，只保留最新快照。

## Task 6: 文档同步与静态验证

**Files:** 两个本地文档及板端任务文档副本

- [ ] 在每个任务完成后同步“已实现/未实机验证/剩余标定值”；不把静态验证写成实机通过。
- [ ] 在本地运行 `python -m py_compile` 检查所有修改后的Python文件；不启动主程序、不打开相机、不打开UART。
- [ ] 运行已有纯数学测试和新增逻辑测试；如果项目规则不允许默认执行测试，则至少运行语法检查并记录测试未运行原因。
- [ ] RDK连接恢复后先读取板端状态和用户未提交改动，再以字节保持方式同步文件，重新运行板端 `py_compile`。

## 验收标准

- 对角线交点、PnP和预测坐标在纯数学测试中通过。
- STP23L背景→靶面转换可以重新锁定，15cm融合门限行为正确。
- 激光点短暂丢失时外部坐标保持最后有效值，内部能使用归一化点重捕获。
- PnP约每3帧更新，完整YOLO和240×190兜底不在锁定状态下每帧执行。
- 基础题激光关闭时RDK仍持续发送中心和预测；发挥前两项能持续发送实际激光点或最后有效点。
- 尚未连接RDK前不报告板端运行通过；基础2/4秒和D1≤2cm必须保留为实机验收项。

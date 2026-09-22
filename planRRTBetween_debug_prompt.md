## planRRTBetween 返回 false 排查提示词

背景项目：D:\test\QtWidgetsApplication3 (Qt Widgets 项目，Yaskawa 机器人切边路径规划)

问题：BallScan::applyMeasLinePathPredict 中 processLineResult 触发的 RRT 回退分支
里，planRRTBetween 返回的 ok 永远是 false，导致 RRT 兜底失效。需要定位根因
（不要直接动手改代码，先让我看 RRT 内部失败统计后再决定改法）。

### 调用链（已确认）

```
BallScan.cpp:1481  processLineResult(..., rrtCtxPtr)
  → process_line_result.cpp:337  planRRTBetween(...)
    → process_line_result.cpp:182  PathPlan::PlanPathRRTconnect_new(...)
       实现在 D:\test\Vision_1\FTC_TZ\IZ_new_transcode\PlanPathRRTconnect.cpp
```

### 关键事实（已读源码）

1) planRRTBetween 仅在两处分支返回 false（process_line_result.cpp:190-195）：
   - `res.goal_ind != 1`                → RRT 没找到路径（最大可能）
   - `res.BZD.rows() < 2 || cols < 6`  → 找到但 BZD 太小（次要）

2) PlanPathRRTconnect.h 默认参数极其紧：
   - `m_maxNodes   = 20`      （只允许 20 次主循环迭代）
   - `m_step_size  = 300`     （mm 级笛卡尔步长，偏大）
   - `m_threshold  = 100`
   - `m_neighbourhood = 750`

3) rrtExtend 的采样策略按 numNO 硬编码：
   - `numNO ≤ 3`     → 直接朝 HOME 方向推
   - `numNO 4..7`    → 朝双树中点推
   - `numNO 8..10`   → 反向推
   - `numNO ≥ 11`    → 才走 getSample 随机采样

   配合 m_maxNodes=20，真正"随机搜索"机会只有约 10 次主循环。

4) isObstacleFree 内 ConfigureCheck 只对 FANUC/KUKA 设置 invalid_config，
   对 Yaskawa（本项目 brand）走 else 分支只打印 cerr，invalid_config 留 0，
   理论上能通过，但要确认 robot_tech_params.brand 真的不是 FANUC/KUKA。

5) rrtExtend.cpp 内有失败计数器，每次 PlanPathRRTconnect_new 结束都会打印：

   ```
   [RRT_STATS] robot=xxx, iter=N, path_found=0,
               ik_failed=N1, joint_failed=N2,
               collision_failed=N3, connection_failed=N4
   ```

   这是定位根因最关键的一行日志。

### 请你按这个顺序排查（先看，不要急着改代码）

**第 1 步：搜代码确认 RRTContext 真的被填齐了，没被空指针短路**
- grep "RRTContext 未完整配置" 看 stderr 里有没有这句
- 看 BallScan.cpp:1468-1478 makeRRTContext 路径是否走到（看 paramIt 是否找到）

**第 2 步：定位 [RRT_STATS] 日志**
- grep "RRT_STATS" 所有源码，确认是 std::cout 打印（PlanPathRRTconnect.cpp:150 调
  用 printRrtFailureStats）
- 让我运行一次复现，把 [RRT_STATS] 那一行原文贴给你
- 根据 ik_failed / joint_failed / collision_failed / connection_failed 的占比判断：
  - collision_failed 占大头  → 起终点本身已经在碰撞配置上，或路径周围障碍密集
  - joint_failed 占大头      → 起终点 J4/J6 角度差超过 210°（J4_6_constraints）
  - ik_failed 占大头         → IK 失败，可能起点 xyz 在工作空间外
  - connection_failed 占大头 → 只是迭代次数不够，需要调 m_maxNodes

**第 3 步：看 fromPoint / toPoint 的 15 维 p 向量是否合法**
- 在 process_line_result.cpp:178-179 之后插一行临时 std::cerr 打印 from/to 的
  joints + xyz + ijk（仅 debug，不提交）
- 重点确认：
  - 起终点 joints 是否含 NaN
  - 起终点 |J4_from - J4_to| 与 |J6_from - J6_to| 是否 > 210°
  - 起点 (fromPoint.x, y, z) 是否在 robot_tech_params.robotWorkSpace 内

**第 4 步：核对品牌分支**
- 找到 robot_tech_params.brand 实际值（main.cpp 启动时载入位置）
- 如果不是 FANUC/KUKA，确认 steering / isObstacleFree 那个 else 分支只 cerr
  不致命（应该是，但要确认）

**第 5 步（拿到结论后再做）：根据第 2 步的失败计数对症下药**
- connection_failed 主导：把 PlanPathRRTconnect.h 里 m_maxNodes 调到 200~500
- joint_failed 主导：放宽 J4_6_constraints（rrtExtend.cpp:396 = 210，
  PlanPathRRTconnect.cpp:40 也有个 300 的常量但没用上）
- collision_failed 主导：步长 m_step_size 从 300 改小到 100~150，让中间插值更密
- ik_failed 主导：检查 fromPoint/toPoint 的 ijk/ijk2 计算（buildMeasurePointPoseSetFromViewPoint
  在 process_line_result.cpp，看其 i/j/k 是否取自有效旋转矩阵）

### 约束（必须遵守）

- 在 ViewPoint / 数据层修复，不要在 JBIExporter 等输出层补丁
- PlanPathRRTconnect.cpp/.h 在 D:\test\Vision_1\FTC_TZ\IZ_new_transcode 下，
  改前先确认 QtWidgetsApplication3 的 .vcxproj 是否真的包含/链接这份源码（否则改
  了不生效）
- 只在拿到 [RRT_STATS] 实际数字后再动 m_maxNodes / step_size / J4_6_constraints，
  否则就是瞎调参

请先做第 1-2 步并把 [RRT_STATS] 那行原文给我，再继续。

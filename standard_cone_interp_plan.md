# Standard 锥形测点「同位置换姿」关节插值方案

> 记录 Standard / Standard_circle 锥形视点之间过渡运动的插值问题分析与改进思路。
> 关联文档：`standard_points_task.md`（锥形视点生成任务）、`changes_skip_line_predict.md`（直线预测跳过记录）。

---

## 一、现状链路确认

Standard 点在可达性分析后**已经进入最后的直线预测**，路径如下：

1. `main.cpp:873` —— `ballScan.analyzeStandardPoints(...)` 锥形 N 视点可达性分析，可达视点 → `standardPath`。
2. `main.cpp:883-886` —— 改写每个视点 `partRotateAngle = angle`，并入 `allStandardPath`。
3. `main.cpp:997-998` —— 与面点 `allSurfacePath`、切边点 `combinedEdgePath` 一起按转台角度装入 `aaa`。
4. `main.cpp:999-1009` —— 逐角度组在对应碰撞环境下调用 `BallScan::applyMeasLinePathPredict`（内部逐段 `linePathPredict`/`measLinePathPredict` + `processLineResult`）。

> 注：`main.cpp:988-992` 注释写「Standard 不在此处并入」，与实际代码 997-998 不符，注释已过时，**以代码为准（已并入）**。

---

## 二、问题根因（两层）

### 第一层：`twoOrientationInterpolation` 步数只看位置、不看姿态
`two_orientation_interpolation.cpp:126-130`：
```cpp
const int N = static_cast<int>(std::floor(dist / stepMm));
if (N <= 0) return {};
```
步数 `N` 完全由 TCP 位置距离 `dist` 决定。同一测点的 `coneCount` 个锥形视点**共用同一 TCP 位置**（`dist≈0`）→ `N=0` → 返回空。内部明明有完整四元数 SLERP（132-178 行），却因判据用错量被白白丢弃，被迫回落到 `jointSpaceInterpolation`。

### 第二层：`jointSpaceInterpolation` 用关节线性插值表达「同位置换姿」，几何上错误
`ViewpointPlanner.cpp:378-382` 做各轴独立线性插值 `q(t)=q0+t·(q1-q0)` 再 FK。`q0`、`q1` 是**同位置、两个不同姿态**各自解出的 IK，可能落在差别很大的解支上：
- **TCP 位置不守恒**：中间帧 TCP 鼓出去画大弧，激光笔甩离工件再甩回，甩出量不可控，与「激光笔原地不动只转姿态」的设计意图相反。
- **中间点被丢弃**：这些点 `isImportant=false`，在 `processLineResult`（`ViewpointPlanner.cpp:538`）里全被过滤，真正执行的是一条 `q0→q1` 的大 MOVJ。碰撞检测查的是那条「甩出去的大弧」，而非预期的原地转动轨迹。

**本质矛盾**：关节线性插值保证两端 `q0/q1` 严格对齐（接缝连续），却牺牲 TCP 位置；又把中间点全扔了，等于既没保住位置、又没保住轨迹。

---

## 三、最终思路：绕原始法向「细分方位角 φ」的参数化插值

### 核心洞察
同一测点的两个相邻锥形视点，差别**就是绕原始法向 `z_axis` 转了 `Δφ = 360/coneCount` 度**（位置 `tcp_pos`、半顶角 `tiltDeg` 都不变）。因此过渡插值无需在两个 IK 解之间硬插，直接复用 `analyzeStandardPoints` 里「掀 `tiltDeg` + 绕法向转 `φ`」(`BallScan.cpp:824-869`) 的生成方式，把 `φ` 步进调细、沿同一圆锥弧逐帧生成位姿、逐帧 IK 即可：
- TCP 严格不动（激光笔原地）；
- 姿态绕法向纯滚转，中间帧天然落在圆锥面上；
- 即「把旋转补偿（方位角步进）设置得更小」。

> 数学等价性：同位置下「细分 φ」与「按姿态角驱动的 SLERP」等价（两位姿仅差绕固定轴的旋转，SLERP 退化为绕该轴匀速转角）。但「细分 φ」更透明、不碰四元数符号/nlerp 边界，且保证落在圆锥上，作为实现方案更稳。

### 锥形位姿生成方式（来自 `analyzeStandardPoints`）
- 基准系：`z = normalize(法向)`，`x = computeOptimalX(z, forward_x)`，`y = normalize(z×x)`，`x = y×z`。
- TCP 位置（不动）：`tcp_pos = mp + cfg.safeHeight·z_axis`。
- 倾斜系（绕局部 x 掀 `tiltDeg`，`ct=cos`,`st=sin`）：
  - `tx = x`
  - `ty = ct·y + st·z`
  - `tz = -st·y + ct·z`
- 绕原始法向 `z_axis` 旋转 `φ`（Rodrigues）：`v' = v·cosφ + (z×v)·sinφ + z·(z·v)(1-cosφ)`，对 `tx/ty/tz` 各算一次得到该 `φ` 的朝向列向量，拼成 `tcp_pose`。

---

## 四、实现要求

1. **抽取可复用的锥形位姿构造函数**
   把 `BallScan.cpp:824-869` 的「倾斜系 + `rotAboutNormal(φ)` → `Matrix4d tcp_pose`」逻辑抽成 helper：输入 `(z_axis, x_axis, y_axis, tcp_pos, tiltDeg, phi)`，返回位姿矩阵。`coneCount` 主循环和过渡插值都调它，保证完全一致。

2. **同位置过渡 densify**
   对同一测点**相邻两个可达锥形视点** `ci → ci+1`，在方位角区间 `[φ_ci, φ_{ci+1}]` 内按细步长 `coneInterpStepDeg`（新增 `RobotConfig` 字段，默认 2°~3°）等分生成中间位姿，`tcp_pos`/`tiltDeg`/`z_axis` 全部沿用该测点值。

3. **逐帧 IK + 种子连续性**
   每个中间位姿 `robotReachability` 解 IK，种子接上一帧关节（首帧接 `φ_ci` 视点关节），开启 `needVelCons`。
   - 可达 → 作为**真实路径点保留**：`isPathJoint=true`（使其在 `processLineResult` 存活）、`moveType=0`（MOVJ）、`isImportant` 按现有保留规则；**不写入 `outReachable`**（不是测量点，测量视点数量保持 `coneCount` 不变）。
   - 不可达 → 沿用现有 `mt=-2 → markAndCleanMinusTwo → RRT` 兜底（`ViewpointPlanner.cpp:511-521`），该子段交 RRT。

4. **避免下游二次插值**
   须保证 densify 出的同位置点**不再**被 `applyMeasLinePathPredict` 的 `twoOrientationInterpolation`/`jointSpaceInterpolation` 重新插值。放置位置二选一：
   - **(a) 推荐**：在 `analyzeStandardPoints` 生成阶段直接产出 dense MOVJ 链（此处具备全部生成参数，且已在对应转台角度碰撞环境下），并让 standard 段跳过下游同位置再插值。
   - (b) 仍走下游，但把锥形生成参数（`z_axis`/`tcp_pos`/`φ`）随 `ViewPoint` 携带，在线预测阶段重建 `φ` 细分。
   推荐 (a)：更直接、避免参数丢失。

5. **跨测点过渡照旧**
   不同测点之间（TCP 位置不同）的过渡仍走正常 `twoOrientationInterpolation` + 笛卡尔直线预测，不动。

6. **端点连续性**
   densify 链首帧关节 = `φ_ci` 视点关节、末帧 ≈ `φ_{ci+1}` 视点关节，保证接缝连续；末帧 IK 落异解支偏差大时按不可达走 RRT。

---

## 五、验收标准
- 同一测点锥形过渡执行为一串小步 MOVJ，FK 验证全程 TCP 位置漂移 < 阈值（如 1mm），姿态绕法向单调滚转。
- `jointSpaceInterpolation` 对 standard 场景不再被触发（可加日志确认）。
- 测量视点数量仍为 `点数 × coneCount`，JBI 导出路径连续、无大翻腕长 MOVL。

---

## 六、关键代码位置索引
- `BallScan.cpp:748` —— `analyzeStandardPoints`
- `BallScan.cpp:824-869` —— 锥形位姿生成（倾斜系 + `rotAboutNormal`），待抽取
- `BallScan.cpp:854-947` —— `coneCount` 视点循环 + 可达性判断
- `two_orientation_interpolation.cpp:126-130` —— 步数判据（只看位置）
- `ViewpointPlanner.cpp:365` —— `jointSpaceInterpolation`（待替代）
- `ViewpointPlanner.cpp:422` —— `measLinePathPredict`（消费插值点 + `-2`/RRT）
- `ViewpointPlanner.cpp:438-443` —— 退化回落到 `jointSpaceInterpolation` 的分支
- `ViewpointPlanner.cpp:511-521` —— 不可达压 `-2` 兜底
- `ViewpointPlanner.cpp:533-546` —— `processLineResult`（按 `isImportant`/`isPathJoint` 过滤）
- `main.cpp:873` —— `analyzeStandardPoints` 调用
- `main.cpp:997-1009` —— `allStandardPath` 并入 `aaa` + 逐角度 `applyMeasLinePathPredict`
- `RobotConfig.h:68` —— `jointInterpStepDeg`（新增 `coneInterpStepDeg` 参考）

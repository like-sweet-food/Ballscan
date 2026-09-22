# MOVL 直线预测 关节角速度约束 修改记录

## 修改目的

在**直线预测（MOVL）**链路中，对相邻两点的关节角速度加约束：
若「上一个已通过点 → 当前正在判定点」的某个 IK 解关节角速度过大（典型为奇异 / 腕翻导致「小笛卡尔位移 → 大关节位移」），
则**剔除该 IK 分支**。若该点所有分支都超速则判为不可达，由外层走扰动 / RRT。

**仅对 MOVL 生效**：通过新增开关参数 `needVelCons` 控制，默认 `false`；
只有 `linePathPredict` / `measLinePathPredict` 链路显式传 `true`。
**MOVJ / 面点 / 切边点等其它调用保持默认 `false`，行为完全不变。**

---

## 约束原理

```
角速度[i] = |sol[i] - prevQ[i]| / dt          （单位：度/秒）
dt        = 两点笛卡尔直线距离 / movlTcpSpeed   （单位：秒）
dt        = dt * movlDtSafetyFactor            （安全系数收紧，<1 更严）
若任一轴 角速度[i] > jointVelLimit[i] → 剔除该 IK 分支
```

- 两点取自**同一坐标系（用户坐标系 user frame）**下的位姿平移分量，距离计算正确。
- `sol` / `prevQ` 单位为度（IK 返回度），`dt` 单位为秒，故角速度单位为 **度/秒**。
- **上一个点** = `lastViews`（直线预测里传入的是 `validHistory`）中从后往前第一个关节非全零的有效点；
  **当前点** = 正在判定可达性的插值点 `tcpPose`，对应解 `sol`。
- 判定是**逐 IK 分支**进行：超速分支被剔除，只要还剩合规分支该点仍可达；全部超速才返回 false。

> 注意：相邻插值点笛卡尔距离 ≈ `minInterpolateDist`（固定值），故 dt 近似为常数，
> 该速度约束实质上等价于「对每步关节角差设上限」，真正决定行为的是 `jointVelLimit / movlTcpSpeed` 的比值。

---

## 涉及文件与改动点

### 1. `RobotConfig.h` —— 新增 3 个配置字段

在 `anglesThresholds` 下方新增：

```cpp
double movlTcpSpeed = 100.0;                                          // MOVL 的 TCP 速度 mm/s（用于估算 dt）
std::array<double, 6> jointVelLimit = {180, 180, 180, 180, 180, 180}; // 各轴角速度上限（度/秒，J1~J6）
double movlDtSafetyFactor = 0.7;                                      // 时间安全系数（<1 收紧；=1 关闭裕度）
```

> 以上均为占位默认值，需按实际机器人各轴允许速度与 MOVL 工艺速度调整。

### 2. `RobotReachability.h` —— 函数签名末尾加默认参数

```cpp
bool robotReachability(
    const Matrix4d&               tcpPose,
    const RobotConfig&            cfg,
    const std::vector<ViewPoint>& lastViews,
    bool                          needJointCons,
    JointVec6&                    outJoints,
    bool                          needVelCons = false);   // ★新增，默认 false
```

> 放在 `outJoints` 之后并给默认值：所有既有 5 参数调用无需改动。

### 3. `RobotReachability.cpp` —— 4 处

- **(3a)** 定义签名同步加 `bool needVelCons`（定义处不写默认值）。
- **(3b)** ⚠️ 修改已有行：原 `if (needJointCons)` 改为 `if (needJointCons || needVelCons)`；
  循环内在赋值 `prevQ` 处**额外取** `prevPose = lastViews[i].globalT`。
- **(3c)** 新增 dt 计算块（循环后）：

```cpp
double dt = 1e-6;   // 兜底，避免除零
if (needVelCons && hasPrev) {
    double dx = tcpPose[0][3] - prevPose[0][3];
    double dy = tcpPose[1][3] - prevPose[1][3];
    double dz = tcpPose[2][3] - prevPose[2][3];
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    dt = (dist > 1e-6) ? dist / cfg.movlTcpSpeed : 1e-6;
    dt *= cfg.movlDtSafetyFactor;   // 乘安全系数收紧时间
}
```

- **(3d)** 在已有「前后关节角约束」下方**新增**速度约束分支过滤（不改原约束）：

```cpp
if (needVelCons && hasPrev) {
    bool velOk = true;
    for (int i = 0; i < 6; ++i)
        if (std::abs(sol[i] - prevQ[i]) / dt > cfg.jointVelLimit[i]) { velOk = false; break; }
    if (!velOk) continue;   // 该 IK 分支角速度超限，剔除
}
```

### 4. `ViewpointPlanner.h` —— 2 个声明末尾加默认参数

- `adjustViewPose(...)` 末尾加 `bool needVelCons = false`
- `tryLocalPerturbation(...)` 末尾加 `bool needVelCons = false`

### 5. `ViewpointPlanner.cpp` —— 把开关接到扰动并在 MOVL 处打开

- **(5a)** `tryLocalPerturbation` 定义加形参；⚠️ 内部 `robotReachability(newT, ..., true, out_j)` 调用末尾透传 `needVelCons`。
- **(5b)** `adjustViewPose` 定义加形参；⚠️ 调用 `tryLocalPerturbation(...)` 末尾透传 `needVelCons`。
- **(5c)** ⚠️ `linePathPredict`：
  - `robotReachability(currentPose, ..., true, outJoints, /*needVelCons=*/true)`
  - `adjustViewPose(currentPose, validHistory, newTcp, adjJoints, /*outPartRotateAngle=*/nullptr, /*needVelCons=*/true)`
- **(5d)** ⚠️ `measLinePathPredict`：同 (5c) 两处调用末尾加 `true`。

> 因 `adjustViewPose` 的 `needVelCons` 在 `outPartRotateAngle` 之后，(5c)/(5d) 需显式传 `nullptr` 占位。

---

## 未改动 / 不受影响

- `BallScan.cpp` 中所有 `robotReachability` / `adjustViewPose` 调用一律未改，`needVelCons` 取默认 `false`，MOVJ / 面点 / 切边点链路行为不变。
- `enc_temp_folder\...\ViewpointPlanner.cpp` 为 VS 临时备份，未改动。

---

## 调参说明

| 参数（`RobotConfig.h`） | 含义 | 调整方向 |
|---|---|---|
| `movlTcpSpeed` | MOVL TCP 线速度 mm/s | 改为实际工艺速度 |
| `jointVelLimit[6]` | 各轴角速度上限 度/秒 | 某轴易甩则调小该轴 |
| `movlDtSafetyFactor` | 时间安全系数 | 越小越严；=1 关闭裕度 |

下列三种调法**数学等价**，择一即可，勿叠加：
`movlDtSafetyFactor` 调小 ≡ `jointVelLimit` 整体调小 ≡ `movlTcpSpeed` 调大。

---

## 行为小结

MOVL 直线预测每个插值点：
1. `robotReachability` 先剔除超速 IK 分支，剩合规分支则点正常可达（保住点且选不甩腕的解）；
2. 全部分支超速 → 返回 false → 进扰动，扰动内部同样带速度门槛，只接受合规姿态；
3. 扰动也无解 → 压 `mt=-2` 占位 → Pass2（`path_rrt_splice`）整体清理触发 RRT。

---

## 后续变更：删除 `needJointCons` 参数，前后关节角约束改为恒定启用

### 背景
原 `needJointCons`（第 4 个参数）控制「前后关节角约束」（`anglesThresholds` 角度差筛选），
各调用点 true/false 不一：面点/直线预测/扰动为 true，切边点等 6 处为 false。
现需求改为**所有调用点都启用**该约束，故直接删除该参数，函数内部恒定执行。

### 改动点

**`RobotReachability.h` / `.cpp`**：函数签名删除 `bool needJointCons`。
新签名（注意 `needVelCons` 顺位前移）：
```cpp
bool robotReachability(
    const Matrix4d& tcpPose, const RobotConfig& cfg,
    const std::vector<ViewPoint>& lastViews,
    JointVec6& outJoints,
    bool needVelCons = false);   // .h 带默认值
```

**`RobotReachability.cpp` 内部逻辑**：
- prevQ 求取：原 `if (needJointCons || needVelCons)` 改为**无条件**（角约束恒开，prevQ 始终需要）。
- 角约束判断：原 `if (needJointCons && hasPrev)` 改为 `if (hasPrev)`（恒定启用）。
- 速度约束（`needVelCons`）逻辑不变。

**所有调用点删除第 4 个实参**：
- `BallScan.cpp`：7 处调用删除 `true`/`false` 实参（603 面点、821/1073/1309、1705、1737、1790）；
  另更新 4 处过时注释（600/818/1070/1306）。
- `ViewpointPlanner.cpp`：3 处删除 `true` 实参（190 扰动、298 linePathPredict、406 measLinePathPredict），
  其中 298/406 保留末尾 `/*needVelCons=*/true`。

### 影响
- 原本 `false` 的 6 处（切边点等）现在也会做前后关节角约束（`anglesThresholds`，默认 120°）。
- 副作用：个别「关节跳变大但 IK 合法」的点可能变不可达 → 触发更多扰动 / RRT。改完建议跑一遍验证。
- 本节描述**取代**前文中「`needJointCons` 第 4 参数」「既有 5 参数调用无需改动」等表述。

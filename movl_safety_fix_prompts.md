# MOVL 直线轨迹超限修复 — 修改提示词

## 背景

切边路径使用直线轨迹（MOVL）导出 JBI 文件后，在 Motosim play 模式下出现关节超限。
根本原因链：

1. `measLinePathPredict` 已用 `robotReachability` 逐一验证中间笛卡尔插值点可达性
2. `processLineResult` 默认 `keepOnlyKeyNodesAtEnd = true`，将所有非关键中间点丢弃
3. JBI 只含稀疏关键端点，Motosim 执行 MOVL 时自行重新插值，可能选不同 IK 分支超限
4. MOVJ 规划逻辑（`idxEmpty` 触发）因 `moveType=-1` 从不出现而永远不执行
5. `isJointStateSafe` 只做静态碰撞检查，未检查关节限位

涉及文件：`BallScan.cpp`

---

## 修改一：保留中间验证路径点

**位置**：`BallScan.cpp` — `applyMeasLinePathPredict`，最后一行 `processLineResult` 调用

**问题**：
```cpp
path = processLineResult(predicted, callbacks, {});
//                                             ^^
// 默认 ProcessLineOptions：keepOnlyKeyNodesAtEnd = true
// → measLinePathPredict 已用 robotReachability 验证的中间 MOVL 点全部被丢弃
```

**修改为**：
```cpp
ProcessLineOptions opts;
opts.keepOnlyKeyNodesAtEnd = false;   // 保留全部已验证中间路径点
path = processLineResult(predicted, callbacks, opts);
```

---

## 修改二：`isJointStateSafe` 加入关节限位检查

**位置**：`BallScan.cpp` — `makeProcessLineCallbacks`，`hooks.isJointStateSafe` lambda

**问题**：
```cpp
hooks.isJointStateSafe =
    [robot_name](const JointVec6& q_deg, double /*partRotate*/) -> bool {
        const std::vector<double> q(q_deg.begin(), q_deg.end());
        return !KinematicsApi::instance().checkCollision(robot_name, q, q);
        // q 传两次 → 只做静态碰撞，未检查关节限位
    };
```

**修改为**：
```cpp
hooks.isJointStateSafe =
    [robot_name, cfg](const JointVec6& q_deg, double /*partRotate*/) -> bool {
        // 关节限位检查（对应 MATLAB robotReachability_joint_hyj 的限位判断）
        for (int i = 0; i < 6; ++i) {
            if (q_deg[i] < cfg.jointLimits[i].minDeg ||
                q_deg[i] > cfg.jointLimits[i].maxDeg)
                return false;
        }
        // 碰撞检查
        const std::vector<double> q(q_deg.begin(), q_deg.end());
        return !KinematicsApi::instance().checkCollision(robot_name, q, q);
    };
```

---

## 修改三：过滤后对相邻导出点做 MOVL 段安全性二次验证

**位置**：`BallScan.cpp` — `applyMeasLinePathPredict`，在修改一之后（`processLineResult` 调用之后）追加

**问题**：
`processLineResult` 过滤为关键点后，相邻导出点之间的 MOVL 段从未经过任何可达性验证。
Motosim 执行这段 MOVL 时可能超限。

**新增逻辑**：
对过滤后的 `path` 中每对相邻节点（均为 `moveType == 1`），调用
`robot_planner::twoOrientationInterpolation` 采样笛卡尔路径，
再对每个中间姿态调用 `robot_planner::robotReachability` 做可达性检查。
若任意中间姿态不可达，将该段右端点 `moveType` 设为 `-1`（触发标记），
最后对整个 `path` 再次调用 `processLineResult`，令 `idxEmpty` 非空，
使 MOVJ 规划逻辑自动在超限段插入 MOVJ 过渡点。

**伪代码结构**：
```cpp
// --- 二次验证：检查过滤后每条 MOVL 段是否真正可达 ---
bool needReprocess = false;
for (size_t i = 0; i + 1 < path.size(); ++i) {
    if (path[i].moveType != 1 || path[i+1].moveType != 1) continue;

    // 构建历史视点（供 robotReachability 做关节连续性约束）
    std::vector<ViewPoint> history;
    for (size_t h = 0; h <= i; ++h) {
        bool hasNaN = false;
        for (int k = 0; k < 6; ++k)
            if (std::isnan(path[h].joints[k])) { hasNaN = true; break; }
        if (!hasNaN) history.push_back(path[h]);
    }

    // 采样中间笛卡尔姿态
    auto interPoses = robot_planner::twoOrientationInterpolation(
        path[i].globalT, path[i+1].globalT, cfg);

    // 逐一做可达性检查
    for (const auto& pose : interPoses) {
        robot_planner::JointVec6 dummy{};
        bool ok = robot_planner::robotReachability(pose, cfg, history, true, dummy);
        if (!ok) {
            path[i+1].moveType = -1;   // 触发标记，令 idxEmpty 检测到此段
            needReprocess = true;
            break;
        }
    }
}

// 若有不安全段，重新执行 processLineResult 触发 MOVJ 规划逻辑
if (needReprocess) {
    ProcessLineOptions opts2;
    opts2.keepOnlyKeyNodesAtEnd = false;
    path = processLineResult(path, callbacks, opts2);
}
```

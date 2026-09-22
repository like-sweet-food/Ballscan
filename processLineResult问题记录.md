# processLineResult 问题记录

## 问题现象

1. `processLineResult` 输出的所有点 `moveType` 全为 1（MOVL），没有 0（MOVJ）
2. `processLineResult` 内部 `idxEmpty` 始终为空，规划逻辑从未执行

---

## 根本原因

### 原因一：`planMoveJ` 回调未接入

两处调用 `processLineResult` 时，callbacks 均为空 `{}`：

```cpp
// BallScan.cpp:604（面点路径）
outSurfacePath = processLineResult(outSurfacePath, {}, {});

// BallScan.cpp:1054（切边点路径，applyMeasLinePathPredict 内）
path = processLineResult(predicted, {}, {});
```

`ViewpointPlanner::planMoveJ`（ViewpointPlanner.cpp 第436行）已实现，但**从未作为 callback 传入**。即使将来 `idxEmpty` 非空，关节空间规划也永远无法执行。

### 原因二：`idxEmpty` 始终为空

`idxEmpty` 收集 `moveType == -1` 的点。`moveType = -1` 只在以下情况被赋值：

```
linePathPredict / measLinePathPredict 中：
  插值点不可达（IK失败）
       ↓
  调用 adjustViewPose 扰动补救
       ↓ 失败
  vp.moveType = -1  ← 只有这里才会出现 -1
```

当前测试场景中，所有 Cartesian 插值点要么直接可达，要么扰动补救成功，**`moveType=-1` 从未被设置**，因此 `idxEmpty` 始终为空，修复逻辑完全跳过。

---

## 数据流梳理

```
analyzeEdgePoints / analyzeEdgePointsRotated
  → outEdgePath（仅可达关键点，moveType=1, isImportant=true）
  → combinedEdgePath 拼接

applyMeasLinePathPredict(combinedEdgePath)
  → measLinePathPredict 逐段插值
      ├─ 可达中间点：moveType=1, isPathJoint=false
      ├─ 扰动成功点：moveType=1, isPathJoint=true
      └─ 扰动失败点：moveType=-1, isPathJoint=true, joints=NaN  ← 当前测试无此情况
  → processLineResult(predicted, {/*空*/}, {})
      ├─ 预处理：NaN joint 关键点的 isImportant/isPathJoint 置 false
      ├─ idxEmpty 检测 moveType==-1 → 当前始终为空
      ├─ 最终过滤：只保留 isImportant||isPathJoint 的点
      └─ 输出：全为 moveType=1（MOVL）的关键点
```

---

## 签名对比（需要适配）

`process_line_result.h` 要求的 callback 签名：

```cpp
std::function<PlanMoveJResult(const JointVec6&, const JointVec6&, double, bool)> planMoveJ;
//                              J1               J2               segRotate  useSimpleMid
// 返回：PlanMoveJResult { bool isSafe; std::optional<JointVec6> qMid; }
```

`ViewpointPlanner::planMoveJ` 实际签名：

```cpp
MoveJResult planMoveJ(const JointVec6& startDeg, const JointVec6& endDeg) const;
// 返回：MoveJResult { std::vector<JointVec6> path; bool isSafe; bool hasMid; JointVec6 midJoint; }
```

两者参数数量和返回类型均不同，**需要适配 lambda 桥接**：

```cpp
ProcessLineCallbacks cb;
cb.planMoveJ = [&planner](const JointVec6& j1, const JointVec6& j2,
                           double /*segRotate*/, bool /*useSimpleMid*/) -> PlanMoveJResult {
    auto res = planner.planMoveJ(j1, j2);
    PlanMoveJResult out;
    out.isSafe = res.isSafe;
    if (res.hasMid) out.qMid = res.midJoint;
    return out;
};
```

---

## 待解决事项

- [ ] 确认测试场景中是否有真正不可达的插值段（运行后查看是否有 `[linePathPredict] 不可达且扰动失败` 日志）
- [ ] 将 `ViewpointPlanner::planMoveJ` 适配并接入 `processLineResult` 的 callbacks
- [ ] 确认 `planMoveJ` 中 `segRotate` 和 `useSimpleMid` 参数是否需要传入 BiRRTPlanner
- [ ] 确认 `planRRTFallback` callback 是否也需要接入

---

## 相关文件

| 文件 | 关键位置 |
|------|---------|
| `process_line_result.h` | `ProcessLineCallbacks` 定义，callback 签名 |
| `process_line_result.cpp` | `idxEmpty` 检测（第117-123行），规划逻辑（第130-276行） |
| `ViewpointPlanner.h/.cpp` | `planMoveJ` 实现（第436-463行） |
| `BallScan.cpp` | `processLineResult` 调用（第604、1054行），`analyzeEdgePoints` 输出（第773-782行） |
| `main.cpp` | 三阶段调用流程，`applyMeasLinePathPredict`（第613行） |

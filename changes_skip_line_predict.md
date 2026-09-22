# 跳过直线轨迹预测的修改记录

## 修改目的

直线预测（`linePathPredict` / `applyMeasLinePathPredict`）存在问题，暂时跳过该步骤，直接使用原始视点输出 JBI 文件。

---

## 修改1：`main.cpp`（约第 613 行）

**跳过切边路径的直线预测及 state 同步。**

### 注释掉的代码

```cpp
BallScan::applyMeasLinePathPredict(combinedEdgePath, cfg);
if (!combinedEdgePath.empty()) {
    state.lastViewPose = combinedEdgePath.back().globalT;
    state.lastJoints   = combinedEdgePath.back().joints;
}
```

### 修改后

```cpp
// BallScan::applyMeasLinePathPredict(combinedEdgePath, cfg);
// if (!combinedEdgePath.empty()) {
// 	state.lastViewPose = combinedEdgePath.back().globalT;
// 	state.lastJoints   = combinedEdgePath.back().joints;
// }
```

**效果：** `combinedEdgePath` 保持原始切边视点不变，直接进入 JBI 导出。

---

## 修改2：`BallScan.cpp`（约第 590 行，`analyzeSurfacePoints` 内）

**跳过面点路径的直线预测，直接将可达面点赋值给输出路径。**

### 注释掉的代码

```cpp
if (!reach_surface_vps.empty())
{
    for (size_t i = 0; i + 1 < reach_surface_vps.size(); ++i) {
        auto seg = planner.linePathPredict(
            reach_surface_vps[i], reach_surface_vps[i + 1]);
        outSurfacePath.insert(outSurfacePath.end(),
                              seg.begin(), seg.end());
    }
    // HOME 标记：对应 MATLAB index==length 时追加 start_info（倒数第二个视点）
    if (reach_surface_vps.size() >= 2) {
        outSurfacePath.push_back(reach_surface_vps[reach_surface_vps.size() - 2]);
    }
}
```

### 新增代码

```cpp
outSurfacePath = reach_surface_vps;
```

### 修改后完整片段

```cpp
// if (!reach_surface_vps.empty())
// {
//     for (size_t i = 0; i + 1 < reach_surface_vps.size(); ++i) {
//         auto seg = planner.linePathPredict(
//             reach_surface_vps[i], reach_surface_vps[i + 1]);
//         outSurfacePath.insert(outSurfacePath.end(),
//                               seg.begin(), seg.end());
//     }
//     // HOME 标记：对应 MATLAB index==length 时追加 start_info（倒数第二个视点）
//     if (reach_surface_vps.size() >= 2) {
//         outSurfacePath.push_back(reach_surface_vps[reach_surface_vps.size() - 2]);
//     }
// }
outSurfacePath = reach_surface_vps;
```

**效果：** `outSurfacePath` 直接为所有可达面点的原始视点列表，无插值、无 HOME 标记。

---

## 说明

- `state` 更新（`state.lastViewPose` / `state.lastJoints`）依赖 `outSurfacePath.back()`，两处修改后仍可正常取到末尾视点，无需额外调整。
- JBI 导出逻辑（`combinedPath = allSurfacePath + combinedEdgePath`）不受影响。
- 如需恢复直线预测，取消上述注释并删除新增的赋值行即可。

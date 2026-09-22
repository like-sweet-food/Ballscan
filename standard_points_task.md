# 任务:为 Standard / Standard_circle 测点添加可达性分析

## 背景
`main.cpp` 中已有面点(`analyzeSurfacePoints`)和切边点的可达性分析逻辑。现需为两类新测点 `Standard` 和 `Standard_circle` 增加分析,整体框架与面点一致。输入容器已在 `main.cpp` 准备好:`fileStandardMPs`、`fileStandardCircleMPs`。

## 任务一:在 main.cpp 增加「阶段3」
在**切边点(阶段2)逻辑之后**,新增一段 Standard / Standard_circle 的分析,框架照搬阶段1/阶段2(转台角度循环、不可达点 carry-over 滚动、收集可达/不可达结果到 `measure_point_result` / `un_measure_point_result`、追加路径到 `allSurfacePath`)。分别用 `fileStandardMPs`、`fileStandardCircleMPs` 作为输入,调用新函数 `analyzeStandardPoints`。

## 任务二:新增函数 `analyzeStandardPoints`(在 BallScan.cpp / BallScan.h)
仿照 `analyzeSurfacePoints`,签名追加锥形参数:

```cpp
void analyzeStandardPoints(
    const std::string& robotName,
    const std::vector<MeasurePoint>& standardPoints,
    RobotConfig& cfg, ScanState& state,
    std::vector<ViewPoint>& outPath,
    std::vector<MeasurePointPoseSet>& outReachable,
    std::vector<MeasurePointPoseSet>& outUnreachable,
    double tiltDeg,      // 锥半顶角,绕局部 x 轴掀开的角度
    int    coneCount);   // 一圈视点数(步进角 = 360/coneCount)
```

调用:
- Standard:`tiltDeg = 40, coneCount = 6`(步进 60°)
- Standard_circle:`tiltDeg = 40, coneCount = 12`(步进 30°)

**与 `analyzeSurfacePoints` 相同的部分**(照搬):
1. 建规划器 `ViewpointPlanner planner(cfg)`、`reachReasonReset()`。
2. 第2步「前向 X 轴预计算」`forward_x_axes`(首点/中间点/末点的差分规则一致)。
3. 每个点的基准坐标系:`z = normalize(i,j,k)`,`x = computeOptimalX(z, forward_x_axes[i])`,`y = normalize(z×x)`,`x = y×z`;基准位置 `tcp_pos = mp + cfg.safeHeight * z`。
4. 可达性判断 `robotReachability` + 不可达时 `adjustViewPose` 扰动补救;可达写 `outReachable`+`reachHistory`+`reach_vps`,不可达写 `outUnreachable`;结尾更新 `state.lastViewPose/lastJoints`、`reachReasonDump`。

**唯一不同的部分 —— 用「锥形 N 视点」替代 `constructNextBestView`:**
对每个 Standard 点,基于其基准坐标系生成 `coneCount` 个候选位姿:
1. 先绕**局部 x 轴**旋转 `tiltDeg`(40°),把 z 轴掀开 → 得到倾斜朝向 `z'`(圆锥半顶角)。
2. 再把这个倾斜后的坐标系绕**该点原始 z 法向矢量**每隔 `360/coneCount` 度旋转一次,转满一圈 → 共 `coneCount` 个朝向,`z'` 扫出一个以原法向为中轴、半顶角 40° 的圆锥。
3. **「激光笔原地不动」**:这 `coneCount` 个位姿**共用同一个 TCP 位置** `tcp_pos`(= `mp + safeHeight*原法向`),只有朝向不同。
4. 这 `coneCount` 个位姿**各自独立**走可达性判断流程,**可达的全部写入结果集**(不是只取第一个)。每个位姿输出为独立的 `MeasurePointPoseSet`,name 在原测点名后加后缀区分(如 `_C00` ~ `_C11`),避免下游按 name 回查撞名。

锥形示意:
```
        z(原法向)
         │      ___ 倾斜40°的 z' (coneCount 个, 绕z均分一圈)
         │    ╱
         │  ╱ )40°
   ──────●──────  ← 同一 TCP 位置(激光笔不动)
```

## 已有铺垫(无需重做)
`main.cpp` 顶部已加 `isStandardMP` / `isStandardCircleMP` 两个判别函数,且分流循环中已先判 `Standard_circle`(因其含子串 "Standard")再判 `Standard`,容器 `fileStandardMPs` / `fileStandardCircleMPs` 已填充。

## 关键参考位置
- `main.cpp:28-43` —— `isSurfaceMP` / `isTrimMP` 判别函数(`isStandardMP` / `isStandardCircleMP` 紧随其后)
- `main.cpp:380` 附近 —— 测点分流循环,已填充 `fileStandardMPs` / `fileStandardCircleMPs`
- `main.cpp:554` 附近 —— `analyzeSurfacePoints` 调用处(阶段1)
- `main.cpp:588` 附近 —— 阶段2 切边点逻辑起点(阶段3 加在其后)
- `BallScan.cpp:505` —— `analyzeSurfacePoints` 函数实现(照抄模板)
- `BallScan.cpp:574-708` —— 逐点视点生成 + 可达性主循环(锥形逻辑替换 622 行的 `constructNextBestView`)
- `ViewpointPlanner.cpp:50` —— `constructNextBestView`(被替换的函数,可参考其旋转矩阵写法)
- `ViewpointPlanner.cpp:110` —— `buildTcpPoseLocal`(由 pos/normal/x 直接建姿,锥形可复用)

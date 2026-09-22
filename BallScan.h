#pragma once
// =============================================================================
// BallScan.h  —  球扫可达性分析（拆分为三阶段入口）
//
// 三个公开函数分别处理：
//   1. analyzeSurfacePoints()      — 面点可达性分析 + 路径预测
//   2. analyzeEdgePoints()         — 切边点 SA排序 + 可达性分析
//   3. analyzeEdgePointsRotated()  — 切边点旋转后 SA排序 + 可达性分析
//
// 切边点与旋转切边点的直线路径预测统一在所有可达性分析完成后，
// 由调用方对 combinedEdgePath 调用 applyLinePathPredict() 执行一次。
//
// 通过 ScanState 在三个阶段间传递关节连续性状态。
// 通过 KinematicsApi::instance() 单例调用正逆解和碰撞检测。
// =============================================================================

#include <map>
#include <vector>
#include <string>
#include "parameters_type.h"
#include "Types.h"
#include "RobotConfig.h"

class BallScan {
public:
    // =========================================================================
    // ShiftedPoint — 偏移后的测点（法向量归一化 + 安全高度偏移）
    // =========================================================================
    struct ShiftedPoint {
        robot_planner::Vec3d pos;     // 偏移后的 TCP 位置
        robot_planner::Vec3d normal;  // 归一化法向量（z_axis）
    };

    // =========================================================================
    // ScanState — 跨阶段共享状态（关节连续性 + 视点姿态连续性）
    //
    // 由 main.cpp 创建，每个机器人一个实例，依次传给三个阶段函数。
    // 保证关节角从 Home 位开始，经过面点→切边点→旋转切边点连续过渡。
    // =========================================================================
    struct ScanState {
        std::vector<robot_planner::ViewPoint> reachHistory;  // 可达视点关节历史
        robot_planner::Matrix4d lastViewPose = robot_planner::eye4(); // 上一个视点 TCP 姿态
        robot_planner::JointVec6 lastJoints = {};            // 上一个可达视点关节角
        bool hasLastJoints = false;                           // 是否已有上一视点
    };

    // =========================================================================
    // 阶段1：面点可达性分析
    //
    // 对应原 BallScan::allocation() 中面点部分（段4a + 路径预测）。
    // 内部流程：
    //   1) 前向X轴预计算（相邻面点差分方向）
    //   2) 逐点视点生成 + IK可达性 + 扰动补救
    //   3) 相邻可达面点间 linePathPredict 路径预测（对应 MATLAB LinePathPredict.m）
    // =========================================================================
    void analyzeSurfacePoints(
        const std::string& robotName,
        const std::vector<MeasurePoint>& surfacePoints,
        robot_planner::RobotConfig& cfg,
        ScanState& state,
        std::vector<robot_planner::ViewPoint>& outSurfacePath,
        std::vector<MeasurePointPoseSet>& outReachable,
        std::vector<MeasurePointPoseSet>& outUnreachable);

    // =========================================================================
    // 阶段3：Standard / Standard_circle 测点可达性分析（锥形 N 视点）
    //
    // 框架照搬 analyzeSurfacePoints（前向X轴预计算 + 基准坐标系 + 可达性判断 +
    // 扰动补救 + state 更新），唯一区别：用「锥形 N 视点」替代 constructNextBestView。
    //   - 先绕基准坐标系局部 x 轴掀开 tiltDeg（圆锥半顶角）；
    //   - 再绕该点原始 z 法向每隔 360/coneCount 度旋转一圈，共 coneCount 个朝向，
    //     z' 扫出以原法向为中轴、半顶角 tiltDeg 的圆锥；
    //   - coneCount 个位姿共用同一 TCP 位置（激光笔原地不动），仅朝向不同；
    //   - 每个位姿各自独立判可达，可达的全部写入 outReachable，name 追加 _C00.._Cnn。
    // 调用：Standard      → tiltDeg=40, coneCount=6 （步进 60°）
    //       Standard_circle→ tiltDeg=40, coneCount=12（步进 30°）
    // =========================================================================
    void analyzeStandardPoints(
        const std::string& robotName,
        const std::vector<MeasurePoint>& standardPoints,
        robot_planner::RobotConfig& cfg,
        ScanState& state,
        std::vector<robot_planner::ViewPoint>& outPath,
        std::vector<MeasurePointPoseSet>& outReachable,
        std::vector<MeasurePointPoseSet>& outUnreachable,
        double tiltDeg,      // 锥半顶角，绕局部 x 轴掀开的角度
        int    coneCount);   // 一圈视点数（步进角 = 360/coneCount）

    // =========================================================================
    // 阶段2：切边点可达性分析
    //
    // 对应原 BallScan::allocation() 中切边点部分（段1~段3）。
    // 内部流程：
    //   isFirst=true （第一次调用，全量切边点）：
    //     1) 批量预处理（法向量归一化 + measSafeHeight 偏移）
    //     2) 以 placeEndTcp 作为起点参考运行 SA 排序（或复用 cachedOrder）
    //     3) 按排序后顺序链式构建 TCP 姿态 + 可达性检测
    //   isFirst=false（后续转台角度，仅传入上一次不可达子集）：
    //     - 跳过步骤2（SA 起点参考）和步骤4（SA 排序）
    //     - 传入点已由 main.cpp 变换至新转台角度坐标，直接做预处理 + 可达性检测
    //     - 等价于 MATLAB adjustViewPoseMeas 在新转台角度下重新判断可达性
    //      （直线预测不在此处执行，由调用方统一调用 applyLinePathPredict）
    // =========================================================================
    void analyzeEdgePoints(
        const std::string& robotName,
        const std::vector<MeasurePoint>& edgePoints,
        robot_planner::RobotConfig& cfg,
        ScanState& state,
        const robot_planner::Matrix4d& placeEndTcp,
        double currentTurnTableAngle,   // 本次调用对应的转台角度（度），写入可达视点的 partRotateAngle
        std::vector<MeasurePointPoseSet>& outReachable,
        std::vector<MeasurePointPoseSet>& outUnreachable,
        std::vector<robot_planner::ViewPoint>& outEdgePath);

    // =========================================================================
    // 阶段3：切边点旋转后可达性分析
    //
    // 对应原 BallScan::allocation() 中段4（SA2 + 旋转 + 可达性）。
    // 内部流程：
    //   1) 批量预处理（法向量归一化 + measSafeHeight 偏移）
    //   2) 若 precomputedOrder 为空，以 state.lastViewPose 末尾位置运行 SA2 排序；
    //      否则直接使用 precomputedOrder（跳过 SA）
    //   3) 按排序后顺序提取坐标，整体旋转 rotateAngleDeg 度
    //   4) 链式构建 TCP 姿态 + 可达性检测
    //      （直线预测不在此处执行，由调用方统一调用 applyLinePathPredict）
    // =========================================================================
    void analyzeEdgePointsRotated(
        const std::string& robotName,
        const std::vector<MeasurePoint>& edgePoints,
        double rotateAngleDeg,
        robot_planner::RobotConfig& cfg,
        ScanState& state,
        double currentTurnTableAngle,   // 本次调用对应的转台角度（度），写入可达视点的 partRotateAngle
        const robot_planner::Matrix4d& placeEndTcp,
        std::vector<MeasurePointPoseSet>& outReachable,
        std::vector<MeasurePointPoseSet>& outUnreachable,
        std::vector<robot_planner::ViewPoint>& outEdgePath);

    // =========================================================================
    // analyzeEdgePointsCombined — 切边点 + 旋转切边点合并可达性分析（单次 SA）
    //
    // 用于把原阶段2（切边点）与阶段3（旋转切边点）合并到同一转台角度内：
    //   - 输入 points 中包含 normal 切边点和旋转切边点的副本，旋转切边点的
    //     name 以 ROTATED_NAME_SUFFIX("_R") 结尾、xyz/ijk 与原切边点完全一致；
    //   - 函数内部只做一次 SA 排序（每个转台角度调用一次即可），同名 normal /
    //     rotated 因 pos 重合会被排成相邻，从而最小化腕部姿态变化；
    //   - 在链式构建 TCP 后，对 name 带 _R 后缀的测点额外右乘 Rx(rotateAngleDeg)，
    //     使其姿态绕 TCP 自身 X 轴倾斜，平移列保持不变（与 analyzeEdgePointsRotated
    //     中段5.x 写法完全一致）；
    //   - si==0 时首点 preset_x 取下一邻居差分方向，遇到 pos 重合（同名 normal /
    //     rotated）会自动跳过到下一个不同位置的邻居，保证差分有效。
    //
    // 不可达点写入 outUnreachable，name 保留 _R 后缀；外层可据此用名字
    // 在 allCombinedPoints 中回查到原始未变换测点，作为下一个转台角度的输入。
    // =========================================================================
    static constexpr const char* ROTATED_NAME_SUFFIX = "_R";

    void analyzeEdgePointsCombined(
        const std::string& robotName,
        const std::vector<MeasurePoint>& points,   // 已 transform 至当前转台坐标系；含 _R 后缀副本
        double rotateAngleDeg,                     // 应用于 _R 后缀测点（典型值 -45.0）
        robot_planner::RobotConfig& cfg,
        ScanState& state,
        const robot_planner::Matrix4d& placeEndTcp,
        double currentTurnTableAngle,
        std::vector<MeasurePointPoseSet>& outReachable,
        std::vector<MeasurePointPoseSet>& outUnreachable,
        std::vector<robot_planner::ViewPoint>& outEdgePath);

    // =========================================================================
    // applyLinePathPredict — 对路径做直线轨迹预测（对应 LinePathPredict.m）
    //
    // 在切边点和旋转切边点的可达性分析全部完成后，由调用方对 combinedEdgePath
    // 统一调用一次，避免分批预测导致段间衔接断裂。
    // =========================================================================
    static void applyLinePathPredict(
        std::vector<robot_planner::ViewPoint>& path,
        robot_planner::RobotConfig& cfg);

    // =========================================================================
    // applyMeasLinePathPredict — 对切边合并路径做直线轨迹预测 + 后处理
    //
    // 对应 MATLAB: MeasLinePathPredict.m + processLineResult
    // 与 applyLinePathPredict 的差异（继承自 measLinePathPredict）：
    //   1) 插补点从第0帧开始（包含全部插补点）
    //   2) 遇到不可达点时先将前一可达点标记 isImportant=true
    // 函数内部已调用 processLineResult，调用方无需再次调用。
    // =========================================================================
    static void applyMeasLinePathPredict(
        std::vector<robot_planner::ViewPoint>& path,
        robot_planner::RobotConfig& cfg);

    // =========================================================================
    // computeTransitionPose — 计算过渡点 TCP 姿态
    //
    // 在每个转台角度处理完成后调用，以 lastViewPose 为基础，
    // 沿 mainNormal 方向抬起 shiftDistance，构建过渡点 TCP 姿态，
    // 用作下一个转台角度 SA 排序的起点参考（placeEndTcp）。
    // 若过渡点不可达，回退返回 lastViewPose。
    // outJoints 不为 nullptr 时写入对应关节角，供调用方构建 ViewPoint。
    // =========================================================================
    static robot_planner::Matrix4d computeTransitionPose(
        const robot_planner::Matrix4d& lastViewPose,
        const robot_planner::Vec3d&    mainNormal,
        double                         shiftDistance,
        robot_planner::RobotConfig&    cfg,
        const ScanState&               state,
        robot_planner::JointVec6*      outJoints = nullptr);

private:
    // =========================================================================
    // saSort — 模拟退火路径排序（提取自原 SA1/SA2 共用逻辑）
    //
    // 参数：
    //   shifted      — 偏移后的测点列表
    //   startRef     — SA 起点参考位置（距此最近的点为起点）
    //   seed         — 随机种子（SA1: cfg.randomSeed; SA2: cfg.randomSeed+1）
    //   canonicalize — 是否规范化首个重复块（SA1=true, SA2=false）
    //
    // 返回：best_tour 索引序列（不含尾部追加起点）
    // =========================================================================
    std::vector<size_t> saSort(
        const std::vector<ShiftedPoint>& shifted,
        const robot_planner::Vec3d& startRef,
        unsigned int seed,
        bool canonicalize = true) const;
};

#pragma once
// =============================================================================
// path_rrt_splice.h  —  dense 路径的 RRT 拼接与稀疏化
//
// 替代 processLineResult 在 applyMeasLinePathPredict 中的角色，只做两件事：
//   (a) 扫描 measLinePathPredict 输出的 dense 路径，对每个"连续
//       moveType=-1 run"调 PathPlan 的 RRT，把 run 替换为 joint-space 中间点；
//   (b) 稀疏化输出：MOVL 控制器会自己做笛卡尔直线插补，dense 的可达插补点
//       没有必要保留；最终只保留原始 waypoint、gap 边界、RRT 中间点。
//
// 与 process_line_result.* 的差异：
//   · 过滤规则：isImportant || isPathJoint 才保留（与旧 isKeyNode 对齐——
//     既留原始 waypoint / RRT 端点（isImportant=true），也留 main.cpp 在
//     转台切换时插入的 MOVJ 过渡点（isPathJoint=true / isImportant=false））；
//   · 入口先调 markAndCleanMinusTwo（Pass 2）把 mt=-2 占位点删除并把左右
//     邻居 moveType 改为 -1，从而让"扰动失败"在 ViewpointPlanner 层只需
//     一次性 push 一个 -2 标记，跨段合并语义全部交给 Pass 2 处理；
//   · 不做关键节点扩展回退（RRT 失败直接降级为 MOVL，让上层兜底）。
// =============================================================================

#include <vector>

// 复用 process_line_result.h 中已经定义好的 RRTContext / ViewPoint 等。
// PROCESS_LINE_RESULT_USE_EXTERNAL_VIEWPOINT 由调用方（BallScan.cpp）定义。
#include "process_line_result.h"

namespace robot_planner {

// dense: measLinePathPredict + applyMeasLinePathPredict 拼接好的密集路径，
//        每个点的 moveType 取值：
//          1  → MOVL 可达点（普通插补点或原始 waypoint）
//          0  → MOVJ 点（一般不出现在 measLine 阶段，保留兼容）
//         -1  → gap 边界占位（与下一段 -1 配对形成一个 RRT run）
//        每个点的 isImportant：
//          true  → 原始关键点（analyzeEdgePointsCombined 标的 waypoint）
//          false → dense 插补点 / gap 边界
//
// rrtCtx: 复用 process_line_result.h 中的 RRTContext，必须配置完整
//         （tech_params/ik_fn/fk_fn/collision_fn 都非空），否则本函数会
//         打日志并把所有 -1 run 降级为 MOVL（与 RRT 失败行为一致）。
//
// 返回：稀疏化后的路径，包含所有 isImportant 或 isPathJoint 为 true 的
//       点（原始 waypoint + gap 边界 + RRT 中间点 + main.cpp 转台过渡点
//       + 扰动成功的过渡点）。
std::vector<ViewPoint> spliceRRTIntoDensePath(
    const std::vector<ViewPoint>& dense,
    const RRTContext& rrtCtx);

}  // namespace robot_planner

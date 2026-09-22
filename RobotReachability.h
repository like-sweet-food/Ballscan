#pragma once
// =============================================================================
// RobotReachability.h  —  单视点可达性检测
//
// 对应 MATLAB：robotReachability.m
//
// 给定 TCP 姿态，依次执行：
//   1. 手眼 + 用户坐标系变换 → Base 坐标系
//   2. IK 求解
//   3. 关节限位筛选（Yaskawa 轴约定转换）
//   4. 环境碰撞检测
//   5. 自碰撞检测（J2/J3/J4 夹角 < 27°）
//   6. 关节连续性约束（可选）
//   7. 加权最近解选优
//
// 返回 true  = 可达，outJoints 为所有满足约束的关节角（度）
// 返回 false = 不可达 / 碰撞
// =============================================================================

#include <vector>
#include "Types.h"
#include "RobotConfig.h"
#include "parameters_type.h"

namespace robot_planner {

bool robotReachability(
    const Matrix4d&               tcpPose,
    const RobotConfig&            cfg,
    const std::vector<ViewPoint>& lastViews,
    JointVec6&                    outJoints,
    // ===== MOVL 直线预测的关节角速度约束开关 =====
    // 默认 false：既有调用无需改动。仅 movl 链路（linePathPredict /
    // measLinePathPredict）显式传 true。
    // 注：原 needJointCons 参数已删除，前后关节角约束（anglesThresholds）
    //     改为函数内部恒定启用，不再由调用方控制。
    bool                          needVelCons = false);

// =============================================================================
// [诊断] 可达性失败原因计数（只统计，不改判定逻辑）
//   robotReachability 内部在每个 IK 分支被剔除处 / 整点不可达处累加计数。
//   reachReasonReset() 清零；reachReasonDump(tag) 打印并不清零。
//   典型用法：在 applyMeasLinePathPredict 开头 reset、结尾 dump，
//   即可得到"本转台角一段路径"的失败原因分布。
// =============================================================================
void reachReasonReset();
void reachReasonDump(const char* tag);

} // namespace robot_planner

#pragma once

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <vector>

// If your project already defines ViewPoint / Matrix4d / JointVec6,
// define PROCESS_LINE_RESULT_USE_EXTERNAL_VIEWPOINT before including this header.
#ifndef PROCESS_LINE_RESULT_USE_EXTERNAL_VIEWPOINT
using Matrix4d = std::vector<std::vector<double>>;
using JointVec6 = std::array<double, 6>;

struct ViewPoint {
    Matrix4d globalT;
    JointVec6 joints = {};
    bool isImportant = false;
    bool isPathJoint = false;
    int moveType = 1;  // -1 = empty, 0 = MoveJ, 1 = MoveL
    double partRotateAngle = 0.0;
    Matrix4d partTransform;
};
#endif

// PathPlan 接口所需类型（来自 D:\test\3part\m_include\parameters_type.h）
#include "parameters_type.h"

struct PlanMoveJResult {
    bool isSafe = false;
    std::optional<JointVec6> qMid;
};

struct ProcessLineCallbacks {
    // MATLAB equivalent:
    // planmovej_hyj(J1, J2, config, segPartRotate, use_simple_mid)
    std::function<PlanMoveJResult(const JointVec6&, const JointVec6&, double, bool)> planMoveJ;

    // Optional FK callback for inserted mid nodes.
    std::function<Matrix4d(const JointVec6&)> forwardKinematics;
};

struct ProcessLineOptions {
    int emptyMoveTypeValue = -1;
    bool useSimpleMid = true;
    bool keepOnlyKeyNodesAtEnd = true;  // same as MATLAB final filtering
};

struct ExistingPlannerHooks {
    // Required for MoveJ sampling (time + quintic interpolation).
    std::function<double(const JointVec6&, const JointVec6&)> calculatetimecpp;
    std::function<JointVec6(const JointVec6&, const JointVec6&, double, double)> mutinomial;

    // Required for safety check of sampled joint states.
    std::function<bool(const JointVec6&, double)> isJointStateSafe;

    // Optional: direct midpoint search from your project.
    std::function<std::optional<JointVec6>(const JointVec6&, const JointVec6&, double, bool)> findReachableMidpoint;

    // Optional FK callback.
    std::function<Matrix4d(const JointVec6&)> forwardKinematics;
};

// Build process callbacks from your existing planner hooks.
// MoveJ callback is implemented using calculatetimecpp + mutinomial.
ProcessLineCallbacks makeCallbacksWithExistingPlanner(const ExistingPlannerHooks& hooks,
                                                      double sampleStep = 0.05);

// RRT 上下文：用于把直线段不通时的回退调用转发到 PathPlan::PlanPathRRTconnect_new。
// 三个函数指针的签名直接来自 parameters_type.h 的 typedef，
// 项目入口已通过 RobotKinematicsCollisionInterface 提供同签名实现。
struct RRTContext {
    std::string robot_name;
    const RobotTechParameters* tech_params = nullptr;
    RobotIKFunction ik_fn = nullptr;
    RobotFKFunction fk_fn = nullptr;
    CollisionTestFunction collision_fn = nullptr;
};

// C++ replacement of MATLAB processLineResult:
// input/output contain HOME at index 0.
// rrtCtx 为空指针时跳过 RRT 回退（行为退化为原 MATLAB 无 RRT 分支版本）。
std::vector<ViewPoint> processLineResult(const std::vector<ViewPoint>& lineProcessResult,
                                         const ProcessLineCallbacks& callbacks,
                                         const ProcessLineOptions& options = {},
                                         const RRTContext* rrtCtx = nullptr);

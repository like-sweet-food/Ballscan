#pragma once
// =============================================================================
// ViewpointPlanner.h  —  视点规划工具类
//
// 通过 KinematicsApi::instance() 单例直接调用正逆解和碰撞检测，
// 不需要传入函数指针。
// =============================================================================

#include <vector>
#include <array>
#include <string>
#include "Types.h"
#include "RobotConfig.h"
#include "RobotReachability.h"

namespace robot_planner {

class ViewpointPlanner {
public:
    ViewpointPlanner(RobotConfig& cfg);

    // =========================================================================
    // 视点规划接口
    // =========================================================================

    // 视点姿态扰动补救（Z移动 × Z旋转 双重搜索，无转台备选）
    // outPartRotateAngle: 可选输出；当前无转台版本固定写 0
    bool adjustViewPose(
        const Matrix4d&               originalTcp,
        const std::vector<ViewPoint>& lastViews,
        Matrix4d&                     outNewTcp,
        JointVec6&                    outJoints,
        double*                       outPartRotateAngle = nullptr,
        // 新增：把速度约束开关透传给内部的 robotReachability。默认 false，movj 调用不受影响。
        bool                          needVelCons = false) const;

    // 两视点间直线路径预测（TCP空间插值 + IK可达性检测 + 扰动补救）
    // 对应 MATLAB: LinePathPredict.m
    std::vector<ViewPoint> linePathPredict(
        const ViewPoint& startVp,
        const ViewPoint& endVp) const;

    // 两视点间直线路径预测（对应 MATLAB: MeasLinePathPredict.m）
    // 与 linePathPredict 的差异：
    //   1) 从插补序列第0帧开始（MATLAB idx=1，包含全部插补点）
    //   2) 遇到不可达点时，先将 result 末尾可达点标记 isImportant=true，再扰动
    std::vector<ViewPoint> measLinePathPredict(
        const ViewPoint& startVp,
        const ViewPoint& endVp) const;

    // 路径后处理
    std::vector<std::array<double, 8>> processLineResult(
        const std::vector<ViewPoint>& viewpoints) const;

    // 关节空间路径规划（MoveJ，含 BiRRT）
    struct MoveJResult {
        std::vector<JointVec6> path;
        bool      isSafe = false;
        bool      hasMid = false;
        JointVec6 midJoint = {};
    };
    MoveJResult planMoveJ(const JointVec6& startDeg,
                          const JointVec6& endDeg) const;

    // =========================================================================
    // 视点姿态构建工具
    // =========================================================================
    Vec3d    computeOptimalX(const Vec3d& zAxis, const Vec3d& presetX) const;
    Matrix4d constructNextBestView(const Matrix4d& lastView, const Vec3d& newNormal) const;
    Matrix4d buildTcpPoseLocal(const Vec3d& posLocal,
                               const Vec3d& normalLocal,
                               const Vec3d& xAxisLocal) const;

    // =========================================================================
    // 数学工具（静态）
    // =========================================================================
    static Vec3d    normalize3(const Vec3d& v);
    static double   dot3(const Vec3d& a, const Vec3d& b);
    static Vec3d    cross3(const Vec3d& a, const Vec3d& b);
    static Matrix4d multiplyTransforms(const Matrix4d& A, const Matrix4d& B);
    static Matrix4d invertTransform(const Matrix4d& T);

    // =========================================================================
    // 配置访问
    // =========================================================================
    RobotConfig&       config()       { return m_cfg; }
    const RobotConfig& config() const { return m_cfg; }

private:
    RobotConfig&   m_cfg;

    // 内部共用：在 baseTcp 基础上执行 Z移动×Z旋转 双重搜索
    bool tryLocalPerturbation(
        const Matrix4d&               baseTcp,
        const std::vector<ViewPoint>& lastViews,
        const std::vector<double>&    searchAnglesRad,
        Matrix4d&                     outNewTcp,
        JointVec6&                    outJoints,
        // 新增：速度约束开关，透传给内部 robotReachability。默认 false。
        bool                          needVelCons = false) const;

    // 计算绕 Z 轴（过 axisPoint）旋转 angleDeg 度的 4×4 变换矩阵
    static Matrix4d makeZRotAroundPoint(const Vec3d& axisPoint, double angleDeg);
};


} // namespace robot_planner

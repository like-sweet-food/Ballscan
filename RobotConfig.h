#pragma once
// RobotConfig.h — 全局配置参数（默认值兜底，前端有值时覆盖）

#include <array>
#include <vector>
#include <string>
#include "Types.h"
#include <parameters_type.h>

namespace robot_planner {

struct JointLimit {
    double minDeg;
    double maxDeg;
};

struct RobotConfig {
    // 新变量结构体初始化，最新赋值在前端赋予
    // 机器人基本信息
    std::string robotName = "YASKAWA_GP20HL";

    // 6轴关节限位（度，含 joint_delta=10 余量）
    std::array<JointLimit, 6> jointLimits = {{
        {-170.0,  170.0},   // J1
        { -35.0,  170.0},   // J2
        {-160.0,  105.0},   // J3
        {-190.0,  190.0},   // J4
        {-140.0,  140.0},   // J5
        {-445.0,  445.0}    // J6
    }};

    double collisionTolerance = 50.0;                              // 碰撞容忍距离 mm
    std::array<double, 6> anglesThresholds = {120, 120, 120, 120, 120, 120}; // 相邻视点关节角变化阈值（度）

    // ===== 新增：MOVL 直线预测的关节角速度约束参数 =====
    // 仅在 needVelCons=true（即 linePathPredict / measLinePathPredict 链路）时生效。
    // 速度 = |当前解关节角 - 上一点关节角| / dt，其中 dt = 两点笛卡尔距离 / movlTcpSpeed。
    // 任一轴速度超过 jointVelLimit 即剔除该 IK 分支。
    double movlTcpSpeed = 100.0;                                    // MOVL 的 TCP 速度 mm/s（用于估算 dt）
    std::array<double, 6> jointVelLimit = {999, 999, 999, 999, 999, 999}; // 各轴角速度上限（度/秒）
    // MOVL 速度约束的时间安全系数（<1 收紧约束）：dt 乘此值，相当于按更短耗时来判定角速度，
    // 留出运动裕度。例如 0.7 表示假设这段要在 70% 的时间内走完，门槛更严、更易剔除点。
    double movlDtSafetyFactor = 1;
    std::string desiredBody   = "front";             // 期望构型 body：front / rear
    std::string desiredElbow  = "up";               // 期望构型 elbow：up / down
    std::string desiredWrist  = "noflip";           // 期望构型 wrist：noflip / flip
    std::string desiredSrtStr = "S<180,R<180,T<180"; // 期望构型 srt_str：S=J1,R=J4,T=J6

    // 坐标系变换
    Matrix4d T_user = {
        {  1,  0,  0, 0},
        { 0,  1,  0, 0},
        {  0,  0,  1,  0},
        {  0,  0,  0,    1}
    };

    XYZWPR T_he = { -433.34, -1.26, 545.93, 90.0, 0.0, 135.0 };

    // 算法参数
    double collisionRate     = 0.4;
    double safeHeight        = 0.0;      // 面扫安全抬起高度 mm
    double measSafeHeight    = 0.0;    // 测点安全抬起高度 mm
    std::array<double, 3> windowSize = {60.0, 60.0, 60.0};
    std::array<double, 3> stepSize   = {60.0, 60.0, 60.0};
	double minInterpolateDist = 5.0;// 插值时两点间最小距离 mm，也就是步长
    // twoOrientationInterpolation 退化（TCP 距离≈0，同位置换姿）时改走关节空间插值，
    // 此为关节各轴线性插值的角步长（度）：相邻采样点的最大单轴角位移上限，越小越密、
    // 越不易漏检碰撞。仅 jointSpaceInterpolation 使用。
    double jointInterpStepDeg = 10;//绕着一根轴的旋转角度步长
    unsigned int randomSeed   = 666;

    // 扰动搜索参数（对应 MATLAB config.zMoveSteps / zMoveDistance）
    int    zMoveSteps    = 10;   // 沿Z轴最大移动步数
    double zMoveDistance = 5.0;  // 每步移动距离 mm

    // 相机模型参数
    double ballRadius      = 30.0;
    double rotateTheta     = 40.0;
    double camOffset       = 100.0;
    double measRotateTheta = -40.0;

    // 转台参数
    Vec3d measTableAxisXYZ = {8539.80, 2702.78, 0.0};
    std::vector<double> rotateRange = {0.0, 90.0, 180.0, 270.0};

    // HOME位 & 上一视点
    JointVec6 robotHome = {-90.07, -70.0, -50.0, 0.07, 60.0, -0.08};
    JointVec6 lastView  = {-90.07, -70.0, -50.0, 0.07, 60.0, -0.08};

    // 外部轴
    double slidePos = -3500.0;

    // 过渡点参数
    Vec3d  mainNormal    = {0.0, 0.0, 1.0};  // 零件主法向量，需前端赋值
    double shiftDistance = 50.0;             // 过渡点沿主法向抬起距离 mm
};

inline RobotConfig defaultConfig() { return RobotConfig{}; }

} // namespace robot_planner

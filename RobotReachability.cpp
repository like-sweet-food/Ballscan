// =============================================================================
// RobotReachability.cpp  —  单视点可达性检测实现
//
// 对应 MATLAB：robotReachability.m
// =============================================================================

#include "RobotReachability.h"
#include "kinematicsApi.h"
#include "math_utils.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace robot_planner {

// =============================================================================
// [诊断] 可达性失败原因计数器（只读统计，不参与任何判定）
//   · 分支级 br* : 每个 IK 分支被某约束剔除时 +1（一个点可有多条分支）
//   · 整点级 call*/unreach* : 每次 robotReachability 调用的结果
//   · velKilledSomeInCall : 本次调用中是否有分支"恰好死在速度门"
//       —— unreachVelInvolved 统计：不可达点里有多少"沾了速度约束"
// =============================================================================
namespace {
struct ReachReasonCounter {
    long calls = 0;          // 总调用次数（= 可达性检查的点数）
    long reachable = 0;      // 返回 true
    long unreachIkEmpty = 0; // IK 无解（含 looseAngleIk 也无解）
    long unreachFiltered = 0;// 有 IK 解但全被约束剔除
    // 分支级剔除原因
    long brJointLimit = 0;
    long brAngle = 0;
    long brVel = 0;
    long brCollision = 0;
    long brPosture = 0;
    long brValid = 0;        // 通过全部约束的分支
    // 不可达点里"有分支死在速度门"的点数
    long unreachVelInvolved = 0;
};
ReachReasonCounter g_reach;
} // namespace


// -----------------------------------------------------------------------------
// 内部数学工具（匿名命名空间，不对外暴露）
// -----------------------------------------------------------------------------
namespace {

Matrix4d mat_multiply(const Matrix4d& A, const Matrix4d& B)
{
    Matrix4d C(4, std::vector<double>(4, 0.0));
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Matrix4d mat_invert(const Matrix4d& T)
{
    Matrix4d inv = eye4();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            inv[i][j] = T[j][i];
    for (int i = 0; i < 3; ++i) {
        inv[i][3] = 0.0;
        for (int j = 0; j < 3; ++j)
            inv[i][3] -= inv[i][j] * T[j][3];
    }
    return inv;
}

double vec3_dot(const Vec3d& a, const Vec3d& b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// 构型判断（对应 MATLAB classify_ik_posture_hyj，全部4字段）
// elbow : j3 < 0 → "up"  / "down"      (MATLAB 第130~133行)
// wrist : j5 > 0 → "flip"/ "noflip"    (MATLAB 第137~143行)
// srt   : |q|>=180 → ">=180" / "<180"  (MATLAB srt_sector_tag_raw, eps=0)
// body  : FK(link1/link5) 投影正负      (MATLAB 第169~188行)
bool checkPosture(const JointVec6& sol, const RobotConfig& cfg)
{
    // 构型标签（与外部库 ConfigureCheck::isPreferred 对齐）
    std::string elbow = (sol[2] < 0.0) ? "up" : "down";
    std::string wrist = (sol[4] > 0.0) ? "flip" : "noflip";
    // srt: S=J1(0), R=J4(3), T=J6(5)
    std::string sS = (sol[0] >= 180.0 || sol[0] <= -180.0) ? ">=180" : "<180";
    std::string sR = (sol[3] >= 180.0 || sol[3] <= -180.0) ? ">=180" : "<180";
    std::string sT = (sol[5] >= 180.0 || sol[5] <= -180.0) ? ">=180" : "<180";

    // body: (FK(link5).pos - FK(link1).pos) · FK(link1).x_axis 正负
    // FK 不可用时默认按 front 处理（保持原有「FK 缺失则不卡 body」的行为）
    std::string body = "front";
    std::vector<double> jv(sol.begin(), sol.end());
    auto fk1 = KinematicsApi::instance().fk(cfg.robotName, jv, static_cast<LinkName>(1));
    auto fk5 = KinematicsApi::instance().fk(cfg.robotName, jv, static_cast<LinkName>(5));
    if (fk1.ut.size() >= 4 && fk5.ut.size() >= 4) {
        double proj = (fk5.ut[0][3] - fk1.ut[0][3]) * fk1.ut[0][0]
                    + (fk5.ut[1][3] - fk1.ut[1][3]) * fk1.ut[1][0]
                    + (fk5.ut[2][3] - fk1.ut[2][3]) * fk1.ut[2][0];
        body = (proj >= 0.0) ? "front" : "rear";
    }

    // 默认优先构型（对应 ConfigureCheck::isPreferred）：
    // 1) No-flip / Upper / Front / S<180 / R<180 / T<180
    // 2) Flip    / Upper / Front / S<180 / R<180 / T<180
    // 3) Flip    / Upper / Front / S<180 / R>=180 / T<180
    if (body != "front" || elbow != "up" || sS != "<180" || sT != "<180")
        return false;

    return (wrist == "noflip" && sR == "<180")
        || (wrist == "flip"   && (sR == "<180" || sR == ">=180"));
}

// Extrinsic XYZ（对应 robot.h gp_Extrinsic_XYZ）：T = Rz(w) * Ry(p) * Rx(r)
Matrix4d xyzwpr_to_matrix4d(const XYZWPR& xp)
{
    const double w = xp.w * M_PI / 180.0;
    const double p = xp.p * M_PI / 180.0;
    const double r = xp.r * M_PI / 180.0;
    const double cw = std::cos(w), sw = std::sin(w);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cr = std::cos(r), sr = std::sin(r);
    return {
        { cw*cp,  cw*sp*sr - sw*cr,  cw*sp*cr + sw*sr,  xp.x },
        { sw*cp,  sw*sp*sr + cw*cr,  sw*sp*cr - cw*sr,  xp.y },
        {   -sp,           cp*sr,             cp*cr,     xp.z },
        {   0.0,             0.0,               0.0,      1.0 }
    };
}

} // anonymous namespace

// =============================================================================
// robotReachability  —  对应 MATLAB robotReachability.m
// =============================================================================
bool robotReachability(
    const Matrix4d&               tcpPose,
    const RobotConfig&            cfg,
    const std::vector<ViewPoint>& lastViews,
    JointVec6&                    outJoints,
    bool                          needVelCons)   // 新增参数（默认值只写在 .h，定义处不写）
{
    ++g_reach.calls;                       // [诊断] 本次可达性检查计数
    bool velKilledSomeInCall = false;      // [诊断] 本次是否有分支死在速度门

    // -----------------------------------------------------------------------
    // 1. 手眼 + 用户坐标系：tcpPose（用户系）→ Base 系
    //    T_base = inv(T_user) * tcpPose * inv(T_he)   （对应 MATLAB 第60行）
    // -----------------------------------------------------------------------
    //Matrix4d T_base = mat_multiply(
    //    mat_invert(cfg.T_user),
    //    mat_multiply(tcpPose, mat_invert(xyzwpr_to_matrix4d(cfg.T_he))));

    // -----------------------------------------------------------------------
    // 2. 提取 ZYX 欧拉角，构造 XYZWPR，调用 IK
    // -----------------------------------------------------------------------
    std::vector<std::vector<double>> R(3, std::vector<double>(3));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = tcpPose[i][j];
    auto eul = rotm2eul_XYZ(R);

    XYZWPR xyzwpr;
    xyzwpr.x = tcpPose[0][3];
    xyzwpr.y = tcpPose[1][3];
    xyzwpr.z = tcpPose[2][3];
    xyzwpr.w = eul[0] * 180.0 / M_PI;
    xyzwpr.p = eul[1] * 180.0 / M_PI;
    xyzwpr.r = eul[2] * 180.0 / M_PI;
    //xyzwpr.x = 1670.87;
    //xyzwpr.y = 1454.259;
    //xyzwpr.z = 2095.818;
    //xyzwpr.w = 73;
    //xyzwpr.p = -3.4;
    //xyzwpr.r = -70.2;
    auto rawSols = KinematicsApi::instance().ik(cfg.robotName, xyzwpr);
    if (rawSols.empty())
        rawSols = KinematicsApi::instance().looseAngleIk(cfg.robotName, xyzwpr);
    if (rawSols.empty()) { ++g_reach.unreachIkEmpty; outJoints = {}; return false; }


    std::vector<JointVec6> solutions;
    for (const auto& sol : rawSols) {
        if (sol.size() < 6) continue;
          
        JointVec6 jv;
        for (int i = 0; i < 6; ++i) jv[i] = sol[i];
        solutions.push_back(jv);
    }
    if (solutions.empty()) { ++g_reach.unreachIkEmpty; outJoints = {}; return false; }

    // -----------------------------------------------------------------------
    // 3~5. 单循环依次筛选：关节限位 → 前后关节角约束 → 碰撞 → 自碰撞 → 构型判断
    //      对应 MATLAB robotReachability.m 第102~215行 for 循环结构
    // -----------------------------------------------------------------------

    // 提前计算前后关节角约束所需的 prevQ
    bool    hasPrev = false;
    JointVec6 prevQ = cfg.lastView;
    Matrix4d  prevPose;                       // 新增：上一个有效点的位姿（计算速度 dt 用）
    // 关节角约束已改为内部恒定启用，故 prevQ 始终需要计算（needVelCons 时亦然）
    {
        for (int i = static_cast<int>(lastViews.size()) - 1; i >= 0; --i) {
            bool allZero = true;
            for (int j = 0; j < 6; ++j)
                if (std::abs(lastViews[i].joints[j]) > 1e-6) { allZero = false; break; }
            if (!allZero) {
                prevQ    = lastViews[i].joints;
                prevPose = lastViews[i].globalT;   // 新增：取同一个点的位姿
                hasPrev  = true;
                break;
            }
        }
    }

    // ===== 新增：计算本步的时间增量 dt（仅速度约束需要）=====
    // dt = 上一点到当前点的笛卡尔直线距离 / MOVL 速度。
    // 距离越大说明走得越远、耗时越长，允许的关节运动量也越大，物理上自洽。
    double dt = 1e-6;   // 兜底，避免除零
    if (needVelCons && hasPrev) {
        double dx = tcpPose[0][3] - prevPose[0][3];
        double dy = tcpPose[1][3] - prevPose[1][3];
        double dz = tcpPose[2][3] - prevPose[2][3];
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        dt = (dist > 1e-6) ? dist / cfg.movlTcpSpeed : 1e-6;
    }

    std::vector<JointVec6> validSols;

    for (const auto& sol : solutions) {
        // --- 关节限位判断（对应 MATLAB 第107~112行）---
        bool jointValidFlag = true;
        for (int i = 0; i < 6; ++i) {
            if (sol[i] < cfg.jointLimits[i].minDeg ||
                sol[i] > cfg.jointLimits[i].maxDeg) {
                jointValidFlag = false; break;
            }
        }
        if (!jointValidFlag) { ++g_reach.brJointLimit; continue; }

        // --- 前后关节角约束（对应 MATLAB 第118~135行）---
        // 注：原 needJointCons 开关已删除，约束恒定启用，仅在有 prevQ 时生效
        if (hasPrev) {
            bool contOk = true;
            for (int i = 0; i < 6; ++i)
                if (std::abs(sol[i] - prevQ[i]) > cfg.anglesThresholds[i]) { contOk = false; break; }
            if (!contOk) { ++g_reach.brAngle; continue; }
        }

        // ===== 新增：关节角速度约束（仅 MOVL 直线预测开启 needVelCons 时生效）=====
        // 与上面的「关节角约束」区别：这里是角度差再除以 dt，限制的是角速度（度/秒），
        // 能识别「小笛卡尔位移 → 大关节位移」的奇异/腕翻分支。
        // 在加权最近解选优之前做分支过滤：超速分支被剔除，若仍有合规分支则点照常可达；
        // 若所有分支都超速则本函数返回 false，由外层走扰动 / RRT。
        if (needVelCons && hasPrev) {
            bool velOk = true;
            for (int i = 0; i < 6; ++i)
                if (std::abs(sol[i] - prevQ[i]) / dt > cfg.jointVelLimit[i]) { velOk = false; break; }
            if (!velOk) { ++g_reach.brVel; velKilledSomeInCall = true; continue; }   // 该 IK 分支角速度超限，剔除
        }

        // --- 环境碰撞检测（对应 MATLAB 第143~147行）---
        std::vector<double> jv(sol.begin(), sol.end());
        /*jv= {  1.82, 14.63,-24.48, 138.54, 102.21, -37.22};*/
        if (KinematicsApi::instance().checkCollision(cfg.robotName, jv, jv))
            { ++g_reach.brCollision; continue; }

        // --- 自碰撞检测（对应 MATLAB 第154~169行）---
        //auto fkRes2 = KinematicsApi::instance().fk(cfg.robotName, jv, static_cast<LinkName>(2));
        //auto fkRes3 = KinematicsApi::instance().fk(cfg.robotName, jv, static_cast<LinkName>(3));
        //auto fkRes4 = KinematicsApi::instance().fk(cfg.robotName, jv, static_cast<LinkName>(4));
        //auto extractPos = [](const FowardKinematicsReslutstd& res) -> Vec3d {
        //    if (res.ut.size() < 4) return {0, 0, 0};
        //    return {res.ut[0][3], res.ut[1][3], res.ut[2][3]};
        //};
        //Vec3d p2 = extractPos(fkRes2);
        //Vec3d p3 = extractPos(fkRes3);
        //Vec3d p4 = extractPos(fkRes4);
        //Vec3d n1 = {p2[0]-p3[0], p2[1]-p3[1], p2[2]-p3[2]};
        //Vec3d n2 = {p4[0]-p3[0], p4[1]-p3[1], p4[2]-p3[2]};
        //double len1 = std::sqrt(vec3_dot(n1, n1));
        //double len2 = std::sqrt(vec3_dot(n2, n2));
        //if (len1 > 1e-9 && len2 > 1e-9) {
        //    double cosA = vec3_dot(n1, n2) / (len1 * len2);
        //    cosA = std::max(-1.0, std::min(1.0, cosA));
        //    if (std::acos(cosA) * 180.0 / M_PI < 27.0) continue;
        //}

        //// --- 构型判断（对应 MATLAB 第175~206行）---
        //if (!checkPosture(sol, cfg)) { ++g_reach.brPosture; continue; }

        ++g_reach.brValid;
        validSols.push_back(sol);
    }

    if (validSols.empty()) {
        ++g_reach.unreachFiltered;
        if (velKilledSomeInCall) ++g_reach.unreachVelInvolved;
        outJoints = {};
        return false;
    }

    // -----------------------------------------------------------------------
    // 6. 加权最近解选优（对应 MATLAB 注释区第298~305行）
    // -----------------------------------------------------------------------
    double minDist = std::numeric_limits<double>::max();
    for (const auto& sol : validSols) {
        double d = 2.5 * std::pow(cfg.lastView[0] - sol[0], 2)
                 + 1.0 * (std::pow(cfg.lastView[1] - sol[1], 2)
                         + std::pow(cfg.lastView[2] - sol[2], 2))
                 + 3.0 * (std::pow(cfg.lastView[3] - sol[3], 2)
                         + std::pow(cfg.lastView[4] - sol[4], 2)
                         + std::pow(cfg.lastView[5] - sol[5], 2));
        if (d < minDist) { minDist = d; outJoints = sol; }
    }
    ++g_reach.reachable;
    return true;
}

// =============================================================================
// [诊断] 失败原因计数：清零 / 打印
// =============================================================================
void reachReasonReset()
{
    g_reach = ReachReasonCounter{};
}

void reachReasonDump(const char* tag)
{
    const long unreach = g_reach.unreachIkEmpty + g_reach.unreachFiltered;
    std::cerr << "[reachReason][" << (tag ? tag : "") << "]"
              << " calls=" << g_reach.calls
              << " reachable=" << g_reach.reachable
              << " unreachable=" << unreach
              << " | ikEmpty=" << g_reach.unreachIkEmpty
              << " filtered=" << g_reach.unreachFiltered
              << " (velInvolved=" << g_reach.unreachVelInvolved << ")"
              << " || branch: jointLimit=" << g_reach.brJointLimit
              << " angle=" << g_reach.brAngle
              << " vel=" << g_reach.brVel
              << " collision=" << g_reach.brCollision
              << " posture=" << g_reach.brPosture
              << " valid=" << g_reach.brValid
              << std::endl;
}

} // namespace robot_planner

// =============================================================================
// ViewpointPlanner.cpp  —  视点规划工具实现
//
// 通过 KinematicsApi::instance() 单例调用正逆解和碰撞检测。
// 面点生成逻辑已移至 SurfacePointGenerator.h，
// 转台旋转由 main.cpp 外层循环处理，此处不再涉及。
// =============================================================================

#include "ViewpointPlanner.h"
#include "RobotReachability.h"
#include "kinematicsApi.h"
#include "two_orientation_interpolation.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace robot_planner {

// =============================================================================
// 构造函数
// =============================================================================
ViewpointPlanner::ViewpointPlanner(RobotConfig& cfg)
    : m_cfg(cfg)
{
}

// =============================================================================
// computeOptimalX
// =============================================================================
Vec3d ViewpointPlanner::computeOptimalX(const Vec3d& zAxis,
                                        const Vec3d& presetX) const
{
    Vec3d x = presetX;
    double proj = dot3(x, zAxis);
    x[0] -= proj * zAxis[0];
    x[1] -= proj * zAxis[1];
    x[2] -= proj * zAxis[2];
    return normalize3(x);
}

// =============================================================================
// constructNextBestView
// =============================================================================
Matrix4d ViewpointPlanner::constructNextBestView(const Matrix4d& lastView,
                                                 const Vec3d&    newNormal) const
{
    // 提取旧Z轴（lastView第2列）
    Vec3d Z1 = {lastView[0][2], lastView[1][2], lastView[2][2]};

    // 归一化新Z轴
    Vec3d Z2 = normalize3(newNormal);

    // 计算旋转轴与角度
    Vec3d  axis     = cross3(Z1, Z2);
    double axisNorm = std::sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    double cosTheta = std::max(-1.0, std::min(1.0, dot3(Z1, Z2)));
    double theta    = std::acos(cosTheta);

    // 3×3 Rodrigues旋转矩阵 R（初始化为单位阵）
    double R[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

    if (axisNorm < 1e-10) {
        if (cosTheta < 0.0) {
            // Z1 ≈ -Z2（180°旋转）：选一个与Z1不平行的轴
            Vec3d arb = (std::abs(Z1[0]) < 0.9) ? Vec3d{1,0,0} : Vec3d{0,1,0};
            axis = normalize3(cross3(Z1, arb));
            double ax = axis[0], ay = axis[1], az = axis[2];
            // R = 2 * outer(axis,axis) - I
            R[0][0] = 2*ax*ax-1;  R[0][1] = 2*ax*ay;    R[0][2] = 2*ax*az;
            R[1][0] = 2*ay*ax;    R[1][1] = 2*ay*ay-1;  R[1][2] = 2*ay*az;
            R[2][0] = 2*az*ax;    R[2][1] = 2*az*ay;    R[2][2] = 2*az*az-1;
        }
        // else: Z1 ≈ Z2，旋转为单位阵，R保持不变
    } else {
        // 一般情况：Rodrigues公式 R = I + sin(θ)K + (1-cos(θ))K²
        axis[0] /= axisNorm;
        axis[1] /= axisNorm;
        axis[2] /= axisNorm;
        double ax = axis[0], ay = axis[1], az = axis[2];
        double s  = std::sin(theta);
        double c  = 1.0 - cosTheta;  // (1 - cos(theta))

        R[0][0] = 1 + c*(-(ay*ay+az*az));  R[0][1] = -s*az + c*ax*ay;          R[0][2] =  s*ay + c*ax*az;
        R[1][0] =  s*az + c*ax*ay;          R[1][1] = 1 + c*(-(ax*ax+az*az));  R[1][2] = -s*ax + c*ay*az;
        R[2][0] = -s*ay + c*ax*az;          R[2][1] =  s*ax + c*ay*az;          R[2][2] = 1 + c*(-(ax*ax+ay*ay));
    }

    // R2 = R * R1，其中R1为lastView左上角3×3
    Matrix4d T = eye4();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            T[i][j] = 0.0;
            for (int k = 0; k < 3; ++k)
                T[i][j] += R[i][k] * lastView[k][j];
        }
        T[i][3] = lastView[i][3];  // 平移不变
    }
    return T;
}

// =============================================================================
// buildTcpPoseLocal
// =============================================================================
Matrix4d ViewpointPlanner::buildTcpPoseLocal(const Vec3d& posLocal,
                                             const Vec3d& normalLocal,
                                             const Vec3d& xAxisLocal) const
{
    Vec3d z = normalize3(normalLocal);
    Vec3d x = computeOptimalX(z, xAxisLocal);
    Vec3d y = normalize3(cross3(z, x));
    x = cross3(y, z);

    Matrix4d T = eye4();
    for (int i = 0; i < 3; ++i) {
        T[i][0] = x[i];
        T[i][1] = y[i];
        T[i][2] = z[i];
        T[i][3] = posLocal[i];
    }
    return T;
}

// =============================================================================
// makeZRotAroundPoint  —  绕 Z 轴（过 axisPoint）旋转 angleDeg 度的变换矩阵
// =============================================================================
Matrix4d ViewpointPlanner::makeZRotAroundPoint(const Vec3d& axisPoint, double angleDeg)
{
    double a  = angleDeg * M_PI / 180.0;
    double ca = std::cos(a), sa = std::sin(a);
    double px = axisPoint[0], py = axisPoint[1];
    Matrix4d T = eye4();
    T[0][0] = ca;  T[0][1] = -sa;  T[0][3] = px * (1.0 - ca) + py * sa;
    T[1][0] = sa;  T[1][1] =  ca;  T[1][3] = py * (1.0 - ca) - px * sa;
    return T;
}

// =============================================================================
// tryLocalPerturbation  —  Z移动 × Z旋转 双重扰动搜索（内部共用）
// 搜索序列: [0°, +5°, -5°, +10°, -10°, ..., +45°, -45°]
// =============================================================================
bool ViewpointPlanner::tryLocalPerturbation(
    const Matrix4d&               baseTcp,
    const std::vector<ViewPoint>& lastViews,
    const std::vector<double>&    searchAnglesRad,
    Matrix4d&                     outNewTcp,
    JointVec6&                    outJoints,
    bool                          needVelCons) const   // 新增参数（默认值只写在 .h）
{
    // Z轴方向 (col 2) 和初始位置 (col 3)
    Vec3d z_axis   = {baseTcp[0][2], baseTcp[1][2], baseTcp[2][2]};
    Vec3d trans_init = {baseTcp[0][3], baseTcp[1][3], baseTcp[2][3]};

    for (int step = 0; step <= m_cfg.zMoveSteps; ++step) {
        double offset = step * m_cfg.zMoveDistance;
        Vec3d cur_trans = {
            trans_init[0] + offset * z_axis[0],
            trans_init[1] + offset * z_axis[1],
            trans_init[2] + offset * z_axis[2]
        };

        for (double ang_rad : searchAnglesRad) {
            double ca = std::cos(ang_rad), sa = std::sin(ang_rad);

            // R_new = R_initial * Rz_local，其中 Rz_local 绕局部 Z 轴旋转
            // col0_new = ca*col0 + sa*col1
            // col1_new = -sa*col0 + ca*col1
            // col2_new = col2
            Matrix4d newT = eye4();
            for (int r = 0; r < 3; ++r) {
                newT[r][0] =  ca * baseTcp[r][0] + sa * baseTcp[r][1];
                newT[r][1] = -sa * baseTcp[r][0] + ca * baseTcp[r][1];
                newT[r][2] = baseTcp[r][2];
                newT[r][3] = cur_trans[r];
            }

            JointVec6 out_j;
            //newT = {
            //    {-0.342015904932513, - 0.939681316913906,	0.00491359503282464,	1524.25577215020},
            //{ 0.939692579835501, - 0.342020252541154, - 4.74740573638481e-05,	457.647113434946 },
            //{0.00172515949875456,	0.00460103190997179,	0.999987927092156,	26.9798486921925},
            //    {0,0,0,1}
            //};
            // 末尾透传 needVelCons：扰动搜索的每个候选姿态也按速度门槛逐分支筛选
            if (robot_planner::robotReachability(newT, m_cfg, lastViews, out_j, needVelCons)) {
                outNewTcp = newT;
                outJoints = out_j;
                return true;
            }
        }
    }
    return false;
}

// =============================================================================
// adjustViewPose  —  视点姿态扰动补救（无转台版本）
// 对应 adjustViewPoseMeasNoTurntable.m：
// 只在当前姿态做 Z移动×Z旋转 双重搜索，不再遍历转台候选角度
// =============================================================================
bool ViewpointPlanner::adjustViewPose(
    const Matrix4d&               originalTcp,
    const std::vector<ViewPoint>& lastViews,
    Matrix4d&                     outNewTcp,
    JointVec6&                    outJoints,
    double*                       outPartRotateAngle,
    bool                          needVelCons) const   // 新增参数（默认值只写在 .h）
{
    // 构建旋转搜索序列: [0°, +5°, -5°, +10°, -10°, ..., +45°, -45°]
    std::vector<double> searchAnglesRad;
    searchAnglesRad.push_back(0.0);
    for (int deg = 5; deg <= 45; deg += 5) {
        searchAnglesRad.push_back( deg * M_PI / 180.0);
        searchAnglesRad.push_back(-deg * M_PI / 180.0);
    }

    // 当前姿态下的局部扰动搜索：
    // 1) 沿视点局部 Z 轴做平移
    // 2) 视点自身绕局部 Z 轴做角度扰动
    // 末尾透传 needVelCons，让扰动搜索内部的可达性判断也带上速度门槛
    if (tryLocalPerturbation(originalTcp, lastViews, searchAnglesRad, outNewTcp, outJoints, needVelCons)) {
        if (outPartRotateAngle) *outPartRotateAngle = 0.0;
        return true;
    }
    outNewTcp = originalTcp;

    outJoints = {};
    if (outPartRotateAngle) *outPartRotateAngle = 0.0;
    return false;
}

// =============================================================================
// linePathPredict  —  直线路径预测（对应 MATLAB: LinePathPredict.m）
//
// 算法：
//   1) twoOrientationInterpolation 在 TCP 空间生成 N 个中间位姿（不含起终点）
//   2) 从 index=1 开始（跳过第一个中间帧，与 MATLAB for idx=2:N 一致）
//   3) 对每帧做 robotReachability（IK可达性），不可达时调用 adjustViewPose 扰动
//
// 输出点的 flag 约定：
//   · 直接可达：isImportant=false, isPathJoint=false, moveType=1
//   · 扰动成功：isImportant=true,  isPathJoint=true,  moveType=1
//     （isImportant=true 是必需的——path_rrt_splice 的单一过滤规则只看
//      isImportant，若为 false 会被一刀切掉）
//   · 扰动失败：不丢点，压入一个 moveType=-2 的占位标记，joints 全 NaN，
//     globalT 暂存 currentPose。该占位点等待 Pass 2（markAndCleanMinusTwo
//     in path_rrt_splice.cpp）在外层做整体清理——所有 -2 删除、左右邻居
//     moveType 改为 -1，构成 path_rrt_splice 识别的 "连续 -1 run"，
//     从而触发 PathPlan 的 RRT 规划。
// =============================================================================
std::vector<ViewPoint> ViewpointPlanner::linePathPredict(
    const ViewPoint& startVp,
    const ViewPoint& endVp) const
{
    static const double kNaN = std::numeric_limits<double>::quiet_NaN();

    // 步骤0：结果初始化，第一个元素是起点
    // 对应 MATLAB: processing_viewpoints = [{start_info_in}]
    std::vector<ViewPoint> result;
    result.push_back(startVp);

    // 步骤1：TCP 空间中间位姿插值
    // 对应 MATLAB: interViewPose = twoOrientationInterpolation(T_start, T_end, config)
    // 返回 N 个中间位姿（不含起点和终点），每 5mm 取一个插值点
    std::vector<Matrix4d> interPoses =
        twoOrientationInterpolation(startVp.globalT, endVp.globalT, m_cfg);

    // N==0 时直接返回，只有起点（对应 MATLAB: if isempty(interViewPose), return）
    if (interPoses.empty()) {
        return result;
    }

    // 步骤2：从 index=1 开始逐帧做 IK 可达性检测
    // 对应 MATLAB: for idx = 2:size(interViewPose, 3)   ← 跳过第一个中间帧(idx=1)
    for (size_t idx = 1; idx < interPoses.size(); ++idx) {
        const Matrix4d& currentPose = interPoses[idx];

        // 构建有效历史：过滤 result 中 joints 含 NaN 的点（不可达点不能作为 IK 种子）
        // 对应 MATLAB: processing_viewpoints 中 joint=[] 的点不影响 IK 种子选取
        std::vector<ViewPoint> validHistory;
        validHistory.reserve(result.size());
        for (const auto& vp : result) {
            bool hasNaN = false;
            for (int i = 0; i < 6; ++i) {
                if (std::isnan(vp.joints[i])) { hasNaN = true; break; }
            }
            if (!hasNaN) validHistory.push_back(vp);
        }

        JointVec6 outJoints{};
        // 对应 MATLAB: robotReachability(current_pose, config, partRotateAngle, processing_viewpoints)
        // 末尾 needVelCons=true：这是 MOVL 直线预测链路，开启关节角速度约束
        // （前后关节角约束已在函数内部恒定启用，无需再传参）
        bool isReachable = robot_planner::robotReachability(
            currentPose, m_cfg, validHistory, outJoints, /*needVelCons=*/true);

        if (isReachable) {
            // 可达：标记为普通中间点（非关键节点），processLineResult 末尾会过滤掉
            // 对应 MATLAB: isImportant=0, isPathJoint=0, move_type=1
            ViewPoint vp;
            vp.globalT         = currentPose;
            vp.joints          = outJoints;
            vp.moveType        = 1;
            vp.isImportant     = false;
            vp.isPathJoint     = false;
            vp.partRotateAngle = startVp.partRotateAngle;
            result.push_back(vp);
        } else {
            // 不可达：尝试沿 Z 轴扰动（同样使用过滤后的 validHistory）
            // 对应 MATLAB: adjustViewPosePlace(current_pose, config, 0, processing_viewpoints)
            Matrix4d newTcp;
            JointVec6 adjJoints{};
            // 末尾 needVelCons=true：扰动同样受速度约束（须显式传 outPartRotateAngle=nullptr 占位）
            bool adjusted = adjustViewPose(currentPose, validHistory, newTcp, adjJoints,
                                           /*outPartRotateAngle=*/nullptr, /*needVelCons=*/true);

            if (adjusted) {
                // 扰动成功：isImportant=true 是 path_rrt_splice 单一过滤
                // 规则的必要条件，必须保留这个机器人被迫绕道的真实姿态
                ViewPoint vp;
                vp.globalT         = newTcp;
                vp.joints          = adjJoints;
                vp.moveType        = 1;
                vp.isImportant     = true;
                vp.isPathJoint     = true;
                vp.partRotateAngle = startVp.partRotateAngle;
                result.push_back(vp);
            } else {
                // 扰动仍不可达：压入 mt=-2 占位标记，joints 设为 NaN
                // Pass 2（markAndCleanMinusTwo @ path_rrt_splice.cpp）会
                // 整体删除所有 -2、把左右邻居 moveType 改 -1，从而触发 RRT
                ViewPoint vp;
                vp.globalT         = currentPose;
                vp.joints          = {kNaN, kNaN, kNaN, kNaN, kNaN, kNaN};
                vp.moveType        = -2;
                vp.isImportant     = false;
                vp.isPathJoint     = false;
                vp.partRotateAngle = startVp.partRotateAngle;
                result.push_back(vp);
                std::cout << "[linePathPredict] 扰动失败 idx=" << idx
                          << "，压入 mt=-2 标记，等待 Pass 2 处理\n";
            }
        }
    }

    // 对应 MATLAB: 循环结束后不追加终点，终点由外层下一段的起点承接
    return result;
}

// =============================================================================
// 旋转矩阵工具（3x3，纯旋转矩阵运算，不用四元数）——供 jointSpaceInterpolation 做
// 「绕固定轴 Rodrigues 旋转插值」用。（新增）
//
// Rodrigues 公式与 BallScan.cpp analyzeStandardPoints 的 rotAboutNormal 绕原始法向
// 旋转完全一致；从旋转矩阵抽轴角是其逆过程。前缀 jsi 避免撞名，不另造新算法。
// =============================================================================
namespace {

using JsiMat3 = std::array<std::array<double, 3>, 3>;
using JsiVec3 = std::array<double, 3>;

// 取 Matrix4d 的 3x3 旋转部分
JsiMat3 jsiRot3(const Matrix4d& T)
{
    JsiMat3 R{};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            R[r][c] = T[r][c];
    return R;
}

JsiMat3 jsiTranspose3(const JsiMat3& A)
{
    JsiMat3 R{};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            R[r][c] = A[c][r];
    return R;
}

JsiMat3 jsiMul3(const JsiMat3& A, const JsiMat3& B)
{
    JsiMat3 R{};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += A[r][k] * B[k][c];
            R[r][c] = s;
        }
    return R;
}

// 绕单位轴 axis 旋转 angle（弧度）的旋转矩阵——Rodrigues 公式
// R = I*cos + (1-cos)*a*aᵀ + sin*[a]_x
// 与 analyzeStandardPoints 的 rotAboutNormal 同一公式。
JsiMat3 jsiRodrigues(const JsiVec3& axis, double angle)
{
    const double c = std::cos(angle), s = std::sin(angle), t = 1.0 - c;
    const double x = axis[0], y = axis[1], z = axis[2];
    return JsiMat3{ {
        { c + t*x*x,     t*x*y - s*z,   t*x*z + s*y },
        { t*x*y + s*z,   c + t*y*y,     t*y*z - s*x },
        { t*x*z - s*y,   t*y*z + s*x,   c + t*z*z   }
    } };
}

} // anonymous namespace

// =============================================================================
// jointSpaceInterpolation  —  同位置换姿离散产点器（与 twoOrientationInterpolation 对仗）
//
// 角色和 twoOrientationInterpolation 完全一致：只负责在「起点位姿 → 终点位姿」之间
// 产出一串中间离散位姿（Matrix4d），不做任何可达性 / 碰撞判断——可达性留给外部
// measLinePathPredict 循环（robotReachability 单点检测）。用于 twoOrientation
// 退化（TCP 距离≈0，同位置换姿：Standard 锥形之间 / 切边 _R 副本）时的兜底产点。
//
// 【关键：直接在 globalT（世界系）上插值，绝不 FK】
//   入参 T0/T1 必须是 startVp.globalT / endVp.globalT——与 twoOrientationInterpolation
//   完全同一坐标系，也正是下游 robotReachability(currentPose) 做 IK 所期望的世界系。
//   早先版本对 joints 做 FK(KinematicsApi::fk) 取位姿，得到的是机器人基座系 TCP，
//   不含 globalT 里烘焙的滑轨(slidePos)/转台(partRotateAngle)偏移，frame 不一致 →
//   IK 解出完全不同的构型 → 关节漂移、XYZ 错位。故此处不再 FK。
//
// 算法：从相对旋转抽轴 → 绕固定轴 Rodrigues 旋转插值。
//   1) 相对旋转 R_rel = R1 * R0ᵀ，从中直接抽出转轴 axis 与总转角 angle
//      （同位置换姿时 axis 即测点原始 z 法向，与 analyzeStandardPoints 绕的轴一致）；
//   2) 中间帧旋转 = Rodrigues(axis, t*angle) * R0，绕这根固定轴单调滚转；
//   3) 位置按 t 线性插值（同位置时≈固定不动）。
// 与 BallScan.cpp analyzeStandardPoints 沿原始法向扫锥的产点方式完全对仗。步数按姿态
// 旋转角自适应；返回不含起终点（与 twoOrientation 对齐）。全程旋转矩阵运算，不用四元数。
// =============================================================================
static std::vector<Matrix4d> jointSpaceInterpolation(
    const Matrix4d& T0, const Matrix4d& T1, const RobotConfig& cfg)
{
    std::vector<Matrix4d> poses;
    if (T0.size() < 4 || T1.size() < 4) return poses;
    for (int r = 0; r < 4; ++r)
        if (T0[r].size() < 4 || T1[r].size() < 4) return poses;

    // 两端位置（世界系；同位置换姿时 dist≈0）
    const double p0x = T0[0][3], p0y = T0[1][3], p0z = T0[2][3];
    const double p1x = T1[0][3], p1y = T1[1][3], p1z = T1[2][3];
    const double dx = p1x - p0x, dy = p1y - p0y, dz = p1z - p0z;
    const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    // 相对旋转 R_rel = R1 * R0ᵀ，从中抽轴角（无四元数）
    const JsiMat3 R0    = jsiRot3(T0);
    const JsiMat3 R1    = jsiRot3(T1);
    const JsiMat3 R_rel = jsiMul3(R1, jsiTranspose3(R0));

    double cosAng = (R_rel[0][0] + R_rel[1][1] + R_rel[2][2] - 1.0) * 0.5;
    cosAng = std::max(-1.0, std::min(1.0, cosAng));
    const double angle    = std::acos(cosAng);              // 总转角（弧度）
    const double angleDeg = angle * 180.0 / M_PI;
    const double sinAng   = std::sin(angle);

    // 退化：姿态与位置都几乎不变 → 无中间点（与原 twoOrientation 退化行为一致）
    if (angleDeg < 1e-6 && dist < 1e-6)
        return poses;

    // 从 R_rel 反对称部分抽转轴；sinAng→0（转角≈0 或≈180°）时轴不稳定，置标志兜底
    JsiVec3 axis{ 0.0, 0.0, 1.0 };
    const bool axisStable = std::abs(sinAng) > 1e-6;
    if (axisStable) {
        const double inv = 1.0 / (2.0 * sinAng);
        axis = {
            (R_rel[2][1] - R_rel[1][2]) * inv,
            (R_rel[0][2] - R_rel[2][0]) * inv,
            (R_rel[1][0] - R_rel[0][1]) * inv
        };
        const double an = std::sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
        if (an > 1e-12) { axis[0] /= an; axis[1] /= an; axis[2] /= an; }
    }

    // 自适应步数：按姿态旋转角 / 步距向上取整保证密度
    const double stepDeg = (cfg.jointInterpStepDeg > 1e-6) ? cfg.jointInterpStepDeg : 1.5;
    const int orientN = static_cast<int>(std::ceil(angleDeg / stepDeg));
    const int N = std::max(1, orientN);

    // s 从 1 到 N-1，不含起终点（与 twoOrientation 对齐）
    for (int s = 1; s < N; ++s) {
        const double t = static_cast<double>(s) / N;

        // 旋转：绕固定轴转 t 比例的角 → 叠加到起点姿态。轴不稳定时直接用起点姿态
        // （转角≈0，姿态几乎不变；纯位置插值由下方平移列承担）。
        JsiMat3 R = R0;
        if (axisStable) {
            const JsiMat3 Rt = jsiRodrigues(axis, t * angle);
            R = jsiMul3(Rt, R0);
        }

        Matrix4d T = eye4();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                T[r][c] = R[r][c];
        T[0][3] = (1.0 - t) * p0x + t * p1x;   // 位置线性插值（同位置时≈固定不动）
        T[1][3] = (1.0 - t) * p0y + t * p1y;
        T[2][3] = (1.0 - t) * p0z + t * p1z;
        poses.push_back(std::move(T));
    }
    return poses;
}

// =============================================================================
// measLinePathPredict  —  直线路径预测（对应 MATLAB: MeasLinePathPredict.m）
//
// 与 linePathPredict 的两处差异：
//   1) 插补点循环从 idx=0 开始（MATLAB for idx=1:N，包含全部插补点）
//      linePathPredict 从 idx=1 跳过第一帧（MATLAB for idx=2:N）
//   2) 遇到不可达点时，先将 result.back() 的 isImportant 置 true，
//      再调用 adjustViewPose 扰动（linePathPredict 无此步骤；MATLAB
//      heritage：processing_viewpoints{end}.isImportant = 1，扰动是否
//      成功都执行）
//
// 输出点的 flag 约定：
//   · 直接可达：isImportant=false, isPathJoint=false, moveType=1
//   · 扰动成功：isImportant=true,  isPathJoint=true,  moveType=1
//     （isImportant=true 是必需的，path_rrt_splice 单一过滤会丢 false 的）
//   · 扰动失败：不丢点，压入 moveType=-2 占位标记（joints=NaN，
//     globalT=currentPose）。Pass 2（markAndCleanMinusTwo
//     @ path_rrt_splice.cpp）会整体删除 -2、左右邻居 moveType 改 -1，
//     触发 PathPlan 的 RRT 规划。
// =============================================================================
std::vector<ViewPoint> ViewpointPlanner::measLinePathPredict(
    const ViewPoint& startVp,
    const ViewPoint& endVp) const
{
    static const double kNaN = std::numeric_limits<double>::quiet_NaN();

    std::vector<ViewPoint> result;
    result.push_back(startVp);

    std::vector<Matrix4d> interPoses =
        twoOrientationInterpolation(startVp.globalT, endVp.globalT, m_cfg);

    // twoOrientationInterpolation 退化（TCP 距离≈0，同位置换姿：Standard 锥形之间 /
    // 切边 _R 副本）时返回空。改用 jointSpaceInterpolation 在「位置固定 + 绕固定轴
    // 旋转」上均匀取点。注意传 globalT（世界系）而非 joints——必须与上面
    // twoOrientationInterpolation 同一坐标系，下游 robotReachability(currentPose) 做
    // IK 才不会漂移。产出的位姿同样塞进 interPoses，后续 for 循环 / robotReachability
    // 单点可达性 / 压 -2 / RRT 全部原样消费。jointSeg 标记本段为关节插值段，使可达点
    // 标 MOVJ 而非 MOVL（同位置笛卡尔大角度换姿会触发脉冲极限）。（修改）
    bool jointSeg = false;
    if (interPoses.empty()) {
        interPoses = jointSpaceInterpolation(startVp.globalT, endVp.globalT, m_cfg);
        jointSeg = true;
    }

    if (interPoses.empty()) {
        return result;   // 关节空间也产不出（起终点几乎重合）才退场
    }

    // 从 idx=0 开始（对应 MATLAB: for idx=1:N，包含全部插补点）
    for (size_t idx = 0; idx < interPoses.size(); ++idx) {
        const Matrix4d& currentPose = interPoses[idx];

        std::vector<ViewPoint> validHistory;
        validHistory.reserve(result.size());
        for (const auto& vp : result) {
            bool hasNaN = false;
            for (int i = 0; i < 6; ++i) {
                if (std::isnan(vp.joints[i])) { hasNaN = true; break; }
            }
            if (!hasNaN) validHistory.push_back(vp);
        }

        JointVec6 outJoints{};
        // needVelCons 仅笛卡尔段开启：MOVL 测边直线预测靠「笛卡尔位移 → dt」识别
        // 奇异/腕翻分支。关节插值段（jointSeg，同位置换姿）TCP 位置不动，相邻点笛卡尔
        // 距离≈0 → dt 跌到兜底 1e-6 → |Δq|/dt 必爆、每个 IK 分支都被速度约束误杀，
        // 故该段关掉速度约束。腕翻仍由函数内恒定启用的「前后关节角连续性约束」
        // (anglesThresholds) 拦截（插值点已密，相邻 Δq 很小自然能过），MOVJ 执行期
        // 控制器自带关节限速兜底。（修改）
        // （前后关节角约束已在函数内部恒定启用，无需再传参）
        bool isReachable = robot_planner::robotReachability(
            currentPose, m_cfg, validHistory, outJoints, /*needVelCons=*/!jointSeg);

        if (isReachable) {
            ViewPoint vp;
            vp.globalT         = currentPose;
            vp.joints          = outJoints;
            vp.moveType        = jointSeg ? 0 : 1;   // 关节插值段 MOVJ，笛卡尔段 MOVL
            //vp.moveType = 1;
            vp.isImportant     = false;
            // 关节插值段（同位置换姿）现产出「TCP 固定 + 姿态 SLERP」的密集位姿：
            // 其可达中间点必须保留为真实路点（isPathJoint=true），才能在
            // processLineResult 存活、组成密集 MOVJ 链；否则路径塌成单条大 MOVJ，
            // 执行期控制器按关节线性插值仍会让 TCP 鼓出去。笛卡尔段保持 false 不变。（修改）
            vp.isPathJoint = jointSeg;
            vp.partRotateAngle = startVp.partRotateAngle;
            result.push_back(vp);
        } else {
            // ───────────────────────────────────────────────────────────────
            // 不可达分支：取消 adjustViewPose 扰动补救，直接压 mt=-2 交给 RRT。
            //
            // 【为什么去掉扰动】
            //   原实现对不可达点做“沿工具 Z 平移 × 绕工具 Z 旋转”扰动，成功就把
            //   扰动后的位姿保留为 MOVL 绕行点(isImportant=isPathJoint=true)。但：
            //   ① 该绕行点已偏离原直线；
            //   ② 其后续插值点仍在原直线上、IK 种子接在扰动点之后被判为“直接可达”，
            //      随后被 keepOnlyKeyNodesAtEnd 删除，整段塌成一条跨大翻腕的长 MOVL；
            //   ③ MotoSim 对这条长 MOVL 自行重插值解 IK，腕部(B/T 轴)大角度扫掠
            //      撞上栏杆——端点静态可达，中间扫掠体却碰撞。
            //   因此改为：不可达 → 压 -2，整段交 RRT 在真实碰撞环境下重新规划。
            //
            // 【为什么单点压 -2 通常就能让整段走 RRT】
            //   robotReachability 的“前后关节角约束 / 角速度约束”都是相对“上一个
            //   有效点”prevQ 的；-2 点 joints=NaN 会被 validHistory 过滤掉，故后续
            //   插值点的 prevQ 仍停在本段最后一个可达点上。大翻腕段里越靠后的插值点
            //   相对同一 prevQ 的角度差只会更大 → 连锁不可达 → 整段内部一起变 -2 →
            //   markAndCleanMinusTwo 把左右括号落到 startVp / endVp → RRT 正好规划
            //   整段 startVp→endVp。
            //   (注：若某点是因“静态碰撞 / 关节限位 / 构型”等位姿相关原因单独不可达、
            //    不形成连锁，则此处仅产生局部 -1 run，由 RRT 补该小段。)
            // ───────────────────────────────────────────────────────────────

            // 先标记前一个可达点为 isImportant=true（保留原 MeasLinePathPredict 行为）。
            // 对应 MATLAB: processing_viewpoints{end}.isImportant = 1。
            // 作用：保证 processLineResult 在该处断开 MOVL 段，且该点作为 -1 run 的
            // 左括号能被 keepOnlyKeyNodesAtEnd 保留下来。
            if (result.size() > 1) {
                result.back().isImportant = true;
            }

            // 压入 mt=-2 占位标记，joints 设为 NaN。
            // Pass 2(markAndCleanMinusTwo)会整体删除所有 -2、把左右最近邻的 moveType
            // 改为 -1，构成 path_rrt_splice 可识别的“连续 -1 run”，进而触发 RRT 规划。
            ViewPoint vp;
            vp.globalT         = currentPose;
            vp.joints          = {kNaN, kNaN, kNaN, kNaN, kNaN, kNaN};
            vp.moveType        = -2;
            vp.isImportant     = false;
            vp.isPathJoint     = false;
            vp.partRotateAngle = startVp.partRotateAngle;
            result.push_back(vp);
            std::cout << "[measLinePathPredict] 不可达 idx=" << idx
                      << "，取消扰动，直接压 mt=-2 交 RRT 处理\n";
        }
    }

    return result;
}

// =============================================================================
// processLineResult  —  路径后处理
// =============================================================================
std::vector<std::array<double, 8>> ViewpointPlanner::processLineResult(
    const std::vector<ViewPoint>& viewpoints) const
{
    std::vector<std::array<double, 8>> result;
    for (const auto& vp : viewpoints) {
        if (!vp.isImportant && !vp.isPathJoint) continue;
        std::array<double, 8> row;
        for (int i = 0; i < 6; ++i) row[i] = vp.joints[i];
        row[6] = vp.partRotateAngle;
        row[7] = m_cfg.slidePos;
        result.push_back(row);
    }
    return result;
}

// =============================================================================
// planMoveJ  —  关节空间路径规划
// =============================================================================
//ViewpointPlanner::MoveJResult ViewpointPlanner::planMoveJ(
//    const JointVec6& startDeg,
//    const JointVec6& endDeg) const
//{
//    MoveJResult res;
//
//    std::vector<double> mjSv(startDeg.begin(), startDeg.end());
//    std::vector<double> mjEv(endDeg.begin(), endDeg.end());
//    if (!KinematicsApi::instance().checkCollision(m_cfg.robotName, mjSv, mjEv)) {
//        res.path   = {startDeg, endDeg};
//        res.isSafe = true;
//        res.hasMid = false;
//        return res;
//    }
//
//    auto rrtPath = m_biRrtPlanner.plan(startDeg, endDeg);
//    if (rrtPath.empty()) {
//        res.isSafe = false;
//        return res;
//    }
//
//    res.path   = rrtPath;
//    res.isSafe = true;
//    res.hasMid = rrtPath.size() > 2;
//    if (res.hasMid)
//        res.midJoint = rrtPath[rrtPath.size() / 2];
//    return res;
//}

// =============================================================================
// 静态数学工具
// =============================================================================
Vec3d ViewpointPlanner::normalize3(const Vec3d& v)
{
    double n = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (n < 1e-10) return {0, 0, 1};
    return {v[0]/n, v[1]/n, v[2]/n};
}

double ViewpointPlanner::dot3(const Vec3d& a, const Vec3d& b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

Vec3d ViewpointPlanner::cross3(const Vec3d& a, const Vec3d& b)
{
    return {
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}

Matrix4d ViewpointPlanner::multiplyTransforms(const Matrix4d& A, const Matrix4d& B)
{
    Matrix4d C(4, std::vector<double>(4, 0.0));
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Matrix4d ViewpointPlanner::invertTransform(const Matrix4d& T)
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

} // namespace robot_planner

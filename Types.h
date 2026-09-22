#pragma once
// =============================================================================
// Types.h  —  robot_planner 项目专用数据类型
//
// 说明：
//   - 扩展 parameters_type.h 中已有的通用类型（JointAngles、XYZWPR 等）
//   - 此文件只定义规划器内部使用的复合类型，不重复定义 SDK 已有类型
//   - 对应 MATLAB：数据结构（struct/cell 数组）
// =============================================================================

#include <array>
#include <vector>
#include <string>
#include <memory>
#include <limits>
#include <cmath>

// 引入二次开发包基础类型
#include "parameters_type.h"

// 前向声明（PQP 来自 m_include/PQP.h）
struct PQP_Model;

namespace robot_planner {

// -----------------------------------------------------------------------------
// 基础线性代数类型
// -----------------------------------------------------------------------------

// 4×4 齐次变换矩阵（行优先，与 parameters_type.h 的 t_gom 格式一致）
// row-major: T[row][col]
using Matrix4d = std::vector<std::vector<double>>;

// 创建 4×4 单位矩阵
inline Matrix4d eye4()
{
    return {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
}

// 三维向量（x, y, z）
using Vec3d = std::array<double, 3>;

// 6 关节角向量（度或弧度，由上下文决定）
using JointVec6 = std::array<double, 6>;

// -----------------------------------------------------------------------------
// 点云类型
// N×3 坐标列表，每个点为 {x, y, z}（单位 mm）
// 对应 MATLAB：N×3 的 double 矩阵
// -----------------------------------------------------------------------------
using PointCloud = std::vector<Vec3d>;

// 带法向量的点
struct PointWithNormalLocal {
    Vec3d pos;       // 位置 (x, y, z) mm
    Vec3d normal;    // 法向量 (nx, ny, nz)，已归一化
};

// -----------------------------------------------------------------------------
// 视点信息
// 对应 MATLAB：initViewPoints{index} / LineProcessResult{index} 等 struct
// -----------------------------------------------------------------------------
struct ViewPoint {
    Matrix4d  globalT;              // 视点在用户坐标系下的 4×4 齐次变换矩阵
    JointVec6 joints       = {};    // 对应关节角（度）
    bool      isImportant  = false; // 是否为关键视点（行首/行末）
    bool      isPathJoint  = false; // 是否为过渡点（碰撞扰动后插入）
    int       moveType     = 1;     // 0 = MoveJ（关节运动），1 = MoveL（直线运动）
    double    partRotateAngle = 0.0;// 零件当前对应的转台旋转角度（度）
    Matrix4d  partTransform;        // 零件当前对应的转台旋转变换矩阵（4×4）

    // 原始测点元数据，仅用于在路径组装阶段保持“一个基准测点生成的全部姿态”
    // 为一个不可拆分的连续组，并按原始 XYZ 与弓字形面点匹配。路径插值/RRT
    // 生成的中间点可以保持默认值；它们不参与 aaa 的初始排序。
    std::string sourcePointId;
    Vec3d       sourcePointPosition = {};
    bool        hasSourcePointPosition = false;
};

// -----------------------------------------------------------------------------
// BiRRT 树节点
// 对应 MATLAB：Tree1.v(i) / Tree2.v(i) struct
// -----------------------------------------------------------------------------
struct RRTNode {
    JointVec6 theta     = {};  // 当前节点关节角（弧度）
    JointVec6 thetaPrev = {};  // 父节点关节角（弧度）
    double    dist      = 0.0; // 与父节点的关节空间距离
    int       indPrev   = 0;   // 父节点索引（1-based，与 MATLAB 保持一致）
    bool      lastFlag  = false; // 是否为叶节点（最新扩展节点）
};

// BiRRT 树
struct RRTTree {
    std::vector<RRTNode> nodes; // nodes[0] = 根节点（起点或终点）
};

// -----------------------------------------------------------------------------
// 规划结果
// 对应 MATLAB：total_result（N×8 矩阵）
// -----------------------------------------------------------------------------
struct PlanResult {
    std::vector<ViewPoint> waypoints;  // 所有规划路径点（有序）
    bool        success  = false;
    std::string errorMsg;
};

// -----------------------------------------------------------------------------
// BVH 场景（一个转台旋转角度对应的碰撞场景）
// 对应 MATLAB：BVH{index} struct
// -----------------------------------------------------------------------------
struct BVHScene {
    double   rotateAngle = 0.0;     // 转台旋转角度（度）
    Matrix4d transformMatrix;       // 对应的 4×4 旋转变换矩阵

    // PQP 碰撞模型（加载后有效）
    std::shared_ptr<PQP_Model> partModel;       // 零件
    std::shared_ptr<PQP_Model> fixtureModel;    // 夹具
    std::shared_ptr<PQP_Model> measTableModel;  // 测量台
    std::shared_ptr<PQP_Model> envModel;        // 环境

    // 旋转后的点云（用于可视化/范围计算）
    PointCloud rotatedPoints;       // 旋转后零件点云
    PointCloud rotatedFixture;      // 旋转后夹具点云
    PointCloud rotatedMeasTable;    // 旋转后转台点云
    PointCloud env;                 // 环境点云（不旋转）
};

// -----------------------------------------------------------------------------
// 滑动窗口栅格单元
// 对应 MATLAB：sub_clouds{index} struct
// -----------------------------------------------------------------------------
struct GridCell {
    Vec3d  centerPos    = {};      // 栅格中心在局部坐标系下的位置（mm）
    Vec3d  centerNormal = {};      // 栅格中心对应的法向量（归一化）
    Vec3d  xAxis        = {};      // 视点 X 轴方向（扫描方向）
    bool   isImportant  = false;   // 是否为行首/行末关键点
};

} // namespace robot_planner

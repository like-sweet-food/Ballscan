// =============================================================================
// path_rrt_splice.cpp  —  spliceRRTIntoDensePath 实现
//
// 算法：
//   1. 一次性扫描 dense，找出所有"连续 moveType=-1 run"，长度 >= 2 的视为
//      有效 gap；长度为 1 的孤儿 -1 跳过，不触发 RRT（由日志提醒上游
//      measLinePathPredict 段首/段尾 corner case）。
//   2. 对每个有效 run [L_idx, R_idx] 调一次 PathPlan::PlanPathRRTconnect_new：
//        · 成功 → run 内 L_idx / R_idx 保留为输出端点，中间 RRT 点插入；
//                 run 内其他 -1 点（如长 run 中间的 9.7）一律丢弃。
//        · 失败 → run 内所有 -1 点降级为 MOVL（isImportant=true, moveType=1），
//                 不插中间点。
//   3. 最终输出做一次稀疏化：只保留 isImportant=true 的点。
//
// 与 process_line_result.cpp 的关系：
//   planRRTBetween / buildMeasurePointPoseSetFromViewPoint 这两个函数是从
//   process_line_result.cpp 匿名 namespace 里"照抄"过来的拷贝（prompt 二选
//   一里的"避免动 process_line_result"那条）。语义完全一致，所以在两个 TU
//   里并存不会有问题；如果以后 process_line_result.cpp 删掉，本文件仍能
//   独立工作。
// =============================================================================

#define PROCESS_LINE_RESULT_USE_EXTERNAL_VIEWPOINT
#include "Types.h"
using robot_planner::Matrix4d;
using robot_planner::JointVec6;
using robot_planner::ViewPoint;

#include "path_rrt_splice.h"

// PathPlan RRT 入口与 FK / 欧拉角换算
#include "PlanPathRRTconnect.h"
#include "kinematicsApi.h"
#include "math_utils.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

// -----------------------------------------------------------------------------
// 把一个 ViewPoint 包装成 PathPlan::PlanPathRRTconnect_new 需要的
// MeasurePointPoseSet。PlanPathRRTconnect_new 内部只读 configurations[0]
// 的关节字段和外层 x/y/z/i/j/k；slider/turnTable/euler_angle 在 RRT 链路里
// 都没人消费，本函数不装配。
//
// 与 process_line_result.cpp 中同名 helper 是拷贝关系，逻辑保持一致。
// -----------------------------------------------------------------------------
MeasurePointPoseSet buildMeasurePointPoseSetFromViewPoint(
    const ViewPoint& vp,
    const std::string& robotName)
{
    MeasurePointPoseSet mps;
    mps.name = "rrt_node";

    PoseConfiguration config;
    config.joints.j1 = vp.joints[0];
    config.joints.j2 = vp.joints[1];
    config.joints.j3 = vp.joints[2];
    config.joints.j4 = vp.joints[3];
    config.joints.j5 = vp.joints[4];
    config.joints.j6 = vp.joints[5];
    mps.configurations = { config };

    const std::vector<double> q(vp.joints.begin(), vp.joints.end());
    const auto fk = KinematicsApi::instance().fk(robotName, q, LinkName::TCP);

    if (fk.position.size() >= 3) {
        mps.x = fk.position[0];
        mps.y = fk.position[1];
        mps.z = fk.position[2];
    } else {
        std::cerr << "[spliceRRT] buildMeasurePoint: FK position 维度不足，xyz 置 0\n";
        mps.x = 0.0; mps.y = 0.0; mps.z = 0.0;
    }

    // wpr：从 FK 旋转矩阵提取 XYZ 欧拉角（度）
    bool hasFullUt = (fk.ut.size() >= 3)
        && (fk.ut[0].size() >= 3)
        && (fk.ut[1].size() >= 3)
        && (fk.ut[2].size() >= 3);
    if (hasFullUt) {
        std::vector<std::vector<double>> R(3, std::vector<double>(3));
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                R[r][c] = fk.ut[r][c];
        const auto eul = rotm2eul_XYZ(R);
        if (eul.size() >= 3) {
            mps.i = eul[0] * 180.0 / M_PI;
            mps.j = eul[1] * 180.0 / M_PI;
            mps.k = eul[2] * 180.0 / M_PI;
        } else {
            std::cerr << "[spliceRRT] buildMeasurePoint: rotm2eul_XYZ 维度不足，wpr 置 0\n";
            mps.i = 0.0; mps.j = 0.0; mps.k = 0.0;
        }
    } else {
        std::cerr << "[spliceRRT] buildMeasurePoint: FK ut 矩阵维度不足，wpr 置 0\n";
        mps.i = 0.0; mps.j = 0.0; mps.k = 0.0;
    }

    mps.i2 = 0.0; mps.j2 = 0.0; mps.k2 = 0.0;
    return mps;
}

// -----------------------------------------------------------------------------
// 调 PathPlan::PlanPathRRTconnect_new，把剪枝后的中间关节角填进 midJoints
// （不含起点终点）。成功返回 true。
//
// 与 process_line_result.cpp 中的 planRRTBetween 是拷贝关系，逻辑保持一致。
// -----------------------------------------------------------------------------
bool planRRTBetween(const ViewPoint& from,
                    const ViewPoint& to,
                    const RRTContext& ctx,
                    std::vector<JointVec6>& midJoints)
{
    midJoints.clear();
    if (!ctx.tech_params || !ctx.ik_fn || !ctx.fk_fn || !ctx.collision_fn) {
        std::cerr << "[spliceRRT] RRTContext 未完整配置，跳过 RRT\n";
        return false;
    }

    MeasurePointPoseSet fromPoint = buildMeasurePointPoseSetFromViewPoint(from, ctx.robot_name);
    MeasurePointPoseSet toPoint   = buildMeasurePointPoseSetFromViewPoint(to,   ctx.robot_name);

     ////=========================================================================
     ////调试测试案例：DEBUG_RRT_CASE=1 时，**忽略实际入参**，硬编码
     ////combinedEdgePath_debug.json line 299 (L_idx) → line 326 (R_idx) 这对
     ////出问题的 joints，独立喂给 PlanPathRRTconnect_new，dump 完整 BZD 与
     ////相邻 collision_fn(qPrev,qNext) 自检结果。本函数随后直接 return false，
     ////让外层 spliceRRTIntoDensePath 走"RRT 失败 → 整 run 降级 MOVL"分支，
     ////避免污染后续业务。
    
     ////环境变量未设时整段不执行（编译器会折叠常量分支），零开销。
     // for (int i = 0; i < 5; i++) {
     //   
     //       ViewPoint testFromVp;
     //       testFromVp.joints = { 5.3123, 36.5706, -63.7761, -47.7576, -138.5738, -155.9808 };
     //       testFromVp.joints = {  10.8530, 85.0382, -118.0571, -54.4575, -77.8420, -120.8395};
     //       ViewPoint testToVp;
     //       testToVp.joints = { 20.8111,  58.0415,   -98.4749,   -67.0807 , -121.8397, -113.2231 };

     //       MeasurePointPoseSet testFromPoint =
     //           buildMeasurePointPoseSetFromViewPoint(testFromVp, ctx.robot_name);
     //       MeasurePointPoseSet testToPoint =
     //           buildMeasurePointPoseSetFromViewPoint(testToVp, ctx.robot_name);

     //       PathPlan testPp;
     //       Result_BZD testRes = testPp.PlanPathRRTconnect_new(
     //           testFromPoint, testToPoint,
     //           *ctx.tech_params,
     //           ctx.ik_fn,
     //           ctx.collision_fn,
     //           ctx.fk_fn,
     //           ctx.robot_name);

     //        std::cerr << "[debugRRT] goal_ind=" << testRes.goal_ind
     //           << " BZD.rows=" << testRes.BZD.rows()
     //           << " BZD.cols=" << testRes.BZD.cols() << "\n";
     //       for (Eigen::Index r = 0; r < testRes.BZD.rows(); ++r) {
     //           std::cerr << "[debugRRT] row[" << r << "]:";
     //           for (Eigen::Index c = 0; c < testRes.BZD.cols(); ++c)
     //               std::cerr << " " << testRes.BZD(r, c);
     //           std::cerr << "\n";
     //       }
     //   
     //   }
    

    PathPlan pp;
    Result_BZD res = pp.PlanPathRRTconnect_new(
        fromPoint, toPoint,
        *ctx.tech_params,
        ctx.ik_fn,
        ctx.collision_fn,
        ctx.fk_fn,
        ctx.robot_name);

               std::cerr << "[debugRRT] goal_ind=" << res.goal_ind
              << " BZD.rows=" << res.BZD.rows()
              << " BZD.cols=" << res.BZD.cols() << "\n";
          for (Eigen::Index r = 0; r < res.BZD.rows(); ++r) {
              std::cerr << "[debugRRT] row[" << r << "]:";
              for (Eigen::Index c = 0; c < res.BZD.cols(); ++c)
                  std::cerr << " " << res.BZD(r, c);
              std::cerr << "\n";
          }

    if (res.goal_ind != 1) return false;
    if (res.BZD.rows() < 2 || res.BZD.cols() < 6) return false;

    // BZD 每行：j1..j6, x,y,z, i,j,k, i2,j2,k2
    // 去掉首尾两行（endpoints excluded），仅保留中间节点
    const Eigen::Index totalRows = res.BZD.rows();
    for (Eigen::Index r = 1; r + 1 < totalRows; ++r) {
        JointVec6 q{};
        for (int c = 0; c < 6; ++c) q[c] = res.BZD(r, c);
        midJoints.push_back(q);
    }
    return true;
}

// -----------------------------------------------------------------------------
// Pass 2 —— 把 dense 路径里所有 moveType=-2 的占位点删除，并把每个 -2 点
// 的左/右邻居（按删除前的原始索引、跳过其它 -2 后找到的第一个非 -2 点）
// 的 moveType 改为 -1，构成 spliceRRTIntoDensePath 主流程能识别的
// "连续 -1 run"。
//
// 设计要点：
//   · 先识别后处理。"边扫边改"会把刚被标 -1 的点又当作 -2 重新处理，
//     逻辑乱。这里用 isMinusTwo / toMarkMinusOne 两张 bool 表把"读"
//     和"写"完全隔开。
//   · 邻居搜索跳过所有 -2 索引——连续多个 -2（如 9.6+9.7 双失败）的
//     左邻居就是 9.5，右邻居就是 9.8，删除后 9.5/9.8 在 vector 里相邻，
//     构成 path_rrt_splice 识别的 -1 run。
//   · 段首/段尾 -2 找不到对应边的邻居 → 那侧不标 -1，由主流程的
//     孤儿 -1 检测打日志兜底，不在此处特殊处理。
//   · 段首失败导致左邻居是原始 waypoint（isImportant=true）→ 允许覆盖
//     其 moveType=-1。waypoint 的"必留"语义在 spliceRRT 处理 -1 run 时
//     会被无条件恢复（run 内所有点 isImportant 强制 true、moveType
//     重写为 1 或 0），不需要在这里做保护。
//   · 只动 moveType，不动 isImportant / isPathJoint。
// -----------------------------------------------------------------------------
std::vector<ViewPoint> markAndCleanMinusTwo(const std::vector<ViewPoint>& dense)
{
    const int n = static_cast<int>(dense.size());

    // 步骤1：识别所有 -2 索引，同时建立 isMinusTwo 快查表
    std::vector<bool> isMinusTwo(n, false);
    std::vector<int>  m2Indices;
    m2Indices.reserve(n);
    for (int i = 0; i < n; ++i) {
        if (dense[static_cast<std::size_t>(i)].moveType == -2) {
            isMinusTwo[static_cast<std::size_t>(i)] = true;
            m2Indices.push_back(i);
        }
    }
    if (m2Indices.empty()) return dense;   // 无 -2，原样返回

    // 步骤2：对每个 -2 找左/右第一个非 -2 索引，登记到 toMarkMinusOne
    std::vector<bool> toMarkMinusOne(n, false);
    for (int m2 : m2Indices) {
        // 左：从 m2-1 向左找第一个非 -2
        for (int L = m2 - 1; L >= 0; --L) {
            if (!isMinusTwo[static_cast<std::size_t>(L)]) {
                toMarkMinusOne[static_cast<std::size_t>(L)] = true;
                break;
            }
        }
        // 右：从 m2+1 向右找第一个非 -2
        for (int R = m2 + 1; R < n; ++R) {
            if (!isMinusTwo[static_cast<std::size_t>(R)]) {
                toMarkMinusOne[static_cast<std::size_t>(R)] = true;
                break;
            }
        }
    }

    // 步骤3：构造输出——跳过 -2，被 toMarkMinusOne 标记的点 moveType
    // 改为 -1（拷贝一份再改，避免污染入参；只动 moveType）
    std::vector<ViewPoint> out;
    out.reserve(static_cast<std::size_t>(n - static_cast<int>(m2Indices.size())));
    for (int i = 0; i < n; ++i) {
        if (isMinusTwo[static_cast<std::size_t>(i)]) continue;
        ViewPoint vp = dense[static_cast<std::size_t>(i)];
        if (toMarkMinusOne[static_cast<std::size_t>(i)]) {
            vp.moveType = -1;
        }
        out.push_back(vp);
    }

    std::cout << "[spliceRRT/Pass2] -2 占位点 " << m2Indices.size()
              << " 个已删除；邻居 -1 已就位\n";
    return out;
}

// -----------------------------------------------------------------------------
// 把 RRT 输出的 JointVec6 包装成一个新的 ViewPoint（带 FK 算出的 globalT）。
// 后续 MOVL 安全验证循环会读 globalT，所以这里必须填好。
// -----------------------------------------------------------------------------
ViewPoint makeRRTMidNode(const JointVec6& q,
                         double partRotateAngle,
                         const std::string& robotName)
{
    ViewPoint vp;
    vp.joints          = q;
    vp.moveType        = 0;                  // RRT 中间点 = MOVJ
    vp.isImportant     = true;               // 保活，过稀疏过滤
    vp.isPathJoint     = true;
    vp.partRotateAngle = partRotateAngle;

    const std::vector<double> qv(q.begin(), q.end());
    const auto fk = KinematicsApi::instance().fk(robotName, qv, LinkName::TCP);

    Matrix4d T = robot_planner::eye4();
    bool hasFullUt = (fk.ut.size() >= 4);
    if (hasFullUt) {
        for (int r = 0; r < 4 && hasFullUt; ++r) {
            if (fk.ut[r].size() < 4) { hasFullUt = false; break; }
            for (int c = 0; c < 4; ++c) T[r][c] = fk.ut[r][c];
        }
    }
    if (!hasFullUt) {
        std::cerr << "[spliceRRT] makeRRTMidNode: FK ut 维度不足，globalT 退化为单位矩阵\n";
        T = robot_planner::eye4();
    }
    vp.globalT = T;
    return vp;
}

}  // namespace

namespace robot_planner {

// =============================================================================
// spliceRRTIntoDensePath
// =============================================================================
std::vector<ViewPoint> spliceRRTIntoDensePath(
    const std::vector<ViewPoint>& dense,
    const RRTContext& rrtCtx)
{
    if (dense.empty()) return {};

    // -------------------------------------------------------------------------
    // 步骤0（Pass 2）：把 dense 里所有 mt=-2 占位点删掉，左右邻居 mt 改 -1。
    // 调用约定（隐性契约）：NaN joints 仅出现在 mt=-2 占位点；本步之后
    // cleaned 里不应再有任何 NaN，所有触及 joints 的代码路径（包括
    // planRRTBetween）都在 cleaned 上跑。
    // -------------------------------------------------------------------------
    std::vector<ViewPoint> cleaned = markAndCleanMinusTwo(dense);

    const int n = static_cast<int>(cleaned.size());
    if (n == 0) return {};

    // -------------------------------------------------------------------------
    // 步骤1：扫描所有 -1 run，记录每个 run 的 [L_idx, R_idx]
    //   · 长度 == 1 的孤儿 -1：单独记下来，下面稀疏化阶段当普通 mt=1 处理
    //     （isImportant=false → 被丢），仅打日志提醒。
    //   · 长度 >= 2 的 run：当一个整体处理，做一次 RRT。
    // -------------------------------------------------------------------------
    std::vector<std::pair<int, int>> validRuns;     // 长度 >= 2 的 run
    std::vector<int>                 orphanRuns;    // 长度 == 1 的 -1 索引
    {
        int i = 0;
        while (i < n) {
            if (cleaned[static_cast<std::size_t>(i)].moveType != -1) {
                ++i;
                continue;
            }
            int j = i;
            while (j + 1 < n && cleaned[static_cast<std::size_t>(j + 1)].moveType == -1) {
                ++j;
            }
            if (j == i) {
                orphanRuns.push_back(i);
                std::cout << "[spliceRRT] 跳过孤儿 -1 at idx=" << i
                          << "（前后无配对，本函数不处理；段首/段尾 corner case）\n";
            } else {
                validRuns.emplace_back(i, j);
            }
            i = j + 1;
        }
    }

    // -------------------------------------------------------------------------
    // 步骤2：对每个有效 run 调 RRT
    //   · 用一个可变副本 work，存储被改写的 moveType/isImportant
    //   · runMidNodes[run_index] = 该 run 应在 L_idx 之后插入的 RRT 中间节点
    //   · runDropMask[idx] = true 表示该索引（仅 run 内部除 L/R 之外的 -1）
    //     在最终输出前要从 work 里跳过
    // -------------------------------------------------------------------------
    std::vector<ViewPoint>                 work = cleaned;
    std::vector<std::vector<ViewPoint>>    runMidNodes(validRuns.size());
    std::vector<bool>                      runDropMask(work.size(), false);

    for (std::size_t ri = 0; ri < validRuns.size(); ++ri) {
        const int L_idx = validRuns[ri].first;
        const int R_idx = validRuns[ri].second;

        std::vector<JointVec6> midJoints;
        const bool ok = planRRTBetween(
            work[static_cast<std::size_t>(L_idx)],
            work[static_cast<std::size_t>(R_idx)],
            rrtCtx,
            midJoints);

        //if (ok && !midJoints.empty()) {
        if ( ok ) {
            // 成功：L 为 MOVL 接入点，R 为 MOVJ 衔接点
            work[static_cast<std::size_t>(L_idx)].moveType    = 1;
            work[static_cast<std::size_t>(L_idx)].isImportant = true;
            work[static_cast<std::size_t>(R_idx)].moveType    = 0;
            work[static_cast<std::size_t>(R_idx)].isImportant = true;

            // run 内部除 L/R 之外的 -1 点：标记丢弃
            for (int idx = L_idx + 1; idx <= R_idx - 1; ++idx) {
                runDropMask[static_cast<std::size_t>(idx)] = true;
            }

            // 构造中间节点（继承 L 的 partRotateAngle）
            const double segRotate =
                work[static_cast<std::size_t>(L_idx)].partRotateAngle;
            runMidNodes[ri].reserve(midJoints.size());
            for (const auto& q : midJoints) {
                runMidNodes[ri].push_back(
                    makeRRTMidNode(q, segRotate, rrtCtx.robot_name));
            }

            std::cout << "[spliceRRT] run [" << L_idx << ".." << R_idx
                      << "] RRT 成功，插入 " << midJoints.size()
                      << " 个中间点\n";
        } else {
            // 失败：run 内**所有** -1 点降级为 MOVL 关键点（保命）
            for (int idx = L_idx; idx <= R_idx; ++idx) {
                work[static_cast<std::size_t>(idx)].moveType    = 1;
                work[static_cast<std::size_t>(idx)].isImportant = true;
            }
            // runMidNodes[ri] 保持空
            std::cout << "[spliceRRT] run [" << L_idx << ".." << R_idx
                      << "] RRT 失败，降级为 MOVL（" << (R_idx - L_idx + 1)
                      << " 个点 isImportant=true, moveType=1）\n";
        }
    }

    // -------------------------------------------------------------------------
    // 步骤3：组装输出 + 稀疏化
    //   · 遍历 work；按 runDropMask 跳过 run 内部的 -1；
    //   · 单一过滤规则：只保留 isImportant=true 的点；
    //   · 在每个 run 的 L_idx 之后插入对应的 runMidNodes。
    // -------------------------------------------------------------------------
    // 建一个 idx → run_index 的索引（仅在 L_idx 处生效），方便插入顺序
    std::vector<int> insertAfter(work.size(), -1);
    for (std::size_t ri = 0; ri < validRuns.size(); ++ri) {
        insertAfter[static_cast<std::size_t>(validRuns[ri].first)] =
            static_cast<int>(ri);
    }

    std::vector<ViewPoint> out;
    out.reserve(work.size());
    for (int i = 0; i < n; ++i) {
        if (runDropMask[static_cast<std::size_t>(i)]) continue;

        const ViewPoint& vp = work[static_cast<std::size_t>(i)];
        // 过滤规则：isImportant || isPathJoint
        //   · isImportant=true：原始 waypoint、扰动成功点、RRT 端点/中间点
        //   · isPathJoint=true 且 isImportant=false：main.cpp 在转台切换时插入的
        //     MOVJ 过渡点（避免关节空间大跳）；必须保留
        //   · 两者皆 false：dense 可达插补点（mt=1）或 mt=-2 占位（Pass 2 漏网）
        if (vp.isImportant || vp.isPathJoint) {
            out.push_back(vp);
        }

        const int ri = insertAfter[static_cast<std::size_t>(i)];
        if (ri >= 0) {
            for (const auto& mid : runMidNodes[static_cast<std::size_t>(ri)]) {
                out.push_back(mid);
            }
        }
    }

    return out;
}

}  // namespace robot_planner

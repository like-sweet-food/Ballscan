#define PROCESS_LINE_RESULT_USE_EXTERNAL_VIEWPOINT
#include "Types.h"
using robot_planner::Matrix4d;
using robot_planner::JointVec6;
using robot_planner::ViewPoint;

#include "process_line_result.h"

// 直接使用 PathPlan 提供的两点之间 RRT 规划入口
#include "PlanPathRRTconnect.h"

// FK 与欧拉角提取（来自 m_include，已在 IncludePath 中）
#include "kinematicsApi.h"
#include "math_utils.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

bool hasNaNJoint(const JointVec6& q) {
    for (int i = 0; i < 6; ++i) {
        if (std::isnan(q[i])) {
            return true;
        }
    }
    return false;
}

bool isKeyNode(const ViewPoint& s) {
    return s.isImportant || s.isPathJoint;
}

int findLeftKeyStop(const std::vector<ViewPoint>& tail, int startIdx) {
    int idx = startIdx;
    while (idx > 0 && !isKeyNode(tail[static_cast<std::size_t>(idx)])) {
        --idx;
    }
    return idx;
}

int findRightKeyStop(const std::vector<ViewPoint>& tail, int startIdx) {
    int idx = startIdx;
    const int n = static_cast<int>(tail.size());
    while (idx < n - 1 && !isKeyNode(tail[static_cast<std::size_t>(idx)])) {
        ++idx;
    }
    return idx;
}

double getSegRotate(const std::vector<ViewPoint>& tail, int L, int R) {
    if (L >= 0 && L < static_cast<int>(tail.size())) {
        return tail[static_cast<std::size_t>(L)].partRotateAngle;
    }
    if (R >= 0 && R < static_cast<int>(tail.size())) {
        return tail[static_cast<std::size_t>(R)].partRotateAngle;
    }
    return 0.0;
}

PlanMoveJResult attemptPlanSafe(const std::vector<ViewPoint>& tail,
                                int L,
                                int R,
                                const ProcessLineCallbacks& cb,
                                const ProcessLineOptions& opt) {
    PlanMoveJResult out;
    if (!cb.planMoveJ) {
        return out;
    }
    const double segRotate = getSegRotate(tail, L, R);
    return cb.planMoveJ(tail[static_cast<std::size_t>(L)].joints,
                        tail[static_cast<std::size_t>(R)].joints,
                        segRotate,
                        opt.useSimpleMid);
}

ViewPoint buildMidNodeFromTemplate(const ViewPoint& tpl,
                                   const JointVec6& qDeg,
                                   double partRotateAngle,
                                   const ProcessLineCallbacks& cb) {
    ViewPoint s = tpl;
    s.joints = qDeg;
    s.moveType = 0;
    s.isImportant = true;
    s.partRotateAngle = partRotateAngle;

    if (cb.forwardKinematics) {
        s.globalT = cb.forwardKinematics(qDeg);
    }
    return s;
}

// 从 ViewPoint 构造 PathPlan 需要的 MeasurePointPoseSet。
// PlanPathRRTconnect_new 仅读取 configurations[0].joints.j1..j6 和外层 x/y/z/i/j/k/i2/j2/k2，
// slider/turnTable/euler_angle 在 RRT 链路（rrtExtend / jianzhi230628 / setPath）中全部无人消费，
// 因此此处不再装配这些字段。
MeasurePointPoseSet buildMeasurePointPoseSetFromViewPoint(const ViewPoint& vp,
                                                          const std::string& robotName) {
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
        std::cerr << "[buildMeasurePointPoseSet] FK position 维度不足，xyz 置 0" << std::endl;
        mps.x = 0.0;
        mps.y = 0.0;
        mps.z = 0.0;
    }

    // i/j/k = FK 计算出的 wpr（度）。提取 fk.ut 的 3×3 旋转矩阵，
    // 经 rotm2eul_XYZ 得到欧拉角（弧度），再换算为度。
    bool hasFullUt = (fk.ut.size() >= 3)
        && (fk.ut[0].size() >= 3)
        && (fk.ut[1].size() >= 3)
        && (fk.ut[2].size() >= 3);
    if (hasFullUt) {
        std::vector<std::vector<double>> R(3, std::vector<double>(3));
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R[r][c] = fk.ut[r][c];
            }
        }
        const auto eul = rotm2eul_XYZ(R);
        if (eul.size() >= 3) {
            mps.i = eul[0] * 180.0 / M_PI;
            mps.j = eul[1] * 180.0 / M_PI;
            mps.k = eul[2] * 180.0 / M_PI;
        } else {
            std::cerr << "[buildMeasurePointPoseSet] rotm2eul_XYZ 返回维度不足，wpr 置 0" << std::endl;
            mps.i = 0.0; mps.j = 0.0; mps.k = 0.0;
        }
    } else {
        std::cerr << "[buildMeasurePointPoseSet] FK ut 矩阵维度不足，wpr 置 0" << std::endl;
        mps.i = 0.0; mps.j = 0.0; mps.k = 0.0;
    }

    mps.i2 = 0.0;
    mps.j2 = 0.0;
    mps.k2 = 0.0;

    return mps;
}

// 调用 PathPlan::PlanPathRRTconnect_new 做两点之间路径规划。
// 成功返回 true，midJoints 填充剪枝后路径的中间关节角（不含起点终点）。
bool planRRTBetween(const ViewPoint& from,
                    const ViewPoint& to,
                    const RRTContext& ctx,
                    std::vector<JointVec6>& midJoints) {
    midJoints.clear();
    if (!ctx.tech_params || !ctx.ik_fn || !ctx.fk_fn || !ctx.collision_fn) {
        std::cerr << "[processLineResult] RRTContext 未完整配置，跳过 RRT 回退" << std::endl;
        return false;
    }

    MeasurePointPoseSet fromPoint = buildMeasurePointPoseSetFromViewPoint(from, ctx.robot_name);
    MeasurePointPoseSet toPoint   = buildMeasurePointPoseSetFromViewPoint(to,   ctx.robot_name);

    PathPlan pp;
    Result_BZD res = pp.PlanPathRRTconnect_new(
        fromPoint, toPoint,
        *ctx.tech_params,
        ctx.ik_fn,
        ctx.collision_fn,
        ctx.fk_fn,
        ctx.robot_name);

    if (res.goal_ind != 1) {
        return false;
    }
    if (res.BZD.rows() < 2 || res.BZD.cols() < 6) {
        return false;
    }

    // BZD 每行：j1..j6, x,y,z, i,j,k, i2,j2,k2
    // 去掉首尾两行（endpoints excluded），仅保留中间节点
    const Eigen::Index totalRows = res.BZD.rows();
    for (Eigen::Index r = 1; r + 1 < totalRows; ++r) {
        JointVec6 q{};
        for (int c = 0; c < 6; ++c) {
            q[c] = res.BZD(r, c);
        }
        midJoints.push_back(q);
    }
    return true;
}

}  // namespace

std::vector<ViewPoint> processLineResult(const std::vector<ViewPoint>& lineProcessResult,
                                         const ProcessLineCallbacks& callbacks,
                                         const ProcessLineOptions& options,
                                         const RRTContext* rrtCtx) {
    if (lineProcessResult.empty()) {
        return {};
    }

    std::vector<ViewPoint> src = lineProcessResult;
    for (auto& s : src) {
        if (isKeyNode(s) && hasNaNJoint(s.joints)) {
            s.isImportant = false;
            s.isPathJoint = false;
        }
    }

    if (src.size() == 1) {
        return src;
    }

    const ViewPoint head = src.front();
    std::vector<ViewPoint> tail(src.begin() + 1, src.end());

    const int n = static_cast<int>(tail.size());
    if (n == 0) {
        return src;
    }

    std::vector<int> idxEmpty;
    idxEmpty.reserve(tail.size());
    for (int i = 0; i < n; ++i) {
        if (tail[static_cast<std::size_t>(i)].moveType == options.emptyMoveTypeValue) {
            idxEmpty.push_back(i);
        }
    }

    std::vector<ViewPoint> tailNew;

    if (idxEmpty.empty()) {
        tailNew = tail;
    } else {
        std::vector<bool> keepMask(static_cast<std::size_t>(n), true);
        std::vector<std::pair<int, ViewPoint>> insertions;

        int k = 0;
        while (k < static_cast<int>(idxEmpty.size())) {
            int runStart = idxEmpty[static_cast<std::size_t>(k)];
            int runEnd = runStart;
            while (k + 1 < static_cast<int>(idxEmpty.size()) &&
                   idxEmpty[static_cast<std::size_t>(k + 1)] == idxEmpty[static_cast<std::size_t>(k)] + 1) {
                ++k;
                runEnd = idxEmpty[static_cast<std::size_t>(k)];
            }

            if (!keepMask[static_cast<std::size_t>(runStart)]) {
                ++k;
                continue;
            }

            int leftIdx = runStart - 1;
            int rightIdx = runEnd + 1;

            if (leftIdx < 0 || rightIdx >= n) {
                ++k;
                continue;
            }

            if (tail[static_cast<std::size_t>(leftIdx)].moveType == options.emptyMoveTypeValue ||
                tail[static_cast<std::size_t>(rightIdx)].moveType == options.emptyMoveTypeValue) {
                ++k;
                continue;
            }

            int useL = leftIdx;
            int useR = rightIdx;

            PlanMoveJResult planRes = attemptPlanSafe(tail, useL, useR, callbacks, options);
            bool isSafe = planRes.isSafe;

            int Lstop = useL;
            int Rstop = useR;

            if (!isSafe) {
                Lstop = findLeftKeyStop(tail, leftIdx);
                Rstop = findRightKeyStop(tail, rightIdx);
                const int maxStep = std::max(leftIdx - Lstop, Rstop - rightIdx);

                for (int step = 1; step <= maxStep; ++step) {
                    int Ltry = leftIdx - step;
                    int Rtry = rightIdx + step;
                    if (Ltry < Lstop) {
                        Ltry = Lstop;
                    }
                    if (Rtry > Rstop) {
                        Rtry = Rstop;
                    }

                    PlanMoveJResult tryRes = attemptPlanSafe(tail, Ltry, Rtry, callbacks, options);
                    if (tryRes.isSafe) {
                        useL = Ltry;
                        useR = Rtry;
                        planRes = tryRes;
                        isSafe = true;
                        break;
                    }

                    if (Ltry == Lstop && Rtry == Rstop) {
                        break;
                    }
                }
            }

            bool usedRRTFallback = false;
            std::vector<ViewPoint> rrtMidNodes;

            if (!isSafe && rrtCtx != nullptr) {
                // 对应 MATLAB processLineResult 第 261-313 行：
                // rrt_L_try 从 L_stop 开始逐步向左减 1，直到规划成功或越界
                // 直接调用 PathPlan::PlanPathRRTconnect_new
                const double segRotate = getSegRotate(tail, Lstop, Rstop);
                int rrtLtry = Lstop;
                bool rrtSuccess = false;

                while (!rrtSuccess && rrtLtry >= 0) {
                    std::vector<JointVec6> midJoints;
                    const bool ok = planRRTBetween(
                        tail[static_cast<std::size_t>(rrtLtry)],
                        tail[static_cast<std::size_t>(Rstop)],
                        *rrtCtx,
                        midJoints);

                    if (ok && !midJoints.empty()) {
                        useL = rrtLtry;
                        useR = Rstop;
                        usedRRTFallback = true;
                        isSafe = true;
                        rrtSuccess = true;

                        rrtMidNodes.reserve(midJoints.size());
                        for (const auto& q : midJoints) {
                            rrtMidNodes.push_back(
                                buildMidNodeFromTemplate(
                                    tail[static_cast<std::size_t>(useL)], q, segRotate, callbacks));
                        }
                    } else if (ok && midJoints.empty()) {
                        // BZD 存在但路径无中间点：对应 MATLAB "break"，不再向左扩展
                        break;
                    } else {
                        // BZD 判断不通过：对应 MATLAB "左端点减 1 重试"
                        --rrtLtry;
                    }
                }
            }

            if (isSafe) {
                tail[static_cast<std::size_t>(useL)].isImportant = true;
                tail[static_cast<std::size_t>(useL)].moveType = 1;
                tail[static_cast<std::size_t>(useR)].isImportant = true;
                tail[static_cast<std::size_t>(useR)].moveType = 0;

                if (usedRRTFallback) {
                    for (const auto& node : rrtMidNodes) {
                        insertions.emplace_back(useL, node);
                    }
                    if (useR - useL >= 2) {
                        for (int i = useL + 1; i <= useR - 1; ++i) {
                            keepMask[static_cast<std::size_t>(i)] = false;
                        }
                    }
                } else if (planRes.qMid.has_value()) {
                    const double segRotate = getSegRotate(tail, useL, useR);
                    ViewPoint midNode = buildMidNodeFromTemplate(
                        tail[static_cast<std::size_t>(useL)], planRes.qMid.value(), segRotate, callbacks);
                    insertions.emplace_back(useL, midNode);

                    if (useR - useL >= 2) {
                        for (int i = useL + 1; i <= useR - 1; ++i) {
                            keepMask[static_cast<std::size_t>(i)] = false;
                        }
                    }
                } else {
                    if (useR - useL >= 2) {
                        for (int i = useL + 1; i <= useR - 1; ++i) {
                            keepMask[static_cast<std::size_t>(i)] = false;
                        }
                    }
                }
            }

            ++k;
        }

        tailNew.reserve(tail.size() + insertions.size());
        for (int i = 0; i < n; ++i) {
            if (!keepMask[static_cast<std::size_t>(i)]) {
                continue;
            }

            tailNew.push_back(tail[static_cast<std::size_t>(i)]);

            for (const auto& ins : insertions) {
                if (ins.first == i) {
                    tailNew.push_back(ins.second);
                }
            }
        }
    }

    if (options.keepOnlyKeyNodesAtEnd) {
        std::vector<ViewPoint> keyOnly;
        keyOnly.reserve(tailNew.size());
        for (const auto& s : tailNew) {
            if (isKeyNode(s)) {
                keyOnly.push_back(s);
            }
        }
        tailNew.swap(keyOnly);
    }

    // 末尾 NaN joint 二次过滤（对应 MATLAB processLineResult 第 401-409 行）
    // 过滤掉关键节点中 joints 含 NaN 的点（防止上游不可达点漏网）
    {
        std::vector<ViewPoint> validOnly;
        validOnly.reserve(tailNew.size());
        for (const auto& s : tailNew) {
            if (hasNaNJoint(s.joints)) {
                std::cerr << "[processLineResult] 警告：移除无效关键点（joint 含 NaN）" << std::endl;
            } else {
                validOnly.push_back(s);
            }
        }
        tailNew.swap(validOnly);
    }

    std::vector<ViewPoint> out;
    out.reserve(1 + tailNew.size());
    out.push_back(head);
    out.insert(out.end(), tailNew.begin(), tailNew.end());
    return out;
}

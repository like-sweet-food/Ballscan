#define PROCESS_LINE_RESULT_USE_EXTERNAL_VIEWPOINT
#include "Types.h"
using robot_planner::Matrix4d;
using robot_planner::JointVec6;
using robot_planner::ViewPoint;

#include "process_line_result.h"

#include <cmath>
#include <vector>

namespace {

bool isPathSafeBySampling(const JointVec6& q1,
                          const JointVec6& q2,
                          double segRotate,
                          double sampleStep,
                          const ExistingPlannerHooks& hooks) {
    if (!hooks.calculatetimecpp || !hooks.mutinomial || !hooks.isJointStateSafe) {
        return false;
    }

    const double T = hooks.calculatetimecpp(q1, q2);
    if (!(T > 0.0)) {
        return false;
    }

    std::vector<double> times;
    for (double t = 0.0; t < T; t += sampleStep) {
        times.push_back(t);
    }
    if (times.empty() || std::abs(times.back() - T) > 1e-9) {
        times.push_back(T);
    }

    for (double t : times) {
        const JointVec6 q = hooks.mutinomial(q1, q2, t, T);
        if (!hooks.isJointStateSafe(q, segRotate)) {
            return false;
        }
    }
    return true;
}

std::optional<JointVec6> findMidpointFallback(const JointVec6& q1,
                                              const JointVec6& q2,
                                              double segRotate,
                                              double sampleStep,
                                              const ExistingPlannerHooks& hooks) {
    JointVec6 mid{};
    for (int i = 0; i < 6; ++i) {
        mid[i] = 0.5 * (q1[i] + q2[i]);
    }

    if (isPathSafeBySampling(q1, mid, segRotate, sampleStep, hooks) &&
        isPathSafeBySampling(mid, q2, segRotate, sampleStep, hooks)) {
        return mid;
    }
    return std::nullopt;
}

}  // namespace

ProcessLineCallbacks makeCallbacksWithExistingPlanner(const ExistingPlannerHooks& hooks,
                                                      double sampleStep) {
    ProcessLineCallbacks cb;

    cb.planMoveJ = [hooks, sampleStep](const JointVec6& q1,
                                       const JointVec6& q2,
                                       double segRotate,
                                       bool useSimpleMid) -> PlanMoveJResult {
        (void)useSimpleMid;
        PlanMoveJResult out;

        // 1) Direct path check: calculatetimecpp + mutinomial sampling.
        const bool directSafe = isPathSafeBySampling(q1, q2, segRotate, sampleStep, hooks);
        if (directSafe) {
            out.isSafe = true;
            out.qMid.reset();
            return out;
        }

        // 2) Midpoint search: prefer your existing midpoint routine.
        if (hooks.findReachableMidpoint) {
            const std::optional<JointVec6> mid = hooks.findReachableMidpoint(q1, q2, segRotate, useSimpleMid);
            if (mid.has_value()) {
                out.isSafe = true;
                out.qMid = mid;
                return out;
            }
        }

        // 3) Fallback midpoint strategy.
        const std::optional<JointVec6> midFallback =
            findMidpointFallback(q1, q2, segRotate, sampleStep, hooks);
        if (midFallback.has_value()) {
            out.isSafe = true;
            out.qMid = midFallback;
            return out;
        }

        out.isSafe = false;
        out.qMid.reset();
        return out;
    };

    cb.forwardKinematics = hooks.forwardKinematics;
    return cb;
}


#include "process_line_result_qtapp2_adapter.h"

#ifdef PROCESS_LINE_RESULT_ENABLE_QT_APP2_ADAPTER

#include <algorithm>
#include <memory>
#include <vector>

#include "PlanPathRRTconnect.h"
#include "kinematicsApi.h"
#include "parameters_type.h"
#include "trajectory.h"

namespace {

MeasurePointPoseSet buildPointFromJoint(const std::string& robotName,
                                        const JointVec6& qDeg,
                                        RobotFKFunction robotFKFunction) {
    MeasurePointPoseSet p{};
    PoseConfiguration cfgPose{};
    cfgPose.joints.j1 = qDeg[0];
    cfgPose.joints.j2 = qDeg[1];
    cfgPose.joints.j3 = qDeg[2];
    cfgPose.joints.j4 = qDeg[3];
    cfgPose.joints.j5 = qDeg[4];
    cfgPose.joints.j6 = qDeg[5];
    p.configurations = { cfgPose };

    if (robotFKFunction) {
        const std::vector<double> qv(qDeg.begin(), qDeg.end());
        FowardKinematicsReslutstd fk = robotFKFunction(robotName, qv, LinkName::TCP);

        if (fk.position.size() >= 3) {
            p.x = fk.position[0];
            p.y = fk.position[1];
            p.z = fk.position[2];
        }

        if (fk.ut.size() >= 3 && fk.ut[0].size() >= 3 && fk.ut[1].size() >= 3 && fk.ut[2].size() >= 3) {
            // Approximate vector fields from FK rotation matrix.
            p.i = fk.ut[0][2];
            p.j = fk.ut[1][2];
            p.k = fk.ut[2][2];
            p.i2 = fk.ut[0][0];
            p.j2 = fk.ut[1][0];
            p.k2 = fk.ut[2][0];
        }
    }

    return p;
}

}  // namespace

ProcessLineCallbacks makeQtApp2Callbacks(const std::string& robotName,
                                         const RobotTechParameters& techParams,
                                         CollisionTestFunction collisionTest,
                                         RobotFKFunction robotFKFunction,
                                         RobotIKFunction robotInverseSolutionFn,
                                         double sampleStep) {
    ExistingPlannerHooks hooks;

    hooks.calculatetimecpp = [](const JointVec6& q1, const JointVec6& q2) -> double {
        return calculateTime(q1, q2, 1, 1);
    };

    hooks.mutinomial = [](const JointVec6& q1,
                          const JointVec6& q2,
                          double t,
                          double T) -> JointVec6 {
        return multinomial(q1, q2, t, T);
    };

    hooks.isJointStateSafe = [robotName, collisionTest](const JointVec6& qDeg, double /*partRotate*/) -> bool {
        if (!collisionTest) {
            return false;
        }
        const std::vector<double> q(qDeg.begin(), qDeg.end());
        const bool hasCollision = collisionTest(robotName, q, q);
        return !hasCollision;
    };

    hooks.findReachableMidpoint = nullptr;

    hooks.rrtPlanner = [robotName, techParams, collisionTest, robotFKFunction, robotInverseSolutionFn]
        (const JointVec6& qL, const JointVec6& qR, double /*segRotate*/) -> RRTFallbackResult {
        RRTFallbackResult out;
        if (!collisionTest || !robotFKFunction || !robotInverseSolutionFn) {
            out.success = false;
            return out;
        }

        MeasurePointPoseSet fromPoint = buildPointFromJoint(robotName, qL, robotFKFunction);
        MeasurePointPoseSet toPoint = buildPointFromJoint(robotName, qR, robotFKFunction);

        PathPlan planner;
        Result_BZD bz = planner.PlanPathRRTconnect_new(
            fromPoint,
            toPoint,
            techParams,
            robotInverseSolutionFn,
            collisionTest,
            robotFKFunction,
            robotName
        );

        if (bz.goal_ind != 1 || bz.BZD.rows() <= 2 || bz.BZD.cols() < 6) {
            out.success = false;
            return out;
        }

        out.success = true;
        for (int r = 1; r < bz.BZD.rows() - 1; ++r) {
            JointVec6 q{};
            for (int c = 0; c < 6; ++c) {
                q[c] = bz.BZD(r, c);
            }
            out.midJoints.push_back(q);
        }
        return out;
    };

    hooks.forwardKinematics = nullptr;

    return makeCallbacksWithExistingPlanner(hooks, sampleStep);
}

#endif

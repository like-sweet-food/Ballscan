#pragma once

#include <array>
#include <vector>

using Joint6 = std::array<double, 6>;
using Trajectory = std::vector<Joint6>;

struct TrajectoryResult {
    double T;
    std::vector<double> time_list;
    Trajectory trajectory;
};

double calculateTime(const Joint6& start,
    const Joint6& goal,
    int L,
    int motionSign);

Joint6 multinomial(const Joint6& start,
    const Joint6& goal,
    double t,
    double T);

TrajectoryResult generateJointTrajectory(const Joint6& start,
    const Joint6& goal,
    int L,
    int motionSign,
    double step);
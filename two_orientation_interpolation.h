#pragma once

#include <vector>

#include "RobotConfig.h"
#include "Types.h"

namespace robot_planner {

std::vector<Matrix4d> twoOrientationInterpolation(
    const Matrix4d& Ts,
    const Matrix4d& Tg,
    const RobotConfig& config);


} // namespace robot_planner

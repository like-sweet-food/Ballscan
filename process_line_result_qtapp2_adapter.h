#pragma once

#include "process_line_result.h"
#include <string>
#include <vector>

// This adapter is for:
// D:\celiang\test\QtWidgetsApplication2\QtWidgetsApplication2
//
// It binds:
// - MoveJ sampling: calculateTime + multinomial
// - RRT fallback : robot_planner::BiRRTPlanner::plan
//
// To enable, compile this file together with the Qt project and define:
// PROCESS_LINE_RESULT_ENABLE_QT_APP2_ADAPTER

#ifdef PROCESS_LINE_RESULT_ENABLE_QT_APP2_ADAPTER
struct RobotTechParameters;
struct FowardKinematicsReslutstd;
enum class LinkName;
using CollisionTestFunction = bool (*)(const std::string&, const std::vector<double>&, const std::vector<double>&);
using RobotFKFunction = FowardKinematicsReslutstd(*)(std::string, std::vector<double>, LinkName);
using RobotIKFunction = std::vector<std::vector<double>>(*)(std::string, const struct XYZWPR&);

ProcessLineCallbacks makeQtApp2Callbacks(const std::string& robotName,
                                         const RobotTechParameters& techParams,
                                         CollisionTestFunction collisionTest,
                                         RobotFKFunction robotFKFunction,
                                         RobotIKFunction robotInverseSolutionFn,
                                         double sampleStep = 0.05);
#endif

#pragma once
// =============================================================================
// SurfacePointGen.h  —  面点生成独立接口
//
// 输入：前端提供的零件点云（已变换）
// 输出：面扫覆盖测量点
// =============================================================================

#include <vector>
#include "parameters_type.h"

std::vector<MeasurePoint> generate_surface_points(
    const std::vector<PointWithNormal>& point_cloud);

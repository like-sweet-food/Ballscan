#pragma once
// =============================================================================
// SurfacePointGenerator.h  —  面扫覆盖点生成工具
//
// 从 RobotReachability 中提取 PCA / 局部坐标系 / 滑动窗口栅格逻辑，
// 独立为工具类，供 main.cpp 在循环前生成面点并插入
// toturnTableAssignMeasurePoint。
// =============================================================================

#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "parameters_type.h"

// =============================================================================
// 基础类型（与 Types.h 中定义兼容，此处独立定义避免引入 robot_planner 命名空间）
// =============================================================================

namespace surface_gen {

using Vec3d    = std::array<double, 3>;
using Matrix4d = std::vector<std::vector<double>>;

inline Matrix4d eye4()
{
    return {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
}

// 滑动窗口栅格单元
struct GridCell {
    Vec3d  centerPos    = {};
    Vec3d  centerNormal = {};
    Vec3d  xAxis        = {};
    bool   isImportant  = false;
};

// =============================================================================
// 数学工具（内联）
// =============================================================================

inline Vec3d normalize3(const Vec3d& v)
{
    double n = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (n < 1e-10) return {0, 0, 1};
    return {v[0]/n, v[1]/n, v[2]/n};
}

inline double dot3(const Vec3d& a, const Vec3d& b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

inline Vec3d cross3(const Vec3d& a, const Vec3d& b)
{
    return {
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}

inline Matrix4d invertTransform(const Matrix4d& T)
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

// =============================================================================
// PCA 主法向量（Jacobi 特征分解，无 Eigen 依赖）
// =============================================================================

inline Vec3d computeMainNormal(const std::vector<PointWithNormal>& points)
{
    if (points.empty()) return {0, 0, 1};

    const size_t N = points.size();

    double cx = 0.0, cy = 0.0, cz = 0.0;
    for (const auto& p : points) {
        cx += p.X; cy += p.Y; cz += p.Z;
    }
    cx /= N; cy /= N; cz /= N;

    double c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
    for (const auto& p : points) {
        double dx = p.X - cx, dy = p.Y - cy, dz = p.Z - cz;
        c00 += dx * dx; c01 += dx * dy; c02 += dx * dz;
        c11 += dy * dy; c12 += dy * dz; c22 += dz * dz;
    }
    const double inv = 1.0 / static_cast<double>(N - 1);
    c00 *= inv; c01 *= inv; c02 *= inv;
    c11 *= inv; c12 *= inv; c22 *= inv;

    double A[3][3] = {{c00, c01, c02}, {c01, c11, c12}, {c02, c12, c22}};
    double V[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};

    for (int iter = 0; iter < 100; ++iter) {
        int p = 0, q = 1;
        double maxOff = std::abs(A[0][1]);
        if (std::abs(A[0][2]) > maxOff) { p = 0; q = 2; maxOff = std::abs(A[0][2]); }
        if (std::abs(A[1][2]) > maxOff) { p = 1; q = 2; maxOff = std::abs(A[1][2]); }
        if (maxOff < 1e-15) break;

        double tau = (A[q][q] - A[p][p]) / (2.0 * A[p][q]);
        double t = (tau >= 0 ? 1.0 : -1.0) / (std::abs(tau) + std::sqrt(1.0 + tau * tau));
        double cosT = 1.0 / std::sqrt(1.0 + t * t);
        double sinT = t * cosT;

        double app = A[p][p], aqq = A[q][q], apq = A[p][q];
        A[p][p] = app - t * apq;
        A[q][q] = aqq + t * apq;
        A[p][q] = A[q][p] = 0.0;

        for (int r = 0; r < 3; ++r) {
            if (r == p || r == q) continue;
            double arp = A[r][p], arq = A[r][q];
            A[r][p] = A[p][r] = cosT * arp - sinT * arq;
            A[r][q] = A[q][r] = sinT * arp + cosT * arq;
        }
        for (int r = 0; r < 3; ++r) {
            double vrp = V[r][p], vrq = V[r][q];
            V[r][p] = cosT * vrp - sinT * vrq;
            V[r][q] = sinT * vrp + cosT * vrq;
        }
    }

    int minIdx = 0;
    if (A[1][1] < A[minIdx][minIdx]) minIdx = 1;
    if (A[2][2] < A[minIdx][minIdx]) minIdx = 2;

    return {V[0][minIdx], V[1][minIdx], V[2][minIdx]};
}

// =============================================================================
// 局部坐标系变换
// =============================================================================

inline std::vector<Vec3d> transformToLocalFrame(
    const std::vector<PointWithNormal>& points,
    const Vec3d&      mainNormal,
    Matrix4d&         outLocal2Global)
{
    outLocal2Global = eye4();

    Vec3d center = {0, 0, 0};
    for (const auto& p : points) {
        center[0] += p.X; center[1] += p.Y; center[2] += p.Z;
    }
    if (!points.empty()) {
        center[0] /= points.size();
        center[1] /= points.size();
        center[2] /= points.size();
    }

    Vec3d zAxis = normalize3(mainNormal);
    Vec3d tmp   = {1, 0, 0};
    if (std::abs(dot3(zAxis, tmp)) > 0.9)
        tmp = {0, 1, 0};
    Vec3d xAxis = normalize3(cross3(tmp, zAxis));
    Vec3d yAxis = cross3(zAxis, xAxis);

    outLocal2Global = {
        {xAxis[0], yAxis[0], zAxis[0], center[0]},
        {xAxis[1], yAxis[1], zAxis[1], center[1]},
        {xAxis[2], yAxis[2], zAxis[2], center[2]},
        {0,        0,        0,        1         }
    };

    Matrix4d global2Local = invertTransform(outLocal2Global);

    std::vector<Vec3d> localPts;
    localPts.reserve(points.size());
    for (const auto& p : points) {
        Vec3d lp = {
            global2Local[0][0]*p.X + global2Local[0][1]*p.Y + global2Local[0][2]*p.Z + global2Local[0][3],
            global2Local[1][0]*p.X + global2Local[1][1]*p.Y + global2Local[1][2]*p.Z + global2Local[1][3],
            global2Local[2][0]*p.X + global2Local[2][1]*p.Y + global2Local[2][2]*p.Z + global2Local[2][3]
        };
        localPts.push_back(lp);
    }
    return localPts;
}

// =============================================================================
// 滑动窗口栅格生成
// =============================================================================

inline std::vector<GridCell> buildSlidingWindowGrid(
    const std::vector<Vec3d>& localPoints,
    const Vec3d&      minCoords,
    const Vec3d&      maxCoords,
    double wx, double wy,
    double sx, double sy)
{
    std::vector<GridCell> cells;

    double z_min = minCoords[2], z_max = maxCoords[2];

    std::vector<double> x_range;
    for (double x = minCoords[0]; x <= maxCoords[0] - wx; x += sx)
        x_range.push_back(x);

    for (size_t xi = 0; xi < x_range.size(); ++xi) {
        double x = x_range[xi];

        std::vector<double> y_range;
        for (double y = minCoords[1]; y <= maxCoords[1] - wy; y += sy)
            y_range.push_back(y);
        if (xi % 2 == 1)
            std::reverse(y_range.begin(), y_range.end());

        for (double y : y_range) {
            double x_min = x, x_max = x + wx;
            double y_min = y, y_max = y + wy;

            bool hasPoints = false;
            for (const auto& p : localPoints) {
                if (p[0] >= x_min && p[0] <= x_max &&
                    p[1] >= y_min && p[1] <= y_max) {
                    hasPoints = true;
                    break;
                }
            }
            if (!hasPoints) continue;

            GridCell cell;
            cell.centerPos    = {(x_min + x_max) / 2.0,
                                  (y_min + y_max) / 2.0,
                                  (z_min + z_max) / 2.0};
            cell.centerNormal = {0.0, 0.0, 1.0};
            cell.isImportant  = false;

            cells.push_back(cell);
        }
    }

    return cells;
}

// =============================================================================
// generateSurfaceMeasurePoints  —  主入口
//
// 输入：已变换的零件点云 (PointWithNormal)
// 输出：面扫覆盖点，以 MeasurePoint 格式返回
//
// 参数说明：
//   safeHeight  — 面扫安全抬起高度（mm），对应 RobotConfig::safeHeight
//   windowSize  — 滑动窗口大小 [wx, wy]
//   stepSize    — 滑动步长 [sx, sy]
// =============================================================================

inline std::vector<MeasurePoint> generateSurfaceMeasurePoints(
    const std::vector<PointWithNormal>& pointCloud,
    double safeHeight,
    double wx, double wy,
    double sx, double sy)
{
    std::vector<MeasurePoint> result;

    if (pointCloud.empty()) return result;

    // 1. PCA 求主法向
    Vec3d mainNormal = computeMainNormal(pointCloud);
    std::cout << "[SurfacePointGenerator] 主法向量：["
              << mainNormal[0] << ", "
              << mainNormal[1] << ", "
              << mainNormal[2] << "]" << std::endl;

    // 2. 变换到局部坐标系
    Matrix4d local2Global;
    std::vector<Vec3d> localPoints = transformToLocalFrame(
        pointCloud, mainNormal, local2Global);

    // 3. 求 AABB
    Vec3d minCoords = localPoints[0], maxCoords = localPoints[0];
    for (const auto& p : localPoints) {
        for (int i = 0; i < 3; ++i) {
            minCoords[i] = std::min(minCoords[i], p[i]);
            maxCoords[i] = std::max(maxCoords[i], p[i]);
        }
    }

    // 4. 滑动窗口生成栅格
    std::vector<GridCell> gridCells = buildSlidingWindowGrid(
        localPoints, minCoords, maxCoords, wx, wy, sx, sy);

    std::cout << "[SurfacePointGenerator] 滑动窗口生成栅格数：" << gridCells.size() << std::endl;

    // 5. 法向量取反 + xAxis 计算（与原 planSurfaceViews 一致）
    for (size_t i = 0; i < gridCells.size(); ++i) {
        gridCells[i].centerNormal = {
            -gridCells[i].centerNormal[0],
            -gridCells[i].centerNormal[1],
            -gridCells[i].centerNormal[2]
        };

        if (gridCells.size() <= 1) {
            gridCells[i].xAxis = {1.0, 0.0, 0.0};
        } else if (i == 0 || i == gridCells.size() - 1) {
            size_t ref = (i == 0) ? 1 : gridCells.size() - 2;
            Vec3d diff = {
                gridCells[ref].centerPos[0] - gridCells[i].centerPos[0],
                gridCells[ref].centerPos[1] - gridCells[i].centerPos[1],
                gridCells[ref].centerPos[2] - gridCells[i].centerPos[2]
            };
            gridCells[i].xAxis = normalize3(diff);
        } else {
            Vec3d diff = {
                gridCells[i+1].centerPos[0] - gridCells[i].centerPos[0],
                gridCells[i+1].centerPos[1] - gridCells[i].centerPos[1],
                gridCells[i+1].centerPos[2] - gridCells[i].centerPos[2]
            };
            gridCells[i].xAxis = normalize3(diff);
        }
    }

    // 6. 栅格中心从局部坐标系变换回全局坐标系，生成 MeasurePoint
    for (size_t idx = 0; idx < gridCells.size(); ++idx) {
        const GridCell& cell = gridCells[idx];

        // 局部坐标系下的 TCP 位置（中心 + safeHeight × 法向量）
        Vec3d posLocal = {
            cell.centerPos[0] + safeHeight * (-cell.centerNormal[0]),
            cell.centerPos[1] + safeHeight * (-cell.centerNormal[1]),
            cell.centerPos[2] + safeHeight * (-cell.centerNormal[2])
        };

        // 变换回全局坐标系
        double gx = local2Global[0][0]*posLocal[0] + local2Global[0][1]*posLocal[1]
                   + local2Global[0][2]*posLocal[2] + local2Global[0][3];
        double gy = local2Global[1][0]*posLocal[0] + local2Global[1][1]*posLocal[1]
                   + local2Global[1][2]*posLocal[2] + local2Global[1][3];
        double gz = local2Global[2][0]*posLocal[0] + local2Global[2][1]*posLocal[1]
                   + local2Global[2][2]*posLocal[2] + local2Global[2][3];

        // 法向量变换回全局（只旋转，不平移）
        Vec3d nLocal = cell.centerNormal;
        double gni = local2Global[0][0]*nLocal[0] + local2Global[0][1]*nLocal[1] + local2Global[0][2]*nLocal[2];
        double gnj = local2Global[1][0]*nLocal[0] + local2Global[1][1]*nLocal[1] + local2Global[1][2]*nLocal[2];
        double gnk = local2Global[2][0]*nLocal[0] + local2Global[2][1]*nLocal[1] + local2Global[2][2]*nLocal[2];

        // xAxis 变换回全局
        Vec3d xLocal = cell.xAxis;
        double gxi = local2Global[0][0]*xLocal[0] + local2Global[0][1]*xLocal[1] + local2Global[0][2]*xLocal[2];
        double gxj = local2Global[1][0]*xLocal[0] + local2Global[1][1]*xLocal[1] + local2Global[1][2]*xLocal[2];
        double gxk = local2Global[2][0]*xLocal[0] + local2Global[2][1]*xLocal[1] + local2Global[2][2]*xLocal[2];

        MeasurePoint mp;
        mp.point_number = static_cast<int>(idx + 1);
        mp.name = "SurfacePoint_" + std::to_string(idx + 1);
        mp.vct_vectro_type = "Surface";
        mp.x  = gx;  mp.y  = gy;  mp.z  = gz;
        mp.i  = gni; mp.j  = gnj; mp.k  = gnk;
        mp.i2 = gxi; mp.j2 = gxj; mp.k2 = gxk;

        result.push_back(mp);
    }

    std::cout << "[SurfacePointGenerator] 生成面点数：" << result.size() << std::endl;
    return result;
}

} // namespace surface_gen

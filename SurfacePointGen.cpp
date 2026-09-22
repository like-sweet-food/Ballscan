// =============================================================================
// SurfacePointGen.cpp  —  面点生成实现
//
// 从前端提供的零件点云生成面扫覆盖测量点。
// 流程：PCA 主法向 → 局部坐标系 → 滑动窗口栅格 → 变换回全局 → MeasurePoint
// =============================================================================

#include "SurfacePointGen.h"
#include <cmath>
#include <array>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>

// =============================================================================
// 内部类型
// =============================================================================
using Vec3d    = std::array<double, 3>;
using Matrix4d = std::vector<std::vector<double>>;

// 内部视点结构体，对应 MATLAB sub_clouds{i}
struct BallScanMeasurePoint {
    Vec3d  center_pos    = {};
    Vec3d  center_normal = {};   // 局部坐标系下法向（取反前）
    bool   is_important  = false;
};

// =============================================================================
// 数学工具
// =============================================================================
static Matrix4d make_eye4()
{
    return {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
}

static Vec3d normalize3(const Vec3d& v)
{
    double n = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (n < 1e-10) return {0, 0, 1};
    return {v[0]/n, v[1]/n, v[2]/n};
}

static double dot3(const Vec3d& a, const Vec3d& b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static Vec3d cross3(const Vec3d& a, const Vec3d& b)
{
    return {
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}

static Matrix4d invert_transform(const Matrix4d& t)
{
    Matrix4d inv = make_eye4();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            inv[i][j] = t[j][i];
    for (int i = 0; i < 3; ++i) {
        inv[i][3] = 0.0;
        for (int j = 0; j < 3; ++j)
            inv[i][3] -= inv[i][j] * t[j][3];
    }
    return inv;
}

// =============================================================================
// PCA 主法向量（Jacobi 特征分解）
// =============================================================================
static Vec3d compute_main_normal(const std::vector<PointWithNormal>& points)
{
    if (points.empty()) return {0, 0, 1};

    const size_t n = points.size();

    double cx = 0.0, cy = 0.0, cz = 0.0;
    for (const auto& pt : points) {
        cx += pt.X; cy += pt.Y; cz += pt.Z;
    }
    cx /= n; cy /= n; cz /= n;

    double c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
    for (const auto& pt : points) {
        double dx = pt.X - cx, dy = pt.Y - cy, dz = pt.Z - cz;
        c00 += dx * dx; c01 += dx * dy; c02 += dx * dz;
        c11 += dy * dy; c12 += dy * dz; c22 += dz * dz;
    }
    const double inv = 1.0 / static_cast<double>(n - 1);
    c00 *= inv; c01 *= inv; c02 *= inv;
    c11 *= inv; c12 *= inv; c22 *= inv;

    double a[3][3] = {{c00, c01, c02}, {c01, c11, c12}, {c02, c12, c22}};
    double v[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};

    for (int iter = 0; iter < 100; ++iter) {
        int p = 0, q = 1;
        double max_off = std::abs(a[0][1]);
        if (std::abs(a[0][2]) > max_off) { p = 0; q = 2; max_off = std::abs(a[0][2]); }
        if (std::abs(a[1][2]) > max_off) { p = 1; q = 2; max_off = std::abs(a[1][2]); }
        if (max_off < 1e-15) break;

        double tau = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        double t = (tau >= 0 ? 1.0 : -1.0) / (std::abs(tau) + std::sqrt(1.0 + tau * tau));
        double cos_t = 1.0 / std::sqrt(1.0 + t * t);
        double sin_t = t * cos_t;

        double app = a[p][p], aqq = a[q][q], apq = a[p][q];
        a[p][p] = app - t * apq;
        a[q][q] = aqq + t * apq;
        a[p][q] = a[q][p] = 0.0;

        for (int r = 0; r < 3; ++r) {
            if (r == p || r == q) continue;
            double arp = a[r][p], arq = a[r][q];
            a[r][p] = a[p][r] = cos_t * arp - sin_t * arq;
            a[r][q] = a[q][r] = sin_t * arp + cos_t * arq;
        }
        for (int r = 0; r < 3; ++r) {
            double vrp = v[r][p], vrq = v[r][q];
            v[r][p] = cos_t * vrp - sin_t * vrq;
            v[r][q] = sin_t * vrp + cos_t * vrq;
        }
    }

    int min_idx = 0;
    if (a[1][1] < a[min_idx][min_idx]) min_idx = 1;
    if (a[2][2] < a[min_idx][min_idx]) min_idx = 2;

    return {v[0][min_idx], v[1][min_idx], v[2][min_idx]};
}

// =============================================================================
// 局部坐标系变换
// =============================================================================
static std::vector<Vec3d> transform_to_local_frame(
    const std::vector<PointWithNormal>& points,
    const Vec3d&      main_normal,
    Matrix4d&         out_local2global)
{
    out_local2global = make_eye4();

    Vec3d center = {0, 0, 0};
    for (const auto& pt : points) {
        center[0] += pt.X; center[1] += pt.Y; center[2] += pt.Z;
    }
    if (!points.empty()) {
        center[0] /= points.size();
        center[1] /= points.size();
        center[2] /= points.size();
    }

    //Vec3d z_axis = normalize3(main_normal);
    //Vec3d tmp    = {1, 0, 0};
    //if (std::abs(dot3(z_axis, tmp)) > 0.9)
    //    tmp = {0, 1, 0};
    //Vec3d x_axis = normalize3(cross3(tmp, z_axis));
    //Vec3d y_axis = cross3(z_axis, x_axis);

    //out_local2global = {
    //    {x_axis[0], y_axis[0], z_axis[0], center[0]},
    //    {x_axis[1], y_axis[1], z_axis[1], center[1]},
    //    {x_axis[2], y_axis[2], z_axis[2], center[2]},
    //    {0,         0,         0,         1         }
    //};

    //Matrix4d global2local = invert_transform(out_local2global);
    Vec3d main_axis = normalize3(main_normal);
    Vec3d target_z = { 0, 0, 1 };
    Vec3d v = cross3(main_axis, target_z);
    double s = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    double c = dot3(main_axis, target_z);

    double R[3][3];
    if (s < 1e-6) {
        R[0][0] = 1; R[0][1] = 0; R[0][2] = 0;
        R[1][0] = 0; R[1][1] = 1; R[1][2] = 0;
        R[2][0] = 0; R[2][1] = 0; R[2][2] = 1;
    }
    else {
        double vx[3][3] = {
            {    0, -v[2],  v[1]},
            { v[2],     0, -v[0]},
            {-v[1],  v[0],     0}
        };
        double vx2[3][3] = {};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    vx2[i][j] += vx[i][k] * vx[k][j];

        double coeff = (1.0 - c) / (s * s);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R[i][j] = (i == j ? 1.0 : 0.0) + vx[i][j] + vx2[i][j] * coeff;
    }

    // R 是 global→local，R^T 是 local→global
    out_local2global = {
        {R[0][0], R[1][0], R[2][0], center[0]},
        {R[0][1], R[1][1], R[2][1], center[1]},
        {R[0][2], R[1][2], R[2][2], center[2]},
        {0,       0,       0,       1         }
    };

    Matrix4d global2local = invert_transform(out_local2global);
    std::vector<Vec3d> local_pts;
    local_pts.reserve(points.size());
    for (const auto& pt : points) {
        Vec3d lp = {
            global2local[0][0]*pt.X + global2local[0][1]*pt.Y + global2local[0][2]*pt.Z + global2local[0][3],
            global2local[1][0]*pt.X + global2local[1][1]*pt.Y + global2local[1][2]*pt.Z + global2local[1][3],
            global2local[2][0]*pt.X + global2local[2][1]*pt.Y + global2local[2][2]*pt.Z + global2local[2][3]
        };
        local_pts.push_back(lp);
    }
    return local_pts;
}

// =============================================================================
// 滑动窗口栅格生成，同时标记每行首尾为关键点（对应 MATLAB isImportant）
// =============================================================================
static std::vector<BallScanMeasurePoint> build_sliding_window_grid(
    const std::vector<Vec3d>& local_points,
    const Vec3d&      min_coords,
    const Vec3d&      max_coords,
    double wx, double wy,
    double sx, double sy)
{
    std::vector<BallScanMeasurePoint> cells;

    double z_min = min_coords[2], z_max = max_coords[2];

    std::vector<double> x_range;
    for (double x = min_coords[0]; x <= max_coords[0] - wx; x += sx)
        x_range.push_back(x);

    for (size_t xi = 0; xi < x_range.size(); ++xi) {
        double x = x_range[xi];

        std::vector<double> y_range;
        for (double y = min_coords[1]; y <= max_coords[1] - wy; y += sy)
            y_range.push_back(y);
        if (xi % 2 == 1)
            std::reverse(y_range.begin(), y_range.end());

        // 记录本行在 cells 中的起始索引，用于关键点标记
        size_t row_start = cells.size();

        for (double y : y_range) {
            double x_lo = x,     x_hi = x + wx;
            double y_lo = y,     y_hi = y + wy;

            bool has_points = false;
            for (const auto& p : local_points) {
                if (p[0] >= x_lo && p[0] <= x_hi &&
                    p[1] >= y_lo && p[1] <= y_hi) {
                    has_points = true;
                    break;
                }
            }
            if (!has_points) continue;

            BallScanMeasurePoint cell;
            cell.center_pos    = {(x_lo + x_hi) / 2.0,
                                  (y_lo + y_hi) / 2.0,
                                  (z_min + z_max) / 2.0};
            cell.center_normal = {0.0, 0.0, 1.0};
            cell.is_important  = false;
            cells.push_back(cell);
        }

        // 标记本行首尾为关键点（对应 MATLAB gridCenterInfo 关键点标记）
        size_t row_end = cells.size();
        if (row_end > row_start) {
            cells[row_start].is_important        = true;
            cells[row_end - 1].is_important      = true;
        }
    }

    return cells;
}

// =============================================================================
// 内部视点 → MeasurePoint（局部坐标系 → 全局坐标系）
// =============================================================================
static MeasurePoint to_measure_point(
    const BallScanMeasurePoint& cell,
    const Matrix4d& local2global,
    double safe_height,
    int index)
{
    // 位置加安全高度偏移（沿法向方向抬起）
    Vec3d pos_local = {
        cell.center_pos[0] + safe_height * cell.center_normal[0],
        cell.center_pos[1] + safe_height * cell.center_normal[1],
        cell.center_pos[2] + safe_height * cell.center_normal[2]
    };

    double gx = local2global[0][0]*pos_local[0] + local2global[0][1]*pos_local[1]
               + local2global[0][2]*pos_local[2] + local2global[0][3];
    double gy = local2global[1][0]*pos_local[0] + local2global[1][1]*pos_local[1]
               + local2global[1][2]*pos_local[2] + local2global[1][3];
    double gz = local2global[2][0]*pos_local[0] + local2global[2][1]*pos_local[1]
               + local2global[2][2]*pos_local[2] + local2global[2][3];

    Vec3d n_local = cell.center_normal;
    double gni = local2global[0][0]*n_local[0] + local2global[0][1]*n_local[1] + local2global[0][2]*n_local[2];
    double gnj = local2global[1][0]*n_local[0] + local2global[1][1]*n_local[1] + local2global[1][2]*n_local[2];
    double gnk = local2global[2][0]*n_local[0] + local2global[2][1]*n_local[1] + local2global[2][2]*n_local[2];

    MeasurePoint mp;
    mp.point_number   = index;
    mp.name           = "SurfacePoint_" + std::to_string(index);
    mp.vct_vectro_type = "SurfacePoint";
    mp.x  = gx;  mp.y  = gy;  mp.z  = gz;
    mp.i  = gni; mp.j  = gnj; mp.k  = gnk;
    mp.i_old = mp.i; mp.j_old = mp.j; mp.k_old = mp.k;
	mp.i2 = 0.0; mp.j2 = 0.0; mp.k2 = 1;
    mp.i2_old = mp.i2; mp.j2_old = mp.j2; mp.k2_old = mp.k2;// X轴留给 BallScan 视点生成时计算
    return mp;
}

// =============================================================================
// generate_surface_points  —  主入口
// =============================================================================
std::vector<MeasurePoint> generate_surface_points(
    const std::vector<PointWithNormal>& point_cloud)
{
    // ===== 测试：从 txt 文件加载点云（覆盖参数）=====
    //std::vector<PointWithNormal> point_cloud_test;
    //{
    //    std::ifstream fin("D:/test/Code_v11/data/QianGai/part_points_transformed.txt");
    //    std::string line;
    //    while (std::getline(fin, line)) {
    //
    //        if (line.empty()) continue;
    //        std::istringstream ss(line);
    //        double x, y, z, nx, ny, nz;
    //        if (ss >> x >> y >> z >> nx >> ny >> nz) {
    //            PointWithNormal pt;
    //            pt.X = x; pt.Y = y; pt.Z = z;
    //            point_cloud_test.push_back(pt);
    //        }
    //    }
    //    std::cout << "[Test] 读入点数：" << point_cloud_test.size() << std::endl;
    //}
    // ===== 测试结束 =====

    std::vector<MeasurePoint> result;

    if (point_cloud.empty()) return result;

    // 内部参数
    const double safe_height = 0.0;
    const double wx = 60.0, wy = 60.0;
    const double sx = 80.0, sy = 60.0;

    // 1. PCA 求主法向
    Vec3d main_normal = compute_main_normal(point_cloud);
    std::cout << "[SurfacePointGen] 主法向量：["
              << main_normal[0] << ", "
              << main_normal[1] << ", "
              << main_normal[2] << "]" << std::endl;

    // 2. 变换到局部坐标系
    Matrix4d local2global;
    std::vector<Vec3d> local_points = transform_to_local_frame(
        point_cloud, main_normal, local2global);

    // 3. 求 AABB
    Vec3d min_coords = local_points[0], max_coords = local_points[0];
    for (const auto& p : local_points) {
        for (int i = 0; i < 3; ++i) {
            min_coords[i] = std::min(min_coords[i], p[i]);
            max_coords[i] = std::max(max_coords[i], p[i]);
        }
    }

    // 4. 滑动窗口生成栅格（内部 BallScanMeasurePoint，含关键点标记）
    std::vector<BallScanMeasurePoint> grid_cells = build_sliding_window_grid(
        local_points, min_coords, max_coords, wx, wy, sx, sy);

    std::cout << "[SurfacePointGen] 滑动窗口生成栅格数：" << grid_cells.size() << std::endl;

    if (grid_cells.empty()) return result;

    // 5. 法向量保持原始方向（局部 Z 轴朝外），不取反
    //    BallScan.cpp 直接用作 TCP Z 轴，与测点处理方式一致

    // //6. 转换：BallScanMeasurePoint → MeasurePoint（局部 → 全局）
    // //   X轴（第二矢量）留给 BallScan.cpp 在生成视点时计算
    //const size_t N = grid_cells.size();
    //for (size_t idx = 0; idx < N; ++idx) {
    //    result.push_back(to_measure_point(grid_cells[idx], local2global,
    //                                      safe_height, static_cast<int>(idx + 1)));
    //}

    //    只保留弓字形转角点（每行首尾，is_important==true），其余直线段上的点丢弃
    const size_t N = grid_cells.size();
    int kept = 0;
    for (size_t idx = 0; idx < N; ++idx) {
        if (!grid_cells[idx].is_important) continue;   // 仅保留转角处的点
        result.push_back(to_measure_point(grid_cells[idx], local2global,
                                          safe_height, ++kept));
    }

    std::cout << "[SurfacePointGen] 生成面点数：" << result.size() << std::endl;
    return result;
}

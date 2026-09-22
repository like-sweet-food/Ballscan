// =============================================================================
// BallScan.cpp  —  球扫可达性分析实现（三阶段拆分版）
//
// 包含四个函数实现：
//   saSort()                  — 模拟退火路径排序（SA1/SA2 共用）
//   analyzeSurfacePoints()    — 面点可达性分析 + 路径预测
//   analyzeEdgePoints()       — 切边点 SA排序 + 可达性分析
//   analyzeEdgePointsRotated()— 切边点旋转后 SA排序 + 可达性分析
// =============================================================================

#include "BallScan.h"
#include "RobotConfig.h"
#include "FixedAxisRotation.h"
#include "BiRRTPlanner.h"
#include "ViewpointPlanner.h"
#include "RobotReachability.h"
#include "kinematicsApi.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <array>
#include <cstdint>
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>
#include <numeric>

using namespace robot_planner;

#define PROCESS_LINE_RESULT_USE_EXTERNAL_VIEWPOINT
#include "process_line_result.h"

// -----------------------------------------------------------------------
// buildTcpPose  —  由测点数据构建 TCP 齐次变换矩阵（静态辅助，保留兼容）
// -----------------------------------------------------------------------
static Matrix4d buildTcpPose(const MeasurePoint& mp,
                              const RobotConfig&  cfg,
                              ViewpointPlanner&   planner)
{
    Vec3d z_axis = ViewpointPlanner::normalize3({mp.i, mp.j, mp.k});
    Vec3d x_ref  = {mp.i2, mp.j2, mp.k2};
    double x_ref_norm = std::sqrt(x_ref[0]*x_ref[0] + x_ref[1]*x_ref[1] + x_ref[2]*x_ref[2]);
    if (x_ref_norm < 1e-6) x_ref = {1.0, 0.0, 0.0};

    Vec3d x_axis = planner.computeOptimalX(z_axis, x_ref);
    Vec3d y_axis = ViewpointPlanner::normalize3(ViewpointPlanner::cross3(z_axis, x_axis));
    x_axis = ViewpointPlanner::cross3(y_axis, z_axis);

    double safeH = (mp.vct_vectro_type == "Surface") ? cfg.safeHeight : cfg.measSafeHeight;

    Vec3d tcp_pos = {
        mp.x + safeH * z_axis[0],
        mp.y + safeH * z_axis[1],
        mp.z + safeH * z_axis[2]
    };

    Matrix4d T = eye4();
    for (int i = 0; i < 3; ++i) {
        T[i][0] = x_axis[i];
        T[i][1] = y_axis[i];
        T[i][2] = z_axis[i];
        T[i][3] = tcp_pos[i];
    }
    return T;
}

// =============================================================================
// simulatedAnnealing — 模拟退火（环形 2-opt）
//
// 输入: D   — N×N 平铺 1D 距离矩阵（D[i*N+j]），节点以 1-based 寻址
//       N   — 节点总数
// 返回: {最优路径长度, 最优路径（1-based 整数序列，path[0]=1 固定）}
//
// 特点：
//   · 50000 次随机 shuffle 暖启动，搜索比旧算法更充分
//   · T₀=10000, L=1000, α=0.9999, Tf=1e-5
//   · 使用 random_device（非确定性，不依赖外部 seed）
// =============================================================================
static std::pair<double, std::vector<int>> simulatedAnnealing(
    const std::vector<double>& D, int N)
{
    if (N < 3) return {0.0, {}};

    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution<double> ur(0.0, 1.0);

    // 中间节点列表：2..N-1（1-based）
    std::vector<int> path(N), path0(N), path_best(N), mid(N - 1);
    for (int i = 0; i < N - 1; i++) mid[i] = i + 2;

    double len = std::numeric_limits<double>::infinity();
    double len_best = len;

    // 暖启动：50000 次随机 shuffle，找最优初始解
    for (int iter = 0; iter < 50000; iter++) {
        std::shuffle(mid.begin(), mid.end(), rng);
        path0[0] = 1;
        for (int i = 0; i < N - 1; i++) path0[i + 1] = mid[i];

        double temp = 0;
        for (int i = 0; i < N - 1; i++)
            temp += D[(path0[i] - 1) * N + (path0[i + 1] - 1)];
        temp += D[(path0[N - 1] - 1) * N + (path0[0] - 1)];  // 闭环项（回程距离，起终点相同）

        if (temp < len) { len = temp; path = path0; }
    }
    len_best = len;
    path_best = path;

    // SA 参数
    const double e  = std::pow(0.1, 5);  // 终止温度 1e-5
    const int    L  = 1000;
    const double at = 0.9999;
    double T = 10000.0;
    double inv_T = 1.0 / T;  // 预计算倒数，内层循环用乘法代替除法

    std::uniform_int_distribution<int> dist_c(2, N);  // c1,c2 ∈ [2, N]（1-based，全部中间节点）

    while (T >= e) {
        for (int k = 0; k < L; k++) {
            int c1 = dist_c(rng);
            int c2 = dist_c(rng);
            if (c1 > c2) std::swap(c1, c2);

            double change = 0;
            if (c2 == N) {
                // 末尾节点不固定，下一节点环绕为 path[0]=起点（D 矩阵 index=0）
                change = D[(path[c1 - 2] - 1) * N + (path[c2 - 1] - 1)]
                       + D[(path[c1 - 1] - 1) * N]
                       - D[(path[c1 - 2] - 1) * N + (path[c1 - 1] - 1)]
                       - D[(path[c2 - 1] - 1) * N];
            } else {
                change = D[(path[c1 - 2] - 1) * N + (path[c2 - 1] - 1)]
                       + D[(path[c1 - 1] - 1) * N + (path[c2] - 1)]
                       - D[(path[c1 - 2] - 1) * N + (path[c1 - 1] - 1)]
                       - D[(path[c2 - 1] - 1) * N + (path[c2] - 1)];
            }

            if (change < 0 || std::exp(-change * inv_T) > ur(rng)) {
                std::reverse(path.begin() + (c1 - 1), path.begin() + c2);
                len += change;
                if (len < len_best) { len_best = len; path_best = path; }
            }
        }
        T *= at;
        inv_T = 1.0 / T;
    }

    return {len_best, path_best};
}

// =============================================================================
// saSort — 模拟退火路径排序（提取自原 SA1/SA2 共用逻辑）
//
// 步骤：
//   1) 找距 startRef 最近的点作为 start_node
//   2) 构建索引重映射（start_node → 位置 0）
//   3) 直接按重映射顺序构建平铺 1D 距离矩阵（省去中间 dists 矩阵）
//   4) 调用 simulatedAnnealing，将结果映射回原始 0-based 索引
//   5) 可选：规范化首个重复块（仅 SA1 需要）
// =============================================================================
std::vector<size_t> BallScan::saSort(
    const std::vector<ShiftedPoint>& shifted,
    const Vec3d& startRef,
    unsigned int seed,
    bool canonicalize) const
{
    const size_t measN = shifted.size();
    if (measN == 0) return {};
    if (measN == 1) return {0};

    // ------------------------------------------------------------------
    // 1. 找起点：距 startRef 最近的点
    //    对应 MATLAB: dist_matrix = sum((...).^2, 2); start_node = find(min)
    // ------------------------------------------------------------------
    size_t start_node = 0;
    {
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < measN; ++i) {
            double dx = startRef[0] - shifted[i].pos[0];
            double dy = startRef[1] - shifted[i].pos[1];
            double dz = startRef[2] - shifted[i].pos[2];
            double d = dx * dx + dy * dy + dz * dz;
            if (d < min_dist) { min_dist = d; start_node = i; }
        }
    }

    const int N = (int)measN;

    // ------------------------------------------------------------------
    // [旧 SA 已注释保留，新实现见步骤3~5]
    //
    // auto totalDistance = [&](const std::vector<size_t>& tour) {
    //     double d = 0.0;
    //     size_t num_cities = tour.size();
    //     for (size_t i = 0; i + 1 < num_cities; ++i)
    //         d += dists[tour[i]][tour[i + 1]];
    //     return d;
    // };
    //
    // const double initial_temp = 100.0;
    // const double final_temp   = 1e-3;
    // const double alpha        = 0.995;
    // const int    max_iter     = 200;
    //
    // std::mt19937 rng(seed);
    // auto matlabRand = [&]() -> double {
    //     const std::uint32_t a = rng();
    //     const std::uint32_t b = rng();
    //     return ((double)(a >> 5) * 67108864.0 + (double)(b >> 6))
    //          / 9007199254740992.0;
    // };
    // auto matlabRandperm = [&](size_t n) {
    //     std::vector<std::pair<double, size_t>> keyed;
    //     keyed.reserve(n);
    //     for (size_t i = 0; i < n; ++i) keyed.push_back({matlabRand(), i});
    //     std::stable_sort(keyed.begin(), keyed.end(),
    //         [](const std::pair<double, size_t>& lhs,
    //            const std::pair<double, size_t>& rhs) {
    //             if (lhs.first != rhs.first) return lhs.first < rhs.first;
    //             return lhs.second < rhs.second;
    //         });
    //     std::vector<size_t> perm;
    //     perm.reserve(n);
    //     for (const auto& item : keyed) perm.push_back(item.second);
    //     return perm;
    // };
    //
    // std::vector<size_t> other_nodes;
    // other_nodes.reserve(N > 0 ? N - 1 : 0);
    // for (size_t i = 0; i < N; ++i)
    //     if (i != start_node) other_nodes.push_back(i);
    // std::vector<size_t> initial_permutation = other_nodes;
    // {
    //     const std::vector<size_t> perm_idx = matlabRandperm(other_nodes.size());
    //     for (size_t i = 0; i < perm_idx.size(); ++i)
    //         initial_permutation[i] = other_nodes[perm_idx[i]];
    // }
    // std::vector<size_t> current_tour;
    // current_tour.reserve(N);
    // current_tour.push_back(start_node);
    // for (size_t v : initial_permutation) current_tour.push_back(v);
    // double current_dist = totalDistance(current_tour);
    // std::vector<size_t> best_tour = current_tour;
    // double best_dist = current_dist;
    // int iter_count = 0;
    //
    // double current_temp = initial_temp;
    // while (current_temp > final_temp) {
    //     for (int i = 0; i < max_iter; ++i) {
    //         if (N < 3) break;
    //         std::vector<size_t> draw_base(N - 1);
    //         std::iota(draw_base.begin(), draw_base.end(), 0);
    //         size_t j1 = static_cast<size_t>(std::floor((N - 1) * matlabRand()));
    //         if (j1 >= N - 1) j1 = N - 2;
    //         std::swap(draw_base[0], draw_base[j1]);
    //         size_t j2 = 1 + static_cast<size_t>(std::floor((N - 2) * matlabRand()));
    //         if (j2 >= N - 1) j2 = N - 2;
    //         std::swap(draw_base[1], draw_base[j2]);
    //         size_t idx1 = std::min(draw_base[0], draw_base[1]) + 1;
    //         size_t idx2 = std::max(draw_base[0], draw_base[1]) + 1;
    //         double r = matlabRand();
    //         std::vector<size_t> new_tour = current_tour;
    //         std::reverse(new_tour.begin() + idx1, new_tour.begin() + idx2 + 1);
    //         double new_dist = totalDistance(new_tour);
    //         double delta_dist = new_dist - current_dist;
    //         if (delta_dist < 0.0) {
    //             current_tour = new_tour; current_dist = new_dist;
    //             if (current_dist < best_dist) { best_tour = current_tour; best_dist = current_dist; }
    //         } else {
    //             if (matlabRand() < std::exp(-delta_dist / current_temp))
    //                 { current_tour = new_tour; current_dist = new_dist; }
    //         }
    //     }
    //     ++iter_count;
    //     current_temp *= alpha;
    // }
    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    // 2. 构建索引重映射：start_node → 位置 0（D 矩阵的 1-based 节点 1）
    //    其余节点依原始编号顺序填入位置 1..N-1
    // ------------------------------------------------------------------
    std::vector<size_t> idx_map((size_t)N);
    idx_map[0] = start_node;
    {
        size_t pos = 1;
        for (int i = 0; i < N; ++i)
            if ((size_t)i != start_node) idx_map[pos++] = (size_t)i;
    }

    // ------------------------------------------------------------------
    // 3. 直接按重映射顺序构建平铺 1D 距离矩阵（省去中间 dists 矩阵）
    // ------------------------------------------------------------------
    std::vector<double> D((size_t)N * N, 0.0);
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            size_t ai = idx_map[i], aj = idx_map[j];
            double dx = shifted[ai].pos[0] - shifted[aj].pos[0];
            double dy = shifted[ai].pos[1] - shifted[aj].pos[1];
            double dz = shifted[ai].pos[2] - shifted[aj].pos[2];
            double d = std::sqrt(dx * dx + dy * dy + dz * dz);
            D[i * N + j] = d;
            D[j * N + i] = d;
        }
    }

    // ------------------------------------------------------------------
    // 4. 调用 simulatedAnnealing（tuihuo.cpp 版）
    //    返回值：1-based 路径，路径[0]=1（start_node）
    // ------------------------------------------------------------------
    std::vector<size_t> best_tour;
    double best_dist = 0.0;

    if (N < 3) {
        // N==2 直接返回，无需优化
        best_tour.resize(N);
        for (size_t i = 0; i < N; ++i) best_tour[i] = idx_map[i];
    } else {
        auto sa_result = simulatedAnnealing(D, N);
        best_dist = sa_result.first;
        const std::vector<int>& sa_path = sa_result.second;
        best_tour.resize(N);
        for (size_t i = 0; i < N; ++i)
            best_tour[i] = idx_map[(size_t)(sa_path[i] - 1)];  // 1-based → 重映射位置 → 原始 0-based
    }

    // 终点固定回起点：路径首尾相同，构成完整环路
    best_tour.push_back(start_node);

    // ------------------------------------------------------------------
    // 6. 可选：规范化首个重复块（SA1 需要，SA2 不需要）
    //    当存在坐标完全相同的点时，确保确定性输出
    // ------------------------------------------------------------------
    if (canonicalize)
    {
        auto sameShiftedPoint = [&](size_t lhs, size_t rhs) {
            constexpr double eps = 1e-9;
            for (int axis = 0; axis < 3; ++axis) {
                if (std::abs(shifted[lhs].pos[axis] - shifted[rhs].pos[axis]) > eps) return false;
                if (std::abs(shifted[lhs].normal[axis] - shifted[rhs].normal[axis]) > eps) return false;
            }
            return true;
        };

        for (size_t i = 1; i < best_tour.size(); ++i) {
            if (!sameShiftedPoint(best_tour[i - 1], best_tour[i])) continue;
            size_t block_begin = i - 1;
            size_t block_end = i;
            while (block_end + 1 < best_tour.size()
                && sameShiftedPoint(best_tour[block_end], best_tour[block_end + 1])) {
                ++block_end;
            }
            std::sort(best_tour.begin() + block_begin, best_tour.begin() + block_end + 1);
            break;
        }
    }

    std::cout << "[BallScan] SA done. start_node=" << start_node
              << " best_dist=" << best_dist << std::endl;

    return best_tour;
}

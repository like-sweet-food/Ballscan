// =============================================================================
// BallScan.cpp  —  球扫可达性分析实现（三阶段拆分版）
//
// 包含的函数实现：
//   saSort()                  — 模拟退火路径排序（SA1/SA2 共用）
//   analyzeSurfacePoints()    — 面点可达性分析 + 路径预测
//   analyzeEdgePoints()       — 切边点 SA排序 + 可达性分析（保留兼容入口）
//   analyzeEdgePointsRotated()— 切边点旋转后 SA排序 + 可达性分析（保留兼容入口）
//   analyzeEdgePointsCombined()— 切边点 + 旋转切边点合并分析（单次 SA / 转台角）
// =============================================================================

#include "BallScan.h"
#include "RobotConfig.h"
#include "ViewpointPlanner.h"
#include "RobotReachability.h"
#include "two_orientation_interpolation.h"
#include "kinematicsApi.h"
#include "trajectory.h"
#include "robot_robotKinematicsCollisionInterface.h"
#include "base_sceneDocument.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>
#include <numeric>

using namespace robot_planner;

#define PROCESS_LINE_RESULT_USE_EXTERNAL_VIEWPOINT
#include "process_line_result.h"
#include "path_rrt_splice.h"

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

static ProcessLineCallbacks makeProcessLineCallbacks(const RobotConfig& cfg)
{
    ExistingPlannerHooks hooks;
    const std::string robot_name = cfg.robotName;

    hooks.calculatetimecpp =
        [](const JointVec6& q1, const JointVec6& q2) -> double {
            return calculateTime(q1, q2, 1, 1);
        };

    hooks.mutinomial =
        [](const JointVec6& q1,
           const JointVec6& q2,
           double t,
           double T) -> JointVec6 {
            return multinomial(q1, q2, t, T);
        };

    hooks.isJointStateSafe =
        [robot_name, cfg](const JointVec6& q_deg, double /*partRotate*/) -> bool {
            // 修改二：补充关节限位检查（原实现只做静态碰撞，未检查关节是否超限）
            for (int i = 0; i < 6; ++i) {
                if (q_deg[i] < cfg.jointLimits[i].minDeg ||
                    q_deg[i] > cfg.jointLimits[i].maxDeg)
                    return false;
            }
            const std::vector<double> q(q_deg.begin(), q_deg.end());
            return !KinematicsApi::instance().checkCollision(robot_name, q, q);
        };

    hooks.forwardKinematics =
        [robot_name](const JointVec6& q_deg) -> Matrix4d {
            const std::vector<double> q(q_deg.begin(), q_deg.end());
            const auto fk_res =
                KinematicsApi::instance().fk(robot_name, q, LinkName::TCP);

            Matrix4d T = eye4();
            if (fk_res.ut.size() < 4) {
                return T;
            }

            for (int r = 0; r < 4; ++r) {
                if (fk_res.ut[r].size() < 4) {
                    return eye4();
                }
                for (int c = 0; c < 4; ++c) {
                    T[r][c] = fk_res.ut[r][c];
                }
            }
            return T;
        };

    return makeCallbacksWithExistingPlanner(hooks, 0.05);
}

// =============================================================================
// makeRRTContext — 为 processLineResult 的 RRT 回退分支构造上下文
//
// 三个函数指针直接复用 RobotKinematicsCollisionInterface 的静态成员函数，
// 它们的签名与 PathPlan 期望的 typedef 完全一致（main.cpp 启动时也是用它们
// bind 到 KinematicsApi 单例的，两边共享同一实现）。
// RobotTechParameters 从 SceneDocument 单例按机器人名取得。
// =============================================================================
static RRTContext makeRRTContext(const RobotConfig& cfg,
                                 const RobotTechParameters& techParams)
{
    RRTContext ctx;
    ctx.robot_name   = cfg.robotName;
    ctx.tech_params  = &techParams;
    ctx.ik_fn        = &RobotKinematicsCollisionInterface::inverseSolution;
    ctx.fk_fn        = &RobotKinematicsCollisionInterface::forwardSolution;
    ctx.collision_fn = &RobotKinematicsCollisionInterface::checkCollision;
    return ctx;
}

// =============================================================================
// Xoshiro256++ — 高速伪随机数生成器（替代 mt19937，吞吐量约为 2~4 倍）
// =============================================================================
struct Xoshiro256pp {
    uint64_t s[4];

    Xoshiro256pp() {
        std::random_device rd;
        for (int i = 0; i < 4; ++i)
            s[i] = ((uint64_t)rd() << 32) | (uint64_t)rd();
    }

    // 确定性种子构造：SplitMix64 把 64-bit seed 扩展为 4 个状态字。
    // 用于让 SA 等流程可复现（外部传 cfg.randomSeed）
    explicit Xoshiro256pp(uint64_t seed) {
        uint64_t x = seed;
        for (int i = 0; i < 4; ++i) {
            x += 0x9E3779B97F4A7C15ULL;
            uint64_t z = x;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
            s[i] = z ^ (z >> 31);
        }
    }

    uint64_t next() {
        const uint64_t result = rotl64(s[0] + s[3], 23) + s[0];
        const uint64_t t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = rotl64(s[3], 45);
        return result;
    }

    // [lo, hi] 均匀整数（轻微 modulo bias，SA 可接受）
    int nextInt(int lo, int hi) {
        return lo + (int)(next() % (uint64_t)(hi - lo + 1));
    }

    // [0, 1) 均匀浮点
    double nextDouble() {
        return (double)(next() >> 11) * (1.0 / (double)(1ULL << 53));
    }

private:
    static uint64_t rotl64(uint64_t x, int k) {
        return (x << k) | (x >> (64 - k));
    }
};

// =============================================================================
// simulatedAnnealing — 模拟退火（环形 2-opt）
//
// 输入: D   — N×N 平铺 1D 距离矩阵（D[i*N+j]），节点以 1-based 寻址
//       N   — 节点总数
// 返回: {最优路径长度, 最优路径（1-based 整数序列，path[0]=1 固定）}
//
// 特点：
//   · 最近邻贪心初始化，初始解质量优于随机 shuffle
//   · T₀=10000, L=1000, α=0.9999, Tf=1e-5
//   · Xoshiro256++ RNG（非确定性，吞吐量约为 mt19937 的 2~4 倍）
// =============================================================================
static std::pair<double, std::vector<int>> simulatedAnnealing(
    const std::vector<double>& D, int N, unsigned int seed)
{
    if (N < 3) return {0.0, {}};

    Xoshiro256pp rng(static_cast<uint64_t>(seed));

    std::vector<int> path(N), path_best(N);

    // 最近邻贪心初始化：从节点1出发，每步选最近未访问节点，O(N²) 一次完成
    {
        std::vector<bool> visited(N, false);
        path[0] = 1;
        visited[0] = true;
        for (int step = 1; step < N; ++step) {
            int cur0 = path[step - 1] - 1;  // 当前节点 0-based 索引
            double best_d = std::numeric_limits<double>::infinity();
            int best_j = -1;
            for (int j = 0; j < N; ++j) {
                if (!visited[j] && D[cur0 * N + j] < best_d) {
                    best_d = D[cur0 * N + j];
                    best_j = j;
                }
            }
            path[step] = best_j + 1;  // 转回 1-based
            visited[best_j] = true;
        }
    }

    double len = 0.0;
    for (int i = 0; i < N - 1; ++i)
        len += D[(path[i] - 1) * N + (path[i + 1] - 1)];
    len += D[(path[N - 1] - 1) * N];  // 闭环回程（path[0]=1，列索引=0）

    double len_best = len;
    path_best = path;

    // SA 参数
    const double e  = std::pow(0.1, 5);  // 终止温度 1e-5
    const int    L  = 1000;
    const double at = 0.9999;
    double T = 10000.0;
    double inv_T = 1.0 / T;  // 预计算倒数，内层循环用乘法代替除法

    while (T >= e) {
        for (int k = 0; k < L; k++) {
            int c1 = rng.nextInt(2, N);
            int c2 = rng.nextInt(2, N);
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

            if (change < 0 || std::exp(-change * inv_T) > rng.nextDouble()) {
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
        auto sa_result = simulatedAnnealing(D, N, seed);
        best_dist = sa_result.first;
        const std::vector<int>& sa_path = sa_result.second;
        best_tour.resize(N);
        for (size_t i = 0; i < N; ++i)
            best_tour[i] = idx_map[(size_t)(sa_path[i] - 1)];  // 1-based → 重映射位置 → 原始 0-based
    }

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

// =============================================================================
// analyzeSurfacePoints — 面点可达性分析
//
// 对应原 BallScan::allocation() 第106行～第256行
// =============================================================================
void BallScan::analyzeSurfacePoints(
    const std::string& robotName,
    const std::vector<MeasurePoint>& surfacePoints,
    RobotConfig& cfg,
    ScanState& state,
    std::vector<ViewPoint>& outSurfacePath,
    std::vector<MeasurePointPoseSet>& outReachable,
    std::vector<MeasurePointPoseSet>& outUnreachable)
{
    // ------------------------------------------------------------------
    // 1. 创建规划器
    // ------------------------------------------------------------------
    ViewpointPlanner planner(cfg);

    // [CHK] 可达性失败原因计数清零，循环结束后 dump，定位斜面点死在哪道筛子
    robot_planner::reachReasonReset();

    const size_t surfN = surfacePoints.size();
    // 本次调用的可达面点列表（用于路径预测，不放入 state.reachHistory）
    std::vector<ViewPoint> reach_surface_vps;

    // ------------------------------------------------------------------
    // 2. 前向X轴预计算（对应 MATLAB 步骤7）
    //    首点/第二点：用 point[2]-point[1] 方向
    //    中间点：用 point[i+1]-point[i] 方向
    //    末点：复制前一个方向
    // ------------------------------------------------------------------
    std::vector<Vec3d> forward_x_axes(surfN, {1.0, 0.0, 0.0});
    if (surfN >= 2) {
        for (size_t i = 0; i < surfN; ++i) {
            if (i == 0) {
                size_t next = (surfN > 2) ? 2 : 1;
                size_t cur  = (surfN > 2) ? 1 : 0;
                Vec3d diff = {
                    surfacePoints[next].x - surfacePoints[cur].x,
                    surfacePoints[next].y - surfacePoints[cur].y,
                    surfacePoints[next].z - surfacePoints[cur].z
                };
                forward_x_axes[0] = ViewpointPlanner::normalize3(diff);
            } else if (i < surfN - 1) {
                Vec3d diff = {
                    surfacePoints[i+1].x - surfacePoints[i].x,
                    surfacePoints[i+1].y - surfacePoints[i].y,
                    surfacePoints[i+1].z - surfacePoints[i].z
                };
                forward_x_axes[i] = ViewpointPlanner::normalize3(diff);
            } else {
                forward_x_axes[i] = forward_x_axes[i - 1];
            }
        }
    }

    // ------------------------------------------------------------------
    // 3. 逐点视点生成 + 可达性分析
    //    对应 MATLAB: 步骤7(X轴确定) + 视点姿态构建 + 可达性判断
    // ------------------------------------------------------------------
    for (size_t si = 0; si < surfN; ++si)
    {
        if (si == 166) {
            std::cout << "si == 166" << std::endl;
        }
        const MeasurePoint& mp = surfacePoints[si];
        // 法向量（全局，已经是取反后的视点方向）
        Vec3d z_axis = ViewpointPlanner::normalize3({mp.i, mp.j, mp.k});

        // 位置 + 安全高度抬起（使用 safeHeight，面点专用）
        Vec3d tcp_pos = {
            mp.x + cfg.safeHeight * z_axis[0],
            mp.y + cfg.safeHeight * z_axis[1],
            mp.z + cfg.safeHeight * z_axis[2]
        };

        Matrix4d tcp_pose;

        // 斜面点（棱边增补点）：名字以 SurfaceDiagonal_ 开头，且自带有效第二矢量 i2
        const bool isDiagonal = (mp.name.rfind("SurfaceDiagonal_", 0) == 0);

        if (si == 0) {
            // 第一个视点：用前向差分X轴 + computeOptimalX 构造姿态
            Vec3d preset_x = forward_x_axes[0];
            Vec3d x_axis = planner.computeOptimalX(z_axis, preset_x);
            Vec3d y_axis = ViewpointPlanner::normalize3(
                ViewpointPlanner::cross3(z_axis, x_axis));
            x_axis = ViewpointPlanner::cross3(y_axis, z_axis);

            tcp_pose = eye4();
            for (int r = 0; r < 3; ++r) {
                tcp_pose[r][0] = x_axis[r];
                tcp_pose[r][1] = y_axis[r];
                tcp_pose[r][2] = z_axis[r];
                tcp_pose[r][3] = tcp_pos[r];
            }
        } else if (isDiagonal) {
            // 斜面点：用自身第二矢量(i2)直接建姿，不继承上一个点的 roll。
            // 上游面点链的起手方向 forward_x_axes[0] 会随“面点抽稀”而改变，
            // 经 constructNextBestView 最小旋转链一路传到此处会污染斜面点姿态，
            // 使其落出可达窗口 → 求不出解。这里切断该依赖，使其与上游解耦。
            Vec3d preset_x = {mp.i2, mp.j2, mp.k2};
            tcp_pose = planner.buildTcpPoseLocal(tcp_pos, z_axis, preset_x);
        } else {
            // 后续视点：基于最小旋转变换量，从上一个姿态推导
            tcp_pose = planner.constructNextBestView(state.lastViewPose, z_axis);
            for (int r = 0; r < 3; ++r)
                tcp_pose[r][3] = tcp_pos[r];
        }
        // 无论可达与否都更新 lastViewPose（与原代码行为一致）
        state.lastViewPose = tcp_pose;

        // 可达性判断（面点；前后关节角约束已在 robotReachability 内部恒定启用）
        JointVec6 out_joints;
        bool isReachable = robot_planner::robotReachability(
            tcp_pose, cfg, state.reachHistory, out_joints);

        // 不可达时尝试扰动补救（无转台备选）
        if (!isReachable)
        {
            Matrix4d new_tcp;
            JointVec6 adj_joints;
            bool adjusted = planner.adjustViewPose(
                tcp_pose, state.reachHistory, new_tcp, adj_joints);
            if (adjusted) {
                tcp_pose    = new_tcp;
                out_joints  = adj_joints;
                isReachable = true;
            } else {
                // 扰动后仍不可达：直接丢弃该点，不写入路径容器
                // （避免 NaN 占位点流入 processLineResult / PlanPathRRTconnect_new）
                std::cout << "[BallScan] 面点扰动仍不可达，已丢弃该点" << std::endl;
            }
        }

        // 构建结果 MeasurePointPoseSet（新增：原代码面点没有输出到结果集）
        MeasurePointPoseSet pose_set;
        pose_set.point_number    = mp.point_number;
        pose_set.name            = mp.name;
        pose_set.vct_vector_type = mp.vct_vectro_type;
        pose_set.x = mp.x;  pose_set.y = mp.y;  pose_set.z = mp.z;
        pose_set.i = mp.i;  pose_set.j = mp.j;  pose_set.k = mp.k;
        pose_set.i2 = mp.i2; pose_set.j2 = mp.j2; pose_set.k2 = mp.k2;
        pose_set.isMeasurePoint = false;  // 面点不是测点

        if (isReachable)
        {
            // 填充可达姿态配置
            PoseConfiguration pose_cfg;
            pose_cfg.point_name   = mp.name;
            pose_cfg.point_number = mp.point_number;
            pose_cfg.xyzwpr.x     = mp.x;
            pose_cfg.xyzwpr.y     = mp.y;
            pose_cfg.xyzwpr.z     = mp.z;
            pose_cfg.joints.j1    = out_joints[0];
            pose_cfg.joints.j2    = out_joints[1];
            pose_cfg.joints.j3    = out_joints[2];
            pose_cfg.joints.j4    = out_joints[3];
            pose_cfg.joints.j5    = out_joints[4];
            pose_cfg.joints.j6    = out_joints[5];
            pose_set.configurations.push_back(pose_cfg);
            pose_set.is_accessible = true;
            outReachable.push_back(pose_set);

            // 记录可达视点
            ViewPoint vp;
            vp.joints          = out_joints;
            vp.globalT         = tcp_pose;
            vp.isImportant     = true;
            vp.isPathJoint     = false;
            vp.moveType        = 1;
            vp.partRotateAngle = 0.0;
            vp.sourcePointId = mp.vct_vectro_type + "|" + mp.name + "|"
                + std::to_string(mp.point_number);
            vp.sourcePointPosition = {mp.x, mp.y, mp.z};
            vp.hasSourcePointPosition = true;
            reach_surface_vps.push_back(vp);    // 用于路径预测
            // 调试：定位第300个"可达输出点"。
            // reach_surface_vps.size() = 即将写入点的 0-based 输出索引；
            // 第300个(1-based) 对应索引299，即 size()==299 时正要 push 它。
            if (reach_surface_vps.size() == 90) {
                std::cout << "[Debug] 第451个输出点: si(输入下标)=" << si
                    << " 测点名=" << mp.name
                    << " point_number=" << mp.point_number << std::endl;  // ← 在这行下断点
            }
            state.reachHistory.push_back(vp);    // 跨阶段关节历史
            cfg.lastView = out_joints;
            state.lastJoints    = out_joints;
            state.hasLastJoints = true;
        }
        else
        {
            pose_set.is_accessible = false;
            outUnreachable.push_back(pose_set);
        }
    }

    // ------------------------------------------------------------------
    // 4. 面点路径预测（相邻可达面点之间做 linePathPredict）
    //    对应 MATLAB: partPlaceViewPlanning.m 第348-366行
    //
    //    MATLAB 结果结构（N 个可达视点）：
    //      LineProcessResult 初始为空，每段直接整体拼接 LinePathPredict 输出（含起点）
    //    - 最后一轮（index==length）追加 start_info（即 vp(N-2)）作为 HOME 标记
    //    - vp(N-1)（最后一个视点）不出现在结果中
    // ------------------------------------------------------------------
    // if (!reach_surface_vps.empty())
    // {
    //     for (size_t i = 0; i + 1 < reach_surface_vps.size(); ++i) {
    //         auto seg = planner.linePathPredict(
    //             reach_surface_vps[i], reach_surface_vps[i + 1]);
    //         outSurfacePath.insert(outSurfacePath.end(),
    //                               seg.begin(), seg.end());
    //     }
    //     // HOME 标记：对应 MATLAB index==length 时追加 start_info（倒数第二个视点）
    //     if (reach_surface_vps.size() >= 2) {
    //         outSurfacePath.push_back(reach_surface_vps[reach_surface_vps.size() - 2]);
    //     }
    // }
    outSurfacePath = reach_surface_vps;

    //outSurfacePath = processLineResult(outSurfacePath, {}, {});

    // 对应 MATLAB partPlaceViewPlanning.m: config.last_view = LineProcessResult(end, 1:6)
    // 必须在 processLineResult 之后更新，与 MATLAB 赋值时机一致
    if (!outSurfacePath.empty()) {
        state.lastViewPose = outSurfacePath.back().globalT;
        state.lastJoints   = outSurfacePath.back().joints;
    }

    std::cout << "[BallScan] 机器人 " << robotName
              << " 面点可达性：" << reach_surface_vps.size()
              << "/" << surfacePoints.size() << " 可达" << std::endl;

    // [CHK] 打印本次可达性分析的失败原因分布（ikEmpty/filtered + 各分支剔除原因）
    robot_planner::reachReasonDump("surface");
}

// =============================================================================
// analyzeStandardPoints — Standard / Standard_circle 测点可达性分析（锥形 N 视点）
//
// 框架与 analyzeSurfacePoints 完全一致（前向X轴预计算、基准坐标系、可达性判断 +
// 扰动补救、state 更新），唯一区别在于用「锥形 N 视点」替代 constructNextBestView：
//   1) 先绕测点基准坐标系的局部 x 轴掀开 tiltDeg（圆锥半顶角），得到倾斜朝向；
//   2) 再把倾斜后的坐标系绕该点原始 z 法向矢量每隔 360/coneCount 度旋转一圈，
//      共 coneCount 个朝向，z' 扫出以原法向为中轴、半顶角 tiltDeg 的圆锥；
//   3) 这 coneCount 个位姿共用同一 TCP 位置（激光笔原地不动），只有朝向不同；
//   4) 每个位姿各自独立走可达性判断，可达的全部写入结果集，name 追加 _C00.._Cnn
//      后缀以区分，避免下游按 name 回查撞名。
// =============================================================================
void BallScan::analyzeStandardPoints(
    const std::string& robotName,
    const std::vector<MeasurePoint>& standardPoints,
    RobotConfig& cfg,
    ScanState& state,
    std::vector<ViewPoint>& outPath,
    std::vector<MeasurePointPoseSet>& outReachable,
    std::vector<MeasurePointPoseSet>& outUnreachable,
    double tiltDeg,
    int    coneCount)
{
    // ------------------------------------------------------------------
    // 1. 创建规划器
    // ------------------------------------------------------------------
    ViewpointPlanner planner(cfg);
    robot_planner::reachReasonReset();

    const size_t stdN = standardPoints.size();
    if (stdN == 0 || coneCount <= 0) return;

    // 本次调用的可达视点列表（用于路径，不放入 state.reachHistory 之外的逻辑）
    std::vector<ViewPoint> reach_std_vps;

    // ------------------------------------------------------------------
    // 2. 前向X轴预计算（与面点一致）
    // ------------------------------------------------------------------
    std::vector<Vec3d> forward_x_axes(stdN, {1.0, 0.0, 0.0});
    if (stdN >= 2) {
        for (size_t i = 0; i < stdN; ++i) {
            if (i == 0) {
                size_t next = (stdN > 2) ? 2 : 1;
                size_t cur  = (stdN > 2) ? 1 : 0;
                Vec3d diff = {
                    standardPoints[next].x - standardPoints[cur].x,
                    standardPoints[next].y - standardPoints[cur].y,
                    standardPoints[next].z - standardPoints[cur].z
                };
                forward_x_axes[0] = ViewpointPlanner::normalize3(diff);
            } else if (i < stdN - 1) {
                Vec3d diff = {
                    standardPoints[i+1].x - standardPoints[i].x,
                    standardPoints[i+1].y - standardPoints[i].y,
                    standardPoints[i+1].z - standardPoints[i].z
                };
                forward_x_axes[i] = ViewpointPlanner::normalize3(diff);
            } else {
                forward_x_axes[i] = forward_x_axes[i - 1];
            }
        }
    }

    const double tiltRad = tiltDeg * M_PI / 180.0;
    const double ct = std::cos(tiltRad);
    const double st = std::sin(tiltRad);

    // ------------------------------------------------------------------
    // 3. 逐点：基准坐标系 → 锥形 coneCount 视点 → 逐个可达性判断
    // ------------------------------------------------------------------
    for (size_t si = 0; si < stdN; ++si)
    {
        const MeasurePoint& mp = standardPoints[si];

        // 基准坐标系（与面点一致）：z = 法向, x = computeOptimalX, y = z×x, x = y×z
        Vec3d z_axis = ViewpointPlanner::normalize3({mp.i, mp.j, mp.k});
        Vec3d x_axis = planner.computeOptimalX(z_axis, forward_x_axes[si]);
        Vec3d y_axis = ViewpointPlanner::normalize3(
            ViewpointPlanner::cross3(z_axis, x_axis));
        x_axis = ViewpointPlanner::cross3(y_axis, z_axis);

        // 基准 TCP 位置（激光笔不动）：mp + safeHeight * 原法向
        Vec3d tcp_pos = {
            mp.x + cfg.safeHeight * z_axis[0],
            mp.y + cfg.safeHeight * z_axis[1],
            mp.z + cfg.safeHeight * z_axis[2]
        };

        // 绕局部 x 轴掀开 tiltDeg → 倾斜后的坐标系（列向量，世界系表达）
        //   tx = x_axis
        //   ty = ct*y + st*z
        //   tz = -st*y + ct*z   （半顶角 tiltDeg 的圆锥中线初始方向）
        Vec3d tx = x_axis;
        Vec3d ty = {
            ct * y_axis[0] + st * z_axis[0],
            ct * y_axis[1] + st * z_axis[1],
            ct * y_axis[2] + st * z_axis[2]
        };
        Vec3d tz = {
            -st * y_axis[0] + ct * z_axis[0],
            -st * y_axis[1] + ct * z_axis[1],
            -st * y_axis[2] + ct * z_axis[2]
        };

        // 绕原始 z 法向（单位轴）旋转 phi 的 Rodrigues 公式：
        //   v' = v*cos + (a×v)*sin + a*(a·v)*(1-cos)
        auto rotAboutNormal = [&](const Vec3d& v, double cphi, double sphi) -> Vec3d {
            Vec3d axv = ViewpointPlanner::cross3(z_axis, v);
            double adv = z_axis[0]*v[0] + z_axis[1]*v[1] + z_axis[2]*v[2];
            double k = adv * (1.0 - cphi);
            return Vec3d{
                v[0]*cphi + axv[0]*sphi + z_axis[0]*k,
                v[1]*cphi + axv[1]*sphi + z_axis[1]*k,
                v[2]*cphi + axv[2]*sphi + z_axis[2]*k
            };
        };

        // 锥形 coneCount 个候选位姿，逐个独立判可达
        for (int ci = 0; ci < coneCount; ++ci)
        {
            double phi  = (2.0 * M_PI * ci) / coneCount;
            double cphi = std::cos(phi), sphi = std::sin(phi);

            Vec3d cx = rotAboutNormal(tx, cphi, sphi);
            Vec3d cy = rotAboutNormal(ty, cphi, sphi);
            Vec3d cz = rotAboutNormal(tz, cphi, sphi);

            Matrix4d tcp_pose = eye4();
            for (int r = 0; r < 3; ++r) {
                tcp_pose[r][0] = cx[r];
                tcp_pose[r][1] = cy[r];
                tcp_pose[r][2] = cz[r];
                tcp_pose[r][3] = tcp_pos[r];
            }
            // 无论可达与否都更新 lastViewPose（与面点行为一致）
            state.lastViewPose = tcp_pose;

            // 可达性判断
            JointVec6 out_joints;
            bool isReachable = robot_planner::robotReachability(
                tcp_pose, cfg, state.reachHistory, out_joints);

            // 不可达时尝试扰动补救
            if (!isReachable)
            {
                Matrix4d new_tcp;
                JointVec6 adj_joints;
                bool adjusted = planner.adjustViewPose(
                    tcp_pose, state.reachHistory, new_tcp, adj_joints);
                if (adjusted) {
                    tcp_pose    = new_tcp;
                    out_joints  = adj_joints;
                    isReachable = true;
                }
            }

            // 视点名追加锥形后缀（_C00.._Cnn），避免下游按 name 回查撞名
            char suffix[8];
            std::snprintf(suffix, sizeof(suffix), "_C%02d", ci);
            const std::string poseName = mp.name + suffix;

            MeasurePointPoseSet pose_set;
            pose_set.point_number    = mp.point_number;
            pose_set.name            = poseName;
            pose_set.vct_vector_type = mp.vct_vectro_type;
            pose_set.x = mp.x;  pose_set.y = mp.y;  pose_set.z = mp.z;
            pose_set.i = mp.i;  pose_set.j = mp.j;  pose_set.k = mp.k;
            pose_set.i2 = mp.i2; pose_set.j2 = mp.j2; pose_set.k2 = mp.k2;
            pose_set.isMeasurePoint = true;   // Standard / Standard_circle 属于测点

            if (isReachable)
            {
                PoseConfiguration pose_cfg;
                pose_cfg.point_name   = poseName;
                pose_cfg.point_number = mp.point_number;
                pose_cfg.xyzwpr.x     = mp.x;
                pose_cfg.xyzwpr.y     = mp.y;
                pose_cfg.xyzwpr.z     = mp.z;
                pose_cfg.joints.j1    = out_joints[0];
                pose_cfg.joints.j2    = out_joints[1];
                pose_cfg.joints.j3    = out_joints[2];
                pose_cfg.joints.j4    = out_joints[3];
                pose_cfg.joints.j5    = out_joints[4];
                pose_cfg.joints.j6    = out_joints[5];
                pose_set.configurations.push_back(pose_cfg);
                pose_set.is_accessible = true;
                outReachable.push_back(pose_set);

                ViewPoint vp;
                vp.joints          = out_joints;
                vp.globalT         = tcp_pose;
                vp.isImportant     = true;
                vp.isPathJoint     = false;
                // Standard/Standard_circle 走 MOVJ（关节空间）：同一测点 N 个锥形
                // 视点位置相同(TCP 距离=0)，不进笛卡尔直线预测；过渡按需求直接
                // MOVJ、不做碰撞检查。可达 + 扰动成功都走这一支，故统一记 0。
                // 详见 main.cpp 中 allStandardPath 在直线预测「之后」追加的说明。
                vp.moveType        = 0;
                vp.partRotateAngle = 0.0;
                vp.sourcePointId = mp.vct_vectro_type + "|" + mp.name + "|"
                    + std::to_string(mp.point_number);
                vp.sourcePointPosition = {mp.x, mp.y, mp.z};
                vp.hasSourcePointPosition = true;
                reach_std_vps.push_back(vp);

                state.reachHistory.push_back(vp);   // 跨阶段关节历史
                cfg.lastView        = out_joints;
                state.lastJoints    = out_joints;
                state.hasLastJoints = true;
            }
            else
            {
                pose_set.is_accessible = false;
                outUnreachable.push_back(pose_set);
            }
        } // end 锥形 coneCount 循环
    } // end 逐点循环

    outPath = reach_std_vps;
    if (!outPath.empty()) {
        state.lastViewPose = outPath.back().globalT;
        state.lastJoints   = outPath.back().joints;
    }

    std::cout << "[BallScan] 机器人 " << robotName
              << " Standard 点可达视点：" << reach_std_vps.size()
              << "（输入 " << stdN << " 点 × " << coneCount << " 视点）" << std::endl;

    robot_planner::reachReasonDump("standard");
}

// =============================================================================
// analyzeEdgePoints — 切边点可达性分析
//
// 对应原 BallScan::allocation() 第260行～第624行（段1~段3）
// =============================================================================
void BallScan::analyzeEdgePoints(
    const std::string& robotName,
    const std::vector<MeasurePoint>& edgePoints,
    RobotConfig& cfg,
    ScanState& state,
    const Matrix4d& placeEndTcp,
    double currentTurnTableAngle,
    std::vector<MeasurePointPoseSet>& outReachable,
    std::vector<MeasurePointPoseSet>& outUnreachable,
    std::vector<ViewPoint>& outEdgePath)
{
    const size_t measN = edgePoints.size();
    if (measN == 0) return;

    // ------------------------------------------------------------------
    // 1. 创建规划器（isFirst 无论 true/false 都需要，步骤5用到）
    // ------------------------------------------------------------------
    ViewpointPlanner planner(cfg);

    // ------------------------------------------------------------------
    // 2. 取上一视点（或过渡点）TCP 位置作为 SA 起点参考
    //    对应 MATLAB: end_tcp = config.T_user * end_6 * config.T_he
    // ------------------------------------------------------------------
    Matrix4d end_tcp = placeEndTcp;
    Vec3d end_point = { end_tcp[0][3], end_tcp[1][3], end_tcp[2][3] };

    // ------------------------------------------------------------------
    // 3. 段1：批量预处理（法向量归一化 + measSafeHeight 偏移 → shifted）
    //    对应 MATLAB partMeasPointsViewPlanning.m 第1步
    //    isFirst 无论 true/false 都需要：步骤5通过 shifted[idx] 取 z_axis/tcp_pos
    // ------------------------------------------------------------------
    std::vector<ShiftedPoint> shifted(measN);
    for (size_t i = 0; i < measN; ++i) {
        Vec3d n = ViewpointPlanner::normalize3(
            {edgePoints[i].i, edgePoints[i].j, edgePoints[i].k});
        shifted[i].normal = n;
        shifted[i].pos = {
            edgePoints[i].x + cfg.measSafeHeight * n[0],
            edgePoints[i].y + cfg.measSafeHeight * n[1],
            edgePoints[i].z + cfg.measSafeHeight * n[2]
        };
    }

    // ------------------------------------------------------------------
    // 4. 段2：SA 模拟退火排序
    //    每次调用都以 placeEndTcp（或过渡点）为起点参考重新运行 SA，
    //    保证不同转台角度下路径连续性最优。
    // ------------------------------------------------------------------
    std::vector<size_t> sorted_indices =
        saSort(shifted, end_point, cfg.randomSeed, /*canonicalize=*/true);

    // ------------------------------------------------------------------
    // 5. 段3：链式姿态生成 + 可达性检测
    //    对应 MATLAB partMeasPointsViewPlanning.m 第3步
    // ------------------------------------------------------------------
    for (size_t si = 0; si < sorted_indices.size(); ++si)
    {
        size_t idx = sorted_indices[si];
        const MeasurePoint& mp = edgePoints[idx];
        const Vec3d& z_axis  = shifted[idx].normal;
        const Vec3d& tcp_pos = shifted[idx].pos;

        Matrix4d tcp_pose;
        if (si == 0)
        {
            // 第一个测点：前向差分方向作 preset_x + computeOptimalX
            Vec3d preset_x = {1.0, 0.0, 0.0};
            if (sorted_indices.size() >= 2) {
                size_t nextIdx = sorted_indices[1];
                Vec3d diff = {
                    shifted[nextIdx].pos[0] - tcp_pos[0],
                    shifted[nextIdx].pos[1] - tcp_pos[1],
                    shifted[nextIdx].pos[2] - tcp_pos[2]
                };
                double nrm = std::sqrt(
                    diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);
                if (nrm > 1e-6)
                    preset_x = { diff[0]/nrm, diff[1]/nrm, diff[2]/nrm };
            }
            Vec3d x_axis = planner.computeOptimalX(z_axis, preset_x);
            Vec3d y_axis = ViewpointPlanner::normalize3(
                ViewpointPlanner::cross3(z_axis, x_axis));
            x_axis = ViewpointPlanner::cross3(y_axis, z_axis);

            tcp_pose = eye4();
            for (int r = 0; r < 3; ++r) {
                tcp_pose[r][0] = x_axis[r];
                tcp_pose[r][1] = y_axis[r];
                tcp_pose[r][2] = z_axis[r];
                tcp_pose[r][3] = tcp_pos[r];
            }
        }
        else
        {
            // 后续测点：基于上一个视点链式推导（最小旋转变换）
            tcp_pose = planner.constructNextBestView(state.lastViewPose, z_axis);
            for (int r = 0; r < 3; ++r)
                tcp_pose[r][3] = tcp_pos[r];
        }
        // 无论可达与否都更新（与 MATLAB 一致）
        state.lastViewPose = tcp_pose;

        // 可达性判断（切边点；前后关节角约束已在 robotReachability 内部恒定启用）
        JointVec6 out_joints;
        bool isReachable = robot_planner::robotReachability(
            tcp_pose, cfg, state.reachHistory, out_joints);

        double out_part_rotate = 0.0;
        if (!isReachable)
        {
            Matrix4d new_tcp;
            JointVec6 adj_joints;
            bool adjusted = planner.adjustViewPose(
                tcp_pose, state.reachHistory, new_tcp, adj_joints, &out_part_rotate);
            if (adjusted) {
                tcp_pose     = new_tcp;
                out_joints   = adj_joints;
                isReachable  = true;
            } else {
                // 扰动后仍不可达：直接丢弃该点，不写入路径容器
                // （避免 NaN 占位点流入 processLineResult / PlanPathRRTconnect_new）
                std::cout << "[BallScan] 切边点扰动仍不可达，已丢弃该点" << std::endl;
            }
        }

        // 构建结果
        MeasurePointPoseSet pose_set;
        pose_set.point_number    = mp.point_number;
        pose_set.name            = mp.name;
        pose_set.vct_vector_type = mp.vct_vectro_type;
        pose_set.x = mp.x;  pose_set.y = mp.y;  pose_set.z = mp.z;
        pose_set.i = mp.i;  pose_set.j = mp.j;  pose_set.k = mp.k;
        pose_set.i2 = mp.i2; pose_set.j2 = mp.j2; pose_set.k2 = mp.k2;
        pose_set.isMeasurePoint = true;

        if (isReachable)
        {
            PoseConfiguration pose_cfg;
            pose_cfg.point_name   = mp.name;
            pose_cfg.point_number = mp.point_number;
            pose_cfg.xyzwpr.x     = mp.x;
            pose_cfg.xyzwpr.y     = mp.y;
            pose_cfg.xyzwpr.z     = mp.z;
            pose_cfg.joints.j1    = out_joints[0];
            pose_cfg.joints.j2    = out_joints[1];
            pose_cfg.joints.j3    = out_joints[2];
            pose_cfg.joints.j4    = out_joints[3];
            pose_cfg.joints.j5    = out_joints[4];
            pose_cfg.joints.j6    = out_joints[5];
            pose_set.configurations.push_back(pose_cfg);
            pose_set.is_accessible = true;
            outReachable.push_back(pose_set);

            ViewPoint vp;
            vp.joints          = out_joints;
            vp.globalT         = tcp_pose;
            vp.isImportant     = true;
            vp.partRotateAngle = currentTurnTableAngle;
            vp.sourcePointId = mp.vct_vectro_type + "|" + mp.name + "|"
                + std::to_string(mp.point_number);
            vp.sourcePointPosition = {mp.x, mp.y, mp.z};
            vp.hasSourcePointPosition = true;
            state.reachHistory.push_back(vp);
            outEdgePath.push_back(vp);
            cfg.lastView = out_joints;
            state.lastJoints    = out_joints;
            state.hasLastJoints = true;
        }
        else
        {
            pose_set.is_accessible = false;
            outUnreachable.push_back(pose_set);
        }
    }

    std::cout << "[BallScan] 机器人 " << robotName
              << " 切边点可达性：" << outReachable.size()
              << " 可达，" << outUnreachable.size()
              << " 不可达" << std::endl;
}

// =============================================================================
// analyzeEdgePointsRotated — 切边点旋转后可达性分析
//
// 对应原 BallScan::allocation() 第629行～第883行（段4）
// =============================================================================
void BallScan::analyzeEdgePointsRotated(
    const std::string& robotName,
    const std::vector<MeasurePoint>& edgePoints,
    double rotateAngleDeg,
    RobotConfig& cfg,
    ScanState& state,
    double currentTurnTableAngle,
    const Matrix4d& placeEndTcp,
    std::vector<MeasurePointPoseSet>& outReachable,
    std::vector<MeasurePointPoseSet>& outUnreachable,
    std::vector<ViewPoint>& outEdgePath)
{
    const size_t measN = edgePoints.size();
    if (measN == 0) return;

    // ------------------------------------------------------------------
    // 1. 创建规划器
    // ------------------------------------------------------------------
    ViewpointPlanner planner(cfg);

    // ------------------------------------------------------------------
    // 2. 批量预处理（法向量归一化 + measSafeHeight 偏移 → shifted）
    //    与 analyzeEdgePoints 段1 逻辑相同
    // ------------------------------------------------------------------
    std::vector<ShiftedPoint> shifted(measN);
    for (size_t i = 0; i < measN; ++i) {
        Vec3d n = ViewpointPlanner::normalize3(
            {edgePoints[i].i, edgePoints[i].j, edgePoints[i].k});
        shifted[i].normal = n;
        shifted[i].pos = {
            edgePoints[i].x + cfg.measSafeHeight * n[0],
            edgePoints[i].y + cfg.measSafeHeight * n[1],
            edgePoints[i].z + cfg.measSafeHeight * n[2]
        };
    }

    // ------------------------------------------------------------------
    // 3. SA2 排序
    //    参考点 = placeEndTcp 的 TCP 位置（上一角度过渡点或阶段2末尾位置）
    //    每次调用都重新运行 SA2，保证不同转台角度之间路径连续性最优。
    //    canonicalize = false（SA2 行为）
    // ------------------------------------------------------------------
    Vec3d ref2 = { placeEndTcp[0][3], placeEndTcp[1][3], placeEndTcp[2][3] };

    // [DEBUG] 诊断：SA2 起点参考 vs 切边段末尾 TCP 位置
    std::cout << "[DEBUG][RotEdge] SA2 ref2(来自placeEndTcp) = ("
              << ref2[0] << ", " << ref2[1] << ", " << ref2[2] << ")\n"
              << "[DEBUG][RotEdge] state.lastViewPose位置(切边末尾TCP) = ("
              << state.lastViewPose[0][3] << ", "
              << state.lastViewPose[1][3] << ", "
              << state.lastViewPose[2][3] << ")\n"
              << "[DEBUG][RotEdge] cfg.measTableAxisXYZ(转台旋转轴点) = ("
              << cfg.measTableAxisXYZ[0] << ", "
              << cfg.measTableAxisXYZ[1] << ", "
              << cfg.measTableAxisXYZ[2] << ")\n";

    std::vector<size_t> sorted_indices2 =
        saSort(shifted, ref2, cfg.randomSeed + 1, /*canonicalize=*/false);

    // ------------------------------------------------------------------
    // 4. 按排序后顺序提取原始坐标
    //
    //    重要语义修正（对照 MATLAB addMeasViewPoints.m）：
    //    MATLAB 中 rotateAngleDeg(=meas_rotate_theta=45) 是 TCP 局部 X 轴姿态
    //    旋转，作用在 T(1:3,1:3) 上、不动平移；而绕转台轴的整体点云旋转用的
    //    是 end_info.partRotateAngle（即末视点的转台角，0/90/180/270 等），
    //    不是 45。
    //
    //    在当前 main.cpp 流程里，转台角已通过 turnTable.setTurnTableAngle 写入
    //    场景，并经由 measurePointTrsf 把测点变换到正确的位置。BallScan 内部
    //    无需再做"绕转台轴整体旋转点云"的步骤，否则会把已经在正确位置的点
    //    再次绕远端的 measTableAxisXYZ 旋转 45°，导致 x 从 ~2000 跳到 ~5800。
    //
    //    因此这里仅按 SA2 顺序拷出测点，不再调用 rotatePathPointsAndNormals。
    //    45° 的作用挪到段 5 中，作为 LOCAL X 轴姿态旋转右乘到 tcp_pose 上。
    // ------------------------------------------------------------------
    std::vector<MeasurePoint> rot_pts;
    rot_pts.reserve(sorted_indices2.size());
    for (size_t si = 0; si < sorted_indices2.size(); ++si) {
        rot_pts.push_back(edgePoints[sorted_indices2[si]]);
    }

    // 预计算 LOCAL X 轴姿态旋转矩阵 Rx(rotateAngleDeg)
    // 对应 MATLAB addMeasViewPoints.m 第 223-227 行：
    //   theta = deg2rad(config.meas_rotate_theta);
    //   Rx = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
    //   T(1:3,1:3) = T(1:3,1:3) * Rx;
    // 含义：把相机姿态绕"它自己的 X 轴"倾斜 45°，平移完全不变。
    const double localRotRad = rotateAngleDeg * M_PI / 180.0;
    const double cosLocal    = std::cos(localRotRad);
    const double sinLocal    = std::sin(localRotRad);

    // ------------------------------------------------------------------
    // 5. 链式姿态生成 + 可达性检测（与 analyzeEdgePoints 段3 逻辑相同）
    // ------------------------------------------------------------------
    for (size_t si = 0; si < rot_pts.size(); ++si)
    {
        const MeasurePoint& mp = rot_pts[si];
        Vec3d z_axis = ViewpointPlanner::normalize3({ mp.i, mp.j, mp.k });
        Vec3d tcp_pos = {
            mp.x + cfg.measSafeHeight * z_axis[0],
            mp.y + cfg.measSafeHeight * z_axis[1],
            mp.z + cfg.measSafeHeight * z_axis[2]
        };

        Matrix4d tcp_pose;
        if (si == 0)
        {
            // 第一个测点：前向差分方向 + computeOptimalX
            // z=旋转后法向量，x=指向下一个旋转测点的方向，满足"前后点连线是x轴"要求
            Vec3d preset_x = { 1.0, 0.0, 0.0 };
            if (rot_pts.size() >= 2) {
                Vec3d n1 = ViewpointPlanner::normalize3(
                    { rot_pts[1].i, rot_pts[1].j, rot_pts[1].k });
                Vec3d p1 = {
                    rot_pts[1].x + cfg.measSafeHeight * n1[0],
                    rot_pts[1].y + cfg.measSafeHeight * n1[1],
                    rot_pts[1].z + cfg.measSafeHeight * n1[2]
                };
                Vec3d diff = { p1[0]-tcp_pos[0], p1[1]-tcp_pos[1], p1[2]-tcp_pos[2] };
                double nrm = std::sqrt(diff[0]*diff[0]+diff[1]*diff[1]+diff[2]*diff[2]);
                if (nrm > 1e-6) preset_x = { diff[0]/nrm, diff[1]/nrm, diff[2]/nrm };
            }
            Vec3d x_axis = planner.computeOptimalX(z_axis, preset_x);
            Vec3d y_axis = ViewpointPlanner::normalize3(
                ViewpointPlanner::cross3(z_axis, x_axis));
            x_axis = ViewpointPlanner::cross3(y_axis, z_axis);
            tcp_pose = eye4();
            for (int r = 0; r < 3; ++r) {
                tcp_pose[r][0] = x_axis[r];
                tcp_pose[r][1] = y_axis[r];
                tcp_pose[r][2] = z_axis[r];
                tcp_pose[r][3] = tcp_pos[r];
            }
        }
        else
        {
            // 后续测点：链式推导
            tcp_pose = planner.constructNextBestView(state.lastViewPose, z_axis);
            for (int r = 0; r < 3; ++r)
                tcp_pose[r][3] = tcp_pos[r];
        }

        // // 所有旋转切边点统一从上一个视点链式推导腕部姿态（已废弃，si==0位置会错误）
        // Matrix4d tcp_pose = planner.constructNextBestView(state.lastViewPose, z_axis);
        // for (int r = 0; r < 3; ++r)
        //     tcp_pose[r][3] = tcp_pos[r];

        // ------------------------------------------------------------------
        // 5.x  LOCAL X 轴姿态旋转（对应 MATLAB T(1:3,1:3) = T(1:3,1:3) * Rx）
        //
        //   Rx = [1   0     0  ]
        //        [0  cos  -sin ]
        //        [0  sin   cos ]
        //   作用：相机姿态绕"它自己的 X 轴"倾斜 rotateAngleDeg 度，
        //         平移 tcp_pose[*][3] 完全不变 —— 这是与原来错误实现的本质区别。
        //
        //   右乘的列变换公式：
        //     新 Y 列 = 原 Y 列 * cos + 原 Z 列 * sin
        //     新 Z 列 = -原 Y 列 * sin + 原 Z 列 * cos
        //     X 列、平移列不变
        // ------------------------------------------------------------------
        for (int r = 0; r < 3; ++r) {
            const double colY = tcp_pose[r][1];
            const double colZ = tcp_pose[r][2];
            tcp_pose[r][1] =  colY * cosLocal + colZ * sinLocal;
            tcp_pose[r][2] = -colY * sinLocal + colZ * cosLocal;
        }

        // 无论可达与否都更新
        state.lastViewPose = tcp_pose;

        // 可达性判断（前后关节角约束已在 robotReachability 内部恒定启用）
        JointVec6 out_joints;
        bool isReachable = robot_planner::robotReachability(
            tcp_pose, cfg, state.reachHistory, out_joints);

        double out_part_rotate = 0.0;
        if (!isReachable)
        {
            Matrix4d new_tcp;
            JointVec6 adj_joints;
            bool adjusted = planner.adjustViewPose(
                tcp_pose, state.reachHistory, new_tcp, adj_joints, &out_part_rotate);
            if (adjusted) {
                tcp_pose    = new_tcp;
                out_joints  = adj_joints;
                isReachable = true;
            } else {
                // 扰动后仍不可达：直接丢弃该点，不写入路径容器
                // （避免 NaN 占位点流入 processLineResult / PlanPathRRTconnect_new）
                std::cout << "[BallScan] 旋转切边点扰动仍不可达，已丢弃该点" << std::endl;
            }
        }

        // 构建结果
        MeasurePointPoseSet pose_set;
        pose_set.point_number    = mp.point_number;
        pose_set.name            = mp.name;
        pose_set.vct_vector_type = mp.vct_vectro_type;
        pose_set.x = mp.x;  pose_set.y = mp.y;  pose_set.z = mp.z;
        pose_set.i = mp.i;  pose_set.j = mp.j;  pose_set.k = mp.k;
        pose_set.i2 = mp.i2; pose_set.j2 = mp.j2; pose_set.k2 = mp.k2;
        pose_set.isMeasurePoint = true;

        if (isReachable)
        {
            PoseConfiguration pose_cfg;
            pose_cfg.point_name   = mp.name;
            pose_cfg.point_number = mp.point_number;
            pose_cfg.xyzwpr.x     = mp.x;
            pose_cfg.xyzwpr.y     = mp.y;
            pose_cfg.xyzwpr.z     = mp.z;
            pose_cfg.joints.j1    = out_joints[0];
            pose_cfg.joints.j2    = out_joints[1];
            pose_cfg.joints.j3    = out_joints[2];
            pose_cfg.joints.j4    = out_joints[3];
            pose_cfg.joints.j5    = out_joints[4];
            pose_cfg.joints.j6    = out_joints[5];
            pose_set.configurations.push_back(pose_cfg);
            pose_set.is_accessible = true;
            outReachable.push_back(pose_set);

            ViewPoint vp;
            vp.joints          = out_joints;
            vp.globalT         = tcp_pose;
            vp.isImportant     = true;
            vp.partRotateAngle = currentTurnTableAngle;
            vp.sourcePointId = mp.vct_vectro_type + "|" + mp.name + "|"
                + std::to_string(mp.point_number);
            vp.sourcePointPosition = {mp.x, mp.y, mp.z};
            vp.hasSourcePointPosition = true;
            state.reachHistory.push_back(vp);
            outEdgePath.push_back(vp);
            cfg.lastView = out_joints;
            state.lastJoints    = out_joints;
            state.hasLastJoints = true;
        }
        else
        {
            pose_set.is_accessible = false;
            outUnreachable.push_back(pose_set);
        }
    }

    std::cout << "[BallScan] 机器人 " << robotName
              << " 旋转切边点可达性：" << outReachable.size()
              << " 可达，" << outUnreachable.size()
              << " 不可达" << std::endl;
}

// =============================================================================
// analyzeEdgePointsCombined — 切边点 + 旋转切边点合并可达性分析
//
// 设计要点：
//   1) 输入 points 由调用方在进入转台循环前一次性拼接好，每个原始切边点会
//      产生两条 MeasurePoint：一条 name 与原始一致（normal 姿态），一条
//      name 追加 ROTATED_NAME_SUFFIX("_R")（旋转姿态）。两者的 xyz/ijk 完全相同。
//   2) 整个流程只调用一次 saSort —— 同名 normal/rotated 的 shifted.pos 相同，
//      SA 会把它们排成相邻，从而最小化机械臂腕部姿态在两种姿态间切换的代价。
//   3) 链式 TCP 构造完全沿用 analyzeEdgePoints 段3 的逻辑；唯一区别是：
//      构造完 tcp_pose 后，如果当前测点名字以 _R 结尾，则对 tcp_pose 右乘
//      Rx(rotateAngleDeg)（平移列不变，仅旋转列改变），等价于把相机姿态绕
//      自身 X 轴倾斜 rotateAngleDeg 度，公式与 analyzeEdgePointsRotated 段5.x 一致。
//   4) si==0 首点 preset_x 取下一个邻居差分方向时，若邻居 pos 与当前点重合
//      （同名 normal/rotated 必然如此），就继续向后扫描直到找到 pos 不同的
//      邻居；找不到则退回 {1,0,0}。这避免了差分退化为零向量的问题。
//   5) 可达 / 不可达 / 占位 VP 的写法与 analyzeEdgePoints 完全相同；
//      MeasurePointPoseSet.name 与输入 MeasurePoint.name 一致（含 _R 后缀），
//      由此构成"normal 与 rotated 是相互独立的两条记录"，分别独立判定、独立 carry。
// =============================================================================
void BallScan::analyzeEdgePointsCombined(
    const std::string& robotName,
    const std::vector<MeasurePoint>& points,
    double rotateAngleDeg,
    RobotConfig& cfg,
    ScanState& state,
    const Matrix4d& placeEndTcp,
    double currentTurnTableAngle,
    std::vector<MeasurePointPoseSet>& outReachable,
    std::vector<MeasurePointPoseSet>& outUnreachable,
    std::vector<ViewPoint>& outEdgePath)
{
    const size_t measN = points.size();
    if (measN == 0) return;

    // ------------------------------------------------------------------
    // 0. 后缀判定 lambda：name 是否以 ROTATED_NAME_SUFFIX 结尾
    //    → 决定是否对 tcp_pose 右乘 Rx(rotateAngleDeg)
    // ------------------------------------------------------------------
    const std::string rotSuffix = ROTATED_NAME_SUFFIX;
    auto isRotated = [&rotSuffix](const std::string& name) -> bool {
        return name.size() >= rotSuffix.size() &&
               std::equal(rotSuffix.rbegin(), rotSuffix.rend(), name.rbegin());
    };

    // ------------------------------------------------------------------
    // 1. 创建规划器（与 analyzeEdgePoints 段1 一致）
    // ------------------------------------------------------------------
    ViewpointPlanner planner(cfg);

    // ------------------------------------------------------------------
    // 2. SA 起点参考：上一视点（或过渡点）TCP 位置
    // ------------------------------------------------------------------
    Matrix4d end_tcp = placeEndTcp;
    Vec3d end_point = { end_tcp[0][3], end_tcp[1][3], end_tcp[2][3] };

    // ------------------------------------------------------------------
    // 3. 批量预处理：法向量归一化 + measSafeHeight 偏移 → shifted
    //    同名 normal / rotated 测点 xyz/ijk 完全一致，shifted 结果也完全一致。
    // ------------------------------------------------------------------
    std::vector<ShiftedPoint> shifted(measN);
    for (size_t i = 0; i < measN; ++i) {
        Vec3d n = ViewpointPlanner::normalize3(
            {points[i].i, points[i].j, points[i].k});
        shifted[i].normal = n;
        shifted[i].pos = {
            points[i].x + cfg.measSafeHeight * n[0],
            points[i].y + cfg.measSafeHeight * n[1],
            points[i].z + cfg.measSafeHeight * n[2]
        };
    }

    // ------------------------------------------------------------------
    // 4. 单次 SA 排序：每个转台角度只调用一次（共 4 个有效角度 → 4 次退火）
    //    同 pos 的 normal/rotated 会被排成相邻，符合期望。
    // ------------------------------------------------------------------
    std::vector<size_t> sorted_indices =
        saSort(shifted, end_point, cfg.randomSeed, /*canonicalize=*/true);

    // ------------------------------------------------------------------
    // 5. 预计算 LOCAL X 轴旋转 Rx(rotateAngleDeg)，作用于 _R 后缀测点的 tcp_pose
    //    公式：右乘 Rx 等价于
    //      新 Y 列 =  原 Y 列 * cos + 原 Z 列 * sin
    //      新 Z 列 = -原 Y 列 * sin + 原 Z 列 * cos
    //      X 列与平移列不变
    // ------------------------------------------------------------------
    const double localRotRad = rotateAngleDeg * M_PI / 180.0;
    const double cosLocal    = std::cos(localRotRad);
    const double sinLocal    = std::sin(localRotRad);

    // ------------------------------------------------------------------
    // 6. 链式姿态生成 + 可达性检测
    // ------------------------------------------------------------------
    for (size_t si = 0; si < sorted_indices.size(); ++si)
    {
        if (si == 50) {
            std::cout << "[BallScan] " << std::endl;
        }
        size_t idx = sorted_indices[si];
        const MeasurePoint& mp = points[idx];
        const Vec3d& z_axis  = shifted[idx].normal;
        const Vec3d& tcp_pos = shifted[idx].pos;
        const bool   rotated = isRotated(mp.name);

        Matrix4d tcp_pose;
        if (si == 0)
        {
            // 首点：preset_x 取下一个 pos 不同（>1e-6）的邻居差分方向
            // 跳过 pos 重合的同名 normal/rotated 邻居，避免差分退化
            Vec3d preset_x = {1.0, 0.0, 0.0};
            for (size_t sj = 1; sj < sorted_indices.size(); ++sj) {
                const Vec3d& next_pos = shifted[sorted_indices[sj]].pos;
                Vec3d diff = {
                    next_pos[0] - tcp_pos[0],
                    next_pos[1] - tcp_pos[1],
                    next_pos[2] - tcp_pos[2]
                };
                double nrm = std::sqrt(
                    diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);
                if (nrm > 1e-6) {
                    preset_x = { diff[0]/nrm, diff[1]/nrm, diff[2]/nrm };
                    break;
                }
            }
            Vec3d x_axis = planner.computeOptimalX(z_axis, preset_x);
            Vec3d y_axis = ViewpointPlanner::normalize3(
                ViewpointPlanner::cross3(z_axis, x_axis));
            x_axis = ViewpointPlanner::cross3(y_axis, z_axis);

            tcp_pose = eye4();
            for (int r = 0; r < 3; ++r) {
                tcp_pose[r][0] = x_axis[r];
                tcp_pose[r][1] = y_axis[r];
                tcp_pose[r][2] = z_axis[r];
                tcp_pose[r][3] = tcp_pos[r];
            }
        }
        else
        {
            // 后续测点：链式推导（最小旋转变换）+ 覆盖平移列
            tcp_pose = planner.constructNextBestView(state.lastViewPose, z_axis);
            for (int r = 0; r < 3; ++r)
                tcp_pose[r][3] = tcp_pos[r];
        }

        // ------------------------------------------------------------------
        // 6.x  _R 后缀测点：右乘 Rx(rotateAngleDeg)，腕部绕自身 X 轴倾斜
        //      （平移列不变，等价于 analyzeEdgePointsRotated 段5.x）
        // ------------------------------------------------------------------
        if (rotated) {
            for (int r = 0; r < 3; ++r) {
                const double colY = tcp_pose[r][1];
                const double colZ = tcp_pose[r][2];
                tcp_pose[r][1] =  colY * cosLocal + colZ * sinLocal;
                tcp_pose[r][2] = -colY * sinLocal + colZ * cosLocal;
            }
        }

        // 无论可达与否都更新（与 MATLAB 一致）
        state.lastViewPose = tcp_pose;

        // 可达性判断（前后关节角约束已在 robotReachability 内部恒定启用）
        JointVec6 out_joints;
        bool isReachable = robot_planner::robotReachability(
            tcp_pose, cfg, state.reachHistory, out_joints);

        double out_part_rotate = 0.0;
        if (!isReachable)
        {
            Matrix4d new_tcp;
            JointVec6 adj_joints;
            bool adjusted = planner.adjustViewPose(
                tcp_pose, state.reachHistory, new_tcp, adj_joints, &out_part_rotate);
            if (adjusted) {
                tcp_pose    = new_tcp;
                out_joints  = adj_joints;
                isReachable = true;
            } else {
                // 扰动后仍不可达：直接丢弃该点，不写入路径容器
                // （避免 NaN 占位点流入 processLineResult / PlanPathRRTconnect_new）
                std::cout << "[BallScan] " << (rotated ? "旋转切边点" : "切边点")
                          << "(" << mp.name << ") 扰动仍不可达，已丢弃该点" << std::endl;
            }
        }

        // ------------------------------------------------------------------
// 7. 构建结果（name 含 _R 后缀；normal / rotated 是相互独立的两条记录）
        // ------------------------------------------------------------------
        MeasurePointPoseSet pose_set;
        pose_set.point_number    = mp.point_number;
        pose_set.name            = mp.name;
        pose_set.vct_vector_type = mp.vct_vectro_type;
        pose_set.x = mp.x;  pose_set.y = mp.y;  pose_set.z = mp.z;
        pose_set.i = mp.i;  pose_set.j = mp.j;  pose_set.k = mp.k;
        pose_set.i2 = mp.i2; pose_set.j2 = mp.j2; pose_set.k2 = mp.k2;
        pose_set.isMeasurePoint = true;

        if (isReachable)
        {
            PoseConfiguration pose_cfg;
            pose_cfg.point_name   = mp.name;
            pose_cfg.point_number = mp.point_number;
            pose_cfg.xyzwpr.x     = mp.x;
            pose_cfg.xyzwpr.y     = mp.y;
            pose_cfg.xyzwpr.z     = mp.z;
            pose_cfg.joints.j1    = out_joints[0];
            pose_cfg.joints.j2    = out_joints[1];
            pose_cfg.joints.j3    = out_joints[2];
            pose_cfg.joints.j4    = out_joints[3];
            pose_cfg.joints.j5    = out_joints[4];
            pose_cfg.joints.j6    = out_joints[5];
            pose_set.configurations.push_back(pose_cfg);
            pose_set.is_accessible = true;
            outReachable.push_back(pose_set);

            ViewPoint vp;
            vp.joints          = out_joints;
            vp.globalT         = tcp_pose;
            vp.isImportant     = true;
            vp.partRotateAngle = currentTurnTableAngle;
            vp.sourcePointId = mp.vct_vectro_type + "|" + mp.name + "|"
                + std::to_string(mp.point_number);
            vp.sourcePointPosition = {mp.x, mp.y, mp.z};
            vp.hasSourcePointPosition = true;
            state.reachHistory.push_back(vp);
            outEdgePath.push_back(vp);
            cfg.lastView = out_joints;
            state.lastJoints    = out_joints;
            state.hasLastJoints = true;
        }
        else
        {
            pose_set.is_accessible = false;
            outUnreachable.push_back(pose_set);
        }
    }

    std::cout << "[BallScan] 机器人 " << robotName
              << " 合并可达性：" << outReachable.size()
              << " 可达，" << outUnreachable.size()
              << " 不可达（含 _R 旋转副本）" << std::endl;
}

// =============================================================================
// applyLinePathPredict — 对合并后的切边路径统一做直线轨迹预测
//
// 对应 MATLAB MeasLinePathPredict：逐段 linePathPredict，每段去掉末尾点，
// 最后追加原路径末尾点，保证首尾连续。
// 调用时机：切边点和旋转切边点的可达性分析全部完成，combinedEdgePath 组装完毕后。
// =============================================================================
void BallScan::applyLinePathPredict(
    std::vector<ViewPoint>& path,
    robot_planner::RobotConfig& cfg)
{
    if (path.size() < 2) return;

    ViewpointPlanner planner(cfg);

    std::vector<ViewPoint> predicted;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        auto seg = planner.linePathPredict(path[i], path[i + 1]);
        if (!seg.empty())
            predicted.insert(predicted.end(), seg.begin(), seg.end() - 1);
    }
    if (!predicted.empty())
        predicted.push_back(path.back());

    path = predicted;
}

// =============================================================================
// applyMeasLinePathPredict — 切边合并路径直线轨迹预测 + processLineResult
//
// 对应 MATLAB: 切边点和旋转切边点全部完成后，逐段调用 MeasLinePathPredict，
// 再统一调用 processLineResult 得到最终路径。
// 内部已调用 processLineResult，调用方无需再次调用。
// =============================================================================
void BallScan::applyMeasLinePathPredict(
    std::vector<ViewPoint>& path,
    robot_planner::RobotConfig& cfg)
{
    if (path.size() < 2) return;

    ViewpointPlanner planner(cfg);

    // [诊断计数] 记录入口点数，函数末尾与输出做类别拆分对比（不影响逻辑）
    const size_t kDiagInSize = path.size();
    // [诊断] 入口标记：能看到这行就证明新编译的代码确实在跑、函数被进入。
    std::cerr << "[applyMeasLinePathPredict][diag] ENTER robot=" << cfg.robotName
              << " in=" << kDiagInSize << std::endl;
    // [诊断] 清零可达性失败原因计数，仅统计本段路径
    robot_planner::reachReasonReset();

    std::vector<ViewPoint> predicted;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        if (i == 60) {
            std::cout << "i=60" << std::endl;
        }
        auto seg = planner.measLinePathPredict(path[i], path[i + 1]);
        if (!seg.empty())
            predicted.insert(predicted.end(), seg.begin(), seg.end() - 1);

        // 新增：seg.size() == 1 ⇔ measLinePathPredict 内部 twoOrientation
        // Interpolation 返回空 ⇔ path[i] 与 path[i+1] TCP 距离 <
        // cfg.minInterpolateDist。最常见来源是 analyzeEdgePointsCombined
        // 的 _R 后缀副本与 normal 测点 xyz/ijk 完全相同，但 _R 把 tcp_pose
        // 右乘 Rx(rotateAngleDeg) 让腕部绕自身 X 轴倾斜，IK 走完全不同的
        // 分支，关节解经常相差 100°+。这种"TCP 距离≈0、关节差极大"的相邻
        // 对若两点都走 MOVL，motosim 会被迫在零 TCP 距离内做笛卡尔直线
        // 插值 → 关节瞬间扫过大角度 → 编码器累加器爆表 → 脉冲极限错误。
        //
        // 解法：把 path[i+1] 改 MOVJ，让控制器走关节空间插值，规避 TCP
        // 直线约束。Yaskawa moveType 描述"机器人**走到**该点"的方式：
        //   · MOVL P_{i+1} = 从 P_i 笛卡尔直线插值到 P_{i+1}（脉冲极限处）
        //   · MOVJ P_{i+1} = 从 P_i 关节空间插值到 P_{i+1}（无 TCP 直线约束）
        // 所以改的是 path[i+1] 而不是 path[i]。
        //
        // isImportant 同步置 true，防止下游 spliceRRTIntoDensePath 末尾按
        // (isImportant || isPathJoint) 过滤时把它丢掉（虽然 analyzeEdge
        // PointsCombined 输出时已置 true，这里做防御性保留）。
        //
        // path 是非 const 引用，此修改对下一轮 iteration 可见：path[i+1]
        // 会成为下次 iteration 的 startVp，带着 moveType=0 进 predicted。
        //
        // 边界情况：
        //   1) i==0 就是近对：path[1] 改 MOVJ，path[0] 不动 ✓
        //   2) i+1==path.size()-1 末段近对：函数末尾 predicted.push_back
        //      (path.back()) 会把改过的 path[i+1] 带进 predicted ✓
        //   3) 连续多对都近 (P1≈P2≈P3≈P4)：每次 iteration 都触发，
        //      P2、P3、P4 全 MOVJ ✓
        //   4) 远距离正常段：seg.size() > 1，本 if 不触发，行为不变 ✓
        //
        // 同时救回 path[i]：现有 insert(seg.begin(), seg.end()-1) 在
        // seg.size()==1 时插入零元素，path[i] 本身会从 predicted 中消失。
        // 如果不补救，后果是所有 normal 测点全丢、predicted 里只剩 _R 副本
        // （且全部被改成 moveType=0），最终 JBI 表现为"连续 MOVJ 链"。
        // 这里手动 push_back(seg[0]) 把 P_i 救回来，使最终 JBI 保持
        // "MOVL P_i → MOVJ P_i_R → MOVL P_{i+1} → MOVJ P_{i+1}_R" 的交替
        // 结构（normal 走 MOVL，_R 走 MOVJ 规避脉冲极限）。
        if (seg.size() == 1) {
            // 新增（方案 A）：在 (path[i], path[i+1]) 之间插入 mt=-2 占位，
            // 让下游 spliceRRTIntoDensePath 对这对端点调 PathPlan RRT 做关节空间规划。
            // 触发链路（path_rrt_splice.cpp:221-272）:
            //   插入的 -2 → markAndCleanMinusTwo 删除 -2 并把左右第一个非 -2 邻居
            //   的 moveType 改 -1 → 构成 len=2 的 -1 run → 调 RRT(path[i], path[i+1])

            // 救回 path[i]:正常段 insert(begin, end-1) 在 seg.size()==1 时
            // begin==end-1,区间退化为空,插入 0 元素,path[i] 不会进 predicted。
            // 若不补 path[i],下面压入的 -2 占位会向左滑到上一段的尾插补点,
            // 把无关点错锚为 mt=-1,RRT run 的 L_idx 跑偏到错误起点。
            predicted.push_back(path[i]);

            // 压入 mt=-2 NaN 占位。字段约束按"必须性"分级:
            //   硬条件(触发 RRT + 防退化分支 NaN 泄漏):
            //     - moveType=-2     : markAndCleanMinusTwo 识别占位的唯一依据
            //     - isImportant=false / isPathJoint=false : 当 RobotTechParameters
            //       缺失走退化分支(本函数 RRT 拼接 if (paramIt == allParams.end())
            //       分支)时,占位点会被 (isImportant||isPathJoint) 过滤直接保留,
            //       NaN joints 流到 MOVL 安全验证 / JBI 导出 → 崩
            //   防御性(正常分支占位会被物理删除前不读,仍按 measLinePathPredict
            //   行 436-443 的模板填,保持语义一致):
            //     - joints=NaN  : 万一被误读,NaN 立即暴露问题,0 会被当成合法关节
            //     - globalT     : 任意有效 4x4 即可,但不能空 vector(下游维度断言)
            //     - partRotateAngle : 占位点概念上属于 path[i] 段
            static const double kNaN_rrtPlaceholder = std::numeric_limits<double>::quiet_NaN();
            ViewPoint rrtPlaceholder;
            rrtPlaceholder.globalT         = path[i + 1].globalT;
            rrtPlaceholder.joints          = {kNaN_rrtPlaceholder, kNaN_rrtPlaceholder,
                                              kNaN_rrtPlaceholder, kNaN_rrtPlaceholder,
                                              kNaN_rrtPlaceholder, kNaN_rrtPlaceholder};
            rrtPlaceholder.moveType        = -2;
            rrtPlaceholder.isImportant     = false;
            rrtPlaceholder.isPathJoint     = false;
            rrtPlaceholder.partRotateAngle = path[i].partRotateAngle;
            predicted.push_back(rrtPlaceholder);

            // 注:下面两行(已有)的 MOVJ 标记会被 markAndCleanMinusTwo 覆盖为 mt=-1。
            //   RRT 成功 → path[i+1] 在 spliceRRT 里重写为 mt=0 (MOVJ),等价旧方案
            //   RRT 失败 → path[i+1] 在 spliceRRT 里降级为 mt=1 (MOVL),旧方案
            //              规避的脉冲极限风险在这种边界场景回归(已知失败模式)
            // 保留这两行不动:RRT 失败兜底成 MOVL 时,行为至少回到方案 A 引入前的状态。
            path[i + 1].moveType    = 0;
            path[i + 1].isImportant = true;
        }
    }
    if (!predicted.empty())
        predicted.push_back(path.back());

    // -------------------------------------------------------------------------
    // RRT 拼接 + 稀疏化（新 path_rrt_splice.cpp 替代旧 processLineResult）
    //   · measLinePathPredict 已把 gap 两侧最近的可达点标 moveType=-1；
    //   · spliceRRTIntoDensePath 扫描连续 -1 run、调 PathPlan RRT 插中间点，
    //     最后只保留 isImportant=true 的点（原始 waypoint + gap 边界 +
    //     RRT 中间点），dense 的可达插补点（mt=1, isImportant=false）全丢——
    //     MOVL 控制器自己会做笛卡尔直线插补，没必要在 JBI 里写出来。
    // -------------------------------------------------------------------------
    auto& allParams = SceneDocument::instance().getAllRobotTechParameters();
    auto paramIt = allParams.find(cfg.robotName);
    if (paramIt == allParams.end()) {
        // RRT 上下文凑不齐：退化为"isImportant || isPathJoint"稀疏过滤，
        // 不做 RRT；与 spliceRRTIntoDensePath 末尾的过滤规则保持一致。
        // 同时把 mt=-2 占位点（NaN joints, 两个 flag 都 false）一并丢掉，
        // 防止 NaN 流到下游 MOVL 安全验证 / JBI 导出。
        std::cerr << "[applyMeasLinePathPredict] 未找到 RobotTechParameters("
                  << cfg.robotName << ")，跳过 RRT 拼接，按 dense 路径回退\n";
        std::vector<ViewPoint> sparse;
        for (const auto& vp : predicted)
            if (vp.isImportant || vp.isPathJoint) sparse.push_back(vp);
        path = sparse;
    } else {
        RRTContext rrtCtx = makeRRTContext(cfg, paramIt->second);
        path = spliceRRTIntoDensePath(predicted, rrtCtx);
    }

    // -------------------------------------------------------------------------
    // [诊断计数] 输出路径按"被谁保留"拆分类别，定位 in→out 翻倍来源。
    //   只读 path，不修改任何点，不影响下游逻辑。
    //   · impPathJoint : isImportant && isPathJoint
    //       —— measLinePathPredict 扰动成功点(VP.cpp:440) / RRT 端点·中间点
    //   · impOnly      : isImportant && !isPathJoint
    //       —— 原始 waypoint / 不可达前一个可达插值点(VP.cpp:424)
    //   · pathJointOnly: !isImportant && isPathJoint
    //       —— 转台切换 MOVJ 过渡点
    //   · nan          : joints 含 NaN —— mt=-2 占位漏过过滤（异常，应为 0）
    //   · other        : 两个 flag 都 false 却仍被保留（异常，应为 0）
    // -------------------------------------------------------------------------
    {
        // 标签用纯 ASCII，避免控制台 GBK 把中文显示成乱码。
        // impOnly        : isImportant && !isPathJoint  -> waypoint / 插值断点(VP.cpp:424)
        // pertub (mt==1) : isImportant &&  isPathJoint && moveType==1 -> 扰动成功点(VP.cpp:440)
        // rrtMid (mt==0) : isImportant &&  isPathJoint && moveType==0 -> RRT 中间点(makeRRTMidNode)
        // ipjOther       : isImportant &&  isPathJoint && 其它 moveType
        // pjOnly         : !isImportant && isPathJoint  -> 转台过渡点
        // nan / other    : 异常，应为 0
        size_t impOnly = 0, pertub = 0, rrtMid = 0, ipjOther = 0,
               pjOnly = 0, nanCnt = 0, other = 0;
        for (const auto& vp : path) {
            bool hasNaN = false;
            for (int k = 0; k < 6; ++k)
                if (std::isnan(vp.joints[k])) { hasNaN = true; break; }
            if (hasNaN)                                 ++nanCnt;
            else if (vp.isImportant && vp.isPathJoint) {
                if      (vp.moveType == 1)              ++pertub;
                else if (vp.moveType == 0)              ++rrtMid;
                else                                    ++ipjOther;
            }
            else if (vp.isImportant)                    ++impOnly;
            else if (vp.isPathJoint)                    ++pjOnly;
            else                                        ++other;
        }
        // 与 path_rrt_splice.cpp:187 同一个 std::cerr 流。
        std::cerr << "[applyMeasLinePathPredict][diag] robot=" << cfg.robotName
                  << " in=" << kDiagInSize
                  << " out=" << path.size()
                  << " | impOnly=" << impOnly
                  << " pertub(mt1)=" << pertub
                  << " rrtMid(mt0)=" << rrtMid
                  << " ipjOther=" << ipjOther
                  << " pjOnly=" << pjOnly
                  << " nan=" << nanCnt
                  << " other=" << other
                  << std::endl;
        // [诊断] 打印本段路径累计的可达性失败原因分布
        robot_planner::reachReasonDump(cfg.robotName.c_str());
    }

    /* ---------- 旧实现：processLineResult 调用（保留作对照，2026-05 注释停用）
    const ProcessLineCallbacks callbacks = makeProcessLineCallbacks(cfg);

    // 构造 RRT 上下文：从 SceneDocument 单例取本机器人完整 RobotTechParameters
    auto& allParams_legacy = SceneDocument::instance().getAllRobotTechParameters();
    auto paramIt_legacy = allParams_legacy.find(cfg.robotName);
    const RRTContext* rrtCtxPtr = nullptr;
    RRTContext rrtCtxStorage;
    if (paramIt_legacy != allParams_legacy.end()) {
        rrtCtxStorage = makeRRTContext(cfg, paramIt_legacy->second);
        rrtCtxPtr = &rrtCtxStorage;
    } else {
        std::cerr << "[applyMeasLinePathPredict] 警告：未找到机器人 "
                  << cfg.robotName << " 的 RobotTechParameters，跳过 RRT 回退" << std::endl;
    }

    // 使用默认选项（keepOnlyKeyNodesAtEnd=true），保持输出稀疏（仅关键节点）
    path = processLineResult(predicted, callbacks, {}, rrtCtxPtr);
    ---------- 旧实现结束 ---------- */

    // 对过滤后相邻 MOVL 段做安全性验证：
    // 采样段间笛卡尔路径，若任意中间点不可达，将右端点改为 MOVJ（moveType=0），
    // 避免 Motosim 自行插值时选不同 IK 分支导致关节超限。
    //for (size_t i = 0; i + 1 < path.size(); ++i) {
    //    if (path[i].moveType != 1 || path[i + 1].moveType != 1) continue;

    //    std::vector<ViewPoint> history;
    //    for (size_t h = 0; h <= i; ++h) {
    //        bool hasNaN = false;
    //        for (int k = 0; k < 6; ++k)
    //            if (std::isnan(path[h].joints[k])) { hasNaN = true; break; }
    //        if (!hasNaN) history.push_back(path[h]);
    //    }

    //    auto interPoses = robot_planner::twoOrientationInterpolation(
    //        path[i].globalT, path[i + 1].globalT, cfg);

    //    for (const auto& pose : interPoses) {
    //        robot_planner::JointVec6 dummy{};
    //        bool ok = robot_planner::robotReachability(pose, cfg, history, true, dummy);
    //        if (!ok) {
    //            path[i + 1].moveType = 0;  // 改为 MOVJ，保留测量点但避免超限
    //            break;
    //        }
    //    }
    //}
}

// =============================================================================
// 过渡点关节空间扰动（对应 MATLAB adjustViewPointsInJointSpace.m）
//
// 流程（与 MATLAB 一致）：
//   1) 对 orin_pose 调用 IK 取候选关节解
//   2) 在外层循环每轮生成一组 [0,10) 度的随机扰动，叠加到所有 IK 解上
//   3) 对每个扰动后的关节解做 FK→TCP（user 坐标系）→robotReachability 验证
//   4) 超过 try_threshold 仍失败：使用 lastViews 中最近一个有 joint 的视点姿态，
//      仅替换平移到 orin_pose 的位置，再做一次可达性兜底
// =============================================================================

// XYZ extrinsic Euler 提取（与 RobotReachability.cpp::xyzwpr_to_matrix4d 同约定：
// T = Rz(w) * Ry(p) * Rx(r)）
static std::array<double, 3> rotmToEulerXYZ_local(const Matrix4d& M)
{
    double sp = -M[2][0];
    if (sp > 1.0)  sp = 1.0;
    if (sp < -1.0) sp = -1.0;
    double p = std::asin(sp);
    double w, r;
    if (std::abs(std::cos(p)) > 1e-9) {
        w = std::atan2(M[1][0], M[0][0]);
        r = std::atan2(M[2][1], M[2][2]);
    } else {
        // 万向锁，约定 w=0
        w = 0.0;
        r = std::atan2(-M[1][2], M[1][1]);
    }
    return {w, p, r};
}

// XYZWPR → Matrix4d（与 RobotReachability.cpp::xyzwpr_to_matrix4d 完全一致）
static Matrix4d xyzwprToMatrix4d_local(const XYZWPR& xp)
{
    const double w = xp.w * M_PI / 180.0;
    const double p = xp.p * M_PI / 180.0;
    const double r = xp.r * M_PI / 180.0;
    const double cw = std::cos(w), sw = std::sin(w);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cr = std::cos(r), sr = std::sin(r);
    return {
        { cw*cp,  cw*sp*sr - sw*cr,  cw*sp*cr + sw*sr,  xp.x },
        { sw*cp,  sw*sp*sr + cw*cr,  sw*sp*cr - cw*sr,  xp.y },
        {   -sp,           cp*sr,             cp*cr,     xp.z },
        {   0.0,             0.0,               0.0,      1.0 }
    };
}

static Matrix4d mat4Multiply_local(const Matrix4d& A, const Matrix4d& B)
{
    Matrix4d C(4, std::vector<double>(4, 0.0));
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

static bool adjustViewPointsInJointSpace(
    const Matrix4d&               orin_pose,
    RobotConfig&                  cfg,
    const std::vector<ViewPoint>& lastViews,
    Matrix4d&                     outNewPose,
    JointVec6&                    outJoints)
{
    // ---------- 1) IK：取候选关节解 ----------
    auto eul = rotmToEulerXYZ_local(orin_pose);
    XYZWPR xyzwpr;
    xyzwpr.x = orin_pose[0][3];
    xyzwpr.y = orin_pose[1][3];
    xyzwpr.z = orin_pose[2][3];
    xyzwpr.w = eul[0] * 180.0 / M_PI;
    xyzwpr.p = eul[1] * 180.0 / M_PI;
    xyzwpr.r = eul[2] * 180.0 / M_PI;
    auto rawSols = KinematicsApi::instance().ik(cfg.robotName, xyzwpr);

    const Matrix4d T_he_mat = xyzwprToMatrix4d_local(cfg.T_he);

    // ---------- 2) 外层循环：每轮生成一组随机扰动 ----------
    std::mt19937 rng(cfg.randomSeed + 7919u);
    std::uniform_real_distribution<double> dist(0.0, 10.0);
    const int kTryThreshold = 200;

    for (int try_step = 1; try_step <= kTryThreshold && !rawSols.empty(); ++try_step) {
        std::array<double, 6> perturb{};
        for (int i = 0; i < 6; ++i) perturb[i] = dist(rng);

        // 对应 MATLAB: joint_new = orin_joint + (rand(6,1)*10)';
        for (const auto& sol : rawSols) {
            if (sol.size() < 6) continue;
            std::vector<double> jv(6);
            for (int i = 0; i < 6; ++i) jv[i] = sol[i] + perturb[i];

            auto fkRes = KinematicsApi::instance().fk(
                cfg.robotName, jv, LinkName::LINK_6);
            if (fkRes.ut.size() < 4) continue;
            Matrix4d fk_mat(4, std::vector<double>(4, 0.0));
            bool fkOk = true;
            for (int i = 0; i < 4 && fkOk; ++i) {
                if (fkRes.ut[i].size() < 4) { fkOk = false; break; }
                for (int j = 0; j < 4; ++j) fk_mat[i][j] = fkRes.ut[i][j];
            }
            if (!fkOk) continue;

            // curr_viewpoint = T_user * FK * T_he
            Matrix4d new_tcp = mat4Multiply_local(
                mat4Multiply_local(cfg.T_user, fk_mat), T_he_mat);

            JointVec6 verifyJoints{};
            if (robot_planner::robotReachability(
                    new_tcp, cfg, lastViews, verifyJoints))
            {
                outNewPose = new_tcp;
                outJoints  = verifyJoints;
                return true;
            }
        }
    }

    // ---------- 3) 兜底：上一个有 joint 的姿态 + orin_pose 的平移 ----------
    bool foundPrev = false;
    Matrix4d prev_T;
    for (int j = static_cast<int>(lastViews.size()) - 1; j >= 0; --j) {
        bool hasJoints = false;
        for (int k = 0; k < 6; ++k) {
            if (std::abs(lastViews[j].joints[k]) > 1e-9) { hasJoints = true; break; }
        }
        if (hasJoints && lastViews[j].globalT.size() >= 4) {
            prev_T = lastViews[j].globalT;
            foundPrev = true;
            break;
        }
    }
    if (!foundPrev) return false;

    Matrix4d inter_view = prev_T;
    inter_view[0][3] = orin_pose[0][3];
    inter_view[1][3] = orin_pose[1][3];
    inter_view[2][3] = orin_pose[2][3];

    JointVec6 verifyJoints{};
    if (robot_planner::robotReachability(
            inter_view, cfg, lastViews, verifyJoints))
    {
        outNewPose = inter_view;
        outJoints  = verifyJoints;
        return true;
    }
    return false;
}

// =============================================================================
// computeTransitionPose — 计算过渡点 TCP 姿态
//
// 以 lastViewPose 位置为基准，沿 mainNormal 方向抬起 shiftDistance，
// 构建过渡点 TCP 姿态，供下一个转台角度的 SA 排序使用（placeEndTcp）。
// 对应 MATLAB partMeasPointsViewPlanning.m 中"过渡点添加"段逻辑（约 488-535 行）。
//
// 三级回退（与 MATLAB 对齐）：
//   1) inter_pose 直接 robotReachability 可达 → 直接返回
//      对应 MATLAB 504 行 robotReachability(inter_view, ...)
//   2) 不可达 → adjustViewPose（Z 平移 × Z 旋转扰动搜索）
//      对应 MATLAB 518 行 adjustViewPosePlace(inter_view, ...)
//   3) 仍不可达 → adjustViewPointsInJointSpace（关节空间随机扰动）
//      对应 MATLAB 522 行 adjustViewPointsInJointSpace(inter_view, ...)
// =============================================================================
robot_planner::Matrix4d BallScan::computeTransitionPose(
    const Matrix4d&     lastViewPose,
    const Vec3d&        mainNormal,
    double              shiftDistance,
    RobotConfig&        cfg,
    const ScanState&    state,
    JointVec6*          outJoints)
{
    ViewpointPlanner planner(cfg);

    // 过渡点位姿构造：沿 mainNormal 抬起 shift_distance，姿态由
    // constructNextBestView 从 lastViewPose 旋转得到，再覆盖平移。
    // 对应 MATLAB 491-497 行。
    Vec3d n = ViewpointPlanner::normalize3(mainNormal);
    Vec3d before_point = { lastViewPose[0][3], lastViewPose[1][3], lastViewPose[2][3] };
    Vec3d inter_point = {
        before_point[0] + shiftDistance * n[0],
        before_point[1] + shiftDistance * n[1],
        before_point[2] + shiftDistance * n[2]
    };

    Matrix4d inter_pose = planner.constructNextBestView(lastViewPose, n);
    inter_pose[0][3] = inter_point[0];
    inter_pose[1][3] = inter_point[1];
    inter_pose[2][3] = inter_point[2];

    // 一级：原始 inter_pose 可达性
    JointVec6 j1;
    if (robot_planner::robotReachability(
            inter_pose, cfg, state.reachHistory, j1))
    {
        if (outJoints) *outJoints = j1;
        return inter_pose;
    }

    // 二级：Z 平移 × Z 旋转扰动搜索（对应 MATLAB adjustViewPosePlace）
    Matrix4d adj_pose;
    JointVec6 adj_joints;
    if (planner.adjustViewPose(inter_pose, state.reachHistory, adj_pose, adj_joints))
    {
        if (outJoints) *outJoints = adj_joints;
        return adj_pose;
    }

    // 三级：关节空间随机扰动 + 兜底姿态（对应 MATLAB adjustViewPointsInJointSpace）
    Matrix4d js_pose;
    JointVec6 js_joints;
    if (adjustViewPointsInJointSpace(
            inter_pose, cfg, state.reachHistory, js_pose, js_joints))
    {
        if (outJoints) *outJoints = js_joints;
        return js_pose;
    }

    // 三级仍失败：返回 lastViewPose 作为最终兜底，避免污染下一轮 SA 起点参考
    if (outJoints) *outJoints = state.lastJoints;
    return lastViewPose;
}

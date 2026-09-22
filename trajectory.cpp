#include "trajectory.h"
#include <cmath>
#include <stdexcept>

double calculateTime(const Joint6& start,
    const Joint6& goal,
    int L,
    int motionSign)
{
    (void)motionSign;

    Joint6 v{};
    Joint6 amax{};

    if (L == 1 || L == 3 || L == 4 || L == 6) {
        v = { 160, 120, 120, 225, 225, 225 };
        amax = { 157.4992, 161.4016, 183.0406, 344.7656, 356.0938, 520.3219 };
    }
    else {
        v = { 110, 110, 110, 170, 170, 260 };
        amax = { 157.4992, 161.4016, 183.0406, 344.7656, 356.0938, 520.3219 };
    }

    Joint6 S_max{};
    Joint6 delta{};
    Joint6 diff{};

    for (int i = 0; i < 6; ++i) {
        S_max[i] = std::abs(amax[i] * (0.8 * 0.8) / 2.0);
        delta[i] = std::abs(start[i] - goal[i]) / 2.0;
        diff[i] = delta[i] - S_max[i];
    }

    int count_positive = 0;
    for (int i = 0; i < 6; ++i) {
        if (diff[i] > 0.0) {
            ++count_positive;
        }
    }

    if (count_positive == 0) {
        double max_t = 0.0;
        for (int i = 0; i < 6; ++i) {
            double t = std::sqrt(2.0 * delta[i] / amax[i]);
            if (t > max_t) {
                max_t = t;
            }
        }
        return 2.0 * max_t;
    }
    else {
        double t0 = 1.6;
        double max_t1 = 0.0;
        bool has_valid = false;

        for (int i = 0; i < 6; ++i) {
            double S0 = 2.0 * delta[i] - amax[i] * (0.8 * 0.8);
            if (S0 > 0.0) {
                double inside = 2.0 * (S0 + (v[i] * v[i]) / (2.0 * amax[i])) / amax[i];
                if (inside < 0.0) {
                    throw std::runtime_error("Negative value inside sqrt in calculateTime.");
                }
                double t1 = std::sqrt(inside) - v[i] / amax[i];
                if (!has_valid || t1 > max_t1) {
                    max_t1 = t1;
                    has_valid = true;
                }
            }
        }

        if (!has_valid) {
            throw std::runtime_error("No valid joint found in calculateTime.");
        }

        return max_t1 + t0;
    }
}

Joint6 multinomial(const Joint6& start,
    const Joint6& goal,
    double t,
    double T)
{
    if (T <= 0.0) {
        throw std::runtime_error("T must be positive.");
    }

    Joint6 result{};

    for (int i = 0; i < 6; ++i) {
        double a0 = start[i];
        double a1 = 0.0;
        double a2 = 0.0;
        double dq = goal[i] - start[i];
        double a3 = 10.0 * dq / std::pow(T, 3);
        double a4 = -15.0 * dq / std::pow(T, 4);
        double a5 = 6.0 * dq / std::pow(T, 5);

        result[i] = a0
            + a1 * t
            + a2 * t * t
            + a3 * std::pow(t, 3)
            + a4 * std::pow(t, 4)
            + a5 * std::pow(t, 5);
    }

    return result;
}

TrajectoryResult generateJointTrajectory(const Joint6& start,
    const Joint6& goal,
    int L,
    int motionSign,
    double step)
{
    if (step <= 0.0) {
        throw std::runtime_error("step must be positive.");
    }

    TrajectoryResult output{};
    output.T = calculateTime(start, goal, L, motionSign);

    for (double tt = 0.0; tt <= output.T; tt += step) {
        output.time_list.push_back(tt);
    }

    if (output.time_list.empty() || std::abs(output.time_list.back() - output.T) > 1e-9) {
        output.time_list.push_back(output.T);
    }

    for (double tt : output.time_list) {
        Joint6 q = multinomial(start, goal, tt, output.T);
        output.trajectory.push_back(q);
    }

    return output;
}
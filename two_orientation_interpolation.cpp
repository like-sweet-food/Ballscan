#include "two_orientation_interpolation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace robot_planner {

namespace {

using Quaternion = std::array<double, 4>; // [w, x, y, z]

void validateTransform(const Matrix4d& T, const char* name)
{
    if (T.size() < 4) {
        throw std::runtime_error(std::string(name) + ": row size < 4");
    }
    for (int r = 0; r < 4; ++r) {
        if (T[r].size() < 4) {
            throw std::runtime_error(std::string(name) + ": col size < 4");
        }
    }
}

Vec3d getPosition(const Matrix4d& T)
{
    return { T[0][3], T[1][3], T[2][3] };
}

double norm3(const Vec3d& v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

Quaternion normalizeQuat(const Quaternion& q)
{
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n <= 1e-12) {
        throw std::runtime_error("twoOrientationInterpolation: zero quaternion");
    }
    return { q[0] / n, q[1] / n, q[2] / n, q[3] / n };
}

double dotQuat(const Quaternion& a, const Quaternion& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

Quaternion rotmToQuat(const Matrix4d& T)
{
    const double r00 = T[0][0], r01 = T[0][1], r02 = T[0][2];
    const double r10 = T[1][0], r11 = T[1][1], r12 = T[1][2];
    const double r20 = T[2][0], r21 = T[2][1], r22 = T[2][2];

    Quaternion q{};
    const double trace = r00 + r11 + r22;
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q[0] = 0.25 * s;
        q[1] = (r21 - r12) / s;
        q[2] = (r02 - r20) / s;
        q[3] = (r10 - r01) / s;
    }
    else if (r00 > r11 && r00 > r22) {
        const double s = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
        q[0] = (r21 - r12) / s;
        q[1] = 0.25 * s;
        q[2] = (r01 + r10) / s;
        q[3] = (r02 + r20) / s;
    }
    else if (r11 > r22) {
        const double s = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
        q[0] = (r02 - r20) / s;
        q[1] = (r01 + r10) / s;
        q[2] = 0.25 * s;
        q[3] = (r12 + r21) / s;
    }
    else {
        const double s = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
        q[0] = (r10 - r01) / s;
        q[1] = (r02 + r20) / s;
        q[2] = (r12 + r21) / s;
        q[3] = 0.25 * s;
    }

    return normalizeQuat(q);
}

Matrix4d quatToRotm(const Quaternion& qIn)
{
    const Quaternion q = normalizeQuat(qIn);
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];

    Matrix4d T = eye4();
    T[0][0] = 1.0 - 2.0 * (y * y + z * z);
    T[0][1] = 2.0 * (x * y - z * w);
    T[0][2] = 2.0 * (x * z + y * w);
    T[1][0] = 2.0 * (x * y + z * w);
    T[1][1] = 1.0 - 2.0 * (x * x + z * z);
    T[1][2] = 2.0 * (y * z - x * w);
    T[2][0] = 2.0 * (x * z - y * w);
    T[2][1] = 2.0 * (y * z + x * w);
    T[2][2] = 1.0 - 2.0 * (x * x + y * y);
    return T;
}

} // namespace

std::vector<Matrix4d> twoOrientationInterpolation(
    const Matrix4d& Ts,
    const Matrix4d& Tg,
    const RobotConfig& config)
{
    validateTransform(Ts, "Ts");
    validateTransform(Tg, "Tg");

    const Vec3d ps = getPosition(Ts);
    const Vec3d pg = getPosition(Tg);
    const Vec3d diff = { ps[0] - pg[0], ps[1] - pg[1], ps[2] - pg[2] };
    const double dist = norm3(diff);

    const double stepMm = config.minInterpolateDist;
    const int N = static_cast<int>(std::floor(dist / stepMm));
    if (N <= 0) {
        return {};
    }

    const Quaternion Qs = rotmToQuat(Ts);
    Quaternion Qg = rotmToQuat(Tg);

    if (dotQuat(Qs, Qg) < 0.0) {
        for (double& value : Qg) {
            value = -value;
        }
    }

    double c = dotQuat(Qs, Qg);
    c = std::max(-1.0, std::min(1.0, c));
    const double theta = std::acos(c);
    const double step = 1.0 / static_cast<double>(N + 1);
    const bool useNlerp = std::abs(std::sin(theta)) < std::sin(5.0 * 3.14159265358979323846 / 180.0);

    std::vector<Matrix4d> sequence;
    sequence.reserve(N);

    for (int i = 1; i <= N; ++i) {
        const double t = step * static_cast<double>(i);

        Quaternion Q{};
        if (useNlerp) {
            Q = {
                (1.0 - t) * Qs[0] + t * Qg[0],
                (1.0 - t) * Qs[1] + t * Qg[1],
                (1.0 - t) * Qs[2] + t * Qg[2],
                (1.0 - t) * Qs[3] + t * Qg[3]
            };
            Q = normalizeQuat(Q);
        }
        else {
            const double sinTheta = std::sin(theta);
            Q = {
                (std::sin((1.0 - t) * theta) * Qs[0] + std::sin(t * theta) * Qg[0]) / sinTheta,
                (std::sin((1.0 - t) * theta) * Qs[1] + std::sin(t * theta) * Qg[1]) / sinTheta,
                (std::sin((1.0 - t) * theta) * Qs[2] + std::sin(t * theta) * Qg[2]) / sinTheta,
                (std::sin((1.0 - t) * theta) * Qs[3] + std::sin(t * theta) * Qg[3]) / sinTheta
            };
            Q = normalizeQuat(Q);
        }

        Matrix4d T = quatToRotm(Q);
        T[0][3] = (1.0 - t) * ps[0] + t * pg[0];
        T[1][3] = (1.0 - t) * ps[1] + t * pg[1];
        T[2][3] = (1.0 - t) * ps[2] + t * pg[2];
        sequence.push_back(std::move(T));
    }

    return sequence;
}

} // namespace robot_planner

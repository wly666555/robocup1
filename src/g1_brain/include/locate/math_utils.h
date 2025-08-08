#pragma once

#include <cmath>
#include <vector>
#include "types.h"
#include <iostream>

using namespace std;

template <typename T>//
using Vec3 = typename Eigen::Matrix<T, 3, 1>;//

template <typename T>//
using RotMat = typename Eigen::Matrix<T, 3, 3>;//

// degrees to radians
inline double deg2rad(double deg)
{
    return deg / 180.0 * M_PI;
}

// radians to degrees
inline double rad2deg(double rad)
{
    return rad / M_PI * 180.0;
}

// arithmetic mean
inline double mean(double x, double y)
{
    return (x + y) / 2;
}

// truncate the number to a range
inline double cap(double x, double upper_limit, double lower_limit)
{
    return max(min(x, upper_limit), lower_limit);
}

// Calculate the L2 norm (the square root of the sum of the squares of two numbers).
inline double norm(double x, double y)
{
    return sqrt(x * x + y * y);
}

// Calculate the L2 norm (the square root of the sum of the squares of two numbers).
inline double norm(vector<double> v)
{
    return sqrt(v[0] * v[0] + v[1] * v[1]);
}

// Convert an angle to the range of [-M_PI, M_PI).
inline double toPInPI(double theta)
{
    int n = static_cast<int>(fabs(theta / 2 / M_PI)) + 1;
    return fmod(theta + M_PI + 2 * n * M_PI, 2 * M_PI) - M_PI;
}

// In any Cartesian coordinate system, calculate the angle θ (in radians) between a vector v 
// and the x-axis, with the range (-M_PI, M_PI).
inline double thetaToX(vector<double> v)
{
    vector<double> x = {1, 0};
    double ang = atan2(v[1], v[0]);
    return toPInPI(ang);
}

// Transform a point from coordinate system 0 to coordinate system 1, where coordinate system 1 
// is rotated by an angle θ relative to coordinate system 0.
inline Point2D transform(Point2D p0, double theta)
{
    Point2D p1;
    p1.x = p0.x * cos(theta) + p0.y * sin(theta);
    p1.y = -p0.x * sin(theta) + p0.y * cos(theta);
    return p1;
}

/**
 * @brief Transform a Pose (xs, ys, thetas) from source coordinate system (s) to target coordinate system (t).
 *        The source coordinate system's origin (xst, yst, thetast) is represented in the target coordinate system.
 *
 * @param xs, ys, thetas Pose (position and orientation) in the source coordinate system (s), with theta in radians.
 * @param xst, yst, thetast Position and orientation of the source coordinate system's origin in the target coordinate system (t), with theta in radians.
 * @param xt, yt, thetat Output the Pose (position and orientation) in the target coordinate system (t), with theta in radians.
 */

inline void transCoord(const double &xs, const double &ys, const double &thetas, const double &xst, const double &yst, const double &thetast, double &xt, double &yt, double &thetat)
{
    thetat = toPInPI(thetas + thetast);
    xt = xst + xs * cos(thetast) - ys * sin(thetast);
    yt = yst + xs * sin(thetast) + ys * cos(thetast);
}
inline RotMat<double> quatToRotMat(const Quat<double> &q)//
{
    double e0 = q(0);
    double e1 = q(1);
    double e2 = q(2);
    double e3 = q(3);

    RotMat<double> R;
    R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
        2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
        1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
        2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
        1 - 2 * (e1 * e1 + e2 * e2);
    return R;
}
inline Vec3<double> homoVec(const Vec2<double>& v2)//
{
    Vec3<double> v3;
    v3.block(0, 0, 2, 1) = v2;
    v3(2) = 1;
    return v3;
}

inline Vec2<double> dehomoVec(const Vec3<double>& v3)//
{
    return Vec2<double>(v3(0), v3(1));
}

inline RotMat<double> rotx(const double &theta)//
{
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat<double> R;
    R << 1, 0, 0, 0, c, -s, 0, s, c;
    return R;
}

inline RotMat<double> roty(const double &theta)//
{
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat<double> R;
    R << c, 0, s, 0, 1, 0, -s, 0, c;
    return R;
}

inline RotMat<double> rotz(const double &theta)//
{
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat<double> R;
    R << c, -s, 0, s, c, 0, 0, 0, 1;
    return R;
}
inline RotMat<double> rpyToRotMat(const double &row, const double &pitch, const double &yaw)//
{
    // RotMat<double> m = rotz(yaw) * roty(pitch) * rotx(row);
    RotMat<double> m =rotx(row) *  roty(pitch) * rotz(yaw);
    return m;
}
inline Vec3<double> rotMatToRPY(const Mat3<double> &R)//
{
    Vec3<double> rpy;
    rpy(0) = atan2(R(2, 1), R(2, 2));
    rpy(1) = asin(-R(2, 0));
    rpy(2) = atan2(R(1, 0), R(0, 0));
    return rpy;
}
inline HomoMat2<double> homoMatrix(RotMat2<double> m, Vec2<double> p)//
{
    HomoMat2<double> homoM;
    homoM.setZero();
    homoM.topLeftCorner(2, 2) = m;
    homoM.topRightCorner(2, 1) = p;
    homoM(2, 2) = 1;
    return homoM;
}
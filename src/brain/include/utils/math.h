#pragma once

#include <cmath>
#include <vector>
#include "../types.h"

using namespace std;

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
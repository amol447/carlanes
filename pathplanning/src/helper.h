//
// Created by amol on 21/05/18.
//

#ifndef PATH_PLANNING_HELPER_H
#define PATH_PLANNING_HELPER_H
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
double deg2rad(double );
double rad2deg(double );
constexpr double pi(){ return M_PI; }
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
enum Lane {
        LEFT, CENTRE, RIGHT
    };
struct angleInRadians;
struct angleInDegrees{
    double angle;
    explicit angleInDegrees(double x):angle{x}{}
    angleInDegrees(const angleInRadians& x);
};
struct angleInRadians{
    double angle;
    explicit angleInRadians(double x):angle{x}{}
    angleInRadians(const angleInDegrees& x);
};
double lane2d(Lane l);

#endif //PATH_PLANNING_HELPER_H

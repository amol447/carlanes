//
// Created by amol on 21/05/18.
//

#ifndef PATH_PLANNING_HELPER_H
#define PATH_PLANNING_HELPER_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
double deg2rad(double );
double rad2deg(double );
double distance(double x1, double y1, double x2, double y2);
constexpr double pi(){ return M_PI; }
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
enum Lane {
        LEFT, CENTRE, RIGHT
    };
Lane d2Lane(double );
struct AngleInRadians;
struct AngleInDegrees{
    double angle;
    explicit AngleInDegrees(double x):angle{x}{}
    AngleInDegrees(const AngleInRadians& x);
};

struct AngleInRadians{
    double angle;
    explicit AngleInRadians(double x):angle{x}{}
    AngleInRadians(const AngleInDegrees& x);
};

AngleInRadians operator+(AngleInRadians x, AngleInRadians y);
double lane2d(Lane l);

struct FrenetPoint{
    double s;
    double d;
    explicit FrenetPoint(double,double);
};

struct CartesianPoint{
    double x;
    double y;
    explicit CartesianPoint(double, double);
    CartesianPoint();
};




struct CarStateCartesian{
    CartesianPoint car_position;
    AngleInRadians car_angle;
    double car_speed_in_mps;
    CarStateCartesian(double x,double y, AngleInRadians theta,double speed);
    CarStateCartesian(const CarStateCartesian &x);
};

CartesianPoint rotation_translation(const CartesianPoint & point, const CartesianPoint& ref, const AngleInRadians& ref_angle);
CartesianPoint inverse_rotation_translation(const CartesianPoint & point, const CartesianPoint & ref, const AngleInRadians & ref_angle);
FrenetPoint getFrenet(CartesianPoint p, AngleInRadians theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
CartesianPoint getXY(FrenetPoint p,const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
CarStateCartesian moveForward(CarStateCartesian start_state, double time_in_sec);
std::vector<CartesianPoint> calcPathSpline(const std::vector<double>&, const std::vector<double>&, CarStateCartesian, FrenetPoint,
                                           const Lane, const double,const std::vector<double> &, const std::vector<double> &,
                                           const std::vector<double> &);
std::vector<CartesianPoint> calcFullPathSpline(const std::vector<double>&, const std::vector<double>&, CarStateCartesian, FrenetPoint,
                                           const Lane, const double,const std::vector<double> &, const std::vector<double> &,
                                           const std::vector<double> &);

bool collisionDetection(const std::vector<CartesianPoint>& trajectory,const std::vector<std::vector<double>> & other_cars);
double distance(CartesianPoint const & ,CartesianPoint const &);
#endif //PATH_PLANNING_HELPER_H

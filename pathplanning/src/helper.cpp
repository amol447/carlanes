//
// Created by amol on 23/05/18.
//
#include <functional>
#include <algorithm>
#include "helper.h"
#include <math.h>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"

double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double distance(CartesianPoint const &a, CartesianPoint const &b){
    return distance(a.x,a.y,b.x,b.y);
}

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
    angle = std::min(2*pi() - angle, angle);

    if(angle > pi()/4)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}
// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}

double lane2d(Lane l){
    return 2+l*4;
}

Lane d2Lane(double d){
    if(d<=4) return LEFT;
    else if (d<=8) return CENTRE;
    else return RIGHT;
}
AngleInDegrees::AngleInDegrees(const AngleInRadians &x) {
    angle=rad2deg(x.angle);
}
AngleInRadians::AngleInRadians(const AngleInDegrees &x) {
    angle = deg2rad(x.angle);
}
double sindeg(AngleInDegrees a){
    return sin(AngleInRadians(a).angle);
}
double cosdeg(AngleInDegrees a){
    return cos(AngleInRadians(a).angle);
}

double sinrad(AngleInRadians a){
    return sin(a.angle);
}
double cosrad(AngleInRadians a){
    return cos(a.angle);
}

FrenetPoint::FrenetPoint(double _s, double _d) {
    this->s = _s;
    this->d = _d;
}

CartesianPoint::CartesianPoint(double _x, double _y) {
    x = _x;
    y = _y;
}

CarStateCartesian::CarStateCartesian(double x, double y, AngleInRadians theta, double speed):car_position{CartesianPoint(x,y)},car_angle{theta} {
    car_speed_in_mps = speed;
}

CarStateCartesian::CarStateCartesian(const CarStateCartesian &x):car_angle {x.car_angle},car_position{x.car_position}{
    car_speed_in_mps = x.car_speed_in_mps;
}

FrenetPoint getFrenet(CartesianPoint p , AngleInRadians theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y){
    auto temp = getFrenet(p.x,p.y,theta.angle,maps_x,maps_y);
    return FrenetPoint(temp[0],temp[1]);
}

CartesianPoint getXY(FrenetPoint p, const std::vector<double> &maps_s, const std::vector<double> &maps_x,const std::vector<double> &maps_y){
    auto temp = getXY(p.s,p.d,maps_s,maps_x,maps_y);
    return CartesianPoint(temp[0],temp[1]);
}


CarStateCartesian moveForward(CarStateCartesian start_state, double time_in_sec){
    CarStateCartesian result(start_state);
    result.car_position.x = start_state.car_position.x + start_state.car_speed_in_mps*cosrad(start_state.car_angle)*time_in_sec;
    result.car_position.y = start_state.car_position.y + start_state.car_speed_in_mps*sinrad(start_state.car_angle)*time_in_sec;
    return result;
}

FrenetPoint add_to_s_set_d(const FrenetPoint& x, double s,double d){
    return FrenetPoint(x.s+s,d);
}
FrenetPoint add_to_s(const FrenetPoint&x, double s){
    return FrenetPoint(x.s+s,x.d);
}

AngleInRadians operator+(AngleInRadians x, AngleInRadians y){
    return AngleInRadians(x.angle+y.angle);
}
AngleInRadians operator-(AngleInRadians x, AngleInRadians y){
    return  AngleInRadians(x.angle-y.angle);
}

CartesianPoint::CartesianPoint() {
    x=0.0;
    y=0.0;
}
CartesianPoint rotation_translation(const CartesianPoint &point, const CartesianPoint &ref, const AngleInRadians &ref_angle) {
    CartesianPoint result(point);
    double shift_x = point.x - ref.x;
    double shift_y = point.y- ref.y;
    result.x = shift_x*cosrad(AngleInRadians(0.0)-ref_angle) - shift_y*sinrad(AngleInRadians(0.0)-ref_angle);
    result.y = shift_x*sinrad(AngleInRadians(0.0)-ref_angle) + shift_y*cosrad(AngleInRadians(0.0)-ref_angle);
    return result;
}

CartesianPoint inverse_rotation_translation(const CartesianPoint & point, const CartesianPoint & ref, const AngleInRadians & ref_angle){
    CartesianPoint result(point);
    result.x = ref.x + point.x*cosrad(ref_angle) - point.y*sinrad(ref_angle);
    result.y = ref.y + point.x*sinrad(ref_angle) + point.y*cosrad(ref_angle);
    return result;
}
std::vector<CartesianPoint> calcPathSpline(const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y,
                                           CarStateCartesian car_state, FrenetPoint frenet_state, const Lane desired_lane,const double target_speed,
                                           const std::vector<double> &maps_s, const std::vector<double> &maps_x,
                                           const std::vector<double> &maps_y){
    const unsigned int desired_points = 50;
    std::vector<CartesianPoint> spline_init_points;
    CartesianPoint ref=car_state.car_position;
    AngleInRadians ref_yaw = car_state.car_angle;
    double ref_speed = 0.0;
    const double max_acceleration = 10.0;
    const unsigned int n = previous_path_x.size();
    if (previous_path_x.size() >= 2) {
        ref = CartesianPoint(previous_path_x[n-1],previous_path_y[n-1]);
        spline_init_points.push_back(ref);
        spline_init_points.emplace_back(CartesianPoint(previous_path_x[n-2],previous_path_y[n-2]));
        ref_yaw = AngleInRadians(atan2(previous_path_y[n-1]-previous_path_y[n-2],previous_path_x[n-1]-previous_path_x[n-2]));
        ref_speed = fabs(distance(previous_path_x[n-1],previous_path_y[n-1],previous_path_x[n-2],previous_path_y[n-2])/0.02);
        ref_speed = std::max(ref_speed-max_acceleration*0.05,std::min(target_speed,ref_speed+ max_acceleration*0.05));
        } else {
        spline_init_points.push_back(car_state.car_position);
        if(car_state.car_speed_in_mps>0){
            spline_init_points.push_back(moveForward(car_state, -0.02).car_position);
        }else{
            CarStateCartesian new_state(car_state);
            new_state.car_speed_in_mps = 0.2;
            spline_init_points.push_back(moveForward(new_state,-0.02).car_position);
        }
        ref_speed = std::max(car_state.car_speed_in_mps-max_acceleration*0.05,std::min(car_state.car_speed_in_mps+max_acceleration*0.05,target_speed));
    }
    std::vector<CartesianPoint> next_wp(3);
    const double FORWARD_TIME=1.2;
    double forward_s=std::max(50.0,FORWARD_TIME*car_state.car_speed_in_mps);
    double desired_d = lane2d(desired_lane);
    for(unsigned long i=0;i<3;i++){
    next_wp[i] = getXY(add_to_s_set_d(frenet_state,forward_s*(i+1),desired_d),maps_s,maps_x, maps_y);
    spline_init_points.push_back(next_wp[i]);
    }
    tk::spline s;
    std::transform(spline_init_points.begin(),spline_init_points.end(),spline_init_points.begin(),[&ref,ref_yaw](CartesianPoint point){return rotation_translation(point,ref,ref_yaw); });
    std::sort(spline_init_points.begin(),spline_init_points.end(),[](CartesianPoint x,CartesianPoint y){return (x.x<=y.x);});
    std::vector<double> wp_x,wp_y;
    std::transform(spline_init_points.begin(),spline_init_points.end(),std::back_inserter(wp_x),[](CartesianPoint x){return x.x;});
    std::transform(spline_init_points.begin(),spline_init_points.end(),std::back_inserter(wp_y),[](CartesianPoint x){return x.y;});
    s.set_points(wp_x,wp_y);

    //get points on spline that do not violate speed constraints
    //it's ok to do this in transformed domain because the transformation doesn't scale, only translate and rotate
    std::vector<CartesianPoint> result(desired_points);
    for(unsigned int i=0;i<n;i++){
        result[i] = CartesianPoint(previous_path_x[i],previous_path_y[i]);
    }
    double target_x = std::max(50.0,FORWARD_TIME*car_state.car_speed_in_mps);
    double target_y = s(target_x);
    double distance_to_end = distance(0.0,0.0,target_x,target_y);
    AngleInRadians angle_of_triangle = AngleInRadians(atan2(target_y,target_x));
    int num_points = desired_points - n;
    if(num_points<=0){
        std::cout<< "something is not right"<<std::endl;
    }
    //std::cout<<"ref_speed="<<ref_speed<<std::endl;
    int spline_total_points = (unsigned  int) ceil(distance_to_end*50.0/ref_speed);
    double spline_x,spline_y;
    for(unsigned int i=0;i <num_points;i++){
        spline_x=(i+1)*cosrad(angle_of_triangle)*distance_to_end/spline_total_points;
        spline_y=s(spline_x);
        CartesianPoint p(spline_x,spline_y);
        result[n+i] = inverse_rotation_translation(p,ref,ref_yaw);
    }
return result;

}
std::vector<CartesianPoint> calcFullPathSpline(const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y,
                                           CarStateCartesian car_state, FrenetPoint frenet_state, const Lane desired_lane,const double target_speed,
                                           const std::vector<double> &maps_s, const std::vector<double> &maps_x,
                                           const std::vector<double> &maps_y){
    const unsigned int desired_points = 50;
    std::vector<CartesianPoint> spline_init_points;
    CartesianPoint ref=car_state.car_position;
    AngleInRadians ref_yaw = car_state.car_angle;
    double ref_speed = 0.0;
    const double max_acceleration = 10.0;
    const double frame_rate = 0.02;
    const unsigned int n = previous_path_x.size();
    if (previous_path_x.size() >= 2) {
        //ref = CartesianPoint(previous_path_x[n-1],previous_path_y[n-1]);
        for(unsigned int i=0;i<n;i++){
            spline_init_points.emplace_back(CartesianPoint(previous_path_x[i],previous_path_y[i]));
        }
        //ref_yaw = AngleInRadians(atan2(previous_path_y[n-1]-previous_path_y[n-2],previous_path_x[n-1]-previous_path_x[n-2]));
        ref_speed = fabs(distance(previous_path_x[n-1],previous_path_y[n-1],previous_path_x[n-2],previous_path_y[n-2])/frame_rate);
        ref_speed = std::max(ref_speed-max_acceleration*frame_rate,std::min(target_speed,ref_speed+ max_acceleration*frame_rate));
        } else {
        spline_init_points.push_back(car_state.car_position);
        if(car_state.car_speed_in_mps>0){
            spline_init_points.push_back(moveForward(car_state, -1*frame_rate).car_position);
        }else{
            CarStateCartesian new_state(car_state);
            new_state.car_speed_in_mps = 0.2;
            spline_init_points.push_back(moveForward(new_state,-1*frame_rate).car_position);
        }
        ref_speed = std::max(car_state.car_speed_in_mps-max_acceleration*frame_rate,std::min(car_state.car_speed_in_mps+max_acceleration*frame_rate,target_speed));
    }
    std::vector<CartesianPoint> next_wp(3);
    const double FORWARD_TIME=1.2;
    double forward_s=std::max(30.0,FORWARD_TIME*car_state.car_speed_in_mps);
    double desired_d = lane2d(desired_lane);
    for(unsigned long i=0;i<3;i++){
    next_wp[i] = getXY(add_to_s_set_d(frenet_state,forward_s*(i+1),desired_d),maps_s,maps_x, maps_y);
    spline_init_points.push_back(next_wp[i]);
    }
    tk::spline s;
    std::transform(spline_init_points.begin(),spline_init_points.end(),spline_init_points.begin(),[&ref,ref_yaw](CartesianPoint point){return rotation_translation(point,ref,ref_yaw); });
    std::sort(spline_init_points.begin(),spline_init_points.end(),[](CartesianPoint x,CartesianPoint y){return (x.x<=y.x);});
    std::vector<double> wp_x,wp_y;
    std::transform(spline_init_points.begin(),spline_init_points.end(),std::back_inserter(wp_x),[](CartesianPoint x){return x.x;});
    std::transform(spline_init_points.begin(),spline_init_points.end(),std::back_inserter(wp_y),[](CartesianPoint x){return x.y;});
    s.set_points(wp_x,wp_y);

    //get points on spline that do not violate speed constraints
    //it's ok to do this in transformed domain because the transformation doesn't scale, only translate and rotate
    std::vector<CartesianPoint> result(desired_points);
    double target_x = std::max(52.0,(FORWARD_TIME+1.8)*car_state.car_speed_in_mps);
    double target_y = s(target_x);
    double distance_to_end = distance(0.0,0.0,target_x,target_y);
    int spline_total_points = (unsigned  int) ceil(distance_to_end/(frame_rate*ref_speed));
    AngleInRadians angle_of_triangle = AngleInRadians(atan2(target_y,target_x));
    double spline_x,spline_y;
    for(unsigned int i=0;i <desired_points;i++){
        spline_x=(i+1)*cosrad(angle_of_triangle)*distance_to_end/spline_total_points;
        spline_y=s(spline_x);
        CartesianPoint p(spline_x,spline_y);
        result[i] = inverse_rotation_translation(p,ref,ref_yaw);
    }
return result;

}

bool collisionDetection(const std::vector<CartesianPoint> & trajectory, const std::vector<std::vector<double>> & other_cars){
    for(unsigned int i=0;i<other_cars.size();i++){

    }
}
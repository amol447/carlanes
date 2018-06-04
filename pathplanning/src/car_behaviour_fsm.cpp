//
// Created by amol on 28/05/18.
//

#include "car_behaviour_fsm.hpp"
#include <iostream>
NextFrame::NextFrame(CarStateCartesian cs, FrenetPoint fs, std::vector<std::vector<double>> const & oc, std::vector<double>const &ppx,
                     std::vector<double>const &ppy,std::vector<double>const & maps_s_in, std::vector<double>const & maps_x_in,std::vector<double>const& maps_y_in):curr_d{fs.d},curr_s{ fs.s },car_yaw{ cs.car_angle },
                                             car_x{cs.car_position.x},car_y{cs.car_position.y},
                                             previous_path_x{ppx},previous_path_y{ppy},other_cars{oc},
                                             car_speed_mps{cs.car_speed_in_mps},maps_s{maps_s_in},maps_x{maps_x_in},maps_y{maps_y_in} {

}
double calcOtherCarSpeed(std::vector<double> car_info){
    return distance(CartesianPoint(0.0,0.0),CartesianPoint(car_info[X_SPEED_POS],car_info[Y_SPEED_POS]));
}

std::vector<std::vector<double>> find_cars_in_desired_lane(NextFrame const & nextFrame,Lane l){
    std::vector<std::vector<double>> result;
    std::copy_if(nextFrame.other_cars.begin(),nextFrame.other_cars.end(),std::back_inserter(result),[&l](std::vector<double>const & car){ return (d2Lane(car[D_POS])==l);});
    return result;
}

std::vector<double> generateOtherCarTrajectory(std::vector<double> const &other_car_info,NextFrame const & nextFrame,
                                               unsigned int trajectory_size){
    std::vector<double> result;
    double other_car_speed = calcOtherCarSpeed(other_car_info);
    for(unsigned  int i=0;i<trajectory_size;i++){
        result.push_back(other_car_info[S_POS]+(i+1)*0.02*other_car_speed);
    }
    return result;
}
bool detect_collision(std::vector<double>const & trajectory1,std::vector<double>const &trajectory2){
    for(unsigned int i=0;i<trajectory1.size();i++){
        if(fabs(trajectory1[i]-trajectory2[i])<15.0){
            return true;
        }
    }
    return false;
}

void stretch_trajectory(std::vector<double>  & trajectory){
    unsigned  int L =trajectory.size();
    double avg_speed_at_end = (trajectory[L-1]-trajectory[L-3])/0.04;
    for(unsigned int i=0;i<100;i++){
        trajectory.push_back(trajectory[L+i-1]+0.02*avg_speed_at_end);
    }
}
laneChangeParams::laneChangeParams(bool sc,double s_in):should_change{sc},speed{s_in}{};
laneChangeParams  safeToChangeLane(NextFrame const & nextFrame,Lane desired_lane){
    std::vector<std::vector<double>> cars_in_desired_lane = find_cars_in_desired_lane(nextFrame,desired_lane);
    std::vector<std::vector<double>> cars_in_current_lane=find_cars_in_desired_lane(nextFrame,d2Lane(nextFrame.curr_d));
    double potential_target_speed = nextFrame.car_speed_mps;
    auto trajectoryXY = calcPathSpline(nextFrame.previous_path_x, nextFrame.previous_path_y,
                                           CarStateCartesian(nextFrame.car_x,nextFrame.car_y,nextFrame.car_yaw,nextFrame.car_speed_mps), FrenetPoint(nextFrame.curr_s,nextFrame.curr_d),desired_lane,potential_target_speed,
                                           nextFrame.maps_s, nextFrame.maps_x,
                                           nextFrame.maps_y);
    std::vector<double> trajectoryS;
    std::transform(trajectoryXY.begin(),trajectoryXY.end(),std::back_inserter(trajectoryS),[&nextFrame](CartesianPoint input){ return getFrenet(input,nextFrame.car_yaw,nextFrame.maps_x,nextFrame.maps_y).s;});
    stretch_trajectory(trajectoryS);
    bool collision=false;
    for(unsigned int i=0;i<cars_in_desired_lane.size();i++){
        auto temp =generateOtherCarTrajectory(cars_in_desired_lane[i],nextFrame,trajectoryS.size());
        collision = detect_collision(temp,trajectoryS);
        if(collision) {
            std::cout<<"detected collision-not transitioning to other lane"<<std::endl;
            return laneChangeParams(false, 0.0);
        }
    }
   return laneChangeParams(true,potential_target_speed);
}

double calcSafeLaneSpeed(NextFrame const & nextFrame,double MAX_SPEED){
    carInLaneInfo temp = isOtherCarInLaneSlower(nextFrame,MAX_SPEED);
    double result = MAX_SPEED;
    if(temp.present) {
        auto car_info = nextFrame.other_cars[temp.pos];
        double car_infront_speed = calcOtherCarSpeed(car_info);
        //target_speed_mps = std::max(nextFrame.car_speed_mps-0.4,car_infront_speed -1.0);
        result = car_infront_speed-1.0;
    }
    return result;
}

void PrepareLaneChangeLeft::react(NextFrame const & nextFrame) {
    if(num_times_in_prepare>20){
        transit<KeepLane>();
    }
    Lane curr_lane=d2Lane(nextFrame.curr_d);
    Lane desired_lane = curr_lane;
    if(curr_lane!=LEFT){
        desired_lane = (Lane) (int(curr_lane)-1 );
    }
    std::cout<<"in prepare lane change left"<<std::endl;
    laneChangeParams lcp=safeToChangeLane(nextFrame,desired_lane);
    if(lcp.should_change){
        desired_d = lane2d(desired_lane);
        target_speed_mps=lcp.speed;
        transit<LaneChangeLeft>();
        return;
    }else{
        target_speed_mps=calcSafeLaneSpeed(nextFrame,initial_v);
        num_times_in_prepare++;
        }
}

void LaneChangeLeft::react(NextFrame const & nextFrame)  {
    if(fabs(nextFrame.curr_d-desired_d)<1.0){
        transit<KeepLane>();
        return;
    }else{
        //target_speed_mps=calcSafeLaneSpeed(nextFrame,initial_v);
    }
//TODO:generate trajectory with desired_d?
}

void PrepareLaneChangeRight::react(NextFrame const & nextFrame) {
    if(num_times_in_prepare>20){
        transit<KeepLane>();

    }
    Lane curr_lane = d2Lane(nextFrame.curr_d);
    Lane desired_lane=curr_lane;
    if(curr_lane!=RIGHT){
        desired_lane = (Lane)( int(curr_lane)+1);
    }
    std::cout<<"in prepare lane change right"<<std::endl;
    laneChangeParams lcp=safeToChangeLane(nextFrame,desired_lane);
    if(lcp.should_change){
        desired_d = lane2d(desired_lane);
        target_speed_mps = lcp.speed;
        transit<LaneChangeRight>();
        return;
    } else{
        target_speed_mps = calcSafeLaneSpeed(nextFrame,initial_v);
        num_times_in_prepare++;
    }
}
double find_min_front_speed(std::vector<std::vector<double>> other_car_info, NextFrame const &nextFrame){
    std::vector<std::vector<double>> other_cars_infront_and_close;
    double SAFE_DISTANCE=80.0;
    std::copy_if(other_car_info.begin(),other_car_info.end(),std::back_inserter(other_cars_infront_and_close),[&nextFrame,&SAFE_DISTANCE](std::vector<double> car){
        return within(car[S_POS],nextFrame.curr_s,nextFrame.curr_s+SAFE_DISTANCE);});
    if(other_cars_infront_and_close.empty())
        return 1000.0;
     auto  it = std::min_element(other_cars_infront_and_close.begin(), other_cars_infront_and_close.end(),
                                                   [](std::vector< double> const &car1,std::vector<double>const &car2){return  (calcOtherCarSpeed(car1)<=calcOtherCarSpeed(car2));});
    return calcOtherCarSpeed(*it);
}
void KeepLane::react(NextFrame const &nextFrame) {
    desired_d = lane2d(d2Lane(nextFrame.curr_d));
    target_speed_mps = initial_v;
    carInLaneInfo temp = isOtherCarInLaneSlower(nextFrame,initial_v);
    std::cout<<"in Keep Lane"<<std::endl;
    if(temp.present){
        std::cout<<"in Keep lane and found other car in lane"<<std::endl;
        std::vector<std::vector<double>> left_lane_cars,right_lane_cars,centre_lane_cars;
        left_lane_cars = find_cars_in_desired_lane(nextFrame,LEFT);
        right_lane_cars = find_cars_in_desired_lane(nextFrame,RIGHT);
        centre_lane_cars = find_cars_in_desired_lane(nextFrame,CENTRE);
        double left_min_speed,centre_min_speed,right_min_speed;
        left_min_speed = find_min_front_speed(left_lane_cars,nextFrame);
        right_min_speed = find_min_front_speed(right_lane_cars,nextFrame);
        centre_min_speed = find_min_front_speed(centre_lane_cars,nextFrame);
        auto car_info = nextFrame.other_cars[temp.pos];
        double car_infront_speed = calcOtherCarSpeed(car_info);
        //target_speed_mps = std::max(nextFrame.car_speed_mps-0.2,car_infront_speed -1.0);
        Lane curr_lane=d2Lane(nextFrame.curr_d);
        if( (curr_lane==LEFT ) && (centre_min_speed>car_infront_speed) ){
            transit<PrepareLaneChangeRight>();
            return;
        }
        if( (curr_lane==RIGHT) && (centre_min_speed>car_infront_speed) ){
            transit<PrepareLaneChangeLeft>();
            return;
        }
        if( curr_lane==CENTRE){
            if( (left_min_speed>=right_min_speed) && (left_min_speed>car_infront_speed) ){
                transit<PrepareLaneChangeLeft>();
                return;
            }else if ( (left_min_speed< right_min_speed) && (right_min_speed>car_infront_speed) ){
                transit<PrepareLaneChangeRight>();
                return;
            }
        }

        target_speed_mps = car_infront_speed-1.0;


    }
}

void LaneChangeRight::react(NextFrame const & nextFrame) {
    if(fabs(nextFrame.curr_d-desired_d)<1.0){
        transit<KeepLane>();
        return;
    }else{
     //target_speed_mps = calcSafeLaneSpeed(nextFrame,initial_v);
    }

//TODO generate trajectory with desired_d?
}
bool within(const double x,const double left, const double right){
    return ((x>=left) && (x<right));
}

carInLaneInfo::carInLaneInfo() {
    present= false;
    id = -1.0;
    pos = 100000;
}

carInLaneInfo isOtherCarInLaneSlower(NextFrame const &nextFrame, double const MAX_SPEED){
    double min_s=100000.0;
    carInLaneInfo result=carInLaneInfo();
    double const SAFE_DISTANCE=50;
    for(unsigned int i=0;i<nextFrame.other_cars.size();i++){
        if(d2Lane(nextFrame.other_cars[i][D_POS]) == d2Lane(nextFrame.curr_d)){
           if(  within(nextFrame.other_cars[i][S_POS]-nextFrame.curr_s,0.0,SAFE_DISTANCE) && (calcOtherCarSpeed(nextFrame.other_cars[i])< MAX_SPEED)){
               if(min_s>nextFrame.other_cars[i][S_POS]){

                   result.present = true;
                   result.id = nextFrame.other_cars[i][ID_POS];
                   result.pos = i;
                   min_s=nextFrame.other_cars[i][S_POS];
               }

           }
        }
    }
    return result;
}
void PrepareLaneChangeRight::entry() {num_times_in_prepare=0;};
void PrepareLaneChangeLeft::entry() {num_times_in_prepare=0;};
void  PrepareLaneChangeRight::exit() {num_times_in_prepare=0;};
void  PrepareLaneChangeLeft::exit() {num_times_in_prepare=0;};

//Base class event handling
void CarBehaviour::react(NextFrame const &) {};
void CarBehaviour::entry() {};
void CarBehaviour::exit() {};
double CarBehaviour::desired_d = CarBehaviour::initial_d;
double CarBehaviour::target_speed_mps = CarBehaviour::initial_v;
int CarBehaviour::num_times_in_prepare = CarBehaviour::initial_num_times_in_prepare;
FSM_INITIAL_STATE(CarBehaviour,KeepLane);
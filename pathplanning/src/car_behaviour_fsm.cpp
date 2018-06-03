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


bool  safeToChangeLane(NextFrame const & nextFrame,Lane desired_lane){

    std::vector<std::vector<double>> cars_in_desired_lane = find_cars_in_desired_lane(nextFrame,desired_lane);
    std::vector<std::vector<double >> cars_behind_in_desired_lane_faster;
    double curr_speed = nextFrame.car_speed_mps;
    double curr_s = nextFrame.curr_s;
    std::copy_if(cars_in_desired_lane.begin(),cars_in_desired_lane.end(),std::back_inserter(cars_behind_in_desired_lane_faster),
                 [&curr_speed,&curr_s](std::vector<double> car){ return within(car[S_POS],curr_s-60,curr_s+20) && calcOtherCarSpeed(car)>curr_speed-3 ;});
    return cars_behind_in_desired_lane_faster.empty();
}
void PrepareLaneChangeLeft::react(NextFrame const & nextFrame) {
    Lane curr_lane=d2Lane(nextFrame.curr_d);
    Lane desired_lane = curr_lane;
    if(curr_lane!=LEFT){
        desired_lane = (Lane) (int(curr_lane)-1 );
    }
    std::cout<<"in prepare lane change left"<<std::endl;
    if(safeToChangeLane(nextFrame,desired_lane)){
        desired_d = lane2d(desired_lane);
        auto cars_in_desired_lane = find_cars_in_desired_lane(nextFrame,desired_lane);
        double MAX_SPEED=initial_v;
        auto new_target_speed = std::min(find_min_front_speed(cars_in_desired_lane,nextFrame)-1.0,MAX_SPEED);
        if(new_target_speed>=nextFrame.car_speed_mps) {
            target_speed_mps = new_target_speed;
            transit<LaneChangeLeft>();
            return;
        }else{
            //stale info return to keep lane
            transit<KeepLane>();
            return;
        }
    }else{
        carInLaneInfo temp = isOtherCarInLaneSlower(nextFrame,initial_v);
        if(temp.present) {
            auto car_info = nextFrame.other_cars[temp.pos];
            double car_infront_speed = calcOtherCarSpeed(car_info);
            //target_speed_mps = std::max(nextFrame.car_speed_mps-0.4,car_infront_speed -1.0);
            target_speed_mps = car_infront_speed-1.0;


            } else{
            target_speed_mps = initial_v;
        }
        }
}

void LaneChangeLeft::react(NextFrame const & nextFrame)  {
    if(fabs(nextFrame.curr_d-desired_d)<1.0){
        transit<KeepLane>();
        return;
    }
//TODO:generate trajectory with desired_d?
}

void PrepareLaneChangeRight::react(NextFrame const & nextFrame) {
    Lane curr_lane = d2Lane(nextFrame.curr_d);
    Lane desired_lane=curr_lane;
    if(curr_lane!=RIGHT){
        desired_lane = (Lane)( int(curr_lane)+1);
    }
    std::cout<<"in prepare lane change right"<<std::endl;
    if(safeToChangeLane(nextFrame,desired_lane)){
        desired_d = lane2d(desired_lane);
        auto cars_in_desired_lane = find_cars_in_desired_lane(nextFrame,desired_lane);
        double MAX_SPEED=initial_v;
        auto new_target_speed = std::min(find_min_front_speed(cars_in_desired_lane,nextFrame)-1.0,MAX_SPEED);
        if(new_target_speed>=nextFrame.car_speed_mps){
            target_speed_mps = new_target_speed;
            transit<LaneChangeRight>();
            return;
        }else{
            //stale info go to keep lane
            transit<KeepLane>();
            return;
        }

    } else{
        carInLaneInfo temp = isOtherCarInLaneSlower(nextFrame,initial_v);
        if(temp.present) {
        auto car_info = nextFrame.other_cars[temp.pos];
        double car_infront_speed = calcOtherCarSpeed(car_info);
        //target_speed_mps = std::max(nextFrame.car_speed_mps-0.4,car_infront_speed -1.0);
         target_speed_mps = car_infront_speed-1.0;
        }else{
            target_speed_mps = initial_v;
        }
    }
}
double find_min_front_speed(std::vector<std::vector<double>> other_car_info, NextFrame const &nextFrame){
    std::vector<std::vector<double>> other_cars_infront;
    std::copy_if(other_car_info.begin(),other_car_info.end(),std::back_inserter(other_cars_infront),[&nextFrame](std::vector<double> car){
        return car[S_POS]>nextFrame.curr_s;});
    if(other_cars_infront.empty())
        return 1000.0;
     auto  it = std::min_element(other_cars_infront.begin(), other_cars_infront.end(),
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
                target_speed_mps = car_infront_speed-1.0;
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



    }
}

void LaneChangeRight::react(NextFrame const & nextFrame) {
    if(fabs(nextFrame.curr_d-desired_d)<1.0){
        transit<KeepLane>();
        return;
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

//Base class event handling
void CarBehaviour::react(NextFrame const &) {};
void CarBehaviour::entry() {};
void CarBehaviour::exit() {};
double CarBehaviour::desired_d = CarBehaviour::initial_d;
double CarBehaviour::target_speed_mps = CarBehaviour::initial_v;
FSM_INITIAL_STATE(CarBehaviour,KeepLane);
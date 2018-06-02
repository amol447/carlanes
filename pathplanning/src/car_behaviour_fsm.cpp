//
// Created by amol on 28/05/18.
//

#include "car_behaviour_fsm.hpp"
NextFrame::NextFrame(CarStateCartesian cs, FrenetPoint fs, std::vector<std::vector<double>> & oc, std::vector<double>ppx,
                     std::vector<double>ppy):curr_d{fs.d},curr_s{ fs.s },car_yaw{ cs.car_angle },
                                             car_x{cs.car_position.x},car_y{cs.car_position.y},
                                             previous_path_x{ppx},previous_path_y{ppy},other_cars{oc},car_speed_mps{cs.car_speed_in_mps} {

}

void PrepareLaneChangeLeft::react(NextFrame const & nextFrame) {
    Lane curr_lane=d2Lane(nextFrame.curr_d);
    Lane desired_lane = curr_lane;
    if(curr_lane!=LEFT){
        desired_lane = (Lane) (int(curr_lane)-1 );
    }
    desired_d = lane2d(desired_lane);
}

void LaneChangeLeft::react(NextFrame const & nextFrame)  {
    if(fabs(nextFrame.curr_d-desired_d)<1.0){
        transit<KeepLane>();
    }
//TODO:generate trajectory with desired_d?
}

void PrepareLaneChangeRight::react(NextFrame const & nextFrame) {
    Lane curr_lane = d2Lane(nextFrame.curr_d);
    Lane desired_lane=curr_lane;
    if(curr_lane!=RIGHT){
        desired_lane = (Lane)( int(curr_lane)+1);
    }
    desired_d = lane2d(desired_lane);
}
void KeepLane::react(NextFrame const &nextFrame) {
    desired_d = lane2d(d2Lane(nextFrame.curr_d));
    carInLaneInfo temp = isOtherCarInLane(nextFrame);
    if(temp.present){
        auto car_info = nextFrame.other_cars[temp.pos];
        target_speed_mps = distance(CartesianPoint(0.0,0.0),CartesianPoint(car_info[X_SPEED_POS],car_info[Y_SPEED_POS]))
                -1.0;


    }
}

void LaneChangeRight::react(NextFrame const & nextFrame) {
    if(fabs(nextFrame.curr_d-desired_d)<0.1){
        transit<KeepLane>();
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

carInLaneInfo isOtherCarInLane(NextFrame const &nextFrame){
    double min_s=1000.0;
    carInLaneInfo result=carInLaneInfo();
    for(unsigned int i=0;i<nextFrame.other_cars.size();i++){
        if(d2Lane(nextFrame.other_cars[i][D_POS]) == d2Lane(nextFrame.curr_d)){
           if(  (nextFrame.other_cars[i][S_POS]-nextFrame.curr_s)<100.0 ){
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
//
// Created by amol on 28/05/18.
//

#ifndef PATH_PLANNING_CAR_BEHAVIOUR_FSM_HPP
#define PATH_PLANNING_CAR_BEHAVIOUR_FSM_HPP

#include <tinyfsm.hpp>
#include "tinyfsm.hpp"
#include "helper.h"
#include <vector>
/*
struct OtherCarInLaneEvent:tinyfsm::Event{
 int other_car_id;
 double other_car_speed_x;
 double other_car_speed_y;
 double other_car_d;
 double other_car_s;
};

struct CurrentLaneClear:tinyfsm::Event{

};

struct LaneChangeAchieved:tinyfsm::Event{

};

struct CarsInDesiredLane:tinyfsm::Event{

};
struct DesiredLaneClear:tinyfsm::Event{

};
*/
struct NextFrame:tinyfsm::Event{
    std::vector<std::vector<double>> _other_cars;
    double _curr_d;
    double _curr_s;
    double _car_speed_mps;
    double car_x;
    double car_y;
    AngleInRadians car_yaw;
    std::vector<double> previous_path_x,previous_path_y;
};
struct CarBehaviour:public tinyfsm::Fsm<CarBehaviour>{
void react(tinyfsm::Event const &){};
//virtual void react(OtherCarInLaneEvent const & );
//virtual void react(CurrentLaneClear const &);
//virtual void react(LaneChangeAchieved const &);
//virtual void react(CarsInDesiredLane const &);
virtual void react(NextFrame const &);
virtual void entry(void);
void exit(void);
static double initial_d = 6;
//static double curr_d;
static double desired_d;
static double target_speed_mps;
std::vector<std::vector<double>> other_cars;
};


struct KeepLane;
struct PrepareLaneChangeLeft:public CarBehaviour{
void entry() override{
}
void react(NextFrame const & nextFrame)override {
    desired_d = std::min(1.0,lane2d(d2Lane(nextFrame._curr_d)-1));
}
};

struct LaneChangeLeft:public CarBehaviour{
    void entry() override{
        //generate trajectory with desired_d
    }
    void react(NextFrame const & nextFrame) override {
        if(fabs(nextFrame._curr_d-desired_d)<1.0){
            transit<KeepLane>();
        }
        //generate trajectory with desired_d
    }

};
#endif //PATH_PLANNING_CAR_BEHAVIOUR_FSM_HPP

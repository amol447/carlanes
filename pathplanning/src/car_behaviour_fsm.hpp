//
// Created by amol on 28/05/18.
//

#ifndef PATH_PLANNING_CAR_BEHAVIOUR_FSM_HPP
#define PATH_PLANNING_CAR_BEHAVIOUR_FSM_HPP

#include <tinyfsm.hpp>
#include "tinyfsm.hpp"
#include "helper.h"
#include <vector>
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

struct NextFrame:tinyfsm::Event{
    std::vector<std::vector<double>> _other_cars;
    double _curr_d;
    double _curr_s;
    double _car_speed_mps;
    double car_x;
    double car_y;
    std::vector<double> previos_path_x,previous_path_y;
};
struct CarBehaviour:public tinyfsm::Fsm<CarBehaviour>{
void react(tinyfsm::Event const &){};
virtual void react(OtherCarInLaneEvent const & );
virtual void react(CurrentLaneClear const &);
virtual void react(LaneChangeAchieved const &);
virtual void react(CarsInDesiredLane const &);
virtual void entry(void);
void exit(void);
static double initial_d = 0;
static double curr_d;
static double desired_d;
std::vector<std::vector<double>> other_cars;
};


struct KeepLane;
struct PrepareLaneChangeLeft:public CarBehaviour{
void entry() override{
    desired_d = curr_d-4;
}
};
#endif //PATH_PLANNING_CAR_BEHAVIOUR_FSM_HPP

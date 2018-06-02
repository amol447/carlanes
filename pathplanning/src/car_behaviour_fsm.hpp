//
// Created by amol on 28/05/18.
//

#ifndef PATH_PLANNING_CAR_BEHAVIOUR_FSM_HPP
#define PATH_PLANNING_CAR_BEHAVIOUR_FSM_HPP

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
    std::vector<std::vector<double>> other_cars;
    double curr_d;
    double curr_s;
    double car_speed_mps;
    double car_x;
    double car_y;
    AngleInRadians car_yaw;
    std::vector<double> previous_path_x,previous_path_y;
    NextFrame(CarStateCartesian,FrenetPoint,std::vector<std::vector<double>>&,std::vector<double>,std::vector<double>);
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
static double constexpr initial_d = 6;
static double constexpr initial_v = 49.0/2.24;
//static double curr_d;
static double desired_d;
static double target_speed_mps;
std::vector<std::vector<double>> other_cars;
};


struct KeepLane;
struct PrepareLaneChangeLeft:public CarBehaviour{
void entry() override{
}
void react(NextFrame const &) override;
};

struct LaneChangeLeft:public CarBehaviour{
    void entry() override{
        //generate trajectory with desired_d
    }

    void react(NextFrame const &) override;
};

struct PrepareLaneChangeRight:public CarBehaviour{
    void entry() override{

    }

    void react(NextFrame const &) override;
};
struct LaneChangeRight:public CarBehaviour{
    void entry()override{
        //TODO:generate trajectory with desired_d
    }
    void react(NextFrame const &) override;
};
struct KeepLane:public CarBehaviour{
    void entry()override {

    }
    void react(NextFrame const &) override;
};
struct carInLaneInfo{
    bool present;
    double id;
    unsigned int pos;
    carInLaneInfo();
};
carInLaneInfo isOtherCarInLane(NextFrame const &);
enum otherCarInfo{
    ID_POS=0,X_POS=1,Y_POS=2,X_SPEED_POS=3,Y_SPEED_POS=4,S_POS=5,D_POS=6
};
#endif //PATH_PLANNING_CAR_BEHAVIOUR_FSM_HPP

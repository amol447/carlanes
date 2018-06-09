# Path Planning Project
The aim of the project was to generate paths for a self driving car that are *comfortable* and *safe*. The program takes the current senson fusion input(to know where other cars are) as well as map information (to know where is the road/lane etc) as input. 
The output is a set of coordinates that the car should visit. The car must not move faster than 50mph/ must not accelrate and jerk more than 10mph^2 and 10mph^3 respectively. There are two seperate parts of the path generation.

  * Path generation to stay in lane or change lane while obeying the speed/acceleration constraints.
  * Deciding when it is safe/efficient to change lane.
  
# Path generation
The main function that generates path in code is *calcPathSpline* in helper.cpp. It uses the spline library to generate smooth paths that pass through certain points in lane. The way to change lane is to generate points in future that are in the desired lane(different from current lane). To maintain smoothness, the path generator keeps the path generated in the previous step intact and only generates more points at the end of previous path. This is conveniently done because the simulator returns previous path points that are yet to be visited by the car.  We need to handle additional complication of staying in lane vs lane change conveniently. Frenet coordinates help with this goal as maintaining *d* in frenet coordinates while advancing *s*  is effectively maintaining lane.  Spline generation is only half of the problem of path generation since spline gives you which points need to be visited but doesn't answer *when* to visit those points. The "when" depends on the constraints the car is under such as *target_speed*, *max_acceleration*, *max_jerk*. The path generation using splines can be further broken down into following parts

  * Handle previous path points to generate smooth splines. Sometimes there arent' enough previous points and we need to handle that case. *see helper.cpp:257-274*

  * Generate new points far out on desired lane. *see helper.cpp275-282*. These are not the final points in trajectory but guide points that are used later to generate actual points. Since these points are usually for lane guidance, frenet coordinates are convienient and hence we generate them in frenet system but convert to XY system before calculating spline. 

  * Rotate and translate candidate points for spline fitting. This is for computational convinience and mathematical stability of spline fitting. *see helper.cpp:231-245* for these coordinate transforms.

  * Generate points on spline. The closer the points are, the higher the speed. Hence *target_speed* is used to properly space the points.  see *helper.cpp:308-312*. Since the simulator visits each points every 20ms, the distance between points should be less than *target_speed x 20ms* to maintain speed limit. The spline is however a non linear curve and it is hard to find distance of points accurately. Hence a linearization scheme is used as an approximation.
At this point, we can generate paths that can maintain speed limits/comfort limits and change lanes if needed

# Path planning
This part uses sensor fusion data to find paths that overall advance car faster towards the goal while maintaining safety. The previous part can generate trajectories with required characteristics(such as speed/acceleration/lane) and this part *decides* what those values should be. We use a state machine to make these decisions. Each call back from the simulator is an event for the state machine. See figure ![StateMachine](statemachine.jpg?raw=true "State Machine") for the state machine. We have used the tinyFSM library for creating the state machine. The "output" of the FSM is *target_speed* and *desired_d*. The react function in each state reacts to simulator call back. All the information needed to calculate state transitions are part of the event data. *see the struct NextFrame struct in car_behaviour_fsm.hpp:39-49*. The code closely follows the State machine diagram. 

  * If the current lane is free(car can move at full speed) or other lanes are even slower(we calculate minimum of speed of cars in other lanes that are ahead of our car to guess lane speed) stay in current lane. The safe speed in current lane is minimum of speed limit and the speed of car infront of us(minus a small amount to account for variance). If on the other hand a lane faster than current lane is found, we switch to prepare lane change(left|right). *see car_behaviour_fsm.cpp:151-187*

  * In prepare lane change state we determine if it is indeed safe to change lanes. We use the trajectory generation function using splines for our own car as well as projections of car movements of other cars in desired lane to detect potential collision. We do ignore case where other cars may decide to change lanes when deciding safety. However, we feel this behaviour is not part of path planning but vehicle safety. There is more that can be done in this state to actually change lanes(for example speeding up or slowing down to effect a faster change lane). In addition, we keep track of how much time we spend in prepare state. This is because the decision to change lane becomes stale as time moves forward and indeed some other lane may become better eventually. *see car_behaviour_fsm.cpp:117-138, 85-105* 


  * If it is safe to change lane, we move to lane change state and set the *desired_d* in state machine. This value is picked up by the path generation function to generate trajectory. Once the desired_d and current_d are close enough, lane change is achieved and we can transition to keep lane state. car_behaviour_fsm.cpp implements most of the state machine behaviour. 


# Future Work
We do not look too aggresively for lane change opportunities. In prepare lane change state, we can try to accelerate or slow down to see if lane change can be quickly effected. It is too often that the car is unable to change lanes because of a car at approximately the same speed as our car, right behind us in desired lane. However, this is a problem even for a human driver!. It is actually difficult to find a safe way to change lanes in this situation especially if we assume that the other car can speed up as well. We also don't look for lane change opportunites from all the way left to the right and vice versa.(We always look for left->center->right or right->center->left). 



  
  
 

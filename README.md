# Vehicle Control Package

## Launch File Tree:

* `system.launch`: Launches all active packages by including their parent launch file
   1. `state_estimator.launch`: Launches the state estimator node belonging to the **autonomx_state_estimator** ROS package
   
   1. `perception.launch`: Launches all active  nodes belonging to the **perception_pkg** by calling their individual launch files
      * `lane_detector.launch`: Launches the node responsible for performing the lane detection needed for online planning

      * `object_detector.launch`: Launches the node responsible for peforming 2D object detection, needed for online planning

   1. `planner.launch`: Launches all active nodes belonging to the **path_planning_pkg** ROS package by calling their individual launch files
      * `offline_planner.launch`: Launches the offline planner node, utilized for generating predefined trajectories
      
      * `global_planner.launch`: Launches the GPS-like global planner node, finds shortest path to goal according to a predefined graph of nodes

      * `behavioral_planner.launch`: Launches the intermediate planner needed for processing the global plan and guiding the local planner to the next goal

      * `local_planner.launch`: Launches the local planner, responsible for generating an obstacle-free and feasible trajectory connecting the current and target poses
   
   1. `vehicle_control.launch`: Launches all active nodes belonging to the **vehicle_control_pkg** ROS package by calling their individual launch files
      * `teleop_keyboard_controller.launch`: Launches the keyboard controller node if **controller_type** = *'teleop_keyboard'*
      
      * `open_loop_controller.launch`: Launches the open loop controller node if **controller_type** = *'open_loop'*
      
      * `stanley_control.launch`: Launches the Stanley lateral controller node if **controller_type** == *'stanley'* (default controller choice)
      
      * `longitudinal_controller.launch`: Launches the discrete PID longitudinal controller if **controller_type** == *'stanley'* (default controller choice) 

## Package Summary

The **vehicle_control_pkg** contains all nodes responsible for ensuring that the vehicle follows its planned trajectory. It is made up of four different controllers as follows:
   * `teleop_keyboard_controller`: A manual keyboard-based controller, implmented for the sake of testing other modules before an operational closed-loop controller was available. The control keys of the controller are printed out upon the node's initialization.

   * `open_loop_controller`: An offline open-loop controller, implemented due to the competition's requirements. Calculates the timestamp at which each target pose should be ideally reached, based on the kinematic vehicle model, as well as the steering input needed, based on the path's curvature. Afterwards, based on the current time, the appropriate steering input is sent along with target velocity from the trajectory to the vehicle.

   * `stanley_controller`: An implementation of the well-known Stanley closed-loop lateral controller needed for eliminating both lateral as well as heading errors to the planned trajectory. Relies on linear interpolation between waypoints, in order to avoid sudden changes in the target waypoint.

   * `longitudinal_controller`: A discrete PID closed-loop longitudinal controller responsible for ensuring the vehicle follows the planned velocity profile. Similar to Stanley, it also relies on linear interpolation to determine the target velocity, such that the velocity profile varies smoothly.  

## Nodes Summary

### Teleop Keyboard Controller

#### Node Parameters:
   * `steering_topic_name`: Name of the topic to publish calculated steering angles to, according to the vehicle model within the simulator.
   * `throttle_topic_name`: Name of the topic to publish calculated throttle inputs to, according to the vehicle model within the simulator.
   * `brakes_topic_name`: Name of the topic to publish calculated brake inputs to, according to the vehicle model within the simulator.  
   * `steering_max`: Maximum absolute steering angle in radians.
   * `steering_incr`: Postive or negative change applied to the current steering angle upon the use of keyboard inputs related to steering.
   * `velocity_max`: Maximum forward velocity of the vehicle in (m/s) corresponding to the maximum throttle input.
   * `velocity_reverse_max`: Maximum reverse velocity of the vehicle in (m/s).
   * `velocity_incr`: Postive or negative change applied to the current vehicle velocity upon the use of keyboard inputs related to velocity.
   * `sample_time`: The sampling time used by the controller in seconds.

#### Known Issues and Possible Solutions:
   * **Issue**: Noticeable input delay between button presses and vehicle response.
   * **Solution**: Possibley changing the library used to check for keyboard button presses but the node's limited use makes this issue non-critical.

### Open-Loop Controller

#### Node Parameters:
   * `traj_topic_name`: Name of the topic where the planned trajectory is published. 
   * `steering_topic_name`: Name of the topic to publish calculated steering angles to, according to the vehicle model within the simulator.
   * `throttle_topic_name`: Name of the topic to publish calculated throttle inputs to, according to the vehicle model within the simulator.
   * `traj_time_scale_factor`: A scaling factor applied to the times calculated for reaching each of the target waypoints along the planned trajectory.
   * `velocity_max`: Maximum forward velocity of the vehicle in (m/s) corresponding to the maximum throttle input.
   * `lookahead`: A lookahead distance for targeting points ahead of the vehicle's current location (unused).
   * `wheelbase`: The length of the vehicle's wheelbase in meters.
   * `rear_to_cg`: The distace from the center of the rear-axle to the vehicle center of gravity along its longitudinal axis.  
   * `sample_time`: The sampling time used by the controller in seconds.

#### Known Issues and Possible Solutions:
   * **Issue**: The vehicle either stops prematurely or after it should have stopped. This is caused by the inaccuracies of the kinematic model as well as the simulation not running in real-time.
   * **Solution**: Manually adjusting the *traj_time_scale_factor* until peformance is satisfactory.

### Stanley Controller

#### Node Parameters:
   * `traj_topic_name`: Name of the topic where the planned trajectory is published.
   * `pose_topic_name`: Name of the topic where the vehicle's current pose and twist are published.
   * `steering_topic_name`: Name of the topic to publish calculated throttle inputs to, according to the vehicle model within the simulator.
   * `steering_max`: Maximum absolute steering angle in radians.
   * `wheelbase`: The length of the vehicle's wheelbase in meters.
   * `rear_to_cg`: The distace from the center of the rear-axle to the vehicle center of gravity along its longitudinal axis.

#### Known Issues and Possible Solutions:
   * **Issue**: Controller was tuned to be slightly aggressive for fast and accurate trajectories and thus tends to frequently saturate the steering input. This tune is not suitable for a real vehicle.
   * **Solution**: Re-tune Stanley's parameters for a slightly slower but smoother response.

### Longitudinal Controller:

#### Node Parameters:
   * `traj_topic_name`: Name of the topic where the planned trajectory is published.
   * `pose_topic_name`: Name of the topic where the vehicle's current pose and twist are published.
   * `velocity_topic_name`: Name of the topic to publish calculated throttle inputs to, according to the vehicle model within the simulator.
   * `brakes_topic_name`: Name of the topic to publish calculated brake inputs to, according to the vehicle model within the simulator.
   * `velocity_max`: Maximum velocity of the vehicle in (m/s) corresponding to the maximum throttle input.
   * `acc_long_max`: Maximum allowable longitudinal acceleration in (m/s^2)that can be applied to the vehicle
   * `dec_long_max`: Maximum allowable longitudinal deceleration in (m/s^2) that can be applied to the vehicle.
   * `brake_dec_max`: Theoretical maximum deceleration in (m/s^2) that can be applied to the vehicle if all braking power was used.
   * `rear_to_cg`: The distace from the center of the rear-axle to the vehicle center of gravity along its longitudinal axis.
   * `forward_horizon`: A forward horizon in seconds according to which the vehicle's current pose will be propogated forward in time before calculating the control input.

#### Known Issues and Possible Solutions:
   * **Issue**: Controller is tough to tune due to the throttle being a velocity-controlled joint and brakes being force-controllerd; hence, having different responses.
   * **Solution**: Tuning an independent set of gains for each control input and switching between the two based on the sign of the needed acceleration.

  
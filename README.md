# Vehicle-control

Farghaaaaaaaaaaaal: Rename repo to vehicle_control_pkg to avoid errors when compiling as that's the name of the ROS package or rename in local directory but the former is the better solution.

## Launch File Tree:

* `system.launch`: Launches all active packages by including their parent launch file
  1. `vehicle_control.launch`: Launches all active nodes belonging to the `vehicle_control_pkg` ROS package by calling their individual launch files
     * `teleop_keyboard_controller.launch`: Launches the keyboard controller node if `controller_type == 'teleop_keyboard'` (default controller choice)
     * `open_loop_controller.launch`: Launches the open loop controller node if `controller_type == 'open_loop'`
  1. `path_planner.launch`: Launches all active nodes belonging to the `path_planning_pkg` ROS pakcage by calling their individual launch files (in the future)

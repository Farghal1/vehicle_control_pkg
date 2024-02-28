#include "open_loop_controller.h"

/* Class Definition */
Open_Loop_Controller::Open_Loop_Controller(ros::NodeHandle &nh)
{
    // Load in parameters from the ROS parameter server
    std::string traj_topic_name, steering_topic_name, velocity_topic_name;
    nh.param("/open_loop_controller/traj_topic_name", traj_topic_name, std::string("/trajectory"));
    nh.param("/open_loop_controller/steering_topic_name", steering_topic_name, std::string("/steer"));
    nh.param("/open_loop_controller/velocity_topic_name", velocity_topic_name, std::string("/velocity"));
    nh.param("/open_loop_controller/lookahead", _lookahead, 0.5);
    nh.param("/open_loop_controller/wheelbase", _wheelbase, 2.0);
    nh.param("/open_loop_controller/rear_to_cg", _rear_to_cg, 1.0);
    nh.param("/open_loop_controller/sample_time", _sample_time, 0.02);

    // Report the values of all parameters
    ROS_INFO("traj_topic_name: %s", traj_topic_name.c_str());
    ROS_INFO("steering_topic_name: %s", steering_topic_name.c_str());
    ROS_INFO("velocity_topic_name: %s", velocity_topic_name.c_str());
    ROS_INFO("lookahead: %.4f (m)", _lookahead);
    ROS_INFO("wheelbase: %.4f (m)", _wheelbase);
    ROS_INFO("rear_to_cg: %.4f (m)", _rear_to_cg);
    ROS_INFO("sample_time: %.4f (sec)", _sample_time);

    // Initialize class members (Non-ROS)
    _pose_current    = Vector3d::Zero();
    _steering        = 0.0;
    _velocity        = 0.0;

    // Initialize publishers and subscribers
    _steering_pub   = nh.advertise<std_msgs::Float64>(steering_topic_name, 0, true);
    _velocity_pub   = nh.advertise<std_msgs::Float64>(velocity_topic_name, 0, true);
    _pose_pub       = nh.advertise<geometry_msgs::PoseStamped>("/open_loop_controller/pose", 1, true);
    _traj_sub       = nh.subscribe(traj_topic_name, 1, &Open_Loop_Controller::callback_trajectory, this);
}

void Open_Loop_Controller::callback_trajectory(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    // Return immediately if msg size is inappropriate
    if((msg->layout.dim.size() < 2) || (msg->layout.dim[1].size < 4))
    {
        return;
    }

    // Copy the components of the trajectory into the associated vectors
    uint32_t traj_size = msg->layout.dim[0].size;
    _traj_x.resize(traj_size);
    _traj_y.resize(traj_size);
    _traj_kappa.resize(traj_size);
    _traj_vel.resize(traj_size);
    for (uint32_t i = 0; i < traj_size; i++)
    {
        _traj_x[i] = msg->data[msg->layout.data_offset + i];
        _traj_y[i] = msg->data[msg->layout.data_offset + traj_size + i];
        _traj_kappa[i] = msg->data[msg->layout.data_offset + 2 * traj_size + i];
        _traj_vel[i] = msg->data[msg->layout.data_offset + 3 * traj_size + i]; 
    }
}

void Open_Loop_Controller::update_control_input(double &steering, double &velocity)
{
    // Update pose using motion model and prev inputs
    update_pose();

    // Find closest point in trajectory (considering the lookahead distance)
    double curvature, velocity_new;
    update_nearest_neighbor(curvature, velocity_new);

    // Calculate new inputs based on curvature and planned velocity
    _steering = atan(_wheelbase * curvature);
    _velocity = velocity_new;
}

void Open_Loop_Controller::publish_msgs(const double steering, const double velocity) const
{
    // Declare msgs to be published
    std_msgs::Float64 steering_msg;
    std_msgs::Float64 velocity_msg;
    geometry_msgs::PoseStamped pose_msg;

    // Initialize the msgs
    steering_msg.data = _steering * (180.0/M_PI);
    velocity_msg.data = _velocity;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = _pose_current(0);
    pose_msg.pose.position.y = _pose_current(1);

    tf2::Quaternion orientation_tf2;
    orientation_tf2.setRPY(0.0, 0.0, _pose_current(2));
    orientation_tf2.normalize();
    pose_msg.pose.orientation.x = orientation_tf2.x();
    pose_msg.pose.orientation.y = orientation_tf2.y();
    pose_msg.pose.orientation.z = orientation_tf2.z();
    pose_msg.pose.orientation.w = orientation_tf2.w();

    // Publish all msgs
    _steering_pub.publish(steering_msg);
    _velocity_pub.publish(velocity_msg);
    _pose_pub.publish(pose_msg);
}

double Open_Loop_Controller::get_sample_time()
{
    return _sample_time;
}

/* End of Class Definition */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "open_loop_controller");
    ros::NodeHandle nh;

    Open_Loop_Controller open_loop_controller(nh);
    ros::Rate loop_rate(1.0/open_loop_controller.get_sample_time());

    while (ros::ok())
    {
        ros::spinOnce();
        double steering, velocity;
        open_loop_controller.update_control_input(steering, velocity);
        open_loop_controller.publish_msgs(steering, velocity);
        loop_rate.sleep();
    }

    return 0;
}
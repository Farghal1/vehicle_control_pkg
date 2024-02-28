#ifndef OPEN_LOOP_CONTROLLER
#define OPEN_LOOP_CONTROLLER

/* C/C++ Header Files */
#include <cmath>
#include <vector>
#include <string>
#include <Eigen/Dense>

/* ROS Header Files */
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Quaternion.h"

using std::vector;
using Eigen::Vector3d;

/* Open Loop Controller Class */
class Open_Loop_Controller
{
    public:
    // Constructor and public methods
    Open_Loop_Controller(ros::NodeHandle &nh);
    void callback_trajectory(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void update_control_input(double &steering, double &velocity);
    void publish_msgs(const double steering, const double velocity) const;
    double get_sample_time();

    private:
    // Private methods
    void update_pose()
    {
        double beta = atan2(_rear_to_cg * tan(_steering), _wheelbase);

        // Define a lambda function for computing the motion model
        auto motion_model = [this, &beta](const Vector3d &pose) -> Vector3d
        {
            return Vector3d(_velocity * cos(beta + pose(2)), 
                            _velocity * sin(beta + pose(2)), 
                            _velocity * tan(_steering) * cos(beta)/_wheelbase);
        };

        // Update pose using motion model (RK4) and inputs
        Vector3d k1 = motion_model(_pose_current);
        Vector3d k2 = motion_model(_pose_current + (_sample_time/2.0) * k1);
        Vector3d k3 = motion_model(_pose_current + (_sample_time/2.0) * k2);
        Vector3d k4 = motion_model(_pose_current + _sample_time * k3);

        _pose_current += _sample_time * ((k1 + 2 * k2 + 2 * k3 + k4)/6.0); 
    }

    void update_nearest_neighbor(double &curvature, double &velocity) const
    {
        uint32_t traj_length = _traj_x.size();
        if (__builtin_expect(traj_length != 0, true))
        {
            double min_dist = std::numeric_limits<double>::infinity();
            double min_dist_lookahead = std::numeric_limits<double>::infinity();
            uint32_t nearest_neighbor, nearest_neighbor_lookahead;
            double delta_x, delta_y, euclid_dist, euclid_dist_lookahead;
            for (uint32_t i = 0; i < traj_length; i++)
            {
                delta_x = _traj_x[i] - _pose_current(0);
                delta_y = _traj_y[i] - _pose_current(1); 
                euclid_dist = sqrt(delta_x * delta_x + delta_y * delta_y);
                euclid_dist_lookahead = abs(euclid_dist - _lookahead);

                if (euclid_dist < min_dist)
                {
                    min_dist = euclid_dist;
                    nearest_neighbor = i;
                }

                if ((euclid_dist_lookahead < min_dist_lookahead) && (abs(atan2(delta_y, delta_x)) < M_PI_2))
                {
                    min_dist_lookahead = euclid_dist_lookahead;
                    nearest_neighbor_lookahead = i;
                }
            }

            curvature = _traj_kappa[nearest_neighbor_lookahead];
            velocity = _traj_vel[nearest_neighbor];
        }
        else
        {
            curvature = 0.0;
            velocity = 0.0;
        }
    }

    // Private members

    // Trajectory related
    vector<float> _traj_x;
    vector<float> _traj_y;
    vector<float> _traj_kappa;
    vector<float> _traj_vel;

    // Pose (2D) related
    Vector3d _pose_current;

    // Control related
    double _steering;
    double _velocity;
    double _lookahead;
    double _wheelbase;
    double _rear_to_cg;
    double _sample_time;

    // ROS publishers and subscribers
    ros::Publisher _steering_pub;
    ros::Publisher _velocity_pub;
    ros::Publisher _pose_pub;
    ros::Subscriber _traj_sub;
};

#endif
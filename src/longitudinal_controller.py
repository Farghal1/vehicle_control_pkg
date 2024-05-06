#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float64, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion

class LongitudinalController:
    def __init__(self):
        rospy.init_node('longitudinal_controller')
        
        # Load parameters from the ROS parameter server
        traj_topic_name = rospy.get_param('~traj_topic_name', '/path_planner/trajectory')
        pose_topic_name = rospy.get_param('~pose_topic_name', '/odom')
        velocity_topic_name = rospy.get_param('~velocity_topic_name', '/velocity')
        brakes_topic_name = rospy.get_param('~brakes_topic_name', '/brakes')
        self.velocity_max = rospy.get_param('~velocity_max', 20.0)
        self.acc_long_max = rospy.get_param('~acc_long_max', 2.5)
        self.dec_long_max = rospy.get_param('~dec_long_max', 3.0)
        self.brake_dec_max = rospy.get_param('~brake_dec_max', 8.0)
        self.rear_to_cg = rospy.get_param('~rear_to_cg', 1.0)

        # Report the values of all retreived parameters
        rospy.loginfo("traj_topic_name: %s", traj_topic_name)
        rospy.loginfo("pose_topic_name: %s", pose_topic_name)
        rospy.loginfo("velocity_topic_name: %s", velocity_topic_name)
        rospy.loginfo("brakes_topic_name: %s", brakes_topic_name)
        rospy.loginfo("velocity_max: %.4f (m/s)", self.velocity_max)
        rospy.loginfo("acc_long_max: %.4f (m/s^2)", self.acc_long_max)
        rospy.loginfo("dec_long_max: %.4f (m/s^2)", self.dec_long_max)
        rospy.loginfo("brake_dec_max: %.4f (m/s^2)", self.brake_dec_max)
        rospy.loginfo("rear_to_cg: %.4f (m)", self.rear_to_cg)
        rospy.loginfo("\n\n")

        # Trajectory related
        self.traj_x = None
        self.traj_y = None
        self.traj_vel = None
        self.traj_acc = None
        self.closest_index_old = 0

        # Controller related
        self.kp = 6.0
        self.ki = 0.1
        self.kd = 0.3
        self.cut_off = 100
        self.err = 0.0
        self.err_intg = 0.0
        self.err_dot = 0.0
        self.sample_time = -1
        self.odom_time_prev = None

        # Initialize ROS publishers and subscribers
        self.velocity_pub = rospy.Publisher(velocity_topic_name, Float64, queue_size=1)
        self.brakes_pub = rospy.Publisher(brakes_topic_name, Float64, queue_size=1)
        self.err_pub = rospy.Publisher("/longitudinal_controller/vel_err", PointStamped, queue_size=1)
        self.traj_sub = rospy.Subscriber(traj_topic_name, Float32MultiArray, self.trajectory_callback)
        self.odom_sub = rospy.Subscriber(pose_topic_name, Odometry, self.odom_callback)

    def odom_callback(self, msg):
        # Update sampling time
        curr_time = msg.header.stamp
        if self.odom_time_prev is not None:
            self.sample_time = (curr_time - self.odom_time_prev).to_sec()
        self.odom_time_prev = curr_time

        if (self.traj_x is not None) and (self.sample_time > 0):
            # Get the current vehicle position and velocity
            quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
            )
            _, _, yaw = euler_from_quaternion(quaternion)

            position_current = np.array([msg.pose.pose.position.y + self.rear_to_cg * np.cos(yaw), 
                                         -msg.pose.pose.position.x + self.rear_to_cg * np.sin(yaw)])
            velocity_current = np.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

            # Perform acceleration test if needed (uncomment)
            # self.perform_accel_test(velocity_current, accel_cmd=2.0, brake_cmd=0.1, test_duration=20.0)

            # Get the target velocity and acceleration based on elapsed time
            velocity_target, accel_target = self.get_target_vel_acc(position_current)

            # Calculate the feedback acceleration using the PID control law then saturate accleration command
            accel_target += self.run_pid_controller(velocity_current, velocity_target)
            accel_target = np.clip(accel_target, -self.dec_long_max, self.acc_long_max)

            # Publish actuation commands
            self.publish_messages(velocity_current, velocity_target, accel_target, debug=True)

    def get_target_vel_acc(self, position_current):
        # Find the next waypoint along the trajectory
        distance = np.hypot(self.traj_x - position_current[0], self.traj_y - position_current[1])
        closest_index = np.argmin(distance)
        if (closest_index != (self.traj_x.shape[0]-1)) and \
            ((closest_index == 0) or (distance[closest_index + 1] < distance[closest_index - 1])):
            # Closest waypoint is behind so increment by one 
            closest_index += 1

        # Check if trajectory has been completed
        if (closest_index >= self.closest_index_old):
            # Get target velocity and acceleration using linear interpolation
            self.closest_index_old = closest_index
            delta_dist = distance[closest_index - 1]/(distance[closest_index] + distance[closest_index - 1]) 
            velocity_target = self.traj_vel[closest_index - 1] + (self.traj_vel[closest_index] - self.traj_vel[closest_index - 1]) * delta_dist
            accel_target = self.traj_acc[closest_index - 1] + (self.traj_acc[closest_index] - self.traj_acc[closest_index - 1]) * delta_dist
        else:
            velocity_target = self.traj_vel[-1]
            accel_target = self.traj_acc[-1]
            print("Trajectory Complete")

        return velocity_target, accel_target
    
    def run_pid_controller(self, velocity_current, velocity_target):
        # Update velocity error and error integral (using trapezoidal integration)
        err_curr = velocity_target - velocity_current
        self.err_intg += 0.5 * (err_curr + self.err) * self.sample_time

        # Estimate the rate of change of the error using discrete differentiator + low-pass filter
        rc = 1.0/(2 * np.pi + self.cut_off)
        alpha = self.sample_time/(self.sample_time + rc)
        err_dot_curr = (err_curr - self.err)/self.sample_time
        self.err_dot += alpha * (err_dot_curr - self.err_dot)

        # Update error and return acceleration command
        self.err = err_curr
        return self.kp * self.err + self.ki * self.err_intg + self.kd * self.err_dot
    
    def publish_messages(self, velocity_current, velocity_target, accel_target, debug=False):
        # Calculate and publish new velocity or brake command based on acceleration
        if accel_target > 0:
            # Publish throttle message
            velocity_msg = Float64()
            velocity_msg.data = np.clip((velocity_target + accel_target * self.sample_time)/self.velocity_max, -1.0, 1.0)
            self.velocity_pub.publish(velocity_msg)
        else:
            # Publish brakes message
            brakes_msg = Float64()
            brakes_msg.data = np.clip(-accel_target/self.brake_dec_max, 0.0, 1.0)
            self.brakes_pub.publish(brakes_msg)

        # Publish velocity tracking error for tuning purposes
        if debug:
            vel_err_msg = PointStamped()
            vel_err_msg.header.stamp = rospy.Time.now()
            vel_err_msg.point.x = velocity_current
            vel_err_msg.point.y = velocity_target
            vel_err_msg.point.z = self.err
            self.err_pub.publish(vel_err_msg)

    def perform_accel_test(self, velocity_current, accel_cmd, brake_cmd, test_duration):
        # Accelerate using the given accleration command for half the test duration
        curr_time = rospy.Time.now()
        elapsed_time = (curr_time - self.traj_start_time).to_sec()
        if elapsed_time < test_duration:
            if elapsed_time < test_duration/2:
                # Publish velocity message
                velocity_msg = Float64()
                velocity_msg.data = np.clip((accel_cmd * elapsed_time)/self.velocity_max, -1.0, 1.0)
                self.velocity_pub.publish(velocity_msg)
            # Decelerate using the given braking command
            else:
                # Publish brakes message
                brakes_msg = Float64()
                brakes_msg.data = np.clip(brake_cmd, 0.0, 1.0)
                self.brakes_pub.publish(brakes_msg)

            # Publish current velocity
            vel_err_msg = PointStamped()
            vel_err_msg.header.stamp = curr_time - self.traj_start_time
            vel_err_msg.point.x = velocity_current
            vel_err_msg.point.y = accel_cmd * elapsed_time if elapsed_time < test_duration/2 else 0.0
            vel_err_msg.point.z = 0.0
            self.err_pub.publish(vel_err_msg)

    def trajectory_callback(self, msg):
        # Ensure the message length is divisible by 4 for x, y, heading, and vel
        if len(msg.data) % 4 != 0:
            rospy.loginfo("Received trajectory data is not correctly formatted")
            return

        # Calculate the length of each segment of the trajectory
        traj_length = len(msg.data) // 4

        # Extract trajectory data from the message (only velocity needs to be stored)
        self.traj_x = np.array(msg.data[:traj_length])
        self.traj_y = np.array(msg.data[traj_length:2*traj_length])
        self.traj_vel = np.array(msg.data[3*traj_length:])
        self.traj_acc = np.empty_like(self.traj_vel)

        # Reset controller non-const variables
        self.err = 0.0
        self.err_intg = 0.0
        self.err_dot = 0.0

    def generate_time_steps(self, traj_x, traj_y):
        # Calculate the accumulated Euclidean distance to approximate the distance travelled
        dx = np.diff(traj_x)
        dy = np.diff(traj_y)
        distance = np.empty_like(traj_x)
        distance[0] = 0.0
        distance[1:] = np.hypot(dx, dy)     
        
        # Calculate longitudinal accelerations and time steps
        self.traj_acc[:-1] = np.diff(np.power(self.traj_vel, 2))/(2 * distance[1:])
        self.traj_acc[-1] = 0.0
        epsilon = 1e-3
        self.traj_time[1:] = np.where(np.abs(self.traj_acc[:-1]) > epsilon, 
                                      np.diff(self.traj_vel)/self.traj_acc[:-1], 
                                      distance[1:]/self.traj_vel[:-1])
        self.traj_time[0] = 0.0
        self.traj_time = np.cumsum(self.traj_time)     
                
        rospy.loginfo("Trajectory Time: %.4f", self.traj_time[-1])


if __name__ == '__main__':
    try:
        controller = LongitudinalController()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
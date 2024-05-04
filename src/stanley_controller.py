#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped,Quaternion
from std_msgs.msg import Float32MultiArray
import tf.transformations
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
#from libs import normalise_angle


class StanleyController:
    def __init__(self):
        rospy.init_node('stanley_controller')

       # Load parameters from the ROS parameter server
        traj_topic_name = rospy.get_param('~traj_topic_name', '/path_planner/trajectory')
        pose_topic_name = rospy.get_param('~pose_topic_name', '/odom')
        steering_topic_name = rospy.get_param('~steering_topic_name', '/steer')
        velocity_topic_name = rospy.get_param('~velocity_topic_name', '/velocity')
        self.max_steer = rospy.get_param('~steering_max', np.deg2rad(45))
        self.velocity_max = rospy.get_param('~velocity_max', 20.0)
        self.wheelbase = rospy.get_param('~wheelbase', 2.0)
        self.rear_to_cg = rospy.get_param('~rear_to_cg', 1.0)

        # Report the values of all retreived parameters
        rospy.loginfo("traj_topic_name: %s", traj_topic_name)
        rospy.loginfo("pose_topic_name: %s", pose_topic_name)
        rospy.loginfo("steering_topic_name: %s", steering_topic_name)
        rospy.loginfo("velocity_topic_name: %s", velocity_topic_name)
        rospy.loginfo("steering_max: %.4f (deg)", np.rad2deg(self.max_steer))
        rospy.loginfo("velocity_max: %.4f (m/s)", self.velocity_max)
        rospy.loginfo("wheelbase: %.4f (m)", self.wheelbase)
        rospy.loginfo("rear_to_cg: %.4f (m)", self.rear_to_cg)
        rospy.loginfo("\n\n")

        # Trajectory related
        self.traj_x = None
        self.traj_y = None
        self.traj_headings = None
        self.traj_vel = None

        # Controller related
        self.k =0.5
        self.k_soft = 0.1
        self.k_yaw_rate = 0.0
        self.k_damp_steer = 0.0
        self.closest_index_old = 0

        # Initialize class members (Non-ROS)
        self.pose_current = np.zeros(3)
        self.velocity_current = 0.0
        self.steering_cmd = 0.0
        self.velocity_cmd = 0.0

        self.steering_pub = rospy.Publisher(steering_topic_name, Float64, queue_size=1)
        self.velocity_pub = rospy.Publisher(velocity_topic_name, Float64, queue_size=1)
        self.traj_sub = rospy.Subscriber(traj_topic_name, Float32MultiArray, self.callback_trajectory)
        self.pose_sub = rospy.Subscriber(pose_topic_name, Odometry, self.odometry_callback)
    

    def callback_trajectory(self, msg):
        # Ensure the message length is divisible by 4 for x, y, kappa, and vel
        if len(msg.data) % 4 != 0:
            rospy.loginfo("Received trajectory data is not correctly formatted")
            return

        # Calculate the length of each segment of the trajectory
        segment_length = len(msg.data) // 4
        #print("segment length is",segment_length)

        # Extract trajectory data from the message
        self.traj_x = np.array(msg.data[:segment_length])
        self.traj_y = np.array(msg.data[segment_length:2*segment_length])
        self.traj_headings = np.array(msg.data[2*segment_length:3*segment_length])
        self.traj_vel = np.array(msg.data[3*segment_length:])

        #print("GOT THAT")


    def odometry_callback(self,msg):
        # Extract position
        position_x = msg.pose.pose.position.y+self.rear_to_cg* np.cos(self.pose_current[2])
        position_y = -msg.pose.pose.position.x+self.rear_to_cg* np.sin(self.pose_current[2])

        # Extract orientation (quaternion)
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)

        # Set current pose and velocity
        self.pose_current = np.array([position_x, position_y, yaw])
        self.velocity_current = np.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        #print("self.pose_current: ",self.pose_current)

        # Update control commands and publish new messages
        self.update_control_input()
        self.publish_msgs()


    def publish_msgs(self):
        steering_msg = Float64()
        velocity_msg = Float64()

        # Initialize the msgs
        steering_msg.data = np.rad2deg(self.steering_cmd)
        velocity_msg.data = np.minimum(self.velocity_cmd / self.velocity_max, 1.0)

        # Publish the msgs
        self.steering_pub.publish(steering_msg)
        self.velocity_pub.publish(velocity_msg)
        # rospy.loginfo("steering: %.4f",steering_msg.data)
        # rospy.loginfo("velocity: %.4f",velocity_msg.data)
    

    def normalizeAngle(self, angle):
        angle = np.fmod(angle, 2*np.pi)  # Normalize angle to be within [0, 2π]
        
        if angle > np.pi:  # Shift to [-π, π] if necessary
            angle -= 2.0 * np.pi
        elif angle<-np.pi:
            angle+= 2*np.pi    
        return angle
    

    def getClosestIndex(self, arr, dx, dy):
        masked_arr = np.ma.array(arr, mask=False) # Create masked array from original with no mask for now

        # Find the closest index that is located within [-pi, pi] with respect to the vehicle's heading
        while True:
            closest_index = masked_arr.argmin()
            angle=self.normalizeAngle(np.arctan2(-dy[closest_index],-dx[closest_index]) - self.pose_current[2])
            
            if(angle<np.pi/2 and angle>-np.pi/2):
                return closest_index
            else:
                masked_arr.mask[closest_index] = True # Mask value in order to be ignored in next iteration


    def update_control_input(self):

        # Ensure trajectory data is available
        if self.traj_x is not None: 
            # Calculate the Euclidean distance to each point along the trajectory
            dx = self.pose_current[0] - self.traj_x     # Find the x-axis of the front axle relative to the path
            dy = self.pose_current[1] - self.traj_y     # Find the y-axis of the front axle relative to the path
            d = np.hypot(dx, dy)                        # Find the distance from the front axle to the path

            # Get index of closest point along the trajectory that is infront of the vehicle
            closest_index = self.getClosestIndex(d, dx, dy)
            # print("closest_index",closest_index)

            # Linear interpolate to get target point
            if closest_index != 0:
                delta_prog = d[closest_index - 1]/(d[closest_index] + d[closest_index - 1])
                dx_target = dx[closest_index - 1] + (dx[closest_index] - dx[closest_index - 1]) * delta_prog
                dy_target = dy[closest_index - 1] + (dy[closest_index] - dy[closest_index - 1]) * delta_prog
                heading_target = self.traj_headings[closest_index - 1] + (self.traj_headings[closest_index] - 
                                                                          self.traj_headings[closest_index - 1]) * delta_prog
                velocity_target = self.traj_vel[closest_index - 1] + (self.traj_vel[closest_index] - 
                                                                      self.traj_vel[closest_index - 1]) * delta_prog
            else:
                dx_target = dx[closest_index]
                dy_target = dy[closest_index]
                heading_target = self.traj_headings[closest_index]       
                velocity_target = self.traj_vel[closest_index]

            # Calculate the heading error
            vehicle_heading = self.pose_current[2] 
            heading_error = self.normalizeAngle(heading_target-vehicle_heading)

            # Calculate the crosstrack error
            front_axle_vec_rot_90 = np.array([[np.cos(vehicle_heading - np.pi/2.0)], 
                                              [np.sin(vehicle_heading - np.pi/2.0)]])
            vec_target_2_front = np.array([[dx_target], 
                                           [dy_target]])
            cross_track_error = np.dot(vec_target_2_front.T, front_axle_vec_rot_90)
            
            # print(f"heading_error : {np.rad2deg(heading_error):.4f} (deg)")  
            # print(f"cross_track_error : {cross_track_error[0, 0]:.4f} (m)") 

            # Calculate the new steering angle
            Kp = 1 # Proportional gain
            steering = Kp * heading_error + np.arctan2((self.k * cross_track_error), (self.k_soft + self.velocity_current))

            # Check if trajectory has been completed
            if (closest_index >= self.closest_index_old):
                self.closest_index_old = closest_index
                velocity_cmd = velocity_target
            else:
                velocity_cmd = 0.0
                print("Trajectory complete") 

            # Calculate new inputs based on steering and planned velocity
            self.steering_cmd = np.clip(steering, -self.max_steer, self.max_steer)
            self.velocity_cmd = velocity_cmd


if __name__ == '__main__':
    try:
        controller = StanleyController()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
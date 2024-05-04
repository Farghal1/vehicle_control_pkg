#!/usr/bin/env python3

import rospy
import math
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
        rospy.init_node('Stanley_controller')

       # Load parameters from the ROS parameter server
        #self.nh = rospy.get_param('Stanley_controller')
        self.traj_topic_name = rospy.get_param('~traj_topic_name', '/path_planner/trajectory')
        self.pose_topic_name = rospy.get_param('~pose_topic_name', '/odom')
        self.steering_topic_name = rospy.get_param('~steering_topic_name', '/steer')
        self.velocity_topic_name = rospy.get_param('~velocity_topic_name', '/velocity')
        self.traj_time_scale_factor = rospy.get_param('~traj_time_scale_factor', 1.63)
        self.velocity_max = rospy.get_param('~velocity_max', 20.0)
        self.lookahead = rospy.get_param('~lookahead', 0.5)
        self.wheelbase = rospy.get_param('~wheelbase', 2.0)
        self.rear_to_cg = rospy.get_param('~rear_to_cg', 1.0)
        #self.sample_time = rospy.get_param('~sample_time', 0.02)

        # Trajectory related
        self.traj_x = []
        self.traj_y = []
        self.traj_headings = []
        self.traj_vel = []
        self.traj_time = []
        self.k =0.5
        self.k_soft = 0.0
        self.k_yaw_rate = 0.0
        self.k_damp_steer = 0.0
        self.max_steer = np.deg2rad(30)
        self.old_closest_index=0
    


        # Report the values of all parameters
        rospy.loginfo("traj_topic_name: %s", self.traj_topic_name)
        rospy.loginfo("steering_topic_name: %s", self.steering_topic_name)
        rospy.loginfo("velocity_topic_name: %s", self.velocity_topic_name)
        rospy.loginfo("traj_time_scale_factor: %.4f", self.traj_time_scale_factor)
        rospy.loginfo("velocity_max: %.4f", self.velocity_max)
        rospy.loginfo("lookahead: %.4f (m)", self.lookahead)
        rospy.loginfo("wheelbase: %.4f (m)", self.wheelbase)
        rospy.loginfo("rear_to_cg: %.4f (m)", self.rear_to_cg)
       #rospy.loginfo("sample_time: %.4f (sec)", self.sample_time)


        # Initialize class members (Non-ROS)
        self.traj_start_time = rospy.Time.now()
        self.traj_prev_index = 0
        self.pose_current = np.zeros(3)
        self.steering = 0.0
        self.velocity = 0.0

        self.steering_pub = rospy.Publisher(self.steering_topic_name, Float64, queue_size=1)
        self.velocity_pub = rospy.Publisher(self.velocity_topic_name, Float64, queue_size=1)
        self.pose_pub = rospy.Publisher('/Stanley_controller/pose', PoseStamped, queue_size=1)
        self.traj_sub = rospy.Subscriber(self.traj_topic_name, Float32MultiArray, self.callback_trajectory)
        self.pose_sub = rospy.Subscriber(self.pose_topic_name, Odometry, self.odometry_callback)
    
    def callback_trajectory(self, msg):
        # Ensure the message length is divisible by 4 for x, y, kappa, and vel
        if len(msg.data) % 4 != 0:
            rospy.loginfo("Received trajectory data is not correctly formatted")
            return

        # Calculate the length of each segment of the trajectory
        segment_length = len(msg.data) // 4
        #print("segment length is",segment_length)

        # Extract trajectory data from the message
        self.traj_x = msg.data[:segment_length]
        self.traj_y = msg.data[segment_length:2*segment_length]
        self.traj_headings  = msg.data[2*segment_length:3*segment_length]
        self.traj_vel = msg.data[3*segment_length:]
        #print(self.traj_x)
        #print(self.traj_y)


        # Reset previous trajectory and generate time steps for new trajectory
        self.traj_prev_index = 0
        self.generate_time_steps(segment_length)
        #print(self.traj_time)
        self.traj_start_time = rospy.Time.now()
        # print(self.traj_vel[:5])  
        #print(self.traj_headings)  

        #print("GOT THAT")

    def odometry_callback(self,msg):
        # Extract position
        position_x = -msg.pose.pose.position.y+self.rear_to_cg* math.cos(self.pose_current[2])
        position_y = msg.pose.pose.position.x+self.rear_to_cg* math.sin(self.pose_current[2])
        position_z = msg.pose.pose.position.z
        # Extract orientation (quaternion)
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.pose_current=[position_x,position_y,yaw]
        self.velocity=np.sqrt(msg.twist.twist.linear.x**2+msg.twist.twist.linear.y**2)
        #print("self.pose_current: ",self.pose_current)


    def publish_msgs(self):
        steering_msg = Float64()
        velocity_msg = Float64()
        pose_msg = PoseStamped()

         # Initialize the msgs
        steering_msg.data = self.steering * (180.0 / np.pi)
        velocity_msg.data = min(self.velocity / self.velocity_max, 1.0)

         # Convert pose_current to PoseStamped
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = self.pose_current[0]
        pose_msg.pose.position.y = self.pose_current[1]
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.pose_current[2])
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        self.steering_pub.publish(steering_msg)
        self.velocity_pub.publish(velocity_msg)
        rospy.loginfo("steering: %.4f",steering_msg.data)
        rospy.loginfo("velocity: %.4f",velocity_msg.data)

    

    
    def generate_time_steps(self, traj_length):
        # Check if the trajectory lists are empty
        if not self.traj_x or not self.traj_y or not self.traj_vel:
            rospy.loginfo("Trajectory data is empty")
            return

        # Generate time steps assuming constant acceleration between points
        self.traj_time.append(0.0)
        for i in range(1, traj_length):
            if i < len(self.traj_x) and i < len(self.traj_y) and i < len(self.traj_vel):
                delta_x = self.traj_x[i] - self.traj_x[i - 1]
                delta_y = self.traj_y[i] - self.traj_y[i - 1]
                euclid_dist = math.sqrt(delta_x ** 2 + delta_y ** 2)
                
                # Prevent division by zero by checking if avg_vel is close to zero
                avg_vel = (self.traj_vel[i] + self.traj_vel[i - 1]) / 2.0
                if abs(avg_vel) > 1e-4:
                    self.traj_time.append(self.traj_time[i - 1] + self.traj_time_scale_factor * euclid_dist / avg_vel)
                else:
                    # Handle case where velocity is close to zero
                    self.traj_time.append(self.traj_time[i - 1])
                    #print(self.traj_time)
            else:
                rospy.loginfo("Trajectory data is incomplete")
                break

        rospy.loginfo("Trajectory Time: %.4f", self.traj_time[-1])
        #print(self.traj_time)
   

    # def calculate_headings(self):
    #     headings = []
    #     for i in range(len(self.traj_x) - 1):
    #         delta_x = self.traj_x[i+1] - self.traj_x[i]
    #         delta_y = self.traj_y[i+1]- self.traj_y[i]
    #         heading = np.arctan2(delta_y, delta_x)
    #         headings.append(heading)
    # # Add the last heading as the same as the previous one
    #     if headings:
    #         headings.append(headings[-1])

    #     #print("headings array:", headings)
    #     return headings

    # def calculate_headings(self, initial_heading=0.0):
    #     # Convert curvature to heading using numerical integration
    #     headings = [initial_heading]
    #     for k in self.traj_kappa:
    #         # Assuming constant step size for simplicity
    #         ds = 1  # Adjust this value based on your actual path discretization
    #         heading = headings[-1] + k * ds
    #         headings.append(heading)
    #     return headings

    def find_next_point(self,elapsed_time):
         # Brute force search
        traj_length = len(self.traj_x)
    
        if traj_length != 0:
            for i in range(self.traj_prev_index, traj_length):
                if self.traj_time[i] > elapsed_time:
                    self.traj_prev_index = i - 1
                    return True
            self.traj_prev_index = traj_length
            return False
        else:
            return False
    

    def normalizeAngle(self, angle):
      
        angle = np.fmod(angle, 2*np.pi)  # Normalize angle to be within [0, 2π]
        if angle > np.pi:  # Shift to [-π, π] if necessary
            angle -= 2.0 * np.pi
        elif angle<-np.pi:
            angle+= 2*np.pi    
        return angle
    
        
    
    
    def update_control_input(self):

        # Ensure trajectory data is available
        if len(self.traj_x) > 0: 
            # Find the next point in trajectory based on elapsed time since trajectory start time


            # Find the new velocity based on the elapsed time
            velocity_new = 0.0
    
            # Calculate position of the front axle
            vehicle_position = np.array([self.pose_current[0], self.pose_current[1]])
            fx, fy = vehicle_position
            reference_path = [np.array([x, y]) for x, y in zip(self.traj_x, self.traj_y)]

            dx = fx - self.traj_x    # Find the x-axis of the front axle relative to the path
            dy = fy - self.traj_y    # Find the y-axis of the front axle relative to the path


            d = np.hypot(dx, dy) # Find the distance from the front axle to the path
            print("min. distance",np.min(d))
            closest_index = np.argmin(d) # Find the shortest distance in the array
            if(closest_index-self.old_closest_index<500 ):
                self.old_closest_index=closest_index
            else:
                closest_index=self.old_closest_index
                print("ya3abeet")
            
            velocity_new=self.traj_vel[closest_index]
            print("closest_index",closest_index)    
            absolute_error=d[closest_index]
            print("absolute_error",absolute_error)  
            nearest_path_vector = np.array([dx[closest_index], dy[closest_index]])

            #calculate the heading error
            vehicle_heading =self.pose_current[2] 
            desired_heading = self.traj_headings[closest_index]
            print("desired_heading",desired_heading)
            print("actual_heading",vehicle_heading)
           
            # Calculate the lookahead point
            lookahead_index = closest_index+1 
            if lookahead_index >= len(self.traj_x):
                lookahead_index = len(self.traj_x) - 1
            lookahead_heading=self.traj_headings[lookahead_index]


            # heading_error=self.normalizeAngle(desired_heading-vehicle_heading)
            heading_error=desired_heading-vehicle_heading
            front_axle_vec_rot_90=np.array([[math.cos(vehicle_heading-np.pi/2.0)],[math.sin(vehicle_heading-np.pi/2.0)]])
            vec_target_2_front=np.array([[dx[closest_index]],[dy[closest_index]]])
            ef=np.dot(vec_target_2_front.T,front_axle_vec_rot_90)


            # Calculate cross-track error 
            front_axle_vector = np.array([math.cos(vehicle_heading+self.steering), math.sin(vehicle_heading+self.steering)])                
            cross_track_error = np.sign(nearest_path_vector@front_axle_vector) * absolute_error
            #cross_track_error=np.sqrt((fx - self.traj_x[closest_index])**2+(fy - self.traj_y[closest_index])**2)
            print("heading_error : ",np.rad2deg(heading_error))  
            print("cross_track_error : ",ef) 
            print("self.velocity : ",self.velocity) 


            # Calculate the new steering angle
            Kp = 1 # Proportional gain
            steer = Kp * heading_error +math.atan2((self.k * ef), (self.k_soft + velocity_new))

            # Apply steering angle limits
            steer = np.clip(steer, -self.max_steer, self.max_steer)

            # Calculate new inputs based on steering and planned velocity
            self.steering = steer
            self.velocity = velocity_new  
        else:
            # Reset trajectory start time and prev index
            self.traj_prev_index = 0
            self.traj_start_time = rospy.Time.now()





if __name__ == '__main__':
    try:
        controller = StanleyController()
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            controller.update_control_input()
            controller.publish_msgs()        
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
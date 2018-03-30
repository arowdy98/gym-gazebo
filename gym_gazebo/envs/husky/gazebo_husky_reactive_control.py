import gym
import rospy
import roslaunch
import time
import numpy as np
import subprocess
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Empty as EmptyM
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import sys
from std_srvs.srv import Empty
import os
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String	
from subprocess import call
from gym.utils import seeding
import time
class GazeboHuskyReactiveControlEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "GazeboHuskyReactiveControlLidar_v0.launch")
        print("Initialize simulation")
        time.sleep(30)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.time_reset=rospy.Publisher('/reset_time', EmptyM, queue_size=5)
        self.confidence=0

        self.action_space = spaces.Discrete(5) #F,L,R
        self.reward_range = (-np.inf, np.inf)

        self.last_waypoint_achieved=0
        self.prev_odometry=Odometry()
        self.last_k_steps=[]
        self.max_k_steps=40

        self.bot_max_velocity_x=6.0
        self.bot_max_angular_z=1.3
        self.goal=PoseStamped()
        self.plan= Path()
        self.x_y_tolerance=0.2
        self.theta_tolerance=0.3

        self._seed()

    def in_tolerance(self,pose1,pose2,x_y_tolerance,theta_tolerance):
    	if(np.sqrt(self.sq(pose1.pose.position.x-pose2.pose.position.x)+self.sq(pose1.pose.position.y-pose2.pose.position.y)+self.sq(pose1.pose.position.z-pose2.pose.position.z))<self.x_y_tolerance):
    		if(np.sqrt(self.sq(pose1.pose.orientation.x-pose2.pose.orientation.x)+self.sq(pose1.pose.orientation.y-pose2.pose.orientation.y)+self.sq(pose1.pose.orientation.z-pose2.pose.orientation.z)+self.sq(pose1.pose.orientation.w-pose2.pose.orientation.w))<self.theta_tolerance):
    			return True
    	return False

    def get_pose_dist(self,odom1,odom2):
    	return np.sqrt(self.sq(odom1.pose.pose.position.x-odom2.pose.pose.position.x)+self.sq(odom1.pose.pose.position.y-odom2.pose.pose.position.y)+self.sq(odom1.pose.pose.position.z-odom2.pose.pose.position.z)+self.sq(odom1.pose.pose.orientation.x-odom2.pose.pose.orientation.x)+self.sq(odom1.pose.pose.orientation.y-odom2.pose.pose.orientation.y)+self.sq(odom1.pose.pose.orientation.z-odom2.pose.pose.orientation.z)+self.sq(odom1.pose.pose.orientation.w-odom2.pose.pose.orientation.w))

    def sq(self,x):
        return x*x






    def get_state(self,data,odometry,done):
        # Compute vector from current position to next unvisited nearest waypoint
        self.x_y_tolerance=0.2
        self.theta_tolerance=0.3
        new_ranges=36
        min_range = 0.4 #min_obstacle_range
        deviation_threshold=100
    

        if(len(self.last_k_steps)<self.max_k_steps):
        	self.last_k_steps.append(odometry)
        else:	
        	self.last_k_steps.pop(0)
        	self.last_k_steps.append(odometry)



        discretized_ranges = []
        mod = len(data.ranges)/new_ranges
        for i, item in enumerate(data.ranges):
            if (i%mod==0):
                if data.ranges[i] == float ('Inf') or np.isinf(data.ranges[i]):
                	# Why append 6?
                    discretized_ranges.append(6)
                elif np.isnan(data.ranges[i]):
                    discretized_ranges.append(0)
                else:
                    discretized_ranges.append(float(data.ranges[i]))

        orientation=[float(odometry.pose.pose.orientation.x),float(odometry.pose.pose.orientation.y),float(odometry.pose.pose.orientation.z),float(odometry.pose.pose.orientation.w)]
        state =discretized_ranges+orientation
        # print(state)
        return state,done






    def discretize_observation(self,data,new_ranges):
        discretized_ranges = []
        min_range = 0.4
        done = False
        mod = len(data.ranges)/new_ranges
        for i, item in enumerate(data.ranges):
            if (i%mod==0):
                if data.ranges[i] == float ('Inf') or np.isinf(data.ranges[i]):
                    discretized_ranges.append(12)
                elif np.isnan(data.ranges[i]):
                    discretized_ranges.append(0)
                else:
                    discretized_ranges.append(int(data.ranges[i]))
            if (min_range > data.ranges[i] > 0):
                done = True
        return discretized_ranges,done

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
    	#  Blocks until a service is available
    	# print("Taking a step")
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # Define the messages to publish for each action here
        if action == 0: #FORWARD PUSH
            vel_cmd = Twist()
            vel_cmd.linear.x = min(4.0+self.prev_odometry.twist.twist.linear.x,self.bot_max_velocity_x)
            vel_cmd.angular.z = self.prev_odometry.twist.twist.angular.z
            self.vel_pub.publish(vel_cmd)
        elif action == 1: #HARD LEFT
            vel_cmd = Twist()
            vel_cmd.linear.x = min(self.prev_odometry.twist.twist.linear.x,self.bot_max_velocity_x)
            vel_cmd.angular.z = min(1.0+self.prev_odometry.twist.twist.angular.z,self.bot_max_angular_z)
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = min(self.prev_odometry.twist.twist.linear.x,self.bot_max_velocity_x)
            vel_cmd.angular.z = max(-1.0+self.prev_odometry.twist.twist.angular.z,0)
            self.vel_pub.publish(vel_cmd)
        elif action==3: #Backward
            vel_cmd = Twist()
            vel_cmd.linear.x = max(-4.0+self.prev_odometry.twist.twist.linear.x,0)
            vel_cmd.angular.z = 0.0+self.prev_odometry.twist.twist.angular.z
            self.vel_pub.publish(vel_cmd)
        elif action==4: # Continue as it is
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.0+self.prev_odometry.twist.twist.linear.x
            vel_cmd.angular.z = 0.0+self.prev_odometry.twist.twist.angular.z
            self.vel_pub.publish(vel_cmd)
        	

        # Define the states for our MDP

        # Get laserscan data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass

        # Read Odometry
        odometry= None
        while odometry is None:
        	try:
        		odometry = rospy.wait_for_message('/base_pose_ground_truth', Odometry, timeout=5)
        	except:
        		pass

        # Get current position and velocity
        #odometry=subprocess.check_output(["rosservice","call","gazebo/get_model_state", "{model_name: mobile_base}"])
       
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        # state,done = self.discretize_observation(data,5)
        done=False
        state,done=self.get_state(data,odometry,done)
        # Rewards
        min_range = 0.5 #min_obstacle_range
        min_distance_moved=0.2
        reward=0


        if not done:
	        # If bot reaches goal
	        if self.in_tolerance(self.goal,odometry.pose,self.x_y_tolerance,self.theta_tolerance):
	            done=True
	            reward=10000
	            return

	        # Bot hits a obstacle
	        # if np.any(np.asarray(data.ranges)<min_range & np.asarray(data.ranges)>0):
	        # 	reward=-100
	        	# done= True
	        for i, item in enumerate(data.ranges):
	            if (min_range > data.ranges[i] > 0):
	            	#print(data.ranges[i])
	            	self.confidence+=1
	                if(self.confidence==15):
	                	reward=-200
	                	self.confidence=0
	                	#done = True

	        # bot has not moved in last k time steps
	        distance_travelled=0.0;
	        for i in range(0,len(self.last_k_steps)-2):
	        	distance_travelled+=self.get_pose_dist(self.last_k_steps[i+1],self.last_k_steps[i])

	        if distance_travelled<min_distance_moved:
	            #print("got reward here")
	            reward=-100

	        # If bot achieves next waypoint

	    



        # if not done:
        #     if action == 0:
        #         reward = 5
        #     else:
        #         reward = 1
        # else:

        #     reward = -200 

        return state, reward, done, {}

    def _reset(self):
        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        empty_msg=EmptyM()
        self.confidence=0
        try:
            #reset_proxy.call()
            #pub_reset.publish(empty_msg)
            self.reset_proxy()
            self.time_reset.publish(empty_msg)

        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # TO DO
        # IF PLAN NOT RECEIVED RESET SIMULATION
        # Create a publisher that publishes the goal to navfn planner
        #pub_reset=rospy.Publisher('/reset_time', EmptyM, queue_size=5)



        #call(["roslaunch", "husky_navigation", "move_base_mapless_demo.launch"])
        #os.system('roslaunch husky_navigation move_base_mapless_demo.launch')
        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        pub_goal=rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
        
        self.goal.header.frame_id="map"
        self.goal.pose.position.x=7.0
        self.goal.pose.position.y=7.0
        self.goal.pose.position.z=0.0
        self.goal.pose.orientation.x=0.0
        self.goal.pose.orientation.y=0.0
        self.goal.pose.orientation.z=0.26
        self.goal.pose.orientation.w=1.0

        pub_goal.publish(self.goal)


        # Read plan
        # self.plan= None
        # ctr =0 
        # while self.plan is None:
        #     try:
        #     	# Navfn planner node responds with a plan from current position to goal
        #     	pub_goal.publish(goal)
        #         self.plan = rospy.wait_for_message('/move_base/NavfnROS/plan', Path, timeout=5)
        #         if(self.plan != None):
        #         	if(len(self.plan.poses)==0):
        #         		self.plan=None
        #     except:
        #         pass


        #print(plan)
        # Read laser data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass


        # Read Odometry
        odometry= None

        while odometry is None:
        	try:
        		odometry = rospy.wait_for_message('/base_pose_ground_truth', Odometry, timeout=5)
        	except:
        		pass

        self.prev_odometry=odometry
        # print(data)
        # print(odometry)

   #      if data == None or odometry == None or self.plan==None:
			# return -1       		 

		# print(len(self.plan.poses))
  #       self.last_waypoint_achieved=self.plan.poses[0].pose       

        # progress=0
        done= False
        state,done=self.get_state(data,odometry,done)

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        # state = self.discretize_observation(data,5)

        return state

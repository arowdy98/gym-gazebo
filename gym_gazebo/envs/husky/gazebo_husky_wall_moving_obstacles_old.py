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

from std_srvs.srv import Empty
import os
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String	
from subprocess import call
from gym.utils import seeding

class GazeboHuskyWallMovingObstaclesLidarEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "GazeboHuskyWallMovingObstaclesLidar_v0.launch")
        print("Initialize simulation")
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.action_space = spaces.Discrete(3) #F,L,R
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def discretize_observation(self,data,new_ranges):
        discretized_ranges = []
        min_range = 0.4
        done = False
        mod = len(data.ranges)/new_ranges
        for i, item in enumerate(data.ranges):
            if (i%mod==0):
                if data.ranges[i] == float ('Inf') or np.isinf(data.ranges[i]):
                    discretized_ranges.append(6)
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
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # Define the messages to publish for each action here
        if action == 0: #FORWARD
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.5
            vel_cmd.angular.z = 0.0
            self.vel_pub.publish(vel_cmd)
        elif action == 1: #LEFT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.02
            vel_cmd.angular.z = 0.3
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = -0.02
            vel_cmd.angular.z = -0.3
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

        state,done = self.discretize_observation(data,5)

        if not done:
            if action == 0:
                reward = 5
            else:
                reward = 1
        else:

            reward = -200

        return state, reward, done, {}

    def _reset(self):
        print("Reset simulation")
        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        empty_msg=EmptyM()
        pub_reset=rospy.Publisher('/reset_time', EmptyM, queue_size=5)
        try:
            #reset_proxy.call()
            #pub_reset.publish(empty_msg)
            self.reset_proxy()
            pub_reset.publish(empty_msg)

        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")


        # Create a publisher that publishes the goal to navfn planner
        pub_goal=rospy.Publisher('/navfn_node/goal', PoseStamped, queue_size=5)
        goal=PoseStamped()
        goal.header.frame_id="map"
        goal.pose.position.x=8.0
        goal.pose.position.y=8.0
        goal.pose.position.z=0.0
        goal.pose.orientation.x=0.0
        goal.pose.orientation.x=0.0
        goal.pose.orientation.x=0.26
        goal.pose.orientation.x=1.0
        pub_goal.publish(goal)

        #call(["roslaunch", "husky_navigation", "move_base_mapless_demo.launch"])
        #os.system('roslaunch husky_navigation move_base_mapless_demo.launch')
        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # Read plan
        plan= None
        ctr =0 
        while plan is None:
            if(ctr==5):
                print("Path not found")
                break
            try:
            	# Navfn planner node responds with a plan from current position to goal
                plan = rospy.wait_for_message('/navfn_node/navfn_planner/plan', Path, timeout=5)
            except:
                ctr=ctr+1
                pass

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


        state_vector=(plan,data,odometry)
        print("State vector created")

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state = self.discretize_observation(data,5)

        return state

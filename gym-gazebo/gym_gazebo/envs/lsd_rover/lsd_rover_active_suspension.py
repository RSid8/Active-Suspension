from math import degrees, inf, radians
import rospy
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates
from std_srvs.srv import Empty
from gym import utils, spaces
from tf.transformations import euler_from_quaternion
from gym_gazebo.envs import gazebo_env
from sensor_msgs.msg import Imu, LaserScan
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import time
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import PoseStamped
rospack = rospkg.RosPack()

class LsdEnv(gazebo_env.GazeboEnv):
    def __init__(self):

        gazebo_env.GazeboEnv.__init__(self, "custom_world.launch")

        self.force_fl = 0
        self.force_fr = 0
        self.force_ml = 0
        self.force_mr = 0
        self.force_rl = 0
        self.force_rr = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.y_displacement=0
        self.ground_clearance=0
        self.reward = 0
        self.observation_space = spaces.Box(-inf, inf, shape=(7,), dtype=np.float32)
        self.orientation_list = []
        self.action_space = spaces.Box(-40, 40, shape=(4,), dtype=np.float32)
        self.obstacle_distance = 0
        self.obstacle_height = 0
        self.obstacle_offset = 0
        self.chassis_angle = 0
        self.actual_speed = 0
        self.done = False
        self.package_path = rospack.get_path('lsd')

        rospy.Subscriber("/imu", Imu, self.callback_imu)

        rospy.Subscriber("/odom", Odometry, self.callback_pose)

        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


        self.joint_1_publisher = rospy.Publisher("/lsd/fl_joint_position_controller/command",
                                                 Float64, queue_size=10)
        self.joint_2_publisher = rospy.Publisher("/lsd/fr_joint_position_controller/command",
                                                 Float64, queue_size=10)
        self.joint_3_publisher = rospy.Publisher("/lsd/bl_joint_position_controller/command",
                                                 Float64, queue_size=10)
        self.joint_4_publisher = rospy.Publisher("/lsd/br_joint_position_controller/command",
                                                 Float64, queue_size=10)

        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.Subscriber("/centroid_point", PoseStamped, self.callback_point)
        self.rate = rospy.Rate(10)

    def forward(self):

        vel_cmd = Twist()
        vel_cmd.linear.x = -1.5
        vel_cmd.angular.z =0

        self.velocity_publisher.publish(vel_cmd)

    def teleport(self):

        state_msg = ModelState()
        
        state_msg.model_name = 'lsd'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.5
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_state(state_msg)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


    def callback_imu(self, msg):

        self.orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z,
                                 msg.orientation.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(self.orientation_list)
        self.pitch = degrees(self.pitch)
        self.roll = degrees(self.roll)
        self.yaw = degrees(self.yaw)

    def callback_point(self, msg):
        self.centroid = msg


    def callback_pose(self, msg):
        self.actual_speed=msg.twist.twist.linear.x
        self.y_displacement=msg.pose.pose.position.y

    def transform_centroid(self):
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform("base_link", "r200_camera_rviz", 
                    rospy.Time(0))
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException):
                    self.rate.sleep()
                    continue
            pose_transformed = tf2_geometry_msgs.do_transform_pose(self.centroid, trans)
            return pose_transformed

    def get_observation(self):
        pose_transformed = self.transform_centroid()
        self.obstacle_distance = pose_transformed.pose.position.x
        self.obstacle_height = 2*pose_transformed.pose.position.y
        self.obstacle_offset = pose_transformed.pose.position.z
        self.observation_space = [self.pitch, self.roll, self.actual_speed, self.y_displacement, 
            self.obstacle_distance, self.obstacle_height, self.obstacle_offset]
        return self.observation_space

    def step(self, action):


        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")
        
        self.forward()
        self.joint_1_publisher.publish(radians(action[0]))
        self.joint_2_publisher.publish(radians(action[1]))
        self.joint_3_publisher.publish(radians(action[2]))
        self.joint_4_publisher.publish(radians(action[3]))

        time.sleep(1)
        # publish till the action taken is completed      
        observation_ = self.observation_space

        self.get_reward()

        
        state_ = (observation_, self.reward, self.done, {})
        return state_

    def get_reward(self):

        if(self.pitch>17):
            self.reward-=5

        elif(self.y_displacement>5):
            self.reward-=5

        elif(self.actual_speed<0.4):
            self.reward-=10    
        
        elif(28<(100*self.ground_clearance)<33 and abs(self.pitch)<5):
            self.reward+=10     
    
    def reset(self):

        self.teleport()

        vel_cmd = Twist()
        vel_cmd.linear.x = 0
        vel_cmd.angular.z = 0

        self.velocity_publisher.publish(vel_cmd)

        self.joint_1_publisher.publish(0)
        self.joint_2_publisher.publish(0)
        self.joint_3_publisher.publish(0)
        self.joint_4_publisher.publish(0)

        time.sleep(5)

        # unpause simulation to make an observation and reset the values
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            # resp_pause = pause.call()
            self.unpause()
        except rospy.ServiceException:
            print("/gazebo/unpause_physics service call failed")

        self.reward = 0

        initial_reading = self.get_observation()

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException:
            print("/gazebo/pause_physics service call failed")

        return initial_reading

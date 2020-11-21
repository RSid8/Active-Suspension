from math import degrees, inf, radians
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, WrenchStamped
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
        self.reward = 0
        self.observation_space = spaces.Box(-inf, inf, shape=(8, 1), dtype=np.float32)
        self.orientation_list = []
        self.action_space = spaces.Box(-60, 60, shape=(4, 1), dtype=np.float32)
        self.obstacle_distance = 0
        self.chassis_angle = 0
        self.done = False
        self.package_path = rospack.get_path('lsd')



        rospy.Subscriber("/fl_wheel_ft_sensor", WrenchStamped, self.callback_fl)
        rospy.Subscriber("/fr_wheel_ft_sensor", WrenchStamped, self.callback_fr)
        rospy.Subscriber("/ml_wheel_ft_sensor", WrenchStamped, self.callback_ml)
        rospy.Subscriber("/mr_wheel_ft_sensor", WrenchStamped, self.callback_mr)
        rospy.Subscriber("/rl_wheel_ft_sensor", WrenchStamped, self.callback_rl)
        rospy.Subscriber("/rr_wheel_ft_sensor", WrenchStamped, self.callback_rr)

        rospy.Subscriber("/imu", Imu, self.callback_imu)

        #rospy.Subscriber("/r200/camera/depth_registered/points", Pointcloud2, self.callback_Pointcloud2)


        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.joint_1_publisher = rospy.Publisher("/lsd/joint1_position_controller/command",
                                                 Float64, queue_size=10)
        self.joint_2_publisher = rospy.Publisher("/lsd/joint2_position_controller/command",
                                                 Float64, queue_size=10)
        self.joint_3_publisher = rospy.Publisher("/lsd/joint3_position_controller/command",
                                                 Float64, queue_size=10)
        self.joint_4_publisher = rospy.Publisher("/lsd/joint4_position_controller/command",
                                                 Float64, queue_size=10)

        self.pause = rospy.ServiceProxy("/gazebo/pause", Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)


    def forward(self):

        vel_cmd = Twist()
        vel_cmd.linear.x = -3
        vel_cmd.angular.z = 0

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


    def callback_fl(self, msg):
        self.force_fl = msg.wrench.force

    def callback_fr(self, msg):
        self.force_fr = msg.wrench.force

    def callback_ml(self, msg):
        self.force_ml = msg.wrench.force

    def callback_mr(self, msg):
        self.force_mr = msg.wrench.force

    def callback_rl(self, msg):
        self.force_rl = msg.wrench.force

    def callback_rr(self, msg):
        self.force_rr = msg.wrench.force

    def callback_imu(self, msg):

        self.orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z,
                                 msg.orientation.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(self.orientation_list)
        self.pitch = degrees(self.pitch)
        self.roll = degrees(self.roll)
        self.yaw = degrees(self.yaw)

    #def callback_Pointcloud2(self, msg):

        
    def get_observation(self):
        self.observation_space = np.array([self.force_fl.x, self.force_fr.x, self.force_ml.x,
                                           self.force_mr.x, self.force_rl.x, self.force_rr.x,
                                           self.pitch, self.roll])
        return self.observation_space

    def step(self, action):
        
        self.forward()

        self.joint_1_publisher.publish(action[0])
        self.joint_2_publisher.publish(action[1])
        self.joint_3_publisher.publish(action[2])
        self.joint_4_publisher.publish(action[3])
        # publish till the action taken is completed
        
        observation_ = self.observation_space
        # condition to check if the episode is complete

        if(self.pitch>0.5236 or self.pitch<-0.5236 or self.roll>0.5236 or self.roll<-0.5236):
            
            self.done= True
        else:
            
            self.done= False

        self.get_reward()

        # compute reward due to the action taken
        state_ = (observation_, self.reward, self.done, {})
        return state_

    def get_reward(self):

        threshold = (-0.5236, 0.5236)
        if threshold[0] > self.pitch > threshold[1]:
            self.reward += 1
        elif threshold[0] > self.roll > threshold[1]:
            self.reward += 1
        else:    
            self.reward -= 15
            self.done = True
        # force thresholds to be added

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

        time.sleep(10)

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

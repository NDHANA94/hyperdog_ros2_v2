"""
MIT License

Copyright (c) 2023 Nipun Dhananjaya

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from sensor_msgs.msg import Imu
from hyperdogv2_msgs.msg import PybulletUserDebugParams
from ament_index_python.packages import get_package_share_directory
import os

import pybullet as p
import pybullet_data
import numpy as np
import math
import matplotlib.pyplot as plt
from threading import Thread, Event
import time


class Ros2HyperdogPybulletNode(Node):
    def __init__(self):
        # ros params
        self._is_debug = None
        self._num_of_joints = None
        self._hip_joint_ids = None
        self._thigh_joint_ids = None
        self._calf_joint_ids = None
        self._leg_joint_home_positions = None
        self._max_motor_torque = None
        self._motor_torque_limit = None
        self._robot_height_limit = None
        self._robot_euler_angle_limits = None
        self._sim_plane_id = None
        self._robot_urdf_id = None
        # pybullet sim
        self._pybullet_client = None
        self._dt = None
        self._robot = None
        self._plane = None
        # userDebugParams Ids
        self._maxForceId = None
        self._heightId = None
        self._rollId = None
        self._pitchId = None
        self._yawId = None
        self._moveXId = None
        self._moveYId = None
        # flags
        self._is_realtime_sim = True
        self._is_params_loaded = False
        self._is_pybullet_initialized = False
        self.is_pybullet_running = False
        # for Imu
        self.base_last_measured_linear_vel = None
        self.imu_last_measured_time = None
        # Node 
        self.node_name = 'hyperdog_ros2_pybullet_node'
        super().__init__(self.node_name)
        self.logger = self.get_logger()
        self.declare_parameters(
            namespace='',
            parameters=[
                ('is_debug', None),
                ('num_of_motors', None),
                ('hip_joint_ids', None),
                ('thigh_joint_ids', None),
                ('calf_joint_ids', None),
                ('leg_joint_home_positions', None),
                ('max_motor_torque', None),
                ('sim_step_time', None),
                ('realtime_sim', None),
                ('sim_plane', None),
                ('robot_urdf_path', None),
                ('motor_torque_limit', None),
                ('robot_height_limit', None),
                ('robot_euler_angle_limit', None)
                ])
        self._sub = self.create_subscription(Float32MultiArray, 'hyperdog_jointController/commands', self.__sub_callback, 10)
        self._joint_state_pub = self.create_publisher(Float32MultiArray, 'hyperdog_sim_joint_position_feedback', 10)
        self._imu_pub = self.create_publisher(Imu, 'hyperdog_sim_imu', 10)
        # self._joint_state_pub_timer = self.create_timer(timer_period_sec=0.005, callback=self.__joint_state_pub_callback)
        self.update_params_timer = self.create_timer(1, self.__update_ros_params)
        self.pybullet_timer = self.create_timer(timer_period_sec=0.001, callback=self.__pybullet_sim_timer_callback) 
        self._user_debug_params_pub = None #self.create_publisher(PybulletUserDebugParams, 'hyperdog_sim_userdebug_params', 10)
        # --------------------------------------------------
        self.target_joint_positions = Float32MultiArray()
        self.current_joint_positions = Float32MultiArray()
        self.imu_ = Imu()
        self.user_debug_params_ = PybulletUserDebugParams()
        # --------------------------------------------------
        
        
        
        # self.timer

    def __sub_callback(self, msg:Float32MultiArray):
        if len (msg.data) == self._num_of_joints:
            self.target_joint_positions.data = [msg.data[i]*np.pi/180 for i in range(12)]
        

    def __update_ros_params(self):
        try:
            dt_prev = self._dt
            self._is_debug = self.get_parameter('is_debug').get_parameter_value().bool_value
            self._num_of_joints = self.get_parameter('num_of_motors').get_parameter_value().integer_value
            self._hip_joint_ids = self.get_parameter('hip_joint_ids').get_parameter_value().integer_array_value
            self._thigh_joint_ids = self.get_parameter('thigh_joint_ids').get_parameter_value().integer_array_value
            self._calf_joint_ids = self.get_parameter('calf_joint_ids').get_parameter_value().integer_array_value
            self._leg_joint_home_positions = self.get_parameter('leg_joint_home_positions').get_parameter_value().integer_array_value
            self._max_motor_torque = self.get_parameter('max_motor_torque').get_parameter_value().double_value
            self._dt = self.get_parameter('sim_step_time').get_parameter_value().double_value
            self._is_realtime_sim = self.get_parameter('realtime_sim').get_parameter_value().bool_value
            self._sim_plane_id = self.get_parameter('sim_plane').get_parameter_value().string_value
            self._robot_urdf_id = self.get_parameter('robot_urdf_path').get_parameter_value().string_array_value
            self._motor_torque_limit = self.get_parameter('motor_torque_limit').get_parameter_value().double_array_value
            self._robot_height_limit = self.get_parameter('robot_height_limit').get_parameter_value().double_array_value
            self._robot_euler_angle_limits = self.get_parameter('robot_euler_angle_limit').get_parameter_value().double_array_value
            self._is_params_loaded = True
            # self.logger.info("[__update_params]: parametes are loaded") # for debug
            # ========= if pybullet simulation time step is changed, update the timer_period_sec of the pybullet timer =========
            if self._dt != dt_prev:
                self.pybullet_timer.timer_period_ns = int(float(self._dt) * 1000 * 1000 * 1000)
                self.logger.info(f"pybullet simulation timer is updated tp {self._dt}")
            # if is_debug true, enable hyperdog_sim_userdebug_params publisher
            if self._is_debug and self._user_debug_params_pub == None:
                self._user_debug_params_pub = self.create_publisher(PybulletUserDebugParams, 'hyperdog_sim_userdebug_params', 10)
        except:
            self.logger.error("could not load parameters")
            # self._is_params_loaded = False

    def __pybullet_sim_timer_callback(self):
        # ========= if pybullet is not initialized, initiate it =========
        if not self._is_pybullet_initialized:
            self.is_pybullet_running = False
            self.__init_pybullet_sim()
        # ========= else, run the simulation ============================
        else:
            if self._is_debug:
                self.update_userDebugParams(publish=False)
            # ========= update joint positions ==========================
            self.set_joint_positions()
            # ========= make one step of simulation per on time beat of the timer =========
            p.stepSimulation()
            # ========= make is_pybullet_running flag True ==============
            self.is_pybullet_running = True
            # ========= update joint states =============================
            self.read_joint_states()
            # ========= publish ros2 topics =============================
            if self._joint_state_pub.get_subscription_count() != 0:
                self._joint_state_pub.publish(self.current_joint_positions)
            self.publish_imu()
            
            
    def __init_pybullet_sim(self):
        # ========= wait untill the parameters are loaded =========
        self.logger.info("[__init_pybullet_sim]: Initializing hyperdog pybullet simulation")
        while 1:
            self.update_params_timer.callback()
            if self._is_params_loaded:
                self._pybullet_client = p.connect(p.GUI)
                p.setAdditionalSearchPath(pybullet_data.getDataPath())
                self._plane = p.loadURDF(self._sim_plane_id)
                p.setGravity(0,0,-9.81, physicsClientId=self._pybullet_client)
                p.setTimeStep(self._dt)
                urdfFlags = p.URDF_USE_SELF_COLLISION
                robot_urdf_path = os.path.join(get_package_share_directory(self._robot_urdf_id[0]), self._robot_urdf_id[1])
                self.logger.info(robot_urdf_path) # for debug
                self._robot = p.loadURDF(robot_urdf_path, [0,0,0.008], [0,0,0,1], useFixedBase=False) # flags = urdfFlags
                # check and set num of joints
                if not self._num_of_joints == p.getNumJoints(self._robot):
                    self.logger.error(f"robot's num of joints are not matched. \
                                    expected num of joints {self._num_of_joints}, but got {p.gtNumJoints(self._robot)} ")
                else:
                    # ========= set the robot to home position =========
                    self.set_robot_home_position() 
                    # ========= print joint infos ======================
                    # for j in range(self._num_of_joints):
                    #     self.logger.info(f"[__init_pybullet_sim]: {p.getJointInfo(self._robot, j)}")
                    # ========= enable collision between lower legs ====
                    for l0 in self._calf_joint_ids:
                        for l1 in self._calf_joint_ids:
                            if(l1>l0):
                                enableCollision = 1
                                print("collision for pair", l0, l1, p.getJointInfo(self._robot, l0)[12], p.getJointInfo(self._robot,l1)[12], "enabled=", enableCollision)
                                p.setCollisionFilterPair(self._robot, self._robot, 2,5,enableCollision)
                    # ========= set joint dynamics =====================
                    for j in range (self._num_of_joints):
                        p.changeDynamics(self._robot, j, linearDamping=0, angularDamping=0) 
                    # ========= enable torque sensors ==================
                    [p.enableJointForceTorqueSensor(self._robot, i, True) for i in range(self._num_of_joints)]     
                    # ========= set camera =============================
                    p.getCameraImage(480,320)               
                    # ========= set real-time simulation ===============
                    p.setRealTimeSimulation(self._is_realtime_sim)               
                    # ========= set user debug parameters ==============
                    if self._is_debug:
                        self.declare_userDebugParameters()
                    self._is_pybullet_initialized = True
                    self.logger.info("[__init_pybullet_sim]: HyperDog pybullet simulation is successfully launched")
                break
            else:
                self.logger.info("waiting for parametes are loaded")
                self.logger.info(f"is_params_loaded: {self._is_params_loaded}")
            time.sleep(1)

    def set_robot_home_position(self):
        torques = [self._max_motor_torque]*self._num_of_joints
        self.target_joint_positions.data = [p*np.pi/180 for p in self._leg_joint_home_positions]*4       
        self.logger.info(f"[set_robot_home_position]: joint positions {self.target_joint_positions.data}")
        p.setJointMotorControlArray(self._robot, 
                                    range(self._num_of_joints), 
                                    p.POSITION_CONTROL, 
                                    targetPositions=self.target_joint_positions.data,
                                    forces=torques)
    
    def declare_userDebugParameters(self):
        self._maxForceId = p.addUserDebugParameter("maxForce", self._motor_torque_limit[0], self._motor_torque_limit[1], self._max_motor_torque)
        self._heightId = p.addUserDebugParameter("Height",      self._robot_height_limit[0],       self._robot_height_limit[1], self._robot_height_limit[0])
        self._rollId = p.addUserDebugParameter("roll",   -self._robot_euler_angle_limits[0], self._robot_euler_angle_limits[0], 0)
        self._pitchId = p.addUserDebugParameter("pitch", -self._robot_euler_angle_limits[1], self._robot_euler_angle_limits[1], 0)
        self._yawId = p.addUserDebugParameter("yaw",     -self._robot_euler_angle_limits[2], self._robot_euler_angle_limits[2], 0)
        self._moveXId = p.addUserDebugParameter('move_x', -150, 150, 0)
        self._moveYId = p.addUserDebugParameter('move_y', -150, 150, 0)
    
    def update_userDebugParams(self, publish=False):
        self.user_debug_params_.max_torque = p.readUserDebugParameter(self._maxForceId)
        self.user_debug_params_.body_transformation.rotation.x = p.readUserDebugParameter(self._rollId)
        self.user_debug_params_.body_transformation.rotation.y = p.readUserDebugParameter(self._pitchId)
        self.user_debug_params_.body_transformation.rotation.z = p.readUserDebugParameter(self._yawId)
        self.user_debug_params_.body_transformation.translation.x = p.readUserDebugParameter(self._moveXId)
        self.user_debug_params_.body_transformation.translation.y = p.readUserDebugParameter(self._moveYId)
        self.user_debug_params_.body_transformation.translation.z = p.readUserDebugParameter(self._heightId)
        if publish and self._is_debug and self._user_debug_params_pub != None:
            self.user_debug_params_.header.stamp = self.get_clock().now().to_msg()
            self._user_debug_params_pub.publish(self.user_debug_params_)
            
            
    def read_joint_states(self):
        """Read joint states of the robot in simulation"""
        if self.is_pybullet_running:
            joint_states = np.array(p.getJointStates(self._robot, [i for i in range(self._num_of_joints)]))
            joint_positions = list(joint_states[:,0])
            self.current_joint_positions.data = joint_positions
    
    def set_joint_positions(self, target_position=None):
        """Set robot's joint positions to self.target_joint_positions.data, if taget_position of the function not provided."""
        torque = self.user_debug_params_.max_torque  if self._is_debug else self._max_motor_torque
        torques = [torque]*self._num_of_joints
        if target_position == None:
            positions = self.target_joint_positions.data
        elif len(target_position) == self._num_of_joints:
            positions = target_position
        if len(positions) == self._num_of_joints:
            p.setJointMotorControlArray(self._robot, 
                                    range(self._num_of_joints), 
                                    p.POSITION_CONTROL, 
                                    targetPositions=self.target_joint_positions.data,
                                    forces=torques)
        
    def publish_imu(self):
        if self._imu_pub.get_subscription_count() != 0:
            _, base_orientation = p.getBasePositionAndOrientation(self._robot) 
            base_linear_velocity , base_angular_velocity = p.getBaseVelocity(self._robot)
            measured_time = time.time_ns()
            # base_orientation_cov = None
            # base_angular_velocity_cov = None
            # base_linear_acceleration_cov = None
            self.imu_.header.stamp = self.get_clock().now().to_msg()
            if self.imu_last_measured_time == None:
                self.base_last_measured_linear_vel =  base_linear_velocity
                self.imu_last_measured_time = measured_time
            else:
                dt = (time.time_ns() - self.imu_last_measured_time)/(1000*1000*1000)
                linear_acceleration = [(base_linear_velocity[i] - self.base_last_measured_linear_vel[i])/dt for i in range(3)]
                self.imu_.linear_acceleration.x = linear_acceleration[0]
                self.imu_.linear_acceleration.y = linear_acceleration[1]
                self.imu_.linear_acceleration.z = linear_acceleration[2]
                self.imu_.orientation.x = base_orientation[0]
                self.imu_.orientation.y = base_orientation[1]
                self.imu_.orientation.z = base_orientation[2]
                self.imu_.orientation.w = base_orientation[3]
                self.imu_.angular_velocity.x = base_angular_velocity[0]
                self.imu_.angular_velocity.y = base_angular_velocity[1]
                self.imu_.angular_velocity.z = base_angular_velocity[2]
                self._imu_pub.publish(self.imu_)
            

def main(args=None):
    rclpy.init(args=args)
    node = Ros2HyperdogPybulletNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
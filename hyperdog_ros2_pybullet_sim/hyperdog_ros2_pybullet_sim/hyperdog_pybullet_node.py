import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
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
        self._is_params_loaded = False
        # ------------------ Pybullet Simulation -----------
        self._dt = None
        self._sim_plane_id = None
        self._robot_urdf_id = None
        self._robot = None
        self._plane = None
        self._pybullet_client = None
        self._maxForceId = None
        self._heightId = None
        self._rollId = None
        self._pitchId = None
        self._yawId = None
        self._moveXId = None
        self._moveYId = None
        self._is_pybullet_initialized = False
        self.is_pybullet_running = False
        # ------------------- Node -------------------------
        super().__init__('hyperdog_ros2_pybullet_node')
        self.logger = self.get_logger()
        self.declare_parameters(
            namespace='',
            parameters=[
                ('IS_DEBUG', None),
                ('NUM_OF_JOINTS', None),
                ('HIP_JOINT_IDS', None),
                ('THIGH_JOINT_IDS', None),
                ('CALF_JOINT_IDS', None),
                ('LEG_JOINT_HOME_POSITIONS', None),
                ('MAX_MOTOR_TORQUE', None),
                ('SIM_STEP_TIME', None),
                ('SIM_PLANE', None),
                ('ROBOT_URDF', None),
                ('MOTOR_TORQUE_LIMIT', None),
                ('ROBOT_HEIGHT_LIMIT', None),
                ('ROBOT_EULER_ANGLE_LIMITS', None),
                ])
        self._sub = self.create_subscription(Float64MultiArray, 'hyperdog_joint_positions', self.__sub_callback, 10)
        self._joint_state_pub = self.create_publisher(Float64MultiArray, 'hyperdog_joint_position_feedback', 10)
        self._joint_state_pub_timer = self.create_timer(timer_period_sec=0.005, callback=self.__joint_state_pub_callback)
        self.update_params_timer = self.create_timer(1, self.__update_params)
        self.pybullet_timer = self.create_timer(timer_period_sec=0.05, callback=self.__pybullet_sim_timer_callback)
        self.target_joint_positions = Float64MultiArray()
        self.current_joint_positions = Float64MultiArray()
        # --------------------------------------------------
        
        
        
        # self.timer

    def __sub_callback(self, msg:Float64MultiArray):
        self.target_joint_positions.data = msg.data

    def __joint_state_pub_callback(self):
        if self.is_pybullet_running and len(self.current_joint_positions.data) == p.getNumJoints(self._robot) and self._joint_state_pub.get_subscription_count() != 0:
            self._joint_state_pub.publish(self.current_joint_positions)

            


    def __update_params(self):
        try:
            dt_prev = self._dt
            self._is_debug = self.get_parameter('IS_DEBUG').get_parameter_value().bool_value
            self._num_of_joints = self.get_parameter('NUM_OF_JOINTS').get_parameter_value().integer_value
            self._hip_joint_ids = self.get_parameter('HIP_JOINT_IDS').get_parameter_value().integer_array_value
            self._thigh_joint_ids = self.get_parameter('THIGH_JOINT_IDS').get_parameter_value().integer_array_value
            self._calf_joint_ids = self.get_parameter('CALF_JOINT_IDS').get_parameter_value().integer_array_value
            self._leg_joint_home_positions = self.get_parameter('LEG_JOINT_HOME_POSITIONS').get_parameter_value().integer_array_value
            self._max_motor_torque = self.get_parameter('MAX_MOTOR_TORQUE').get_parameter_value().double_value
            self._dt = self.get_parameter('SIM_STEP_TIME').get_parameter_value().double_value
            self._sim_plane_id = self.get_parameter('SIM_PLANE').get_parameter_value().string_value
            self._robot_urdf_id = self.get_parameter('ROBOT_URDF').get_parameter_value().string_array_value
            self._motor_torque_limit = self.get_parameter('MOTOR_TORQUE_LIMIT').get_parameter_value().double_array_value
            self._robot_height_limit = self.get_parameter('ROBOT_HEIGHT_LIMIT').get_parameter_value().double_array_value
            self._robot_euler_angle_limits = self.get_parameter('ROBOT_EULER_ANGLE_LIMITS').get_parameter_value().double_array_value
            self._is_params_loaded = True
            # self.logger.info("[__update_params]: parametes are loaded") # for debug
            # \\ if pybullet simulation time step is changed, update the timer_period_sec of the pybullet timer //
            if self._dt != dt_prev:
                self.pybullet_timer.timer_period_ns = int(float(self._dt) * 1000 * 1000 * 1000)
        except:
            self.logger.error("could not load parameters")
            # self._is_params_loaded = False

    def __pybullet_sim_timer_callback(self):
        # \\ if pybullet is not initialized, initiate it //
        if not self._is_pybullet_initialized:
            self.is_pybullet_running = False
            self.__init_pybullet_sim()
        # \\ else, run the simulation
        else:
            if self._is_debug:
                self.update_userDebugParams()
            # \\ update joint positions //
            self.update_joint_positions()
            # \\ make one step of simulation per on time beat of the timer //
            p.stepSimulation()
            # \\ make is_pybullet_running flag True //
            self.is_pybullet_running = True
            # \\ update joint states //
            self.update_joint_states()
            
            
    def __init_pybullet_sim(self):
        # \\ wait untill the parameters are loaded //
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
                # self.logger.info(robot_urdf_path) # for debug
                self._robot = p.loadURDF(robot_urdf_path, [0,0,0.008], [0,0,0,1], useFixedBase=False) # flags = urdfFlags
                # \\ set the robot to home position //
                self.set_robot_home_position() 
                # \\ print joint infos //
                for j in range(p.getNumJoints(self._robot)):
                    self.logger.info(f"[__init_pybullet_sim]: {p.getJointInfo(self._robot, j)}")
                # \\ enable collision between lower legs //
                for l0 in self._calf_joint_ids:
                    for l1 in self._calf_joint_ids:
                        if(l1>l0):
                            enableCollision = 1
                            print("collision for pair", l0, l1, p.getJointInfo(self._robot, l0)[12], p.getJointInfo(self._robot,l1)[12], "enabled=", enableCollision)
                            p.setCollisionFilterPair(self._robot, self._robot, 2,5,enableCollision)
                # \\ set joint dynamics //
                for j in range (p.getNumJoints(self._robot)):
                    p.changeDynamics(self._robot, j, linearDamping=0, angularDamping=0)
                # \\ enable torque sensors //
                [p.enableJointForceTorqueSensor(self._robot, i, True) for i in range(p.getNumJoints(self._robot))]
                # \\ set camera //
                p.getCameraImage(480,320)
                # \\ set real-time simulation //
                p.setRealTimeSimulation(0)
                # \\ set user debug parameters //
                if self._is_debug:
                    self.add_userDebugParams()
                self._is_pybullet_initialized = True
                self.logger.info("[__init_pybullet_sim]: HyperDog pybullet simulation is launched")
                break
            else:
                self.logger.info("waiting for parametes are loaded")
                self.logger.info(f"is_params_loaded: {self._is_params_loaded}")
            time.sleep(1)

    def set_robot_home_position(self):
        torques = [self._max_motor_torque]*12
        positions = [p for p in self._leg_joint_home_positions]*4
        positions = [p*np.pi/180 for p in positions]
        self.logger.info(f"[set_robot_home_position]: joint positions {positions}")
        p.setJointMotorControlArray(self._robot, 
                                    range(12), 
                                    p.POSITION_CONTROL, 
                                    targetPositions=positions,
                                    forces=torques)
    
    def add_userDebugParams(self):
        self._maxForceId = p.addUserDebugParameter("maxForce", self._motor_torque_limit[0], self._motor_torque_limit[1], self._max_motor_torque)
        # ---
        self._heightId = p.addUserDebugParameter("Height",      self._robot_height_limit[0],       self._robot_height_limit[1], self._robot_height_limit[0])
        self._rollId = p.addUserDebugParameter("roll",   -self._robot_euler_angle_limits[0], self._robot_euler_angle_limits[0], 0)
        self._pitchId = p.addUserDebugParameter("pitch", -self._robot_euler_angle_limits[1], self._robot_euler_angle_limits[1], 0)
        self._yawId = p.addUserDebugParameter("yaw",     -self._robot_euler_angle_limits[2], self._robot_euler_angle_limits[2], 0)
        # ---
        self._moveXId = p.addUserDebugParameter('move_x', -150, 150, 0)
        self._moveYId = p.addUserDebugParameter('move_y', -150, 150, 0)
    
    def update_userDebugParams(self):
        pass

    def update_joint_states(self):
        if self.is_pybullet_running:
            joint_states = np.array(p.getJointStates(self._robot, [i for i in range(p.getNumJoints(self._robot))]))
            joint_positions = list(joint_states[:,0])
            self.current_joint_positions.data = joint_positions
    
    def update_joint_positions(self):
        pass #TODO
        
        


def main(args=None):
    rclpy.init(args=args)
    node = Ros2HyperdogPybulletNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
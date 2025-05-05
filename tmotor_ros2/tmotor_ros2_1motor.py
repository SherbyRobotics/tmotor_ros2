#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from motor_driver.canmotorlib import CanMotorController

import time
import numpy as np
from sensor_msgs.msg import JointState

#########################################
class TmotorDriverNode(Node):

    #######################################    
    def __init__(self):
        super().__init__("tmotors")
        # Init subscribers 
        self.sub_cmd = self.create_subscription(JointState, "joints_cmd", self.cmd_received, 10)
        
        # Init publishers
        self.pub_sensor = self.create_publisher(JointState, "joints_sensor", 10)
    
        self.time_now = 0.0
        self.time_last = 0.0
        self.cmd_id = 1
        
        #################
        # Parameters
        #################
        self.offline_debug = False
        self.declare_parameter('inverted', False)
        self.inverted = self.get_parameter('inverted').get_parameter_value().bool_value
        logMsg = 'inverted axes = ' + str(self.inverted)
        self.get_logger().info(logMsg)

        #################
        # Motor init
        #################

        motor1_id = 0x01
        if self.inverted:
            motor1_id = 0x02

        print("Initializing motor")
        self.tmotor = CanMotorController(can_socket='can0', motor_id=motor1_id, socket_timeout=0.5)
        self.tmotor.change_motor_constants(-12.5, 12.5, -41.0, 41.0, 0, 500, 0, 50, -9.0, 9.0)
        self.tmotor_params = {'kp': 5, 'kd': 5, 'vel_kp': 5, 'vel_ki': 2}

        #################
        # Memory
        #################
        # cmd for motor
        self.motor_cmd_mode = 'disable'
        self.motor_cmd_pos  = 0.0
        self.motor_cmd_vel  = 0.0
        self.motor_cmd_tor  = 0.0

        # sensor feedback
        self.motor_name       = 'Joint 0'
        self.motor_sensor_pos = 0.0
        self.motor_sensor_vel = 0.0
        self.motor_sensor_tor = 0.0

    ####################################### 
    def cmd_received(self, JointState):
        """Handle received commands"""
        id = self.cmd_id
        self.motor_cmd_mode = JointState.name[id]  # Mode is in the name field
        self.motor_cmd_pos  = JointState.position[id]
        self.motor_cmd_vel  = JointState.velocity[id]
        self.motor_cmd_tor  = JointState.effort[id]
        
        # Main loop is here
        self.send_cmd_to_motor()

    ##########################################################################################
    def send_cmd_to_motor(self):
        """Send commands to motor and get sensor feedback"""

        self.time_now  = time.time()
        dt = float(self.time_now - self.time_last) # loop period            
        self.time_last = self.time_now #
        
        if self.offline_debug:
            # Kinematic model for debug
            if self.motor_cmd_mode == 'position':
                self.motor_sensor_pos = self.motor_cmd_pos
            elif self.motor_cmd_mode == 'velocity':
                self.motor_sensor_vel = self.motor_cmd_vel
                self.motor_sensor_pos += self.motor_cmd_vel * dt
            elif self.motor_cmd_mode == 'torque':
                self.motor_sensor_tor = self.motor_cmd_tor
                self.motor_sensor_vel += self.motor_cmd_tor * dt
                self.motor_sensor_pos += self.motor_sensor_vel * dt
        
        else:
            # Axis limit checks for motor
            if self.motor_sensor_pos > 6.5 and self.motor_cmd_vel > 0:
                self.motor_cmd_vel = 0.0
            if self.motor_sensor_pos < -6.5 and self.motor_cmd_vel < 0:
                self.motor_cmd_vel = 0.0

            if self.motor_cmd_mode == 'disable':
                self.tmotor.disable_motor()
            elif self.motor_cmd_mode == 'enable':
                self.tmotor.enable_motor()
                self.tmotor.set_zero_position()
            elif self.motor_cmd_mode == 'position':
                self.motor_sensor_pos, self.motor_sensor_vel, self.motor_sensor_tor = self.tmotor.send_rad_command(
                    self.motor_cmd_pos, 0, self.tmotor_params['kp'], self.tmotor_params['kd'], 0)
            elif self.motor_cmd_mode == 'velocity':
                self.motor_sensor_pos, self.motor_sensor_vel, self.motor_sensor_tor = self.tmotor.send_rad_command(
                    0, self.motor_cmd_vel, 0, self.tmotor_params['kd'], 0)
            elif self.motor_cmd_mode == 'torque':
                self.motor_sensor_pos, self.motor_sensor_vel, self.motor_sensor_tor = self.tmotor.send_rad_command(
                    0, 0, 0, 0, self.motor_cmd_tor)

        self.publish_sensor_data()

    ##########################################################################################
    def publish_sensor_data(self):
        """Publish sensor data"""
        motors_msg = JointState()

        motors_msg.name     = [self.motor_name]
        motors_msg.position = [self.motor_sensor_pos]
        motors_msg.velocity = [self.motor_sensor_vel]
        motors_msg.effort   = [self.motor_sensor_tor]

        # Publish msg
        self.pub_sensor.publish(motors_msg)

#########################################
def main(args=None):
    rclpy.init(args=args)
    node = TmotorDriverNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

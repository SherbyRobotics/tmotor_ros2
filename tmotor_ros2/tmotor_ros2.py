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
    
        # Timer
        #self.dt         = 0.02
        #self.timer      = rclpy.Timer( rospy.Duration( self.dt ), self.timed_controller )

        self.time_now = 0.0
        self.time_last = 0.0
        
        #################
        # Paramters
        #################
        
        self.offline_debug = False
        # Inside your class or function where you want to get the parameter value
        self.declare_parameter('inverted', rclpy.Parameter.Type.BOOL)
        self.inverted = self.get_parameter('inverted').get_parameter_value().bool_value
        logMsg = 'inverted axes = ' + str(self.inverted)
        self.get_logger().info(logMsg)

        #################
        # Motors init
        #################


        motor1_id = 0x01
        motor2_id = 0x02
        if self.inverted:
            motor1_id = 0x02
            motor2_id = 0x01

        print("motors call CanMotorController pour les 2 motors")
        self.tmotors = [CanMotorController(can_socket='can0', motor_id=motor1_id, socket_timeout=0.5), CanMotorController(can_socket='can0', motor_id=motor2_id, socket_timeout=0.5)]
        self.tmotors[0].change_motor_constants(-12.5, 12.5, -41.0, 41.0, 0, 500, 0, 50, -9.0, 9.0)
        self.tmotors[1].change_motor_constants(-12.5, 12.5, -41.0, 41.0, 0, 500, 0, 50, -9.0, 9.0)
        self.tmotors_params = [ {'kp': 20, 'kd': 5, 'vel_kp': 5, 'vel_ki': 2} , {'kp': 20, 'kd': 5, 'vel_kp': 5, 'vel_ki': 2} ]
        

        #################
        # Memory
        #################
        
        # cmd for tmotors
        self.motors_cmd_mode = ['disable','disable']
        self.motors_cmd_pos  = [ 0.0 , 0.0 ]
        self.motors_cmd_vel  = [ 0.0 , 0.0 ]
        self.motors_cmd_tor  = [ 0.0 , 0.0 ]
        
        # sensor feedback
        self.motors_names       = ['Joint 0','Joint 1']
        self.motors_sensor_pos  = [ 0.0 , 0.0 ]
        self.motors_sensor_vel  = [ 0.0 , 0.0 ]
        self.motors_sensor_tor  = [ 0.0 , 0.0 ]
        

    ####################################### 
    def cmd_received( self, JointState ):
        """ """
        
        self.motors_cmd_mode = JointState.name # Mode is in the name field
        self.motors_cmd_pos  = JointState.position
        self.motors_cmd_vel  = list(JointState.velocity)
        self.motors_cmd_tor  = list(JointState.effort)
        
        # Main loop is here
        self.send_cmd_to_tmotors()
        
      
    ##########################################################################################
    def send_cmd_to_tmotors(self):
        """ """
        

        self.time_now  = time.time()
        dt = float(self.time_now - self.time_last) # loop period            
        self.time_last = self.time_now #
        
        if self.offline_debug:
            
            # Kinematic model for debug
            

            logMsg = str(dt)                
            # self.get_logger().info(logMsg)
            
            for i in range(2):
                
                #################################################
                if self.motors_cmd_mode[i] == 'position':
                    
                    self.motors_sensor_pos[i] = self.motors_cmd_pos[i]
                    
                #################################################  
                elif self.motors_cmd_mode[i] == 'velocity':
                    
                    self.motors_sensor_vel[i] = self.motors_cmd_vel[i]
                    self.motors_sensor_pos[i] = self.motors_sensor_pos[i] + self.motors_cmd_vel[i] * dt
                    
                #################################################   
                elif self.motors_cmd_mode[i] == 'torque':
                    
                    self.motors_sensor_tor[i] = self.motors_cmd_tor[i]
                    self.motors_sensor_vel[i] = self.motors_sensor_vel[i] + self.motors_cmd_tor[i] * dt
                    self.motors_sensor_pos[i] = self.motors_sensor_pos[i] + self.motors_sensor_vel[i] * dt
                    
        
        else:
            
            # axis limit for motor 1 : [-1 turn, 1 turn]
            if self.motors_sensor_pos[1] > 6.5 and self.motors_cmd_vel[1] > 0:
                self.motors_cmd_vel[1] = 0.0
            if self.motors_sensor_pos[1] < -6.5 and self.motors_cmd_vel[1] < 0:
                self.motors_cmd_vel[1] = 0.0

            if self.motors_sensor_pos[1] > 6.5 and self.motors_cmd_tor[1] > 0:
                self.motors_cmd_tor[1] = 0.0
            if self.motors_sensor_pos[1] < -6.5 and self.motors_cmd_tor[1] < 0:
                self.motors_cmd_tor[1] = 0.0
            
            # Send commands to both motor and read sensor data
            for i in range(2):
                logMsg = ("motor # "+ str(i) + str(self.motors_cmd_vel[i]))               
                # self.get_logger().info(logMsg)
                #################################################
                if self.motors_cmd_mode[i] == 'disable':

                    self.tmotors[i].disable_motor()

                #################################################
                elif self.motors_cmd_mode[i] == 'enable':

                    self.tmotors[i].enable_motor()
                    self.tmotors[i].set_zero_position()

                #################################################
                elif self.motors_cmd_mode[i] == 'position':

                    self.motors_sensor_pos[i] , self.motors_sensor_vel[i], self.motors_sensor_tor[i] = self.tmotors[i].send_rad_command(self.motors_cmd_pos[i], 0, self.tmotors_params[i]['kp'], self.tmotors_params[i]['kd'], 0)
                    
                #################################################  
                elif self.motors_cmd_mode[i] == 'velocity':          
                    
                    self.motors_sensor_pos[i] , self.motors_sensor_vel[i], self.motors_sensor_tor[i] = self.tmotors[i].send_rad_command(0, self.motors_cmd_vel[i], 0, self.tmotors_params[i]['kd'], 0)
                    
                #################################################   
                elif self.motors_cmd_mode[i] == 'torque':
                    
                    self.motors_sensor_pos[i] , self.motors_sensor_vel[i], self.motors_sensor_tor[i] = self.tmotors[i].send_rad_command(0, 0, 0, 0, self.motors_cmd_tor[i])
                    
                #################################################   
                elif self.motors_cmd_mode[i] == 'damped_torque':
                    
                    self.motors_sensor_pos[i] , self.motors_sensor_vel[i], self.motors_sensor_tor[i] = self.tmotors[i].send_rad_command(0, 0, 0, 2.0, self.motors_cmd_tor[i])
        
                #################################################   
                elif self.motors_cmd_mode[i] == 'velocity_plus_torque':
                    
                    self.motors_sensor_pos[i] , self.motors_sensor_vel[i], self.motors_sensor_tor[i] = self.tmotors[i].send_rad_command(0, self.motors_cmd_vel[i], 0, self.tmotors_params[i]['kd'], self.motors_cmd_tor[i])
                
                #################################################   
                elif self.motors_cmd_mode[i] == 'position_velocity_torque':
                    
                    self.motors_sensor_pos[i] , self.motors_sensor_vel[i], self.motors_sensor_tor[i] = self.tmotors[i].send_rad_command(self.motors_cmd_pos[i], self.motors_cmd_vel[i], self.tmotors_params[i]['vel_ki'], self.tmotors_params[i]['vel_kp'], self.motors_cmd_tor[i])
        
        
        # logMsg = str(1/(time.time() - self.time_now))                
        # self.get_logger().info(logMsg)
        self.publish_sensor_data()

    ##########################################################################################
    def publish_sensor_data(self):
        """ """
        #Init msg
        motors_msg = JointState()

        motors_msg.name     = self.motors_names
        motors_msg.position = self.motors_sensor_pos
        motors_msg.velocity = self.motors_sensor_vel
        motors_msg.effort   = self.motors_sensor_tor

        # Publish msg
        self.pub_sensor.publish( motors_msg )
        

#########################################
def main(args=None):
    rclpy.init(args=args)
    node = TmotorDriverNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


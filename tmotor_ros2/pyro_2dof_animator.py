#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
import matplotlib
import matplotlib.pyplot as plt

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

import pyro
from pyro.dynamic  import manipulator
from pyro.dynamic  import pendulum


#########################################
class PyroRobotAnimatorNode(Node):

    #######################################    
    def __init__(self):

        super().__init__("pyro_controller")
        # Init subscribers  
        self.sub_sensor = self.create_subscription(JointState, "joints_sensor", self.read_joints, 1) 
        
        # Graphic output Timer
        self.dt2        = 0.1
        self.timer2     = self.create_timer( self.dt2, self.timed_graphic )
    

        #################
        # Paramters
        #################
        
        # Robot model
        self.sys         = pendulum.DoublePendulum()
        self.sys.l1      = 0.3
        self.sys.l2      = 0.3
        self.sys.lc1     = 0.3
        self.sys.lc2     = 0.3
        self.sys.I1      = 0.5
        self.sys.I2      = 0.2
        self.sys.m1      = 0.9
        self.sys.m2      = 0.05
        self.sys.u_lb[0] = -1.0
        self.sys.u_lb[0] = -1.0
        self.sys.u_ub[0] = 1.0
        self.sys.u_ub[0] = 1.0
        

        #################
        # Memory
        #################
        
        # Robot command
        self.u  = np.array( [ 0.0 , 0.0 ])
        
        # Sensings inputs
        self.x  = np.array([ 0.0, 0.0, 0.0, 0.0]) # State of the system
        self.q  = np.array([ 0.0, 0.0]) # Joint angles of the system
        self.dq = np.array([ 0.0, 0.0]) # Joint velocities of the system
        
        #################
        # Initialization
        #################
        
        # Start loop
        #self.pubish_joints_cmd_msg()
        
        # Start graphic
        self.animator = self.sys.get_animator()
        self.sys.l_domain = 0.6
        #self.animator.show( self.q )
        self.animator.show_plus( self.x , self.u, 0 )
        self.animator.showfig.canvas.draw()
        plt.show(block=False)


    ####################################### 
    def read_joints( self, msg):
        
        """
        self.x[0] = msg.position[0]
        self.x[1] = msg.position[1]
        self.x[2] = msg.velocity[0]
        self.x[3] = msg.velocity[1]
        """
        
        self.q  = np.array([ msg.position[1] - 3.1415 , msg.position[0] ])
        self.dq = np.array([ msg.velocity[1] , msg.velocity[0] ])
        
        self.x  = self.sys.q2x( self.q , self.dq )
        self.u  = np.array([ msg.effort[1] , msg.effort[0] ])
        
        
    #######################################
    def timed_graphic(self):
        
        """
        lines_pts = self.sys.forward_kinematic_lines( self.q )[0]
        robot_line = lines_pts[1]
        self.animator.showlines[1].set_data( robot_line[:, 0 ], robot_line[:, 1 ])
        self.animator.showfig.canvas.draw()
        """
        #print(self.controller_mode)
        self.animator.show_plus_update( self.x, self.u, 0.0 )
        #print(self.x,self.u)
        plt.pause(0.01)


#########################################

def main(args=None):
    rclpy.init(args=args)
    plt.ion()
    matplotlib.use('Qt5Agg')
    #plt.ioff()
    node = PyroRobotAnimatorNode()
    
    #plt.show( block = True )
    #plt.show()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

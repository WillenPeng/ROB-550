import lcm
import time
import numpy as np
import sys
sys.path.append("lcmtypes")

from lcmtypes import dynamixel_command_t
from lcmtypes import dynamixel_command_list_t
from lcmtypes import dynamixel_status_t
from lcmtypes import dynamixel_status_list_t
from lcmtypes import dynamixel_config_t
from lcmtypes import dynamixel_config_list_t

"""Useful Conversions"""
PI = np.pi
D2R = PI/180.0
ANGLE_TOL = 2*PI/180.0 


""" Rexarm Class """
class Rexarm():
    def __init__(self):

        """ TODO: modify this class to add functionality you need """

        """ Commanded Values """
        self.num_joints = 6                         # number of motors, increase when adding gripper
        self.joint_angles = [0.0] * self.num_joints # radians
        
        # you must change this to an array to control each joint speed separately 
        self.speed = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]                         # 0 to 1
        self.max_torque = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]                    # 0 to 1
        self.speed_multiplier = 1
        self.torque_multiplier = 1

        """ Feedback Values """
        self.joint_angles_fb = [0.0] * self.num_joints # radians
        self.speed_fb = [0.0] * self.num_joints        # 0 to 1   
        self.load_fb = [0.0] * self.num_joints         # -1 to 1  
        self.temp_fb = [0.0] * self.num_joints         # Celsius               

        """ Waypoint Plan - TO BE USED LATER """
        self.plan = []
        self.plan_status = 0
        self.wpt_number = 0
        self.wpt_total = 0
        self.moving_status = 0
        """ Setup LCM and subscribe """
        self.lc = lcm.LCM()
        lcmMotorSub = self.lc.subscribe("DXL_STATUS",
                                        self.feedback_handler)

        self.dh_table = [[]]
        self.l1 = 119.5
        self.l2 = 97.70
        self.l3 = 98.03
        # self.offset = 12.5
        self.l45 = 115.8

        self.endeffector_global = np.array([])

    def cmd_publish(self):
        """ 
        Publish the commands to the arm using LCM.
        This publishes the joint angles currently set in the Rexarm object
        You need to call this function to command the arm.
        """    
        msg = dynamixel_command_list_t()
        msg.len = self.num_joints
        self.clamp()
        
        for i in range(msg.len):
            cmd = dynamixel_command_t()
            cmd.utime = int(time.time() * 1e6)
            cmd.position_radians = self.joint_angles[i]
            cmd.speed = self.speed_multiplier * self.speed[i]
            cmd.max_torque = self.torque_multiplier * self.max_torque[i]
            msg.commands.append(cmd)
        self.lc.publish("DXL_COMMAND",msg.encode())

    def cfg_publish_default(self):
        """ 
        Publish default configuration to arm using LCM. 
        """    
        msg = dynamixel_config_list_t()
        msg.len = self.num_joints
        for i in range(msg.len):
            cfg = dynamixel_config_t()
            cfg.utime = int(time.time() * 1e6)
            cfg.kp = 32
            cfg.ki = 0
            cfg.kd = 0
            cfg.compl_margin = 0
            cfg.compl_slope = 16
            cfg.LED = 1
            msg.configs.append(cfg)
        self.lc.publish("DXL_CONFIG",msg.encode())

    def cfg_publish(self):
        """ 
        TODO: implement this function (optional)

        Publish configuration to arm using LCM. 
        You need to activelly call this function to command the arm.
        """    
        pass
    
    def get_feedback(self):
        """
        LCM Handler function
        Called continuously with timer by control_station.py
        times out after 10ms
        """
        self.lc.handle_timeout(10)

    def feedback_handler(self, channel, data):
        """
        Feedback Handler for LCM
        this is run when a feedback message is recieved
        """
        msg = dynamixel_status_list_t.decode(data)
        for i in range(msg.len):
            self.joint_angles_fb[i] = msg.statuses[i].position_radians 
            self.speed_fb[i] = msg.statuses[i].speed 
            self.load_fb[i] = msg.statuses[i].load 
            self.temp_fb[i] = msg.statuses[i].temperature

    def clamp(self):
        """
        TODO: implement this function

        Clamp Function
        Limit the commanded joint angles to ones physically possible 
        so the arm is not damaged.
        """
        if self.joint_angles[1] > 127 * D2R or self.joint_angles[1] < -121 * D2R:
            self.joint_angles = [0.] * self.num_joints
            self.plan = []
            self.moving_status = 1
            print "not possible to move here 1"
            return
        if self.joint_angles[2] > 150 * D2R or self.joint_angles[2] < -121 * D2R:
            self.joint_angles = [0.] * self.num_joints
            self.plan = []
            self.moving_status = 1
            print "not possible to move here 2"
            return
        if self.joint_angles[3] > 107 * D2R or self.joint_angles[3] < -121 * D2R:
            self.joint_angles = [0.] * self.num_joints
            self.plan = []
            self.moving_status = 1
            print "not possible to move here 3"
            return
        if self.joint_angles[5] > 45 * D2R or self.joint_angles[5] < -90 * D2R:
            self.joint_angles = [0.] * self.num_joints
            self.plan = []
            self.moving_status = 1
            print "not possible to move here 5"
            return
        if self.rexarm_collision_check(self.joint_angles[0:4]):
            return 
        else:
            self.joint_angles = [0.] * self.num_joints
            self.plan = []
            self.moving_status = 1
            print "collision will happen"

    def make_dhtable(self):
    
        # theta_i, d_i(mm), a_i(mm), alpha_i
        # dh_table = [[self.joint_angles_fb[0] - np.pi/2, self.l1, 0, np.pi/2],
        #                  [np.pi/2-self.joint_angles_fb[1], 0, self.l2, 0],
        #                  [-self.joint_angles_fb[2], 0, self.l3, 0],
        #                  [-np.pi/2-self.joint_angles_fb[3], self.offset, 0, -np.pi/2],
        #                  [-self.joint_angles_fb[4], self.l45, 0, 0]
        #             ]
        dh_table = [[self.joint_angles_fb[0], self.l1, 0, np.pi/2],
                         [np.pi/2+self.joint_angles_fb[1], 0, self.l2, 0],
                         [self.joint_angles_fb[2], 0, self.l3, 0],
                         [self.joint_angles_fb[3], 0, self.l45, 0]
                    ]
        return dh_table

    def dhtable_to_T_matrix(self,dh_table):
        T_matrix = np.identity(4)
        for i in range(len(dh_table)):
            theta_i = dh_table[i][0]
            d_i = dh_table[i][1]
            a_i = dh_table[i][2]
            alpha_i = dh_table[i][3]
            T_matrix = np.dot(T_matrix, np.array([[np.cos(theta_i), -np.sin(theta_i)*np.cos(alpha_i), np.sin(theta_i)*np.sin(alpha_i), a_i*np.cos(theta_i)],
                                  [np.sin(theta_i), np.cos(theta_i)*np.cos(alpha_i), -np.cos(theta_i)*np.sin(alpha_i), a_i*np.sin(theta_i)],
                                  [0 , np.sin(alpha_i), np.cos(alpha_i), d_i],
                                  [0, 0, 0, 1]]))
        
        return T_matrix

    def rexarm_FK(self):
        

        """
        TODO: implement this function

        Calculates forward kinematics for rexarm
        takes a DH table filled with DH parameters of the arm
        and the link to return the position for
        returns a 4-tuple (x, y, z, phi) representing the pose of the 
        desired link
        """

        endeffector_local = np.array([0, 0, 0, 1])
        dh_table = self.make_dhtable()
        T_matrix = self.dhtable_to_T_matrix(dh_table)

        endeffector_global_xyz = np.dot(T_matrix, endeffector_local)
        phi_angle = self.joint_angles_fb[1] + self.joint_angles_fb[2] + self.joint_angles_fb[3]

        self.endeffector_global = np.append(endeffector_global_xyz[:-1], phi_angle)
        
        
    def rexarm_IK(self, pose, cfg):
        """
        TODO: implement this function
        
        Calculates inverse kinematics for the rexarm
        pose is a tuple (x, y, z, phi) which describes the desired
        end effector position and orientation.  
        cfg describe elbow down (0) or e
        lbow up (1) configuration
        returns a 4-tuple of joint angles or NONE if configuration is impossible
        """
        #####################################
        x = -pose[0]
        y = -pose[1]
        z = pose[2]
        phi_angle = pose[3]
        r = np.sqrt(x**2 + y**2) - 5
        d24 = np.sqrt((z - self.l1 + self.l45)**2 + r**2)
        # d25 = np.sqrt((z - self.l1)**2 + r**2)
        a1_temp = 0
        a0 = 0
        a1 = 0
        a2 = 0
        a3 = 0
        if d24 < self.l2 + self.l3:
            a0 = np.arctan2(y,x)
            a1_temp = np.arccos((self.l2**2 +d24**2 - self.l3**2)/(2*self.l2*d24))
            # print (self.l2**2 +d24**2 - self.l3**2)/(2*self.l2*d24)
            a1 = np.pi/2 - np.arctan((z-self.l1+self.l45)/r) -a1_temp
            a2 = np.pi - np.arccos((self.l2**2 + self.l3**2 -d24**2)/(2*self.l2*self.l3))
            a3 = np.pi - a1 - a2
            # print "come in 1"
        else:
            r = np.sqrt(x**2 + y**2) - 10
            d24 = np.sqrt((z - self.l1)**2 + (r - self.l45)**2)
            a0 = np.arctan2(y,x)
            if (self.l2**2 +d24**2 - self.l3**2)/(2*self.l2*d24) > 1:
                a1_temp = 0
            else:
                a1_temp = np.arccos((self.l2**2 +d24**2 - self.l3**2)/(2*self.l2*d24))
            if (self.l3**2 +d24**2 - self.l2**2)/(2*self.l3*d24) > 1:
                a1_temp_right = 0
            else:
                a1_temp_right = np.arccos((self.l3**2 +d24**2 - self.l2**2)/(2*self.l3*d24))
            a1 = np.pi/2 - np.arctan((z-self.l1)/(r-self.l45)) -a1_temp
            if (self.l2**2 + self.l3**2 -d24**2)/(2*self.l2*self.l3) > 1 or (self.l2**2 + self.l3**2 -d24**2)/(2*self.l2*self.l3) < -1:
                a2 = 0
            else:
                a2 = np.pi - np.arccos((self.l2**2 + self.l3**2 -d24**2)/(2*self.l2*self.l3))

            a3 = -(a1_temp_right - np.arctan((z-self.l1)/(r-self.l45)))

            # print "come in 2"

        print [a0, a1, a2, a3]
        # elif d25 < self.l2 + self.l3 + self.l45:
        #     a0 = np.arctan2(y,x)
        #     # print ((self.l2 + self.l3)**2 + d25**2 - self.l45**2)/(2 * (self.l2 + self.l3) *d25)
        #     a1_temp = np.arccos(((self.l2 + self.l3)**2 + d25**2 - self.l45**2)/(2 * (self.l2 + self.l3) *d25))
        #     a1 = np.pi/2 - np.arctan((z-self.l1)/r) -a1_temp
        #     a2 = 0
        #     a3 = np.pi - np.arccos(((self.l2 + self.l3)**2 + self.l45**2 - d25**2)/(2*(self.l3+self.l45)*self.l45))
        # else:
        #     a0 = 0
        #     a1 = 0
        #     a2 = 0
        #     a3 = 0
        # print "inverse_____________"
        # print d24
        # print z
        # print x
        # print y
        return [a0, a1, a2, a3]

        #####################################
        # x = -pose[0]
        # y = -pose[1]
        # z = pose[2]
        # phi_angle = pose[3]

        # new_x = np.sqrt(x**2 + y**2)
        # new_y = z + self.l45 - self.l1

        # c2 = (new_x**2 + new_y**2 - self.l2**2 - self.l3**2)/(2*self.l2*self.l3)

        # if cfg == 0:  # elbow down
        #     s2 = -np.sqrt(1 - c2**2)
        # else:
        #     s2 = np.sqrt(1 - c2**2)

        # k1 = self.l2 + self.l3 * c2
        # k2 = self.l3 * s2

        # a0 = np.arctan2(y, x)
        # a1 = np.arctan2(new_y, new_x) - np.arctan2(k2, k1)
        # a2 = np.arctan2(s2, c2)
        # a3 = phi_angle - a1 - a2

        # return [a0, -a1, a2, a3]
    def rexarm_collision_check(self, q):
        """
        TODO: implement this function

        Perform a collision check with the ground and the base of the Rexarm
        takes a 4-tuple of joint angles q
        returns true if no collision occurs
        """
        dh_table = [[q[0], self.l1, 0, np.pi/2],
                 [np.pi/2+q[1], 0, self.l2, 0],
                 [q[2], 0, self.l3, 0],
                 [q[3], 0, self.l45, 0]
            ]
        endeffector_global = np.array([])
        endeffector_local = np.array([0, 0, 0, 1])
        dh_table = self.make_dhtable()
        T_matrix = self.dhtable_to_T_matrix(dh_table)

        endeffector_global_xyz = np.dot(T_matrix, endeffector_local)
        phi_angle = q[1] + q[2] + q[3]

        endeffector_global = np.append(endeffector_global_xyz[:-1], phi_angle) 
        if endeffector_global[2] < 10:
            print "111"
            return False
        if endeffector_global[0] ** 2 + endeffector_global[1] ** 2 < 30 ** 2 and endeffector_global[2] < 30:
            print "222"
            return False
        return True

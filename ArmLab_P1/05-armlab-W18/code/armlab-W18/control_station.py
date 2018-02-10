#!/usr/bin/env python
import sys
import cv2
import numpy as np
from PyQt4 import QtGui, QtCore, Qt
from ui import Ui_MainWindow
from rexarm import Rexarm
from video import Video
from statemachine import Statemachine
""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592
SPEED_MX28 = 5.76
SPEED_AX12 = 6.18
SPEED_XL320 = 11.94
""" Pyxel Positions of image in GUI """
MIN_X = 310
MAX_X = 950

MIN_Y = 30
MAX_Y = 510

EVEN2_X = 0
EVEN2_Y = 230 
space_half = 23
space = 46
EVENT3_Y = [-space_half - 3 * space,
            -space_half - 2 * space,
            space_half, 
            space_half + 1 * space,
            -space_half,
            space_half + 3 * space,
            -space_half - space,
            space_half + 2 * space
            ]
EVENT3_X = 100

EVENT4_X = 230 * np.cos(5 * D2R)
EVENT4_Y = 230 * np.sin(5 * D2R)
EVENT4_R = 240
EVENT4_YF = [EVENT4_R *  np.cos(20*D2R),
             (EVENT4_R) *  np.cos(40*D2R),
             EVENT4_R *  np.cos(60*D2R),
             EVENT4_R *  np.cos(170*D2R),
             EVENT4_R *  np.cos(120*D2R)
             ]
EVENT4_XF = [EVENT4_R *  np.sin(20*D2R),
             (EVENT4_R) *  np.sin(40*D2R),
             EVENT4_R *  np.sin(60*D2R),
             EVENT4_R *  np.sin(170*D2R),
             EVENT4_R *  np.sin(120*D2R)
             ]

RE_COLOR  = [0,1,6,4,2,3,7,5]
Record_Value = [[]]
xp = 0
yp = 0
hp = 0
EVENT5_X = []
EVENT5_Y = []
EVENT5_Z = []
EVENT5_R = 230
EVENT5_ANG = 13.5
EVENT5_ANG_OS = -EVENT5_ANG * 3
EVENT_ANG_GAP = 6
idx = 0
# for i in range(8):
#     for j in range(8 - i):
#         EVENT5_X.append(EVENT5_R * np.sin((-j * EVENT5_ANG + EVENT5_ANG_OS - EVENT_ANG_GAP * i) * D2R))
#         EVENT5_Y.append(EVENT5_R * np.cos((-j * EVENT5_ANG + EVENT5_ANG_OS - EVENT_ANG_GAP * i) * D2R))
#         EVENT5_Z.append(i)
for i in range(7):
    for j in range(7 - i):
        EVENT5_X.append(EVENT5_R * np.sin((-j * EVENT5_ANG + EVENT5_ANG_OS - EVENT_ANG_GAP * i) * D2R))
        EVENT5_Y.append(EVENT5_R * np.cos((-j * EVENT5_ANG + EVENT5_ANG_OS - EVENT_ANG_GAP * i) * D2R))
        EVENT5_Z.append(i)
# print EVENT5_X
# print EVENT5_Y

class Gui(QtGui.QMainWindow):
    """ 
    Main GUI Class
    contains the main function and interfaces between 
    the GUI and functions
    """
    def __init__(self,parent=None):
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        """ Main Variables Using Other Classes"""
        self.rex = Rexarm()
        self.video = Video()
        self.statemachine = Statemachine()

        self.pos_idx_teach = 0

        """ Other Variables """
        self.last_click = np.float32([0,0])
        self.check_move_time = 60
        self.check_time = 70
        self.check_move_counter = 0
        self.d_t = 10. * self.check_move_time / 1000.
        self.spe_coef = [0.0] * self.rex.num_joints
        self.pos_pre = [0.0] * self.rex.num_joints
        self.plan_point = 0
        self.move_to_point_cn = 0
        self.event3_color_check = [0] * 8
        self.event3_finish = 0
        self.event4_color_idx = 0
        self.vel = []
        self.event4_f_idx = 0
        self.event5_block_num = 0
        """ Set GUI to track mouse """
        QtGui.QWidget.setMouseTracking(self,True)

        """ 
        GUI timer 

        Creates a timer and calls update_gui() function 
        according to the given time delay (27ms) 
        """
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.update_gui)
        self._timer.start(27)
       
        """ 
        LCM Arm Feedback timer
        Creates a timer to call LCM handler for Rexarm feedback
        """  
        self._timer2 = QtCore.QTimer(self)
        self._timer2.timeout.connect(self.rex.get_feedback)
        self._timer2.start()
        # self._timer_setplan = QtCore.QTimer(self)
        # self._timer_setplan.timeout.connect(self.setplan)
        # self._timer_teach = QtCore.QTimer(self)
        # self._timer_teach.timeout.connect(self.check_move)
        self._timer_waypoint = QtCore.QTimer(self)
        self._timer_waypoint.timeout.connect(self.move_to_point)
        self._timer_event1 = QtCore.QTimer(self)
        self._timer_event2= QtCore.QTimer(self)
        self._timer_event3 = QtCore.QTimer(self)
        self._timer_event4 = QtCore.QTimer(self)
        self._timer_event5 = QtCore.QTimer(self)
        """ 
        Connect Sliders to Function
        TODO: CONNECT THE OTHER 5 SLIDERS IMPLEMENTED IN THE GUI 
        """ 
        self.ui.sldrBase.valueChanged.connect(self.sliderChange)
        self.ui.sldrShoulder.valueChanged.connect(self.sliderChange)
        self.ui.sldrElbow.valueChanged.connect(self.sliderChange)
        self.ui.sldrSpeed.valueChanged.connect(self.sliderChange)
        self.ui.sldrWrist.valueChanged.connect(self.sliderChange)
        self.ui.sldrGrip1.valueChanged.connect(self.sliderChange)
        self.ui.sldrGrip2.valueChanged.connect(self.sliderChange)
        self.ui.sldrMaxTorque.valueChanged.connect(self.sliderChange)
        """ Initial command of the arm to home position"""
        self.sliderChange() 
        
        """ Connect Buttons to Functions 
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        self.ui.btnUser1.setText("Configure Servos")
        self.ui.btnUser1.clicked.connect(self.rex.cfg_publish_default)
        self.ui.btnUser2.setText("Simple Calibration")
        self.ui.btnUser2.clicked.connect(self.simple_cal)
        # self.ui.btnUser3.setText("start teach and repeat")
        # self.ui.btnUser3.clicked.connect(self.teach_repeat_start)
        # self.ui.btnUser4.setText("record position")
        # self.ui.btnUser4.clicked.connect(self.teach_repeat_record)
        # self.ui.btnUser5.setText("play teach and repeat")
        # self.ui.btnUser5.clicked.connect(self.teach_repeat_play)
        # self.ui.btnUser6.setText("save rgb")
        # self.ui.btnUser6.clicked.connect(self.save_rgb)
        self.ui.btnUser3.setText("EVENT1")
        self.ui.btnUser3.clicked.connect(self.event1)
        self.ui.btnUser4.setText("EVENT2")
        self.ui.btnUser4.clicked.connect(self.event2)
        self.ui.btnUser5.setText("EVENT3")
        self.ui.btnUser5.clicked.connect(self.event3)
        self.ui.btnUser6.setText("EVENT4")
        self.ui.btnUser6.clicked.connect(self.event4)
        self.ui.btnUser7.setText("EVENT5")
        self.ui.btnUser7.clicked.connect(self.event5)
        self.ui.btnUser8.setText("depth test")
        self.ui.btnUser8.clicked.connect(self.video.blockDetector)
        self.ui.btnUser9.setText("recover")
        self.ui.btnUser9.clicked.connect(self.recover)
        self.ui.btnUser10.setText("click put")
        self.ui.btnUser10.clicked.connect(self.click_put)
        self.ui.btnUser12.setText("Emergency Stop!")
        self.ui.btnUser12.clicked.connect(self.estop)

    # def save_rgb(self):
    #     img = cv2.cvtColor(self.video.currentVideoFrame, cv2.COLOR_RGB2BGR)
    #     cv2.imwrite('real_example.png',img)
    # def teach_repeat_start(self):
    #     print "i enter"
    #     global Record_Value 
    #     Record_Value = [[]]
    #     self.statemachine.idle(self.rex)
    #     # self.statemachine.moving(self.rex, self.rex.rexarm_IK([207,14, 30, np.pi/2],1) + [0.0, -10], [0.1,0.1,0.1,0.1,0.1,0.1])
    #     # print self.rex.rexarm_IK([-100,-100, 40, np.pi/2],1)

    # def teach_repeat_record(self):
    #     temp_record = [self.rex.joint_angles_fb[0],
    #     self.rex.joint_angles_fb[1],
    #     self.rex.joint_angles_fb[2],
    #     self.rex.joint_angles_fb[3],
    #     self.rex.joint_angles_fb[4]]
    #     if len(Record_Value[0]) == 0:
    #         Record_Value[0] = temp_record
    #     else :
    #         Record_Value.append(temp_record)

    # def teach_repeat_play(self):
    #     print Record_Value
    #     self._timer_teach.start(self.check_time)
    # # def check_move(self):
    #     print "111111"
    #     if len(Record_Value[0]) == 0:
    #         self.check_move_counter = 0 
    #         self._timer_teach.stop()
    #         return
    #     if self.check_move_counter == 0:
    #         T = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,2,0,0,0],[1,self.d_t,self.d_t**2,self.d_t**3,self.d_t**4,self.d_t**5],
    #             [0,1,2*self.d_t,3*self.d_t**2,4*self.d_t**3,5*self.d_t**4],[0,0,2,6*self.d_t,12*self.d_t**2,20*self.d_t**3]])
    #         for i in range(self.rex.num_joints):
    #             self.spe_coef[i] = np.dot(np.linalg.inv(T), np.array([[self.rex.joint_angles_fb[i]],[0.01],[0],
    #                 [Record_Value[self.pos_idx_teach][i]],[0.1],[0.]]))
    #         print "haha"
    #         print Record_Value[self.pos_idx_teach]
    #         print "fb"
    #         print self.rex.joint_angles_fb
    #         self.check_move_counter += 1
    #     pos = [0.] * self.rex.num_joints
    #     speed = [0.] * self.rex.num_joints
    #     for i in range(self.rex.num_joints):
    #         pos[i] = np.polynomial.polynomial.polyval((self.check_move_counter * self.check_move_time / 1000), self.spe_coef[i])
    #         if i == 3:
    #             speed[i] = abs(np.polynomial.polynomial.polyval((self.check_move_counter * self.check_move_time / 1000),
    #             np.polynomial.polynomial.polyder(self.spe_coef[i]))) / SPEED_AX12
    #         else: 
    #             speed[i] = abs(np.polynomial.polynomial.polyval((self.check_move_counter * self.check_move_time / 1000),
    #             np.polynomial.polynomial.polyder(self.spe_coef[i]))) / SPEED_MX28 * 20

    #     if self.check_move_counter * self.check_move_time / 1000. <= self.d_t and np.linalg.norm(np.transpose(np.asarray(Record_Value[self.pos_idx_teach])) - np.transpose(np.asarray(self.rex.joint_angles_fb))) > 0.1:
    #         # and np.linalg.norm(Record_Value[self.pos_idx_teach] - np.asarray(self.rex.joint_angles_fb)) > 0.1
    #         if self.pos_pre != pos:    
    #             self.statemachine.moving(self.rex, pos, speed)
    #         self.pos_pre = pos
    #         print "pos"
    #         print pos
    #         print "feedback"
    #         print self.rex.joint_angles_fb
    #         print "speed"
    #         print speed
    #         print self.check_move_counter
    #         # self.check_move_counter += 1
    #         if np.linalg.norm(np.transpose(np.asarray(pos)) - np.transpose(np.asarray(self.rex.joint_angles_fb)),axis = 1) < 0.01:
    #             self.check_move_counter += 1
    #     else:
    #         self.check_move_counter = 0
    #         if self.pos_idx_teach < len(Record_Value) - 1:
    #             self.pos_idx_teach += 1;
    #         else:
    #             print "i stop"
    #             self.pos_idx_teach = 0
    #             self._timer_teach.stop()
    #     print "222222"
    # def check_move(self):
    #     pos = 0
    #     if len(Record_Value[0]) == 0: 
    #         self._timer_teach.stop()
    #         return
    #     print len(Record_Value[self.pos_idx_teach]) 
    #     print np.linalg.norm(np.asarray(Record_Value[self.pos_idx_teach]) - np.asarray(self.rex.joint_angles_fb[0:5]))
    #     if np.linalg.norm(np.transpose(np.asarray(Record_Value[self.pos_idx_teach])) - np.transpose(np.asarray(self.rex.joint_angles_fb[0:5]))) > 0.04:
    #         pos = Record_Value[self.pos_idx_teach] + [0]
    #         self.statemachine.moving(self.rex, pos,[0.1,0.1,0.1,0.1,0.1,0.1])
    #         print pos
    #     else:
    #         if self.pos_idx_teach < len(Record_Value) - 1:
    #             self.pos_idx_teach += 1;
    #         else:
    #             print "i stop"
    #             self.pos_idx_teach = 0
    #             self._timer_teach.stop()
    def recover(self):
        self.rex.moving_status = 0
    def move_to_point(self):
        pos = 0
        if len(self.rex.plan) == 0: 
            self._timer_waypoint.stop()
            return
        # print len(self.rex.plan[self.pos_idx_teach]) 
        # print self.check_arrival(self.rex.plan[self.pos_idx_teach],self.rex.joint_angles_fb)
        if self.check_arrival(self.rex.plan[self.pos_idx_teach],self.rex.joint_angles_fb) > 0.07 and self.move_to_point_cn < 40:
            pos = self.rex.plan[self.pos_idx_teach]
            vel = self.vel[self.pos_idx_teach]
            self.statemachine.moving(self.rex, pos,vel)
            self.move_to_point_cn += 1
            # print "load load"
            # print self.rex.load_fb
            # print "going to pos:"
            # print pos
        else:
            if self.move_to_point_cn >= 70:
                print "cant move to that point within %d s" %(self.check_time * self.move_to_point_cn) 
            self.move_to_point_cn = 0
            if self.pos_idx_teach < len(self.rex.plan) - 1:
                self.pos_idx_teach += 1;
            else:
                print "i stop"
                self.pos_idx_teach = 0
                self.rex.plan = []
                self.rex.plan_status = 0
                self._timer_waypoint.stop()
    def click_grab(self):
        x = self.last_click[0]
        y = self.last_click[1]
        z = 0.
        world_z = 0.
        rw = np.array([[0],[0],[0]])
        M = self.video.aff_matrix
        M_RGBtoDEP = self.video.aff_matrix_RGBtoDEP;
        v = np.array([[x],[y],[1]])
        DEP_xy = np.dot(M_RGBtoDEP,v)
        if ((DEP_xy[0] >= 0) and (DEP_xy[0] <= 640) and (DEP_xy[1] >= 0) and (DEP_xy[1] <= 480)):
            world_z = self.video.depth - 123.6 * np.tan(self.video.currentDepthFrame[int(DEP_xy[1])][int(DEP_xy[0])]/2842.5 + 1.1863)
            z = self.video.currentDepthFrame[int(DEP_xy[1])][int(DEP_xy[0])]
            rw = np.dot(np.linalg.inv(self.video.WORLDtoRGB[:,[0,1,3]]),np.subtract(v,self.video.WORLDtoRGB[:,2:3] * (-world_z)))
        else:
            z = 0.0
        if rw[2] != 0:
            # self.statemachine.moving(self.rex, self.rex.rexarm_IK([rw[0]/rw[2],rw[1]/rw[2], world_z + 30, np.pi/2],1) + [0.0, -10], [0.1,0.1,0.1,0.1,0.1,0.1])
            t = self.rex.rexarm_IK([rw[0]/rw[2],rw[1]/rw[2], world_z - 9, np.pi],1)
            self.rex.plan = [[t[0],-200,-200,-200,0,-50*D2R],[-200,-200,t[2],t[3],0,-200],[-200,t[1],-200,-200,0,-200],[-200,-200,-200,-200,0,7*D2R],[-200,0,-200,-200,-200,-200],[0,0,0,0*D2R,0,-200]]
            self._timer_waypoint.start(self.check_time)
            print "plan"
            print self.rex.plan
        else :
            print "try again"

    def click_put(self):
        x = self.last_click[0]
        y = self.last_click[1]
        z = 0.
        world_z = 0.
        rw = np.array([[0],[0],[0]])
        M = self.video.aff_matrix
        M_RGBtoDEP = self.video.aff_matrix_RGBtoDEP;
        v = np.array([[x],[y],[1]])
        DEP_xy = np.dot(M_RGBtoDEP,v)
        if ((DEP_xy[0] >= 0) and (DEP_xy[0] <= 640) and (DEP_xy[1] >= 0) and (DEP_xy[1] <= 480)):
            world_z = self.video.depth - 123.6 * np.tan(self.video.currentDepthFrame[int(DEP_xy[1])][int(DEP_xy[0])]/2842.5 + 1.1863)
            z = self.video.currentDepthFrame[int(DEP_xy[1])][int(DEP_xy[0])]
            rw = np.dot(np.linalg.inv(self.video.WORLDtoRGB[:,[0,1,3]]),np.subtract(v,self.video.WORLDtoRGB[:,2:3] * (-world_z)))
        else:
            z = 0.0
        if rw[2] != 0:
            # self.statemachine.moving(self.rex, self.rex.rexarm_IK([rw[0]/rw[2],rw[1]/rw[2], world_z + 30, np.pi/2],1) + [0.0, -10], [0.1,0.1,0.1,0.1,0.1,0.1])
            t = self.rex.rexarm_IK([rw[0]/rw[2],rw[1]/rw[2], world_z + 40, np.pi/2],1)
            self.rex.plan = [[t[0],-200,-200,-200,0,-200],[-200,-200,t[2],t[3],0,-200],[-200,t[1],-200,-200,0,-200],[-200,-200,-200,-200,0,-20*D2R],[-200,0,-200,-200,-200,-200],[0,0,0,0*D2R,0,-200]]
            self._timer_waypoint.start(self.check_time)
            print "plan"
            print self.rex.plan
        else :
            print "try again"

    def check_arrival(self,pos0,pos1):
        t = 0
        for i in range(len(pos0)):
            if pos0[i] != -200:
                if i != 5:
                    t = t + (pos0[i] - pos1[i])**2
                else:
                    t = t + 0.25 * (pos0[i] - pos1[i])**2
        return np.sqrt(t)

    def event1(self):
        print "entering event1"
        self.video.det_pts_ = []
        self.video.blockDetector()
        self.plan_point = 0
        self._timer_event1.timeout.connect(self.event1_timer)
        self._timer_event1.start(self.check_time)

    def event2(self):
        print "entering event2"
        self._timer_event2.stop()
        self.video.det_pts_ = []
        self.video.blockDetector()
        self.plan_point = 0
        self._timer_event2.timeout.connect(self.event2_timer)
        self._timer_event2.start(self.check_time)

    def event3(self):
        print "entering event3"
        # self.video.blockDetector()
        self.video.det_pts_ = []
        self.event3_finish = 0
        self.plan_point = 0
        self.event3_color_check = [0] * 8
        self._timer_event3.timeout.connect(self.event3_timer)
        self._timer_event3.start(self.check_time)

    def event4(self):
        print "entering event4"
        # self.video.blockDetector()
        self.video.det_pts_ = []
        self.plan_point = 0
        self.event4_color_idx = 0
        self.event4_f_idx = 0
        self._timer_event4.timeout.connect(self.event4_timer)
        self._timer_event4.start(self.check_time)

    def event5(self):
        print "entering event5"
        # self.video.blockDetector()
        self.video.det_pts_ = []
        self.plan_point = 0
        self.event5_block_num = 0
        self._timer_event5.timeout.connect(self.event5_timer)
        self._timer_event5.start(self.check_time)
    
    def event1_timer(self):
        if self.rex.moving_status == 1:
                self._timer_event1.stop()
                self.rex.plan = []
                self.vel = []
                self._timer_waypoint.stop()

        if len(self.video.det_pts_) == 0:
            print "no block detected"
            self.plan_point = 0
            self._timer_event1.stop()
        else:
            if self.rex.plan_status == 0 and self.plan_point < len(self.video.det_pts_):
                x = self.video.det_pts_[self.plan_point].rgb_real_pos[0]
                y = self.video.det_pts_[self.plan_point].rgb_real_pos[1]
                print "grapping %d %d %d" %(self.plan_point, x, y)
                world_z = self.video.det_pts_[self.plan_point].depth
                rw = np.array([[0],[0],[0]])
                M = self.video.aff_matrix
                M_RGBtoDEP = self.video.aff_matrix_RGBtoDEP;
                v = np.array([[x],[y],[1]])
                t = self.rex.rexarm_IK([x,y, world_z, np.pi/2],1)
                t1 = self.rex.rexarm_IK([x,-y, world_z, np.pi/2],1)
                # if t[0] < 0:
                #     t[0] = np.pi + t[0]
                #     t[1] =  0 - t[1]
                #     t[2] = 0 - t[2]
                #     t[3] = 0 - t[3]
                # if t1[0] < 0:
                #     t1[0] = np.pi + t1[0]
                #     t1[1] = 0 - t1[1]
                #     t1[2] = 0 - t1[2]
                #     t1[3] = 0 - t1[3]    
                # self.rex.plan = [[t[0],-200,-200,-200,0,-30*D2R],[-200,-200,t[2],t[3],0,-200],[-200,t[1],-200,-200,0,-200],[-200,-200,-200,-200,0,10*D2R],[-200,0,-200,-200,-200,-200],[-200,0,0,0*D2R,0,-200]]
                self.rex.plan = [[t[0],-200,t[2],t[3],0,-30*D2R],
                [-200,t[1],-200,-200,0,-200],
                [-200,-200,-200,-200,0,8*D2R],
                [-200,t1[1] - 55 * D2R if t1[1] - 55 * D2R > 0 else 0,-200,-200,-200,-200]
                ]
                # t = self.rex.rexarm_IK([x,-y, world_z, np.pi/2],1)
                self.rex.plan.extend([[t1[0],-200,-200,-200,0,-200],[-200,-200,t1[2],t1[3],0,-200],[-200,t1[1],-200,-200,0,-200],[-200,-200,-200,-200,0,-15*D2R],[-200,t1[1] - 55 * D2R if t1[1] - 55 * D2R > 0 else 0,t1[2] + 5 * D2R ,-200,-200,-200]])
                self.vel = [[0.3,0.25,0.15,0.15,0.7,0.8]] * len(self.rex.plan)
                self.plan_point += 1
                self._timer_waypoint.start(self.check_time) 
                self.rex.plan_status = 1
            elif self.plan_point == len(self.video.det_pts_) and self.rex.plan_status == 0:
                print "event1 stops"
                self._timer_event1.stop()
                self.rex.plan = []
                self.vel = []
            else:
                pass

    def event2_timer(self):  
        if self.rex.moving_status == 1:
                self._timer_event2.stop()
                self.rex.plan = []
                self._timer_waypoint.stop()
        if len(self.video.det_pts_) == 0:
            print "no block detected"
            self.plan_point = 0
            self._timer_event2.stop()
        else:
            if self.rex.plan_status == 0 and self.plan_point < len(self.video.det_pts_):
                x = self.video.det_pts_[self.plan_point].rgb_real_pos[0]
                y = self.video.det_pts_[self.plan_point].rgb_real_pos[1]
                print "grapping %d %d %d" %(self.plan_point, x, y)
                world_z = self.video.det_pts_[self.plan_point].depth
                rw = np.array([[0],[0],[0]])
                M = self.video.aff_matrix
                M_RGBtoDEP = self.video.aff_matrix_RGBtoDEP;
                v = np.array([[x],[y],[1]])
                t = self.rex.rexarm_IK([x,y, world_z, np.pi/2],1)
                t1 = self.rex.rexarm_IK([EVEN2_X,EVEN2_Y, 25 + 38 * self.plan_point, np.pi/2],1)
                self.rex.plan = [[t[0],-200,t[2],t[3],0,-20*D2R],[-200,t[1],-200,-200,0,-200],[-200,-200,-200,-200,0,8*D2R],[-200,t1[1] - 55 * D2R if t1[1] - 55 * D2R > 0 else 0,t1[2],t1[3],-200,-200]]
                self.rex.plan.extend([[t1[0],-200,t1[2],t1[3],0,-200],[-200,t1[1],-200,-200,0,-200],[-200,-200,-200,-200,0,-20*D2R],[-200,t1[1] - 55 * D2R if t1[1] - 55 * D2R > 0 else 0, t1[2] + 5 * D2R,-200,-200,-200]])
                self.vel = [[0.3,0.25,0.15,0.15,0.7,0.8]] * len(self.rex.plan)
                self.plan_point += 1
                self._timer_waypoint.start(self.check_time) 
                self.rex.plan_status = 1
            elif self.plan_point == len(self.video.det_pts_) and self.rex.plan_status == 0:
                print "event2 stops"
                self._timer_event2.stop()
                self.rex.plan = []
                self.vel = []
            else:
                pass

    def event3_timer(self): 
        if self.rex.moving_status == 1:
                self._timer_event3.stop()
                self.rex.plan = []
                self._timer_waypoint.stop() 
        if len(self.video.det_pts_) == 0:
            print "begin to detect"
            self.video.blockDetector()
            points_temp =  []
            for points_i in self.video.det_pts_:
                if self.event3_color_check[points_i.color] == 0 and points_i.rgb_real_pos[0] < 0:
                    self.event3_color_check[points_i.color] = 1
                    points_temp.append(points_i)
            self.video.det_pts_ = points_temp
            self.plan_point = 0
            if len(self.video.det_pts_) == 0:
                if self.event3_finish == 0 and self.rex.plan_status == 0:
                    print "pushing blocks" 
                    self.rex.plan = [[-115 * D2R,68 * D2R,19 * D2R,76 * D2R,-115 * D2R,0],[-122 * D2R,57 * D2R,47 * D2R,76 * D2R,-122 * D2R,0]]
                    self.vel = [[0.2,0.1,0.05,0.05,0.3,0.3]] * len(self.rex.plan)
                    self._timer_waypoint.start(self.check_time)
                    self.rex.plan_status = 1
                    self.event3_finish = 1
                elif self.event3_finish == 1 and self.rex.plan_status == 0:
                    print "event finishes"
                    self._timer_event3.stop()
                    self.rex.plan = []
                    self.vel = []
                return
            # self._timer_event.stop()

        if self.rex.plan_status == 0 and self.plan_point < len(self.video.det_pts_):
            x = self.video.det_pts_[self.plan_point].rgb_real_pos[0]
            y = self.video.det_pts_[self.plan_point].rgb_real_pos[1]
            print "grapping %d %d %d" %(self.plan_point, x, y)
            world_z = self.video.det_pts_[self.plan_point].depth
            rw = np.array([[0],[0],[0]])
            M = self.video.aff_matrix
            M_RGBtoDEP = self.video.aff_matrix_RGBtoDEP
            v = np.array([[x],[y],[1]])
            if np.arctan2(y,x) < - np.pi/2 and np.arctan2(y,x) > - 3 * np.pi/4:
                t = self.rex.rexarm_IK([x,y, world_z, np.pi/2],1)
            else:
                t = self.rex.rexarm_IK([x,y, world_z - 5, np.pi/2],1)
            self.rex.plan = [[t[0] - 2 * D2R,-200,-200,-200,0,-30*D2R],[-200,-200,t[2],t[3],0,-200],[-200,t[1],-200,-200,0,-200],[-200,-200,-200,-200,0,8*D2R],[-200,0,-200,-200,-200,-200],[-200,0,0,0*D2R,0,-200]]
            self.vel = [[0.2,0.15,0.13,0.13,0.7,0.7]] * len(self.rex.plan)
            color_idx = self.video.det_pts_[self.plan_point].color
            t = self.rex.rexarm_IK([EVENT3_X,EVENT3_Y[color_idx], 24 + 38, np.pi/2],1)
            t1 = self.rex.rexarm_IK([EVENT3_X,EVENT3_Y[color_idx], 24, np.pi/2],1)
            # self.rex.plan.extend([[t[0],-200,-200,-200,t[0],-200],[-200,-200,t[2],t[3],-200,-200],[-200,t[1],-200,-200,-200,-200],[-200,-200,-200,-200,-200,-50*D2R],[-200,0,-200,-200,-200,-200],[-200,0,0,0*D2R,0,-200]])
            t4 = t[0] + np.pi/2 if t[0] < 0 else t[0] - 3 * np.pi/2
            self.rex.plan.extend([[t[0],-200,-200,-200,t4,-200],[-200,-200,t[2],t[3],-200,-200],[-200,t[1],-200,-200,-200,-200],[t1[0],t1[1],t1[2],t1[3],t4,-200],[-200,-200,-200,-200,-200,-50*D2R],[-200,0,-200,-200,-200,-200],[-200,0,0,0*D2R,0,-200]])
            self.vel.extend([[0.2,0.15,0.13,0.13,0.7,0.7],[0.2,0.15,0.13,0.13,0.7,0.7],[0.2,0.15,0.13,0.13,0.7,0.7],[0.07,0.05,0.05,0.05,0.7,0.5],[0.2,0.15,0.13,0.13,0.7,0.7],[0.2,0.15,0.13,0.13,0.7,0.7],[0.2,0.15,0.13,0.13,0.7,0.7]])
            self.event3_color_check[self.video.det_pts_[self.plan_point].color] = 1 
            self.plan_point += 1
            self._timer_waypoint.start(self.check_time)
            self.rex.plan_status = 1
        elif self.plan_point == len(self.video.det_pts_) and self.rex.plan_status == 0:
            print "event3 finish once"
            # self._timer_event.stop()
            self.rex.plan = []
            self.vel = []
            self.plan_point = 0
            self.video.det_pts_ = []
        else:
            pass

    def event4_timer(self): 
        color_status = 0
        if self.rex.moving_status == 1:
                self._timer_event4.stop()
                self.rex.plan = []
                self._timer_waypoint.stop() 
        if len(self.video.det_pts_) == 0:
            print "begin to detect"
            self.video.blockDetector()
            points_temp =  []
            if self.event4_color_idx == 8:
                print "event finishes"
                self._timer_event4.stop()
                self.rex.plan = []
                self.vel = []
                return
            for points_i in self.video.det_pts_:
                if points_i.color == RE_COLOR[self.event4_color_idx]:
                    points_temp.append(points_i)
                    self.event4_color_idx += 1
                    color_status = 1
                    break
            if color_status == 0:
                for points_i in self.video.det_pts_:
                    if points_i.depth > 50 and points_i.rgb_real_pos[0] < 0:
                        points_temp.append(points_i)
                        break
            self.video.det_pts_ = points_temp
            self.plan_point = 0
            # self._timer_event.stop()

        if self.rex.plan_status == 0 and self.plan_point < len(self.video.det_pts_):
            x = self.video.det_pts_[self.plan_point].rgb_real_pos[0]
            y = self.video.det_pts_[self.plan_point].rgb_real_pos[1]
            print "grapping %d %d %d" %(self.plan_point, x, y)
            world_z = self.video.det_pts_[self.plan_point].depth
            rw = np.array([[0],[0],[0]])
            M = self.video.aff_matrix
            M_RGBtoDEP = self.video.aff_matrix_RGBtoDEP
            v = np.array([[x],[y],[1]])
            t = self.rex.rexarm_IK([x,y, world_z, np.pi/2],1)
  
            self.rex.plan = [[t[0],-200,-200,-200,0,-30*D2R],[-200,-200,t[2],t[3],0,-200],[-200,t[1],-200,-200,0,-200],[-200,-200,-200,-200,0,8*D2R],[-200,0,-200,-200,-200,-200],[-200,0,0,0*D2R,0,-200]]
            self.vel = [[0.25,0.2,0.18,0.18,0.7,0.7]] * len(self.rex.plan)
            color_idx = self.video.det_pts_[self.plan_point].color
            if color_status == 1 :
                t = self.rex.rexarm_IK([EVENT4_X,EVENT4_Y,(self.event4_color_idx - 1) * 38 + 24, np.pi/2],1)
            else:
                t = self.rex.rexarm_IK([EVENT4_XF[self.event4_f_idx],EVENT4_YF[self.event4_f_idx],24, np.pi/2],1)
                self.event4_f_idx += 1
            self.rex.plan.extend([[t[0],-200,-200,-200,0,-200],[-200,-200,t[2],t[3],0,-200],[-200,t[1],-200,-200,0,-200],[-200,-200,-200,-200,0,-20*D2R],[-200,0,t[2],-200,-200,-200],[-200,0,0,0*D2R,0,-200]])
            self.vel.extend([[0.25,0.2,0.18,0.18,0.7,0.7]] * 6)
            self.plan_point += 1
            self._timer_waypoint.start(self.check_time)
            self.rex.plan_status = 1
        elif self.plan_point == len(self.video.det_pts_) and self.rex.plan_status == 0:
            print "event4 finish once"
            # self._timer_event.stop()
            self.rex.plan = []
            self.vel = []
            self.plan_point = 0
            self.video.det_pts_ = []
        else:
            pass

    def event5_timer(self):  
        if self.rex.moving_status == 1:
                self._timer_event5.stop()
                self.rex.plan = []
                self._timer_waypoint.stop()

        if len(self.video.det_pts_) == 0:
            if self.event5_block_num == 29:
                print "event finishes"
                self._timer_event5.stop()
                self.rex.plan = []
                self.vel = []
                return
            print "begin to detect"
            self.video.blockDetector()
            points_temp =  []
            points_max = 0
            for points_i in self.video.det_pts_:
                if points_i.rgb_real_pos[0] > 0 and abs(np.arctan2(points_i.rgb_real_pos[1],points_i.rgb_real_pos[0])) > points_max:
                    points_temp.append(points_i)
                    points_max = abs(np.arctan2(points_i.rgb_real_pos[1],points_i.rgb_real_pos[0]))
            # print points_i.depth
            self.video.det_pts_ = [points_temp[-1]]
            self.plan_point = 0

        if self.rex.plan_status == 0 and self.plan_point < len(self.video.det_pts_):
            x = self.video.det_pts_[self.plan_point].rgb_real_pos[0]
            y = self.video.det_pts_[self.plan_point].rgb_real_pos[1]
            print "grapping %d %d %d" %(self.plan_point, x, y)
            world_z = self.video.det_pts_[self.plan_point].depth
            rw = np.array([[0],[0],[0]])
            M = self.video.aff_matrix
            M_RGBtoDEP = self.video.aff_matrix_RGBtoDEP
            v = np.array([[x],[y],[1]])
            t = self.rex.rexarm_IK([x,y, world_z, np.pi/2],1)
            if x > 0 and y > 0:
                t[0] -= 2 * D2R 
            self.rex.plan = [[t[0],-200,-200,-200,0,-30*D2R],[-200,-200,t[2],t[3],0,-200],[-200,t[1],-200,-200,0,-200],[-200,-200,-200,-200,0,8*D2R],[-200,0,-200,-200,-200,-200],[-200,0,0,0*D2R,0,-200]]
            self.vel = [[0.3,0.2,0.18,0.18,0.7,0.7]] * len(self.rex.plan)
            color_idx = self.video.det_pts_[self.plan_point].color
            if self.event5_block_num < 28: 
                t = self.rex.rexarm_IK([EVENT5_X[self.event5_block_num],EVENT5_Y[self.event5_block_num],29 + EVENT5_Z[self.event5_block_num] * 38, np.pi/2],1)
            else:
                t = self.rex.rexarm_IK([EVENT5_X[27],EVENT5_Y[27],29 + EVENT5_Z[27] * 38 + 38, np.pi/2],1)
            self.rex.plan.extend([[t[0],-200,-200,-200,0,-200],[-200,-200,t[2],t[3],0,-200],[-200,t[1],-200,-200,0,-200],[-200,-200,-200,-200,0,-20*D2R],[-200,0,t[2] + 5 * D2R,-200,-200,-200],[-200,0,0,0*D2R,0,-200]])
            self.vel.extend([[0.285,0.2,0.18,0.18,0.7,0.7]] * 6)
            self.plan_point += 1
            self.event5_block_num += 1
            self._timer_waypoint.start(self.check_time)
            self.rex.plan_status = 1
        elif self.plan_point == len(self.video.det_pts_) and self.rex.plan_status == 0:
            print "event5 finish once"
            # self._timer_event.stop()
            self.rex.plan = []
            self.vel = []
            self.plan_point = 0
            self.video.det_pts_ = []
        else:
            pass
    def estop(self):
        self.statemachine.estop(self.rex)

    def update_gui(self):
        """ 
        update_gui function
        Continuously called by timer1 
        """

        """ Renders the video frames
            Displays a dummy image if no video available
            HINT: you can use the radio buttons to select different
            video sources like is done below
        """
        
        if(self.video.kinectConnected == 1):
            try:
                self.video.captureVideoFrame()
                self.video.captureDepthFrame()
            except TypeError:
                self.video.kinectConnected = 0
                self.video.loadVideoFrame()
                self.video.loadDepthFrame()

        if(self.ui.radioVideo.isChecked()):
            self.ui.videoFrame.setPixmap(self.video.convertFrame())
            # x = QtGui.QWidget.mapFromGlobal(self,QtGui.QCursor.pos()).x()
            # y = QtGui.QWidget.mapFromGlobal(self,QtGui.QCursor.pos()).y()
            # if ((x > MIN_X) and (x < MAX_X) and (y > MIN_Y) and (y < MAX_Y)):
            #     img = self.video.currentVideoFrame
            #     hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            #     y -= 30
            #     x -= 310
            #     h = hsv[y][x][0]
            #     s = hsv[y][x][1]
            #     v = hsv[y][x][2]
            #     global hp, xp, yp
            #     if ((xp - x)*(xp - x) > 4 and (yp - y)*(yp - y) > 4) or (hp - h)*(hp - h) >= 1:
            #         print "x: %d y: %d hsv : (%d, %d, %d)" % (x,y,h,s,v)
            #     xp = x
            #     yp = y
            #     hp = h
        if(self.ui.radioDepth.isChecked()):
            self.ui.videoFrame.setPixmap(self.video.convertDepthFrame())

        
        """ 
        Update GUI Joint Coordinates Labels
        """
        self.rex.rexarm_FK()
        self.ui.rdoutBaseJC.setText(str("%.2f" % (self.rex.joint_angles_fb[0]*R2D)))
        self.ui.rdoutShoulderJC.setText(str("%.2f" % (self.rex.joint_angles_fb[1]*R2D)))
        self.ui.rdoutElbowJC.setText(str("%.2f" % (self.rex.joint_angles_fb[2]*R2D)))
        self.ui.rdoutWristJC.setText(str("%.2f" % (self.rex.joint_angles_fb[3]*R2D)))
        self.ui.rdoutX.setText(str("%.2f" % (self.rex.endeffector_global[0])))
        self.ui.rdoutY.setText(str("%.2f" % (self.rex.endeffector_global[1])))
        self.ui.rdoutZ.setText(str("%.2f" % (self.rex.endeffector_global[2])))
        self.ui.rdoutT.setText(str("%.2f" % (self.rex.endeffector_global[3])))
        """ 
        Mouse position presentation in GUI
        TODO: after implementing workspace calibration 
        display the world coordinates the mouse points to 
        in the RGB video image.
        """    
        x = QtGui.QWidget.mapFromGlobal(self,QtGui.QCursor.pos()).x()
        y = QtGui.QWidget.mapFromGlobal(self,QtGui.QCursor.pos()).y()
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)):
            self.ui.rdoutMousePixels.setText("(-,-,-)")
            self.ui.rdoutMouseWorld.setText("(-,-,-)")
        else:
            x = x - MIN_X
            y = y - MIN_Y
            z = 0.
            world_z = 0.
            rw = np.array([[0],[0],[0]])
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%f)" % (x,y,z))
            if (self.video.cal_flag == 2):
                M = self.video.aff_matrix
                M_RGBtoDEP = self.video.aff_matrix_RGBtoDEP;
                v = np.array([[x],[y],[1]])
                DEP_xy = np.dot(M_RGBtoDEP,v)
                if ((DEP_xy[0] >= 0) and (DEP_xy[0] <= 640) and (DEP_xy[1] >= 0) and (DEP_xy[1] <= 480)):
                    world_z = self.video.depth - 123.6 * np.tan(self.video.currentDepthFrame[int(DEP_xy[1])][int(DEP_xy[0])]/2842.5 + 1.1863)
                    z = self.video.currentDepthFrame[int(DEP_xy[1])][int(DEP_xy[0])]
                    rw = np.dot(np.linalg.inv(self.video.WORLDtoRGB[:,[0,1,3]]),np.subtract(v,self.video.WORLDtoRGB[:,2:3] * (-world_z)))
                else:
                    z = 0.0
                if rw[2] != 0:
                    self.ui.rdoutMouseWorld.setText("(%.0f,%.0f, %.0f)" % (rw[0]/rw[2],rw[1]/rw[2],world_z))
                else:
                    self.ui.rdoutMouseWorld.setText("(-,-,-)")
            else:
                self.ui.rdoutMouseWorld.setText("(-,-,-)")

        """ 
        Updates status label when rexarm playback is been executed.
        This can be extended to include other appropriate messages
        """ 
        if(self.rex.plan_status == 1):
            self.ui.rdoutStatus.setText("Playing Back - Waypoint %d"
                                    %(self.rex.wpt_number + 1))

    def sliderChange(self):
        """ 
        Function to change the slider labels when sliders are moved
        and to command the arm to the given position
        """
        self.ui.rdoutBase.setText(str(self.ui.sldrBase.value()))
        self.ui.rdoutShoulder.setText(str(self.ui.sldrShoulder.value()))
        self.ui.rdoutElbow.setText(str(self.ui.sldrElbow.value()))
        self.ui.rdoutWrist.setText(str(self.ui.sldrWrist.value()))
        # self.ui.rdoutGrip1.setText(str(self.ui.sldrGrip1.value()))
        # self.ui.rdoutGrip2.setText(str(self.ui.sldrGrip2.value()))

        self.ui.rdoutTorq.setText(str(self.ui.sldrMaxTorque.value()) + "%")
        self.ui.rdoutSpeed.setText(str(self.ui.sldrSpeed.value()) + "%")
        self.rex.torque_multiplier = self.ui.sldrMaxTorque.value()/100.0
        self.rex.speed_multiplier = self.ui.sldrSpeed.value()/100.0
        self.rex.joint_angles[0] = self.ui.sldrBase.value()*D2R
        self.rex.joint_angles[1] = self.ui.sldrShoulder.value()*D2R
        self.rex.joint_angles[2] = self.ui.sldrElbow.value()*D2R
        self.rex.joint_angles[3] = self.ui.sldrWrist.value()*D2R
        self.rex.joint_angles[4] = self.ui.sldrGrip1.value()*D2R
        self.rex.joint_angles[5] = self.ui.sldrGrip2.value()*D2R
        # self.rex.joint_angles[6] = self.ui.sldrGrip2.value()*D2R
        self.rex.cmd_publish()

    def mousePressEvent(self, QMouseEvent):
        """ 
        Function used to record mouse click positions for calibration 
        """
 
        """ Get mouse posiiton """
        x = QMouseEvent.x()
        y = QMouseEvent.y()

        """ If mouse position is not over the camera image ignore """
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)): return

        """ Change coordinates to image axis """
        self.last_click[0] = x - MIN_X
        self.last_click[1] = y - MIN_Y
       
        """ If calibration has been performed """
        if (self.video.cal_flag == 1):
            
            """ Save last mouse coordinate """
            self.video.mouse_coord[self.video.mouse_click_id] = [(x-MIN_X),(y-MIN_Y)]

            """ Update the number of used poitns for calibration """
            self.video.mouse_click_id += 1

            """ Update status label text """
            self.ui.rdoutStatus.setText("Affine Calibration: Click Point %d" 
                                      %(self.video.mouse_click_id + 1))

            """ 
            If the number of click is equal to the expected number of points
            computes the affine calibration.
            
            LAB TASK: Change this code to use your workspace calibration routine
            and NOT the simple calibration function as is done now.
            """
            if(self.video.mouse_click_id == 2 * self.video.cal_points):
                """ 
                Update status of calibration flag and number of mouse
                clicks
                """
                self.video.cal_flag = 2
                self.video.mouse_click_id = 0
                
                """ Perform affine calibration with OpenCV """
                self.video.aff_matrix = cv2.getAffineTransform(
                                        self.video.mouse_coord[0:3],
                                        self.video.real_coord[0:3])
                self.video.aff_matrix_RGBtoDEP = cv2.getAffineTransform(
                                        self.video.mouse_coord[0:3],
                                        self.video.mouse_coord[5:8])
                self.video.aff_matrix_DEPtoRGB = cv2.getAffineTransform(
                                        self.video.mouse_coord[5:8],
                                        self.video.mouse_coord[0:3])
                object_points = np.insert(self.video.real_coord,2,[0,-38,-76,-76,-38],axis = 1)
                image_points = np.array(self.video.mouse_coord[0:self.video.cal_points])
                reval, rvec, tvec = cv2.solvePnP(object_points, image_points, self.video.intrinsic, self.video.distortion,cv2.SOLVEPNP_EPNP)
                self.video.WORLDtoRGB = np.dot(self.video.intrinsic,np.insert(cv2.Rodrigues(rvec)[0],[3],tvec,axis=1))
                t = np.dot(self.video.WORLDtoRGB,np.array([[-100.0],[100.0],[-114.],[1.]]))
                self.video.WORLDtoRGB /= t[2]
                # print t[2]
                z1_temp = np.tan(self.video.currentDepthFrame[int(self.video.mouse_coord[5][1])][int(self.video.mouse_coord[5][0])]/2842.5 + 1.1863)
                z2_temp = np.tan(self.video.currentDepthFrame[int(self.video.mouse_coord[6][1])][int(self.video.mouse_coord[6][0])]/2842.5 + 1.1863)
                z3_temp = np.tan(self.video.currentDepthFrame[int(self.video.mouse_coord[7][1])][int(self.video.mouse_coord[7][0])]/2842.5 + 1.1863)
                z4_temp = np.tan(self.video.currentDepthFrame[int(self.video.mouse_coord[8][1])][int(self.video.mouse_coord[8][0])]/2842.5 + 1.1863)
                z5_temp = np.tan(self.video.currentDepthFrame[int(self.video.mouse_coord[9][1])][int(self.video.mouse_coord[9][0])]/2842.5 + 1.1863)
                # print z1_temp
                # print z2_temp
                # print z3_temp
                # print z4_temp
                # self.video.depth = 123.6 * (z1_temp + z2_temp) / 2
                self.video.depth = 123.6 * (z1_temp + z2_temp + z3_temp + z4_temp + z5_temp)/5 + 45.5

                """ Updates Status Label to inform calibration is done """ 
                self.ui.rdoutStatus.setText("Waiting for input")

    def simple_cal(self):
        """ 
        Function called when affine calibration button is called.
        Note it only chnages the flag to record the next mouse clicks
        and updates the status text label 
        """
        self.video.cal_flag = 1 
        self.ui.rdoutStatus.setText("Affine Calibration: Click Point %d" 
                                    %(self.video.mouse_click_id + 1))
 

"""main function"""
def main():
    app = QtGui.QApplication(sys.argv)
    app_window = Gui()
    app_window.show()
    sys.exit(app.exec_())
 
if __name__ == '__main__':
    main()

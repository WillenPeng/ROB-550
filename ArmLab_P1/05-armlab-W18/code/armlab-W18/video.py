import cv2
import numpy as np
from PyQt4 import QtGui, QtCore, Qt
import freenect

class Video():
    def __init__(self):
        self.COLORS = [
            ([0, 20, 20], [180, 110, 80]),     #black
            ([169, 120, 120], [180, 255, 210]),      #red
            ([40, 5, 88], [80, 150, 200]),   #green
            ([110, 50, 120], [130, 195, 230]),   #blue
            ([10, 0, 210], [30, 255, 255]),     #yellow
            ([165, 90, 210], [173, 215, 255]),       #pink
            ([0, 135, 170], [10, 255, 255]),   #orange
            ([140, 70, 80], [160, 150, 210])    #purple
        ]
        self.BOUNDARIES = [
                    ([(683)], [(693)]),     #first
                    ([(665)], [(675)]),     #second
                    ([(643)], [(653)]),     #third
                    ([(622)], [(632)]),     #fourth
                    ([(595)], [(605)]),     #fifth
                    ([(566)], [(576)]),     #sixth
                    ([(536)], [(546)]),     #seventh
                    ([(503)], [(513)])      #eighth
                ]
        self.KERNELS = [
            ([4,4], [3,3], [1,1]),     #red
            ([4,4], [3,3], [2,2])      #black
        ]
        self.intrinsic = np.array([[521.5763487,0.,310.1226676],[0.,520.9781503,  272.9414709],[0., 0., 1.]])
        # self.distortion = np.array([0.26028723,-0.83752767,0.0000771,0.00114627,-0.0957])
        self.distortion = np.zeros(4)
        self.WORLDtoRGB = np.array([])
        self.AREA_UP = 900
        self.AREA_DN = 300
        self.currentVideoFrame=np.array([])
        self.currentDepthFrame=np.array([])
        self.kinectConnected = 1 #expect kinect to be available
        self.contourframe = np.array([])

        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthHSV_ = np.zeros((480,640,3)).astype(np.uint16)
        self.DepthCM=np.array([])

        
        """ 
        Calibration Variables 
        
        Currently this takes three points to perform a simple calibration.
        The simple calibration only worls for points on the board and uses
        an affine transform to perform the calibration.

        To use: 
            1. Click on simple calibration button
            2. Click the center of the arm
            3. Click the upper right corner of the board
            4. Click the lower right corner of the board

        Note that OpenCV requires float32 arrays

        TODO: Modify these to enable a robust 3D calibration

        """
        self.cal_points = 5 # number of points for the simple calibration
        self.real_coord = np.float32([[306.0, 306.0], [306.,-306.], [-306.,-306.],[-306.,306.],[306., 0.]])
        self.mouse_coord = np.float32([[0.0, 0.0],[0.0, 0.0],[0.0, 0.0],[0.0, 0.0],[0.0, 0.0],[0.0, 0.0],[0.0, 0.0],[0.0, 0.0],[0.0, 0.0],[0.0, 0.0]])      
        self.mouse_click_id = 0
        self.cal_flag = 0 # 0 - not calibrated, 1 - in progress, 2 - done
        self.aff_matrix = np.empty((2,3)) # affine matrix for simple calibration
        self.aff_matrix_DEPtoRGB = np.empty((2,3));
        self.aff_matrix_RGBtoDEP = np.empty((2,3));
        self.depth = 0.
        self.det_pts_ = []

    class det_pts():
        def __init__(self, depth_pos):
            self.depth_pos = depth_pos
            self.rgb_pos = [0,0] 
            self.depth = [0]
            self.color = [0]
            self.rgb_real_pos = np.array([])
    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        self.currentVideoFrame = freenect.sync_get_video()[0]

    def captureDepthFrame(self):
        """                      
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        self.currentDepthFrame = freenect.sync_get_depth()[0]

    
    def loadVideoFrame(self):
        self.currentVideoFrame = cv2.cvtColor(
            cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB
            )

    def loadDepthFrame(self):
        self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

    def convertFrame(self):
        """ Converts frame to format suitable for QtGui  """
        try:
            img=QtGui.QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QtGui.QImage.Format_RGB888
                             )
            return QtGui.QPixmap.fromImage(img)
        except:
            return None

    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for QtGui  
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """ 
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
            self.DepthHSV_[...,0] = self.currentDepthFrame
            self.DepthHSV_[...,1] = 0x9F
            self.DepthHSV_[...,2] = 0xFF
            img=QtGui.QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QtGui.QImage.Format_RGB888
                             )
            return QtGui.QPixmap.fromImage(img)
        except:
            return None


    def loadCalibration(self):
        """
        TODO (OPTIONAL):
        Load csmera distortion calibration from file and applies to the image:
        Look at camera_cal.py final lines to see how that is done
        This is optional, you do not need to implement distortion correction
        """
        pass

    def colorDetector(self, rgb_pos):
        i = 0
        for (lower, upper) in self.COLORS:
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            current_frame = np.array(cv2.cvtColor(self.currentVideoFrame,cv2.COLOR_RGB2HSV)[rgb_pos[1]][rgb_pos[0]])
            t = cv2.inRange(current_frame, lower, upper)
            if t[0] and t[1] and t[2] > 0:
                return i
            i += 1
        return 0

    def blockDetector(self):
        """
        TODO:
        Implement your block detector here.  
        You will need to locate
        blocks in 3D space
        """
        self.det_pts_ = []
        depth_frame = (freenect.sync_get_depth()[0] + freenect.sync_get_depth()[0] + freenect.sync_get_depth()[0])/3
        np.clip(depth_frame,0,2**10 - 1,depth_frame)
        hsv = depth_frame
        lower = np.array(self.BOUNDARIES[0][0], dtype = "uint16")
        upper = np.array(self.BOUNDARIES[0][1], dtype = "uint16")
        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(hsv, lower, upper)
        for (lower, upper) in self.BOUNDARIES[1:]:
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint16")
            upper = np.array(upper, dtype = "uint16")
         
            # find the colors within the specified boundaries and apply
            # the mask
            mask = cv2.bitwise_or(cv2.inRange(hsv, lower, upper), mask)

        output = cv2.bitwise_and(hsv, hsv, mask = mask)
        rgb_output = mask
        kernel_erose = np.ones(self.KERNELS[1][0],np.uint16)
        rgb_output = cv2.erode(rgb_output,kernel_erose,iterations = 1)
        kernel_open = np.ones(self.KERNELS[1][1],np.uint16)
        rgb_output = cv2.morphologyEx(rgb_output, cv2.MORPH_OPEN, kernel_open)
        kernel_close = np.ones(self.KERNELS[1][2],np.uint16)
        rgb_output = cv2.morphologyEx(rgb_output, cv2.MORPH_CLOSE, kernel_close)

        im2, contours, hierarchy = cv2.findContours(rgb_output,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        # rgb_output >>= 2
        for contour in contours:
            area = cv2.contourArea(contour)
            # print "area %f" %area
            if area > self.AREA_DN and area < self.AREA_UP:
                M = cv2.moments(contour)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                v_DEP = np.float32([cx,cy,1])
                RGB_xy = np.dot(self.aff_matrix_DEPtoRGB,v_DEP)
                if int(RGB_xy[0]) > 0 and int(RGB_xy[0]) < 640 and int(RGB_xy[1]) < 480 and int(RGB_xy[1]) > 0:
                # print RGB_xy
                    self.det_pts_.append(self.det_pts([cx, cy]))
                    self.det_pts_[-1].rgb_pos[0] = int(RGB_xy[0])
                    self.det_pts_[-1].rgb_pos[1] = int(RGB_xy[1])
                    self.det_pts_[-1].depth = self.depth - 123.6 * np.tan(self.currentDepthFrame[cy][cx]/2842.5 + 1.1863)
                    self.det_pts_[-1].color = self.colorDetector([int(RGB_xy[0]),int(RGB_xy[1])])
                    v = np.float32([[self.det_pts_[-1].rgb_pos[0]],[self.det_pts_[-1].rgb_pos[1]],[1]])
                    self.det_pts_[-1].rgb_real_pos = np.dot(np.linalg.inv(self.WORLDtoRGB[:,[0,1,3]]),np.subtract(v,self.WORLDtoRGB[:,2:3] * (-self.det_pts_[-1].depth)))
                    self.det_pts_[-1].rgb_real_pos /= self.det_pts_[-1].rgb_real_pos[2]
                    # print self.det_pts_[-1].rgb_real_pos
                    if self.det_pts_[-1].rgb_real_pos[0] > -306 and self.det_pts_[-1].rgb_real_pos[0] < 306 and self.det_pts_[-1].rgb_real_pos[1] < 306 and self.det_pts_[-1].rgb_real_pos[1] > -306:
                        rect = cv2.minAreaRect(contour)
                        print rect[2]
                        print "-------------------------------------------------"
                        print "rgb x y :%d %d depth : %d color : %d \n real x y z: %f %f %f" %(self.det_pts_[-1].rgb_pos[0], self.det_pts_[-1].rgb_pos[1], self.det_pts_[-1].depth, self.det_pts_[-1].color,
                            self.det_pts_[-1].rgb_real_pos[0], self.det_pts_[-1].rgb_real_pos[1], self.det_pts_[-1].rgb_real_pos[2])
                    else:
                        self.det_pts_.pop(-1)
        print len(self.det_pts_)
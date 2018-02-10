import sys
import cv2
import numpy as np
import freenect

font = cv2.FONT_HERSHEY_SIMPLEX


depth_frame = (freenect.sync_get_depth()[0] + freenect.sync_get_depth()[0] + freenect.sync_get_depth()[0])/3

np.clip(depth_frame,0,2**10 - 1,depth_frame)
# depth_frame >>= 2
# depth_frame = depth_frame.astype(np.uint8)

# cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
# cv2.imshow('window', depth_frame)
class det_pts():
    def __init__(self, depth_pos):
        self.depth_pos = depth_pos
        self.rgb_pos = [0,0] 
        self.depth = [0]
        self.color = [0]
det_pts_ = []
# define the list of boundaries and colors
colors = [
    ([90, 15, 39], [170, 110, 60]),     #black
    ([120, 160, 130], [130, 255, 150]),      #red
    ([52, 90, 110], [68, 130, 125]),   #green
    ([2, 110, 125], [8, 190, 160]),   #blue
    ([90, 220, 240], [95, 255, 255]),     #yellow
    ([125, 170, 209], [135, 210, 225]),       #pink
    ([110, 185, 200], [120, 250, 230]),   #orange
    ([145, 120, 105], [158, 140, 120])    #purple
]
boundaries = [
                    ([(683)], [(693)]),     #first
                    ([(665)], [(675)]),     #second
                    ([(643)], [(653)]),     #third
                    ([(622)], [(632)]),     #fourth
                    ([(595)], [(605)]),     #fifth
                    ([(566)], [(576)]),     #sixth
                    ([(536)], [(546)]),     #seventh
                    ([(503)], [(513)])      #eighth
                ]
kernels = [
    ([4,4], [3,3], [1,1]),     #red
    ([4,4], [3,3], [2,2])      #black
    # ([40, 5, 88], [80, 150, 190]),   #green
    # ([110, 50, 120], [130, 195, 230]),   #blue
    # ([20, 0, 210], [30, 255, 255]),     #yellow
    # ([165, 90, 190], [175, 215, 255]),       #pink
    # ([0, 160, 170], [10, 250, 255]),   #orange
    # ([140, 55, 80], [165, 150, 210])    #purple
]

hsv = depth_frame
# cv2.setMouseCallback("window",mouse_callback)
# cv2.imwrite('real_example.png',img)
# loop over the boundaries
    # create NumPy arrays from the boundaries
lower = np.array(boundaries[0][0], dtype = "uint16")
upper = np.array(boundaries[0][1], dtype = "uint16")
print lower 
print upper
# find the colors within the specified boundaries and apply
# the mask
mask = cv2.inRange(hsv, lower, upper)
print mask[106][173]
print hsv[106][173]
for (lower, upper) in boundaries[1:]:
    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype = "uint16")
    upper = np.array(upper, dtype = "uint16")
 
    # find the colors within the specified boundaries and apply
    # the mask
    mask = cv2.bitwise_or(cv2.inRange(hsv, lower, upper), mask)

output = cv2.bitwise_and(hsv, hsv, mask = mask)
# show the images
cv2.namedWindow("mask",1)
cv2.imshow("mask", mask)
# rgb_output = cv2.cvtColor(np.hstack([output]), cv2.COLOR_HSV2BGR)
rgb_output = mask
kernel0 = np.ones(kernels[1][0],np.uint16)
rgb_output = cv2.erode(rgb_output,kernel0,iterations = 1)
kernel1 = np.ones(kernels[1][1],np.uint16)
rgb_output = cv2.morphologyEx(rgb_output, cv2.MORPH_OPEN, kernel1)
kernel2 = np.ones(kernels[1][2],np.uint16)
rgb_output = cv2.morphologyEx(rgb_output, cv2.MORPH_CLOSE, kernel2)

im2, contours, hierarchy = cv2.findContours(rgb_output,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
# rgb_output >>= 2
output_contour = np.zeros((480,640,3)).astype(np.uint8)
output_contour[...,0] = hsv >> 2
output_contour[...,1] = 0x9F
output_contour[...,2] = 0xFF
imt = cv2.drawContours(output_contour, contours, -1, (0,255,0), 3)
for contour in contours:
    M = cv2.moments(contour)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    area = cv2.contourArea(contour)
    if area > 400 and area < 1200:
        det_pts_.append(det_pts([cx, cy]))
        det_pts_[-1].rgb_pos = [0,0]
        print "cx: %d cy:%d area: %d" %(cx, cy, area)
print len(det_pts_)
cv2.imshow("images_contours", imt)


cv2.imshow("images", rgb_output)
while True:
    ch = 0xFF & cv2.waitKey(10)
    if ch == 27:
        break
cv2.destroyAllWindows()
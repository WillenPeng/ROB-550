import sys
import cv2
import numpy as np
import freenect
font = cv2.FONT_HERSHEY_SIMPLEX
# define the list of boundaries
# boundaries = [
#     ([90, 15, 39], [170, 110, 60]),     #black
#     ([120, 160, 130], [130, 255, 150]),      #red
#     ([52, 90, 110], [68, 130, 125]),   #green
#     ([2, 110, 125], [8, 190, 160]),   #blue
#     ([90, 220, 240], [95, 255, 255]),     #yellow
#     ([125, 170, 209], [135, 210, 225]),       #pink
#     ([110, 185, 200], [120, 250, 230]),   #orange
#     ([145, 120, 105], [158, 140, 120])    #purple
# ]
boundaries = [
    ([0, 20, 20], [180, 110, 80]),     #black
    ([169, 80, 120], [180, 255, 170]),      #red
    ([40, 5, 88], [80, 150, 190]),   #green
    ([110, 50, 120], [130, 195, 230]),   #blue
    ([20, 0, 210], [30, 255, 255]),     #yellow
    ([165, 90, 190], [175, 215, 255]),       #pink
    ([0, 160, 170], [10, 255, 255]),   #orange
    ([140, 100, 80], [155, 150, 160])    #purple
]
kernels = [
    ([4,4], [3,3], [1,1]),     #red
    ([1,1], [3,3], [3,3])      #black
    # ([40, 5, 88], [80, 150, 190]),   #green
    # ([110, 50, 120], [130, 195, 230]),   #blue
    # ([20, 0, 210], [30, 255, 255]),     #yellow
    # ([165, 90, 190], [175, 215, 255]),       #pink
    # ([0, 160, 170], [10, 250, 255]),   #orange
    # ([140, 55, 80], [165, 150, 210])    #purple
]
def mouse_callback(event,x,y,flags,param):
    r = img[y][x][2]
    g = img[y][x][1]
    b = img[y][x][0]
    h = hsv[y][x][0]
    s = hsv[y][x][1]
    v = hsv[y][x][2]
    output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
    output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
    tmp = img.copy()
    cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (0,0,0))
    cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (0,0,0))
    cv2.imshow('window', tmp)
    if event == cv2.EVENT_LBUTTONDOWN:
        print "bgr: (%d, %d, %d) \nhsv: (%d, %d, %d)" % (b,g,r,h,s,v)
        print "x: %d y:%d" %(x, y)
if  1 + 1 == 2:
    # print "Opening " + str(sys.argv[1])

    img = cv2.cvtColor(freenect.sync_get_video()[0], cv2.COLOR_RGB2BGR)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.namedWindow("window",1)
    cv2.imshow('window', img)
    cv2.setMouseCallback("window",mouse_callback)
    # cv2.imwrite('real_example.png',img)
    # loop over the boundaries
        # create NumPy arrays from the boundaries
    lower = np.array(boundaries[0][0], dtype = "uint8")
    upper = np.array(boundaries[0][1], dtype = "uint8")
 
    # find the colors within the specified boundaries and apply
    # the mask
    mask = cv2.inRange(hsv, lower, upper)
    print mask.shape
    # for (lower, upper) in boundaries[1:]:
    #     # create NumPy arrays from the boundaries
    #     lower = np.array(lower, dtype = "uint8")
    #     upper = np.array(upper, dtype = "uint8")
     
    #     # find the colors within the specified boundaries and apply
    #     # the mask
    #     mask = cv2.bitwise_or(cv2.inRange(hsv, lower, upper), mask)
    
    output = cv2.bitwise_and(hsv, hsv, mask = mask)
    # show the images
    cv2.imshow("mask", mask)
    # rgb_output = cv2.cvtColor(np.hstack([output]), cv2.COLOR_HSV2BGR)
    rgb_output = mask
    kernel0 = np.ones(kernels[1][0],np.uint8)
    rgb_output = cv2.erode(rgb_output,kernel0,iterations = 1)
    kernel1 = np.ones(kernels[1][1],np.uint8)
    rgb_output = cv2.morphologyEx(rgb_output, cv2.MORPH_OPEN, kernel1)
    kernel2 = np.ones(kernels[1][2],np.uint8)
    rgb_output = cv2.morphologyEx(rgb_output, cv2.MORPH_CLOSE, kernel2)
    cv2.imshow("images", rgb_output)
    # im2, contours, hierarchy = cv2.findContours(rgb_output,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    # imt = cv2.drawContours(img, contours, -1, (0,1,0), 3)
    # for contour in contours:
    #     M = cv2.moments(contour)
    #     cx = int(M['m10']/M['m00'])
    #     cy = int(M['m01']/M['m00'])
    #     area = cv2.contourArea(contour)
    #     if area > 200 and area < 600:
    #         print "cx: %d cy:%d area: %d" %(cx, cy, area)
    #     # print "cx: %d cy:%d area: %d" %(cx, cy, area)
    # cv2.imshow("images_contours", imt)
    while True:
        ch = 0xFF & cv2.waitKey(10)
        if ch == 27:
            break
    cv2.destroyAllWindows()
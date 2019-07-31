import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge
import os, math, numpy as np
from collections import defaultdict

camera_matrix = np.array([
    [332.47884746146343, 0, 320],
    [0, 333.1761847948052, 240],
    [0, 0, 1]
])

dist_coeffs = np.array([
    2.15356885e-01,  -1.17472846e-01,  -3.06197672e-04,
   -1.09444025e-04,  -4.53657258e-03,   5.73090623e-01,
   -1.27574577e-01,  -2.86125589e-02,   0.00000000e+00,
    0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
    0.00000000e+00,   0.00000000e+00
])

rospy.init_node('ballooner')
bridge = CvBridge()

camera_info = None


def recognize(mat, red_hue_part, param1, param2, gauss_coeff, avg_thresh, center_thresh):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    saturation = hsv[:, :, 1]
    hue_inv = 180 - hue
    red = 2 * (cv2.max(hue, hue_inv) - 90)
    red = 240 * cv2.add(red, -180 + red_hue_part).astype(np.uint16) / red_hue_part
    fitness = red.astype(np.uint16) * saturation.astype(np.uint16)
    fitness = cv2.convertScaleAbs(fitness / 255)
    gauss_kernel = gauss_coeff
    fitness = cv2.GaussianBlur(fitness,(gauss_kernel, gauss_kernel), 0)
    #cv2.imwrite('out/photo3{:02d}_canny.png'.format(index), cv2.Canny(fitness, 20, 40))
    cimg = cv2.cvtColor(fitness,cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(fitness,cv2.HOUGH_GRADIENT,2,20,
                               param1=param1,param2=param2,minRadius=0,maxRadius=0)
    print(circles)
    if circles is None:
        return [], cimg
    circles = np.uint16(np.around(circles))
    red_circles = []
    for i in circles[0,:]:
        center_color=fitness[i[1]-1, i[0]-1]
        circle_img = np.zeros((fitness.shape[0],fitness.shape[1]), np.uint8)
        cv2.circle(circle_img,(i[0],i[1]),i[2],(255,255,255),-1)
        avg_color = cv2.mean(fitness, mask=circle_img)[0]
        #cv2.circle(cimg,(i[0],i[1]),i[2],(avg_color, avg_color, avg_color), 2)
        if avg_color > avg_thresh and center_color > center_thresh:
            overlapping = False
            for circle_index, existing_circle in enumerate(red_circles):
                dist = math.sqrt((existing_circle[0].item() - i[0].item()) ** 2 + (existing_circle[1].item() - i[1].item()) ** 2)
                radia_sum = existing_circle[2] + i[2]
                if dist < radia_sum * 1.1:
                    if existing_circle[2] > i[2]:
                        overlapping = True  # it's better to save avg_thresh and drop the worse circle
                        break
                    else:
                        del red_circles[circle_index]
            if not overlapping:
                red_circles.append(i)
    for i in red_circles:
        cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
        cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
    return red_circles, cimg


def camera_info_cb(msg):
    global camera_info
    camera_info = msg


def image_cb(data):
    if camera is None:
        return

    image = bridge.imgmsg_to_cv2(data, 'bgr8')
    
    circles, debug_img = recognize(image, 60, 100, 18, 11, 75, 75)

    debug_img_pub.publish(bridge.cv2_to_imgmsg(debug_img, 'bgr8'))

    if not circles:
        return

    undist_circles = [cv2.undistortPoints(np.array([[[c[0], c[1]]]], dtype=np.float64), camera_matrix, dist_coeffs)[0][0] * camera_matrix[0][0] for c in circles]
    vectors = [np.array([c[0], c[1], camera_matrix[0][0]]) for c in undist_circles]
    normalized_vectors = [v / np.linalg.norm(v) for v in vectors]
    
    largest_circle_index = -1
    largest_circle_radius = -1
    for i, c in enumerate(circles):
        if c[2] > largest_circle_radius:
            largest_circle_index = i
            largest_circle_radius = c[2]
    v = Vector3Stamped()
    v.vector.x = normalized_vectors[largest_circle_index][0]
    v.vector.y = normalized_vectors[largest_circle_index][1]
    v.vector.z = normalized_vectors[largest_circle_index][2]
    result_pub.publish(v)


image_sub = rospy.Subscriber('front_camera/image_raw', Image, image_cb)
#camera_info_sub = rospy.Subscriber('front_camera/camera_info', CameraInfo, camera_info_cb)
result_pub = rospy.Publisher('~direction', Vector3Stamped)
debug_img_pub = rospy.Publisher('~debug', Image)

rospy.spin()

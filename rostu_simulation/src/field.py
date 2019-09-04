#! /usr/bin/env python

import cv2
import numpy as np
import rospkg
import rospy
import time
import tf

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TransformStamped, Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

rospy.init_node('kinect_color')
rate = rospy.Rate(30)  # 30 hz refresh rate

rospack = rospkg.RosPack()

scanPub = rospy.Publisher('/scan', LaserScan, queue_size=1)
nav_odom = rospy.Publisher('/odom', Odometry, queue_size=1)

base_laser_link_broadcaster = tf.TransformBroadcaster()
base_link_broadcaster = tf.TransformBroadcaster()
base_footprint_broadcaster = tf.TransformBroadcaster()
odom_broadcaster = tf.TransformBroadcaster()
# br = tf.TransformBroadcaster()

def nothing(x):
    pass

field = cv2.imread(rospack.get_path('rostu_simulation') + '/model/field/field.png')
field = cv2.resize(field, (field.shape[0] * 2, field.shape[0] * 2))
print(field.shape[0], field.shape[1])
rect = np.array([[]])
spawn = False

posX = 140
posY = 560
degree = 90

def leftClick(event, x, y, flags, param):
    global spawn, posX, posY

    if event == cv2.EVENT_LBUTTONDOWN:
        spawn = True
    elif event == cv2.EVENT_LBUTTONUP:
        if spawn:
            posX = x
            posY = y

        spawn = False

cv2.namedWindow('field')
cv2.setMouseCallback('field', leftClick)


whiteLowerRange = np.array([150, 150, 150])
whiteUpperRange = np.array([255, 255, 255])

def findDistance(x1, y1, x2, y2, sinH, cosH):
    x2 = x2 + 15
    y2 = y2 - 5

    dis1 = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    dis2 = np.sqrt(((x1 + int(sinH * 20)) - x1)**2 + ((y1 + int(cosH * 20)) - y1)**2)
    dis3 = np.sqrt((x2 - (x1 + int(sinH * 20)))**2 + (y2 - (y1 + int(cosH * 20)))**2)

    angle = np.arctan2((y1 - (y1 + int(cosH * 20))), (x1 - (x1 + int(sinH * 20)))) - np.arctan2(y2-y1, x2-x1)
    angle = angle * 360 / (2 * np.pi)

    if angle < 0:
        angle = angle + 360

    return dis1, angle

odom = Odometry()
odom.header.frame_id = "odom"
# odom.child_frame_id = "base_link"
odom.pose.pose.position.x = 2 * 0.05 * 0.280898876#0.0264583333
odom.pose.pose.position.y = 2 * 0.05 * 0.280898876#0.0264583333
odom.pose.pose.position.z = 0
odom.pose.pose.orientation.x = 0
odom.pose.pose.orientation.y = 0
odom.pose.pose.orientation.z = 0
odom.pose.pose.orientation.w = 0

rtime = rospy.Time.now()

lineScan = LaserScan()
lineScan.header.frame_id = "base_laser_link"
lineScan.header.stamp = rospy.Time()
lineScan.angle_min = -3.14159
lineScan.angle_max = 3.14159
lineScan.angle_increment = 0.0174533
lineScan.range_min = 0.1
lineScan.range_max = 10
lineScan.scan_time = 0.0
lineScan.time_increment = 0.0
lineScan.ranges = np.zeros(360)
lineScan.intensities = np.zeros(360)
for i in range(360):
    lineScan.intensities[i] = 126.0

distance = np.array([], np.float16)
angles = np.array([], np.float16)

start_time = rospy.Time.now()
current_time = rospy.Time.now()
last_time = rospy.Time.now()
q = quaternion_from_euler(0, 0, 0)

x = posX * 0.05 * 0.280898876 - posX * 0.05 * 0.280898876
y = posY * 0.05 * 0.280898876 - posY * 0.05 * 0.280898876
th = 0


while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    heading = 180 - degree
    cosHeading = np.cos(np.radians(heading))
    sinHeading = np.sin(np.radians(heading))

    try:
        data = rospy.wait_for_message("/cmd_vel", Twist, 0.1)
        posX = posX + int(sinHeading * (data.linear.x / 0.5 / 0.280898876))
        posY = posY + int(cosHeading * (data.linear.x / 0.5 / 0.280898876))
        posX = posX + int(cosHeading * (data.linear.y / 0.5 / 0.280898876))
        posY = posY - int(sinHeading * (data.linear.y / 0.5 / 0.280898876))
        degree = degree - int(data.angular.z)

        dt = (current_time - last_time).to_sec()
        delta_x = (data.linear.x * np.cos(th) - data.linear.y * np.sin(th)) * dt
        delta_y = (data.linear.x * np.sin(th) + data.linear.y * np.cos(th)) * dt
        delta_th = data.angular.z * dt

        x += delta_x
        y += delta_y
        th += delta_th

        q = quaternion_from_euler(0, 0, th)

        odom.twist.twist = Twist(Vector3(data.linear.x, data.linear.y, 0), Vector3(0, 0, data.angular.z))

        if degree < 1:
            degree = 360
    except:
        pass

    base_laser_link_broadcaster.sendTransform(
        (0, 0, 0.2),
        quaternion_from_euler(0, 0, 0),
        current_time - start_time,
        "base_laser_link",
        "base_link"
    )

    base_link_broadcaster.sendTransform(
        (0, 0, 0),
        quaternion_from_euler(0, 0, 0),
        current_time - start_time,
        "base_link",
        "base_footprint"
    )

    base_footprint_broadcaster.sendTransform(
        (0 , 0, 0),
        quaternion_from_euler(0, 0, 0),
        current_time - start_time,
        "base_footprint",
        "odom"
    )

    odom_broadcaster.sendTransform(
        (x , y, 0),
        q,
        current_time - start_time,
        "odom",
        "map"
    )

    odom.header.stamp =  current_time - start_time
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*q))

    try:
        field = cv2.imread(rospack.get_path('rostu_simulation') + '/model/field/field.png')
        field = cv2.resize(field, (field.shape[0] * 2, field.shape[0] * 2))
        #Draw Player
        field = cv2.circle(field, (posX, posY), 20, (255, 0, 0), -1)
        field = cv2.line(field, (posX, posY), (posX + int(sinHeading * 20), posY + int(cosHeading * 20)), (0, 0, 255), 2)

        # angle =  np.arctan2(posY-posY, (posX+20)-posX) - np.arctan2((posY - (posY + int(cosHeading * 20))), (posX - (posX + int(sinHeading * 20))))
        # angle = angle * 360 / (2 * np.pi)

        #Draw Rectangle
        rect = np.array([[
            (posX + int(cosHeading * 200) - int(sinHeading * 200) , posY - int(sinHeading * 200) - int(cosHeading * 200)),
            (posX + int(cosHeading * 200) + int(sinHeading * 200), posY - int(sinHeading * 200) + int(cosHeading * 200)),
            (posX - int(cosHeading * 200) + int(sinHeading * 200), posY + int(sinHeading * 200) + int(cosHeading * 200)),
            (posX - int(cosHeading * 200) - int(sinHeading * 200) , posY + int(sinHeading * 200) - int(cosHeading * 200)),
            (posX + int(cosHeading * 200) - int(sinHeading * 200), posY - int(sinHeading * 200) - int(cosHeading * 200))
        ]])

        cv2.drawContours(field, rect, 0, (0, 0, 255), 2, 8)

        mask_for_crop = np.zeros(field.shape, dtype=np.uint8)
        channel_count = field.shape[2]
        ignore_mask_color = (255,) * channel_count
        cv2.fillPoly(mask_for_crop, rect, ignore_mask_color)
        cropped_field = cv2.bitwise_and(mask_for_crop, field)

        mask = cv2.inRange(cropped_field, whiteLowerRange, whiteUpperRange)
        # cv2.imshow('mask', mask)
        lines = cv2.HoughLinesP(mask, 100, np.pi/180, 1)

        distance = np.array([], np.float16)
        angles = np.array([], np.float16)
        for line in lines:
            x1, y1, x2, y2 = line[0]
            dis, ang = findDistance(posX, posY, x1, y1, sinHeading, cosHeading)
            distance = np.append(distance, dis)
            angles = np.append(angles, ang)
            # cv2.line(field, (posX, posY), (x1, y1), (255, 0, 255), 1)
            cv2.line(field, (x1, y1), (x2, y2), (0, 0, 255), 2)

        lineScan.ranges = np.zeros(360)

        for i in range(len(angles)):
            lineScan.ranges[int(angles[i])-1] = distance[i] * 0.05 * 0.280898876

    except:
        pass

    lineScan.header.stamp = rospy.Time.now() - rtime

    scanPub.publish(lineScan)
    nav_odom.publish(odom)

    cv2.imshow("field", field)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

    last_time = current_time
    rate.sleep()

cv2.destroyAllWindows()
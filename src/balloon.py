print 'initializing...'

import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
from clever import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
from aruco_pose.msg import MarkerArray
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Quaternion, Vector3, Point
import tf.transformations as t


rospy.init_node('flight')

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)


def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)


def get_distance_global(lat1, lon1, lat2, lon2):
    return math.hypot(lat1 - lat2, lon1 - lon2) * 1.113195e5


def navigate_wait(x, y, z, speed, frame_id, tolerance=0.2):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id)

    while True:
        telem = get_telemetry(frame_id=frame_id)
        if get_distance(x, y, z, telem.x, telem.y, telem.z) < tolerance:
            break
        rospy.sleep(0.2)


def navigate_global_wait(lat, lon, alt, yaw=float('nan'), speed=2, tolerance=2):
    navigate_global(lat=lat, lon=lon, z=start.z+alt, yaw=yaw, speed=speed)
    while not rospy.is_shutdown():
        telem = get_telemetry()
        if get_distance_global(lat, lon, telem.lat, telem.lon) < tolerance:
            return
        rospy.sleep(0.2)


def direction_cb(msg):
    speed = 1.5
    direction = msg

    direction = tf_buffer.transform(msg, 'map', rospy.Duration(0.1))
    yaw = math.atan2(direction.vector.y, direction.vector.x)
    direction.vector.x *= speed
    direction.vector.y *= speed
    direction.vector.z *= speed

    print 'set velocity'
    # set_velocity(vx=direction.vector.x, vy=direction.vector.y, vz=direction.vector.z, frame_id=direction.header.frame_id, yaw=float('nan'))
    set_velocity(vx=direction.vector.x, vy=direction.vector.y, vz=direction.vector.z, yaw=yaw)


def destroy_target(time=5):
    # TODO: make sure not to crash into ground
    dir_sub = rospy.Subscriber('red_dead_detection/direction', Vector3Stamped, direction_cb)
    rospy.sleep(time)
    print 'finish destroying target'
    dir_sub.unregister()

destroy_start_lat = 56.3346667
destroy_start_lon = 43.6405074
destroy_start_yaw = -0.0942823588848

print 'press enter to start'
raw_input()

print 'take off'
start = get_telemetry()
print navigate(z=3, frame_id='body', auto_arm=True, speed=1)
rospy.sleep(6)

print 'go to destroy start'
navigate_global_wait(destroy_start_lat, destroy_start_lon, alt=3, yaw=destroy_start_yaw, speed=1)

rospy.sleep(6)

print 'destroy target'
destroy_target()

print 'fly home'
navigate_wait(x=start.x, y=start.y, z=start.z+3, frame_id='map', speed=1, tolerance=3)

rospy.sleep(5)

print 'land'
land()

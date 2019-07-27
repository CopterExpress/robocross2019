print 'initializing...'

import rospy
import math
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3Stamped, TwistStamped
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

direction_pub = rospy.Publisher('~direction', TwistStamped, queue_size=1)
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
    if not destroying_target:
        return

    speed = 1.4

    direction = tf_buffer.transform(msg, 'map', rospy.Duration(0.2))
    yaw = math.atan2(direction.vector.y, direction.vector.x)

    if abs(direction.vector.z) > 0.4:
        # approach vertically at first
        direction.vector.x = 0
        direction.vector.y = 0
    else:
        direction.vector.x *= speed
        direction.vector.y *= speed

    direction.vector.z *= speed

    # direction.vector.x *= speed
    # direction.vector.y *= speed
    # direction.vector.z *= speed

    print 'set velocity'
    # set_velocity(vx=direction.vector.x, vy=direction.vector.y, vz=direction.vector.z, frame_id=direction.header.frame_id, yaw=float('nan'))
    set_velocity(vx=direction.vector.x, vy=direction.vector.y, vz=direction.vector.z, yaw=yaw)

    # direction_body = tf_buffer.transform(msg, 'body', rospy.Duration(0.1))
    twist_msg = TwistStamped()
    twist_msg.header.frame_id = 'map'
    twist_msg.header.stamp = msg.header.stamp
    twist_msg.twist.linear = direction.vector
    direction_pub.publish(twist_msg)


blob_size = 99999999


def blob_size_cb(msg):
    global blob_size
    blob_size = msg.data


destroying_target = False


def destroy_target(timeout=rospy.Duration(20)):
    global destroying_target
    destroying_target = True

    # TODO: make sure not to crash into ground
    dir_sub = rospy.Subscriber('red_dead_detection/direction', Vector3Stamped, direction_cb, queue_size=1)
    size_sub = rospy.Subscriber('red_dead_detection/blob_size', Float32, blob_size_cb, queue_size=1)
    not_seeing_count = 0
    seen_target = False

    start_stamp = rospy.get_rostime()
    while not rospy.is_shutdown():
        if rospy.get_rostime() - start_stamp > timeout and blob_size < 10:
            print 'timeout destroying target'
            # break
        if blob_size <= 2:
            not_seeing_count += 1
            if not_seeing_count > 400:
                print 'cant find target, terminate'
                # break
            if not_seeing_count > 100:
                if seen_target:
                    print 'blob size small, terminate'
                    break
                else:
                    print 'search target'
                    # set_velocity(vx=-1, frame_id='body')
        else:
            not_seeing_count = 0
            if blob_size > 4:
                seen_target = True

    print 'finish destroying target'
    destroying_target = False
    dir_sub.unregister()
    size_sub.unregister()


def land_on_marker():
    markers_sub = rospy.Subscriber('aruco_detect/markers', MarkerArray, lambda msg: None)
    state = ''

    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        marker = get_telemetry(frame_id='aruco_225')
        if not marker.armed:
            print 'disarmed, break',
            break

        dist = math.hypot(marker.x, marker.y)
        if not math.isnan(marker.x):
            if marker.z < 0.8:
                print 'final approach'
                telem = get_telemetry()
                set_velocity(vx=telem.vx, vy=telem.vy, vz=-0.8, yaw=float('nan'))
                rospy.sleep(0.5)
                print 'Disarm!'
                # arming(False)
                break
            elif dist > 1 or True:
                state = 'horizontal_approach',
                print 'Horizontal approach'
                set_position(y=dist+0.5, z=dist, frame_id='aruco_225', yaw=float('nan'))
            elif marker.z < 1:
                print 'final approach'
                telem = get_telemetry()
                set_velocity(vx=telem.vx, vy=telem.vy, vz=-0.9, yaw=float('nan'))
                rospy.sleep(0.5)
                print 'Disarm!'
                # arming(False)
                break
            elif marker.z < 2:
                if state != 'landing':
                    print 'Landing without forwarding'
                    navigate(frame_id='aruco_225', speed=1.5, yaw=float('nan'))
                    state = 'landing'
            else:
                if state != 'forwarding':
                    print 'Landing with forwarding'
                    navigate(y=1.5, frame_id='aruco_225', speed=1.5, yaw=float('nan'))
                    state = 'forwarding'
        else:
            # lost marker
            print 'lost marker, land'
            telem = get_telemetry()
            set_velocity(vx=telem.vx, vy=telem.vy, vz=-1, yaw=float('nan'))

        r.sleep()
    markers_sub.unregister()


def do_mission():
    print 'take off'
    print navigate(z=3, frame_id='body', auto_arm=True, speed=1)
    rospy.sleep(4)

    # print 'go to destroy start'
    # navigate_global_wait(destroy_start_lat, destroy_start_lon, alt=2.2, yaw=destroy_start_yaw, speed=2.5, tolerance=2)

    # rospy.sleep(2)

    print 'destroy target'
    destroy_target()

    print 'fly up'
    navigate(z=2, frame_id='body', speed=1)
    rospy.sleep(2)

    print 'fly home'
    navigate_wait(x=start.x, y=start.y, z=start.z+4, frame_id='map', speed=2.5, tolerance=3)

    rospy.sleep(1)

    print 'land'
    land()
    # land_on_marker()


destroy_start_lat = 56.3346667
destroy_start_lon = 43.6405074
destroy_start_yaw = -0.0942823588848

print 'press enter to start'
raw_input()

rospy.sleep(4)

start = get_telemetry()

# destroy_target()

do_mission()

# print 'take off'
# print navigate(z=4, frame_id='body', auto_arm=True, speed=1.5)
# rospy.sleep(7)

# print 'land'
# land_on_marker()

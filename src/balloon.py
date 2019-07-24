import rospy
import math
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


def direction_cb(msg):
    speed = 3
    direction = msg

    direction = tf_buffer.transform(msg, 'map', rospy.Duration(0.1))
    yaw = math.atan2(direction.vector.y, direction.vector.x)
    direction.vector.x *= speed
    direction.vector.y *= speed
    direction.vector.z *= speed

    print 'set velocity'
    # set_velocity(vx=direction.vector.x, vy=direction.vector.y, vz=direction.vector.z, frame_id=direction.header.frame_id, yaw=float('nan'))
    set_velocity(vx=direction.vector.x, vy=direction.vector.y, vz=direction.vector.z, yaw=yaw)


def destroy_target():
    rospy.Subscriber('red_dead_detection/direction', Vector3Stamped, direction_cb)
    rospy.spin()


print 'take off'
navigate(z=3, frame_id='body', auto_arm=True, speed=1)
rospy.sleep(6)

print 'destroy target'
destroy_target()

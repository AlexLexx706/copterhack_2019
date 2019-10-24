import rospy
from std_msgs.msg import String
from clever import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandLong
from pymavlink.dialects.v20 import common as mavlink

rospy.init_node('my_ros_node')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
send_command_long = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

HZ =  5

r = rospy.Rate(HZ)
send_command_long(False, mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
rospy.sleep(25)

# show aruco telemetry
while not rospy.is_shutdown():
	telemetry = get_telemetry(frame_id='aruco_map')
	print('a pos:%03.3f :%03.3f :%03.3f' % (telemetry.x, telemetry.y, telemetry.z))
	print('aruco telemetry: %s\n' % (telemetry, ))

	#telemetry = get_telemetry(frame_id='body')
	#print('body telemetry:%s\n\n' % (telemetry,))

	telemetry = get_telemetry(frame_id='map')
	print('m pos:%03.3f :%03.3f :%03.3f' % (telemetry.x, telemetry.y, telemetry.z))

        telemetry = get_telemetry(frame_id='body')
        print('b pos:%03.3f :%03.3f :%03.3f\n' % (telemetry.x, telemetry.y, telemetry.z))

	#print('map pos:%s\n\n' % ((telemetry.x, telemetry.y, telemetry.z),))

	#print('body telemetry:%s\n\n' % (telemetry,))
	r.sleep()


telemetry = get_telemetry()
print("telemetry:%s" % (telemetry, ))

def foo_callback(msg):
    print msg.data


rospy.Subscriber('/foo', String, foo_callback)

rospy.spin()

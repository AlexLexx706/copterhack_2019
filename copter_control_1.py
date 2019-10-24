import rospy
from std_msgs.msg import String
from clever import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandLong
from pymavlink.dialects.v20 import common as mavlink
import math
import logging

logging.basicConfig(level=logging.INFO)
LOG = logging.getLogger(__name__)

rospy.init_node('my_ros_node', log_level=rospy.INFO)

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
send_command_long = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

HZ = 10
r = rospy.Rate(HZ)


def check_yaw(angle, limit=math.radians(3), frame_id='body', wait=True):
	"""check yaw value"""
	if not wait:
		telemetry = get_telemetry(frame_id)
		rospy.logdebug('check_yaw telemetry:%s' % (telemetry, ))
		angle_diff = math.fabs(telemetry.yaw - angle)
		rospy.loginfo('angle_diff:%s' % (angle_diff, ))
		return angle_diff < limit
	else:
		while not rospy.is_shutdown():
			telemetry = get_telemetry(frame_id)
			rospy.logdebug('check_yaw telemetry:%s' % (telemetry, ))
			angle_diff = math.fabs(telemetry.yaw - angle)
			rospy.loginfo('angle_diff:%s' % (angle_diff, ))

			if angle_diff < limit:
				return True
			r.sleep()


def check_position(pos, limit=0.25, frame_id='body', wait=True):
	'''check copter position'''
	if not wait:
		telemetry = get_telemetry(frame_id)
		rospy.logdebug('check_position telemetry:%s' % (telemetry, ))
		d_x = telemetry.x - pos[0]
		d_y = telemetry.y - pos[1]
		d_z = telemetry.z - pos[2]
		cur_pos_diff = math.sqrt(d_x ** 2 + d_y ** 2 + d_z ** 2)
		rospy.loginfo("cur_pos_diff:%s" % (cur_pos_diff, ))
		return cur_pos_diff < limit
	else:
		while not rospy.is_shutdown():
			telemetry = get_telemetry(frame_id)
			rospy.logdebug('check_position telemetry:%s' % (telemetry, ))
			d_x = telemetry.x - pos[0]
			d_y = telemetry.y - pos[1]
			d_z = telemetry.z - pos[2]
			cur_pos_diff = math.sqrt(d_x ** 2 + d_y ** 2 + d_z ** 2)
			rospy.loginfo("cur_pos_diff:%s" % (cur_pos_diff, ))
			if cur_pos_diff < limit:
				 return
			r.sleep()

def takeoff(z=1.6, speed=0.2):
	telemetry = get_telemetry('map')
	navigate(x=telemetry.x, y=telemetry.y, z=z, speed=speed, yaw=float('nan'), frame_id='map', auto_arm=True)
	return check_position(pos, limit=pos_limit, frame_id='map', wait=True)

def move_to(x, y, z, yaw=float('nan'), speed=0.3, yaw_rate=0):
	'''move to point in aruco map'''
	navigate(x=x, y=y, z=z, speed=speed, yaw=yaw, yaw_rate=yaw_rate, frame_id='aruco_map')
	return check_position(pos, limit=pos_limit, frame_id='aruco_map', wait=True)

def landing():
	return land()

def rotate(yaw, yaw_rate):
	navigate(yaw=yaw, frame_id='body')
	check_yaw(yaw, yaw_rate=yaw_rate, limit=math.radians(3), frame_id='body', wait=True)

# rospy.logwarn('Reset mavlink, wait 20 sec ...')
# send_command_long(False, mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
# rospy.sleep(25)
rospy.loginfo('Mavlink should reay to fly')

# step 1. takeoff
takeoff()
rospy.loginfo('step 1. takeoff, done ')
rospy.sleep(3.0)

# step 2. hold aruco
telemetry = get_telemetry('aruco_map')
move_to(telemetry.x, telemetry.y, telemetry.z)
rospy.loginfo('step 2. hold aruco, done')
rospy.sleep(3.0)

# step 3, move on 5 meters
# telemetry = get_telemetry('aruco_map')
# move_to(telemetry.x + 2, telemetry.y, telemetry.z)
# rospy.loginfo('step 3. move to 2 metres, done')
# rospy.sleep(3.0)

# step 3, rotate 90
rotate(math.radians(90), math.radians(30))
rospy.loginfo('step 3. rotate, done')
rospy.sleep(3.0)

# state 4. lending
landing()
rospy.loginfo('step 4. landing, done')

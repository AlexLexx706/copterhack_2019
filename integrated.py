import rospy
from std_msgs.msg import String
from clever import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandLong
from pymavlink.dialects.v20 import common as mavlink
import math
import logging
import time

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
	navigate(x=telemetry.x, y=telemetry.y, z=z, speed=speed,
			 yaw=float('nan'), frame_id='map', auto_arm=True)
	return check_position([telemetry.x, telemetry.y, z], frame_id='map')


def move_to(x, y, z, yaw=float('nan'), speed=0.3, yaw_rate=0):
	'''move to point in aruco map'''
	navigate(x=x, y=y, z=z, speed=speed, yaw=yaw,
			 yaw_rate=yaw_rate, frame_id='aruco_map')
	return check_position([x, y, z], frame_id='aruco_map')


def landing():
	return land()


rospy.logwarn('Reset mavlink, wait 20 sec ...')
send_command_long(
	False, mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
rospy.sleep(25)
rospy.loginfo('Mavlink should reay to fly')



#

# 64x64 ~4m
# 230x230 ~1m

# camera resolution
# 640x480

camera = [
	640,
	480,
]


was = ""

def box_center(coords):
	c = [
		coords[0] + coords[2] / 2,
		coords[1] + coords[3] / 2,
	]

	return c


def to_camera(point):
	camera_center = box_center([
		0, 0,
		camera[0],
		camera[1],
	])

	diff = [
		(point[0] - camera_center[0]) / camera_center[0],
		(point[1] - camera_center[1]) / camera_center[1],
	]

	return diff


def size_to_z(size):
	max = 220
	min = 64

	cam = 480

	inv = size / min
	z = 4 / inv

	#print(size)
	#print(inv)
	#print(z)
	
	return z / 2


# step 1. takeoff
takeoff()
rospy.loginfo('step 1. takeoff, done ')
rospy.sleep(3.0)

while 1:
	f = open('../drift.txt', 'r')
	line = f.readline()
	f.close()

	rospy.sleep(0.5) 

	if was == line:
		continue

	was = line

	coords = line.split('_')

	for i in range(0, 4):
		coords[i] = int(coords[i])
	
	rel = to_camera(coords)
	z = size_to_z(coords[2])

	rospy.loginfo("depth = {}".format(z))

	target = [
		rel[0] / 10,
		z - 1,
		-rel[1] / 10
	]

	# print(coords)
	rospy.loginfo('%s' % (target, ))

	curpos = get_telemetry('aruco_map')
	point = [
		curpos.x + target[0],
		curpos.y + target[1],
		curpos.z + target[2]
	]

	move_to(point[0], point[1], point[2])
	rospy.sleep(4.0)

	if z < 2:
		landing()
		break


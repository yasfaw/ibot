import rospy
from math import sqrt
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
	def __init__(self, *args, **kwargs):

		# define a sample rate:
		self.sampling_rate = 20.

		# Get System Parameters: (Just a basic implementation of PID is done, so many are not used)
		self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
		self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
		self.brake_deadband = rospy.get_param('~brake_deadband', .1)
		self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
		self.decel_limit = rospy.get_param('~decel_limit', -5)
		self.accel_limit = rospy.get_param('~accel_limit', 1.)
		self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
		self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
		self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
		self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
		self.max_acceleration = rospy.get_param('~max_acceleration', 1.5)

		# Define PID object:
		self.yaw_controller = YawController(wheel_base=self.wheel_base,
											steer_ratio=self.steer_ratio,
											min_speed=1.0, # set to this value
											max_lat_accel=self.max_lat_accel,
											max_steer_angle=self.max_steer_angle)

		self.throttle_pid = PID(kp=1., ki=0., kd=0., mn=self.decel_limit, mx=self.accel_limit)

	def control(self, target_speed, target_angular_speed, current_velocity, dbw_enabled):

		# delta time:
		deltat = 1. / self.sampling_rate

		# calculate speed difference:
		current_speed_mod = sqrt(current_velocity[0] ** 2 + current_velocity[1] ** 2)
		target_speed_mod = sqrt(target_speed[0] ** 2 + target_speed[1] ** 2)
		speed_diff = target_speed_mod - current_speed_mod

		# PID values:
		throttle = self.throttle_pid.step(speed_diff, deltat)
		steer = self.yaw_controller.get_steering(target_speed[0],
												 target_angular_speed,
												 current_velocity[0])

		# check if manual mode is ON
		if dbw_enabled:
			self.throttle_pid.reset()
			# self.steer_pid.reset()
		else:
			rospy.logwarn("Safe drive mode OFF!")

		# Since break and throttle are different commands, get their values:
		brake = 0.
		if throttle < 0.:
			brake = 0. - throttle
			throttle = 0.

		return throttle, brake, steer

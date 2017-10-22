import rospy
from math import sqrt
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_SPEED = 40.0
BRAKE_MAX = 0.6
MAX_BRAKE_TORQUE= 20000
MAX_ACCEL_TORQUE= 100.0
MAX_ACCELTORQUE_PERCENTAGE = 1.0


class Controller(object):
	def __init__(self, *args, **kwargs):

		# Get System Parameters: (Just a basic implementation of PID is done, so many are not used)
		self.vehicle_mass = rospy.get_param('~vehicle_mass')
		self.fuel_capacity = rospy.get_param('~fuel_capacity')

		self.total_vehicle_mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY

		self.brake_deadband = rospy.get_param('~brake_deadband')
		self.wheel_radius = rospy.get_param('~wheel_radius')
		self.decel_limit = rospy.get_param('~decel_limit')
		self.accel_limit = rospy.get_param('~accel_limit')
		self.wheel_base = rospy.get_param('~wheel_base')
		self.steer_ratio = rospy.get_param('~steer_ratio')
		self.max_lat_accel = rospy.get_param('~max_lat_accel')
		self.max_steer_angle = rospy.get_param('~max_steer_angle')

		self.sampling_rate = 2.

		self.max_brake_torque = BRAKE_MAX * self.total_vehicle_mass * abs(self.decel_limit) * self.wheel_radius

		self.prev_time = rospy.get_time()

		# Define PID object:
		self.yaw_controller = YawController(wheel_base=self.wheel_base,
											steer_ratio=self.steer_ratio,
											min_speed=1.0*0.447, # set to this value
											max_lat_accel=self.max_lat_accel,
											max_steer_angle=self.max_steer_angle)

		self.throttle_pid = PID(kp=1.0, ki=0.1, kd=0.0, mn=self.decel_limit, mx=self.accel_limit)

	def control(self, target_speed, target_angular_speed, current_velocity, dbw_enabled):

		throttle = 0.0
		brake = 0.0
		steering = 0.0

		if not all((target_speed, current_velocity)):
			return throttle, brake, steering

		if dbw_enabled:
			# calculate speed difference:
			speed_diff = target_speed - current_velocity
			# throttle = self.throttle_pid.step(speed_diff, dt)

			current_time = rospy.get_time()
			dt = current_time - self.prev_time
			self.prev_time = current_time

			acceleration = speed_diff*0.3
			# print(dt, target_speed, current_velocity, acceleration, self.accel_limit, self.decel_limit)
			if acceleration >= 0:
				throttle = min(self.accel_limit, acceleration)
				brake = 0.
			else:
				brake = max(self.decel_limit, acceleration)
				throttle = 0.

			# torque = self.total_vehicle_mass * acceleration * self.wheel_radius
			# if torque > 0:
			# 	throttle, brake = min(MAX_ACCELTORQUE_PERCENTAGE, torque / MAX_ACCEL_TORQUE), 0.0
			# else:
			# 	throttle, brake = 0.0, min(abs(torque), MAX_BRAKE_TORQUE)

			# PID values:
			steer = self.yaw_controller.get_steering(target_speed, target_angular_speed, current_velocity)

			# All parameters should be in the range [0-1]
			# rospy.loginfo("Values: {0}, {1}, {2}".format(target_speed, target_angular_speed, current_velocity))
			# rospy.loginfo("Parameters: {0}, {1}, {2}".format(throttle, brake, steer))
			return throttle, brake, steer
		else:
			self.throttle_pid.reset()
			rospy.loginfo("This mode is up")
			return throttle, brake, steering

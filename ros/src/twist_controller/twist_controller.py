import rospy

from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED = 0.1


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement

        # First store all local variables
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.brake_deadband = kwargs['brake_deadband']
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']

        self.brake_tourque_const = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius

        self.tau = 0.5
        self.ts = 0.02

        # Some simple controllers to well... control...
        self.yaw_controller = YawController(wheel_base=self.wheel_base,
                                            steer_ratio=self.steer_ratio,
                                            min_speed=MIN_SPEED,
                                            max_lat_accel=self.max_lat_accel,
                                            max_steer_angle=self.max_steer_angle)
        self.throttle_controller = PID(kp=0.3, ki=0.1, kd=0.0, mn=self.decel_limit, mx=self.accel_limit)
        self.steering_controller = PID(kp=0.5, ki=0.05, kd=0.1, mn=-self.max_steer_angle, mx=self.max_steer_angle)

        self.vel_lpf = LowPassFilter(tau=self.tau, ts=self.ts)
        self.steer_lpf = LowPassFilter(tau=2, ts=1)

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # Return throttle, brake, steer

        if not dbw_enabled:
            rospy.logwarn("DBW NOT ENABLED")
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        steering = self.steer_lpf.filt(steering)

        vel_error = linear_vel - current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(error=vel_error, sample_time=sample_time)
        brake = 0

        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0
            brake = 770  # N*m
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            # brake = abs(decel) * self.vehicle_mass * self.wheel_radius
            brake = abs(decel) * self.brake_tourque_const if abs(decel) > self.brake_deadband else 0.
            # brake = abs(decel) * self.brake_tourque_const

        rospy.logwarn("Throttle = {}; Brake = {}; Steering = {}".format(throttle, brake, steering))
        return throttle, brake, steering

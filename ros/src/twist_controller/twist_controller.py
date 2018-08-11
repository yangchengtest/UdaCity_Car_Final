import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, cp):
        self.cp = cp
        self.yaw_controller = YawController(
            wheel_base=cp.wheel_base,
            steer_ratio=cp.steer_ratio,
            min_speed=cp.min_speed,
            max_lat_accel=cp.max_lat_accel,
            max_steer_angle=cp.max_steer_angle)
        self.cp = cp
        self.pid = PID(kp=5, ki=0.5, kd=0.5, mn=cp.decel_limit, mx=cp.accel_limit)
        #self.vel_lpf = LowPassFilter(tau=0.5, ts=0.02)
        self.s_lpf = LowPassFilter(tau=3, ts=1)
        self.t_lpf = LowPassFilter(tau=3, ts=1)

        self.last_time = 0

    def reset(self):
        self.pid.reset()

    def control(self,linear_vel, angular_vel, current_vel,dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.reset()
            return 0, 0, 0
        else:
            #current_vel = self.vel_lpf.filt(current_vel)
            vel_err = linear_vel - current_vel

            steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
            steering = self.s_lpf.filt(steering)
            current_time = rospy.get_time()
            sample_time = current_time - self.last_time

            acceleration = self.pid.step(vel_err, sample_time)
            acceleration = self.t_lpf.filt(acceleration)

            if acceleration > 0.0:
                throttle = acceleration
                brake = 0.0
            else:
                throttle = 0.0
                deceleration = -acceleration

                if deceleration < self.cp.brake_deadband:
                    deceleration = 0.0

                deceleration = min(deceleration, -self.cp.decel_limit)
                brake = deceleration * (
                            self.cp.vehicle_mass + self.cp.fuel_capacity * GAS_DENSITY) * self.cp.wheel_radius

            # Return throttle, brake, steer
            return throttle, brake, steering



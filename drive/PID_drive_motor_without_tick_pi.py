#!/usr/bin/env python3

import rospy
from simple_pid import PID
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16 
from std_msgs.msg import Float32
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

import time
from math import pi
import RPi.GPIO as GPIO


class Kalman_Filter(object):
    def update(self, Z):
        self.Z = Z
        self.K = self._P / (self._P + self.R)
        self.X = self._X + self.K * (self.Z - self._X)
        self.P = (1 - self.K) * self._P

    def predict(self):
        self._P = self.P + self.Q
        self._X = self.X


GPIO.setmode(GPIO.BCM)
HIGH = True
LOW = False

# Parameters
# =====================================

# This code use PCA9685 to produce PWM, no need to use PWM pins
# PCA9685: Use channel 1 for motor 1, channel 2 for motor 2

# Motor 1 pins
AIN1 = 27
AIN2 = 22
ENCODER_A_1 = 20
ENCODER_B_1 = 16

# Motor 2 pins

BIN1 = 6
BIN2 = 5
ENCODER_A_2 = 10
ENCODER_B_2 = 9

STANDBY = 17

# =====================================

# PID parameters of motor 1
Kp_1 = 3
Ki_1 = 3
Kd_1 = 0

# PID parameters of motor 2
Kp_2 = 3.1
Ki_2 = 3
Kd_2 = 0.07

PWM_FREQUENCY = 1000
SAMPLE_TIME = 5e-3
PULSES_OF_ENCODER_PER_ROUND = 480
DIAMETER_OF_LEFT_WHEEL = 0.09  # m
DIAMETER_OF_RIGHT_WHEEL = 0.09  # m

DELAY_KF_PERIOD = 30
KF_TIMER = 0
# =====================================

# Non-configured parameters
prev_time = time.time()
send_time_prev = time.time()

POS_1 = 0
PREV_POS_1 = 0
filtered_encoder_count_RPM_1 = 0
encoder_count_RPM_1 = 0
encoder_count_RPM_prev_1 = 0
velocity_motor_1 = 0  # m/s
PWM_VALUE_MOTOR_1 = 0

POS_2 = 0
PREV_POS_2 = 0
filtered_encoder_count_RPM_2 = 0
encoder_count_RPM_2 = 0
encoder_count_RPM_prev_2 = 0
velocity_motor_2 = 0  # m/s
PWM_VALUE_MOTOR_2 = 0

left_wheel_direction = True
right_wheel_direction = True
LINEAR_VELOCITY_X_LEFT = 0  # m/s
LINEAR_VELOCITY_X_RIGHT = 0  # m/s
ANGULAR_VELOCITY_Z = 0  # PWM (temporary PWM, it should be rad/s (need a little math))
ROBOT_VELOCITY_X = 0
ROBOT_VELOCITY_X_PREV = 0

POS_RAD_LEFT_WHEEL_RPM = -23  # RPM for 1 rad/s (counterclockwise)
POS_RAD_RIGHT_WHEEL_RPM = 23  # RPM for 1 rad/s (counterclockwise)
NEG_RAD_LEFT_WHEEL_RPM = 23  # RPM for -1 rad/s (clockwise)
NEG_RAD_RIGHT_WHEEL_RPM = -23  # RPM for -1 rad/s (clockwise)

print("Waiting for creating PCA9685 instance...")

I2C_BUS = busio.I2C(SCL, SDA)
PCA = PCA9685(I2C_BUS)
PCA.frequency = PWM_FREQUENCY

print("Ready!")
# State variable to reset KF when start going straight
current_state_is_straight = True
prev_state_is_straight = True

PERIMETER_OF_LEFT_WHEEL = DIAMETER_OF_LEFT_WHEEL * pi
PERIMETER_OF_RIGHT_WHEEL = DIAMETER_OF_RIGHT_WHEEL * pi


KF_1 = Kalman_Filter()
KF_2 = Kalman_Filter()
DELAY_KF_TIME = DELAY_KF_PERIOD * SAMPLE_TIME

# Initialize 2 PID Controllers
PID_1 = PID(
    sample_time=SAMPLE_TIME,
    Kp=Kp_1,
    Ki=Ki_1,
    Kd=Kd_1,
    output_limits=(0, 100),
    setpoint=0,
)

PID_2 = PID(
    sample_time=SAMPLE_TIME,
    Kp=Kp_2,
    Ki=Ki_2,
    Kd=Kd_2,
    output_limits=(0, 100),
    setpoint=0,
)


# Initialize 2 Kalman Filters
# VALUES FOR KALMAN FILTER ARE RPM OF ENCODERS


def reset_KF():
    global KF_1, KF_2, KF_TIMER
    KF_1._P = 10000
    KF_1.Q = 0
    KF_1.R = 273
    KF_1._X = 0
    KF_1.X = 0

    KF_2._P = 10000
    KF_2.Q = 0
    KF_2.R = 273
    KF_2._X = 0
    KF_2.X = 0

    KF_TIMER = time.time()


def reset_PID_controller():
    PID_1.setpoint = 0
    PID_2.setpoint = 0


def encoder_check_1(channel):
    global POS_1
    if GPIO.input(ENCODER_B_1):
        POS_1 += 1
    else:
        POS_1 -= 1


def encoder_check_2(channel):
    global POS_2
    if GPIO.input(ENCODER_B_2):
        POS_2 += 1
    else:
        POS_2 -= 1


def calculate_velocity_motor_1(delta_time):
    global filtered_encoder_count_RPM_1, encoder_count_RPM_prev_1, POS_1, PREV_POS_1, KF_1
    encoder_count_per_second_1 = abs(POS_1 - PREV_POS_1) / delta_time
    encoder_count_RPM_1 = (
        encoder_count_per_second_1 / PULSES_OF_ENCODER_PER_ROUND * 60.0
    )

    # Low pass filter (smaller than 25Hz pass)
    filtered_encoder_count_RPM_1 = (
        0.854 * filtered_encoder_count_RPM_1
        + 0.0728 * encoder_count_RPM_1
        + 0.0728 * encoder_count_RPM_prev_1
    )
    encoder_count_RPM_prev_1 = encoder_count_RPM_1

    # Kalman Filter
    if time.time() - KF_TIMER <= DELAY_KF_TIME:
        KF_1.X = filtered_encoder_count_RPM_1
    else:
        KF_1.update(filtered_encoder_count_RPM_1)
        KF_1.predict()

    PREV_POS_1 = POS_1
    real_velocity_motor_1 = KF_1.X
    return real_velocity_motor_1


def calculate_velocity_motor_2(delta_time):
    global filtered_encoder_count_RPM_2, encoder_count_RPM_prev_2, POS_2, PREV_POS_2, KF_2
    encoder_count_per_second_2 = abs(POS_2 - PREV_POS_2) / delta_time
    encoder_count_RPM_2 = (
        encoder_count_per_second_2 / PULSES_OF_ENCODER_PER_ROUND * 60.0
    )

    # Low pass filter (smaller than 25Hz pass)
    filtered_encoder_count_RPM_2 = (
        0.854 * filtered_encoder_count_RPM_2
        + 0.0728 * encoder_count_RPM_2
        + 0.0728 * encoder_count_RPM_prev_2
    )
    encoder_count_RPM_prev_2 = encoder_count_RPM_2

    # Kalman Filter
    if time.time() - KF_TIMER <= DELAY_KF_TIME:
        KF_2.X = filtered_encoder_count_RPM_2
    else:
        KF_2.update(filtered_encoder_count_RPM_2)
        KF_2.predict()

    PREV_POS_2 = POS_2
    real_velocity_motor_2 = KF_2.X
    return real_velocity_motor_2


def pre_calculate(msg):
    global ROBOT_VELOCITY_X, ROBOT_VELOCITY_X_PREV, left_wheel_direction, right_wheel_direction, LINEAR_VELOCITY_X_LEFT, LINEAR_VELOCITY_X_RIGHT, ANGULAR_VELOCITY_Z, PWM_VALUE_MOTOR_1, PWM_VALUE_MOTOR_2, prev_state_is_straight, current_state_is_straight

    ROBOT_VELOCITY_X = msg.linear.x  # m/s

    if ROBOT_VELOCITY_X != ROBOT_VELOCITY_X_PREV:
        reset_KF()
        reset_PID_controller()
        ROBOT_VELOCITY_X_PREV = ROBOT_VELOCITY_X

    LINEAR_VELOCITY_X_LEFT = msg.linear.x / PERIMETER_OF_LEFT_WHEEL * 60  # RPM
    LINEAR_VELOCITY_X_RIGHT = msg.linear.x / PERIMETER_OF_RIGHT_WHEEL * 60  # RPM
    ANGULAR_VELOCITY_Z = msg.angular.z  # rad/s

    if ANGULAR_VELOCITY_Z:
        current_state_is_straight = False
    elif not ANGULAR_VELOCITY_Z:
        current_state_is_straight = True

    if current_state_is_straight != prev_state_is_straight:
        reset_KF()
        prev_state_is_straight = current_state_is_straight

    if ANGULAR_VELOCITY_Z >= 0:
        LINEAR_VELOCITY_X_LEFT += POS_RAD_LEFT_WHEEL_RPM * abs(ANGULAR_VELOCITY_Z)
        LINEAR_VELOCITY_X_RIGHT += POS_RAD_RIGHT_WHEEL_RPM * abs(ANGULAR_VELOCITY_Z)
    elif ANGULAR_VELOCITY_Z < 0:
        LINEAR_VELOCITY_X_LEFT += NEG_RAD_LEFT_WHEEL_RPM * abs(ANGULAR_VELOCITY_Z)
        LINEAR_VELOCITY_X_RIGHT += NEG_RAD_RIGHT_WHEEL_RPM * abs(ANGULAR_VELOCITY_Z)

    if LINEAR_VELOCITY_X_LEFT >= 0:
        left_wheel_direction = True
    elif LINEAR_VELOCITY_X_LEFT < 0:
        left_wheel_direction = False

    if LINEAR_VELOCITY_X_RIGHT >= 0:
        right_wheel_direction = True
    elif LINEAR_VELOCITY_X_RIGHT < 0:
        right_wheel_direction = False


def PID_diff_control(velocity_motor_1=0, velocity_motor_2=0):
    global PWM_VALUE_MOTOR_1, PWM_VALUE_MOTOR_2, PID_1, PID_2

    # PID Controller (include differential drive)

    PID_1.setpoint = abs(LINEAR_VELOCITY_X_LEFT)
    PWM_VALUE_MOTOR_1 = PID_1(input_=velocity_motor_1)

    PID_2.setpoint = abs(LINEAR_VELOCITY_X_RIGHT)
    PWM_VALUE_MOTOR_2 = PID_2(input_=velocity_motor_2)


def control_motors(
    pwm_value_motor_1=0,
    pwm_value_motor_2=0,
    left_wheel_direction=True,
    right_wheel_direction=True,
):
    # pwm_value_motor_1
    # pwm_value_motor_2
    # left_wheel_direction
    # right_wheel_direction

    global PCA

    if (not LINEAR_VELOCITY_X_LEFT) and (not LINEAR_VELOCITY_X_RIGHT):
        reset_KF()
        reset_PID_controller()
        PCA.channels[1].duty_cycle = 0
        PCA.channels[2].duty_cycle = 0

    else:

        if left_wheel_direction:
            GPIO.output(AIN1, LOW)
            GPIO.output(AIN2, HIGH)
        elif not left_wheel_direction:
            GPIO.output(AIN1, HIGH)
            GPIO.output(AIN2, LOW)

        if right_wheel_direction:
            GPIO.output(BIN1, HIGH)
            GPIO.output(BIN2, LOW)
        elif not right_wheel_direction:
            GPIO.output(BIN1, LOW)
            GPIO.output(BIN2, HIGH)

        PCA.channels[1].duty_cycle = round(pwm_value_motor_1 * 65535 / 100)
        PCA.channels[2].duty_cycle = round(pwm_value_motor_2 * 65535 / 100)


def setup():
    GPIO.setwarnings(False)

    GPIO.setup(ENCODER_A_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(ENCODER_B_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    GPIO.setup(ENCODER_A_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(ENCODER_B_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    GPIO.add_event_detect(ENCODER_A_1, GPIO.RISING, callback=encoder_check_1)
    GPIO.add_event_detect(ENCODER_A_2, GPIO.RISING, callback=encoder_check_2)

    GPIO.setup(AIN1, GPIO.OUT)
    GPIO.setup(AIN2, GPIO.OUT)

    GPIO.setup(BIN1, GPIO.OUT)
    GPIO.setup(BIN2, GPIO.OUT)

    GPIO.setup(STANDBY, GPIO.OUT)

    GPIO.output(AIN1, HIGH)
    GPIO.output(AIN2, LOW)

    GPIO.output(BIN1, HIGH)
    GPIO.output(BIN2, LOW)

    GPIO.output(STANDBY, HIGH)


def turn_off_all_pwm_pca():
    global PCA
    for ch in range(0, 16):
        PCA.channels[ch].duty_cycle = 0


def cleanup():
    GPIO.setmode(GPIO.BCM)
    turn_off_all_pwm_pca()
    for pin in range(2, 27):
        try:
            GPIO.setup(pin, GPIO.IN)
            GPIO.output(pin, False)
        except:
            continue


def main():
    global velocity_motor_1, velocity_motor_2, prev_time, send_time_prev, PCA
    right_tick_pub = rospy.Publisher("right_ticks", Int16, queue_size=1000)
    left_tick_pub = rospy.Publisher("left_ticks", Int16, queue_size=1000)
    right_wheel_speed = rospy.Publisher("right_wheel_speed", Float32, queue_size=1000)
    left_wheel_speed = rospy.Publisher("left_wheel_speed", Float32, queue_size=1000)

    rospy.init_node("motor_with_tick", anonymous=False)

    rospy.Subscriber("cmd_vel", Twist, pre_calculate)

    reset_KF()
    reset_PID_controller()

    print("hello")
    # PCA.channels[1].duty_cycle = 65535
    # PCA.channels[2].duty_cycle = 65535

    while not rospy.is_shutdown():
        current_time = time.time()
        send_time = time.time()
        if (current_time - prev_time) >= SAMPLE_TIME:
            delta_time = current_time - prev_time

            velocity_motor_1 = calculate_velocity_motor_1(delta_time)
            velocity_wheel_1 = velocity_motor_1 / 60 * PERIMETER_OF_LEFT_WHEEL

            velocity_motor_2 = calculate_velocity_motor_2(delta_time)
            velocity_wheel_2 = velocity_motor_2 / 60 * PERIMETER_OF_RIGHT_WHEEL

            PID_diff_control(velocity_motor_1, velocity_motor_2)
            control_motors(
                pwm_value_motor_1=PWM_VALUE_MOTOR_1,
                pwm_value_motor_2=PWM_VALUE_MOTOR_2,
                left_wheel_direction=left_wheel_direction,
                right_wheel_direction=right_wheel_direction,
            )

            prev_time = current_time

        if send_time - send_time_prev >= 0.1:
            left_tick_pub.publish(POS_1)
            right_tick_pub.publish(POS_2)
            # left_wheel_speed.publish(velocity_wheel_1)
            # right_wheel_speed.publish(velocity_wheel_2)

            send_time_prev = send_time
            # print(velocity_motor_2)

        # if time.time() - send_time >= 1:
        #     rospy.loginfo("Setpoint velocity: " + str(ROBOT_VELOCITY_X))
        #     rospy.loginfo("velocity_motor_1: " + str(velocity_motor_1))
        #     rospy.loginfo("velocity_motor_2: " + str(velocity_motor_2))

        #     send_time = time.time()

        # left_tick_pub.publish(POS_1)
        # right_tick_pub.publish(POS_2)
        # left_wheel_speed.publish(velocity_motor_1)
        # right_wheel_speed.publish(velocity_motor_2)

        # rospy.loginfo("Setpoint velocity: " + str(ROBOT_VELOCITY_X))
        # rospy.loginfo("velocity_motor_1: " + str(velocity_motor_1))
        # rospy.loginfo("velocity_motor_2: " + str(velocity_motor_2))

        time.sleep(0.0001)

        # rate.sleep()


if __name__ == "__main__":
    try:
        setup()
        main()
    finally:
        cleanup()
        exit(0)

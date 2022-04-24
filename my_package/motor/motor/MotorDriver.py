from motor.KalmanFilter import Kalman_Filter
import time

class MotorDriver(object):
    def __init__(
        self, diameter, pulse_per_round_of_encoder, pwm_frequency, sample_time
    ):
        """Motor driver class

        Args:
            diameter (float): diameter of wheel (m)
            pulse_per_round_of_encoder (int): as name
            pwm_frequency (int): pwm frequency
            sample_time (float): sample time to drive motor
        """
        self.__checkConditions(
            diameter, pulse_per_round_of_encoder, pwm_frequency, sample_time
        )

        self.__initializeParameters()

        # Initialize Kalman Filter

        # Initialize PID controller

    def __checkConditions(
        self, diameter, pulse_per_round_of_encoder, pwm_frequency, sample_time
    ):
        """Check conditions of MotorDriver parameters."""
        self.__checkDiamater(diameter)
        self.__checkPulsePerRoundOfEncoder(pulse_per_round_of_encoder)
        self.__checkPwmFrequency(pwm_frequency)
        self.__checkSampleTime(sample_time)

    def __checkDiamater(self, diameter):
        if float(diameter) and diameter > 0:
            self.__diameter = diameter
        else:
            raise Exception("Invalid value of diameter!")

    def __checkPulsePerRoundOfEncoder(self, pulse_per_round_of_encoder):
        if float(pulse_per_round_of_encoder) and pulse_per_round_of_encoder > 0:
            self.__pulse_per_round_of_encoder = pulse_per_round_of_encoder
        else:
            raise Exception("Invalid value of pulse_per_round_of_encoder!")

    def __checkPwmFrequency(self, pwm_frequency):
        if float(pwm_frequency) and pwm_frequency > 0:
            self.__pwm_frequency = pwm_frequency
        else:
            raise Exception("Invalid value of pwm_frequency!")

    def __checkSampleTime(self, sample_time):
        if float(sample_time) and sample_time > 0:
            self.__sample_time = sample_time
        else:
            raise Exception("Invalid value of sample_time!")

    def __initializeParameters(self):
        """Initialize private parameters."""
        self.__timer = 0

        self.__lowPassFilteredRPM = 0
        self.__RPM = 0

        self.__previous_tick = 0
        self.__previous_RPM = 0

        self.__filtered_RPM_coefficient = 0.854
        self.__RPM_coefficient = 0.0728
        self.__previous_RPM_coefficient = 0.0728

        self.__KF = Kalman_Filter()

    # Low pass filter (smaller than 25Hz pass)
    def __lowPassFilter(self):
        """Filter high-frequency interference signal of the encoder."""
        self.__lowPassFilteredRPM = (
            self.__filtered_RPM_coefficient * self.__lowPassFilteredRPM
            + self.__RPM_coefficient * self.__RPM
            + self.__previous_RPM_coefficient * self.__previous_RPM
        )
        self.__previous_RPM = self.__RPM

    # Calculate RPM of Motor
    def calculateRPM(self, current_tick):
        if time.time() - self.__timer >= self.__sample_time:
            self.__encoder_count_per_second = (
                abs(current_tick - self.__previous_tick) / self.__sample_time
            )
            self.__RPM = (
                self.__encoder_count_per_second / self.__pulse_per_round_of_encoder * 60.0
            )
            self.__lowPassFilter()
            self.__previous_tick = current_tick
            # something with KF

            self.__timer = time.time()

    def changeCoefficientLowPassFilter(
        self, filtered_RPM_coefficient, RPM_coefficient, previous_RPM_coefficient
    ):
        """Change coefficients of Low-pass Filter.

        Args:
            (float) filtered_RPM_coefficient = 0 \n
            (float) RPM_coefficient = 0 \n
            (float) previous_RPM_coefficient = 0 \n
        """
        self.__filtered_RPM_coefficient = filtered_RPM_coefficient
        self.__RPM_coefficient = RPM_coefficient
        self.__previous_RPM_coefficient = previous_RPM_coefficient

    def changeCoefficientKalmanFilter(self):
        pass

    def getRPM(self):
        return self.__lowPassFilteredRPM

    def getPWMFrequency(self):
        return self.__pwm_frequency

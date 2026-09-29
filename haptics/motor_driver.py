import numpy as np
from gpiozero import PWMOutputDevice

class MotorDriver:
    def __init__(self, pins=(17, 27, 22, 23, 24), min_duty=0.18, d_min=0.4, d_max=1.5):
        self.motor_angles = np.array([-90.0, -45.0, 0.0, 45.0, 90.0])
        self.min_duty = min_duty
        self.d_min = d_min
        self.d_max = d_max
        self.motors = [PWMOutputDevice(pin, frequency=200) for pin in pins]

    def _distance_to_intensity(self, dist_m):
        if dist_m <= self.d_min:
            return 1.0
        if dist_m >= self.d_max:
            return 0.0
        return (self.d_max - dist_m) / (self.d_max - self.d_min)

    def apply_feedback(self, dynamic_threats):
        """
        Distributes proximity-based vibration intensities across the 5 coin motors
        using linear angular interpolation for obstacles within the warning bubble.
        """
        motor_intensities = np.zeros(5)

        for threat in dynamic_threats:
            dist_m = threat["dist_m"]
            angle = threat["angle_deg"]

            if dist_m > self.d_max or not (-90.0 <= angle <= 90.0):
                continue

            base_power = self._distance_to_intensity(dist_m)

            for i in range(len(self.motor_angles) - 1):
                left_a = self.motor_angles[i]
                right_a = self.motor_angles[i + 1]

                if left_a <= angle <= right_a:
                    span = right_a - left_a
                    ratio = (angle - left_a) / span
                    w_left = 1.0 - ratio
                    w_right = ratio

                    motor_intensities[i] = max(motor_intensities[i], base_power * w_left)
                    motor_intensities[i + 1] = max(motor_intensities[i + 1], base_power * w_right)
                    break

        for i in range(5):
            val = motor_intensities[i]
            if val > 0.02:
                duty = self.min_duty + val * (1.0 - self.min_duty)
                self.motors[i].value = min(max(duty, 0.0), 1.0)
            else:
                self.motors[i].value = 0.0

    def stop_all(self):
        for m in self.motors:
            m.value = 0.0
            m.close()
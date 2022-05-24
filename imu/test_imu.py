from mpu6050 import mpu6050
import time

sensor = mpu6050(0x68)

imu_data = sensor.get_all_data()

if __name__ == "__main__":
    mpu = mpu6050(0x68)
    while True:
        temp = mpu.get_temp()
        accel_data = mpu.get_accel_data()
        gyro_data = mpu.get_gyro_data()
        print(
            "Gx=%.2f" % gyro_data["x"],
            u"\u00b0" + "/s",
            "\tGy=%.2f" % gyro_data["y"],
            u"\u00b0" + "/s",
            "\tGz=%.2f" % gyro_data["z"],
            u"\u00b0" + "/s",
            "\tAx=%.2f g" % accel_data["x"],
            "\tAy=%.2f g" % accel_data["y"],
            "\tAz=%.2f g" % accel_data["z"],
            "\tTemp=%.2f oC" % temp,
        )

        time.sleep(0.1)

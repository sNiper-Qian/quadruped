from MPU6050.MPU6050 import MPU6050
import time
import numpy as np

class IMU():
    def __init__(self):
        i2c_bus = 1
        device_address = 0x68
        x_accel_offset = int(2435)
        y_accel_offset = int(2955)
        z_accel_offset = int(-647)
        x_gyro_offset = int(32)
        y_gyro_offset = int(-44)
        z_gyro_offset = int(39)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.offset = np.array([2.5, 84.7, 24.8])
        self.last_rpy = self.offset
        enable_debug_output = True
        self.mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
                      z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,enable_debug_output)
        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)
        self.mpu_int_status = self.mpu.get_int_status()
        # print(hex(self.mpu_int_status))

        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()
        # print(self.packet_size)
        self.FIFO_count = self.mpu.get_FIFO_count()
        # print(self.FIFO_count)
        

        self.FIFO_buffer = [0]*64
        
    def callibrate(self):
        last_yaw = 0
        try:
            while True:
                try:
                    self.FIFO_count = self.mpu.get_FIFO_count()
                    self.mpu_int_status = self.mpu.get_int_status()
                except:
                    continue
                # If overflow is detected by status or fifo count we want to reset
                if (self.FIFO_count == 1024) or (self.mpu_int_status & 0x10):
                    self.mpu.reset_FIFO()
                    # print('overflow!')
                # Check if fifo data is ready
                elif (self.mpu_int_status & 0x02):
                    # Wait until packet_size number of bytes are ready for reading, default
                    # is 42 bytes
                    while self.FIFO_count < self.packet_size:
                        self.FIFO_count = self.mpu.get_FIFO_count()
                    self.FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
                    accel = self.mpu.DMP_get_acceleration_int16(self.FIFO_buffer)
                    quat = self.mpu.DMP_get_quaternion_int16(self.FIFO_buffer)
                    grav = self.mpu.DMP_get_gravity(quat)
                    roll_pitch_yaw = self.mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
                    
                    str_show = "roll: %.2f  pitch: %.2f  yaw: %.2f        "%(roll_pitch_yaw.x,roll_pitch_yaw.y,roll_pitch_yaw.z)
                    if last_yaw == 0:
                        last_yaw = roll_pitch_yaw.z - 1
                    if roll_pitch_yaw.z - last_yaw < 0.01:
                        break
                    last_yaw = roll_pitch_yaw.z
                    print("\r %s"%(str_show),end='')
            self.offset = np.array([roll_pitch_yaw.x, roll_pitch_yaw.y, roll_pitch_yaw.z])
            self.last_rpy = self.offset
            print(self.offset)
        except KeyboardInterrupt:
            print('\n Ctrl + C QUIT')

    def read(self):
        try_cnt = 0
        while True:
            try:
                self.FIFO_count = self.mpu.get_FIFO_count()
                self.mpu_int_status = self.mpu.get_int_status()
            except:
                continue
            # If overflow is detected by status or fifo count we want to reset
            if (self.FIFO_count == 1024) or (self.mpu_int_status & 0x10):
                self.mpu.reset_FIFO()
                # print('overflow!')
            # Check if fifo data is ready
            elif (self.mpu_int_status & 0x02):
                # Wait until packet_size number of bytes are ready for reading, default
                # is 42 bytes
                while self.FIFO_count < self.packet_size:
                    self.FIFO_count = self.mpu.get_FIFO_count()
                self.FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
                accel = self.mpu.DMP_get_acceleration_int16(self.FIFO_buffer)
                quat = self.mpu.DMP_get_quaternion_int16(self.FIFO_buffer)
                grav = self.mpu.DMP_get_gravity(quat)
                roll_pitch_yaw = self.mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
                
                rpy = np.array([roll_pitch_yaw.x, roll_pitch_yaw.y, roll_pitch_yaw.z])
                if np.linalg.norm(rpy-self.last_rpy) < 20 or try_cnt >= 3: 
                    self.last_rpy = rpy
                    ret = rpy - self.offset
                    self.roll = ret[0]
                    self.pitch = ret[1]
                    self.yaw = ret[2]
                    str_show = "roll: %.2f  pitch: %.2f  yaw: %.2f        "%(self.roll,
                                                                             self.pitch,
                                                                             self.yaw)
                    print("\r %s"%(str_show),end='')
                    break
                else:
                    # print("filtering")
                    try_cnt += 1
                    continue
        return (self.roll, self.pitch, self.yaw)

if __name__ == "__main__":
    imu = IMU()
    imu.callibrate()
    for i in range(1000):
        a, b, c = imu.read()
        print(a, b, c)
        time.sleep(0.1)
    # i2c_bus = 1
    # device_address = 0x68

    # x_accel_offset = int(-354)
    # y_accel_offset = int(-1729)
    # z_accel_offset = int(-331)
    # x_gyro_offset = int(66)
    # y_gyro_offset = int(39)
    # z_gyro_offset = int(30)

    # # x_avg_read: 0.38 x_avg_offset: -160.919625000002
    # # y_avg_read: -0.34 y_avg_offset: -1895.1451250000005
    # # z_avg_read: 0.74 z_avg_offset: -331.15837499999924
    # # x_avg_read: -0.03 x_avg_offset: 66.00206250000024
    # # y_avg_read: 0.18 y_avg_offset: 39.70262499999929
    # # z_avg_read: -0.05 z_avg_offset: 30.895687500000278

    # enable_debug_output = True
    # mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,enable_debug_output)

    # mpu.dmp_initialize()
    # mpu.set_DMP_enabled(True)
    # mpu_int_status = mpu.get_int_status()
    # print(hex(mpu_int_status))

    # packet_size = mpu.DMP_get_FIFO_packet_size()
    # print(packet_size)
    # FIFO_count = mpu.get_FIFO_count()
    # print(FIFO_count)

    # FIFO_buffer = [0]*64
    # try:
    #     while True:
    #         try:
    #             FIFO_count = mpu.get_FIFO_count()
    #             mpu_int_status = mpu.get_int_status()
    #         except:
    #             continue
    #         # If overflow is detected by status or fifo count we want to reset
    #         if (FIFO_count == 1024) or (mpu_int_status & 0x10):
    #             mpu.reset_FIFO()
    #             print('overflow!')
    #         # Check if fifo data is ready
    #         elif (mpu_int_status & 0x02):
    #             # Wait until packet_size number of bytes are ready for reading, default
    #             # is 42 bytes
    #             while FIFO_count < packet_size:
    #                 FIFO_count = mpu.get_FIFO_count()
    #             FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
    #             accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
    #             quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
    #             grav = mpu.DMP_get_gravity(quat)
    #             roll_pitch_yaw = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
                
    #             str_show = "roll: %.2f  pitch: %.2f  yaw: %.2f        "%(roll_pitch_yaw.x,roll_pitch_yaw.y,roll_pitch_yaw.z)
              
    #             print("\r %s"%(str_show),end='')
    # except KeyboardInterrupt:
    #     print('\n Ctrl + C QUIT')
           

    
import time
from MPU6050 import MPU6050
from pin_dic import pin_dic
import RPi.GPIO as GPIO


def action(channel):
    global mpu
    global FIFO_buffer
    global packet_size
    
    try:
        FIFO_count = mpu.get_FIFO_count()
        mpu_int_status = mpu.get_int_status()
    except:
        return
    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count == 1024) or (mpu_int_status & 0x10):
        mpu.reset_FIFO()
        # print('overflow!')
        return
    # Check if fifo data is ready
    elif (mpu_int_status & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default
        # is 42 bytes
        while FIFO_count < packet_size:
            FIFO_count = mpu.get_FIFO_count()
        try:
            FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
        except:
            return    
        accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
        quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
        grav = mpu.DMP_get_gravity(quat)
        roll_pitch_yaw = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
        
        str_show = "roll: %.2f  pitch: %.2f  yaw: %.2f        "%(roll_pitch_yaw.x,roll_pitch_yaw.y,roll_pitch_yaw.z)
      
        print("\r %s"%(str_show),end='')
        
if __name__ == "__main__":        
    pin_IRQ = pin_dic['G27']

    i2c_bus = 1
    device_address = 0x68

    x_accel_offset = int(926)
    y_accel_offset = int(2136)
    z_accel_offset = int(-856)
    x_gyro_offset = int(36)
    y_gyro_offset = int(-8)
    z_gyro_offset = int(-28)

    enable_debug_output = True

    mpu = MPU6050(i2c_bus, device_address, x_accel_offset,
                  y_accel_offset, z_accel_offset, x_gyro_offset,
                  y_gyro_offset, z_gyro_offset, enable_debug_output)
                  
    mpu.dmp_initialize()
    mpu.set_DMP_enabled(True)
    mpu_int_status = mpu.get_int_status()
    print(hex(mpu_int_status))

    packet_size = mpu.DMP_get_FIFO_packet_size()
    print(packet_size)
    FIFO_count = mpu.get_FIFO_count()
    print(FIFO_count)

    FIFO_buffer = [0]*64

    GPIO.setmode(GPIO.BOARD) 
    GPIO.setup(pin_IRQ, GPIO.IN)
    GPIO.add_event_detect(pin_IRQ, GPIO.RISING, callback=action)

    try:
        while True:
            pass
    except KeyboardInterrupt:
        GPIO.cleanup()


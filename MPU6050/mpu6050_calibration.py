from MPU6050 import MPU6050
import time
import numpy as np



def meansensors(mpu):

    buffersize=1000
    buff_ax=0
    buff_ay=0
    buff_az=0
    buff_gx=0
    buff_gy=0
    buff_gz=0
    i = 0
    while i<buffersize+101:
        try:
            a_raw = mpu.get_acceleration()
            g_raw = mpu.get_rotation()
        except:
            continue
        
        
        if i>100 and i<=(buffersize+101):
            buff_ax=buff_ax+a_raw[0]
            buff_ay=buff_ay+a_raw[1]
            buff_az=buff_az+a_raw[2]
            buff_gx=buff_gx+g_raw[0]
            buff_gy=buff_gy+g_raw[1]
            buff_gz=buff_gz+g_raw[2]
        if (i==(buffersize+100)):
            mean_ax=buff_ax/buffersize
            mean_ay=buff_ay/buffersize
            mean_az=buff_az/buffersize
            mean_gx=buff_gx/buffersize
            mean_gy=buff_gy/buffersize
            mean_gz=buff_gz/buffersize
        
        i=i+1
        time.sleep(0.002)
    
    return int(mean_ax),int(mean_ay),int(mean_az),int(mean_gx),int(mean_gy),int(mean_gz)
    
def calibration(mpu,means_out):
    
    acel_deadzone=8
    giro_deadzone=1 
    mean_ax = means_out[0]
    mean_ay = means_out[1]
    mean_az = means_out[2]
    mean_gx = means_out[3]
    mean_gy = means_out[4]
    mean_gz = means_out[5]
    
    ax_offset=-mean_ax/8
    ay_offset=-mean_ay/8
    az_offset=(16384-mean_az)/8

    gx_offset= -mean_gx/4
    gy_offset= -mean_gy/4
    gz_offset= -mean_gz/4
    while True:
        ready = 0
        mpu.set_x_accel_offset(int(ax_offset))
        mpu.set_y_accel_offset(int(ay_offset))
        mpu.set_z_accel_offset(int(az_offset))
        
        mpu.set_x_gyro_offset(int(gx_offset))
        mpu.set_y_gyro_offset(int(gy_offset))
        mpu.set_z_gyro_offset(int(gz_offset))
        
        means_out = meansensors(mpu)
        mean_ax = means_out[0]
        mean_ay = means_out[1]
        mean_az = means_out[2]
        mean_gx = means_out[3]
        mean_gy = means_out[4]
        mean_gz = means_out[5]
    
        print("...")
        print(means_out)
        print(ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset)
        
        
        if abs(mean_ax)<= acel_deadzone:
            ready = ready+1
        else:
            ax_offset=ax_offset-mean_ax/acel_deadzone
       
        if abs(mean_ay)<= acel_deadzone:
            ready = ready+1
        else:
            ay_offset=ay_offset-mean_ay/acel_deadzone
    
        if abs(mean_az)<= acel_deadzone:
            ready = ready+1
        else:
            az_offset=az_offset-mean_az/acel_deadzone
    
        if abs(mean_gx)<= giro_deadzone:
            ready = ready+1
        else:
            gx_offset=gx_offset-mean_gx/(giro_deadzone+1)
            
        if abs(mean_gy)<= giro_deadzone:
            ready = ready+1
        else:
            gy_offset=gy_offset-mean_gy/(giro_deadzone+1)
            
        if abs(mean_gz)<= giro_deadzone:
            ready = ready+1
        else:
            gz_offset=gz_offset-mean_gz/(giro_deadzone+1)
            
            
        if ready==6:
            break
    return ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset
   
  
if __name__ == "__main__":
    
    i2c_bus = 1
    device_address = 0x68

    x_accel_offset = int(925)
    y_accel_offset = int(2139)
    z_accel_offset = int(-850)
    x_gyro_offset = int(36)
    y_gyro_offset = int(-8)
    z_gyro_offset = int(-29)

    # 
    enable_debug_output = True
    mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,enable_debug_output)
   
    
    state =0
    while True:
        if state ==0:
            print("Reading sensors for first time...")
            means_out = meansensors(mpu)
            state = state +1
            time.sleep(1)
        elif state == 1:
            print("\nCalculating offsets...")
            offsets = calibration(mpu,means_out)
            state = state + 1
            time.sleep(1)
        elif state == 2:
            means_out = meansensors(mpu)
            print("means out")
            print(means_out)
            print("offsets")
            print(offsets)
            break
            
            
            

import numpy as np
import math
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import time
import matplotlib.pyplot as plt
from simple_pid import PID
from imu import IMU

CONTROL_INTERVAL = 0.01 # Control frequency of the robot

class Leg():
    def __init__(self, idx, servo1, servo2, side, l1 = 8, l2 = 7) -> None:
        '''
        Class for a single leg of the robot
        Parameters:
            idx: Index of the leg
            servo1: Servo object for the upper leg
            servo2: Servo object for the lower leg
            side: Side of the robot the leg is on, either 'left' or 'right'
            l1: Upper leg length
            l2: Lower leg length
        Returns:
            None
        '''
        self.idx = idx # Index of the leg
        self.l1 = l1 # Upper leg length
        self.l2 = l2 # Lower leg length
        self.side = side # Side of the robot the leg is on
        self.servo1 = servo1 # Upper leg servo
        self.servo2 = servo2 # Lower leg servo
        self.get_thetas() # Get current angles of the upper and lower leg
        self.kinematics(self.theta1, self.theta2) # Get current position of the foot

    def servo2_angle_to_theta2(self, servo_angle):
        '''
        Transform the servo angle to the angle of lower leg 
        Parameters:
            servo_angle: Angle of the servo
        Returns:
            theta: Angle of the lower leg
        '''
        a = 7.1 # Rod length: 7.1cm
        b = 1.7 # Servo blade: 1.7cm
        servo_angle = math.radians(servo_angle)
        alpha = math.asin(math.sin(math.pi-servo_angle)*b/a)  
        gamma = math.pi - alpha - (math.pi-servo_angle)
        c = a * math.sin(gamma)/math.sin(math.pi-servo_angle)
        theta = math.acos((53-c**2)/28)
        theta = math.degrees(theta)
        theta += 25
        return theta
    
    def theta2_to_servo2_angle(self, theta):
        '''
        Transform the angle of lower leg to servo angle
        Parameters:
            theta: Angle of the lower leg
        Returns:
            servo_angle: Angle of the servo
        '''
        a = 7.1 # Rod length: 7.1cm
        b = 1.7 # Servo blade: 1.7cm
        theta -= 25
        theta = math.radians(theta)
        c = math.sqrt(53 - 28 * math.cos(theta))
        servo_angle = math.acos(max(min((b**2+c**2-a**2)/(2*b*c), 1), -1))
        servo_angle = math.degrees(servo_angle)
        servo_angle = 180 - servo_angle
        return servo_angle

    def get_thetas(self):
        '''
        Get the angles of the legs 
        Parameters:
            None
        Returns:
            (theta1, theta2): Tuple of the angles of the upper and lower leg
        '''
        if self.side == 'left':
            theta1 = 180 - self.servo1.angle # If the leg is on the left side, the angle between the upper leg and body is 180 - servo angle
            theta2 = self.servo2_angle_to_theta2(self.servo2.angle)
        else:
            theta1 = self.servo1.angle + 0.44 # Offset 
            theta2 = self.servo2_angle_to_theta2(180 - self.servo2.angle) - 0.29
        self.theta1 = theta1
        self.theta2 = theta2
        # print(self.idx, self.theta1, self.theta2)
        return (theta1, theta2)
    
    def kinematics(self, theta1, theta2):
        '''
        Calculate the position of the foot given the angles of the upper and lower leg
        Parameters:
            theta1: Angle of the upper leg
            theta2: Angle of the lower leg
        Returns:
            (x, y): Tuple of the position of the foot
        '''
        # self.servo1.angle = theta1
        # self.servo2.angle = 180-self.theta2_to_servo2_angle(theta2)
        theta1 = math.radians(theta1)
        theta2 = math.radians(theta2)
        x = self.l1 * math.cos(theta1) + self.l2 * math.cos(theta1 + theta2)
        y = self.l1 * math.sin(theta1) + self.l2 * math.sin(theta1 + theta2)
        self.x = -x
        self.y = y
        return (-x, y)

    def inverse_kinematics(self, x, y):
        '''
        Calculate the angles of the upper and lower leg given the desired position of the foot
        Parameters:
            x: Desired x position of the foot
            y: Desired y position of the foot
        Returns:
            (theta1, theta2): Tuple of the angles of the upper and lower leg
        '''
        x = -x
        theta2 = math.pi - math.acos((x**2 + y**2 - self.l1**2 - self.l2**2) / (-2 * self.l1 * self.l2))
        phi = math.acos((self.l1**2 + x**2 + y**2 - self.l2**2) / (2 * self.l1 * math.sqrt(x**2 + y**2)))
        if x > 0:
            theta1 = math.atan(y / x) - phi
        elif x < 0:
            theta1 = math.pi - abs(math.atan(y / x)) - phi
        else:
            theta1 = math.pi / 2 - phi
        theta1 = math.degrees(theta1)
        theta2 = math.degrees(theta2) 
        # print(self.idx, theta1, theta2)
        return (theta1, theta2)

    def move_servo(self, theta1, theta2):
        '''
        Move the servos to the given angles
        Parameters:
            theta1: Angle of the upper leg
            theta2: Angle of the lower leg
        Returns:
            None
        '''
        delta_servo1 = 0
        delta_servo2 = 0
        # if self.idx == 1:
        #     delta_servo1 = 0
        #     delta_servo2 = -10
        # if self.idx == 3:
        #     delta_servo1 = 5
        #     delta_servo2 = 0
        if self.side == 'left':
            self.servo1.angle = min(max(180 - theta1 + delta_servo1, 0), 180)
            servo_angle = self.theta2_to_servo2_angle(theta2) + delta_servo2
            self.servo2.angle = min(max(servo_angle, 45), 160)
            # print(theta1, servo_angle)
        else:
            self.servo1.angle = min(max(theta1 + delta_servo1, 0), 180) 
            servo_angle = self.theta2_to_servo2_angle(theta2) + delta_servo2 
            self.servo2.angle = min(max(180-servo_angle, 20), 135) 
            # print(theta1, 180-servo_angle)
        
        
class Robot():
    def __init__(self, pca) -> None:
        '''
        Class for the robot
        Parameters:
            pca: PCA9685 object
        Returns:
            None
        '''
        self.legs = []
        for i in range(4):
            if i % 2 == 0:
                self.legs.append(Leg(i, servo.Servo(pca.channels[i*2], min_pulse=480, max_pulse=2600), 
                                    servo.Servo(pca.channels[i*2+1], min_pulse=480, max_pulse=2600), 
                                    'left'))
            else:
                self.legs.append(Leg(i, servo.Servo(pca.channels[i*2], min_pulse=480, max_pulse=2600),
                                    servo.Servo(pca.channels[i*2+1], min_pulse=480, max_pulse=2600),
                                    'right'))
        self.imu = IMU()
        # self.imu.callibrate()
        self.yaws = []
        self.desired_yaws = []
        kp = 0.05
        kd = 0.001
        ki = 0
        self.desired_yaw = 0
        self.stride = 0
        self.stop = True
        self.turning_angle = 0
        self.turning = False
        self.height_changing = False
        self.delta_height = 0
        self.yaw_pid = PID(kp, ki, kd, setpoint=self.desired_yaw, sample_time=None, output_limits=(-0.95, 0.95))
        self.update()
    
    def move_leg(self, leg, x, y):
        '''
        Move a leg to the given position
        Parameters:
            leg: Leg object
            x: Desired x position of the foot
            y: Desired y position of the foot
        Returns:
            (theta1, theta2): Tuple of the angles of the upper and lower leg
        '''
        theta1, theta2 = leg.inverse_kinematics(x, y)
        leg.move_servo(theta1, theta2)
        return (theta1, theta2)

    def update(self):
        for leg in self.legs:
            leg.get_thetas()
            leg.kinematics(leg.theta1, leg.theta2)
        self.roll, self.pitch, self.yaw = self.imu.read()
        self.yaws.append(self.yaw)
        self.desired_yaws.append(self.desired_yaw)

    def trot(self, steps, height=2, Ts=0.2):
        stance_legs = [0, 3]
        swing_legs = [1, 2]
        for i in range(steps):
            self.update()
            stride_offset = self.yaw_pid(self.yaw)
            self.step(height, Ts, swing_legs, stance_legs, stride_offset)
            stance_legs, swing_legs = swing_legs, stance_legs
    
    def troting(self, mutex, height=2, Ts=0.2):
        stance_legs = [0, 3]
        swing_legs = [1, 2]
        height *= self.legs[0].y/10.5 # Adapt step height according to the height of robot
        try:
            while not self.stop:
                mutex.acquire(1)
                self.update()
                mutex.release()
                stride_offset = self.yaw_pid(self.yaw)
                self.step(height, Ts, swing_legs, stance_legs)
                stance_legs, swing_legs = swing_legs, stance_legs
        except OSError:
            return
    
    def walk(self, steps, height=1.5, Ts=0.1):
        legs = [0, 1, 2, 3]
        stance_legs = []
        swing_legs = []
        for i in range(steps):
            swing_legs=[(i+1)%4]
            stance_legs = list(set(legs) - set(swing_legs))
            self.update()
            self.step(height, Ts, swing_legs, stance_legs)
            if i % 4 == 0:
                self.reset()

    def step(self, height=1, Ts=0.5, swing_legs=[1,3], stance_legs=[0,2], stride_offset=0):
        stride = self.stride
        height = abs(stride)
        loop_start = time.time()
        if stride_offset > 0:
            direction = 'right'
        else:
            direction = 'left'
        while time.time() - loop_start < Ts:
            t = time.time() - loop_start
            for i in range(4):
                if i % 2 == 1: # Right side
                    if direction == 'right':
                        stride_adj = stride * (1- stride_offset)
                    else:
                        stride_adj = stride
                else:
                    if direction == 'left':
                        stride_adj = stride * (1- stride_offset)
                    else:
                        stride_adj = stride
                if i in swing_legs:
                    x = stride_adj * (t/Ts - 1/(2*math.pi)*math.sin(2*math.pi*t/Ts)) + self.legs[i].x
                    if t < Ts / 2:
                        y = -2*height*(t/Ts - 1/(4*math.pi)*math.sin(4*math.pi*t/Ts)) + self.legs[i].y
                    else:
                        y = -2*height*(1 - t/Ts + 1/(4*math.pi)*math.sin(4*math.pi*t/Ts)) + self.legs[i].y
                    theta1, theta2 = self.legs[i].inverse_kinematics(x, y)
                    self.move_leg(self.legs[i], x, y)
                elif i in stance_legs:
                    x = self.legs[i].x - stride_adj * (t/Ts)
                    y = self.legs[i].y
                    self.move_leg(self.legs[i], x, y)
    
    def reset(self):
        self.legs[0].servo1.angle = 150
        self.legs[0].servo2.angle = 60
        self.legs[1].servo1.angle = 30
        self.legs[1].servo2.angle = 120
        self.legs[2].servo1.angle = 150
        self.legs[2].servo2.angle = 60
        self.legs[3].servo1.angle = 30
        self.legs[3].servo2.angle = 120
        self.callibrate()
        self.update()
        # print y of legs
        print([(leg.x, leg.y) for leg in self.legs])

    def sleep(self):
        desired_angle = [180, 160, 0, 20, 180, 160, 0, 20]
        initial_angle = []
        for i in range(4):
            initial_angle.append(self.legs[i].servo1.angle)
            initial_angle.append(self.legs[i].servo2.angle)

        for i in range(100):
            for j in range(4):
                self.legs[j].servo1.angle = initial_angle[j*2] + (desired_angle[j*2] - initial_angle[j*2]) / 100 * i
                self.legs[j].servo2.angle = initial_angle[j*2+1] + (desired_angle[j*2+1] - initial_angle[j*2+1]) / 100 * i
            time.sleep(0.01)
        self.update()

    def callibrate(self):
        # There might be offset of the position of endeffector of each leg after some moving 
        # External callibration is needed to run after each stop
        self.update()
        x = np.mean([leg.x for leg in self.legs])
        y = np.mean([leg.y for leg in self.legs])
        for leg in self.legs:
            self.move_leg(leg, x, y)
        self.update()

    def change_height(self, delta_y):
        x = np.mean([leg.x for leg in self.legs])
        y = np.mean([leg.y for leg in self.legs])
        for leg in self.legs:
            self.move_leg(leg, x, y+delta_y)

    def turning_direction(self, mutex):
        while self.turning:
            mutex.acquire(1)
            self.update()
            self.set_yaw(self.yaw + self.turning_angle)
            mutex.release()
            time.sleep(1)
    
    def changing_height(self, mutex):
        while self.height_changing:
            mutex.acquire(1)
            self.update()
            self.change_height(self.delta_height)
            mutex.release()
            time.sleep(0.01)

    def set_yaw(self, desired_yaw):
        self.desired_yaw = desired_yaw
        self.yaw_pid.setpoint = desired_yaw
    
    def set_stride(self, stride):
        self.stride = stride * self.legs[0].y/10.5
    
    def set_turning_angle(self, angle):
        self.turning_angle = angle
    
    def set_delta_height(self, delta_height):
        self.delta_height = delta_height

if __name__ == '__main__':
    # time.sleep(3)
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50
    robot = Robot(pca)
    robot.reset()
    time.sleep(1)
    # robot.move_leg(robot.legs[0], 5, 4)
    # robot.step(2, 2, 0.5, [1, 3], [0, 2])
    robot.update()
    # print(robot.legs[1].kinematics(robot.legs[1].theta1, robot.legs[1].theta2))
    # robot.walk(8)
    # robot.set_yaw(-60)
    # robot.set_stride(3)
    # robot.trot(8)
    robot.change_height(-3)
    ts = np.arange(0, len(robot.yaws)*CONTROL_INTERVAL, CONTROL_INTERVAL)
    plt.plot(ts, robot.yaws, 'r')
    plt.plot(ts, robot.desired_yaws, 'b')
    plt.savefig('yaw.png')
    robot.reset()


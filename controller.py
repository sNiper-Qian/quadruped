from pyPS4Controller.controller import Controller
from robot import Robot
from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA
import threading
import time

class MyController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        i2c = busio.I2C(SCL, SDA)
        pca = PCA9685(i2c)
        pca.frequency = 50
        self.robot = Robot(pca)
        global mutex
        mutex = threading.Lock()

    def on_x_press(self):
        print("Sleep")
        self.robot.sleep()
    
    def on_circle_press(self):
        print("Wakeup")
        self.robot.reset()
    
    def on_R3_up(self, value):
        if value < -10000 and self.robot.stop:
            print("Start", value)
            self.robot.set_stride(3)
            self.robot.stop = False
            global troting_thread 
            troting_thread = threading.Thread(target=self.robot.troting, args=(mutex,))
            troting_thread.start()
            print("End", value)
    
    def on_R3_down(self, value):
        if value > 20000 and self.robot.stop:
            self.robot.set_stride(-3)
            self.robot.stop = False
            global troting_thread 
            troting_thread = threading.Thread(target=self.robot.troting, args=(mutex,))
            troting_thread.start()
    
    def on_R3_y_at_rest(self):
        if not self.robot.stop:
            print("Stop")
            self.robot.stop = True
            troting_thread.join()
            self.robot.callibrate()
            time.sleep(0.3)

    def on_R3_left(self, value):
        print("Turn left")
        if value < -20000 and not self.robot.stop:
            if not self.robot.turning:
                global turning_thread 
                turning_thread = threading.Thread(target=self.robot.turning_direction, args=(mutex,))
                self.robot.set_turning_angle(-30)
                turning_thread.start()
                print("Start turning")
            else:
                self.robot.set_turning_angle(-30)
                print("Change turning angle")
    
    def on_R3_right(self, value):
        print("Turn right")
        if value > 10000 and not self.robot.stop:
            if not self.robot.turning:
                global turning_thread 
                turning_thread = threading.Thread(target=self.robot.turning_direction, args=(mutex,))
                self.robot.set_turning_angle(30)
                turning_thread.start()
                print("Start turning")
            else:
                self.robot.set_turning_angle(30)
                print("Change turning angle")
    
    def on_R3_x_at_rest(self):
        print("Stop turning")
        if self.robot.turning:
            self.robot.turning = False
            turning_thread.join()
            self.robot.set_yaw(self.robot.yaw)
    
    def on_L3_up(self, value):
        if value < -10000 and not self.robot.height_changing:
            self.robot.height_changing = True
            global height_thread 
            height_thread = threading.Thread(target=self.robot.changing_height, args=(mutex,))
            self.robot.set_delta_height(0.1)
            height_thread.start()
            print("Start changing height up")
    
    def on_L3_down(self, value):
        if value > 10000 and not self.robot.height_changing:
            self.robot.height_changing = True
            global height_thread 
            height_thread = threading.Thread(target=self.robot.changing_height, args=(mutex,))
            self.robot.set_delta_height(-0.1)
            height_thread.start()
            print("Start changing height down")
    
    def on_L3_y_at_rest(self):
        if self.robot.height_changing:
            print("Stop changing height")
            self.robot.height_changing = False
            height_thread.join()

if __name__ == "__main__":
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()
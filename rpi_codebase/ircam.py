import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from adafruit_servokit import ServoKit
import time
import busio
import board
import adafruit_amg88xx
import numpy as np

i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c,0x68)
kit = ServoKit(channels=16)
ambient = 25.0
fire = 0

class heatdirPublisher(Node):

    def __init__(self):
        super().__init__('heatdir_publisher')
        self.publisher_ = self.create_publisher(Int8, 'heatdir',10)
        self.launch_subscriber = self.create_subscription(Int8, 'launch', self.launch_callback,10)
        self.move_publisher = self.create_publisher(Int8, 'move',10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.cols = np.zeros(8)
        self.img = np.zeros((8,8))
        self.max_index = -1;
        self.max = 0;
        self.min = 0;
        self.time = time.time()
        self.launch_index = 0
        self.launch_servos = [
            [0,1,2],
            [4,5,6],
            [8,9,10]
        ]

    def launch_callback(self,msg):
        servos = self.launch_servos.pop(0)
        time.sleep(2)
        kit.servo[servos[0]].angle = fire
        time.sleep(2)
        kit.servo[servos[1]].angle = fire
        time.sleep(4)
        kit.servo[servos[2]].angle = fire
        time.sleep(2)
        msg = Int8()
        msg.data = 1
        self.move_publisher.publish(msg)

        


    def timer_callback(self):
        self.max = 0;
        self.min = 0;
        #self.cols = np.zeros(8)
        self.img = np.array(amg.pixels)
        colmax = np.max(self.img, axis=0)
        self.max_index = int(np.argmax(colmax))
        
        
        self.max = np.max(self.img)
        self.min = np.min(self.img)
        if self.max <= ambient:
            self.max_index = -1
        self.get_logger().info('Publishing: "%d"' % self.max_index)
        
        msg = Int8()
        msg.data = self.max_index
        self.publisher_.publish(msg)
        end_time = time.time() - self.time
        self.time = time.time()
        self.get_logger().info('Time: "%f"' % end_time)
        self.get_logger().info('Publishing: "%d"' % msg.data)
        self.get_logger().info('Max temperature:"%f"' % self.max)
        self.get_logger().info('Min temperature:"%f"' % self.min)
        #self.get_logger().info('Col: %d %d %d %d %d %d %d %d' % tuple(self.cols[:8]))
        #print(np.argmax(self.img, axis = ))
        print(self.img)


def main(args=None):

    rclpy.init(args=args)

    heatdir_publisher = heatdirPublisher()

    rclpy.spin(heatdir_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    heatdir_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

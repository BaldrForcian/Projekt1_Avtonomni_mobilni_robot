import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray, Float32MultiArray
import math


class OneMotorTest(Node):

    MIN_SPEED = -100    #poljubno nastavi za testiranje
    MAX_SPEED = 100     #poljubno nastavi za testiranje
    TORQUE_CONST = 0.75 #Navorna konstanta DDSM115 motorja

    Buffer_counter = 0  #stevec za buffer hitrosti
    Set_speed = 0       #Željena hitrost
    Actual_speed = 0    #Dejanska hitrost
    Error_speed = 0     #Razlika med željeno in dejansko hitrostjo
    
    Current = 0
    Torque = 0
    Power = 0

    def __init__(self):        
        super().__init__("one_motor_test")
        self.get_logger().info('Start One motor test node')
        
        #QoS zaradi ddsm115_controller package-a
        qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)

        #Subsrcibers
        self.Cmd_vel_subscriber_ = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.Cmd_vel_callback,
            10
        ) 
        
        self.Rpm_fb_subscriber_ = self.create_subscription(
            Int16MultiArray,
            '/ddsm115/rpm_fb',
            self.Rpm_fb_callback,
            qos_profile = qos
        )

        self.Cur_fb_subscriber_ = self.create_subscription(
            Float32MultiArray,
            '/ddsm115/cur_fb',
            self.Cur_fb_callback,
            qos_profile = qos
        )

        #Publishers
        self.Rpm_publisher_ = self.create_publisher(
            Int16MultiArray,
            '/ddsm115/rpm_cmd',
            qos_profile = qos
        )

        
    def Cmd_vel_callback(self, msg):
        
        Speed = msg.linear.z                                    #ko je teleop_keyboard aktiven pritisni na "t" za + in "b", -
        Speed = self.Clamp(Speed,self.MIN_SPEED,self.MAX_SPEED) #za vecanje drzi q, za manjsanje drzi z(to se navezuje na prejsnji komentar)
        Speed_int = int(Speed)
        self.Set_speed = Speed_int

        rpm_msg = Int16MultiArray()
        rpm_msg.data = [Speed_int]
        self.Rpm_publisher_.publish(rpm_msg)
        #self.get_logger().info('Nastavljeni Obrati: %d' % (Speed_int))


    def Rpm_fb_callback(self, msg):

        self.Actual_speed = msg.data[0]
        self.Error_speed = self.Set_speed - self.Actual_speed

        self.Buffer_counter += 1
        if self.Buffer_counter >= 20:   #dejansko sprejemanje publishov je 100Hz, če izpisujem vsakih 20 je dovolj(5Hz)
            self.get_logger().info('Dejanski Obrati: %d | Nastavljeni Obrati: %d | Razlika: %d | Tok: %f | Navor: %f | Moč: %f' 
                                    % (self.Actual_speed,self.Set_speed,self.Error_speed,self.Current,self.Torque,self.Power))
            self.Buffer_counter = 0


    def Cur_fb_callback(self, msg):
        
        self.Current = msg.data[0]
        self.Torque = self.TORQUE_CONST * self.Current          # T = Kt * I [Nm]
        self.Power = (self.Torque * self.Actual_speed) / 9.549  # P = (T * RPM)/9.549 [W]


    #Clamping function
    def Clamp(self,value,min_val,max_val):
        return min(max_val, max(min_val,value))


def main(args=None):
    rclpy.init(args=args)
    node = OneMotorTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
	main()
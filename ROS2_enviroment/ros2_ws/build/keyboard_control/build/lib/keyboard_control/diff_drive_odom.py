import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from numpy import pi, sin, cos, arctan2
from math import degrees, sqrt
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class DiffDriveOdom(Node):

    R_wheel = 0.05035     #polmer kolesa(cca. 51mm)
    L_wheelbase = 0.245   #medosna razdalja(219mm iz skice)
    dt = 0.01            #perioda osvezevanja odometrije(100Hz)

    RPM_left = 0
    RPM_right = 0
    RPM_left_fb = 0
    RPM_right_fb = 0
    MAX_RPM = 300
    MIN_RPM = -300

    x = 0.0
    y = 0.0
    theta_pred = 0.0
    theta_imu = 0.0
    theta = 0.0
    alpha = 0.02

    Vl = 0.0
    Vr = 0.0


    yaw_rad = 0.0
    yaw_deg = 0.0
    W_imu_z = 0.0


    #inicializacije
    def __init__(self):        
        super().__init__("diff_drive_odom")
        self.get_logger().info('Start differential drive + odom node')
        
        #QoS zaradi ddsm115_controller package-a
        qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)

        #TF
        self.br = TransformBroadcaster(self)
        
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

        self.IMU_fb_subsriber_ = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.IMU_fb_callback,
            10
        )


        #Publishers
        self.Rpm_publisher_ = self.create_publisher(
            Int16MultiArray,
            '/ddsm115/rpm_cmd',
            qos_profile = qos
        )

        self.Odom_publisher_ = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.timer = self.create_timer(self.dt,self.timer_callback)

    #splosne funkcije
    def rpm_to_linear(self,rpm):
        return (2*rpm*pi*self.R_wheel)/60

    def linear_to_rpm(self,v):
        return int((v*60)/(2*pi*self.R_wheel))
    
    def Clamp(self,value,min_val,max_val):
        return min(max_val, max(min_val,value))



    #ROS funkcije
    def Cmd_vel_callback(self, msg):
        
        Vx = msg.linear.x
        Wz = msg.angular.z

        
        Vl = Vx - (Wz * self.L_wheelbase)/2.0       #hitrost levega motorja->glej izpeljavo inverzne kinematike diff drive robota
        Vr = Vx + (Wz * self.L_wheelbase)/2.0       #hitrost desneg motorja->glej izpeljavo inverzne kinematike diff drive robota
        

        self.RPM_left = self.linear_to_rpm(Vl)
        self.RPM_left = self.Clamp(self.RPM_left,self.MIN_RPM,self.MAX_RPM)
        self.RPM_right = self.linear_to_rpm(Vr)
        self.RPM_right = self.Clamp(self.RPM_right,self.MIN_RPM,self.MAX_RPM)

        rpm_msg = Int16MultiArray()
        rpm_msg.data = [-self.RPM_left,self.RPM_right]
        self.Rpm_publisher_.publish(rpm_msg)
        
        
        #self.get_logger().info('Nastavljeni Obrati: Levi: %d | Desni: %d' % (self.RPM_left,self.RPM_right))



    def Rpm_fb_callback(self, msg):

        self.RPM_left_fb = -msg.data[0]
        self.RPM_right_fb = msg.data[1]        #minus ker sta motorja zrcaljeno montirana

        #self.get_logger().info('Dejanski Obrati: Levi: %d | Desni: %d' % (self.RPM_left_fb,self.RPM_right_fb))

    def IMU_fb_callback(self, msg):
        
        quat = msg.orientation
        self.W_imu_z = msg.angular_velocity.z
        #angular_vel = msg.angular_velocity
        #linear_vel = msg.linear_acceleration

        self.yaw_rad = arctan2(2*(quat.w * quat.z + quat.x * quat.y), 1 - 2*(quat.y*quat.y + quat.z*quat.z))
        self.yaw_deg = degrees(self.yaw_rad)
        #self.get_logger().info('Yaw angle: %f' % self.yaw_deg)




    
    #ODOM loop
    def timer_callback(self):

        
        Vl_fb = round(self.rpm_to_linear(self.RPM_left_fb), 3)
        Vr_fb = round(self.rpm_to_linear(self.RPM_right_fb), 3)

        V_odom = (Vl_fb + Vr_fb) / 2

     #Use IMU angular velocity for W_odom if available, else encoders
        #if self.W_imu_z is not None:
        #W_odom = self.W_imu_z
        #else:
        W_odom = -(Vr_fb - Vl_fb) / self.L_wheelbase

    # Use IMU absolute heading for theta if available, else integrate
        #if self.yaw_rad is not None:
        theta = self.yaw_rad        # use IMU absolute heading for position update
        self.theta = self.yaw_rad   # keep self.theta in sync
        #else:
        #self.theta += W_odom * self.dt
        #theta = self.theta

    #    self.get_logger().info(
    #        f"Vl={Vl_fb} Vr={Vr_fb} V={V_odom:.3f} W={W_odom:.3f} "
    #        f"theta={theta:.3f} yaw={self.yaw_rad} "
    #        f"diff={abs(Vr_fb - Vl_fb):.4f} "
    #        f"x={self.x:.3f} y={self.y:.3f}"
    #    )

        straight_threshold = max(0.05 * abs(V_odom), 0.01)

    # Position update using consistent theta
        if abs(Vr_fb - Vl_fb) < straight_threshold:
            self.x += V_odom * cos(theta) * self.dt
            self.y += V_odom * sin(theta) * self.dt
        else:
            R_ICC = (self.L_wheelbase / 2) * ((Vl_fb + Vr_fb) / (Vr_fb - Vl_fb))
            dtheta = W_odom * self.dt
            self.x += R_ICC * (sin(theta + dtheta) - sin(theta))
            self.y += R_ICC * (-cos(theta + dtheta) + cos(theta))
        #self.x += V_odom * cos(theta) * self.dt
        #self.y += V_odom * sin(theta) * self.dt

        #self.get_logger().info(
            #f"Vl={Vl_fb} Vr={Vr_fb} V={V_odom:.3f} theta={theta:.4f} "
            #f"yaw={self.yaw_rad:.4f} x={self.x:.3f} y={self.y:.3f}"
        #)
        

        q = quaternion_from_euler(0,0, self.theta)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"	
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.pose.covariance[0] = 0.0001
        odom_msg.pose.covariance[7] = 0.0001
        odom_msg.pose.covariance[14] = 0.000001	#1e12
        odom_msg.pose.covariance[21] = 0.000001	#1e12
        odom_msg.pose.covariance[28] = 0.000001	#1e12
        odom_msg.pose.covariance[35] = 0.0001
        odom_msg.twist.twist.linear.x = V_odom 
        odom_msg.twist.twist.linear.y = 0.0 
        odom_msg.twist.twist.angular.z = W_odom 
        self.Odom_publisher_.publish(odom_msg)

		
		# construct tf
        t = TransformStamped()
        t.header.frame_id = "odom" 
        t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = "base_link"	
        t.transform.translation.x = self.x 
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = odom_msg.pose.pose.orientation.x
        t.transform.rotation.y = odom_msg.pose.pose.orientation.y
        t.transform.rotation.z = odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = odom_msg.pose.pose.orientation.w
        self.br.sendTransform(t)                           

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
	main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
#import serial.rs485
import serial
import crcmod
import time
import struct
import math
from numpy import pi

# Registri (navajamo samo prvi, ker pri modbus z enim serial sporocilom dostopamo do treh #
#           npr. lahko rečemo kliči 0x34 in 3 mesta -> dobimo nazaj vrednosti 0x34,0x35,0x36

REG_AX = 0x34       #Pospešek v X
REG_GX = 0x37       #Kotna hitrost v X
REG_HX = 0x3A       #Magnetno polje v X
REF_ROLL = 0x3D     #Roll
REG_TEMP = 0x40     #Temperatura
REG_Q0 = 0x51       #Quaterninon 0
REG_KEY = 0x69      #Kljuc za odklepanje
REG_CALSW = 0x01    #Kalibracija
REG_SAVE = 0x00     #Save/Reboot/reset
REG_AXIS6 = 0x24    #Sprememba načina iz 9 axis v 6 axis
REG_BW = 0x1F       #Natavitve pasovne širine(5-256Hz)->default 20hz
REG_BAUD = 0x04     #Nastavitev baud rate-a serijske komunikacije 
REG_MODD = 0x74     #Nastavitve zakasnitve med pošiljanjem

# Konstante #

UNLOCK_KEY = 0xB588 #byte koda za odklep nastavitev
READ_FC = 0x03      #funkcijska koda za branje
WRITE_FC = 0x06     #funkcijska koda za pisanje
BW_42HZ = 0x03      #funkcijska koda za 42Hz
BW_98HZ = 0x02      #funkcijska koda za 98Hz
BW_188HZ = 0x01     #funkcijska koda za 188z
AXIS_9 = 0x00       #funkcijska koda za 9 osni način
AXIS_6 = 0x01       #funkcijska koda za 6 osni način
ZERO_YAW = 0x04     #funkcijska koda za ponastavljanje yaw na 0
SAVE_SAVE = 0x00    #funkcijska koda za shranjevanje kalibracije
BAUD_9600 = 0x02    #funkcijska koda za baud 9600bps
BAUD_115200 = 0x06   #funkcijska koda za baud 115200bps
MODDELAY_1000 = 0x03E8
MODDELAY_500 = 0x01F4

class WT901CNode(Node):

    def __init__(self):
        super().__init__('wt901c_RS485_imu_node')
        
        self.declare_parameter('device', '/dev/ttySC1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('modbus_addr', 0x50)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 200.0)
        self.declare_parameter('use_9axis', True)
        self.declare_parameter('bandwidth', BW_188HZ)

        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.addr = self.get_parameter('modbus_addr').get_parameter_value().integer_value
        self.frame_id =  self.get_parameter('frame_id').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.use_9axis = self.get_parameter('use_9axis').get_parameter_value().bool_value
        self.bandwidth = self.get_parameter('bandwidth').get_parameter_value().integer_value

        try:
            #self.ser = serial.rs485.RS485(self.device, self.baud, timeout=0.01)
            #self.ser.rs485_mode = serial.rs485.RS485Settings()
            self.ser = serial.Serial(
            port=self.device,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.01
            )
            self.get_logger().info("RS485 connection established")
        except serial.SerialException as e:
            self.get_logger().info(f"RS485 connection failed: {e}")
            raise

        #crcmod funkcija za modbus protokol
        self.crc16_func = crcmod.predefined.mkCrcFun('modbus')

        #Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.temp_pub = self.create_publisher(Temperature, '/imu/temp', 10)

        #Konfiguracija
        self.startup_config()

        #Timer
        self.timer = self.create_timer(1.0/self.rate_hz, self.timer_callback)


#-----------------Metode za ustvarjanja RS485 komande-----------------------------------------------------#
    def crc16(self, data: bytes) -> int:
        return self.crc16_func(data)
    
    def build_read_cmd(self, reg_addr: int, reg_count: int) -> bytes:
        
        cmd = bytes([
            self.addr,
            READ_FC,
            (reg_addr >> 8) & 0xFF,
            reg_addr & 0xFF,
            (reg_count >> 8) & 0xFF,
            reg_count & 0xFF
        ])
        
        crc = self.crc16(cmd)
        return cmd + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    
    def build_write_cmd(self, reg_addr: int, reg_data: int) -> bytes:
        
        cmd = bytes([
            self.addr,
            WRITE_FC,
            (reg_addr >> 8) & 0xFF,
            reg_addr & 0xFF,
            (reg_data >> 8) & 0xFF,
            reg_data & 0xFF
        ])
        crc = self.crc16(cmd)
        return cmd + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    
    def parse_int16(self, high: int, low: int) -> int:
        
        val = (high << 8) | low
        if val > 32767:
            val -= 65536
        return val

#-----------------Registerske metode-------------------------------------------------------------------------------#
    def send_read(self, reg_addr: int, reg_count: int) -> bytes | None:
        
        cmd = self.build_read_cmd(reg_addr, reg_count)
        expected_len = 3 + reg_count * 2 + 2    #addr + FC + len + data*2 + crc

        for attempt in range(3):
            self.ser.reset_input_buffer()
            self.ser.write(cmd)
            self.ser.flush()


            response = self.ser.read(expected_len)
            
            if len(response) != expected_len:
                continue
            
            crc_recv = (response[-1] << 8) | response[-2]   #zadnji bit premaknemo v levo za 8 bitov in izvedemo log ali z prezadnjim bitom
            crc_calc = self.crc16(response[:-2])            #glede na pridobljene podatke izracunamo crc brez zadnjih dveh byteov
            if crc_recv != crc_calc:                        #ce se crc ne ujema error
                #self.get_logger().warn('CRC mismatch', throttle_duration_sec=1.0)
                self.get_logger().warn('CRC mismatch',throttle_duration_sec=1.0)
                continue
        
            return response[3:-2]                           #vrnemo samo data byte
            
        self.get_logger().warn('Failed after 3 attempts', throttle_duration_sec=1.0)
        return None

        
    def read_registers(self, reg_addr: int, reg_count: int) -> list[int] | None:

        raw = self.send_read(reg_addr, reg_count)
        if raw is None or len(raw) < reg_count * 2:
            return None
        
        values = []
        for i in range(reg_count):
            val = self.parse_int16(raw[i*2], raw[i*2 + 1])
            values.append(val)
            
        return values
        
    def write_register(self, reg_addr: int, reg_data: int) -> bool:
        
        cmd = self.build_write_cmd( reg_addr, reg_data)
        self.ser.reset_input_buffer()
        self.ser.write(cmd)
        
        response = self.ser.read(8)
        if response != cmd:
            self.get_logger().warn('Command mismatch',throttle_duration_sec=1.0)
            return False
        
        return True

#----------------konfiguracijske metode---------------------------------------------------------------------#
    def unlock(self):
        if not self.write_register(REG_KEY, UNLOCK_KEY):
            self.get_logger().error('Failed to unlock')

    def set_bandwidth(self):
        if not self.write_register(REG_BW, self.bandwidth):
            self.get_logger().error('Failed to set bandwidth')
        else:
            self.get_logger().info('Set desired bandwidth')

    def set_axis(self):
        if not self.write_register(REG_AXIS6, 
                                   AXIS_9 if self.use_9axis else AXIS_6):
            self.get_logger().error('Failed to set axis mode')
        else:
            if self.use_9axis: 
                self.get_logger().info('Set to 9 axis mode')
            else:
                self.get_logger().info('Set to 6 axis mode')

    def zero_yaw(self):
        if not self.write_register(REG_CALSW, ZERO_YAW):
            self.get_logger().error('Failed to zero yaw')
        else:
            self.get_logger().info('Yaw set to 0')

    def save(self):                                         #problem z mismatchom verjetno zaradi shranjevanje v flash
        if not self.write_register(REG_SAVE, SAVE_SAVE):
            self.get_logger().error('Failed to save')
        else:
            self.get_logger().info('Saved configuration')

    def set_baud(self):
        if not self.write_register(REG_BAUD, BAUD_115200):
            self.get_logger().error('Failed to set baud')
        else:
            self.get_logger().info('Baud set')

    def set_moddelay(self):
        if not self.write_register(REG_MODD, MODDELAY_500):
            self.get_logger().error('Failed to set moddelay')
        else:
            self.get_logger().info('Moddelay set')

    def startup_config(self):
        
        self.get_logger().info('Startup configuration and calibration(dont move the imu)')
        
        #self.unlock()           
        #time.sleep(0.1)
        #self.set_baud()
        #time.sleep(0.1)
        #self.unlock()
        #time.sleep(0.1)
        #self.save()
        #time.sleep(0.5)

        #self.ser.close()
        #time.sleep(0.5)
        #self.ser = serial.rs485.RS485(self.device, 115200, timeout=0.1)
        #self.ser.rs485_mode = serial.rs485.RS485Settings()
        #self.get_logger().info('Switched to 115200 baud')
        
        self.unlock()           
        time.sleep(0.1)
        self.set_moddelay()
        time.sleep(0.1)

        self.unlock()           
        time.sleep(0.1)
        self.set_axis()
        time.sleep(0.1)

        self.unlock()
        time.sleep(0.1)
        self.set_bandwidth()
        time.sleep(0.1)

        self.unlock()
        time.sleep(0.1)
        self.zero_yaw()
        time.sleep(0.1)

        #konfiguracije ni potrebno shranit saj se ob vsakem zagonu programa skalibrira
        #self.unlock()
        #time.sleep(0.1)
        #self.save()
        #time.sleep(0.1)


#-------------Branje vrednosti IMU in Publishing vrednosti------------------------------#
    def timer_callback(self):
        
        now = self.get_clock().now().to_msg()
        G = 9.8

        #Preberi Blok podatkov(Ax,Ay,Az,Gx,Gy,Gz,Hx,Hy,Hz)
        block = self.read_registers(REG_AX, 9)
        if block is None:
            self.get_logger().warn('Failed to read data block')
            return
        
        #Preberi quaternions
        quat = self.read_registers(REG_Q0, 4)
        if quat is None:
            self.get_logger().warn('Failed to read quarternions')
            return
        
        #Preberi temperaturo
        #temp = self.read_registers(REG_TEMP, 1)
        #if temp is None:
        #    self.get_logger().warn('Failed to read temperature')
        #    return

        #pospeški
        ax = (block[0] / 32768.0) * 16.0 * G
        ay = (block[1] / 32768.0) * 16.0 * G
        az = (block[2] / 32768.0) * 16.0 * G

        #kotne hitrosti
        gx = (block[3] / 32768.0) * 2000.0
        gy = (block[4] / 32768.0) * 2000.0
        gz = (block[5] / 32768.0) * 2000.0

        #quaternioni
        q0 = quat[0] / 32768.0
        q1 = quat[1] / 32768.0
        q2 = quat[2] / 32768.0
        q3 = quat[3] / 32768.0

        #Publish IMU
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id

        imu_msg.orientation.w = q0
        imu_msg.orientation.x = q1
        imu_msg.orientation.y = q2
        imu_msg.orientation.z = q3

        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        #Covariance
        # orientation - from calibrated config
        imu_msg.orientation_covariance[0] = 0.0479   # x
        imu_msg.orientation_covariance[4] = 0.0207   # y
        imu_msg.orientation_covariance[8] = 0.0041   # z 

        # angular velocity - from calibrated config
        imu_msg.angular_velocity_covariance[0] = 0.0663   # x
        imu_msg.angular_velocity_covariance[4] = 0.1453   # y
        imu_msg.angular_velocity_covariance[8] = 0.0378   # z

        # linear acceleration - from calibrated config
        imu_msg.linear_acceleration_covariance[0] = 0.0364   # x
        imu_msg.linear_acceleration_covariance[4] = 0.0048   # y
        imu_msg.linear_acceleration_covariance[8] = 0.0796   # z

        #imu_msg.angular_velocity_covariance[0] = -1.0
        #imu_msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)

        #Publish magnetometer
        mag_msg = MagneticField()
        mag_msg.header.stamp = now
        mag_msg.header.frame_id = self.frame_id

        mag_msg.magnetic_field.x = block[6] * 1e-8
        mag_msg.magnetic_field.y = block[7] * 1e-8
        mag_msg.magnetic_field.z = block[8] * 1e-8

        self.mag_pub.publish(mag_msg)



    
def main(args=None):
    rclpy.init(args=args)
    node = WT901CNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

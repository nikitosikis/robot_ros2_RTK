import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

import serial
import time

# Отправка и чтение команд/данных с устройства по протоколу

# Протокол следующий:
# Открываюший байт
# Число байт данных
# Байты данных
# Закрывающий байт

# Внутри байтов данных (передача на устройство):
# байт 1 - Индекс устройства (вообще определяет, с чем мы имеем дело, рапределение команд)
# байт 2 - Байт команды
# байты 3-N - данные команды

# CONSTANTS
CONTROLLER=1
#controlled commands
PING=0

PWM = 2
#PWM commands
CMD_SET_PWM_STOP = 1
CMD_SET_PWM_FW = 2
CMD_SET_PWM_BW = 3
CMD_SET_PWM_L = 4
CMD_SET_PWM_R = 5
CMD_SET_PWM_HIGH = 6

directions = ["S", 'FW', 'BW', 'L', 'R', 'H']

def ping_command():
    print("A")
    data=[]
    data.append(CONTROLLER)
    data.append(PING)
    print(data)
    return data

#direction - one of directions
#speed = 0..254
def set_robot_speed_direction(speed, direction):
    data=[]
    if speed<0 or speed>100:
        print(f"Cannot set speed value: {speed}")
        return
    if direction not in directions:
        print(f"Cannot set direction value: {direction}")
        return
    
    dcmd = CMD_SET_PWM_STOP
    if direction == 'S':
        dcmd = CMD_SET_PWM_STOP
    elif direction == 'FW':
        dcmd = CMD_SET_PWM_FW
    elif direction == 'BW':
        dcmd = CMD_SET_PWM_BW
    elif direction == 'L':
        dcmd = CMD_SET_PWM_L
    elif direction == 'R':
        dcmd = CMD_SET_PWM_R
    elif direction == 'H':
        dcmd = CMD_SET_PWM_HIGH

    data.append(PWM)
    data.append(dcmd)
    data.append(int(speed))
    return data


'''
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
'''
class TwistSubscriber(Node):
    def __init__(self):
        super().__init__('kursant1_driver')
        #self.device_name='/dev/AMC0'
        self.declare_parameter('device_name', '/dev/AMC0')
        self.device_name = self.get_parameter('device_name').value
        #self.linear_multiplier = 100
        self.declare_parameter('linear_multiplier', 100)
        self.linear_multiplier = self.get_parameter('linear_multiplier').value
        #self.angular_multiplier = 50
        self.declare_parameter('angular_multiplier', 50)
        self.angular_multiplier = self.get_parameter('angular_multiplier').value
        #self.debug_mode = True
        self.declare_parameter('debug_mode', True)
        self.debug_mode = self.get_parameter('debug_mode').value
        
        self.get_logger().info(f'Init: device: {self.device_name}, lm: {self.linear_multiplier}, am: {self.angular_multiplier}, debug: {self.debug_mode}')

        # subscription to security warning
        self.warning_subscription = self.create_subscription(Bool, '/security/warning', self.warning_callback, 10)
        
        self.publisher_ = self.create_publisher(String, 'status', 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)

        # security warning flag
        self.warning_active = False
        
        if not self.debug_mode:
          # Configure the serial port
          self._ser = serial.Serial(
              port= self.device_name,      # Change this to your port (e.g., '/dev/ttyUSB0' on Linux)
              baudrate=9600,    # Set the baud rate to match your device
              timeout=0.1         # Set a read timeout in seconds
          )
          if not self._ser.is_open:
          	self._ser.open()
          time.sleep(2)
          self.get_logger().info(f'Driver initialized!')


    def warning_callback(self, msg):
        """Security warning handling"""
        if msg.data:
            if not self.warning_active:
                self.warning_active = True
                self.get_logger().error('ОПАСНОСТЬ: препятствие близко! Останавливаю робота.')
                self.emergency_stop()
        else:
            if self.warning_active:
                self.warning_active = False
                self.get_logger().info('Опасности движения нет.')


    def emergency_stop(self):
        """Emergency robot stop"""
        if self.debug_mode:
            self.get_logger().warning('Debug mode: EMERGENCY STOP command')
        else:
            # Send stop command to the robot
            cmd = set_robot_speed_direction(0, 'S')
            if cmd:
                self.send_packet(cmd)
                self.get_logger().warning('Отправлена команда экстренной остановки')



    def send_packet(self, data):
        #global ser
        #print(len(data))
        packet = [
            b'#',                        # Стартовый байт
            bytes([len(data)]),          # Длина данных
            bytes(data),                # Данные
            b'.'                        # Стоповый байт
        ]

        #print(packet)

        #print(packet)
        #return
        for part in packet:
            self._ser.write(part) 

    def process_command(self, command):
        if len(command)==0:
            print("Empty command!") #TODO: replace with debug.log?
            return [], ""
        self.send_packet(command)
        
        # Wait a short time for the device to process (adjust as needed)
        time.sleep(0.01)
        
        # Read the response
        response = self._ser.read_until()  # Reads until timeout or newline
        # Alternative: response = ser.read_all()  # Reads all available bytes
        
        response_str = response.decode('utf-8', errors='replace').strip()

        print(len(response))
        
        print(f"Received: {response_str}")
        return response, response_str

    def twist_callback(self, msg):

        x = msg.linear.x
        rot = msg.angular.z
        
        self.get_logger().info(f'Received cmd_vel linear {x} angular {rot}')
        
        direction='S'
        speed = 0
        
        if x>0.0001 and not self.warning_active:
            direction = 'FW'
            speed = int(abs(x)*self.linear_multiplier)
            
            if speed>=100:
                speed = 100
            if speed<=0:
                speed = 0
        elif x>0.0001 and self.warning_active:
            # checking if the alert is active
            self.get_logger().warning('Предупрежедние активно, игнорирую команду движения вперед')
            if not self.debug_mode:
                cmd = set_robot_speed_direction(0, 'S')
                if cmd:
                    self.send_packet(cmd)
            return
        
        if x<-0.0001:
            direction = 'BW'
            speed = int(abs(x)*self.linear_multiplier)
            
            if speed>=100:
                speed = 100
            if speed<=0:
                speed = 0
                
        elif abs(x)<0.0001:
            if rot>0.0001:
                direction = 'L'
                speed = int(abs(rot)*self.angular_multiplier)
                
                if speed>=100:
                    speed = 100
                if speed<=0:
                    speed = 0
            elif rot<-0.0001:
                direction = 'R'
                speed = int(abs(rot)*self.angular_multiplier)
                
                if speed>=100:
                    speed = 100
                if speed<=0:
                    speed = 0
            elif abs(rot)<0.0001:
                direction = 'S'
                speed = 0

        s=""
        if not self.debug_mode:
          if not self._ser.is_open:
            self._ser.open()
          cmd=set_robot_speed_direction(speed, direction)
          response, response_str = self.process_command(cmd)
          s=response_str
        else:
          s = f'Linear: {msg.linear.x}, Angular: {msg.angular.z}'
        
        self.get_logger().info(f'{s}')
        
        status_msg = String()
        status_msg.data = s
        self.publisher_.publish(status_msg)

    def __del__(self):
      if hasattr(self, "_ser") and self._ser:
        self._ser.close()   





def main(args=None):
    rclpy.init(args=args)

    twist_subscriber = TwistSubscriber()

    rclpy.spin(twist_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    twist_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

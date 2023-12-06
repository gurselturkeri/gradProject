import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class ArduinoToRos(Node):
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        super().__init__('motor_data') 
        self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        self.motorDataPublisher = self.create_publisher(Int64, "motorDataTopic", 10)
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timerCallback)
        self.value = 0 
        self.arduinoData = 0

    def write_read(self, pos):
        self.arduino.write(bytes(pos, 'utf-8'))
        time.sleep(0.05)
        self.arduinoData = self.arduino.readline()
        return self.arduinoData.decode('utf-8')

    def timerCallback(self):
        num = input("Enter desired motor position: ")
        value = self.write_read(num)
        print("current position from Arduino: ", value)
        self.msg = Int64()
        self.msg.data = int(self.arduinoData)
        self.motorDataPublisher.publish(self.msg)
        
    def close(self):
        self.arduino.close()

def main(args=None):
    
    try:
        rclpy.init(args=args)
        arduino_comm = ArduinoToRos()
        rclpy.spin(arduino_comm)
            
    except Exception as e:
        print("An error occurred:", e)
    
    
    arduino_comm.close()
    arduino_comm.destroy_node()
    

if __name__ == "__main__":
    main()

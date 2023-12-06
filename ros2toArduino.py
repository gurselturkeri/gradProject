import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class ArduinoToRos(Node):
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        super().__init__('motor_data') 
        self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        self.currentPose = self.create_publisher(Int64, "currentPose", 10)
        self.desiredPose = self.create_publisher(Int64, "desiredPose", 10)

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timerCallback)
        self.value = 0 
        self.arduinoData = 0

    def write_read(self, pos):
        pos_str = str(pos)  # Convert the integer to a string
        self.arduino.write(pos_str.encode('utf-8'))  # Encode the string as bytes
        time.sleep(0.05)
        self.arduinoData = self.arduino.readline()
        return self.arduinoData.decode('utf-8')

    def timerCallback(self):
        num = int(input("Enter desired motor position: "))
        value = self.write_read(num)
        print("current position from Arduino: ", value)

        if self.arduinoData:
            try:
                arduino_data_int = int(self.arduinoData)
                self.current_msg = Int64()
                self.current_msg.data = arduino_data_int
                self.currentPose.publish(self.current_msg)
                self.desired_msg = Int64()
                self.desired_msg.data = arduino_data_int
                self.desiredPose.publish(self.desired_msg)
            except ValueError:
                print("Invalid data received from Arduino:", self.arduinoData)
        else:
            print("No data received from Arduino")
        
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

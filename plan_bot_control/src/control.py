#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
import serial
import struct
from sensor_msgs.msg import Range
import serial


struct_format = 'iiiiii'
min_distance = 0.2
prev_button = 0
latch = 0
count = 0

Joystick_min_upper = 520
Joystick_max_upper = 1023
speed_min_upper = 0.1
speed_max_upper = 0.6
Joystick_range_upper = Joystick_max_upper - Joystick_min_upper
speed_range_upper = speed_max_upper - speed_min_upper


Joystick_min_lower = 500
Joystick_max_lower = 0
speed_min_lower = -0.1
speed_max_lower = -0.6
Joystick_range_lower = Joystick_max_lower - Joystick_min_lower
speed_range_lower = speed_max_lower - speed_min_lower

Joystick2_min_upper = 520
Joystick2_max_upper = 1023
speed2_min_upper = 0.1
speed2_max_upper = 0.6
Joystick2_range_upper = Joystick2_max_upper - Joystick2_min_upper
speed2_range_upper = speed2_max_upper - speed2_min_upper

Joystick2_min_lower = 500
Joystick2_max_lower = 0
speed2_min_lower = -0.1
speed2_max_lower = -0.6
Joystick2_range_lower = Joystick2_max_lower - Joystick2_min_lower
speed2_range_lower = speed2_max_lower - speed2_min_lower

class SerialReader(Node):
    def __init__(self):
        super().__init__('controlling_turtle')


        self.publisher_distance = self.create_publisher(Float32,'S1_ultrasonic_distance',10)
        self.publisher_button = self.create_publisher(Int32,'S2_button_stop',10)
        self.publisher_joystick1_x = self.create_publisher(Int32,'S3_joystick1_xaxis',10)
        self.publisher_joystick1_y = self.create_publisher(Int32,'S3_joystick1_yaxis',10)
        self.publisher_joystick2_x = self.create_publisher(Int32,'S4_joystick2_xaxis',10)
        self.publisher_joystick2_y = self.create_publisher(Int32,'S4_joystick2_yaxis',10)
        self.publisher_cmdvel = self.create_publisher(Twist,'cmd_vel',10)
        self.publisher_ultrasonic = self.create_publisher(Range,'ultrasonic',10)
        
        

        self.ser = serial.Serial("/dev/ttyUSB0", 115200)
        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        global latch,prev_button,count

        if self.ser.in_waiting >= struct.calcsize(struct_format):
            size = struct.calcsize(struct_format)
            line = self.ser.read(size)
            
            # if line != 'S':
            distance,button,joystick1_x,joystick1_y,joystick2_x,joystick2_y = struct.unpack(struct_format, line)

            
            msg_distance = Float32()

            #########################################ultrasonic######################33

            distance_ultrasonic = Range()
 
            distance_ultrasonic.header.stamp = self.get_clock().now().to_msg()
            distance_ultrasonic.header.frame_id = 'map'
            distance_ultrasonic.radiation_type = Range.ULTRASOUND
            distance_ultrasonic.field_of_view = 0.2618
            distance_ultrasonic.min_range = 0.0
            distance_ultrasonic.max_range = 4.0
            distance_ultrasonic.range = float(distance)/ 100.0
            
            msg_button = Int32()
            msg_joystick1_x = Int32()  
            msg_joystick1_y = Int32()  
            msg_joystick2_x = Int32()  
            msg_joystick2_y = Int32()  
            move = Twist()
           
            msg_distance.data = float(distance) /100
            msg_button.data = int(button)
            msg_joystick1_x.data = int(joystick1_x)
            msg_joystick1_y.data = int(joystick1_y)
            msg_joystick2_x.data = int(joystick2_x)
            msg_joystick2_y.data = int(joystick2_y)

            ##################################### publishing #############################################
            self.publisher_joystick1_x.publish(msg_joystick1_x)
            self.publisher_joystick1_y.publish(msg_joystick1_y)
            self.publisher_joystick2_x.publish(msg_joystick2_x)
            self.publisher_joystick2_y.publish(msg_joystick2_y)
            self.publisher_distance.publish(msg_distance)
            self.publisher_button.publish(msg_button)

            
            # self.get_logger().info(f'Published Button: {msg_button.data}')
            if msg_button.data != prev_button :

                if latch == 0 and count == 0:
                    move.linear.x = 0.0
                    move.linear.z = 0.0
                    move.angular.z = 0.0
                    latch = 1
                    
                    print("Bot on breaks")

                count = count + 1

                if latch == 1 and count == 3:
                    move.linear.x = 0.0
                    move.linear.z = 0.0
                    move.angular.z = 0.0
                    latch = 0
                    count = 0
                    print("Removed from breaks")

            elif msg_distance.data <= min_distance and latch == 0:
                move.linear.x = 0.0
                move.linear.z = 0.0
                self.get_logger().info('Change Direction Obstacle ahead .. ')

                if (520 <= joystick2_y <= 1023):
                    move.angular.z = -((((joystick2_y - Joystick2_min_upper)*speed2_range_upper)/Joystick2_range_upper) + speed2_min_upper)
                    
                    print(" Taking right")

                if (0 <= joystick2_y <= 500):
                    
                    move.angular.z = abs((((joystick2_y - Joystick2_min_lower)*speed2_range_lower)/Joystick2_range_lower) + speed2_min_lower) 
                    # move.angular.z = 0.4
                    print(" Taking left")

            elif latch == 0:
                self.get_logger().info(f'Received from Arduino: {line}')
                self.get_logger().info(f'Published Button: {msg_button.data}')
                self.get_logger().info(f'Published Ultrasonic: {msg_distance.data}')
                self.get_logger().info(f'Published Joystick 1 x axis: {msg_joystick1_x.data}')
                self.get_logger().info(f'Published Joystick 1 y axis: {msg_joystick1_y.data}')
                self.get_logger().info(f'Published Joystick 2 x axis: {msg_joystick2_x.data}')
                self.get_logger().info(f'Published Joystick 2 y axis: {msg_joystick2_y.data}')

                if (520 <= joystick1_x <= 1023):
                    move.linear.x = (((joystick1_x - Joystick_min_upper)*speed_range_upper)/Joystick_range_upper) + speed_min_upper
                    move.linear.z = (((joystick1_x - Joystick_min_upper)*speed_range_upper)/Joystick_range_upper) + speed_min_upper
                    print(" Taking forward")

                if (0 <= joystick1_x <= 500):
                    move.linear.x = (((joystick1_x - Joystick_min_lower)*speed_range_lower)/Joystick_range_lower) + speed_min_lower
                    move.linear.z = (((joystick1_x - Joystick_min_lower)*speed_range_lower)/Joystick_range_lower) + speed_min_lower
                    print(" Taking back")
                

                if (520 <= joystick2_y <= 1023):
                    move.angular.z = -((((joystick2_y - Joystick2_min_upper)*speed2_range_upper)/Joystick2_range_upper) + speed2_min_upper)
                    
                    print(" Taking right")

                if (0 <= joystick2_y <= 500):
                    
                    move.angular.z = abs((((joystick2_y - Joystick2_min_lower)*speed2_range_lower)/Joystick2_range_lower) + speed2_min_lower) 
                    # move.angular.z = 0.4
                    print(" Taking left")
            
            if move.linear.x == 0.0 and move.linear.z == 0.0 and move.angular.z == 0.0:
                self.ser.write(0)
                # self.get_logger().info(f'Inside S')
            else:
                self.ser.write(1)
                # self.get_logger().info(f'Inside R')
            prev_button = msg_button.data

            self.publisher_cmdvel.publish(move)
            self.publisher_ultrasonic.publish(distance_ultrasonic)
            

def main(args=None):
    rclpy.init(args=args)
    serial_reader = SerialReader()
    rclpy.spin(serial_reader)

    serial_reader.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
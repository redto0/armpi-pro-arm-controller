#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import SetBool
from std_msgs.msg import UInt16
from bus_servo_controller.msg import MultiServoCommand, ServoCommand, ServoStatus
from BusServoControl import *

class BusServoController(Node):
    def __init__(self):
        super().__init__('bus_servo_controller')
        
        # Declare parameters with defaults
        self.declare_parameters(
            namespace='',
            parameters=[
                ('servo_ids', [1]),
                ('default_duration_ms', 1000),
                ('publish_rate_hz', 10.0),
                ('auto_detect_servos', False)
            ]
        )
        
        # Get parameters
        self.servo_ids = self.get_parameter('servo_ids').value
        self.default_duration = self.get_parameter('default_duration_ms').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        
        # Service callback group (to allow parallel service execution)
        self.cb_group = ReentrantCallbackGroup()
        
        # Publishers
        self.status_pub = self.create_publisher(
            ServoStatus, 
            'servo_status', 
            10)
        
        # Subscribers
        self.multi_cmd_sub = self.create_subscription(
            MultiServoCommand,
            'multi_servo_command',
            self.multi_command_callback,
            10)
            
        self.single_cmd_sub = self.create_subscription(
            ServoCommand,
            'servo_command',
            self.single_command_callback,
            10)
            
        # Services
        self.enable_srv = self.create_service(
            SetBool,
            'enable_servos',
            self.enable_servos_callback,
            callback_group=self.cb_group)
        
        # Timer for status updates
        self.timer = self.create_timer(
            1.0/self.publish_rate, 
            self.publish_servo_status)
        
        self.get_logger().info(f"Bus Servo Controller initialized with servo IDs: {self.servo_ids}")
    
    def multi_command_callback(self, msg):
        """Handle commands for multiple servos"""
        if len(msg.ids) != len(msg.positions) or len(msg.ids) != len(msg.durations):
            self.get_logger().error("Mismatched array lengths in MultiServoCommand")
            return
            
        for i in range(len(msg.ids)):
            try:
                setBusServoPulse(msg.ids[i], msg.positions[i], msg.durations[i])
            except Exception as e:
                self.get_logger().error(f"Failed to move servo {msg.ids[i]}: {str(e)}")
    
    def single_command_callback(self, msg):
        """Handle command for a single servo"""
        try:
            setBusServoPulse(msg.id, msg.position, msg.duration)
        except Exception as e:
            self.get_logger().error(f"Failed to move servo {msg.id}: {str(e)}")
    
    def publish_servo_status(self):
        """Publish status for all servos"""
        for servo_id in self.servo_ids:
            try:
                status = ServoStatus()
                status.id = servo_id
                status.position = getBusServoPulse(servo_id)
                status.voltage = getBusServoVin(servo_id)
                status.temperature = getBusServoTemp(servo_id)
                self.status_pub.publish(status)
            except Exception as e:
                self.get_logger().error(f"Failed to read status for servo {servo_id}: {str(e)}")
    
    def enable_servos_callback(self, request, response):
        """Enable/disable servos (unload power)"""
        try:
            for servo_id in self.servo_ids:
                if request.data:
                    # Enable servo (load)
                    serial_serro_wirte_cmd(servo_id, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 1)
                else:
                    # Disable servo (unload)
                    unloadBusServo(servo_id)
            
            response.success = True
            response.message = "Success" if request.data else "Servos unloaded"
        except Exception as e:
            response.success = False
            response.message = str(e)
        
        return response

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Use multi-threaded executor for parallel service callbacks
        executor = MultiThreadedExecutor()
        servo_controller = BusServoController()
        executor.add_node(servo_controller)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            servo_controller.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

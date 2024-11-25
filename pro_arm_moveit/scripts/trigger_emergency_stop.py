#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration
import sys

class EmergencyStopClient(Node):
    def __init__(self):
        super().__init__('emergency_stop_client')
        self.switch_controller = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )

    def send_emergency_stop(self, activate=True):
        if not self.switch_controller.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Switch controller service not available')
            self.get_logger().error('Is the robot running?')
            return False

        request = SwitchController.Request()
        
        if activate:
            self.get_logger().info('Stopping arm_trajectory_controller...')
            request.start_controllers = []
            request.stop_controllers = ['arm_trajectory_controller']
        else:
            self.get_logger().info('Starting arm_trajectory_controller...')
            request.start_controllers = ['arm_trajectory_controller']
            request.stop_controllers = []
        
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.start_asap = False
        request.timeout = Duration(sec=0, nanosec=0)
        
        try:
            future = self.switch_controller.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.done():
                result = future.result()
                if result.ok:
                    self.get_logger().info('Successfully changed controller state')
                    return True
                else:
                    self.get_logger().error('Failed to change controller state')
                    return False
            else:
                self.get_logger().error('Service call timed out')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 2 or sys.argv[1] not in ['stop', 'start']:
        print("Usage: trigger_emergency_stop.py [stop|start]")
        return 1
    
    client = EmergencyStopClient()
    
    try:
        success = client.send_emergency_stop(sys.argv[1] == 'stop')
        return 0 if success else 1
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    sys.exit(main()) 
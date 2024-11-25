#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Empty
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration

class EmergencyStopMarker(Node):
    def __init__(self):
        super().__init__('emergency_stop_marker')
        
        # Create callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Flag to track initialization
        self.initialized = False
        
        # Create Interactive Marker Server
        self.get_logger().info("Creating Interactive Marker Server...")
        self.server = InteractiveMarkerServer(
            self, 
            'emergency_stop_button'
        )
        self.get_logger().info("Server created")
        
        # Store the marker for updates
        self.int_marker = None
        
        # Wait a moment for the server to initialize
        self.init_timer = self.create_timer(1.0, self.delayed_init, callback_group=self.callback_group)
        
        # Add a regular marker publisher for testing
        self.test_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # Add publisher for trajectory control
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/pro_arm_controller/joint_trajectory',
            10
        )
        
        # Add service client for stopping MoveIt
        self.stop_moveit = self.create_client(Empty, '/stop_move_group')
        
        # Publisher for controller commands
        self.controller_pub = self.create_publisher(
            String,
            '/pro_arm_controller/commands',
            10
        )
        
        # Flag to track if stop is active
        self.stop_active = False
        
        # Add service client for controller manager
        self.switch_controller = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )

    def delayed_init(self):
        """Initialize marker after server is ready"""
        if self.initialized:
            return
        
        self.initialized = True
        self.init_timer.cancel()
        
        # Create and store the marker
        self.make_e_stop_marker()
        self.get_logger().info('Emergency Stop Button initialized')
        
        # Add periodic update timer with higher frequency
        self.create_timer(0.1, self.update_marker, callback_group=self.callback_group)

    def make_e_stop_marker(self):
        self.get_logger().info("Creating emergency stop marker...")
        
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = "pro_arm_base_link"
        self.int_marker.name = "emergency_stop"
        self.int_marker.description = "Emergency Stop"
        
        # Position it in a convenient location
        self.int_marker.pose.position.x = 0.5
        self.int_marker.pose.position.y = 0.0  # Centered
        self.int_marker.pose.position.z = 0.3  # Lower height for better visibility
        self.int_marker.scale = 0.2  # Slightly smaller overall scale
        
        # Create the main button control
        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.name = "button_control"
        button_control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        button_control.always_visible = True
        
        # Create a red octagon (STOP sign shape)
        button_marker = Marker()
        button_marker.type = Marker.CYLINDER
        button_marker.scale.x = 0.15
        button_marker.scale.y = 0.15
        button_marker.scale.z = 0.04
        button_marker.color.r = 0.8
        button_marker.color.g = 0.0
        button_marker.color.b = 0.0
        button_marker.color.a = 1.0
        # Set marker orientation to face camera
        button_marker.pose.orientation.x = 0.0
        button_marker.pose.orientation.y = 0.7071
        button_marker.pose.orientation.z = 0.0
        button_marker.pose.orientation.w = 0.7071
        
        # Add a border ring
        border_marker = Marker()
        border_marker.type = Marker.CYLINDER
        border_marker.scale.x = 0.17
        border_marker.scale.y = 0.17
        border_marker.scale.z = 0.035
        border_marker.color.r = 1.0
        border_marker.color.g = 1.0
        border_marker.color.b = 1.0
        border_marker.color.a = 1.0
        border_marker.pose.position.z = -0.001
        border_marker.pose.orientation = button_marker.pose.orientation
        
        # Add glow effect
        glow_marker = Marker()
        glow_marker.type = Marker.CYLINDER
        glow_marker.scale.x = 0.19
        glow_marker.scale.y = 0.19
        glow_marker.scale.z = 0.03
        glow_marker.color.r = 1.0
        glow_marker.color.g = 0.0
        glow_marker.color.b = 0.0
        glow_marker.color.a = 0.3
        glow_marker.pose.position.z = -0.002
        glow_marker.pose.orientation = button_marker.pose.orientation
        
        # Create text in a separate control to ensure it's always visible
        text_control = InteractiveMarkerControl()
        text_control.interaction_mode = InteractiveMarkerControl.NONE
        text_control.name = "text"
        text_control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        text_control.always_visible = True
        
        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.text = "STOP"
        text_marker.scale.z = 0.04 # Text size
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.pose.position.x = -0.04  # Move text slightly forward
        
        # Add markers to their respective controls
        button_control.markers.append(border_marker)
        button_control.markers.append(button_marker)
        button_control.markers.append(glow_marker)
        
        text_control.markers.append(text_marker)
        
        # Add both controls to the interactive marker
        self.int_marker.controls.append(button_control)
        self.int_marker.controls.append(text_control)
        
        try:
            self.server.insert(self.int_marker, feedback_callback=self.process_feedback)
            self.server.applyChanges()
            self.get_logger().info("Emergency stop button created")
        except Exception as e:
            self.get_logger().error(f"Failed to create marker: {str(e)}")

    def update_marker(self):
        """Periodically update the marker to ensure visibility"""
        try:
            if self.int_marker:
                # Update the timestamp
                self.int_marker.header.stamp = self.get_clock().now().to_msg()
                
                # Re-insert the marker with updated timestamp
                self.server.insert(self.int_marker, feedback_callback=self.process_feedback)
                self.server.applyChanges()
                self.get_logger().debug("Marker updated")
                
                # Also update test marker
                test_marker = Marker()
                test_marker.header.frame_id = "pro_arm_base_link"
                test_marker.header.stamp = self.get_clock().now().to_msg()
                test_marker.id = 0
                test_marker.type = Marker.SPHERE
                test_marker.action = Marker.ADD
                test_marker.pose.position.x = 0.5
                test_marker.pose.position.y = 0.5
                test_marker.pose.position.z = 0.5
                test_marker.scale.x = 0.2
                test_marker.scale.y = 0.2
                test_marker.scale.z = 0.2
                test_marker.color.r = 1.0
                test_marker.color.g = 0.0
                test_marker.color.b = 0.0
                test_marker.color.a = 1.0
                
                self.test_pub.publish(test_marker)
        except Exception as e:
            self.get_logger().error(f"Failed to update marker: {str(e)}")

    def process_feedback(self, feedback):
        """Process feedback from the interactive marker"""
        if feedback.event_type == feedback.BUTTON_CLICK:
            if not self.stop_active:
                self.activate_emergency_stop()
            else:
                self.deactivate_emergency_stop()

    def activate_emergency_stop(self):
        """Activate emergency stop"""
        try:
            self.get_logger().info('Activating Emergency Stop!')
            
            # Stop all controllers (except joint state broadcaster)
            request = SwitchController.Request()
            request.start_controllers = []
            request.stop_controllers = ['arm_trajectory_controller']
            request.strictness = SwitchController.Request.BEST_EFFORT
            request.start_asap = False
            request.timeout = Duration(sec=0, nanosec=0)
            
            if not self.switch_controller.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Controller manager service not available')
            else:
                future = self.switch_controller.call_async(request)
                self.get_logger().info('Stopping controllers...')
            
            # Update button appearance
            if self.int_marker:
                for control in self.int_marker.controls:
                    for marker in control.markers:
                        if marker.type == Marker.CYLINDER:
                            marker.color.r = 0.7
                            marker.color.g = 0.0
                            marker.color.b = 0.0
                            marker.color.a = 1.0
                
                self.server.insert(self.int_marker)
                self.server.applyChanges()
            
            self.stop_active = True
            self.get_logger().info('Emergency Stop Activated!')
            
        except Exception as e:
            self.get_logger().error(f'Failed to activate emergency stop: {str(e)}')

    def deactivate_emergency_stop(self):
        """Deactivate emergency stop"""
        try:
            # Restart controllers
            request = SwitchController.Request()
            request.start_controllers = ['arm_trajectory_controller']
            request.stop_controllers = []
            request.strictness = SwitchController.Request.BEST_EFFORT
            request.start_asap = False
            request.timeout = Duration(sec=0, nanosec=0)
            
            if self.switch_controller.wait_for_service(timeout_sec=1.0):
                future = self.switch_controller.call_async(request)
                self.get_logger().info('Restarting controllers...')
            
            # Reset button appearance
            if self.int_marker:
                for control in self.int_marker.controls:
                    for marker in control.markers:
                        if marker.type == Marker.CYLINDER:
                            marker.color.r = 1.0
                            marker.color.g = 0.0
                            marker.color.b = 0.0
                            marker.color.a = 1.0
                
                self.server.insert(self.int_marker)
                self.server.applyChanges()
            
            self.stop_active = False
            self.get_logger().info('Emergency Stop Deactivated')
            
        except Exception as e:
            self.get_logger().error(f'Failed to deactivate emergency stop: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
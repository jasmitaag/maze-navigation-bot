import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry

# Import custom algorithms 
from algorithms.a_star import AStarPlanner
from algorithms.bresenham import smooth_path
from algorithms.pid_wall_follower import PIDController
from algorithms.spatial_memory import SpatialMemory
from algorithms.recovery_behaviors import RecoveryManager
from algorithms.ekf_sensor_fusion import EKFSensorFusion
from algorithms.morse_debug import MorseDebugger
from algorithms.dynamic_obstacle_detection import ObstacleDetector

class MazeNavigator(Node):
    def __init__(self):
        super().__init__('maze_navigator')
        
        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer for main loop
        self.create_timer(0.1, self.main_loop)

        # Instantiate modules
        self.a_star = AStarPlanner()
        self.pid = PIDController()
        self.memory = SpatialMemory()
        self.recovery = RecoveryManager()
        self.ekf = EKFSensorFusion()
        self.debugger = MorseDebugger()
        self.obstacle_detector = ObstacleDetector()

        # Sensor data placeholders
        self.laser_data = None
        self.imu_data = None
        self.odom_data = None

        self.get_logger().info('Maze Navigator Initialized')

    def laser_callback(self, msg):
        self.laser_data = msg
        self.obstacle_detector.update_scan(msg)

    def imu_callback(self, msg):
        self.imu_data = msg
        self.ekf.update_imu(msg)
        if self.recovery.tilt_detected(msg):
            self.recovery.execute_tilt_recovery(self.cmd_pub)

    def odom_callback(self, msg):
        self.odom_data = msg
        self.ekf.update_odometry(msg)

    def main_loop(self):
        if not all([self.laser_data, self.imu_data, self.odom_data]):
            return

        # Sensor fusion update
        position = self.ekf.get_fused_pose()

        # Dead-end detection and memory logging
        if self.memory.detect_dead_end(self.laser_data):
            self.recovery.execute_reverse_escape(self.cmd_pub)
            self.memory.log_dead_end(position)
            return

        # Dynamic obstacle handling
        if self.obstacle_detector.detect_dynamic_obstacle():
            self.recovery.execute_rotate_recovery(self.cmd_pub)
            return

        # Plan path using A* and follow with PID
        path = self.a_star.plan(position)
        smooth = smooth_path(path)
        cmd = self.pid.compute_cmd(smooth, position)
        
        # Publish the final velocity command
        self.cmd_pub.publish(cmd)
        
        # Optional debugging (e.g., Morse code messages)
        self.debugger.send_status('N')  # N for normal

def main(args=None):
    rclpy.init(args=args)
    node = MazeNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

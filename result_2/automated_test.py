import time
import math
import csv
import threading
import re
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import Log
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile

# --- IMPORT THƯ VIỆN TF ---
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

# --- CẤU HÌNH TỌA ĐỘ (BẠN SỬA Ở ĐÂY) ---
# Điểm A: Vị trí xuất phát của bài test
POINT_A = {'x': 3.0, 'y': -7.0, 'w': 1.0} 

# Điểm B: Đích đến của bài test
POINT_B = {'x': -1, 'y': 6.0, 'w': 1.0}

SAFETY_DISTANCE = 0.2
NUM_TESTS = 10
OUTPUT_FILE = 'ket_qua_thi_nghiem_final.csv'
# ----------------

class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')
        
        # 1. Thiết lập TF Buffer để theo dõi vị trí robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 2. Timer để cập nhật vị trí và tính quãng đường (10Hz - 0.1s một lần)
        self.timer = self.create_timer(0.1, self.update_position_callback)

        # 3. Subscriber Scan & Log (Giữ nguyên)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            
        self.log_sub = self.create_subscription(Log, '/rosout', self.log_callback, 100)

        # Biến lưu trữ
        self.current_path_length = 0.0
        self.last_x = None
        self.last_y = None
        self.safety_violations = 0
        self.min_obstacle_dist = 999.0
        self.planning_times = []
        
        self.is_recording = False
        self.robot_base_frame = 'base_link' # Tên khung robot (thường là base_link hoặc base_footprint)
        self.global_frame = 'map'           # Tên khung bản đồ

    def update_position_callback(self):
        if not self.is_recording: return
        
        try:
            # Hỏi TF: Robot đang ở đâu so với Map?
            # timeout=0.0: Lấy giá trị mới nhất có sẵn
            trans = self.tf_buffer.lookup_transform(
                self.global_frame, 
                self.robot_base_frame, 
                rclpy.time.Time())
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            # Tính quãng đường
            if self.last_x is not None:
                dist = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
                # Chỉ cộng nếu robot thực sự di chuyển (tránh nhiễu)
                if dist > 0.001: 
                    self.current_path_length += dist
            
            self.last_x = x
            self.last_y = y
            
        except (LookupException, ConnectivityException, ExtrapolationException):
            # Lúc mới bật robot chưa định vị xong có thể lỗi nhẹ, bỏ qua
            pass

    def scan_callback(self, msg):
        if not self.is_recording: return
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        if valid_ranges:
            min_dist = min(valid_ranges)
            if min_dist < self.min_obstacle_dist:
                self.min_obstacle_dist = min_dist
            if min_dist < SAFETY_DISTANCE:
                self.safety_violations += 1

    def log_callback(self, msg):
        if not self.is_recording: return
        log_msg = msg.msg
        time_match = re.search(r"time:?\s*(\d+\.\d+)", log_msg, re.IGNORECASE)
        if time_match:
            try:
                self.planning_times.append(float(time_match.group(1)))
            except ValueError: pass

    def start_logging(self):
        self.current_path_length = 0.0
        self.safety_violations = 0
        self.min_obstacle_dist = 999.0
        self.last_x = None
        self.last_y = None
        self.planning_times = []
        self.is_recording = True

    def stop_logging(self):
        self.is_recording = False
        
    def get_avg_plan_time(self):
        if self.planning_times:
            return sum(self.planning_times) / len(self.planning_times)
        return 0.0

def create_pose(navigator, x, y, w):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = float(w)
    return pose

def main():
    rclpy.init()
    
    logger_node = MetricsLogger()
    executor = MultiThreadedExecutor()
    executor.add_node(logger_node)
    
    spinner_thread = threading.Thread(target=executor.spin, daemon=True)
    spinner_thread.start()

    navigator = BasicNavigator()
    print("Đang đợi Nav2 khởi động...")
    navigator.waitUntilNav2Active()

    # --- BƯỚC 1: VỀ ĐÍCH (ĐIỂM A) ---
    print(f"-> Di chuyển đến điểm xuất phát ({POINT_A['x']}, {POINT_A['y']})...")
    navigator.goToPose(create_pose(navigator, POINT_A['x'], POINT_A['y'], POINT_A['w']))
    while not navigator.isTaskComplete(): pass 
    time.sleep(2.0)

    # --- BƯỚC 2: CHẠY TEST ---
    with open(OUTPUT_FILE, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Test Run', 'Status', 'Time (s)', 'Path Length (m)', 'Min Dist (m)', 'Safety Violations', 'Avg Plan Time (ms)'])

        for i in range(1, NUM_TESTS + 1):
            print(f"\n--- Run {i}/{NUM_TESTS} ---")
            
            goal_pose = create_pose(navigator, POINT_B['x'], POINT_B['y'], POINT_B['w'])

            logger_node.start_logging()
            start_time = time.time()
            navigator.goToPose(goal_pose)

            while not navigator.isTaskComplete(): pass 
            
            end_time = time.time()
            logger_node.stop_logging()
            
            result = navigator.getResult()
            duration = end_time - start_time
            status = "SUCCESS" if result == TaskResult.SUCCEEDED else "FAILED"
            
            if status == "SUCCESS": 
                print(f"-> Xong! Time: {duration:.2f}s | Dist: {logger_node.current_path_length:.2f}m")
            else: 
                print("-> Thất bại!")

            avg_plan = logger_node.get_avg_plan_time()

            writer.writerow([
                i, 
                status, 
                f"{duration:.2f}", 
                f"{logger_node.current_path_length:.2f}", 
                f"{logger_node.min_obstacle_dist:.3f}", 
                logger_node.safety_violations,
                f"{avg_plan:.4f}"
            ])
            
            # Reset
            print("-> Reset về A...")
            navigator.goToPose(create_pose(navigator, POINT_A['x'], POINT_A['y'], POINT_A['w']))
            while not navigator.isTaskComplete(): pass
            time.sleep(1.0)

    print(f"\nDONE! File: {OUTPUT_FILE}")
    executor.shutdown()
    logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
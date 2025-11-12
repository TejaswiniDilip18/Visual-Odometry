import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path, Odometry
from cv_bridge import CvBridge
import tf2_ros
# from tf_transformations import quaternion_from_matrix
from scipy.spatial.transform import Rotation as Rs
import cv2
import numpy as np
import os
from .utm_python import get_absolute_scale  

class VONode(Node):
    def __init__(self):
        super().__init__('vo_main')

        # CV bridge
        self.bridge = CvBridge()

        # Camera intrinsics from KITTI dataset
        self.focal = 718.8560
        self.pp = (607.1928, 185.2157)

        # Camera matrix
        self.K = np.array([
            [self.focal, 0, self.pp[0]],
            [0, self.focal, self.pp[1]],
            [0, 0, 1]
        ], dtype=np.float32)

        # Pose tracking variables
        self.R_f = np.eye(3, dtype=np.float64)  # Current rotation
        self.t_f = np.zeros((3, 1), dtype=np.float64)  # Current translation
        
        self.prev_image = None
        self.prev_features = None

        self.trajectory_points = []

        self.frame_count = 0
        self.intialized = False
        self.camera_info_received = False

        self.declare_parameter('folder_path', '/root/ros2_docker_ws/kitti_dataset/2011_10_03_drive_0027_sync/2011_10_03/2011_10_03_drive_0027_sync')
        self.declare_parameter('oxts_data', '/root/ros2_docker_ws/kitti_dataset/2011_10_03_drive_0027_sync/2011_10_03/2011_10_03_drive_0027_sync/oxts/data')
        self.declare_parameter('true_pose', '/root/ros2_docker_ws/kitti_dataset/2011_10_03_drive_0027_sync/data_odometry_poses/dataset/poses/00.txt')
        self.declare_parameter('frame_rate', 10.0)
        self.declare_parameter('min_num_features', 2000)
        self.declare_parameter('use_camera', False)
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')

        self.folder_path = self.get_parameter('folder_path').value
        self.oxts_data = self.get_parameter('oxts_data').value
        self.true_pose = self.get_parameter('true_pose').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.min_num_features = self.get_parameter('min_num_features').value
        self.use_camera = self.get_parameter('use_camera').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/vo/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/vo/odometry', 10)
        self.path_pub = self.create_publisher(Path, '/vo/path', 10)
        self.image_pub = self.create_publisher(Image, '/vo/current_frame', 10)

        # Publishers for GROUND TRUTH trajectory
        self.gt_pose_pub = self.create_publisher(PoseStamped, '/vo/ground_truth/pose', 10)
        self.gt_path_pub = self.create_publisher(Path, '/vo/ground_truth/path', 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        self.gt_path_msg = Path()  # Ground truth path
        self.gt_path_msg.header.frame_id = 'odom'

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        if self.use_camera:
            self.image_sub = self.create_subscription(Image, self.camera_topic, self.camera_callback, 10)
            self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
            self.get_logger().info(f'Subscribed to camera topic: {self.camera_topic}')
        else:
            self.timer = self.create_timer(1.0 / self.frame_rate, self.process_next_frame)
    
    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.process_frame(cv_image, msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')
    
    def camera_info_callback(self, msg):
        """Update camera parameters from camera_info message"""
        if not self.camera_info_received:
            self.K = np.array(msg.k).reshape(3, 3).astype(np.float64)
            self.focal = self.K[0, 0]  # assuming fx = fy
            self.pp = (self.K[0, 2], self.K[1, 2])
            self.camera_info_received = True
            self.get_logger().info(f'Camera info received: focal={self.focal:.2f}, pp={self.pp}')
    
    def process_next_frame(self):
        """Load and process next frame from the dataset"""

        filename = os.path.join(self.folder_path, "image_02", "data", f"{self.frame_count:010d}.png")

        if not os.path.exists(filename):
            self.get_logger().info(f'Finished processing at frame {self.frame_count}')
            return
        
        cv_image = cv2.imread(filename)

        if cv_image is None:
            self.get_logger().info(f'Failed to load image: {filename}')
            return
        
        timestamp = self.get_clock().now().to_msg()

        self.process_frame(cv_image, timestamp)
        self.frame_count += 1
    
    def process_frame(self, cv_image, timestamp):
        """Main VO processing"""

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        if not self.intialized:
            self.initialize_vo(gray_image)
            return
        
        curr_features = self.track_features(self.prev_image, gray_image, self.prev_features)

        success = self.estimate_motion(self.prev_features, curr_features)
        
        if success:
            self.publish_pose(timestamp)
            self.publish_odometry(timestamp)
            self.publish_path(timestamp)
            self.publish_transform(timestamp)
            if not self.use_camera:
                self.publish_ground_truth(timestamp)

            img_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            img_msg.header.stamp = timestamp
            img_msg.header.frame_id = 'camera'
            self.image_pub.publish(img_msg)

        self.prev_image = gray_image.copy()
        self.prev_features = curr_features

        if len(curr_features) < self.min_num_features:
            new_features = self.detect_features(gray_image)
            if new_features is not None:
                self.prev_features = new_features
    

    def initialize_vo(self, first_image):
        """Initialize VO with first two frames"""

        if self.prev_image is None:
            self.prev_image = first_image.copy()
            self.prev_features = self.detect_features(first_image)
            return
        
        curr_features = self.track_features(self.prev_image, first_image, self.prev_features)
        
        if self.estimate_motion(self.prev_features, curr_features):
            self.intialized = True
            self.get_logger().info('VO initialized successfully')
        
        self.prev_image = first_image.copy()
        self.prev_features = curr_features

    def detect_features(self, image):
        """Detect image features"""
        points = cv2.goodFeaturesToTrack(
            image, 
            maxCorners=2000,
            qualityLevel=0.01,
            minDistance=15
        )
        
        if points is not None and len(points) < 1000:
            points = cv2.goodFeaturesToTrack(
                image, 
                maxCorners=2000,
                qualityLevel=0.005,
                minDistance=15
            )
        
        return points.astype(np.float32) if points is not None else None
    
    def track_features(self, prev_img, curr_img, prev_points):
        """Track Features"""
        if prev_points is None or len(prev_points) == 0:
            return None
     
        lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )
    
        curr_points, status, error = cv2.calcOpticalFlowPyrLK(prev_img, curr_img, prev_points, None, **lk_params,  minEigThreshold=0.001)

        if curr_points is None:
            return None

        good = status.ravel() == 1
        good_prev = prev_points[good]
        good_curr = curr_points[good]

        h, w = curr_img.shape
        good_curr_reshaped = good_curr.reshape(-1, 2)
        in_bounds = (
            (good_curr_reshaped[:, 0] >= 0) & 
            (good_curr_reshaped[:, 0] < w) &
            (good_curr_reshaped[:, 1] >= 0) & 
            (good_curr_reshaped[:, 1] < h)
        )
        
        final_prev = good_prev[in_bounds]
        final_curr = good_curr[in_bounds]

        # Store both for next iteration
        self.prev_features = final_prev
        return final_curr
    
    def estimate_motion(self, prev_points, curr_points):
        """Estimate camera motion from point correspondences"""
        if prev_points is None or curr_points is None:
            return False
        
        if len(prev_points) < 8 or len(curr_points) < 8:
            return False
        
        # Find essential matrix
        E, mask = cv2.findEssentialMat(
            curr_points,
            prev_points,
            cameraMatrix=self.K,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=0.5,
            maxIters= 5000
        )

        _, R, t, pose_mark = cv2.recoverPose(
            E,
            curr_points,
            prev_points,
            focal=self.focal,
            pp=self.pp,
            mask=mask
        )
        if self.use_camera:
            scale = 1
        else:
            gps_distance = get_absolute_scale(self.frame_count, self.oxts_data)
            scale = gps_distance / np.linalg.norm(t)

        if not self.use_camera and scale > 0.1 and t[2, 0] > t[0, 0] and t[2, 0] > t[1, 0]:
            self.t_f = self.t_f + scale * (self.R_f @ t)
            self.R_f = R @ self.R_f
        else:
            print(f"Scale validation failed at frame {self.frame_count}: scale={scale}")

        return True
    
    def matrix_to_quaternion(self, rotation_matrix):
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]"""
        try:
            r = Rs.from_matrix(rotation_matrix)
            quat_xyzw = r.as_quat() 
            return quat_xyzw
        except Exception as e:
            self.get_logger().warn(f'Quaternion conversion failed: {e}, using identity')
            return [0.0, 0.0, 0.0, 1.0] 

    def get_true_pose(self, frame_id, true_pose_path):
        """Get ground truth pose from KITTI pose file"""
        try:
            if not os.path.exists(true_pose_path):
                return None, None
            
            with open(true_pose_path, 'r') as f:
                lines = f.readlines()
                
            if frame_id < len(lines):
                values = [float(x) for x in lines[frame_id].strip().split()]
                if len(values) >= 12:
                    # translation
                    x, y, z = values[3], values[7], values[11]
                    
                    # rotation matrix
                    R_true = np.array([
                        [values[0], values[1], values[2]],
                        [values[4], values[5], values[6]],  
                        [values[8], values[9], values[10]]
                    ], dtype=np.float64)
                    
                    return [x, y, z], R_true
            
            return None, None
            
        except Exception as e:
            self.get_logger().error(f"Error reading ground truth pose: {e}")
            return None, None
    
    def publish_pose(self, timestamp):

        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = 'odom'

        pose_msg.pose.position.x = float(self.t_f[0, 0])
        pose_msg.pose.position.y = -float(self.t_f[1, 0])
        pose_msg.pose.position.z = float(self.t_f[2, 0])

        q = self.matrix_to_quaternion(self.R_f)

        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.pose_pub.publish(pose_msg)
    
    def publish_odometry(self, timestamp):

        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = float(self.t_f[0, 0])
        odom_msg.pose.pose.position.y = float(self.t_f[1, 0])
        odom_msg.pose.pose.position.z = float(self.t_f[2, 0])
        
        q = self.matrix_to_quaternion(self.R_f)
        
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Add covariance 
        odom_msg.pose.covariance[0] = 0.1  # x
        odom_msg.pose.covariance[7] = 0.1  # y
        odom_msg.pose.covariance[14] = 0.1  # z
        odom_msg.pose.covariance[21] = 0.05  # roll
        odom_msg.pose.covariance[28] = 0.05  # pitch
        odom_msg.pose.covariance[35] = 0.05  # yaw
        
        self.odom_pub.publish(odom_msg)
    
    def publish_path(self, timestamp):

        # Add current pose to path
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = timestamp
        pose_stamped.header.frame_id = 'odom'
        
        pose_stamped.pose.position.x = float(self.t_f[0, 0])
        pose_stamped.pose.position.y = -float(self.t_f[1, 0])
        pose_stamped.pose.position.z = float(self.t_f[2, 0])

        q = self.matrix_to_quaternion(self.R_f)
        
        pose_stamped.pose.orientation.x = q[0]
        pose_stamped.pose.orientation.y = q[1]
        pose_stamped.pose.orientation.z = q[2]
        pose_stamped.pose.orientation.w = q[3]
        
        self.path_msg.poses.append(pose_stamped)
        self.path_msg.header.stamp = timestamp
        
        self.path_pub.publish(self.path_msg)
    
    def publish_ground_truth(self, timestamp):
        """Publish GROUND TRUTH pose and path"""
        gt_pose_data, gt_rotation = self.get_true_pose(self.frame_count, self.true_pose)
        
        if gt_pose_data is None or gt_rotation is None:
            return  

        x_true, y_true, z_true = gt_pose_data

        # Publish ground truth pose
        gt_pose_msg = PoseStamped()
        gt_pose_msg.header.stamp = timestamp
        gt_pose_msg.header.frame_id = 'odom'
        
        gt_pose_msg.pose.position.x = float(x_true)
        gt_pose_msg.pose.position.y = float(y_true)
        gt_pose_msg.pose.position.z = float(z_true)

        q_gt = self.matrix_to_quaternion(gt_rotation)
        gt_pose_msg.pose.orientation.x = q_gt[0]
        gt_pose_msg.pose.orientation.y = q_gt[1]
        gt_pose_msg.pose.orientation.z = q_gt[2]
        gt_pose_msg.pose.orientation.w = q_gt[3]

        self.gt_pose_pub.publish(gt_pose_msg)

        # Add to ground truth path
        self.gt_path_msg.poses.append(gt_pose_msg)
        self.gt_path_msg.header.stamp = timestamp
        self.gt_path_pub.publish(self.gt_path_msg)
    
    def publish_transform(self, timestamp):

        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = float(self.t_f[0, 0])
        t.transform.translation.y = float(self.t_f[1, 0])
        t.transform.translation.z = float(self.t_f[2, 0])

        q = self.matrix_to_quaternion(self.R_f)
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):

    rclpy.init(args=args)

    visual_odometry_node = VONode()

    try:
        rclpy.spin(visual_odometry_node)
    except KeyboardInterrupt:
        pass
    finally:
        visual_odometry_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
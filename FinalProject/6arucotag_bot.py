##Subsribes to camera and detects aruco location and publishes coordinates. THis aruco tag is on the turtlebot
###same node from lab 


#Aruco tag detection for bot

#!/usr/bin/env python3
"""
Aruco-only node 

What this node does 
1) Listens to a camera image (/image_raw) and a camera calibration message (/camera_info).
2) Looks for a printed black-and-white ArUco tag (DICT_4X4_50, ID=1) in the image.
3) If the tag is found, estimates the tag's 3D position and orientation relative to the camera.
4) Publishes that pose on /aruco_pose so RViz can show a 3D arrow.
5) Also draws a green box around the tag in the live camera window.

Why we need /camera_info:
- Pose estimation requires knowing your camera's focal length and principal point.
- That info comes from a calibration YAML that your camera driver loads.
"""

import rclpy
from rclpy.node import Node

# Messages we use
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

# OpenCV + helpers
import cv2
import numpy as np
from cv_bridge import CvBridge
import cv2.aruco as aruco

# ---------------------------------------------
#  SETTINGS that could change
# ---------------------------------------------
ARUCO_ID   = 1                 # We look for tag ID=1
ARUCO_DICT = aruco.DICT_4X4_50 # Dictionary to match your printed tag
MARKER_LEN = 0.04              # << TODO: MEASURE YOUR PRINTED TAG SIDE (meters). Example: 4 cm = 0.04

# ---------------------------------------------
# The node
# ---------------------------------------------
class ArucoOnlyNode(Node):
    def __init__(self):
        super().__init__('aruco_only_node')

        # Used to convert ROS Image <-> OpenCV image
        self.bridge = CvBridge()

        # Subscribe to camera topics
        # If your camera is namespaced (e.g. /camera/image_raw), adjust below:
        self.sub_img  = self.create_subscription(Image, '/image_raw',    self.on_image, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/camera_info', self.on_info, 10)

        # We publish the pose here so RViz (or other nodes) can read it
        self.pub_pose = self.create_publisher(PoseStamped, '/aruco_pose_bot', 10)

        # Will be filled once we receive /camera_info
        self.K = None   # 3x3 camera matrix with focal lengths + principal point
        self.D = None   # distortion coefficients
        self.logged_intrinsics = False

        # Configure the ArUco detector
        self.dict = aruco.getPredefinedDictionary(ARUCO_DICT)
        self.params = aruco.DetectorParameters_create()

        self.get_logger().info("Aruco-only: waiting for /camera_info, then looking for DICT_4X4_50 ID=1")

    # -------------------------------------------------
    # /camera_info callback comes from your camera driver
    # -------------------------------------------------
    def on_info(self, msg: CameraInfo):
        # Convert the flat arrays into nice numpy arrays
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)   # camera matrix
        self.D = np.array(msg.d, dtype=np.float64).ravel()         # distortion coeffs

        # Print once so know calibration is actually loaded
        if not self.logged_intrinsics:
            fx, fy, cx, cy = self.K[0,0], self.K[1,1], self.K[0,2], self.K[1,2]
            self.get_logger().info(
                f"[OK] Received /camera_info  -> fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}"
            )
            self.logged_intrinsics = True

    # -------------------------------------------------
    # /image_raw callback, each time the camera gives a frame
    # -------------------------------------------------
    def on_image(self, msg: Image):
        # Convert ROS Image to an OpenCV BGR image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # If we don't have intrinsics yet, we can't do 3D pose
        if self.K is None:
            cv2.putText(frame, "Waiting for /camera_info (did you load YAML?)",
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            cv2.imshow("ArUco Only", frame)
            cv2.waitKey(1)
            return

        # 1) Detect ArUco markers in the image
        corners, ids, _ = aruco.detectMarkers(frame, self.dict, parameters=self.params)

        # 2) If we found any, look for the specific ID we care about
        if ids is not None:
            ids = ids.ravel()  # flatten (e.g., [[1],[7]] -> [1,7])
            if ARUCO_ID in ids:
                idx = list(ids).index(ARUCO_ID)

                # Draw a green box around that marker
                aruco.drawDetectedMarkers(frame, [corners[idx]], borderColor=(0, 255, 0))

                # 3) Estimate pose of the marker using camera intrinsics + marker size
                #    - rvec: rotation vector (axis-angle form)
                #    - tvec: translation vector (x,y,z) of the marker origin in camera frame [meters]
                
                #TODO call aruco.estimatePoseSingleMarkers with the correct parameters
                
		#to estimate the pos of the marker
               ## rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers([corners[idx]], , , )
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers([corners[idx]], MARKER_LEN, self.K, self.D)###############

                rvec = rvecs[0, 0, :]   # shape (3,)
                tvec = tvecs[0, 0, :]   # shape (3,)

                # 4) Convert rotation vector -> rotation matrix -> quaternion (for Pose)
                R, _ = cv2.Rodrigues(rvec)
                qw = np.sqrt(max(1.0 + R[0,0] + R[1,1] + R[2,2], 1e-12)) / 2.0
                qx = (R[2,1] - R[1,2])/(4*qw); qy = (R[0,2] - R[2,0])/(4*qw); qz = (R[1,0] - R[0,1])/(4*qw)

                # 5) Publish PoseStamped so RViz can show a 3D arrow
                pose = PoseStamped()
                pose.header = msg.header
                pose.header.frame_id = "camera"  # pose is relative to the camera
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, tvec)
                pose.pose.orientation.x = float(qx)
                pose.pose.orientation.y = float(qy)
                pose.pose.orientation.z = float(qz)
                pose.pose.orientation.w = float(qw)
                self.pub_pose.publish(pose)

                # 6) Overlay some text
                cv2.putText(frame,
                            f"Tag {ARUCO_ID}: x={tvec[0]:.3f} y={tvec[1]:.3f} z={tvec[2]:.3f} (m)",
                            (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(frame, f"ArUco found, but not ID={ARUCO_ID}",
                            (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 200), 2)
        else:
            cv2.putText(frame, "No ArUco detected",
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        # 7) Always show the image window for quick visual feedback
        cv2.imshow("ArUco Only", frame)
        cv2.waitKey(1)

# ---------------------------------------------
# Main
# ---------------------------------------------
def main():
    rclpy.init()
    node = ArucoOnlyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

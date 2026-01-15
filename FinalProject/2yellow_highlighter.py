#!/usr/bin/env python3
"""
Homography + Blob Mapper:
- Computes homography H from 8x7 checkerboard.
- Segments a colored blob (highlighter) via HSV.
- Maps blob centroid pixel (u,v) -> board (X,Y) meters using H.
- Publishes geometry_msgs/PointStamped on /blob_board with frame_id="board".
"""
import rclpy, cv2, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

PATTERN = (7, 6)
SQUARE  = 0.025

HSV_PRESETS = {
    "yellow": ((20,  90,  90), (35, 255, 255)),
    "orange": ((5,  120, 90), (18, 255, 255)),
    "pink"  : ((150, 80, 90), (179, 255, 255)),
}
ACTIVE_COLOR = "yellow"  # === TODO: change if your marker differs

class HomogBlobMapperNode(Node):
    def __init__(self):
        super().__init__('homog_blob_mapper_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/image_raw', self.on_image, 10)
        self.pub_board = self.create_publisher(PointStamped, '/blob_board_yellow', 10)
        self.H = None
        lo, hi = HSV_PRESETS[ACTIVE_COLOR]
        self.lo, self.hi = np.array(lo), np.array(hi)
        self.get_logger().info("Homog+Blob: publishing /blob_board (X,Y,0) in meters (frame 'board').")

    def compute_h(self, gray):
        ok, corners = cv2.findChessboardCorners(
            gray, PATTERN,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        if not ok:
            return False
        corners = cv2.cornerSubPix(
            gray, corners, (11,11), (-1,-1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
        )
        img_pts = corners.reshape(-1,2).astype(np.float32)
        cols, rows = PATTERN
        board_pts = np.array(
            [[i*SQUARE, j*SQUARE] for j in range(rows) for i in range(cols)],
            dtype=np.float32
        )
        # TODO fill in same homography function here
        #H, _ = cv2.findHomography(, , cv2.RANSAC)
        H, mask = cv2.findHomography(img_pts, board_pts , cv2.RANSAC)
        
        if H is not None:
            if self.H is None or np.max(np.abs(H - self.H)) > 1e-9:
                self.H = H
                self.get_logger().info("Homography updated.")
            return True
        return False

    def uv_to_board(self, u, v):
        if self.H is None: return None
        uv1 = np.array([u, v, 1.0], np.float64)
        XYw = self.H @ uv1
        if abs(XYw[2]) < 1e-9: return None
        return (float(XYw[0]/XYw[2]), float(XYw[1]/XYw[2]))

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Update H if board visible
        self.compute_h(gray)

        # Segment blob
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lo, self.hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5,5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7,7), np.uint8))

	# TODO fill in the same cv2.findContours function here
        #cnts, _ = cv2.findContours(,, )
        cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
        if cnts and self.H is not None:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) > 300:
                M = cv2.moments(c)
                if M['m00'] != 0:
                    cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                    XY = self.uv_to_board(cx, cy)
                    if XY:
                        X, Y = XY
                        pt = PointStamped()
                        pt.header = msg.header
                        pt.header.frame_id = 'board'
                        pt.point.x, pt.point.y, pt.point.z = X, Y, 0.0
                        self.pub_board.publish(pt)

                        cv2.drawContours(frame, [c], -1, (0,255,0), 2)
                        cv2.circle(frame, (cx,cy), 6, (0,0,255), -1)
                        cv2.putText(frame, f"Board: ({X:.3f},{Y:.3f}) m", (20,30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        else:
            cv2.putText(frame, "Need blob + visible checkerboard", (20,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        cv2.imshow("Homog+Blob", frame); cv2.imshow("Mask", mask); cv2.waitKey(1)

def main():
    rclpy.init(); rclpy.spin(HomogBlobMapperNode()); rclpy.shutdown()
if __name__ == '__main__':
    main()

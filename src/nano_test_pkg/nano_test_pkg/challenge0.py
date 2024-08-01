import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2

ARUCO_DICT = {
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    # ... (other dictionary entries remain the same)
}

class ArucoPoseEstimator(Node):

    def __init__(self):
        super().__init__('aruco_pose_estimator')
        
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(PoseStamped, '/pose_estimate', 10)
        
        self.br = CvBridge()
        
        self.aruco_type = "DICT_5X5_100"
        self.arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[self.aruco_type])
        self.arucoParams = cv2.aruco.DetectorParameters()
        
        self.intrinsic_camera = np.array(((2610.92318, 0, 682.632413),(0,2619.77522, 566.430569),(0,0,1)))
        self.distortion = np.array((-.380335290,11.8795390,-.0139544829, -.00494152599, -105.646053))
        self.marker_size = 0.266  # 2cm, adjust as needed

    def image_callback(self, msg):
        cv_image = self.br.imgmsg_to_cv2(msg, '8UC3')
        output, rvecs, tvecs = self.pose_estimation(cv_image, ARUCO_DICT[self.aruco_type], self.intrinsic_camera, self.distortion)
        
        if rvecs and tvecs:
            for rvec, tvec in zip(rvecs, tvecs):
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                
                pose_msg.pose.position.x = float(tvec[0]) 
                pose_msg.pose.position.y = float(tvec[1])
                pose_msg.pose.position.z = float(tvec[2])
                
                # Convert rotation vector to quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
                
                pose_msg.pose.orientation.x = quaternion[0] * 1.0
                pose_msg.pose.orientation.y = quaternion[1] * 1.0
                pose_msg.pose.orientation.z = quaternion[2] * 1.0
                pose_msg.pose.orientation.w = quaternion[3] * 1.0
                
                self.publisher.publish(pose_msg)

    def pose_estimation(self, frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters()

        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        corners, ids, rejected_img_points = detector.detectMarkers(gray)
        
        if len(corners) > 0:
            rvecs, tvecs, _ = self.my_estimatePoseSingleMarkers(corners, self.marker_size, matrix_coefficients, distortion_coefficients)
            
            for i in range(len(ids)):
                # Draw detected markers and axes for visualization
                cv2.aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvecs[i], tvecs[i], 0.01)
        else:
            rvecs, tvecs = [], []
        
        return frame, rvecs, tvecs

    def my_estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
           corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                  [marker_size / 2, marker_size / 2, 0],
                                  [marker_size / 2, -marker_size / 2, 0],
                                  [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        
        for c in corners:
            nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs, trash

    def rotation_matrix_to_quaternion(self, R):
        q = np.empty((4, ))
        t = np.trace(R)
        if t > 0:
            t = np.sqrt(t + 1.0)
            q[3] = 0.5 * t
            t = 0.5 / t
            q[0] = (R[2, 1] - R[1, 2]) * t
            q[1] = (R[0, 2] - R[2, 0]) * t
            q[2] = (R[1, 0] - R[0, 1]) * t
        else:
            i = 0
            if R[1, 1] > R[0, 0]:
                i = 1
            if R[2, 2] > R[i, i]:
                i = 2
            j = (i + 1) % 3
            k = (j + 1) % 3
            t = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0)
            q[i] = 0.5 * t
            t = 0.5 / t
            q[3] = (R[k, j] - R[j, k]) * t
            q[j] = (R[j, i] + R[i, j]) * t
            q[k] = (R[k, i] + R[i, k]) * t
        return q

def main(args=None):
    rclpy.init(args=args)
    aruco_pose_estimator = ArucoPoseEstimator()
    rclpy.spin(aruco_pose_estimator)
    aruco_pose_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
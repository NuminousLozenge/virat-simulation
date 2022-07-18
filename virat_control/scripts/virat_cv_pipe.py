#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import tf

from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry


class imageProcessor():
    """ imageProcessor obtains images from the camera attached to Virat
    contour detection is used to detect boundaries of the potholes
    detected contours consist of arrays of pixel coords (u, v)
    an inverse projection is performed on (u, v) to find (Xc, Yc, Zc)
    that is coordinates w.r.t to the camera.
    
    These coordinates are loaded into a PointCloud2 message and
    published. This point cloud is displayed in Rviz in ground frame
    by transforming the point cloud using the existing tf tree from
    map to camera_link
    """

    def __init__(self):
        
        self.image = None
        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.br = CvBridge()
        
        self.intrinsic_camera_array = None
        
        rospy.Subscriber("/virat/camera1/image_raw", Image, self.callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/virat/camera1/camera_info", CameraInfo, self.camera_info_callback)
        self.pub = rospy.Publisher("/virat/point_cloud", PointCloud2, queue_size = 3)
    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg) # convert image msg to cv2 for image manipulation
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        
    def camera_info_callback(self, msg):
        self.intrinsic_camera_array = msg.K
        
        
    def fn(self, u ,v):
        # Takes u v returns Xc Yc Zc (camera coordinates)
        
        # W = 800            # Width of sensor in pixels
        # H = 800            # Height of sensor in pixels
        # f = 476.703084     # focal length in pixels
        h = 1.18           # height of camera from ground (in meters) (assume constant) (cam_z)
        d = 0.73           # cam_x
        # a = f
        # u0 = W/2
        # v0 = H/2
        
        # intrinsic camera matrix
        # K = np.array([[a, 0, u0],
        #               [0, a, v0],
        #               [0, 0, 1.0]])
                                      
        K = np.reshape(self.intrinsic_camera_array, (3,3))           
        Kinv = np.linalg.inv(K)
        
        # rotation matrix
        roll = 0
        pitch = -0.45
        yaw = 0
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        rotation_ground_to_cam = np.array([[cr*cy+sp*sr+sy, cr*sp*sy-cy*sr, -cp*sy],
                                         [cp*sr, cp*cr, sp],
                                         [cr*sy-cy*sp*sr, -cr*cy*sp -sr*sy, cp*cy]])
                                         
        rotation_cam_to_ground = rotation_ground_to_cam.T # inv of rotation mat is same as its transpose
        translate_cam_to_ground = np.array([0, -h, 0])
        
        n = np.array([0, 1, 0])
        ground_normal_to_cam = (rotation_cam_to_ground.T).dot(n)
        
        # Converts the pixel coordinates (u,v) to coordinates w.r.t camera (Xc, Yc, Zc)
        uv_hom = np.array([u, v, 1])
        Kinv_uv = Kinv.dot(uv_hom)
        denominator = ground_normal_to_cam.dot(Kinv_uv)
        uv_to_cam_frame = h*Kinv_uv/denominator
        
        # cam_frame_to_ground = rotation_cam_to_ground.dot(uv_to_cam_frame) + translate_cam_to_ground
        
        return uv_to_cam_frame
        
    def generate_points(self, cnts):
        points = []
        color = rgb_to_uint32(r=255, g=255, b=255, a=255) # set r g b a values for colour of point cloud
        for c in cnts:
            for p in c:
                coord = self.fn(p[0][0], p[0][1])
                points.append((coord[2], -coord[0], -coord[1]-self.z, color))       #[Z, -X, -Y]
        return points
            
    def process(self):
       fields = [PointField('x', 0, PointField.FLOAT32, 1),
                 PointField('y', 4, PointField.FLOAT32, 1),
                 PointField('z', 8, PointField.FLOAT32, 1),
                 PointField('rgb', 16, PointField.UINT32, 1),
                ]
       header = Header()
       header.frame_id = "camera"
           
       while not rospy.is_shutdown(): 
           if self.image is not None:
               cnts = contour_detector(self.image)
               points = self.generate_points(cnts)
               header.stamp = rospy.Time.now()
               cloud = point_cloud2.create_cloud(header, fields, points)
               self.pub.publish(cloud)            
           rospy.Rate(10).sleep()

def contour_detector(image):
    # Load image, grayscale, Gaussian blur, Otsu's threshold
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3,3), 0)
    thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

    # Dilate with elliptical shaped kernel
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    dilate = cv2.dilate(thresh, kernel, iterations=2)

    # Find contours
    cnts = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1] 
    return cnts
    
def rgb_to_uint32(r=255, g=255, b=255, a=255):
    # return format 0xAaRrGgBb
    return a << 24 | r << 16 | g << 8 | b
    
       
rospy.init_node("virat_img_processor", anonymous=True)
my_processor = imageProcessor()
my_processor.process()     
     
           
                      

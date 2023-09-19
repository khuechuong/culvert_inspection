#!/usr/bin/env python3
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
# from tf2_py import tf2
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

from ultralytics import YOLO
import numpy as np
import math
import cv2

class defect_detection:
    def __init__(self):
        self.bridge = CvBridge()
        self.cloud_map = None
        self.cloud_header = None
        self.rgb_map = None
        self.rgb_seg_map = None
        self.xyz_array = None

        self.rgb_received = False
        self.cloud_received = False

        self.rate = rospy.Rate(10)  # 10hz
        self.rgb_sub = rospy.Subscriber("/zed/zed_nodelet/rgb/image_rect_color", Image, self.rgbCallBack)
        self.cloud_sub = rospy.Subscriber("/zed/zed_nodelet/point_cloud/cloud_registered", PointCloud2, self.cloudCallBack)
        # self.odom_sub = rospy.Subscriber("/zed/zed_nodelet/odom", Odometry, self.odomCallBack)
        self.vis_pub = rospy.Publisher("visualization_marker", MarkerArray, queue_size=1)
        self.marker_ar = MarkerArray()
        self.timer = rospy.Timer(rospy.Duration(1.0/10.0), self.markerCb)
        self.list_crack = []
        self.list_spall = []
        self.index = 0
        model_path = "/home/ara/catkin_ws/src/detection/model/best.pt"
        print('load model')
        self.model = YOLO(model_path)

        self.tf_buffer = tf2_ros.Buffer()
        self.t1= tf2_ros.TransformListener(self.tf_buffer)

        # wait for message before doing anything
        rospy.wait_for_message("/zed/zed_nodelet/rgb/image_rect_color", Image)
        rospy.wait_for_message("/zed/zed_nodelet/point_cloud/cloud_registered", PointCloud2)

    def rgbCallBack(self, data):
        try:
            self.rgb_map = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.rgb_seg_header = data.header
            self.rgb_received = True
        except CvBridgeError as e:
            print(e)
    
    def cloudCallBack(self,data):    
        # print(type(self.tf_buffer))
        # try:
        # except tf2.LookupException as ex:
        #     rospy.logwarn(ex)
        #     return
        # except tf2.ExtrapolationException as ex:
        #     rospy.logwarn(ex)
        #     return
        
        self.xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
        self.cloud_map = data
        # self.cloud_header = data.header
        self.cloud_received = True

    def markerCb(self, event=None):
        if self.cloud_received and self.rgb_received:
            xyz_copy = self.xyz_array.copy()
            list_0 = []
            list_1 = []
            # get detection
            results = self.model(source=self.rgb_map)
            for box in results[0]:            
                if box.boxes.conf.item() >= 0.5:        
                    loc = box.boxes.xywh.cpu()
                    loc = loc.numpy()
                    # x, y, w, h           
                    loc[0][0] = int(loc[0][0])
                    loc[0][1] = int(loc[0][1])
                    loc[0][2] = int(loc[0][2])
                    loc[0][3] = int(loc[0][3])
                    #  spalls
                    if box.boxes.cls.item() == 1.0:
                        list_1.append(loc.flatten().astype('int32'))
                    # cracks
                    elif box.boxes.cls.item() == 0.0:
                        list_0.append(loc.flatten().astype('int32'))
            # get 4 corner (x,y)
            for crack in list_0:
                c0,c1,c2,c3 = self.boundingbox(crack)
                # cv2.rectangle(self.rgb_map, c0, c2, color=(0,0,255), thickness=2)
                self.bboxMarker([c0,c1,c2,c3], xyz_copy, self.cloud_map.header, True)

            
            for spall in list_1:
                c0,c1,c2,c3 = self.boundingbox(spall)
                # cv2.rectangle(self.rgb_map, c0, c2, color=(0,255,0), thickness=2)
                self.bboxMarker([c0,c1,c2,c3], xyz_copy, self.cloud_map.header, False)
            
            self.vis_pub.publish(self.marker_ar)
            # cv2.imshow("Image window", self.rgb_map)
            # cv2.waitKey(3)
            # match 4 corner with point cloud xyz

    def bboxMarker(self, coord, xyz_copy, header, crack):
        line_strip = Marker()
        line_strip.header.frame_id= "map"
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = Marker.ADD
        line_strip.pose.orientation.w = 1.0
        line_strip.id = self.index
        line_strip.scale.x = 0.01
        if crack:
            line_strip.color.r = 1.0
        else:
            line_strip.color.g = 1.0
        line_strip.color.a = 1.0
        for c in coord:
            if self.isRealNumber(xyz_copy[c[1]][c[0]][0]) and self.isRealNumber(xyz_copy[c[1]][c[0]][1]) and self.isRealNumber(xyz_copy[c[1]][c[0]][2]):
                p = Point()
                p.x = xyz_copy[c[1]][c[0]][0] #+ 0.13
                if xyz_copy[c[1]][c[0]][1] < 0:                    
                    if crack:
                        p.y = xyz_copy[c[1]][c[0]][1] #+ 0.08
                        p.z = xyz_copy[c[1]][c[0]][2] #+ 0.47                        
                    else:
                        p.y = xyz_copy[c[1]][c[0]][1] #+ 0.06
                        p.z = xyz_copy[c[1]][c[0]][2] #+ 0.55                    
                else:
                    p.y = xyz_copy[c[1]][c[0]][1]
                    p.z = xyz_copy[c[1]][c[0]][2] #+ 0.52    
                
                trans = self.tf_buffer.lookup_transform("map", "zed_left_camera_frame",
                                    rospy.get_rostime() - rospy.Duration(0.1),
                                    rospy.Duration(0.1))
                # for bag
                # trans = self.tf_buffer.lookup_transform("odom", "zed_left_camera_frame",
                #                     header.stamp,
                #                     rospy.Duration(0.1))
                point_stamp = PointStamped()
                point_stamp.point = p
                point_stamp.header = header
                p_tf = do_transform_point(point_stamp, trans)                
                line_strip.points.append(p_tf.point)
            else:
                return
        line_strip.points.append(line_strip.points[0])
        for mark in self.marker_ar.markers:
            # if same type (crack or spall)
            if line_strip.color.r == mark.color.r:
                dist = math.sqrt( (line_strip.points[0].y - mark.points[0].y)**2 )
                if dist < 1.0:
                    return
        self.marker_ar.markers.append(line_strip)
        self.index = self.index + 1

    def boundingbox(self, loc):
        [x,y,w,h] = loc
        x0 = int(x - w/2)
        y0 = int(y - h/2)
        x1 = int(x - w/2)
        y1 = int(y + h/2)
        x2 = int(x + w/2)
        y2 = int(y + h/2)
        x3 = int(x + w/2)
        y3 = int(y - h/2)
        return (x0,y0), (x1,y1), (x2,y2), (x3,y3)
    
    def isRealNumber(self, number):
        if not np.isnan(number) and not np.isinf(number):
            return True
        return False
if __name__ == '__main__':
    print('START')
    rospy.init_node('detection')
    print('Initialize Node')
    x=defect_detection()
    rospy.spin()


        # img = cv2.imread("/home/ara/Downloads/img_0025.jpg")
        # results = self.model(source=img)

        # for box in results[0]:            
        #     if box.boxes.conf.item() >= 0.5:        
        #         loc = box.boxes.xywh.cpu()
        #         loc = loc.numpy()
                        
        #         loc[0][0] = int(loc[0][0])
        #         loc[0][1] = int(loc[0][1])
        #         loc[0][2] = int(loc[0][2])
        #         loc[0][3] = int(loc[0][3])
                    
        #         if box.boxes.cls.item() == 1.0:
        #             list_1.append(loc.flatten().astype('int32'))

        #         elif box.boxes.cls.item() == 0.0:
        #             list_0.append(loc.flatten().astype('int32'))


        # array_0 = np.array(list_0)
        # array_1 = np.array(list_1)

        # print("Cracks: " + str(len(list_0)))
        # print(array_0)
        # print("Spalls: " + str(len(list_1)))
        # print(array_1)




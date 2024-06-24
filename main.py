#!/usr/bin/env python
# -*- coding: utf-8 -*-

# rospy, OpenCV, bridge import 
import rospy
import cv2
from cv_bridge import CvBridge

# 메시지 타입 import
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import String
from std_msgs.msg import Int64
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers

# 모듈에서 클래스 및 함수 import
from single_stop import *
from tunnel import *
from hough import *
from ar_judgement import *
from start_hough import *  # start_hough 모듈에서 클래스 임포트

class Project:
    def __init__(self):
        rospy.init_node('auto_driver')

        # Subscriber 정의
        rospy.Subscriber('/Single_color', String, self.single_callback)
        rospy.Subscriber('/Left_color', String, self.left_callback)
        rospy.Subscriber('/Right_color', String, self.right_callback)
        rospy.Subscriber('/time_count', Int64, self.time_callback)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_callback)

        # motor Publisher 정의
        self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

        # 20Hz 제어
        self.rate = rospy.Rate(20)

        # ROS bridge 생성
        self.bridge = CvBridge()

        # Subscribe 데이터 초기화
        self.image = None
        self.single = None
        self.left = None
        self.right = None
        self.time = None
        self.lidar_points = None

        # AR 데이터 초기화
        self.arData_id = None
        self.arData_x = 0.
        self.arData_y = 0.

        # xycar 모터 값 초기화
        speed = 0
        angle = 0
        self.motor_msg = xycar_motor()
        self.motor_msg.speed = 0
        self.motor_msg.angle = 0

        # 각 구간에 대한 Solver 생성
        self.start_hough = StartHough()
        self.tunnel = Tunnel()
        self.hough = Hough()
        self.hough_left = Hough_left()
        self.hough_right = Hough_right()

        # 탈출 조건 및 진입 조건 flag 생성
        self.start_flag = 0
        self.tunnel_flag = 0

        # 갈림길 구간 진행 방향
        self.direction = "G"

        while not rospy.is_shutdown():
            # 데이터 정상 수신 여부 확인
            if (self.image is None or self.single is None or self.time is None or
                self.lidar_points is None or self.left is None or self.right is None):
                continue

            #v초기에 start_hough 실행
            if self.start_flag == 0:
                self.start_hough.run()
                self.start_flag = 1
            # 터널 구간 진입 및 터널 주행
            if self.tunnel_flag == 0:
                # escape 함수를 이용해 터널 구간에서의 조향각 계산, 탈출 여부 확인
                angle, self.tunnel_flag = self.tunnel.escape(self.lidar_points)
                speed = 4

                self.motor_msg.angle = int(angle)
                self.motor_msg.speed = speed
                self.motor_pub.publish(self.motor_msg)
                self.rate.sleep()
                continue

            # 터널 탈출 후 정지선 주행(single)
            if self.direction == 'G':
                # 왼쪽, 오른쪽 주행 판단
                self.direction = left_right_judge(self.arData_x, self.arData_y, self.arData_id,self.left, self.right, self.time)
                # 속도(정지선 검출), 조향각(hough line 주행) 판단
                angle = self.hough.line_trace(self.image)
                speed = detect_stop_line(self.image, self.single)
            else:
                # 거리 값을 이용해 주차 여부 확인
                if parking_check(self.arData_x, self.arData_y, self.arData_id):
                    # 주차 조건에 부합하면, while문을 탈출하여 프로그램 종료
                    self.motor_msg.angle = 0
                    self.motor_msg.speed = 0
                    self.motor_pub.publish(self.motor_msg)
                    break

                # 왼쪽 차선 주행
                if self.direction == 'L':
                    # 속도(정지선 검출), 조향각(hough line 주행) 판단
                    angle = self.hough_left.line_trace(self.image)
                    speed = detect_stop_line(self.image, self.left)
                # 오른쪽 차선 주행
                elif self.direction == 'R':
                    # 속도(정지선 검출), 조향각(hough line 주행) 판단
                    angle = self.hough_right.line_trace(self.image)
                    speed = detect_stop_line(self.image, self.right)

            # 최종 조향 및 스피드 인가
            self.motor_msg.angle = int(angle)
            self.motor_msg.speed = speed

            # 모터 명령 Publish
            self.motor_pub.publish(self.motor_msg)

            # 제어주기 동기화
            self.rate.sleep()

    # 이미지 callback 함수
    def img_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    # single 신호 callback 함수
    def single_callback(self, data):
        self.single = data.data

    # left 신호 callback 함수
    def left_callback(self, data):
        self.left = data.data

    # right 신호 callback 함수
    def right_callback(self, data):
        self.right = data.data

    # time callback 함수
    def time_callback(self, data):
        self.time = data.data

    # lidar data callback 함수
    def lidar_callback(self, data):
        self.lidar_points = data.ranges

    # ar data callback 함수
    def ar_callback(self, data):
        for i in data.markers:
            self.arData_id = i.id
            self.arData_x = i.pose.pose.position.x
            self.arData_y = i.pose.pose.position.z

if __name__ == "__main__":
    project = Project()
    # ROS callback이 제대로 호출되도록 spin 추가
    rospy.spin()

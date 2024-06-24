#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 필요한 라이브러리 import
import cv2, time, math
import rospy
import numpy as np

# 메시지 타입 import
from sensor_msgs.msg import Image
from xycar_msgs.msg import xycar_motor
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers

# 튜닝 파라미터
LOW_RANGE = 0
HIGH_RANGE = 80*2
MIN_DIST = 0.27
W = 0.29
L = 0.08
WEIGHT = 0.9
P, I, D = 0.6, 0.0006, 0.2

# 초기 허프라인 차선 주행을 위한 Solver
class StartHough:
    def __init__(self):
        # CvBridge 초기화 및 멤버 변수 설정
        self.bridge = CvBridge()
        self.image = None
        self.lidar_points = None

        # 퍼블리셔 선언
        self.motor_msg = xycar_motor()
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

        # AR 데이터 초기화
        self.arData_id = 0
        self.arData_x = 0.
        self.arData_y = 0.

        # 이미지 변수 초기화 및 선언
        self.WIDTH, self.HEIGHT = 640, 480
        self.ROI_ROW = 230
        self.ROI_HEIGHT = self.HEIGHT - self.ROI_ROW
        self.L_ROW = self.ROI_HEIGHT - 120

        # 섭스크라이버 선언
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_callback)

        # PID 컨트롤러 선언
        self.pid = self.PID(P, I, D)
        self.check = 0

        # 주행을 위한 left, right 픽셀에 대해 초기화
        self.prev_x_left = 0
        self.prev_x_right = self.WIDTH

    # PID 컨트롤러 클래스
    class PID():
        def __init__(self, kp, ki, kd):
            self.Kp = kp
            self.Ki = ki
            self.Kd = kd
            self.p_error = 0.0
            self.i_error = 0.0
            self.d_error = 0.0

        def TotalError(self, cte):
            self.d_error = cte - self.p_error
            self.p_error = cte
            self.i_error += cte
            return self.Kp * self.p_error + self.Ki * self.i_error + self.Kd * self.d_error

    # 이미지 callback 함수
    def img_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    # ar data callback 함수
    def ar_callback(self, data):
        for i in data.markers:
            self.arData_id = i.id
            self.arData_x = i.pose.pose.position.x
            self.arData_y = i.pose.pose.position.z

    # lidar data callback 함수
    def lidar_callback(self, data):
        self.lidar_points = data.ranges

    # 장애물의 회피 여부를 판단하여 기구학적으로 회피 조향각을 계산하는 함수
    def avoid(self):
        # 좌/우측 라이다 값을 Slicing
        right_lidar = self.lidar_points[180+LOW_RANGE:180+HIGH_RANGE]
        left_lidar = self.lidar_points[180-HIGH_RANGE:180-LOW_RANGE]

        # inf 데이터에 대해서 10.0으로 조정
        right_obstacle_distance = [10.0 if (r_dist is None or math.isinf(r_dist)) else r_dist for r_dist in right_lidar]
        left_obstacle_distance = [10.0 if (l_dist is None or math.isinf(l_dist)) else l_dist for l_dist in left_lidar]
        
        # 좌/우측 최소 거리 측정
        r_min = min(right_obstacle_distance)
        l_min = min(left_obstacle_distance)
        # 좌/우측 최소 거리에 해당하는 인덱스(각도)를 계산
        r_index = right_obstacle_distance.index(r_min)
        l_index = left_obstacle_distance.index(l_min)

        # 오른쪽 주행 영역에 대해서 장애물이 있을 시, 기구학적 연산을 통해 회피 조향각 계산
        if W/2+L>math.sin(math.radians((r_index+LOW_RANGE)/2))*r_min and r_min <MIN_DIST:
            r_min=max(r_min,W/2+L)
            self.motor_msg.angle = WEIGHT*(math.degrees(math.acos((W/2+L)/r_min))-float((90-((r_index+LOW_RANGE)/2))))
            self.motor_msg.speed = 5
            self.pub.publish(self.motor_msg)
            time.sleep(0.3)
        # 왼쪽 주행 영역에 대해서 장애물이 있을 시, 기구학적 연산을 통해 회피 조향각 계산
        if W/2+L>math.cos(math.radians((l_index+HIGH_RANGE)/2))*l_min and l_min < MIN_DIST:
            l_min=max(l_min,W/2+L)
            self.motor_msg.angle = WEIGHT*math.degrees(float(((l_index+LOW_RANGE)/2))-math.acos((W/2+L)/l_min))
            self.motor_msg.speed = 5
            self.pub.publish(self.motor_msg)
            time.sleep(0.3)

    def run(self):
        rate = rospy.Rate(10)   # 제어주기-10Hz
        stop_flag = False       # stop_flag 초기화
        while not rospy.is_shutdown():
            # 데이터 정상 수신 여부 확인
            if self.image is None or self.lidar_points is None:
                continue

            # 이미지 처리
            img = self.image.copy()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)
            edge_img = cv2.Canny(np.uint8(blur_gray), 60, 180)
            roi_edge_img = edge_img[self.ROI_ROW:self.HEIGHT, 0:self.WIDTH]
            all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi/180, 50, 80, 20)

            # 터널 진입조건 확인
            # 차량 전방 기준 좌/우의 특정 범위에 해당하는 영역을 검출
            mid_p = self.lidar_points[120:240]
            mid_distances = [dist for dist in mid_p if dist > 0 and not math.isinf(dist)]
            
            # mid points에 대해 평균 거리를 계산
            if len(mid_distances) > 0:
                avg_m = sum(mid_distances) / len(mid_distances)
            else:
                avg_m = 0.1
            
            # 터널 AR태그를 인식하고, 평균거리가 0.6보다 작게될 경우 프로그램 종료
            if stop_flag and avg_m < 0.6:
                self.motor_msg.angle = 0
                self.pub.publish(self.motor_msg)
                break

            # 차선이 보이지 않을 시 continue
            if all_lines is None:
                continue

            # 차선의 기울기와 좌표 리스트 초기화
            slopes = []
            filtered_lines = []

            # 모든 라인에 대해서 필터링
            for line in all_lines:
                x1, y1, x2, y2 = line[0]
                if x2 == x1:    # 수직선의 경우 기울기를 큰 값으로 설정
                    slope = 1000.0
                else:
                    slope = float(y2 - y1) / float(x2 - x1) # 기울기 계산
                if 0.2 < abs(slope):    # 기울기가 일정 이상이면 저장
                    slopes.append(slope)
                    filtered_lines.append(line[0])

            # 오른쪽, 왼쪽 차선 리스트 초기화
            left_lines = []
            right_lines = []

            # 차선 필터링
            for j in range(len(slopes)):
                Line = filtered_lines[j]
                slope = slopes[j]
                x1, y1, x2, y2 = Line
                # 왼쪽 차선 필터링
                if (slope < 0) and (x2 < self.WIDTH / 2):
                    left_lines.append(Line.tolist())
                # 오른쪽 차선 필터링
                elif (slope > 0) and (x1 > self.WIDTH / 2):
                    right_lines.append(Line.tolist())

            # 왼쪽 차선에 대해, 대표 직선 방정식의 기울기와 편향 초기화
            m_left, b_left = 0., 0.
            x_sum, y_sum, m_sum = 0., 0., 0.

            size = len(left_lines)
            # 왼쪽 대표 직선의 방정식의 편향과 기울기 계산
            if size != 0:
                for line in left_lines:
                    x1, y1, x2, y2 = line
                    x_sum += x1 + x2
                    y_sum += y1 + y2
                    if x2 != x1:
                        m_sum += float(y2 - y1) / float(x2 - x1)
                    else:
                        m_sum += 0
                x_avg = x_sum / (size * 2)
                y_avg = y_sum / (size * 2)
                m_left = m_sum / size
                b_left = y_avg - m_left * x_avg

            # 오른쪽 차선에 대해, 대표 직선 방정식의 기울기와 편향 초기화
            m_right, b_right = 0., 0.
            x_sum, y_sum, m_sum = 0., 0., 0.

            size = len(right_lines)
            # 오른쪽 대표 직선의 방정식의 편향과 기울기 계산
            if size != 0:
                for line in right_lines:
                    x1, y1, x2, y2 = line
                    x_sum += x1 + x2
                    y_sum += y1 + y2
                    if x2 != x1:
                        m_sum += float(y2 - y1) / float(x2 - x1)
                    else:
                        m_sum += 0
                x_avg = x_sum / (size * 2)
                y_avg = y_sum / (size * 2)
                m_right = m_sum / size
                b_right = y_avg - m_right * x_avg

            # 기울기가 0일 시 이전의 값 사용(left)
            if m_left == 0.:
                x_left = self.prev_x_left
            # ROI 영역에 해당하는 x좌표 계산(left)
            else:
                x_left = int((self.L_ROW - b_left) / m_left)
                self.prev_x_left = x_left

            # 기울기가 0일 시 이전의 값 사용(right)
            if m_right == 0.0:
                x_right = self.prev_x_right
            # ROI 영역에 해당하는 x좌표 계산(right)
            else:
                x_right = int((self.L_ROW - b_right) / m_right)
                self.prev_x_right = x_right

            # 차선의 검출 여부와 터널 AR태그(stop_flag)인식 여부 판단
            if (len(left_lines) == 0 and len(right_lines) == 0) and not stop_flag:
                # self.check 스택을 사용하여 No line판단
                self.check += 1
                # 연속 두 번 No line이라고 판단하게 될 경우 아래의 코드 실행
                if self.check == 2:
                    # 왼쪽으로 조향명령
                    onelineangle = -48
                    self.motor_msg.angle = int(onelineangle)
                    # 속도를 낮추어 주행
                    for _ in range(3):
                        self.motor_msg.speed = 3
                        self.pub.publish(self.motor_msg)
                        time.sleep(0.5)
                    continue
            else:
                self.check = 0  # 스택 초기화

            # 왼쪽과 오른쪽에 해당하는 x좌표의 평균을 이용해 조향각 계산
            x_midpoint = (x_left + x_right) // 2
            view_center = self.WIDTH // 2
            # PID컨트롤러를 이용해 최종 조향각 도출
            error = x_midpoint - view_center
            angle = self.pid.TotalError(error)

            # 조향각과 속도 조정
            self.motor_msg.angle = int(angle)
            self.motor_msg.speed = 4

            # 장애물 회피를 위한 조향각 계산
            if not stop_flag:
                self.avoid()

            # 터널 AR태그를 특정 거리 미만에서 인식하게 될 경우를 판단
            if self.arData_id == 6 and (0.1 < math.sqrt(math.pow(float(self.arData_x), 2) + math.pow(float(self.arData_y), 2)) < 1.2) and not stop_flag:
                stop_flag = True    # stop_flag를 통해서 터널 진입 조건 확인

            rate.sleep()
            
            # 조향명령 인가
            self.pub.publish(self.motor_msg)

       


#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 필요한 라이브러리 임포트
import cv2, math
import numpy as np

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

# 카메라 및 이미지 설정
CAM_FPS = 30
WIDTH, HEIGHT = 640, 480
ROI_ROW = 230
ROI_HEIGHT = HEIGHT - ROI_ROW
L_ROW = ROI_HEIGHT - 120
S_P, S_I, S_D = 0.4, 0.0005, 0.5
P, I, D = 0.5, 0.0005, 0.2

# 왼쪽 차선 인식 주행 Solver
class Hough_left():
    def __init__(self):
        # PID 컨트롤러 생성
        self.pid = PID(S_P, S_I, S_D)

        # 좌우측 차선 픽셀 초기화
        self.prev_x_left = 0
        self.prev_x_right = WIDTH

    # 이미지를 입력으로 조향각 계산
    def line_trace(self, img):
        # 조향각 초기화
        angle = 0

        # 이미지 처리
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edge_img = cv2.Canny(np.uint8(blur_gray), 60, 180)
        roi_edge_img = edge_img[ROI_ROW:HEIGHT, 0:WIDTH]
        all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi/180, 50, minLineLength=80, maxLineGap=20)

        # 차선이 보이지 않을 시 조향각 0
        if all_lines is None:
            return 0

        # 차선의 기울기와 좌표 리스트 초기화
        slopes = []
        filtered_lines = []

        # 모든 라인에 대해서 필터링
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:  # 수직선의 경우 기울기를 큰 값으로 설정
                slope = 1000.0
            else:
                slope = float(y2 - y1) / float(x2 - x1)  # 기울기 계산
            if 0.2 < abs(slope):  # 기울기가 일정 이상이면 저장
                slopes.append(slope)
                filtered_lines.append(line[0])

        # 왼쪽 차선 리스트 초기화
        left_lines = []

        # 왼쪽 차선 필터링
        for j in range(len(slopes)):
            Line = filtered_lines[j]
            slope = slopes[j]
            x1, y1, x2, y2 = Line
            # 기울기가 0보다 작고, 화면의 폭의 1/3보다 작은 영역에 있을 시 왼쪽 차선으로 인식
            if (slope < 0) and (x2 < WIDTH // 3):
                left_lines.append(Line.tolist())

        # 왼쪽 차선에 대해, 대표 직선 방정식의 기울기와 편향 초기화
        m_left, b_left = 0., 0.

        size_left = len(left_lines)
        # 대표 직선의 방정식의 편향과 기울기 계산
        if size_left != 0:
            x_sum, y_sum, m_sum = 0., 0., 0.
            for line in left_lines:
                x1, y1, x2, y2 = line
                x_sum += x1 + x2
                y_sum += y1 + y2
                if x2 != x1:
                    m_sum += float(y2 - y1) / float(x2 - x1)
                else:
                    m_sum += 0
            x_avg = x_sum / (size_left * 2)
            y_avg = y_sum / (size_left * 2)
            m_left = m_sum / size_left
            b_left = y_avg - m_left * x_avg

        if size_left == 0:
            return 0  # 라인이 검출되지 않은 경우

        # 기울기가 0일 시 이전의 값 사용
        if m_left == 0.:
            x_left = self.prev_x_left
        # ROI 영역에 해당하는 x좌표 계산
        else:
            x_left = int((L_ROW - b_left) / m_left)
            self.prev_x_left = x_left

        # 왼쪽 좌표에 대해 하한값 조절
        x_left=max(x_left,-50.)
        # 조향각 계산(오른쪽 x좌표를 520으로 설정)
        x_midpoint = (x_left + 520) // 2
        view_center = WIDTH // 2
        error = x_midpoint - view_center
        # PID컨트롤러를 이용해 최종 조향각 도출
        angle = self.pid.TotalError(error)
        angle = int(angle)

        return angle

# 오른쪽 차선주행 Solver
class Hough_right():
    def __init__(self):
        # PID 컨트롤러 생성
        self.pid = PID(S_P, S_I, S_D)

        # 좌우측 차선 픽셀 초기화
        self.prev_x_left = 0
        self.prev_x_right = WIDTH

    # 이미지를 입력으로 조향각 계산
    def line_trace(self, img):
        # 조향각 초기화
        angle = 0

        # 이미지 처리
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edge_img = cv2.Canny(np.uint8(blur_gray), 60, 180)
        roi_edge_img = edge_img[ROI_ROW:HEIGHT, 0:WIDTH]
        all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi/180, 50, minLineLength=80, maxLineGap=20)

        # 차선이 보이지 않을 시 조향각 0
        if all_lines is None:
            return 0

        # 차선의 기울기와 좌표 리스트 초기화
        slopes = []
        filtered_lines = []

        # 모든 라인에 대해서 필터링
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:  # 수직선의 경우 기울기를 큰 값으로 설정
                slope = 1000.0
            else:
                slope = float(y2 - y1) / float(x2 - x1)  # 기울기 계산
            if 0.2 < abs(slope):  # 기울기가 일정 이상이면 저장
                slopes.append(slope)
                filtered_lines.append(line[0])

        # 오른쪽 차선 리스트 초기화
        right_lines = []

        # 오른쪽 차선 필터링
        for j in range(len(slopes)):
            Line = filtered_lines[j]
            slope = slopes[j]
            x1, y1, x2, y2 = Line
            # 기울기가 0보다 크고, 화면의 폭의 2/3보다 큰 영역에 있을 시 오른쪽 차선으로 인식
            if (slope > 0) and (x1 > (WIDTH * 2)// 3):
                right_lines.append(Line.tolist())

        # 오른쪽 차선에 대해, 대표 직선 방정식의 기울기와 편향 초기화
        m_right, b_right = 0., 0.
        x_sum, y_sum, m_sum = 0., 0., 0.

        size_right = len(right_lines)
        # 대표 직선의 방정식의 편향과 기울기 계산
        if size_right != 0:
            for line in right_lines:
                x1, y1, x2, y2 = line
                x_sum += x1 + x2
                y_sum += y1 + y2
                if x2 != x1:
                    m_sum += float(y2 - y1) / float(x2 - x1)
                else:
                    m_sum += 0
            x_avg = x_sum / (size_right * 2)
            y_avg = y_sum / (size_right * 2)
            m_right = m_sum / size_right
            b_right = y_avg - m_right * x_avg

        if size_right == 0:
            return 0  # 라인이 검출되지 않은 경우

        # 기울기가 0일 시 이전의 값 사용
        if m_right == 0.:
            x_right = self.prev_x_right
        # ROI 영역에 해당하는 x좌표 계산
        else:
            x_right = int((L_ROW - b_right) / m_right)
            self.prev_x_right = x_right

        # 오른쪽 좌표에 대해 상한값 조절
        x_right=min(x_right,690.)
        # 조향각 계산(왼쪽 x좌표를 100으로 설정)
        x_midpoint = (x_right + 100) // 2
        view_center = WIDTH // 2
        # PID컨트롤러를 이용해 최종 조향각 도출
        error = x_midpoint - view_center
        angle = self.pid.TotalError(error)
        angle = int(angle)

        return angle

# 일반적인 차선주행 Solver
class Hough():
    def __init__(self):
        # PID 컨트롤러 생성
        self.pid = PID(P, I, D)

        # 좌우측 차선 픽셀 초기화
        self.prev_x_left = 0
        self.prev_x_right = WIDTH

    # 이미지를 입력으로 조향각 계산
    def line_trace(self, img):
        # 조향각 초기화
        angle = 0

        # 이미지 처리
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edge_img = cv2.Canny(np.uint8(blur_gray), 60, 180)
        roi_edge_img = edge_img[ROI_ROW:HEIGHT, 0:WIDTH]
        all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi/180, 50, minLineLength=80, maxLineGap=20)

        # 차선이 보이지 않을 시 조향각 0
        if all_lines is None:
            return 0

        # 차선의 기울기와 좌표 리스트 초기화
        slopes = []
        filtered_lines = []

        # 모든 라인에 대해서 필터링
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:  # 수직선의 경우 기울기를 큰 값으로 설정
                slope = 1000.0
            else:
                slope = float(y2 - y1) / float(x2 - x1)  # 기울기 계산
            if 0.2 < abs(slope):  # 기울기가 일정 이상이면 저장
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
            if (slope < 0) and (x2 < WIDTH / 2):
                left_lines.append(Line.tolist())
            # 오른쪽 차선 필터링
            elif (slope > 0) and (x1 > WIDTH / 2):
                right_lines.append(Line.tolist())

        # 오른쪽, 왼쪽 차선에 대해, 대표 직선 방정식의 기울기와 편향 초기화
        m_left, b_left = 0., 0.
        m_right, b_right = 0., 0.
        x_sum, y_sum, m_sum = 0., 0., 0.

        size_right = len(right_lines)
        # 오른쪽 대표 직선의 방정식의 편향과 기울기 계산
        if size_right != 0:
            for line in right_lines:
                x1, y1, x2, y2 = line
                x_sum += x1 + x2
                y_sum += y1 + y2
                if x2 != x1:
                    m_sum += float(y2 - y1) / float(x2 - x1)
                else:
                    m_sum += 0
            x_avg = x_sum / (size_right * 2)
            y_avg = y_sum / (size_right * 2)
            m_right = m_sum / size_right
            b_right = y_avg - m_right * x_avg

        size_left = len(left_lines)
        # 왼쪽 대표 직선의 방정식의 편향과 기울기 계산
        if size_left != 0:
            x_sum, y_sum, m_sum = 0., 0., 0.
            for line in left_lines:
                x1, y1, x2, y2 = line
                x_sum += x1 + x2
                y_sum += y1 + y2
                if x2 != x1:
                    m_sum += float(y2 - y1) / float(x2 - x1)
                else:
                    m_sum += 0
            x_avg = x_sum / (size_left * 2)
            y_avg = y_sum / (size_left * 2)
            m_left = m_sum / size_left
            b_left = y_avg - m_left * x_avg

        if size_left == 0 and size_right == 0:
            return 0  # 라인이 검출되지 않은 경우

        # 기울기가 0일 시 이전의 값 사용(left)
        if m_left == 0.:
            x_left = self.prev_x_left
        # ROI 영역에 해당하는 x좌표 계산(left)
        else:
            x_left = int((L_ROW - b_left) / m_left)
            self.prev_x_left = x_left

        # 기울기가 0일 시 이전의 값 사용(right)
        if m_right == 0.:
            x_right = self.prev_x_right
        # ROI 영역에 해당하는 x좌표 계산(right)
        else:
            x_right = int((L_ROW - b_right) / m_right)
            self.prev_x_right = x_right

        # 왼쪽과 오른쪽에 해당하는 x좌표의 평균을 이용해 조향각 계산
        x_midpoint = (x_left + x_right) // 2
        view_center = WIDTH // 2
        # PID컨트롤러를 이용해 최종 조향각 도출
        error = x_midpoint - view_center
        angle = self.pid.TotalError(error)
        angle = int(angle)

        return angle



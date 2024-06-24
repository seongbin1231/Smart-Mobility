#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

# 튜닝 파라미터
LOW_SCAN, HIGH_SCAN = 180, 300
DIS = 0.25
P, I, D = 1600, 0, 600

# 터널을 탈출하기 위한 Solver 클래스
class Tunnel():
    def __init__(self):
        self.pid = PID(P,I,D)   # 터널 탈출 PID

    # 터널 탈출 함수, 라이다 포인트 값을 이용해 터널 탈출을 위한 조향각 계산 및 터널 탈출 조건 확인
    def escape(self, lidar_points):
        angle = 0   # 각도 초기화
        flag = 0    # 터널 탈출 조건 초기화

        # 라이다 범위를 오른쪽으로 Slicing
        right_points = lidar_points[LOW_SCAN:HIGH_SCAN]

        # 인식 최대 거리(0.7m) 제한 및 inf 거리 값 처리
        right_distances = [(i, dist) for i, dist in enumerate(right_points) if (0<dist<0.7 and not math.isinf(dist))]

        # 오른쪽 라이다 데이터가 Null인지 확인
        if right_distances:
            # 평균거리 측정
            avg_right_distance = sum(dist * math.sin(math.radians((LOW_SCAN - 180 + i) / 2)) for i, dist in right_distances) / len(right_distances)
        else:
            # 유효한 측정값이 없을 경우 기본값
            avg_right_distance = DIS  
        desired_distance = DIS  # 원하는 왼쪽 벽과의 거리

        # 크로스 트랙 에러계산 및, PID를 이용한 조향각 계산
        cte = avg_right_distance - desired_distance  
        angle = self.pid.TotalError(cte)

        # 평균 거리가 0.5초과, 오른쪽의 라이다 값이 모두 Null일 경우 터널 탈출 flag를 True로 전환
        if (avg_right_distance>0.5) or len(right_distances)==0:
            flag = 1

        return angle, flag

# PID클래스 정의
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

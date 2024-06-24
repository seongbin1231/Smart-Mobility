#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2

# 튜닝 파라미터
LOW_THRESH, HIGH_THRESH = 150, 255
LOW_ROI, HIGH_ROI = 300, 480
ZERO_THRESH = 13000
SPEED = 5

# 이미지와 신호 토픽을 받아, 정지 여부를 판단하는 함수
def detect_stop_line(image, signal):
    # img copy를 통해 원본 이미지 저장
    img=image.copy()

    # 이미지 처리
    # 그레이 스케일 -> 이진화 -> ROI영역 검출
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, LOW_THRESH, HIGH_THRESH, cv2.THRESH_BINARY)
    roi = binary[LOW_ROI:HIGH_ROI, :]

    # 흰색 영역 검출
    non_zero_count = cv2.countNonZero(roi)

    # ROI 이미지 영역 내에서 흰색 값이 특정 임계값을 넘어가며, 받은 신호의 색이 "Y" 또는 "R"일 때의 조건 판단
    if (non_zero_count > ZERO_THRESH) and (signal=="Y" or signal=="R"):
        return 0    # 정지
    else:
        return SPEED    # 주행
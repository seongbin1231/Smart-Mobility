#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

# 튜닝 파라미터
CROSS_ID = 2
CROSS_DIS = 2.0
JUDGE_TIME = 4
PARKING_ID = 4
PARKING_DIS = 1.15

# AR태그와 카메라까지의 상대적 위치, AR태그의 ID를 입력으로 받아 갈림길 구간을 인지하고, 좌, 우 signal과 time을 이용해 좌, 우 주행 판단
# return: G-갈림길 전 구간 , L-왼쪽 주행 판단 ,R-오른쪽 주행 판단
def left_right_judge(x,y,ID,left, right,time):
    # AR태그까지의 거리와 ID를 이용해 갈림길 구간 판단
    if 0.1<math.sqrt(math.pow(float(x),2)+math.pow(float(y),2)) < CROSS_DIS and (ID==CROSS_ID):
        # JUDGE_TIME보다 time이 작다면, 반대편 차산으로 주행하고, 아닐 시 빨간색 신호인 차선으로 주행
        if (right == "R"):  # 오른쪽 신호가 Red일 때
            if time < JUDGE_TIME:
                return "L"
            else :
                return "R"
        else:               # 왼쪽 신호가 Red일 때
            if time < JUDGE_TIME:
                return "R"
            else :
                return "L"
    # 위 조건에 부합하지 않을 시 갈림길 전이라고 판단하여 주행
    return "G"

# AR태그와 카메라까지의 상대적 위치, AR태그의 ID를 입력으로 받아 종료 조건 판단
def parking_check(x,y,id):
    # AR태그와의 거리, ID를 이용하여 조건 판단
    if (id == PARKING_ID) and math.sqrt(math.pow(float(x),2)+math.pow(float(y),2))<PARKING_DIS:
        return True # 주차 완료
    return False    # 주차 진행 중
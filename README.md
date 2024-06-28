# 🚗 Smart-Mobility 프로젝트

![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)

---

## 프로젝트 소개

Smart-Mobility 프로젝트는 Xycar 자율주행 플랫폼을 이용해 다양한 주행 환경을 안전하고 정확하게 주행할 수 있도록 설계되었습니다. 이 프로젝트에서는 ROS와 OpenCV를 활용하여 자율 주행 알고리즘을 구현하고, 각 미션에 적합한 센서를 효과적으로 활용하는 것을 중점으로 합니다.

---

## 미션 및 알고리즘 소개

### 1. 곡선 주행
<div align="center">
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/5b441a9a-2d2f-44f3-b861-6c997c275ae5" alt="곡선 주행" width="500" style="margin-bottom:20px"/>
</div>
- 자율 주행 차량이 정해진 곡선 도로를 따라 정확하게 주행하도록 합니다.
<div align="center">
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/260059b7-8a54-436d-948b-aef7ae939f37" alt="곡선 주행" width="500" style="margin-bottom:20px"/>
</div>
**적용 알고리즘**: Hough 선 검출을 사용하여 양쪽 차선을 인식합니다. 각 구역에서 여러 차선들을 하나의 대표 직선으로 표현한 후, 특정 y 값에서의 x 값을 도출합니다. 이 두 x 값을 평균하여 화면 중앙과의 차이를 에러로 정의합니다. 이 에러를 PID 제어에 사용하여 주행을 조정합니다. 곡선에서 PID를 적절하게 조정하면 곡선 주행도 안전하게 수행할 수 있었습니다.

### 2. 한 줄 주행
<div align="center">
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/98b582a2-b6d5-40a7-be04-629a7332c9e0" alt="한 줄 주행" width="500" style="margin-bottom:20px"/>
</div>
- 급커브 구간에서 차선 하나가 없는 상황에서도 반대편 차선을 인식하여 주행합니다.
<div align="center">
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/93f32990-569a-4592-b809-66f41445f8cc" alt="한 줄 주행1" width="500" style="margin-bottom:20px"/>
</div>
**적용 알고리즘**: 한 줄 주행에서는 양쪽 차선이 검출되지 않는 상황을 고려하여, 연속된 제어 주기 동안 차선이 검출되지 않으면 조향각을 왼쪽으로 설정하여 주행합니다.

### 3. 장애물 구간
<div align="center">
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/1171ea38-a0b0-4284-a001-9a0b33a31e20" alt="장애물 구간" width="500" style="margin-bottom:20px"/>
</div>
- 좌/우에 설치된 블록 장애물을 라이다와 이미지 센서로 인식하여 회피 주행합니다.
<div align="center">
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/55630fb9-7b8f-40ab-9189-ed3000635c60" alt="장애물 구간" width="300" style="margin-bottom:20px"/>
</div>
**적용 알고리즘**: 장애물 회피는 라이다 센서로 인식된 데이터를 기반으로 기구학 모델을 적용하여 수행됩니다. 회피 조건이 만족되면, 정의된 회피 조향각을 명령으로 주행합니다.
<div align="center">
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/4ca5483b-0f4a-4e44-835c-83640b214faf" alt="장애물 구간" width="300" style="margin-bottom:20px"/>
</div>

### 4. 터널 구간
<div align="center">
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/1352d129-a3e5-4388-b9dc-118154c5bf00" alt="터널 구간" width="500" style="margin-bottom:20px"/>
</div>
- 차선이 없는 터널 구간에서 라이다 센서만으로 터널에 부딪히지 않고 주행합니다.
<div align="center">
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/c167c894-aa65-44a0-b9b1-abd0460907b2" alt="터널 구간" width="500" style="margin-bottom:20px"/>
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/af2835d0-6dc8-43d7-bdbf-9fb93fb1d5f0" alt="터널 구간" width="200" style="margin-bottom:20px"/>
</div>
**적용 알고리즘**: 터널 주행은 라이다를 이용해 오른쪽 벽과의 거리를 유지하며 주행합니다. 라이다 값의 평균이 일정 임계값을 유지하도록 PID 제어를 통해 조정하며, 전방 라이다 값에 대한 민감도를 줄이기 위해 sin 가중치를 적용합니다.

### 5. 횡단보도
<div align="center">
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/1a8b06ae-94de-4f7b-90d5-6312f36f67d9" alt="횡단보도" width="500" style="margin-bottom:20px"/>
</div>
- Single traffic 신호에 맞게 횡단보도를 주행합니다.
**적용 알고리즘**: 카메라 이미지에서 도로 영역을 ROI로 설정하고, 이진화를 통해 밝기에 따른 정지선을 인식합니다. OpenCV의 non_zero_count 함수를 사용하여 흰색 면적이 임계값 이상일 경우 정지선으로 인식하며, 신호등이 빨간색이나 노란색일 때는 정지하도록 설정합니다.

### 6. 갈림길 구간
<div align="center">
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/d81b88ef-a06f-4e18-8375-f4e58b358e5d" alt="갈림길 구간" width="500" style="margin-bottom:20px"/>
</div>
- 갈림길 구간에서 Cross load traffic 신호를 바탕으로 좌/우 갈림길 중 최적의 길을 선택하여 주행합니다. 횡단보도에서는 각 신호에 맞게 주행합니다.
<div align="center">
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/6e44ce0c-e149-4f7a-9be9-8e142a19140f" alt="갈림길 구간" width="700" style="margin-bottom:20px"/>
</div>
**적용 알고리즘**: AR 태그 인식 후 횡단보도 구간까지 도달하는 시간을 고려하여, 빨간색 신호가 지속되는 시간을 기준으로 최적의 차선을 선택하여 주행합니다. 빨간색 신호가 4초보다 길 경우 해당 차선으로 주행하며, 짧을 경우 반대 차선으로 이동하여 최대 3초 동안 정차 후 주행합니다.

### 7. 주차 구간
<div align="center">
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/0258adf9-cc45-4d27-854f-7ca9708e8376" alt="주차 구간" width="500" style="margin-bottom:20px"/>
</div>
- AR 태그를 이용하여 사각형 구간 내에 정확하게 차량을 주차합니다.
<div align="center">
   <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/a801ca9c-8cf2-4186-9d7e-798043e0f30e" alt="주차 구간" width="150" style="margin-bottom:20px"/>
</div>
**적용 알고리즘**: Hough 라인을 이용해 주차 구역을 인식하며, AR 태그와 카메라 간의 상대 거리를 측정합니다. 특정 경계값에 도달하면 속도와 각도를 0으로 설정하여 주차를 완료합니다.

---

## 주행 영상

> [주행 영상 보기](https://youtube.com/shorts/uRC0T9Ergz4)

---

## Collaborators

- **정훈(Leader)**: [h12365@uos.ac.kr](mailto:h12365@uos.ac.kr)
- **전현욱**: [hyunwook6457@gmail.com](mailto:hyunwook6457@gmail.com)
- **유승원**: [youseungwon1013@gmail.com](mailto:youseungwon1013@gmail.com)
- **조성빈**: [czy4051@uos.ac.kr](mailto:czy4051@uos.ac.kr)

# 🚗 Smart-Mobility 프로젝트

![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)

---

## 프로젝트 소개

Smart-Mobility 프로젝트는 Xycar 자율주행 플랫폼을 이용해 다양한 주행 환경을 안전하고 정확하게 주행할 수 있도록 하는 것을 목표로 합니다. 이 프로젝트에서는 ROS와 OpenCV를 활용하여 자율 주행 알고리즘을 구현하고, 각 미션에 적합한 센서를 효과적으로 활용하는 것을 중점으로 합니다.

---

## 미션 및 알고리즘 소개

1. **곡선 주행**<br>
   <div align="center">
     <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/5b441a9a-2d2f-44f3-b861-6c997c275ae5" alt="곡선 주행" width="500" style="margin-bottom:20px"/>
   </div>
   - 자율 주행 차량이 정해진 곡선 도로를 따라 정확하게 주행하도록 합니다.

2. **한 줄 주행**<br>
   <div align="center">
     <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/98b582a2-b6d5-40a7-be04-629a7332c9e0" alt="한 줄 주행" width="500" style="margin-bottom:20px"/>
   </div>
   - 급커브 구간에서 차선 하나가 없는 상황에서 반대편 차선을 인식하여 주행시킵니다.

3. **장애물 구간**<br>
   <div align="center">
     <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/1171ea38-a0b0-4284-a001-9a0b33a31e20" alt="장애물 구간" width="500" style="margin-bottom:20px"/>
   </div>
   - 좌/우에 차례대로 설치된 블록 장애물을 라이다/이미지 센서로 인식하여 회피 주행시킵니다.

4. **터널 구간**<br>
   <div align="center">
     <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/1352d129-a3e5-4388-b9dc-118154c5bf00" alt="터널 구간" width="500" style="margin-bottom:20px"/>
   </div>
   - 차선이 없는 터널 구간에서 라이다 센서만으로 터널에 부딪히지 않고 주행합니다.

5. **횡단보도**<br>
   <div align="center">
     <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/1a8b06ae-94de-4f7b-90d5-6312f36f67d9" alt="횡단보도" width="500" style="margin-bottom:20px"/>
   </div>
   - Single traffic 신호에 맞게 횡단보도를 주행합니다.

6. **갈림길 구간**<br>
   <div align="center">
     <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/d81b88ef-a06f-4e18-8375-f4e58b358e5d" alt="갈림길 구간" width="500" style="margin-bottom:20px"/>
   </div>
   - 갈림길 구간에서 Cross load traffic 신호를 바탕으로 좌/우 갈림길 중 최적의 길을 선택하여 주행합니다. 이때 횡단보도에서는 각 신호에 맞게 주행할 수 있도록 합니다.

7. **주차 구간**<br>
   <div align="center">
     <img src="https://github.com/seongbin1231/Smart-Mobility/assets/100406734/0258adf9-cc45-4d27-854f-7ca9708e8376" alt="주차 구간" width="500" style="margin-bottom:20px"/>
   </div>
   - AR 태그를 이용하여 사각형 구간 내에 정확하게 차량을 주차시킵니다.

---

## 주행 영상

> [주행 영상 보기](https://youtube.com/shorts/uRC0T9Ergz4)

---

## Collaborators

- **정훈(Leader)**: [h12365@uos.ac.kr](mailto:h12365@uos.ac.kr)
- **전현욱**: [hyunwook6457@gmail.com](mailto:hyunwook6457@gmail.com)
- **유승원**: [youseungwon1013@gmail.com](mailto:youseungwon1013@gmail.com)
- **조성빈**: [czy4051@uos.ac.kr](mailto:czy4051@uos.ac.kr)

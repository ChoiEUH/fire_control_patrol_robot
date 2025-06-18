# 화재 제어 순찰 로봇
<img src="https://github.com/user-attachments/assets/145c48e6-a912-45de-930f-3fe2a6a23835" width= "1000" height= "500"/>

<img src="https://github.com/user-attachments/assets/d1c6d85a-a7e9-48c8-b4ef-6e830db4044d" width= "700" height= "300"/>

2024 디지텍 캡스톤 디자인 경진대회 디지텍상 입상

H/W회로, S/W(F/W)개발 : 최의현

## TECH
<img src="https://img.shields.io/badge/stmicroelectronics-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=blue"> <img src="https://img.shields.io/badge/C-A8B9CC?style=for-the-badge&logo=C&logocolor=white">

## 개발일시
2024/08/30 ~ 2024/11/19


## 기술
1. FreeRTOS를 이용한 로봇 각각의 작업들을 태스크 분배
2. 로봇 모터에 들어가는 TIM_CCRegister를 이용한 PWM 펄스 폭 조절
3. 최소 자원 접근 및 데이터 응답 속도 최적화를 위한 ADC Register를 이용
4. BLE 통신을 이용한 Raspberry PI에 불꽃감지센서 값을 전송 후 PI내에서 모니터링을 위한 CAM 작동(순찰 목적)

## FlowChart
<img src= "https://github.com/user-attachments/assets/13a12957-9ae0-4cc6-bd95-3ea9a0d56fdb" width= "1000" height= "500"/>

## Trouble Shooting
1. 로봇 주행 시 모터 토크 값이 부족함으로서, 전압을 올려봤지만, 한계가 있기에 TIM_CCR을 이용해서 PWM 출력을 70%인 7000에서 80%인 8000으로 상승 시킴. (전류를 높임)
2. 불꽃감지센서의 센서값 전송속도 딜레이로 인해서 감지 능력이 하락되고 느려짐에 따라 고민하다 Task내에 osDelay초를 1ms로 설정함으로 처리속도를 빠르게함.
3.  적외선 거리센서는 서미스터 같은 온도센서와 같이 장애물이 가까워질수록 센서값이 커짐, 이를 고민하고 계산하여 거리값 함수 식 코드를 세워서 가까워지면 센서값이 작아지도록 변경시킴.
4.  로봇이 장애물을 피해 움직일 시 하나의 방향으로만 회전하여 피하는 문제를 고민하면서, 소프트웨어에서의 난수로 움직이게 하는 것이 아닌 센서를 각각 좌 우에 설치함으로서(초음파센서, 적외선 거리센서) 먼저 감지되는 방향의 반대방향으로 자유롭게 움직이게끔 개발시킴.

## 동작 영상
https://github.com/user-attachments/assets/96241d8c-af59-4c15-99d0-e84967e38dbe

https://github.com/user-attachments/assets/81be6f6b-a8d2-4311-8362-08faf38e36d0

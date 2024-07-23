---
description: Px4 스택을 사용하는 차량을 제어하기 위한 레포지토리 브렌치
---

# UV Base/GoKart

## &#x20;[레포지토리](https://github.com/ICSL-hanyang/uv\_base/tree/Go-kart)&#x20;

주된 내용은 드론연구자료의 UV Base와 동일하나 ( 2020/03/17 기준)

* 로버를 위한 모델 변경 (Ouster 16채널 Lidar 등의 센서추가)
* 로버의 Body Frame 기준의 Velocity 제어를 위한 펌웨어 수정&#x20;
* EV Grand Prix 대회를 위한 Gazebo 맵 구현&#x20;
* Cartographer 를 이용한 Localization 및 Navigation 구현
* Rviz 세팅 값 저장

등이 수정 되었다.&#x20;

### **Dependency**

* Px4 Firmware version 1.11.0-beta1
* Ubuntu 18.04
* ROS melodic
* Gazebo9

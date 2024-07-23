---
description: Pixhawk에 올라가는 펌웨어인 PX4와 관계된 참고 사이트
---

# PX4 펌웨어 관련자료

## [PX4 Development guide](https://dev.px4.io/master/en/index.html)

![PX4 Development Guide 사이트](../.gitbook/assets/px4\_dev\_guide.png)

PX4 펌웨어에 대한 전반적인 설명을 참고할 수 있는 사이트. 위의 그림에서 왼쪽 상단의 master 라고 되어 있는 것은 펌웨어의 브랜치를 의미한다.

{% hint style="info" %}
대체로 Master 브랜치의 내용을 확인하면 잘 맞으나 상황에 따라서는 stable 한 최신 버전(2019년 10월 6일 작성일 기준 1.9버전)으로 변경하여 내용을 확인 해보는 것이 좋음. 가끔씩 마스터 버전과 stable 버전의 내용이 다른 경우가 있음.
{% endhint %}

## [PX4 Firmware](https://github.com/PX4/Firmware)

PX4 펌웨어가 있는 깃허브 사이트.

### sitl\_gazebo

펌웨어에서 Tools/sitl\_gazebo는 펌웨어와 연결된 깃 서브모듈로써 SITL(Software In the Loop)과 HITL(Hardware In the Loop) 등으로 시뮬레이션을 하기 위한 다양한 파일들이 존재한다. &#x20;

그 안에는 3DR의 드론인 IRIS의 모델과 시뮬레이션의 배경이 되는 월드 등이 포함된다.&#x20;

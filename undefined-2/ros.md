---
description: 로봇 개발 분야의 큰 트랜드인 ROS와 관련된 자료를 검색가능한 사이트
---

# ROS 관련자료

## [ROS wiki](http://wiki.ros.org/)

![ROS에 관한 다양한 정보를 확인 가능한 ROS wiki 사이트](../.gitbook/assets/ROS\_wiki.png)

ROS의 설치, 튜토리얼, 최근 변화 및 컨셉 등 모든 것들이 기록되어져 있는 ROS가 뭔지 알기 위해서는 필수적으로 들어가봐야 하는 사이트.

{% hint style="info" %}
ROS에서 제공하는 다양한 공식패키지에 대한 사용법이나 Publish 또는 Subscribe 하고 있는 데이터에 관한 문서들도 제공되므로 ROS 위키 우측 상단에 Search를 통해 원하는 패키지의 정보를 찾을 수 있다.
{% endhint %}

## [MAVROS](http://wiki.ros.org/mavros)

![Mavros의 대부분의 모든 정보가 문서화 되어 담겨 있는 사이트](../.gitbook/assets/mavros\_wiki.png)

ROS를 통해 드론을 제어하게 되면 가장 필수적으로 사용하게 되는 ROS 공식 패키지. 드론과 데이터 교환 및 드론의 제어를 위해 개발 된 프로토콜인 Mavlink 메시지를 ROS에서 이해 할 수 있는 메시지로 변경할 뿐만 아니라 반대로 ROS 메시지를 Mavlink 메시지로 변경해주는 브릿지 역할을 하는 패키지. 2019년 10월 기준 최신 패키지는 ROS의 Kinetic 버전 뿐만 아니라 최신 버전인 Melodic에서도 호환성의 문제 없이 잘 동작한다.

차례를 보면 알 수 있듯이 이 패키지가 Publish 하는 토픽들과 Subscribe 하는 토픽들, 제공하는 Service 들에 대해 기록 되어 있고, 해당 토픽이나 서비스를 이용하기 위해사용되고 있는 ROS msg 타입과 형식등이 명시되어 있다. 뿐만아니라 다양한 Plugin 등을 제공하여 드론을 제어하거나 특별한 임무를 수행 할 수 있도록 돕는다.&#x20;

## [eDrone](https://github.com/sooyoungmoon/eDrone/tree/master/src)

ETRI에서 개발한 Drone API로 문성태 연구원이 주로 개발하였다. 17년 12월 부터 개발 시작되었으며 19년 7월 마지막 커밋이 확인된다(2019년 10월  기준). ROS를 사용하여 드론을 제어하기 위한 API로써 우리의 연구와 방향성이 비슷하며 드론을 통해 사업 또는 연구를 하려는 사람들을 위해 제작되어 참고할 만한 코드들이 있으니 참고하면 좋다.

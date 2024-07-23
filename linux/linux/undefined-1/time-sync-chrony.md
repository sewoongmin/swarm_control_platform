---
description: 시간동기화
---

# Time Sync ( chrony )

## 사용목적

**다 수의 컴퓨터**에서 ROS와 같은 시스템 사용시 메세지 교환에 있어서 **시간의 동기화**가 필요하다. 인터넷을 통해 시간을 받아 올 수 있다.&#x20;

## Chrony 설치

```
sudo apt install chrony
```

### Chrony 자동실행 설정

```
sudo systemctl enable chrony.service
sudo systemctl start chrony.service
```

위의 명령어 중 enable 작업을 통해 시스템이 부팅 될 때마다 자동으로 실행시키도록 설정해준다. 밑의 start 명령어는 지금 실행 시키는 명령어이다.&#x20;

### 동작검증&#x20;

```
sudo systemctl status chronyd
```

위 명령어를 실행했을 때 아래 그림과 같이 나오면 잘 실행되고 있는것이다.&#x20;

![](<../../../.gitbook/assets/image (34).png>)

```
timedatectl status
```

위 명령어를 통해 시간동기화가 잘 되고 있는지 확인 가능하다

![](<../../../.gitbook/assets/image (12).png>)

위 그림처럼 System clock synchronized : yes 이면 제대로 동기화 되고 있는 것이다.

만약 System clock synchronized : no로 나온다면 아래 명령어를 실행해 보자

```
sudo timedatectl set-ntp yes
```

#### chrony가 시간동기화를  제대로 진행하고 있는지 확인한다.&#x20;

```
chronyc tracking
```

![](<../../../.gitbook/assets/image (24).png>)

#### chrony가 동기화에 사용되는 네트워크를 확인한다.&#x20;

```
chronyc sources
```

![](<../../../.gitbook/assets/image (16).png>)

### 동기화 서버정보 변경

파일 경로 : /etc/chrony/chrony.conf 수정해야 함.

**인터넷이 안되는 환경**에서 **ROS\_Master와 시간동기화를 위해** 이것을 변경해야 할 가능성이 있음.


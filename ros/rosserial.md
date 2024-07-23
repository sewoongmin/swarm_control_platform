# Rosserial

## [Arduino\_James](https://github.com/3watt/Arduino\_James)

james에 올라갈 arduino code 입니다.

### 적용된 센서

* [x] barometer : MS5611
* [x] ultresound : ---
* [x] IR : --- ( 내부가 비었는지 찼는지 확인용 )
* [ ] Motor ( conveyer control ) **- 추가 예정**
* [ ] Motor ( linear Actuator ) **- 추가 예정**

### Topic & Service

#### Topic (20 hz)

* /sonar0 (ultrasound)
* /temperature (barometer)
* /absolute\_alt (barometer)
*   /relative\_alt (barometer)

    **Service**
* /ir       ( std\_srvs/Trigger)
* /ir\_light ( std\_srvs/SetBool )

### ISSUE (20. 06. 26 기준)

* PC 와 ROS로 연결 시 PC의 rosserial package 의 최신 버전에서 service response를 받을 수 없음. (18년 12월 발생한 문제지만 수정되지 않고 있음)

해결 방법 : [https://github.com/ros-drivers/rosserial](https://github.com/ros-drivers/rosserial) 에서 0.7.7 버전 받아서 사용

## Arduino\_W-station

W-station에 올라갈 arduino code 입니다.

### 적용된 센서

* [x] IR : --- ( 내부가 비었는지 찼는지 확인용 )
* [ ] Motor ( conveyer control ) _10 \*- 추가 예정_
* [ ] Motor ( carrier ) _2 \*- 추가 예정_

### IP Setup

```cpp
IPAddress ip(192, 168, 1, 99);
IPAddress myDns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);
EthernetServer server(3300);
```

위 코드로 아이피를 설정 가능함. 현재 설정된 **IP = 192.168.1.99**

포트번호도 중요한데 현재는 **3300 번**으로 사용중

### Protocol

1. 메세지 종료를 포함하여 메세지는 총 4바이트로 구성됨.
2. 첫번째 바이트는 알파벳 대문자 'A \~ C' 로 구성됨. ( 필요시 Z 까지 확장 가능 )
3. 두번째와 세번째 바이트는 숫자 '0 \~ 9' 로 구성됨.&#x20;
4. 마지막 바이트는 '/'로 끝나야 함.&#x20;

#### 사용 예제

```bash
netcat 192.168.1.99 3300
A00/
B23/
C55/
```

터미널에 위와 같은 명령어를 사용하여 제어.

### 구현

#### IR 센서 제어

1. 메세지의 첫번째 바이트가 'A' 면 IR 센서를 제어함.
2. 메세지의 두번째 바이트가 0이면 LED를 끄고, 1이면 LED를 켬.
3. 메세지의 세번째 바이트가 1이면 물건이 있는지를 확인하고, 0이면 안함.
   1. 물건 확인시 물건이 있으면 클라이언트에게 1을 리턴, 없으면 0을 리턴.
4. 그 외의 명령어에 대해서는 클라이언트에게 "Wrong command" 메세지 리턴

**사용예제**

* A00/ : LED를 끔.
* A10/ : LED를 켬.
* A01/ : LED를 끄고, 물건이 있는지를 확인함.
* A11/ : LED를 켜고, 물건이 있는지를 확인함.

> 주의! : LED 제어는 한번 켜두면 끄기 전까진 계속 켜있는 상태가 됨.

#### Conveyer Motor 제어 ( 구현 예정 )

1. 메세지의 첫번메세지의 첫번째 바이트가 'B' 면 컨베이어 모터를 제어함.
2. 메세지의 두번째 바이트와 세번째 바이트가 제어할 모터 번호임.

> 필요시 두번째 바이트를 모터 번호, 세번째 바이트를 제어 시간으로 사용 가능.

**사용예제**

* B01/ : 1번 컨베이어 제어.
* B10/ : 10번 컨베이어 제어.

#### Carrier Motor 제어 ( 구현 예정 )

1. 메세지의 첫번메세지의 첫번째 바이트가 'C' 면 캐리어 모터를 제어함.
2. 메세지의 두번째 바이트와 세번째 바이트가 캐리어가 가야 할 좌표임.

**사용예제**

* C51/ : 캐리어를 (5, 1)로 보냄.
* C33/ : 캐리어를 (3, 3)로 보냄.


---
description: 드론의 다양한 데이터를 구독할 때마다 자동으로 실행되는 함수들과 그 기능 설명
---

# Callback 함수

{% hint style="info" %}
모든 Callback 함수들은 토픽을 받았을 때만 동작하므로 Private 인터페이스로 정의되어 있다.
{% endhint %}

## void stateCB(const mavros\_msgs::State::ConstPtr &)

### mavros\_msgs::State 메세지 정의

```cpp
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
bool connected
bool armed
bool guided
string mode
uint8 system_status
```

mavros와 px4 간의 연결성을 나타내는 connected 변수. 시동(Arm) 여부를 나타내는 armed 변수. APM 펌웨어에서 제공하는 guided 모드 여부를 나타내는 guided 변수. 현재 모드를 문자열로 저장하고 있는 mode 변수. 드론의 시스템 상태를 숫자로 나타내는 system\_status 변수로 구성되어 있다.

### PX4 펌웨어에서 제공하는 모드 종류

```
MANUAL, STABILIZED,	OFFBOARD, ACRO, ALTCTL,	POSCTL,	RATTITUDE, 
AUTO.TAKEOFF, AUTO.LAND, AUTO.MISSION, AUTO.LOITER,	AUTO.RTL, AUTO.RTGS, AUTO.READY
```

AUTO.로 시작하는 모드들은 GPS가 정상적으로 연결되어 있을 때 사용 가능한 모드들이다. **GPS가 오동작을 하거나 연결이 되어있지 않은 경우에 AUTO.로 시작하는 모드로 변경을 요청하면 해당 요청은 거절**된다.

**드론을 자율주행하기 위해서는 OFFBOARD 모드 상태여야 함을 명심하자.**

### stateCB() 함수 구현 코드

```cpp
void Vehicle::stateCB(const mavros_msgs::State::ConstPtr &msg)
{
	if (cur_state_.connected != msg->connected)
	{
		if (msg->connected == true)
			ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " is connected");
		else
			ROS_WARN_STREAM(vehicle_info_.vehicle_name_ << " is not connected");
	}
	if (cur_state_.armed != msg->armed)
	{
		if (msg->armed == true)
			ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " is armed");
		else
			ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " is disarmed");
	}
	if (cur_state_.mode != msg->mode)
	{
		ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " is " 
		    << msg->mode << " mode");
	}
	cur_state_ = *msg;
}
```

mavros\_mgs::State 타입으로 정의된 멤버 변수 cur\_state\_에 전달 받은 값을 저장하고, 변경되는 데이터가 있을 때 ROS\_INFO를 통해 터미널에 출력하도록 되어 있다.

## void batteryCB(const sensor\_msgs::BatteryState::ConstPtr &)

### sensor\_msgs::BatteryState 메세지 정의

```cpp
uint8 POWER_SUPPLY_STATUS_UNKNOWN=0
uint8 POWER_SUPPLY_STATUS_CHARGING=1
uint8 POWER_SUPPLY_STATUS_DISCHARGING=2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING=3
uint8 POWER_SUPPLY_STATUS_FULL=4
uint8 POWER_SUPPLY_HEALTH_UNKNOWN=0
uint8 POWER_SUPPLY_HEALTH_GOOD=1
uint8 POWER_SUPPLY_HEALTH_OVERHEAT=2
uint8 POWER_SUPPLY_HEALTH_DEAD=3
uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE=4
uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE=5
uint8 POWER_SUPPLY_HEALTH_COLD=6
uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE=7
uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE=8
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN=0
uint8 POWER_SUPPLY_TECHNOLOGY_NIMH=1
uint8 POWER_SUPPLY_TECHNOLOGY_LION=2
uint8 POWER_SUPPLY_TECHNOLOGY_LIPO=3
uint8 POWER_SUPPLY_TECHNOLOGY_LIFE=4
uint8 POWER_SUPPLY_TECHNOLOGY_NICD=5
uint8 POWER_SUPPLY_TECHNOLOGY_LIMN=6
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 voltage
float32 current
float32 charge
float32 capacity
float32 design_capacity
float32 percentage
uint8 power_supply_status
uint8 power_supply_health
uint8 power_supply_technology
bool present
float32[] cell_voltage
string location
string serial_number
```

로봇의 배터리 상태를 전달하기 위해 구현된 메세지 타입. 전압, 전류, 용량, 잔량 등 배터리와 관련된 다양한 정보들로 구성되어 있다.

### batteryCB() 함수 구현 코드

```cpp
void Vehicle::batteryCB(const sensor_msgs::BatteryState::ConstPtr &msg)
{
	cur_battery_ = *msg;
}
```

sensor\_msgs::BatteryState 타입으로 정의 된 멤버 변수 cur\_battery\_에 구독된 값을 기록한다.&#x20;

## void homeCB(const mavros\_msgs::HomePosition::ConstPtr &)

### mavros\_msgs::HomePosition 메세지 정의

```cpp
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geographic_msgs/GeoPoint geo
  float64 latitude
  float64 longitude
  float64 altitude
geometry_msgs/Point position
  float64 x
  float64 y
  float64 z
geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w
geometry_msgs/Vector3 approach
  float64 x
  float64 y
  float64 z
```

드론으로부터 홈 위치에 대한 GPS 좌표값, 로컬 포지션값, 자세값 등을 전달하기 위해 정의되었다. 여기에는 GPS 좌표 기반의 글로벌 홈과 로컬 홈이 둘다 존재하는 것으로 보이지만 로컬 홈 좌표는 개별적으로 설정할 수 없다. mavros/cmd/set\_home 서비스를 통해 임의의 GPS 좌표로 글로벌 홈 좌표를 설정하거나, 현재 GPS 좌표로 글로벌 홈 좌표 설정시 새로 설정된 글로벌 홈과 이전 글로벌 홈 좌표의 차를 [ENU좌표계](https://en.wikipedia.org/wiki/Local\_tangent\_plane\_coordinates)로 변환하여 로컬 홈 좌표계가 변환 된다.

{% hint style="warning" %}
현재 드론의 초기 yaw값을 추정하기 위해 arming 요청시 orientation을 기반으로 yaw값을 기록하는 arming\_yaw보다 HomePosition에 기록된 orientation을 기반으로 구한 yaw 값이 좀더 신뢰도가 높을 것으로 예상된다. 하지만 HomePosition에서 표시되는 orientation과 로컬 포지션에서 표시되는 orientation에서 x와 w의 값이 바뀌어 표시되고 있으므로 검증이 필요하다.
{% endhint %}

### homeCB() 함수 구현 코드

```cpp
void Vehicle::homeCB(const mavros_msgs::HomePosition::ConstPtr &msg)
{
	home_global_.latitude = msg->geo.latitude;
	home_global_.longitude = msg->geo.longitude;
	home_global_.altitude = msg->geo.altitude;
	home_local_.pose.position.x = msg->position.x;
	home_local_.pose.position.y = msg->position.y;
	home_local_.pose.position.z = msg->position.z;
}
```

메세지로 받은 홈 좌표를 sensor\_msgs::NavSatFix 타입의 home\_global\_ 변수와 geometry\_msgs::PoseStamped 타입의 home\_local\_변수에 기록한다.

### sensor\_msgs::NavSatFix 메세지 정의

```cpp
uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
sensor_msgs/NavSatStatus status
  int8 STATUS_NO_FIX=-1
  int8 STATUS_FIX=0
  int8 STATUS_SBAS_FIX=1
  int8 STATUS_GBAS_FIX=2
  uint16 SERVICE_GPS=1
  uint16 SERVICE_GLONASS=2
  uint16 SERVICE_COMPASS=4
  uint16 SERVICE_GALILEO=8
  int8 status
  uint16 service
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type
```

로봇의 GPS 정보를 전달하기 위해 정의된 메세지 타입. longitude, latitude, altitude의 기본적인 정보뿐만 아니라 GPS 값의 신뢰도를 측정할 수 있는 covariance와 수신된 인공위성 개수에 따라 달라지는 status 등으로 구성되어 있다.

### geometry\_msgs::PoseStamped 메세지 정의

```cpp
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```

Time stamp와 함께 로봇의 위치(Position)와 자세(Orientation)을 전달하기 위해 구현된 메세지 타입. Orientation을 표현하기 위해 로봇분야에서 많이 사용되는 Quaternion이 사용된다.&#x20;

## void globalPositionCB(const sensor\_msgs::NavSatFix::ConstPtr &)

```cpp
void Vehicle::globalPositionCB(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	cur_global_ = *msg;
}
```

메세지로 전달받은 드론의 GPS 좌표를 sensor\_msgs::NavSatFix 타입으로 정의된 멤버 변수 cur\_global\_에 기록한다.

## void localPositionCB(const geometry\_msgs::PoseStamped::ConstPtr &)

```cpp
void Vehicle::localPositionCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	cur_local_ = *msg;
}
```

메세지로 전달받은 드론의 로컬 좌표를 geometry\_msgs::PoseStamped 타입으로 정의된 멤버 변수 cur\_local\_에 기록한다.

## void turnCB(const std\_msgs::Bool::ConstPtr &)

### std\_msgs::Bool 메세지 정의&#x20;

```cpp
bool data
```

ROS topic으로 bool 타입의 데이터를 전송하기 위해 구현된 메세지 타입.

### turnCB 함수 구현 코드

```cpp
void Vehicle::turnCB(const std_msgs::Bool::ConstPtr &msg)
{
	//this->turn_flag_ = msg->data;
	turned_yaw_ -= M_PI_2l;
}
```

callback 함수가 콜 될 때마다 생성자에 의해 0으로 초기화 된 double 타입 변수 turned\_yaw의 값이 π / 2 씩 감소한다. yaw 값은 radian으로 표현되며 turned\_yaw\_ 값이 변할 때마다 위치제어 함수인 void gotoLocal() 함수에 의해 드론이 시계방향으로 90도 씩 회전한다.&#x20;

{% hint style="info" %}
/turn 토픽은 드론에서 Publish 하지 않고 비행 중 임의로 드론을 회전시키기 위해 구현되었으므로 터미널에서 "`rostopic pub /turn std_msgs/Bool true "`명령을 통해 실행 할 수 있다.
{% endhint %}

{% hint style="warning" %}
토픽에 사용된 메세지는 std\_msgs::Bool 인데 전달받은 데이터가 true인지 false인지에 영향 없이 코드가 동작하므로 std\_msgs::Empty 메세지 타입으로 변경하는 것이 적절하다.
{% endhint %}

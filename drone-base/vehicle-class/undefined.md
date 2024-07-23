---
description: Vehicle 클래스를 통해 인스턴스가 생성되거나 소멸될 때 실행 되는 함수
---

# 생성자 / 소멸자

## Vehicle()

아래와 같은 코드로 이 API를 사용하는 개발자가 Vehicle 객체를 생성시 실수로 인자를 넣어주지 않은 것을 막기 위해 명시적으로 삭제하였다.&#x20;

```cpp
Vehicle() = delete;
```

## Vehicle(ros::NodeHandle &, ros::NodeHandle &)

Vehicle 객체를 생성하기 위한 최소 조건은 2개의 NodeHandle 객체를 집어 넣어 주는 것이다. 하나의 NodeHandle은 군집드론 제어시 모든 드론에게 동일한 명령을 전달하기 위해 사용 되며, 다른 하나는 파라미터 변경 또는 일반적인 토픽의 Publish, Subscribe에 사용된다. 각각의 객체가 NodeHandle을 직접 보유하고 있어도 되지만, 이 경우 군집드론 제어시 하나의 노드에 너무 많은 NodeHandle이 돌게 되고 이것은 **ROS에서 권장되지 않으므로 객체 생성시 NodeHandle을 입력하도록 했다.**  구현된 코드는 아래와 같다.

```cpp
Vehicle::Vehicle(ros::NodeHandle &nh_mul, ros::NodeHandle &nh_global)
	: vehicle_info_({1, "mavros"}),
	  nh_(ros::NodeHandle(vehicle_info_.vehicle_name_)),
	  nh_mul_(nh_mul),
	  nh_global_(nh_global),
	  pos_(tf2::Vector3(0, 0, 0)),
	  sum_sp_(tf2::Vector3(0, 0, 0)),
	  err_(tf2::Vector3(0, 0, 0)),
	  setpoint_pos_(tf2::Vector3(0, 0, 0)),
	  setpoint_publish_flag_(false),
	  turned_yaw_(0)
{
	vehicleInit();
}
```

초기화 목록을 통해 각각의 멤버 변수를 초기화 해준다. ROS 관련 변수를 초기화 하기 위해 vehicleInit()라는 함수가 실행 되는데 모든 생성자에서 동일한 초기화가 필요하므로 중복을 피하기 위해 함수화 하여 구현하였다. vehicleInit() 함수에 대해서는 아래쪽에서 설명하겠다.

## Vehicle(ros::NodeHandle &, ros::NodeHandle &, const VehicleInfo &)

### VehicleInfo 구조체

```cpp
typedef struct vehicle_info
{
	int vehicle_id_;
	std::string vehicle_name_;
} VehicleInfo;
```

드론의 번호와 이름을 저장할 수 있는 VehicleInfo 구조체이다.

### 생성자 코드

```cpp
Vehicle::Vehicle(ros::NodeHandle &nh_mul, ros::NodeHandle &nh_global, 
    const VehicleInfo &vehicle_info)
	: vehicle_info_(vehicle_info),
	  nh_(ros::NodeHandle(vehicle_info_.vehicle_name_)),
	  nh_mul_(nh_mul),
	  nh_global_(nh_global),
	  pos_(tf2::Vector3(0, 0, 0)),
	  sum_sp_(tf2::Vector3(0, 0, 0)),
	  err_(tf2::Vector3(0, 0, 0)),
	  setpoint_pos_(tf2::Vector3(0, 0, 0)),
	  setpoint_publish_flag_(false),
	  turned_yaw_(0)
{
	vehicleInit();
}
```

Vehicle 객체의 기본 생성자와 동일하지만 드론의 정보를 받아서 객체를 생성하도록 구현하였다. 위의 생성자와 마찬가지로 초기화 목록을 통해 멤버변수들을 초기화 한다.

## Vehicle(const Vehicle &)

복사 생성자. std::vector와 같은 template library를 사용하기 위해서 구현했다. **복사 생성자와 대입 생성자를 구현하지 않고 template library를 사용하면 수 많은 버그들과 대면 할 수 있으니 주의하자.** 구현 코드는 아래와 같다.

```cpp
Vehicle::Vehicle(const Vehicle &rhs)
	: vehicle_info_(rhs.vehicle_info_),
	  nh_(ros::NodeHandle(vehicle_info_.vehicle_name_)),
	  nh_mul_(rhs.nh_mul_),
	  nh_global_(rhs.nh_global_),
	  pos_(rhs.pos_),
	  sum_sp_(rhs.sum_sp_),
	  err_(rhs.err_),
	  setpoint_pos_(rhs.setpoint_pos_),
	  setpoint_publish_flag_(rhs.setpoint_publish_flag_),
	  turned_yaw_(rhs.turned_yaw_)
{
	vehicleInit();
	*this = rhs;
}
```

초기화 목록을 통해 초기화 할 때, **입력된 객체의 데이터를 기반으로 초기화 함을 유의**하라.&#x20;

## const Vehicle \&operator=(const Vehicle &)

대입 생성자. 복사 생성자와 구현 목적은 같다.&#x20;

```cpp
const Vehicle &Vehicle::operator=(const Vehicle &rhs)
{
	if (this == &rhs)
		return *this;

	vehicle_info_ = rhs.vehicle_info_;
	setpoint_publish_flag_ = rhs.setpoint_publish_flag_;

	nh_ = ros::NodeHandle(vehicle_info_.vehicle_name_);
	nh_mul_ = rhs.nh_mul_;
	nh_global_ = rhs.nh_global_;
	pos_ = rhs.pos_;
	sum_sp_ = rhs.sum_sp_;
	err_ = rhs.err_;
	setpoint_pos_ = rhs.setpoint_pos_;

	vehicleInit();
	return *this;
}
```

대입 생성자 구현시 3번 줄과 같이 **대입 연산자의 좌변과 우변의 객체의 주소값이 같은지 확인하는 작업이 필수적으로 선행되어야  함**을 유의하자.

## void vehicleInit()

ROS 관련 변수에 대한 초기화가 모든 생성자에서 공통적으로 필요하므로 중복되는 코드를 줄이기 위해 함수화하여 구현하였다.&#x20;

```cpp
void Vehicle::vehicleInit()
{
	setpoint_global_pub_ = nh_.advertise<mavros_msgs::GlobalPositionTarget>
	    ("mavros/setpoint_position/global", 10);
	setpoint_local_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
	    ("mavros/setpoint_position/local", 10);
	setpoint_vel_pub_ = nh_.advertise<geometry_msgs::Twist>
	    ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

	state_sub_ = nh_.subscribe("mavros/state", 10, &Vehicle::stateCB, this);
	battery_sub_ = nh_.subscribe("mavros/battery", 10, &Vehicle::batteryCB, this);
	home_sub_ = nh_.subscribe
	    ("mavros/home_position/home", 10, &Vehicle::homeCB, this);
	local_pos_sub_ = nh_.subscribe
	    ("mavros/local_position/pose", 10, &Vehicle::localPositionCB, this);
	global_pos_sub_ = nh_.subscribe
	    ("mavros/global_position/global", 10, &Vehicle::globalPositionCB, this);
	turn_yaw = nh_.subscribe("/turn",10, &Vehicle::turnCB,this);

	arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
	    ("mavros/cmd/arming");
	set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	set_home_client_ = nh_.serviceClient<mavros_msgs::CommandHome>
	    ("mavros/cmd/set_home");
	takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>
	    ("mavros/cmd/takeoff");
	land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
	
	multi_arming_sub_ = nh_mul_.subscribe("arming", 10, &Vehicle::multiArming, this);
	multi_set_mode_sub_ = nh_mul_.subscribe
	    ("set_mode", 10, &Vehicle::multiSetMode, this);
	multi_set_home_sub_ = nh_mul_.subscribe
	    ("set_home", 10, &Vehicle::multiSetHome, this);
	multi_takeoff_sub_ = nh_mul_.subscribe
	    ("takeoff", 10, &Vehicle::multiTakeoff, this);
	multi_land_sub_ = nh_mul_.subscribe("land", 10, &Vehicle::multiLand, this);

	ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " instance generated");
}
```

**arming\_client\_, set\_mode\_client\_, set\_home\_client\_, takeoff\_client\_, land\_client\_**는 각각의 서비스를 요청하기 위한 **서비스 클라이언트 임을 유의**하라. 예를 들어 arming\_client\_는 Arm 또는 Disarm을 요청하기위해 arming() 함수 내에서 사용된다.&#x20;

**multi\_arming\_sub, multi\_set\_mode\_sub\_, multi\_set\_home\_sub\_, multi\_takeoff\_sub\_, multi\_land\_sub\_**은 군집 드론 제어시 각각의 서비스를 각 드론에게 동시에 전달하기 위해 1:N으로 설계된 토픽 subscriber이다. 원래 각각의 서비스는 바로 위에 언급된 바와  같이 서비스로 구현되어 있으나 ROS에서 서비스는 1:1 간의 비주기적 데이터 교환을 위해 구현 되어있기 때문에, **1:N으로 구독되는 형태를 위해 토픽으로 구현**하였다. 또한 다른 subscriber나 serviceClient와 달리 **nh\_mul\_이라는 군집명령을 위한 다른 NodeHandle이 사용되었음을 유의**하라.

## \~Vehicle()

Vehicle 객체가 소멸할때 호출 되는 함수. 코드는 아래와 같다. ROS 관련 변수들을 안정적으로 종료시켜주기위한 함수인 release()를 호출한 뒤 종료된다. release() 함수는 여기서만 호출되기 때문에 사실 굳이 함수화 하여 구현할 필요는 없다.

```cpp
Vehicle::~Vehicle()
{
	release();
}
```

## void release()

ROS 관련 변수를 안정적으로 종료시켜 주기 위해 구현된 함수. 꼭 이렇게 각각의 토픽이나 서비스에 연결된 Subscriber나 seviceClient를 안정적으로 종료시켜주지 않아도 프로세스가 종료 될 때 자동을 종료 될 수 있도록 구현되어 있다. 다만 작성자는 혹시 발생할 예외상황을 대비하기 위해 아래와 같이 구현하였다.

```cpp
void Vehicle::release()
{
	setpoint_global_pub_.shutdown();
	setpoint_local_pub_.shutdown();
	setpoint_vel_pub_.shutdown();

	state_sub_.shutdown();
	battery_sub_.shutdown();
	home_sub_.shutdown();
	local_pos_sub_.shutdown();
	global_pos_sub_.shutdown();

	arming_client_.shutdown();
	set_mode_client_.shutdown();
	set_home_client_.shutdown();
	takeoff_client_.shutdown();
	land_client_.shutdown();

	multi_arming_sub_.shutdown();
	multi_set_mode_sub_.shutdown();
	multi_set_home_sub_.shutdown();
	multi_takeoff_sub_.shutdown();
	multi_land_sub_.shutdown();
}
```

---
description: Swarm 클래스를 통해 객체가 생성되거나 소멸될때 실행되는 함수
---

# 생성자 / 소멸자

## SwarmVehicle()

아래와 같은 코드로 이 API를 사용하는 개발자가 Swarm 객체를 생성시 실수로 인자를 넣어주지 않은 것을 막기 위해 명시적으로 삭제하였다.

```cpp
SwarmVehicle() = delete;
```

## SwarmVehicle(ros::NodeHandle &, const std::string &, const int &)

```cpp
SwarmVehicle(ros::NodeHandle &, const std::string &swarm_name = "camila", 
    const int &num_of_vehicle = 1);
```

헤더파일에 명시되어 있는 군집 이름의 기본값 및 군집 대수의 기본값

```cpp
SwarmVehicle::SwarmVehicle(ros::NodeHandle &nh_global, 
	const std::string &swarm_name, const int &num_of_vehicle)
	: swarm_name_(swarm_name),
	  num_of_vehicle_(num_of_vehicle),
	  nh_(ros::NodeHandle()),
	  nh_mul_("multi"),
	  nh_global_(nh_global),
	  swarm_target_local_(tf2::Vector3(0, 0, 0)),
	  multi_setpoint_publish_flag_(false),
	  target_changed_flag_(false)
{
	VehicleInfo vehicle_info_[num_of_vehicle_];
	camila_.reserve(num_of_vehicle_);
	offset_.reserve(num_of_vehicle_);
	scen_hex_.reserve(num_of_vehicle_);

	for (int i = 0; i < num_of_vehicle_; i++)
	{
		vehicle_info_[i].vehicle_id_ = i + 1;
		vehicle_info_[i].vehicle_name_ = swarm_name_ + std::to_string(i + 1);
		camila_.push_back(Vehicle(nh_mul_, nh_global_, vehicle_info_[i]));
	}

	swarmServiceInit();
}
```

초기화 목록을 통해 각각의 변수를 초기화 하고, num\_of\_vehicle\_ 변수값에 따라 Vehicle 타입의 std::vector를 생성한다. 그리고 군집의 이름 뒤에 숫자를 붙여서 각각의 드론 이름을 초기화 한다.

## SwarmVehicle(const SwarmVehicle &)

```cpp
SwarmVehicle::SwarmVehicle(const SwarmVehicle &rhs)
	: swarm_name_(rhs.swarm_name_),
	  num_of_vehicle_(rhs.num_of_vehicle_),
	  nh_(rhs.nh_),
	  nh_mul_(rhs.nh_mul_),
	  nh_global_(rhs.nh_global_),
	  swarm_target_local_(tf2::Vector3(0, 0, 0)),
	  multi_setpoint_publish_flag_(rhs.multi_setpoint_publish_flag_),
	  target_changed_flag_(rhs.target_changed_flag_)
{
	VehicleInfo vehicle_info_[num_of_vehicle_];
	camila_.reserve(num_of_vehicle_);
	offset_.reserve(num_of_vehicle_);
	scen_hex_.reserve(num_of_vehicle_);

	std::vector<Vehicle>::const_iterator it;
	for (it = rhs.camila_.begin(); it != rhs.camila_.end(); it++)
		camila_.push_back(*it);

	nh_ = rhs.nh_;
	nh_mul_ = rhs.nh_mul_;
	nh_global_ = rhs.nh_global_;

	swarmServiceInit();

	*this = rhs;
}
```

std::vector와 같은 템플릿 라이브러리를 사용하기 위해 구현 한 대입 생성자. 대입 된 군집 정보를 바탕으로 군집 객체를 생성한다.&#x20;

{% hint style="info" %}
ros::NodeHandle:
{% endhint %}

## const SwarmVehicle \&operator=(const SwarmVehicle &)

```cpp
const SwarmVehicle &SwarmVehicle::operator=(const SwarmVehicle &rhs)
{
	if (this == &rhs)
		return *this;

	swarm_name_ = rhs.swarm_name_;
	num_of_vehicle_ = rhs.num_of_vehicle_;

	std::vector<Vehicle>().swap(camila_);
	camila_.reserve(num_of_vehicle_);
	offset_.reserve(num_of_vehicle_);
	scen_hex_.reserve(num_of_vehicle_);

	std::vector<Vehicle>::const_iterator it;
	for (it = rhs.camila_.begin(); it != rhs.camila_.end(); it++)
		camila_.push_back(*it);

	nh_ = rhs.nh_;
	nh_mul_ = rhs.nh_mul_;
	nh_global_ = rhs.nh_global_;

	swarmServiceInit();

	return *this;
}
```

## void swarmServiceInit()

```cpp
void SwarmVehicle::swarmServiceInit()
{
	multi_setpoint_local_server_ = nh_.advertiseService("multi_setpoint_local", 
		&SwarmVehicle::multiSetpointLocal, this);
	multi_setpoint_global_server_ = nh_.advertiseService("multi_setpoint_global", 
		&SwarmVehicle::multiSetpointGlobal, this);
	goto_vehicle_server_ = nh_.advertiseService("goto_vehicle", 
		&SwarmVehicle::gotoVehicle, this);
	trigger_sub_ = nh_.subscribe("/trigger", 10, &SwarmVehicle::triggerCB, this);
}
```

## void release()

```cpp
void SwarmVehicle::release()
{
	std::vector<Vehicle>().swap(camila_);
	std::vector<tf2::Vector3>().swap(offset_);
	std::vector<uint8_t>().swap(scen_hex_);
	multi_setpoint_global_server_.shutdown();
	multi_setpoint_local_server_.shutdown();
	goto_vehicle_server_.shutdown();
}
```

## \~SwarmVehicle();

```cpp
SwarmVehicle::~SwarmVehicle()
{
	release();
}
```

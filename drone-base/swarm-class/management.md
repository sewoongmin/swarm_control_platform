# Management 인터페이스

## void setSwarmInfo(const std::string &, const int &)

```cpp
void SwarmVehicle::setSwarmInfo(const std::string &swarm_name, 
	const int &num_of_vehicle)
{
	swarm_name_ = swarm_name;
	num_of_vehicle_ = num_of_vehicle;

	release(); // service 및 camila vector 초기화

	camila_.reserve(num_of_vehicle_);
	VehicleInfo vehicle_info_[num_of_vehicle_];

	for (int i = 0; i < num_of_vehicle_; i++)
	{
		vehicle_info_[i].vehicle_id_ = i + 1;
		vehicle_info_[i].vehicle_name_ = swarm_name_ + std::to_string(i + 1);
		camila_.push_back(Vehicle(nh_mul_, nh_global_, vehicle_info_[i]));
	}

	multi_setpoint_local_server_ = nh_.advertiseService("multi_setpoint_local", 
		&SwarmVehicle::multiSetpointLocal, this);
	multi_setpoint_global_server_ = nh_.advertiseService("multi_setpoint_global", 
		&SwarmVehicle::multiSetpointGlobal, this);
	goto_vehicle_server_ = nh_.advertiseService("goto_vehicle", 
		&SwarmVehicle::gotoVehicle, this);
}
```

## std::string getSwarmInfo() const

```cpp
std::string SwarmVehicle::getSwarmInfo() const
{
	return swarm_name_;
}
```

## void addVehicle(const VehicleInfo &)

```cpp
void SwarmVehicle::addVehicle(const VehicleInfo &vehicle_info)
{
	camila_.push_back(Vehicle(nh_mul_, nh_global_, vehicle_info));
	num_of_vehicle_++;
}
```

## void deleteVehicle(const VehicleInfo &)

```cpp
void SwarmVehicle::deleteVehicle(const VehicleInfo &vehicle_info)
{
	for (iter_ = camila_.begin(); iter_ != camila_.end(); iter_++)
	{
		VehicleInfo temp = iter_->getInfo();
		if ((temp.vehicle_id_ == vehicle_info.vehicle_id_) && 
			(temp.vehicle_name_ == vehicle_info.vehicle_name_))
		{
			iter_ = camila_.erase(iter_);
			ROS_INFO_STREAM("Delete vehicle : sys_id : " << temp.vehicle_id_ 
				<< ", name : " << temp.vehicle_name_);
			num_of_vehicle_--;
			break;
		}
	}
}
```

## void showVehicleList() const

```cpp
void SwarmVehicle::showVehicleList() const
{
	for (auto &vehicle : camila_)
	{
		VehicleInfo info = vehicle.getInfo();
		ROS_INFO_STREAM(info.vehicle_id_ << " " << info.vehicle_name_);
	}
}
```


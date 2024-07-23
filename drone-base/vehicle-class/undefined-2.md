---
description: >-
  군집드론을 제어하기 위해서는  각각의 드론을 제어하기 위한 통합된 좌표계가 필요하다. 그러한 통합좌표계 내에서 드론의 회피나 Path
  Planing을 위해 사용되는 인터페이스들이다.
---

# 군집을 위한 통합좌표계 구현 및 드론간 충돌회피 방지를 위한 인터페이스

## 적용된 드론간 충돌회피를 위한 Potential Field 알고리즘 계산 개념

![충돌 회피를 위한 Potential Field 알고리즘과 적용](<../../.gitbook/assets/Potential Field 계산.png>)

## void setPos(const tf2::Vector3 &)

```cpp
void Vehicle::setPos(const tf2::Vector3 &pos)
{
	pos_ = pos;
}
```

군집드론시 통합좌표계 상에서 자신의 좌표를 계산하기 위해 1번 드론의 Home GPS좌표로 부터 자신의 GPS 값의 차이를 통해 로컬 enu 좌표계로 변환한 좌표계가 pos\_이다.&#x20;

`pos_`변수는 `tf2::Vector3` 타입의 변수로 x, y, z의 값을 가지는 벡터이다.

## tf2::Vector3 getPos() const

```cpp
tf2::Vector3 Vehicle::getPos() const
{
	return pos_;
}
```

통합 좌표계 내의 자신의 좌표를 리턴하는 인터페이

## void setSumOfSp(const tf2::Vector3 &)

```cpp
void Vehicle::setSumOfSp(const tf2::Vector3 &sum_sp)
{
	sum_sp_ = sum_sp;
}
```

주변의 다른 드론들이나 장애물과 멀어지려는 힘이 Separate 벡터이다. 주변의 하나 이상의 장애물에 대해서 Separate 벡터를 합쳐서 가야할 방향을 제시하는 벡터가 sum\_sp\_이다&#x20;

`sum_sp` 역시 `tf2::Vector3` 타입이다.&#x20;

## tf2::Vector3 getSumOfSp() const

```cpp
tf2::Vector3 Vehicle::getSumOfSp() const
{
	return sum_sp_;
}
```

장애물로부터 멀어지려는 힘의 벡터의 합인 sum\_sp\_를 리턴해주는 인터페이

## void setErr(const tf2::Vector3 &)

```cpp
void Vehicle::setErr(const tf2::Vector3 &err)
{
	err_ = err;
}
```

각 드론의 최종 목표지점(tar\_local\_)과 현재위치와의 차이가 err\_ 벡터이다.&#x20;

`err_` 또한 `tf2::Vector3` 타입이다.&#x20;

## tf2::Vector3 getErr() const

```cpp
tf2::Vector3 Vehicle::getErr() const
{
	return err_;
}
```

err\_ 벡터를 리턴해 주는 인터페이스

## void setSetpointPos(const tf2::Vector3 &)

```cpp
void Vehicle::setSetpointPos(const tf2::Vector3 &setpoint)
{
	setpoint_pos_.setX(cur_local_.pose.position.x + setpoint.getX());
	setpoint_pos_.setY(cur_local_.pose.position.y + setpoint.getY());
	setpoint_pos_.setZ(cur_local_.pose.position.z + setpoint.getZ());
}
```

err\_ 벡터와 sum\_sp\_벡터가 pid 게인에 의해 계산되어 더해진 것이 setpoint로 들어오고 자신의 좌표에 setpoint 값을 더하여 장애물을 피하고 목표지점을 향해 갈수 있는 근거리 경로계획지점으로 setpoint\_pos\_에 저장한다.

`setpoint_pos_` 또한 `tf2::Vector3` 타입이다. &#x20;

## tf2::Vector3 getSetpointPos() const

```cpp
void Vehicle::setSetpointPos(const tf2::Vector3 &setpoint)
{
	setpoint_pos_.setX(cur_local_.pose.position.x + setpoint.getX());
	setpoint_pos_.setY(cur_local_.pose.position.y + setpoint.getY());
	setpoint_pos_.setZ(cur_local_.pose.position.z + setpoint.getZ());
}
```

근거리 경로계획의 좌표인 setpoint\_pos\_값을 리턴해주는 인터페이스

# Parameter

## 파라미터

### 경로

```bash
drone_base/config/config.yaml
```

### 내용

```yaml
# set velocity control

takeoff_alt: 1.5

# set velocity control PID gain
pid:
  kp: 0.8
  ki: 0.0
  kd: 0.2

# distance between drones
spacing: 3.0
distance_x: 3.0
distance_y: 3.0
scen6: false

scen_num: 57 # scenario 숫자 9
scen: '987'

# setpoint control params
setpoint:
  kp_seek: 1.5
  kp_sp: 2
  range_sp: 1.5
  separate: true
  vector_speed_limit: 1.5
  final_speed_limit: 0.75

```

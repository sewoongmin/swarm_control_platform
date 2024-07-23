# Example

## Qt Designer 로 간단한 GUI 만들기

### Qt Designer 실행

앞서 [Installation](installation.md#qt-desigener) 에 적혀 있는 대로 Qt Designer를 실행한다

### Dialog 만들기&#x20;

첫 화면에서 Dialog without Buttons 를 선택하면 아래와 같은 화면을 볼 수 있다. &#x20;

![](<../.gitbook/assets/image (1).png>)

왼쪽의 Widget Box에서 Input Widget 의 Line Edit 과 Push Button 등을 이용하여 간단한 Gui를 구성한다.&#x20;

![](<../.gitbook/assets/image (4).png>)

위 화면은 하나의 Vertical Layout에 위 아래로 Horizontal Layout을 집어 넣어서 버튼 4개를 배치한 것이다. 옆에는 글자를 볼 수 있는 Text Browser 를 배치하였다.&#x20;

### UI 저장

ctrl + s 를 통해 편집한 GUI를 적당한 경로에 저장 후 python 파일을 작성하여 연결한다.

### Python file 작성

```python
#! /usr/bin/env python

import sys
import PyQt5
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import uic

import rospy, rospkg
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import BatteryState

rospack = rospkg.RosPack()
vehicle_monitor = rospack.get_path('vehicle_monitor')
monitor_UI = vehicle_monitor +'/vehicle_monitor.ui'

class MainDialog(QDialog):
    def __init__(self, parent=None, flags=Qt.WindowStaysOnTopHint):
        super().__init__(parent=parent, flags=flags)
        uic.loadUi(monitor_UI, self)

        self.arm_pushButton.clicked.connect(lambda state, data  = True : self.arming(state, data))
        self.disarm_pushButton.clicked.connect(lambda state, data  = False : self.arming(state, data))
        self.offboard_pushButton.clicked.connect(lambda state, mode = 'offboard' : self.setMode(state, mode))
        self.manual_pushButton.clicked.connect(lambda state, mode = 'stabilized' : self.setMode(state, mode))
        
        self.timer = QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.stateDisplay)
        self.timer.start()
        
        self._battery = 0
        self._tar_linear_x = 0
        self._tar_angular_z = 0
        self._cur_linear_x = 0
        self._cur_angular_z = 0

        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        # rospy.wait_for_message('mavros/state', State)
        # rospy.wait_for_message('mavros/battery', BatteryState)
        # rospy.wait_for_message('mavros/setpoit_velocity/cmd_vel_unstamped', Twist)
        rospy.Subscriber('mavros/state', State, self.stateSub)
        rospy.Subscriber('mavros/battery', BatteryState, self.batterySub)
        rospy.Subscriber('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, self.tarVelocitySub)
        rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, self.curVelocitySub)
    
    def arming(self, state, data):
        try:
            arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            respose = arming_client(data)
        except rospy.ServiceException as e:
            alert = QMessageBox()
            alert.setText("Service call failed : " + e)
            alert.exec_()
    
    def setMode(self, state, mode):
        try:
            set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
            res = set_mode_client(0, mode)
        except rospy.ServiceException as e:
            alert = QMessageBox()
            alert.setText("Service call failed : " + e)
            alert.exec_()

    def stateDisplay(self):
        connected = 'Connected : ' + str(self._connected)
        armed = 'Armed : ' + str(self._armed)
        mode = 'Mode : ' + self._mode
        battery = 'Battery : ' + str(round(self._battery, 0)) + '%'
        cur_velocity_linear = 'Linear : ' + str(round(self._cur_linear_x, 2)) + ' m/s'
        cur_velocity_angular = 'Angula : ' + str(round(self._cur_angular_z, 2)) + ' rad/s'
        cur_velocity = 'Current Velocity\n    ' + cur_velocity_linear + '\n    ' + cur_velocity_angular
        tar_velocity_linear = 'Linear : ' + str(round(self._tar_linear_x, 2)) + ' m/s'
        tar_velocity_angular = 'Angula : ' + str(round(self._tar_angular_z, 2)) + ' rad/s'
        tar_velocity = 'Target Velocity\n    ' + tar_velocity_linear + '\n    ' + tar_velocity_angular
        self.textBrowser.setText(connected + '\n' + armed + '\n' + mode + '\n' + battery + '\n\n' + cur_velocity + '\n' + tar_velocity)

    def stateSub(self, msg):
        self._connected = msg.connected
        self._armed = msg.armed
        self._mode = msg.mode
    
    def batterySub(self, msg):
        self._battery = msg.percentage
        self._battery *= 100

    def tarVelocitySub(self, msg):
        self._tar_linear_x = msg.linear.x
        self._tar_angular_z = msg.angular.z
    
    def curVelocitySub(self, msg):
        self._cur_linear_x = msg.twist.linear.x
        self._cur_angular_z = msg.twist.angular.z


rospy.init_node("vehicle_monitor")
app = QApplication(sys.argv)
main_dialog = MainDialog()
main_dialog.show()
app.exec_()
rospy.spin()
```

위 코드는 차량제어를 위해 만든 UI와 rosservice 명령어를 이어준 간단한 코드이다.

### 코드 분석

```python
import sys
import PyQt5
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import uic

import rospy, rospkg
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import BatteryState

rospack = rospkg.RosPack()
vehicle_monitor = rospack.get_path('vehicle_monitor')
monitor_UI = vehicle_monitor +'/vehicle_monitor.ui'
```

PyQt5 를 적절 히 사용하기 위해 필수 라이브러리들을 추가해 주었다. 또한 ros 사용을 위해 rospy 와 mavros를 통해 arming과 모드 변경을 위한 서비스도 추가해 주었다. 코드 내에서 .ui 파일의 경로를 쉽게 찾기 위해 rospkg가 사용되었고 이를 통해 QT Designer 를 통해 제작한 .ui 파일을 불러오는 것을 확인 할 수 있다.&#x20;



```python
class MainDialog(QDialog):
    def __init__(self, parent=None, flags=Qt.WindowStaysOnTopHint):
        super().__init__(parent=parent, flags=flags)
        uic.loadUi(monitor_UI, self)

        self.arm_pushButton.clicked.connect(lambda state, data  = True : self.arming(state, data))
        self.disarm_pushButton.clicked.connect(lambda state, data  = False : self.arming(state, data))
        self.offboard_pushButton.clicked.connect(lambda state, mode = 'offboard' : self.setMode(state, mode))
        self.manual_pushButton.clicked.connect(lambda state, mode = 'stabilized' : self.setMode(state, mode))
        
        self.timer = QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.stateDisplay)
        self.timer.start()
        
        self._battery = 0
        self._tar_linear_x = 0
        self._tar_angular_z = 0
        self._cur_linear_x = 0
        self._cur_angular_z = 0

        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        # rospy.wait_for_message('mavros/state', State)
        # rospy.wait_for_message('mavros/battery', BatteryState)
        # rospy.wait_for_message('mavros/setpoit_velocity/cmd_vel_unstamped', Twist)
        rospy.Subscriber('mavros/state', State, self.stateSub)
        rospy.Subscriber('mavros/battery', BatteryState, self.batterySub)
        rospy.Subscriber('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, self.tarVelocitySub)
        rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, self.curVelocitySub)
```

MainDialog 클래스를 정의 해준다.  생성자를 통해 필요한 매개변수 설정들을 해주고 uic.loadUI() 함수를 통해 앞서 적은 .ui파일의 경로를 입력해 주었다. `Qt.WindowsStayOnTopHint` 를 통해 창이 항상 제일 앞에 있을 수 있도록 해주었다.

각각의 버튼에 동작을 위한 함수들을 연결 시켜 주기 위해 Qt Designer 에서 만들 때 작성 해 준 버튼 이름을 이용하여 Button\_name.clicked.connect(self.FunctionName) 과 같은 방법으로 코드를 작성한다.

State 를 주기적으로 업데이트 하여 디스플레이 해주기 위해서 QTimer() 를 사용하였다. 저 부분을 통해 100ms 마다 stateDisplay 함수가 실행된다.

```python
rospy.init_node("vehicle_monitor")
app = QApplication(sys.argv)
main_dialog = MainDialog()
main_dialog.show()
app.exec_()
rospy.spin()
```

이 QUI 자체가 하나의 ros node 로써 동작 할 수 있도록 node를 초기화 해주고 앞서 작성한 GUI 클래스를 불러오기 위한 필수 코드 들을 작성해 준다.&#x20;

![](<../.gitbook/assets/image (35).png>)

위 이미지는 최종 완성된 간단한 GUI 툴 이다. [여기](https://github.com/ICSL-hanyang/vehivle\_monitor)를 통해 다운 받을 수 있다.

## 참고 자료

### [Python Tutorial - qt designer python](https://pythonbasics.org/qt-designer-python/#Prerequisites)

Qt Designer 의 설치와 간단한 예제 까지 제공

### [ Youtube 컴노니 Gui - 계산기 만들기 예제](https://www.youtube.com/playlist?list=PLh665u8WZRR1d1hhLuZQThLhZbaOE5Whf)

PyQt5를 통해 Gui로 계산기를 구현 하면서 버튼과 화면 등의 기본적인 레이아웃을 다뤄 볼 수 있다.

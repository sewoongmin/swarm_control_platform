---
description: 준비된 패키지들을 사용하기 앞서 더 디텍션 성능을 높이기 위한 사전작업
---

# 사전준비(캘리브레이션)

## 카메라 캘리브레이션

### 카메라 모델 (라즈베리파이 전용)

![](<../../.gitbook/assets/image (38).png>)

#### 상세 스펙

* 500만 화소
* 해상도 2952x1944
* 영상해상도 : 1080p30, 720p60, 640x480p60/90
* CMOS 크기 : 1/4 inch
* 조리개값(F) : 2.35
* 초점거리 : 조절가능
* 화각 : 160도
* 크기 : 25mm x 24mm
* 적외선 촬영 불가
* 라즈베리파이 전용 ( **JETSON NANO에서 사용 불가** )

#### 구매는 [여기](https://www.devicemart.co.kr/goods/view?no=1362051)서 가능

### JETSON NANO 호환용 광각 카메라 모듈

![](<../../.gitbook/assets/image (13).png>)

#### 상세스펙

* 해상도 : 3280x2464
* 크기 : 25mmx24mm
* CMOS 크기 : 1/4인치
* 조리개 : F2.35
* 초점거리 3.15mm (조절 가능해 보임)
* 화각 : 160도
* 왜곡 : < 14.3%

#### 구매는 [여기](https://www.devicemart.co.kr/goods/view?no=12538383)서 가능

위의 두 제품은 거의 비슷해 보이나 동일한 제품이 아니므로 **Jetson에 사용할지 라즈베리파이에 사용할지에 따라서 잘 구별해서 살것!**&#x20;

( 보통 Jetson Nano에서 사용가능 한 카메라는 라즈베리파이에서도 사용 가능하나 위 제품은 테스트 해보지 않음)

{% hint style="danger" %}
대회를 준비하다 보니 여러 팀들이 자주 사용하는 카메라가 있었다. 그 카메라는 아래 이미지와 같고 [여기](https://www.logitech.com/ko-kr/product/c930e-webcam)서 확인 가능하다.

대회 준비시 다음엔 꼭 이 카메라를 사용하여 나가는 것이 좋을 것 같다.&#x20;
{% endhint %}

&#x20;

![](<../../.gitbook/assets/image (22).png>)

{% hint style="danger" %}
카메라 파라미터 튜닝도 굉장히 중요하지만 위치도 중요하다. 대부분의 팀이 터틀봇의 중•하단에 설치하였고 실제로 할 경우에는 여러 위치에서 테스트하여 가장 잘 되는 위치를 찾는 것을 추천한다.
{% endhint %}

### UVC-Camera node 설정

여기에서 언급한 바와 같이 raspicam\_node의 지원이 중단되어 ros에서 일반적으로 사용하는

* ros-melodic-uvc-camera

패키지를 사용함. uvc-camera 패키지의 지원이 끝나 libuvc-camera 사용이 권장되지만 사용할때 sudo 권한을 줘야 하는 등 귀찮은 설정들이 있어서 uvc-camera를 사용함.&#x20;

카메라 모델 별로 설정가능한 파라미터가 다른지 광각카메라와 USB 카메라의 파라미터가  상황 발생

![](https://firebasestorage.googleapis.com/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-Lq1PFS\_JV6OCZktAP2m%2Fuploads%2FdXcJEHfWed7M3Qwj7Mgg%2Ffile.jpeg?alt=media)

{% hint style="info" %}
아래 카메라의 파라미터들은 해당 컴퓨터에 연결 후 uvc\_camera를 실행 했을 때 위의 이미지와 같이 파라미터와 범위가 표기 되는 것을 기록 한 것이다.
{% endhint %}

#### 노트북 카메라 파라미터

* Brightness : -64 to 64
* Contrast : 0 to 95
* Saturation : 0 to 100
* Hue : -2000 to 2000
* White Balance Temperature : 0 to 1 (0 = auto )
* Gamma : 100 to 300
* Gain : 1 to 8
* White Balance Temperature : 2800 to 6500
* Sharpness : 1 to 7
* Backlight Compensation : 0 to 3
* Exposure 0 to 3 (0 = auto)
* Exposure (absolute) : 0 to 625
* Exposure Auto Priority : 0 to 1



#### USB 카메라 파라미터

* Brightness : -64 to 64
* Contrast : 0 to 64
* Saturation : 0 to 128
* Hue : -40 to 40
* White Balance Temperature : 0 to 1 (0 = auto )
* Gamma : 72 to 500
* Gain : 0 to 100
* White Balance Temperature : 2800 to 6500
* Sharpness : 0 to 6
* Backlight Compensation : 0 to 2
* Exposure 0 to 3 (0 = auto)
* Exposure (absolute) : 0 to 5000
* Exposure Auto Priority : 0 to 1



#### 라즈베리파이용 초광각 카메라 파라미터

* Brightness : 0 to 100
* Contrast : -100 to 100
* Saturation : -100 to 100
* Red Balance : 1 to 7999&#x20;
* Blue Balance :1 to 7999
* Horizontal Flip : 0 to 1
* Vertical Flip : 0 to 1
* Sharpness : -100 to 100
* Exposure 0 to 3 (0 = auto)
* Exposure Time : 0 to 10000
* Exposure Dynamic Framerate : 0 to 1

![](https://firebasestorage.googleapis.com/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-Lq1PFS\_JV6OCZktAP2m%2Fuploads%2FSoaABBuwyfxHUaxsPjZ8%2Ffile.jpeg?alt=media)

위의 이미지는 앞서 언급한 uvc\_camera를 실행 시킬 때 카메라의 파라미터를 변경하여 실행시키는 런치파일의 예시이다.

위 이미지에서 보면 brightness가 10, contrast 가 100으로 설정되어 있는데 이 파라미터들은 실제 대회에서 사용된 라즈베리용 초광각 카메라의 파라미터 설정 값이다.&#x20;

**밝기 범위가 0에서 100인데 10으로 설정**했기 때문에 카메라를 통해 들어오는 이미지는 전반적으로 어둡고 **contrast를 최대로 올려 색이 다른 부분들을 더욱 강조** 될 수 있도록 했다. **밝기를 낮춘 이유는 차선을 인식함에 있어서 노란색과 흰색을 구분할때 조명의 영향을 최소화 하기 위해**서 낮게 잡았다.&#x20;



## Intrinsic Calibration ( 라즈베리파이에서 라즈베리파이용 광각 카메라)

카메라의 초점 거리, 왜곡 등 카메라의 외부특성을 조절하기 위한 캘리브레이션 과정이다.

### 라즈베리파이 설정

{% tabs %}
{% tab title="turtlebot3_autorace_camera_pi.launch " %}
```markup
  
<launch>
  <remap from="camera_info" to="camera/camera_info" />
  <remap from="image_raw" to="camera/image" />
  <remap from="image_raw/compressed" to="camera/image/compressed" />
  <remap from="image_raw/compressed/parameter_descriptions" to="camera/image/compressed/parameter_descriptions" />
  <remap from="image_raw/compressed/parameter_updates" to="camera/image/compressed/parameter_updates" />
  <remap from="set_camera_info" to="camera/set_camera_info" />
  <node pkg="uvc_camera" type="uvc_camera_node" name="camera" output="screen">
    <rosparam command="load" file="$(find turtlebot3_autorace_construction_camera)/calibration/camera_calibration/camera.yaml" />
    <param name="camera_info_url" value="package://turtlebot3_autorace_construction_camera/calibration/intrinsic_calibration/camerav2_320x240_30fps.yaml"/>
    <param name="width" value="320"/>
    <param name="height" value="240"/>
    <param name="fps" value="30"/>
    <param name="frame_id" value="camera"/>
    <param name="brightness" value="10" />
    <param name="contrast" value="100" />
    <param name="saturation" value="0" />
    <param name="sharpness" value="70" />
  </node>
</launch>
```
{% endtab %}

{% tab title="camera_with_robot.launch" %}
```markup
<launch>
    <include file="$(find turtlebot3_autorace_construction_camera)/launch/turtlebot3_autorace_camera_pi.launch" />
    <include file="$(find turtlebot3_bringup)/launch/turtlebot3_robot.launch" />
</launch>

```
{% endtab %}
{% endtabs %}

위의 코드들은 모두 터틀봇에서 실행되는 런치파일이다. turtlebot3\_autorace\_camera\_pi.launch 파일은 기존의 라즈파이캠 노드를 지우고 uvc\_camera 패키지로 변경해 준다. **실제 실행은 camera\_with\_robot.launch 하나만 라즈베리파이에서 실행**하여 하나의 런치 파일도 카메라 패키지와 모터구동에 사용되는 패키지를 동시에 열어준다.&#x20;

연결되는 다른 패키지들과의 토픽네임을 통일시켜주기 위해 remap을 사용하여 토픽이름을 연결시키고 파라미터가 적용 될 수 있게 해준다.&#x20;

```
roslaunch turtlebot3_autorace_construction_camera camera_with_robot.launch
```

ssh로 라즈베리파이에 접속하여 위 명령어를 입력하여 카메라 패키지를 실행시켜 주자



### 노트북 설정

캘리브레이션에 사용할 **ROS 패키지는 camera\_calibration 이라는 공식 패키지**를 사용한다. `sudo apt install ros-melodic-camera-calibration` 명령어로 직접 설치해도 되지만 turtlebot3\_autorace\_2020 패키지에 포함되어 있다.&#x20;

앞으로 Intrinsic, Extrinsic, Detection Calibration을 해야 하기 때문에 이것들을 환경변수에 추가하고 이 값이 Calibration 일때만 Calibration 과정을 수행한다.&#x20;

![](https://firebasestorage.googleapis.com/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-Lq1PFS\_JV6OCZktAP2m%2Fuploads%2F62LPHZNBUvklefxXddfU%2Ffile.jpeg?alt=media)

위의 이미지와 마찬가지로 .bashrc 에 환경변수를 추가하고 인트린식 캘리브레이션을 위해 `AUTO_IN_CALIB = calibration` 으로 변경 후 터미널에서 `source ~/.bashrc`를 실행 해준다.

```
sourch ~/.bashrc
roslaunch turtlebot3_autorace_construction_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
```

{% hint style="info" %}
위 명령어들은 라즈베리파이에서 camera\_with\_robot.launch 파일이 실행되고 있다고 가정하고 노트북에서 실행되는 명령어들이다.&#x20;
{% endhint %}

![](https://firebasestorage.googleapis.com/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-Lq1PFS\_JV6OCZktAP2m%2Fuploads%2FMGu3e7LMADsyUsni0Qo9%2Ffile.jpeg?alt=media)

실행을 하게되면 위와 같은 화면을 마주할수 있는데 이때 체커보드가 사용된다.

* 체커보드는 **turtlebot3\_autorace\_camera/data/checkerboard\_for\_calibration.pdf** 에 저장되어 있다
* 캘리브레이션 된 파라미터 값은  **turtlebot3\_autorace\_camera/launch/turtlebot3\_autorace\_intrinsic\_camera\_calibration.launch** 에 기존에 있던 값을 바꾸어 저장하면 된다.
* 카메라 캘리브레이션에 대한 자세한 정보는 [여기](http://wiki.ros.org/camera\_calibration)서 확인 가능하다.&#x20;
* uvc\_camera를 통해 밝기를 낮추고 캘리브레이션을 하면 체커 보드 인식이 잘 안될수 있으니 기본파라미터상태에서 캘리브레이션을 하는 것을 추천한다
* 체커보드는 출력후 평평한 판에 붙여서 하는 것을 추천한다.

캘리브레이션을 위해 평평한 판에 출력한 체커보드를 붙이고 카메라 화면에 위, 아래, 좌, 우로 움직이고 회전도 하다보면 어느정도 캘리브레이션이 되어서 아래 이미지와 같이 캘리브레이트 버튼이 활성화 된다

![](https://firebasestorage.googleapis.com/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-Lq1PFS\_JV6OCZktAP2m%2Fuploads%2F8L2ai8X1QMGqbnaIGXL1%2Ffile.jpeg?alt=media)

그 상태에서 캘리브레이트 버튼을 누르고 잠시 기다리면 실행시킨 터미널에서 계산된 파라미터 값이 뜨는 것을 아래와 같 볼 수 있다.

![](https://firebasestorage.googleapis.com/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-Lq1PFS\_JV6OCZktAP2m%2Fuploads%2FHZRCUFbVdqAiGJrwJShq%2Ffile.jpeg?alt=media)

캘리브레이션이 끝나면 캘리브레이션 화면에서도 세이브와 커밋버튼도 활성화 되는데 여기서 save 버튼을 통해 계산된 캘리브레이션 값을 파일로 저장할 수 있다.

![](https://firebasestorage.googleapis.com/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-Lq1PFS\_JV6OCZktAP2m%2Fuploads%2F8m5KzAFeIW3H5A2cvaUs%2Ffile.jpeg?alt=media)

저장 버튼을 눌렀을 때 위처럼 캘리브레이션 된 값이 /tmp/calibrationdata.tar.gz 로 저장 되고 압축을 풀고 파라미터 값을 turtlebot3\_autorace\_construction\_camera/calibration/intrinsic\_calibration/camerav2\_320x240\_30fps.yaml  에 아래와 같이 덮어 씌워주면 된다.

{% tabs %}
{% tab title="camerav3_320x240_30fps.yaml" %}
```yaml
image_width: 320
image_height: 240
camera_name: narrow_stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [167.24709,   0.     , 141.74334,
           0.     , 168.39658, 116.78567,
           0.     ,   0.     ,   1.     ]
camera_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.270722, 0.052308, -0.000257, 0.002980, 0.000000]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1., 0., 0.,
         0., 1., 0.,
         0., 0., 1.]
projection_matrix:
  rows: 3
  cols: 4
  data: [118.4591 ,   0.     , 140.15267,   0.     ,
           0.     , 138.8131 , 114.96196,   0.     ,
           0.     ,   0.     ,   1.     ,   0.     ]
```
{% endtab %}
{% endtabs %}

### Future Work

**현재 이미지 전송 속도를 위해 해상도를 320x240으로 줄여둔 상태**인데 표지판 인식을 잘하기 위해서는 **해상도를 좀 높여볼 가치가 있을 것 같다**. 그러나 차선 추적의 실시간 성을 유지하기 위해서는 해상도의 크기를 낮추거나 압축하여 속도를 높여야 하므로 해상도를 변경해가면서 **다양한 테스트가 필요**하다.



## Extrinsic Camera Calibration

여기서 말하는 extrinsic camera calibration은 차선 추적을 잘 하기 위해서 카메라의 이미지를 잘라 낼때 그 영역을 설정하는 것을 의미한다.

### 노트북 설정

사용된 **ROS Node 는 turtlebot3\_autorace\_construction\_camera의 image\_projection 노드이다.**&#x20;

앞으로 앞서 .bashrc에 작성했던 환경변수 중 `AUTO_EX_CALIB=calibration` 으로 변경 후 실행해야 한다.&#x20;

![](https://firebasestorage.googleapis.com/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-Lq1PFS\_JV6OCZktAP2m%2Fuploads%2FBjlbqf8ZGTG58HwZ6yES%2Ffile.jpeg?alt=media)

1. turtlebot3의 SBC에서 실행

```
roslaunch turtlebot3_autorace_construction_camera camera_with_robot.launch
```

2\. 노트북에서 실행 `Terminal 1`

```
roslaunch turtlebot3_autorace_construction_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
```

3\. 노트북에서 실행 `Terminal 2`

```
source ~/.bashrc
roslaunch turtlebot3_autorace_construction_camera turtlebot3_autorace_extrinsic_camera_calibration.launch 
```

#### 실행시 주의 사항

* `AUTO_IN_CALIB = action` 으로 설정되어야 한다.&#x20;
* `AUTO_EN_CALIB = calibration` 으로 설정되어야 한다.&#x20;

4\. 노트북에서 실행 `Terminal 3`

```
rqt
```

rqt plugins에서 visualization > Image view를 2개 켠다 이미지를 양쫑에 두고 각각을 다음의 토픽으로 열어준다.  `/camera/image_extrinsic_calib/compressed` ,  `/camera/image_projected_compensated` &#x20;

![](https://firebasestorage.googleapis.com/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-Lq1PFS\_JV6OCZktAP2m%2Fuploads%2FM8pxHf0Vn5OLzggrZ3sI%2Ffile.jpeg?alt=media)

왼쪽 이미지는 압축된 원본이미지에 잘라낼 영역을 정하는 파라미터인 top\_x, top\_y, bottom\_x, bottom\_y 값에 따라 사다리꼴로 잘라질 영역을 표시한 이미지이다.&#x20;

오른쪽 이미지는 해당 영역을 잘라내어 Projection 시킨 이미지이다. 이것에 대한 코드의 배경지식은 [여기](https://heromin.gitbook.io/drone-research/-Lq1lREaxFyRbZ-M2ezg/turtlebot-autorace-2020/undefined-3/undefined-2#undefined)에서 볼 수 있다.&#x20;

5\. 노트북에서 실행 `Terminal 4`

```
rosrun rqt_reconfigure rqt_reconfigure
```

6\. `/camera/image_projection` 탭에 있는 `top_x`, `top_y`, `bottom_x`, `bottom_y` 의 값을 변경해가면서 차선의 영역을 지정해 준다.&#x20;

![](<../../.gitbook/assets/image (44).png>)

![](https://firebasestorage.googleapis.com/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-Lq1PFS\_JV6OCZktAP2m%2Fuploads%2FWTmoWsejHsKagxHsGUyS%2Ffile.jpeg?alt=media)

차선 추적을 잘 하기 위해서는 어느정도 이상의 레인을 추적할 필요가 있다.&#x20;

{% hint style="info" %}
실제로 일정길이 이상의 차선을 추적하면 **짧은 구간의 차선을 추적하는 것보다 조명의 영향을 덜 받아서 신뢰도가 높아진**다. 하지만 **너무 길게 하면 현재의 길이 아닌 다른 루트의 차선이 보이는등의 영향**이 있을 있으므로 길이를 조절해 가면서 충분한 테스트가 필요하다.&#x20;
{% endhint %}

7\. 설정된 값은 `turtlebot3_autorace_construction_camera/calibration/extrinsic_calibration/projection.yaml` 에 덮어 씌워주면 된다.&#x20;

## Lane Detection Calibration

이 작업에서는 Projection 된 이미지에서 노란색 선과 흰색선을 잘 추출 할 수 있도록 hsv 파라미터를 변경하여 라인을 추출합니다.&#x20;

### 노트북 설정

사용된 **ROS Node 는 turtlebot3\_autorace\_construction\_detect의 detect\_lane 노드이다.**&#x20;

앞으로 앞서 .bashrc에 작성했던 환경변수 중 `AUTO_DT_CALIB=calibration` 으로 변경 후 실행해야 한다.&#x20;

![](https://firebasestorage.googleapis.com/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-Lq1PFS\_JV6OCZktAP2m%2Fuploads%2FyaAptvYiHAOT9WfQ3UDT%2Ffile.jpeg?alt=media)

1. turtlebot3의 SBC에서 실행

```
roslaunch turtlebot3_autorace_construction_camera camera_with_robot.launch
```

2\. 노트북에서 실행 `Terminal 1`

```
roslaunch turtlebot3_autorace_construction_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
```

3\. 노트북에서 실행 `Terminal 2`

```
roslaunch turtlebot3_autorace_construction_camera turtlebot3_autorace_extrinsic_camera_calibration.launch 
```

4\. 노트북에서 실행 `Terminal 3`

```
roslaunch turtlebot3_autorace_construction_detect turtlebot3_autorace_detect_lane.launch
```

#### 실행시 주의 사항

* `AUTO_IN_CALIB = action` 으로 설정되어야 한다.&#x20;
* `AUTO_EN_CALIB = action` 으로 설정되어야 한다.&#x20;
* `AUTO_DT_CALIB = calibration` 으로 설정되어야 한다.

4\. 노트북에서 실행 `Terminal 4`

```
rqt
```

rqt plugins에서 visualization > Image view를 3개 켠다 이미지를 양쫑에 두고 각각을 다음의 토픽으로 열어준다. `/detect/image_yellow_lane_marker/compressed` ,  `/detect/image_white_lane_marker/compressed` , `/detect/image_lane/compressed`

![](<../../.gitbook/assets/image (41).png>)

위 이미지를 보면 /detect/image\_white\_lane\_maker/compressed 에서 형광등에 비친 조명이 반사 되어 여러줄의 흰색선이 디텍팅 되는 것을 볼 수 있다. 이런 현상을 줄여주기 위해 캘리브레이션이 필요하다. 여기에 사용된 배경지식은 [여기](https://heromin.gitbook.io/drone-research/-Lq1lREaxFyRbZ-M2ezg/turtlebot-autorace-2020/undefined-3/undefined-2#undefined-1)서 볼 수 있다.&#x20;

5\. 노트북에서 실행 Terminal 5

```
rosrun rqt_reconfigure rqt_reconfigure
```

detect\_lane을 클릭하고 파라미터를 조절 한다.&#x20;

![](<../../.gitbook/assets/image (29).png>)

{% hint style="info" %}
`일반적으로 각 색깔의 hue를 먼저 설정후에 saturation을 하나의 선만 잘 잡힐수 있도록 순서대로 조절해준다. lightness의 경우 코드에서 선이 잡히는 정도에 따라 값을 올리고 내리기 때문에  lightness_l 파라미터를 변경하는 것이 의미가 없다. 상한인 lightness_h는 조절 가능하다.`&#x20;
{% endhint %}

특정 환경에서 아래와 같이 파라미터가 설정이 되어 노란색 필터에는 노란색 차선만 뚜렷하게 잡히고, 하얀색 필터에는 하얀색 차선만 뚜렷하게 잡힌다면,  차선 인식을 굉장히 잘 할 수 있다.&#x20;

아래 이미지에서 가장 우측 이미지는 차선이 잘 인식 되었을때 차선에 색을 씌워 구분하는 모습이다. 차선이 하나라도 제대로 잡히지 않는 경우 녹색의 덮어씌운 이미지는 사라진다. 또한 인식된 노란색선은 빨간선으로 덧칠해지고 흰선은 파란선으로 덧칠해 진다.&#x20;

![](<../../.gitbook/assets/image (10).png>)

6\. 설정된 값은`turtlebot3_autorace_construction_detect/param/lane.yaml`에 덮어 씌워주면 된다.&#x20;

## Traffic Light Detection Calibration

이 작업은 터틀봇이 신호등에 맞추어 출발 할 수 있도록 신호등을 인식하기 위한 색인식 부분의 calibration이다.

### 노트북 설정

사용된 **ROS Node 는 turtlebot3\_autorace\_traffic\_light\_detect의 detect\_traffic\_light.py 노드이다.**&#x20;

앞으로 앞서 .bashrc에 작성했던 환경변수 중 `AUTO_DT_CALIB=calibration` 으로 변경 후 실행해야 한다.&#x20;

![](https://firebasestorage.googleapis.com/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-Lq1PFS\_JV6OCZktAP2m%2Fuploads%2FP821550oCdsy0GyUmn8L%2Ffile.jpeg?alt=media)

1. turtlebot3의 SBC에서 실행

```
roslaunch turtlebot3_autorace_construction_camera camera_with_robot.launch
```

2\. 노트북에서 실행 `Terminal 1`

```
roslaunch turtlebot3_autorace_construction_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
```

3\. 노트북에서 실행 `Terminal 2`

```
roslaunch turtlebot3_autorace_construction_camera turtlebot3_autorace_extrinsic_camera_calibration.launch 
```

4\. 노트북에서 실행 `Terminal 3`

```
roslaunch turtlebot3_autorace_traffic_light_detect turtlebot3_autorace_detect_traffic_light.launch
```

#### 실행시 주의 사항

* `AUTO_IN_CALIB = action` 으로 설정되어야 한다.&#x20;
* `AUTO_EN_CALIB = action` 으로 설정되어야 한다.&#x20;
* `AUTO_DT_CALIB = calibration` 으로 설정되어야 한다.

4\. 노트북에서 실행 `Terminal 4`

```
rqt
```

rqt plugins에서 visualization > Image view를 4개 켠다 이미지를 양쫑에 두고 각각을 다음의 토픽으로 열어준다. `/detect/image_red_light` ,  `/detect/image_yellow_light` , `/detect/image_green_light`, `/detect/image_traffic_light`

![](<../../.gitbook/assets/image (14).png>)

{% tabs %}
{% tab title="detect_traffic_light.py" %}
```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Leon Jung, Gilbert
 
import rospy
import numpy as np
import cv2
from enum import Enum
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from turtlebot3_autorace_traffic_light_detect.cfg import DetectTrafficLightParamsConfig
from turtlebot3_autorace_msgs.srv import MissionService

class DetectTrafficLight():
    def __init__(self):
        self.hue_red_l = rospy.get_param("~detect/lane/red/hue_l", 0)
        self.hue_red_h = rospy.get_param("~detect/lane/red/hue_h", 26)
        self.saturation_red_l = rospy.get_param("~detect/lane/red/saturation_l", 239)
        self.saturation_red_h = rospy.get_param("~detect/lane/red/saturation_h", 255)
        self.lightness_red_l = rospy.get_param("~detect/lane/red/lightness_l", 123)
        self.lightness_red_h = rospy.get_param("~detect/lane/red/lightness_h", 250)

        self.hue_yellow_l = rospy.get_param("~detect/lane/yellow/hue_l", 19)
        self.hue_yellow_h = rospy.get_param("~detect/lane/yellow/hue_h", 33)
        self.saturation_yellow_l = rospy.get_param("~detect/lane/yellow/saturation_l", 237)
        self.saturation_yellow_h = rospy.get_param("~detect/lane/yellow/saturation_h", 255)
        self.lightness_yellow_l = rospy.get_param("~detect/lane/yellow/lightness_l", 231)
        self.lightness_yellow_h = rospy.get_param("~detect/lane/yellow/lightness_h", 255)

        self.hue_green_l = rospy.get_param("~detect/lane/green/hue_l", 40)
        self.hue_green_h = rospy.get_param("~detect/lane/green/hue_h", 113)
        self.saturation_green_l = rospy.get_param("~detect/lane/green/saturation_l", 210)
        self.saturation_green_h = rospy.get_param("~detect/lane/green/saturation_h", 255)
        self.lightness_green_l = rospy.get_param("~detect/lane/green/lightness_l", 131)
        self.lightness_green_h = rospy.get_param("~detect/lane/green/lightness_h", 255)

        self.srv_mission = rospy.ServiceProxy("/mission", MissionService)

        self.is_calibration_mode = rospy.get_param("~is_detection_calibration_mode", False)
        if self.is_calibration_mode == True:
            srv_detect_lane = Server(DetectTrafficLightParamsConfig, self.cbGetDetectTrafficLightParam)

        self.sub_image_type = "raw"          # "compressed" / "raw"
        self.pub_image_type = "raw"          # "compressed" / "raw"

        self.counter = 1
        
        if self.sub_image_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = rospy.Subscriber('/detect/image_input/compressed', CompressedImage, self.cbGetImage, queue_size = 1)
        elif self.sub_image_type == "raw":
            # subscribes raw image
            self.sub_image_original = rospy.Subscriber('/detect/image_input', Image, self.cbGetImage, queue_size = 1)
 
        if self.pub_image_type == "compressed":
            # publishes compensated image in compressed type 
            self.pub_image_traffic_light = rospy.Publisher('/detect/image_output/compressed', CompressedImage, queue_size = 1)
        elif self.pub_image_type == "raw":
            # publishes compensated image in raw type
            self.pub_image_traffic_light = rospy.Publisher('/detect/image_output', Image, queue_size = 1)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes light image in compressed type 
                self.pub_image_red_light = rospy.Publisher('/detect/image_output_sub1/compressed', CompressedImage, queue_size = 1)
                self.pub_image_yellow_light = rospy.Publisher('/detect/image_output_sub2/compressed', CompressedImage, queue_size = 1)
                self.pub_image_green_light = rospy.Publisher('/detect/image_output_sub3/compressed', CompressedImage, queue_size = 1)
            elif self.pub_image_type == "raw":
                # publishes light image in raw type
                self.pub_image_red_light = rospy.Publisher('/detect/image_output_sub1', Image, queue_size = 1)
                self.pub_image_yellow_light = rospy.Publisher('/detect/image_output_sub2', Image, queue_size = 1)
                self.pub_image_green_light = rospy.Publisher('/detect/image_output_sub3', Image, queue_size = 1)


        self.sub_traffic_light_finished = rospy.Subscriber('/control/traffic_light_finished', UInt8, self.cbTrafficLightFinished, queue_size = 1)
        self.pub_traffic_light_return = rospy.Publisher('/detect/traffic_light_stamped', UInt8, queue_size=1)
        self.pub_parking_start = rospy.Publisher('/control/traffic_light_start', UInt8, queue_size = 1)
        self.pub_max_vel = rospy.Publisher('/control/max_vel', Float64, queue_size = 1)


        self.pub_traffic_light = rospy.Publisher('/detect/traffic_light', UInt8, queue_size=1)


        self.CurrentMode = Enum('CurrentMode', 'idle lane_following traffic_light parking_lot level_crossing tunnel')
        self.StepOfTrafficLight = Enum('StepOfTrafficLight', 'searching_traffic_light searching_green_light searching_yellow_light searching_red_light waiting_green_light pass_traffic_light')

        self.cvBridge = CvBridge()
        self.cv_image = None

        self.is_image_available = False
        self.is_traffic_light_finished = False


        self.green_count = 0
        self.yellow_count = 0
        self.red_count = 0
        self.stop_count = 0
        self.off_traffic = False
        self.msg_sign = UInt8()
        self.pub_count = 0
        rospy.sleep(1)

        loop_rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.pub_count < 10:
            if self.is_image_available == True:
                if self.is_traffic_light_finished == False:
                    self.fnFindTrafficLight()
                if self.is_traffic_light_finished == True:
                    self.pub_traffic_light.publish(self.msg_sign)
                    self.pub_count = self.pub_count + 1
                    rospy.loginfo("%d", self.pub_count)


            loop_rate.sleep()

    def cbGetDetectTrafficLightParam(self, config, level):
        rospy.loginfo("[Detect Traffic Light] Detect Traffic Light Calibration Parameter reconfigured to")
        rospy.loginfo("hue_red_l : %d", config.hue_red_l)
        rospy.loginfo("hue_red_h : %d", config.hue_red_h)
        rospy.loginfo("saturation_red_l : %d", config.saturation_red_l)
        rospy.loginfo("saturation_red_h : %d", config.saturation_red_h)
        rospy.loginfo("lightness_red_l : %d", config.lightness_red_l)
        rospy.loginfo("lightness_red_h : %d", config.lightness_red_h)

        rospy.loginfo("hue_yellow_l : %d", config.hue_yellow_l)
        rospy.loginfo("hue_yellow_h : %d", config.hue_yellow_h)
        rospy.loginfo("saturation_yellow_l : %d", config.saturation_yellow_l)
        rospy.loginfo("saturation_yellow_h : %d", config.saturation_yellow_h)
        rospy.loginfo("lightness_yellow_l : %d", config.lightness_yellow_l)
        rospy.loginfo("lightness_yellow_h : %d", config.lightness_yellow_h)

        rospy.loginfo("hue_green_l : %d", config.hue_green_l)
        rospy.loginfo("hue_green_h : %d", config.hue_green_h)
        rospy.loginfo("saturation_green_l : %d", config.saturation_green_l)
        rospy.loginfo("saturation_green_h : %d", config.saturation_green_h)
        rospy.loginfo("lightness_green_l : %d", config.lightness_green_l)
        rospy.loginfo("lightness_green_h : %d", config.lightness_green_h)

        self.hue_red_l = config.hue_red_l
        self.hue_red_h = config.hue_red_h
        self.saturation_red_l = config.saturation_red_l
        self.saturation_red_h = config.saturation_red_h
        self.lightness_red_l = config.lightness_red_l
        self.lightness_red_h = config.lightness_red_h

        self.hue_yellow_l = config.hue_yellow_l
        self.hue_yellow_h = config.hue_yellow_h
        self.saturation_yellow_l = config.saturation_yellow_l
        self.saturation_yellow_h = config.saturation_yellow_h
        self.lightness_yellow_l = config.lightness_yellow_l
        self.lightness_yellow_h = config.lightness_yellow_h

        self.hue_green_l = config.hue_green_l
        self.hue_green_h = config.hue_green_h
        self.saturation_green_l = config.saturation_green_l
        self.saturation_green_h = config.saturation_green_h
        self.lightness_green_l = config.lightness_green_l
        self.lightness_green_h = config.lightness_green_h

        return config


    def cbGetImage(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        self.is_image_available = True

    def fnFindTrafficLight(self):
        cv_image_mask = self.fnMaskGreenTrafficLight()
        cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)

        status1 = self.fnFindCircleOfTrafficLight(cv_image_mask, 'green')
        if status1 == 1 or status1 == 5:
            rospy.loginfo("detect GREEN")
            self.stop_count = 0
            self.green_count += 1
        else:
            self.green_count = 0
        
        cv_image_mask = self.fnMaskYellowTrafficLight()
        cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)

        status2 = self.fnFindCircleOfTrafficLight(cv_image_mask, 'yellow')
        if status2 == 2:
            rospy.loginfo("detect YELLOW")
            self.yellow_count += 1
        else:
            self.yellow_count = 0

        cv_image_mask = self.fnMaskRedTrafficLight()
        cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)

        status3 = self.fnFindCircleOfTrafficLight(cv_image_mask, 'red')
        if status3 == 3:
            rospy.loginfo("detect RED")
            self.red_count += 1
        elif status3 == 4:
            self.red_count = 0
            self.stop_count += 1
        else:
            self.red_count = 0
            self.stop_count = 0

        if self.green_count >= 3:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.12
            self.pub_max_vel.publish(msg_pub_max_vel)
            rospy.loginfo("GREEN")
            cv2.putText(self.cv_image,"GREEN", (self.point_col, self.point_low), cv2.FONT_HERSHEY_DUPLEX, 0.5, (80, 255, 0))
            # msg_sign = UInt8()
            self.msg_sign.data = self.CurrentMode.lane_following.value

            self.is_traffic_light_finished = True
            self.srv_mission("TrafficLight", True)
            
        if self.yellow_count >= 3:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.03 if not self.off_traffic else 0.12
            self.pub_max_vel.publish(msg_pub_max_vel)
            rospy.loginfo("YELLOW")
            cv2.putText(self.cv_image,"YELLOW", (self.point_col, self.point_low), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 255))

        if self.red_count >= 3:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.0
            self.pub_max_vel.publish(msg_pub_max_vel)
            rospy.loginfo("RED")
            cv2.putText(self.cv_image,"RED", (self.point_col, self.point_low), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255))

        if self.stop_count >= 8:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.0
            self.pub_max_vel.publish(msg_pub_max_vel)  
            rospy.loginfo("STOP")
            self.off_traffic = True
            cv2.putText(self.cv_image,"STOP", (self.point_col, self.point_low), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255))

        if self.pub_image_type == "compressed":
            # publishes traffic light image in compressed type
            self.pub_image_traffic_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(self.cv_image, "jpg"))

        elif self.pub_image_type == "raw":
            # publishes traffic light image in raw type
            self.pub_image_traffic_light.publish(self.cvBridge.cv2_to_imgmsg(self.cv_image, "bgr8"))

    def fnMaskRedTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_red_l
        Hue_h = self.hue_red_h
        Saturation_l = self.saturation_red_l
        Saturation_h = self.saturation_red_h
        Lightness_l = self.lightness_red_l
        Lightness_h = self.lightness_red_h

        # define range of red color in HSV
        lower_red = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_red = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes red light filtered image in compressed type
                self.pub_image_red_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes red light filtered image in raw type
                self.pub_image_red_light.publish(self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        mask = cv2.bitwise_not(mask)

        return mask

    def fnMaskYellowTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_yellow_l
        Hue_h = self.hue_yellow_h
        Saturation_l = self.saturation_yellow_l
        Saturation_h = self.saturation_yellow_h
        Lightness_l = self.lightness_yellow_l
        Lightness_h = self.lightness_yellow_h

        # define range of yellow color in HSV
        lower_yellow = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes yellow light filtered image in compressed type
                self.pub_image_yellow_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes yellow light filtered image in raw type
                self.pub_image_yellow_light.publish(self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        mask = cv2.bitwise_not(mask)

        return mask

    def fnMaskGreenTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_green_l
        Hue_h = self.hue_green_h
        Saturation_l = self.saturation_green_l
        Saturation_h = self.saturation_green_h
        Lightness_l = self.lightness_green_l
        Lightness_h = self.lightness_green_h

        # define range of green color in HSV
        lower_green = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_green = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes green light filtered image in compressed type
                self.pub_image_green_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes green light filtered image in raw type
                self.pub_image_green_light.publish(self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        mask = cv2.bitwise_not(mask)

        return mask

    def fnFindCircleOfTrafficLight(self, mask, find_color):
        status = 0

        params=cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 50
        params.maxArea = 600

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.4

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.6

        det=cv2.SimpleBlobDetector_create(params)
        keypts=det.detect(mask)
        frame=cv2.drawKeypoints(self.cv_image,keypts,np.array([]),(0,255,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        col1 = 180
        col2 = 270
        col3 = 305

        low1 = 50
        low2 = 170
        low3 = 170
        
        # if detected more than 1 light
        for i in range(len(keypts)):
            self.point_col = int(keypts[i].pt[0])
            self.point_low = int(keypts[i].pt[1])
            if self.point_col > col1 and self.point_col < col2 and self.point_low > low1 and self.point_low < low2:
                if find_color == 'green':
                    status = 1
                elif find_color == 'yellow':
                    status = 2
                elif find_color == 'red':
                    status = 3
            elif self.point_col > col2 and self.point_col < col3 and self.point_low > low1 and self.point_low < low3:
                if find_color == 'red':
                    status = 4
                elif find_color == 'green':
                    status = 5
            else:
                status = 6

        return status

    def cbTrafficLightFinished(self, traffic_light_finished_msg):
        self.is_traffic_light_finished = True

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_traffic_light')
    node = DetectTrafficLight()
    node.main()
```
{% endtab %}
{% endtabs %}

{% hint style="info" %}
신호등 캘리브레이션을 진행 할 때 243-244번째 줄을 주석 처리하고 진행해야지 detect\_traffic\_light 노드가 죽지 않는다. 실제로 실행 할때는 꼭 주석을 해지해주어야 동작한다.&#x20;
{% endhint %}

# 배경지식

## 이미지 변환&#x20;

homography matrix, H는 3x3 행렬로 변환 행렬에 해당되며,

$$
H= \pmatrix{
h_{11} & h_{12} & h_{13} \cr 
h_{21} & h_{22} & h_{23} \cr
h_{31} & h_{32} & 1 \cr
}
$$

cv2. findHomography() 함수를 통해 구할 수 있다.&#x20;

이렇게 구한 H를 통해 우리는 image 1 와 image 2를 정합시킬수 있다. 더 정확히는 image 1을 image 2에 겹치게 변환 시켜줄수 있게 된다.

이 변환 과정은  cv2.warpPerspecive() 함수로 해 줄수 있는데. 변환 과정은 아래와 같다.

$$
\pmatrix{
x' \cr
y' \cr
1}
= \pmatrix{
h_{11} & h_{12} & h_{13} \cr 
h_{21} & h_{22} & h_{23} \cr
h_{31} & h_{32} & 1 \cr
}
\pmatrix{
x \cr
y \cr
1}
$$

위의 수식은 homography matrix를 검색해보면 자주 검색되는 식인데 중요한 점은 위 식 그대로 연산을 하면 안된다는 것이다. 아래가 진짜 식이다

$$
x' = {{
 h_{11}x + h_{12}y + h_{13}} \over{h_{31}x+h_{32}y+1}}
$$

$$
y' = {{
 h_{21}x + h_{22}y + h_{23}} \over{h_{31}x+h_{32}y+1}}
$$

위와 같이 해야 올바른 결과를 얻을 수 있다.&#x20;

아래 코드에서 사용 예시를 보자

{% tabs %}
{% tab title="image_projection.py" %}
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

# Authors: Leon Jung, [AuTURBO] Ki Hoon Kim (https://github.com/auturbo), Gilbert

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from turtlebot3_autorace_construction_camera.cfg import ImageProjectionParamsConfig

class ImageProjection():
    def __init__(self):
        self.top_x = rospy.get_param("/camera/extrinsic_camera_calibration/top_x", 72)
        self.top_y = rospy.get_param("/camera/extrinsic_camera_calibration/top_y", 4)
        self.bottom_x = rospy.get_param("/camera/extrinsic_camera_calibration/bottom_x", 115)
        self.bottom_y = rospy.get_param("/camera/extrinsic_camera_calibration/bottom_y", 120)

        self.is_calibration_mode = rospy.get_param("~is_extrinsic_camera_calibration_mode", False)
        if self.is_calibration_mode == True:
            srv_image_projection = Server(ImageProjectionParamsConfig, self.cbGetImageProjectionParam)

        self.sub_image_type = "compressed"        # "compressed" / "raw"
        self.pub_image_type = "compressed"        # "compressed" / "raw"

        if self.sub_image_type == "compressed":
            # subscribes compressed image 
            self.sub_image_original = rospy.Subscriber('/camera/image_input/compressed', CompressedImage, self.cbImageProjection, queue_size=1)
        elif self.sub_image_type == "raw":
            # subscribes raw image 
            self.sub_image_original = rospy.Subscriber('/camera/image_input', Image, self.cbImageProjection, queue_size=1)

        if self.pub_image_type == "compressed":
            # publishes ground-project image in compressed type 
            self.pub_image_projected = rospy.Publisher('/camera/image_output/compressed', CompressedImage, queue_size=1)
        elif self.pub_image_type == "raw":
            # publishes ground-project image in raw type 
            self.pub_image_projected = rospy.Publisher('/camera/image_output', Image, queue_size=1)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes calibration image in compressed type 
                self.pub_image_calib = rospy.Publisher('/camera/image_calib/compressed', CompressedImage, queue_size=1)
            elif self.pub_image_type == "raw":
                # publishes calibration image in raw type 
                self.pub_image_calib = rospy.Publisher('/camera/image_calib', Image, queue_size=1)

        self.cvBridge = CvBridge()


    def cbGetImageProjectionParam(self, config, level):
        rospy.loginfo("[Image Projection] Extrinsic Camera Calibration Parameter reconfigured to")
        rospy.loginfo("top_x : %d, top_y : %d, bottom_x : %d, bottom_y : %d", config.top_x, config.top_y, config.bottom_x, config.bottom_y)

        self.top_x = config.top_x
        self.top_y = config.top_y
        self.bottom_x = config.bottom_x
        self.bottom_y = config.bottom_y

        return config

    def cbImageProjection(self, msg_img):
        if self.sub_image_type == "compressed":
            # converts compressed image to opencv image
            np_image_original = np.fromstring(msg_img.data, np.uint8)
            cv_image_original = cv2.imdecode(np_image_original, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            # converts raw image to opencv image
            cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, "bgr8")

        # setting homography variables
        top_x = self.top_x
        top_y = self.top_y
        bottom_x = self.bottom_x
        bottom_y = self.bottom_y

        if self.is_calibration_mode == True:
            # copy original image to use for cablibration
            cv_image_calib = np.copy(cv_image_original)

            # draw lines to help setting homography variables
            cv_image_calib = cv2.line(cv_image_calib, (160 - top_x, 180 - top_y), (160 + top_x, 180 - top_y), (0, 0, 255), 1)
            cv_image_calib = cv2.line(cv_image_calib, (160 - bottom_x, 120 + bottom_y), (160 + bottom_x, 120 + bottom_y), (0, 0, 255), 1)
            cv_image_calib = cv2.line(cv_image_calib, (160 + bottom_x, 120 + bottom_y), (160 + top_x, 180 - top_y), (0, 0, 255), 1)
            cv_image_calib = cv2.line(cv_image_calib, (160 - bottom_x, 120 + bottom_y), (160 - top_x, 180 - top_y), (0, 0, 255), 1)

            if self.pub_image_type == "compressed":
                # publishes calibration image in compressed type
                self.pub_image_calib.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_calib, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes calibration image in raw type
                self.pub_image_calib.publish(self.cvBridge.cv2_to_imgmsg(cv_image_calib, "bgr8"))

        # adding Gaussian blur to the image of original
        cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)

        ## homography transform process
        # selecting 4 points from the original image
        pts_src = np.array([[160 - top_x, 180 - top_y], [160 + top_x, 180 - top_y], [160 + bottom_x, 120 + bottom_y], [160 - bottom_x, 120 + bottom_y]])

        # selecting 4 points from image that will be transformed
        pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])

        # finding homography matrix
        h, status = cv2.findHomography(pts_src, pts_dst)

        # homography process
        cv_image_homography = cv2.warpPerspective(cv_image_original, h, (1000, 600))

        # fill the empty space with black triangles on left and right side of bottom
        triangle1 = np.array([[0, 599], [0, 340], [200, 599]], np.int32)
        triangle2 = np.array([[999, 599], [999, 340], [799, 599]], np.int32)
        black = (0, 0, 0)
        white = (255, 255, 255)
        cv_image_homography = cv2.fillPoly(cv_image_homography, [triangle1, triangle2], black)

        if self.pub_image_type == "compressed":
            # publishes ground-project image in compressed type
            self.pub_image_projected.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_homography, "jpg"))

        elif self.pub_image_type == "raw":
            # publishes ground-project image in raw type
            self.pub_image_projected.publish(self.cvBridge.cv2_to_imgmsg(cv_image_homography, "bgr8"))

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_projection')
    node = ImageProjection()
    node.main()

```
{% endtab %}
{% endtabs %}

위의 코드에서 124번째 줄을 통해 homography matrix를 구하고 127줄을 통해 구해진 homography matrix를 사용하여 이미지를 변환한다.&#x20;

{% hint style="info" %}
이미지의 해상도를 현재 320x240에서 더 높일 경우 100\~103번째 줄 과 118, 121, 130\~131 줄의  숫자를 이미지 크기에 맞추어 변환해주어야 한다.
{% endhint %}

위 내용들은 [여기](https://ballentain.tistory.com/38)서 참고 하였다.

## 컬러 필터

이미지에서 원하는 컬러영역을 추출하기 위해서 사용하는 방식이다.

흰색을 추출하는 것과 노란색을 추출하는것은 파라미터 외에는 동일하기 때문에 흰색 기준으로 설명하겠다.&#x20;

순서는 다음과 같다

![](<../../.gitbook/assets/image (21).png>)

아래 코드는 흰색을 추출하는데 사용되는 코드이다. 이 코드를 바탕으로 설명하겠다.

```python
def maskWhiteLane(self, image):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    Hue_l = self.hue_white_l
    Hue_h = self.hue_white_h
    Saturation_l = self.saturation_white_l
    Saturation_h = self.saturation_white_h
    Lightness_l = self.lightness_white_l
    Lightness_h = self.lightness_white_h

    # define range of white color in HSV
    lower_white = np.array([Hue_l, Saturation_l, Lightness_l])
    upper_white = np.array([Hue_h, Saturation_h, Lightness_h])

    # Threshold the HSV image to get only white colors
    mask = cv2.inRange(hsv, lower_white, upper_white)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(image, image, mask = mask)

    fraction_num = np.count_nonzero(mask)

    if self.is_calibration_mode == False:
        if fraction_num > 35000:
            if self.lightness_white_l < 250:
                self.lightness_white_l += 5
        elif fraction_num < 5000:
            if self.lightness_white_l > 50:
                self.lightness_white_l -= 5

    how_much_short = 0

    for i in range(0, 600):
        if np.count_nonzero(mask[i,::]) > 0:
            how_much_short += 1

    how_much_short = 600 - how_much_short

    if how_much_short > 100:
        if self.reliability_white_line >= 5:
            self.reliability_white_line -= 5
    elif how_much_short <= 100:
        if self.reliability_white_line <= 99:
            self.reliability_white_line += 5

    msg_white_line_reliability = UInt8()
    msg_white_line_reliability.data = self.reliability_white_line
    self.pub_white_line_reliability.publish(msg_white_line_reliability)

    if self.is_calibration_mode == True:
        if self.pub_image_type == "compressed":
            # publishes white lane filtered image in compressed type
            self.pub_image_white_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

        elif self.pub_image_type == "raw":
            # publishes white lane filtered image in raw type
            self.pub_image_white_lane.publish(self.cvBridge.cv2_to_imgmsg(mask, "bgr8"))

    return fraction_num, mask
```

### 파라미터 로드

3번째 출의 `cv2.cvtColor(image, cv2.COLOR_BGR2HSV)`  함수로 이미지의 RGB 값을 HSV로 바꿔준다.&#x20;

5 \~ 10번째 줄에서 흰색에 대한 HUE, SATURATION, LIGHTNESS 파라미터를 불러온다.

### 컬러 영역 추출

13 \~ 17번째 줄을 통해 불러온 파라미터 기반의 low 와 high 바운더리를 만들고, 이미지에서 해당 영역을 `cv2.inRange(hsv, lower_white, upper_white)` 함수를 통해 추출한다.

### 추출 결과에 따른 밝기조절

22번째 줄을 통해 걸러진 픽셀의 개수를 파악하고 24\~30번째 줄을 통해 흰색으로 판단된 픽셀이 너무 많으면(35000개 이상) lightness\_low를 높이고, 흰색으로 판단된 픽셀이 너무 적으면 (5000개 이하) lightness\_low를 낮추어 환경의 영향을 줄인다.&#x20;

{% hint style="danger" %}
Input 이미지는 1000x600의 projected 된 이미지이다. law 이미지의 해상도를 변경하였다고 해도 크롭(crop) 영역만 잘 설정 하였고, projected 이미지의 해상도를 변경하지 않는다면 굳이 변경해 줄 필요가 없다.

현재 35000 \~ 5000 범위를 좁히거나 줄이는 것은 테스트를 통해 생각을 좀 해봐야 할 부분이나 라인만 잘 잡혔을 때의 픽셀갯수를 고려해 좁힌다면 차선인식에 도움이 될 가능성은 존재한다.&#x20;
{% endhint %}

### 추출 길이에 따른 신뢰도 조절

32 \~ 45번째 줄을 통해 추출된 영역의 길이가 전체 길이의 5/6 보다 길면 신뢰도를 높이고, 5/6보다 작으면 신뢰도를 낮춘다.&#x20;

{% hint style="danger" %}
전체 세로 픽셀길이 600에서 흰색이 검출되지 않은 row가 100 이상 이면 신뢰도를 낮추고, 100 이하면 신뢰도를 높이기 때문에 5/6 이상이라고 표현하기는 했지만 row상에 흰색이 1픽셀이라도 검출되면 카운트를 하기때문에 정확하게 선길이를 젠다고 표현하긴 힘들다.&#x20;

판단영역을 전체 row가 아닌 흰색은 우측 1/2 영역, 노란색은 좌측 1/2 영역으로 줄인다면 신뢰도의 신뢰도가 높아질 수 있다.&#x20;
{% endhint %}

## 라인 추출

앞서 컬러필터로 추출된 결과물로 인식된 차선을 선으로 그린다.&#x20;

```python
def fit_from_lines(self, lane_fit, image):
    nonzero = image.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    lane_inds = ((nonzerox > (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - margin)) & (
    nonzerox < (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + margin)))

    # Again, extract line pixel positions
    x = nonzerox[lane_inds]
    y = nonzeroy[lane_inds]

    # Fit a second order polynomial to each
    lane_fit = np.polyfit(y, x, 2)

    # Generate x and y values for plotting
    ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])
    lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]
        
    return lane_fitx, lane_fit
```

14번째 줄의 np.polyfit(y, x, 2) 함수를 통해 차선의 그래프의 계수를 구한다.&#x20;

numpy의 polyfit(x, y, d) 을 사용하면 x, y 평면에 대한 d차원의 계수를 구한다. 사용법은 [여기](https://pinkwink.kr/1127)를 참고했다.

{% hint style="danger" %}
여기서는 y와 x를 바꿔서 계수를 추출했다. 곡선 노선을 고려해서 2차원으로 추출 했다. 추출된 피쳐가 적거나 이상한 위치라면 곡선이 이상하게 표현될 수 있다. 마찬가지로 피쳐가 너무 없다면 계수가 추출되지 않을 수 있다.
{% endhint %}

---
description: GPIO(스위치) 입력을 받으면 1초 간격으로 LED를 점멸하다가, 5초 후 카메라 이미지를 캡져하는 파이썬 코드
---

# GPIO / LED / Camera Capture

```python
import time
import picamera
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN)
GPIO.setup(23, GPIO.OUT)

print "Take picture by SW-input with 5 sec delay"

try:
    # now use "camera" instead "picamera.PiCamera()"
    with picamera.PiCamera() as camera:
        camera.start_preview()
        GPIO.wait_for_edge(18, GPIO.RISING)
        
        for i in range(5):  # for( i=0; i<5 ; i++)
            GPIO.output(23, True)
            time.sleep(0.5)
            GPIO.output(23, False)
            time.sleep(0.5): continue
        
        camera.capture('image.jpg')
        caemra.stop_preview()
        
except KeyboardInterrupt:
    GPIO.cleanup()
    print "\nbye~"
```

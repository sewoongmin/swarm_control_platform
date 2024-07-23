# Pi Camera 동작 테스트

## 스틸 이미지 캡쳐

```
raspistill -o img001.jpg
```

연결된 모니텅 5 초간 preview 후, 마지막 영상을 스틸샷으로 캡쳐하여 img001.jpg로 저

## 동영상 캡쳐 테스트

```
raspivid -o video01.h264 -fps 30 -t 10000
```

초당 프레임수 30 으로 10,000ms(1 0초 ) 동안 녹화하여 video01.h264로 저장

```
sudo apt install ffmpeg
```

H264 형식을 mkv 형식으로 변환하기 위한 코덱 설치

```
ffmpeg -r 30 -i video01.h264 -vcodec copy video01.mkv
```

video01.h264 파일을 초당 30 프레임으로 video01.mkv로 형식 변환

---
description: 개발 과정에서 생긴 문제
---

# Issues

## Firmware Build Error

### Gazebo Plugin Error

#### 대표적인 현상

* Firmware를 다운 받은 후 git submodule update --init --recursive 명령어 실행시 sitl\_gazebo 모듈이 버전이 다르다며 업데이트가 안
* git submodule update까진 무리 없이 되었느나 아무 수정도 하지 않았는데 빌드 시도 시 gazebo plugin이 에러가 나면서 빌드가 되지 않음

원인 추정 (둘 중에 하나가 원인이나 정확한 원인은 밝혀내지 못함)

* Ubuntu 16.04 사용 중 Gazebo 7을 지우고 Gazebo 9을 설치
* Ubuntu 16.04 나 ROS kinetic과 px4 펌웨어 최신버전(1.10 이상)에서 사용하는 sitil\_gazebo 패키지의 호환성 문제

#### 해결법

[여기](./#dependency)서 명시한 의존성을 지켜서 사용&#x20;

## Toolchain 설치에러&#x20;

### 현상

[여기](https://dev.px4.io/master/en/setup/dev\_env\_linux\_ubuntu.html)에 명시 되어 있는 것 처럼 /Firmware/Tools/setup/ubuntu.sh 를 통해 Toolchain 설치를 진행하면 설치가 끝나고 reboot 하라는 메세지가 뜬다.

하지만 reboot 후&#x20;

```bash
arm-none-eabi-gcc --version
```

명령어를 입력하면 arm-none-eabi-gcc 가 없으니 sudo apt install 명령어를 통해 설치하라는 문구를 마주하게 된다.&#x20;

### 해결법

실제 설치는 문제 없이 되었으나 경로가 인식이 안 되어서 실행할 수 없는 상태이다.

```bash
gedit ~/.bashrc
```

명령어를 통해 bashrc에 export PATH= 경로에 `/opt/gcc-arm-none-eabi-7-2017-q4-major/bin:` 를 추가해 주면 된다.&#x20;

추가 후&#x20;

```bash
source ~/.bashrc
```

명령어를 통해 변경 된 bashrc를 적용하고 검증 명령어인&#x20;

```bash
arm-none-eabi-gcc --version
```

을 다시 입력했을 때

```bash
arm-none-eabi-gcc (GNU Tools for Arm Embedded Processors 7-2017-q4-major) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]
Copyright (C) 2017 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

위 같이 나온다면 해결 된 것이다.&#x20;

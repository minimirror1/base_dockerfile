# ROS Melodic CAN 환경 도커 이미지

이 도커 이미지는 ROS Melodic 환경과 CAN 통신을 위한 도구들이 설치된 환경을 제공합니다.

## 주요 기능

- ROS Melodic 환경
- CAN 통신 도구 (can-utils, canopen)
- SSH 서버 (포트 2225)
- MCS 장치 자동 감지 및 설정 (vendor ID: 16d0)

## 이미지 빌드 방법

```bash
docker build -t ros-canmanager .
```

## 호스트 시스템에서 CAN 인터페이스 설정

컨테이너를 실행하기 전에 호스트 시스템에서 CAN 인터페이스를 설정해야 합니다:

1. 필요한 커널 모듈 로드:
```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe slcan
```

2. USB 시리얼 장치 확인:
```bash
ls -l /dev/ttyACM*
```

3. slcan 인터페이스 설정:
```bash
sudo slcand -o -s6 -t hw -S 1000000 /dev/ttyACM0 can0
```

4. can0 인터페이스 활성화:
```bash
sudo ip link set can0 up
```

5. 인터페이스 상태 확인:
```bash
ip link show can0
```

## 컨테이너 실행 방법

MCS 장치를 사용하기 위해서는 컨테이너를 `--privileged` 모드로 실행해야 합니다:

```bash
docker run -it --privileged -p 2225:2225 --network=host ros-canmanager
```

또는 docker-compose를 사용하여 실행할 수 있습니다:

```bash
docker-compose up -d
```

`--network=host` 옵션을 사용하면 호스트 시스템의 네트워크 인터페이스(can0 포함)를 컨테이너에서 직접 사용할 수 있습니다.

## CAN 장치 사용 방법

컨테이너가 시작되면 자동으로 MCS 장치(vendor ID: 16d0)를 찾고 can0 인터페이스가 존재하는지 확인합니다. 인터페이스가 존재하면 간단한 CAN 메시지 테스트를 수행합니다.

CAN 인터페이스 상태 확인:

```bash
ip link show can0
```

CAN 메시지 모니터링:

```bash
candump can0
```

CAN 메시지 전송:

```bash
cansend can0 123#DEADBEEF
```

## SSH 접속 방법

```bash
ssh -p 2225 root@<호스트_IP>
```

기본 비밀번호: 7788 

# Docker 컨테이너에서 GUI 애플리케이션 실행하기

이 프로젝트는 Docker 컨테이너 내에서 GUI 애플리케이션을 실행하고, 호스트 PC의 디스플레이에 표시하는 방법을 제공합니다. 특히 XRDP를 통해 원격으로 접속하는 환경에서도 작동하도록 설계되었습니다.

## 시스템 구성

```
윈도우 PC -> XRDP -> 리눅스 호스트 -> Docker 컨테이너
```

## 사전 요구 사항

- 리눅스 호스트 시스템
- Docker 및 Docker Compose 설치
- XRDP 서버 설정 및 실행
- X11 서버 설치

## 설정 방법

### 1. 호스트 시스템 설정

호스트 시스템에서 다음 명령을 실행하여 X11 디스플레이를 Docker 컨테이너에 공유할 수 있도록 설정합니다:

```bash
./setup_xrdp_for_docker.sh
```

이 스크립트는 다음 작업을 수행합니다:
- X11 서버에 모든 클라이언트의 접근을 허용
- .Xauthority 파일 확인 및 생성
- XRDP 세션 정보 확인
- Docker 컨테이너 실행 상태 확인

### 2. Docker Compose 실행

스크립트 실행 후 표시되는 명령을 사용하여 Docker Compose를 실행합니다:

```bash
DISPLAY=$DISPLAY XAUTHORITY=$HOME/.Xauthority docker-compose up -d
```

또는 로그를 확인하려면:

```bash
DISPLAY=$DISPLAY XAUTHORITY=$HOME/.Xauthority docker-compose up
```

### 3. 컨테이너 내에서 GUI 애플리케이션 실행

컨테이너에 접속하여 GUI 애플리케이션을 실행합니다:

```bash
docker exec -it ros-canmanager-container bash
```

컨테이너 내에서 다음과 같이 GUI 애플리케이션을 실행할 수 있습니다:

```bash
xeyes  # 테스트용 간단한 X11 애플리케이션
sublime_text  # Sublime Text 에디터
```

## 문제 해결

### X11 연결 실패 시

1. 호스트 시스템에서 `xhost +` 명령을 실행했는지 확인하세요.
2. DISPLAY 환경 변수가 올바르게 설정되었는지 확인하세요.
3. X11 소켓이 올바르게 마운트되었는지 확인하세요.
4. 컨테이너 내에서 `echo $DISPLAY`를 실행하여 DISPLAY 환경 변수가 설정되었는지 확인하세요.

### XRDP 관련 문제

1. XRDP 세션이 활성화되어 있는지 확인하세요: `who | grep -i xrdp`
2. XRDP 로그를 확인하세요: `cat /var/log/xrdp-sesman.log`

## 참고 사항

- 보안을 위해 `xhost +` 대신 `xhost +local:` 또는 특정 IP만 허용하는 것이 좋습니다.
- 컨테이너를 재시작할 때마다 `xhost +` 명령을 다시 실행해야 할 수 있습니다.
- XRDP 세션이 종료되면 X11 연결도 끊어질 수 있으므로 주의하세요. 
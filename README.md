# ROS 및 MuJoCo 환경 도커 이미지

이 프로젝트는 ROS(Robot Operating System) 환경과 MuJoCo 시뮬레이션 환경을 위한 도커 이미지를 제공합니다.

# 빠른 시작

## MuJoCo 컨테이너 빌드 및 실행
```bash
# MuJoCo 이미지 빌드
docker-compose build ros-mujoco

# MuJoCo 컨테이너 실행
docker-compose up -d ros-mujoco

# 컨테이너 접속
docker exec -it ros-mujoco-container bash
```

## CAN 매니저 컨테이너 빌드 및 실행
```bash
# CAN 매니저 이미지 빌드
docker-compose build ros-canmanager

# CAN 매니저 컨테이너 실행
docker-compose up -d ros-canmanager

# 컨테이너 접속
docker exec -it ros-canmanager-container bash
```

## 모든 컨테이너 관리
```bash
# 모든 이미지 빌드
docker-compose build

# 모든 컨테이너 실행
docker-compose up -d

# 모든 컨테이너 중단 및 제거
docker-compose down

# 고아 컨테이너까지 모두 제거
docker-compose down --remove-orphans
```

## 주요 기능

### ROS CAN 매니저 이미지
- ROS Melodic 환경
- CAN 통신 도구 (can-utils, canopen)
- SSH 서버 (포트 2225)
- MCS 장치 자동 감지 및 설정 (vendor ID: 16d0)
- GUI 애플리케이션 지원 (X11 포워딩)

### ROS MuJoCo 이미지
- ROS Noetic 환경
- MuJoCo 시뮬레이션 환경
- Python 3.10
- SSH 서버 (포트 2235)
- GUI 애플리케이션 지원 (X11 포워딩)

## 도커 이미지 빌드 방법

### 개별 이미지 빌드

ROS CAN 매니저 이미지 빌드:
```bash
docker build -t ros-canmanager -f Dockerfile.canmanager .
```

ROS MuJoCo 이미지 빌드:
```bash
docker build -t ros-mujoco -f Dockerfile.mujoco .
```

### Docker Compose를 사용한 빌드

모든 이미지를 한 번에 빌드:
```bash
docker-compose build
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

## 컨테이너 생성 및 실행 방법

### 개별 컨테이너 실행

ROS CAN 매니저 컨테이너 실행:
```bash
docker run -it --privileged -p 2225:2225 --network=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/tmp/.Xauthority:ro \
  -v ./workspace:/workspace \
  -v /dev:/dev \
  ros-canmanager
```

ROS MuJoCo 컨테이너 실행:
```bash
docker run -it --privileged -p 2235:2235 --network=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/tmp/.Xauthority:ro \
  -v ./workspace:/workspace \
  -v /dev:/dev \
  ros-mujoco
```

### Docker Compose를 사용한 실행

X11 디스플레이 설정 후 모든 컨테이너 실행:
```bash
xhost +
DISPLAY=$DISPLAY XAUTHORITY=$HOME/.Xauthority docker-compose up -d
```

또는 로그를 확인하며 실행:
```bash
xhost +
DISPLAY=$DISPLAY XAUTHORITY=$HOME/.Xauthority docker-compose up
```

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

### ROS CAN 매니저 컨테이너 접속
```bash
ssh -p 2225 root@<호스트_IP>
```

### ROS MuJoCo 컨테이너 접속
```bash
ssh -p 2235 root@<호스트_IP>
```

기본 비밀번호: 7788 

## Docker 컨테이너에서 GUI 애플리케이션 실행하기

이 프로젝트는 Docker 컨테이너 내에서 GUI 애플리케이션을 실행하고, 호스트 PC의 디스플레이에 표시하는 방법을 제공합니다. 특히 XRDP를 통해 원격으로 접속하는 환경에서도 작동하도록 설계되었습니다.

### 시스템 구성

```
윈도우 PC -> XRDP -> 리눅스 호스트 -> Docker 컨테이너
```

### 사전 요구 사항

- 리눅스 호스트 시스템
- Docker 및 Docker Compose 설치
- XRDP 서버 설정 및 실행
- X11 서버 설치

### X11 설정 방법

호스트 시스템에서 다음 명령을 실행하여 X11 디스플레이를 Docker 컨테이너에 공유할 수 있도록 설정합니다:

```bash
xhost +
```

또는 제공된 설정 스크립트를 사용하여 XRDP 환경에서 더 쉽게 설정할 수 있습니다:

```bash
./setup_xrdp_for_docker.sh
```

이 스크립트는 다음 작업을 수행합니다:
- X11 서버에 모든 클라이언트의 접근을 허용
- .Xauthority 파일 확인 및 생성
- XRDP 세션 정보 확인
- DISPLAY 환경 변수 설정
- Docker 컨테이너 실행 상태 확인
- Docker Compose 실행 명령 제안

### 컨테이너 내에서 GUI 애플리케이션 실행

컨테이너에 접속하여 GUI 애플리케이션을 실행합니다:

```bash
docker exec -it ros-canmanager-container bash
```

또는

```bash
docker exec -it ros-mujoco-container bash
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

### CAN 인터페이스 문제

1. 호스트 시스템에서 CAN 인터페이스가 올바르게 설정되었는지 확인하세요.
2. 컨테이너가 `--privileged` 모드로 실행되었는지 확인하세요.
3. `/dev` 디렉토리가 올바르게 마운트되었는지 확인하세요.

## 참고 사항

- 보안을 위해 `xhost +` 대신 `xhost +local:` 또는 특정 IP만 허용하는 것이 좋습니다.
- 컨테이너를 재시작할 때마다 `xhost +` 명령을 다시 실행해야 할 수 있습니다.
- XRDP 세션이 종료되면 X11 연결도 끊어질 수 있으므로 주의하세요. 
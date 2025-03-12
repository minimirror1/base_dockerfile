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
#!/bin/bash

service ssh start
echo "SSH 서버가 시작되었습니다."

# X11 디스플레이 설정 확인
echo "X11 디스플레이 설정을 확인하는 중..."
if [ -n "$DISPLAY" ]; then
  echo "DISPLAY 환경 변수가 설정되어 있습니다: $DISPLAY"
else
  echo "DISPLAY 환경 변수가 설정되어 있지 않습니다. 기본값으로 설정합니다."
  export DISPLAY=:0
fi

# Xauthority 파일 확인
if [ -f "/tmp/.Xauthority" ]; then
  echo "Xauthority 파일이 존재합니다."
else
  echo "Xauthority 파일이 존재하지 않습니다. 호스트의 파일을 사용합니다."
  if [ -f "$HOME/.Xauthority" ]; then
    cp "$HOME/.Xauthority" /tmp/.Xauthority
  fi
fi

# CAN 인터페이스 확인 및 설정
echo "CAN 인터페이스를 확인하는 중..."

if ip link show can0 &>/dev/null; then
  echo "can0 인터페이스가 존재합니다:"
  ip -details link show can0
  
  # can0 인터페이스가 DOWN 상태인 경우 UP으로 변경 시도
  if ! ip link show can0 | grep -q "UP"; then
    echo "can0 인터페이스가 DOWN 상태입니다. UP으로 변경을 시도합니다."
    ip link set can0 up
    if [ $? -eq 0 ]; then
      echo "can0 인터페이스를 UP 상태로 변경했습니다."
    else
      echo "can0 인터페이스를 UP 상태로 변경하지 못했습니다. 권한 문제일 수 있습니다."
    fi
  fi
  
  # CAN 인터페이스 테스트
  echo "CAN 인터페이스 테스트:"
  candump -n 1 can0 &
  CANDUMP_PID=$!
  sleep 1
  cansend can0 123#DEADBEEF
  sleep 1
  kill $CANDUMP_PID 2>/dev/null
  echo "CAN 인터페이스 테스트 완료"
  
  # canopen_manager 패키지의 설정 파일 확인 및 수정
  CANOPEN_CONFIG=~/ros_ws/src/canopen_manager/launch/canopen_manager.launch
  if [ -f "$CANOPEN_CONFIG" ]; then
    echo "canopen_manager 설정 파일이 존재합니다. can_interface 파라미터를 확인합니다."
    if grep -q "can_interface" "$CANOPEN_CONFIG"; then
      echo "can_interface 파라미터를 can0으로 설정합니다."
      sed -i 's/<param name="can_interface" value="[^"]*"/<param name="can_interface" value="can0"/g' "$CANOPEN_CONFIG"
    fi
  else
    echo "canopen_manager 설정 파일을 찾을 수 없습니다."
  fi
else
  echo "can0 인터페이스가 존재하지 않습니다. 호스트 시스템에서 다음 명령을 실행하여 CAN 인터페이스를 설정해 주세요:"
  echo "  sudo modprobe slcan"
  echo "  sudo slcand -o -s8 -t hw -S 1000000 /dev/ttyACM0 can0"
  echo "  sudo ip link set can0 up"
fi

# ROS 작업 공간 설정 추가
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc

# X11 연결 테스트
echo "X11 연결을 테스트합니다..."
if xdpyinfo >/dev/null 2>&1; then
  echo "X11 연결이 성공적으로 설정되었습니다."
  # 간단한 X11 애플리케이션 테스트 (선택 사항)
  # xeyes &
else
  echo "X11 연결에 실패했습니다. 다음 사항을 확인해 주세요:"
  echo "1. 호스트에서 'xhost +' 명령을 실행했는지 확인하세요."
  echo "2. DISPLAY 환경 변수가 올바르게 설정되었는지 확인하세요."
  echo "3. X11 소켓이 올바르게 마운트되었는지 확인하세요."
fi

tail -f /dev/null 
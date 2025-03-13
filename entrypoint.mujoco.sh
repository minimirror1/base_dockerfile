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

# CAN 인터페이스 확인 (선택적)
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
else
  echo "can0 인터페이스가 존재하지 않습니다. MuJoCo 시뮬레이션에는 영향이 없습니다."
fi

# ROS 작업 공간 설정 추가
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc

# Python 환경 확인
echo "Python 환경을 확인합니다..."
python --version
python -m pip list | grep mujoco

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

# MuJoCo 예제 파일 복사 (선택 사항)
if [ -f "/workspace/mujoco_viewer_example.py" ]; then
  echo "MuJoCo 예제 파일이 이미 존재합니다."
else
  echo "MuJoCo 예제 파일을 복사합니다."
  cat > /workspace/mujoco_viewer_example.py << 'EOF'
import mujoco
import numpy as np
from mujoco.glfw import glfw
import time

# MuJoCo 모델 로드
xml = """
<mujoco>
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="1 1 0.1" rgba=".9 .9 .9 1"/>
    <body pos="0 0 1">
      <joint type="free"/>
      <geom type="sphere" size="0.1" rgba="1 0 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# 뷰어 초기화
viewer = mujoco.viewer.launch_passive(model, data)

# 시뮬레이션 실행
for i in range(1000):
    mujoco.mj_step(model, data)
    viewer.sync()
    time.sleep(0.01)

viewer.close()
EOF
  chmod +x /workspace/mujoco_viewer_example.py
  echo "MuJoCo 예제 파일이 생성되었습니다. '/workspace/mujoco_viewer_example.py'를 실행하여 테스트할 수 있습니다."
fi

tail -f /dev/null 
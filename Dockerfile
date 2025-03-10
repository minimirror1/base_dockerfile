# 베이스 이미지 설정
FROM hikim0609/ros_melodic_env_hikim_250305:v1

# 메타데이터 설정
LABEL maintainer="사용자 이메일"
LABEL version="1.0"
LABEL description="ROS Melodic 환경 기반 도커 이미지"

# 환경 변수 설정
ENV DEBIAN_FRONTEND=noninteractive

# 시스템 업데이트 및 필요한 패키지 설치
RUN apt-get update && apt-get install -y \
    build-essential \
    python-pip \
    python-dev \
    openssh-server \
    net-tools \
    iputils-ping \
    iproute2 \
    kmod \
    can-utils \
    usbutils \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Python 패키지 설치
RUN pip install canopen

# SSH 서버 설정
RUN mkdir /var/run/sshd
RUN echo 'root:7788' | chpasswd
# SSH 루트 로그인 허용 설정
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
# SSH 패스워드 인증 허용
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
# X11 포워딩 허용
RUN sed -i 's/#X11Forwarding no/X11Forwarding yes/' /etc/ssh/sshd_config
# SSH 포트 변경
RUN sed -i 's/#Port 22/Port 2222/' /etc/ssh/sshd_config

# SSH 연결을 위한 포트 노출
EXPOSE 2222

# 작업 디렉토리 설정
WORKDIR /workspace

# ROS 환경 설정
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# 시작 스크립트 생성
RUN echo '#!/bin/bash\n\
service ssh start\n\
echo "SSH 서버가 시작되었습니다."\n\
\n\
# MCS CANable2 장치 찾기 및 설정 (vendor ID: 16d0)\n\
echo "CAN 장치를 찾는 중..."\n\
CAN_DEVICE=$(lsusb | grep "16d0:")\n\
\n\
if [ -n "$CAN_DEVICE" ]; then\n\
  echo "MCS 장치를 찾았습니다:"\n\
  echo "$CAN_DEVICE"\n\
  \n\
  # CAN 인터페이스 확인\n\
  if ip link show can0 &>/dev/null; then\n\
    echo "can0 인터페이스가 존재합니다:"\n\
    ip link show can0\n\
    echo "CAN 인터페이스 테스트:"\n\
    candump -n 1 can0 &\n\
    sleep 1\n\
    cansend can0 123#DEADBEEF\n\
    sleep 1\n\
    echo "CAN 인터페이스 테스트 완료"\n\
  else\n\
    echo "can0 인터페이스가 존재하지 않습니다. 호스트 시스템에서 CAN 인터페이스를 설정해 주세요."\n\
  fi\n\
else\n\
  echo "MCS 장치를 찾을 수 없습니다."\n\
fi\n\
\n\
tail -f /dev/null' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# 컨테이너 시작 시 SSH 서버 실행
ENTRYPOINT ["/entrypoint.sh"]

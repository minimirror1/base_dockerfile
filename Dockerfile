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
    git \
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

# ROS 작업 공간 생성 및 GitHub 저장소 클론
RUN mkdir -p ~/ros_ws/src && \
    cd ~/ros_ws/src && \
    git clone https://github.com/minimirror1/canopen_manager.git && \
    cd ~/ros_ws && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

# 시작 스크립트 복사
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 컨테이너 시작 시 SSH 서버 실행
ENTRYPOINT ["/entrypoint.sh"]

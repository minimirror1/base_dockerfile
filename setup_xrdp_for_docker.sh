#!/bin/bash

# XRDP 환경에서 Docker 컨테이너와 X11 연결을 설정하는 스크립트

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== Docker 컨테이너를 위한 XRDP 환경 설정 ===${NC}"

# X11 서버 설정 확인
echo -e "${YELLOW}[1/5] X11 서버 설정을 확인합니다...${NC}"
if ! command -v xhost &> /dev/null; then
    echo -e "${RED}오류: xhost 명령을 찾을 수 없습니다. X11 서버가 설치되어 있는지 확인하세요.${NC}"
    exit 1
fi

# X11 서버에 모든 클라이언트의 접근을 허용
echo -e "${YELLOW}[2/5] X11 서버에 모든 클라이언트의 접근을 허용합니다...${NC}"
xhost +
if [ $? -ne 0 ]; then
    echo -e "${RED}오류: xhost + 명령이 실패했습니다.${NC}"
    exit 1
fi
echo -e "${GREEN}X11 서버 접근 설정이 완료되었습니다.${NC}"

# .Xauthority 파일 확인 및 생성
echo -e "${YELLOW}[3/5] .Xauthority 파일을 확인합니다...${NC}"
if [ -f "$HOME/.Xauthority" ]; then
    echo -e "${GREEN}.Xauthority 파일이 존재합니다: $HOME/.Xauthority${NC}"
else
    echo -e "${YELLOW}.Xauthority 파일이 존재하지 않습니다. 새로 생성합니다...${NC}"
    touch "$HOME/.Xauthority"
    xauth generate $DISPLAY . trusted
    echo -e "${GREEN}.Xauthority 파일이 생성되었습니다.${NC}"
fi

# XRDP 세션 정보 확인
echo -e "${YELLOW}[4/5] XRDP 세션 정보를 확인합니다...${NC}"
XRDP_SESSIONS=$(who | grep -i xrdp)
if [ -z "$XRDP_SESSIONS" ]; then
    echo -e "${YELLOW}경고: 활성화된 XRDP 세션을 찾을 수 없습니다.${NC}"
else
    echo -e "${GREEN}활성화된 XRDP 세션:${NC}"
    echo "$XRDP_SESSIONS"
fi

# DISPLAY 환경 변수 확인
echo -e "${YELLOW}[5/5] DISPLAY 환경 변수를 확인합니다...${NC}"
if [ -z "$DISPLAY" ]; then
    echo -e "${YELLOW}경고: DISPLAY 환경 변수가 설정되어 있지 않습니다.${NC}"
    # XRDP 세션에서 사용 중인 디스플레이 번호 추출 시도
    DISPLAY_NUM=$(who | grep -i xrdp | grep -o ':[0-9]*' | head -1)
    if [ -n "$DISPLAY_NUM" ]; then
        export DISPLAY=$DISPLAY_NUM
        echo -e "${GREEN}DISPLAY 환경 변수를 $DISPLAY로 설정했습니다.${NC}"
    else
        export DISPLAY=:10
        echo -e "${YELLOW}DISPLAY 환경 변수를 기본값 :10으로 설정했습니다.${NC}"
    fi
else
    echo -e "${GREEN}DISPLAY 환경 변수가 $DISPLAY로 설정되어 있습니다.${NC}"
fi

# Docker 컨테이너 실행 상태 확인
echo -e "${YELLOW}Docker 컨테이너 실행 상태를 확인합니다...${NC}"
RUNNING_CONTAINERS=$(docker ps --format "{{.Names}}")
if [ -z "$RUNNING_CONTAINERS" ]; then
    echo -e "${YELLOW}실행 중인 Docker 컨테이너가 없습니다.${NC}"
else
    echo -e "${GREEN}실행 중인 Docker 컨테이너:${NC}"
    echo "$RUNNING_CONTAINERS"
fi

# Docker Compose 명령 제안
echo -e "${BLUE}=== Docker Compose 실행 명령 ===${NC}"
echo -e "${GREEN}다음 명령을 사용하여 Docker Compose를 실행하세요:${NC}"
echo -e "${YELLOW}DISPLAY=$DISPLAY XAUTHORITY=$HOME/.Xauthority docker-compose up -d${NC}"
echo -e "${GREEN}또는 로그를 확인하려면:${NC}"
echo -e "${YELLOW}DISPLAY=$DISPLAY XAUTHORITY=$HOME/.Xauthority docker-compose up${NC}"

echo -e "${BLUE}=== 설정 완료 ===${NC}"
echo -e "${GREEN}X11 디스플레이가 Docker 컨테이너에 공유되도록 설정되었습니다.${NC}"
echo -e "${YELLOW}참고: 보안을 위해 작업이 완료된 후 'xhost -' 명령을 실행하여 X11 서버 접근을 제한하는 것이 좋습니다.${NC}" 
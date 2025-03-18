#!/bin/bash

# 색상 설정
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # 색상 초기화

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}   X11 디스플레이 자동 설정 도구   ${NC}"
echo -e "${YELLOW}========================================${NC}"

# 현재 설정된 DISPLAY 환경변수 확인
echo -e "${GREEN}[1/3] 현재 DISPLAY 설정 확인...${NC}"
if [ -n "$DISPLAY" ]; then
    echo -e "  현재 DISPLAY=${DISPLAY}"
else
    echo -e "  DISPLAY 환경변수가 설정되어 있지 않습니다."
fi

# DISPLAY 변수를 :0으로 설정
echo -e "${GREEN}[2/3] DISPLAY 환경변수를 :0으로 설정...${NC}"
export DISPLAY=:0
echo -e "  DISPLAY=:0 설정 완료"

# X 서버 액세스 허용 시도
echo -e "${GREEN}[3/3] X 서버 액세스 허용 설정...${NC}"
if command -v xhost > /dev/null 2>&1; then
    echo -e "  xhost 명령을 사용할 수 있습니다."
    echo -e "  ${YELLOW}주의: X 서버가 :0에 없는 경우 액세스 허용이 실패할 수 있습니다.${NC}"
    echo -e "  ${YELLOW}그런 경우 실제 X 서버 디스플레이 번호를 확인하세요:${NC}"
    echo -e "  ${YELLOW}ps -ef | grep Xorg${NC}"
else
    echo -e "  ${RED}경고: xhost 명령을 찾을 수 없습니다. X11 관련 패키지가 설치되어 있는지 확인하세요.${NC}"
fi

echo -e "${YELLOW}========================================${NC}"
echo -e "  최종 설정: ${GREEN}DISPLAY=:0${NC}"
echo -e "${YELLOW}========================================${NC}"

# 현재 셸에서 사용할 수 있도록 명령 표시
echo -e "다음 명령을 복사하여 현재 셸에 붙여넣으세요:"
echo -e "${GREEN}export DISPLAY=:0${NC}"

# .bashrc에 추가 제안
echo -e "${YELLOW}터미널 세션마다 자동으로 설정하려면 다음 명령을 실행하세요:${NC}"
echo -e "${GREEN}echo \"export DISPLAY=:0\" >> ~/.bashrc${NC}"

# Docker Compose 실행 안내
echo -e "${YELLOW}Docker Compose를 실행하려면:${NC}"
echo -e "${GREEN}docker compose up -d${NC}" 
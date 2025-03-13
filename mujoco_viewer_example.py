#!/usr/bin/env python3
# MuJoCo 뷰어 예제 스크립트

import mujoco
import mujoco.viewer
import time

# 기본 예제 모델 로드 (humanoid.xml)
model = mujoco.MjModel.from_xml_path('/root/.mujoco/mujoco300/model/humanoid.xml')
data = mujoco.MjData(model)

# 뷰어 실행
with mujoco.viewer.launch_passive(model, data) as viewer:
    # 시뮬레이션 실행
    for _ in range(10000):
        # 시뮬레이션 스텝 진행
        mujoco.mj_step(model, data)
        
        # 뷰어 업데이트
        viewer.sync()
        
        # 실시간 시뮬레이션을 위한 대기
        time.sleep(model.opt.timestep)
        
        # 뷰어가 닫히면 종료
        if viewer.is_stopped():
            break 
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

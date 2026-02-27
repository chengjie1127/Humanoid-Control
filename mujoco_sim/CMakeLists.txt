cmake_minimum_required(VERSION 3.8)
project(mujoco_sim)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  script/draw_torque.py
  script/humanoid_sim.py
  script/joy.py
  script/mujoco_base.py
  script/teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

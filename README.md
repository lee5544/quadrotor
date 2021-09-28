# quadrotor

## 设计框架

<img src="/image/程序设计.jpg" alt="程序设计" style="zoom:25%;" />

## 编译和运行
见/wiki/安装和编译.md
## 实际程序执行-以missionSample为例

- 运行px4_bridge

  ```bash
  roslaunch px4_bridge px4_bridge.launch
  ```

- 运行realsense和感知模块

  ```bash
  roslaunch realsen2_camera rs_camera.launch
  ……
  ```

- 运行规划模块

  ```bash
  roslaunch plan_manage kino_replan_gazebo.launch
  ```

- 运行任务模块

  ```bash
  roslaunch mission mission_sample.launch
  ```

  

## 仿真程序执行-以missionSample为例

- 一键启动

  ```BASH 
  ./Simulator/sitl_sh/gazebo_outdoor.sh
  ```
  

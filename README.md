# quadrotor

## px4_bridge

连接px4飞控，读取状态和发送期望指令

- 节点CommandToFcu

- 节点StatesFromFcu

- 节点StatesToFcu

# 启动说明

1. 启动感知模块

   - 启动realsense节点

     - ```bash
       roslaunch realsense2_camera rs_camera.launch
       ```

   - 启动单目深度估计节点

2. 启动px4_bridge

   - 启动mavros节点

   - 启动command_to_fcu节点

   - 启动states_from_fcu节点

   - 启动states_to_fcu节点

   - ```bash
     roslaunch px4_bridge px4_bridge.launch
     ```

3. 启动规划模块

   - fastplanner

   - ```bash
     roslaunch plan_manager kino_replan_gazebo.launch
     ```

4. 启动mission

   - ```bash
     rosrun mission mymission
     ```



# 命名规范

原则：驼峰，下划线

- class名：首字母大写
- 实例化的class名：首字母小写

- 类中的全局变量：pos_
- 局部变量：pos

- 全局变量：POS
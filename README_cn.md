# RDK X5 具身智能家居清洁机器人代码

本仓库保存基于 RDK X5 与 GD32F407 的具身智能家居清洁机器人主要代码。系统以 ROS2 为上位机核心，融合视觉感知、语音交互、任务规划、导航控制、双机械臂执行和小程序远程交互，实现面向真实场景的清洁、抓取、整理与状态反馈。

## 目录结构

- `ros2_upper_controller/`
  - 运行在 RDK X5 上的 ROS2 上位机代码。
  - `my_ros/src/` 包含机器人视觉感知、导航建图、启动配置、状态聚合、技能服务、消息定义、地图与机器人模型等功能包。
  - `ros2_ws/src/` 包含语音唤醒、语音识别、语音合成、对话交互和 AI 网关相关功能包。
- `gd32_lower_controller/`
  - GD32F407 下位机固件工程。
  - `APPLICATION/` 包含任务逻辑、底盘控制、云台/机械臂相关应用代码和通信交互代码。
  - `BSP/` 包含 CAN、UART、PWM、SPI、延时、FIFO、消息收发等底层驱动。
  - `MODULES/` 包含舵机、电机、PID、LCD、IMU、编码器等设备模块。
  - `FreeRTOS/`、`CMSIS/`、`Library/` 和 `Startup/` 为 RTOS、芯片库和启动支持文件。
  - `project/` 保存 Keil 工程文件。

## 代码说明

本仓库保留了项目展示和复现所需的源代码、配置文件、启动文件、消息接口、地图、机器人模型、固件源码和工程文件。

已清理本地生成文件与无关文件，包括 `.git`、`.vscode`、ROS `build/install/log` 目录、Python 缓存、Keil `Objects/Listings`、调试输出和本地用户配置文件等。


# Robot Kinematics Simulator (2D 3-DOF)

This project implements a **2D planar 3-DOF robot arm kinematics simulator** in Python.

一个使用 Python 编写的 **二维三自由度机械臂运动学仿真项目**。

It provides analytical **forward and inverse kinematics** solutions together with **interactive visualization**, allowing intuitive exploration of the relationship between joint space and task space.

本项目实现了 **正运动学与逆运动学**，并提供交互式可视化界面。

## Features

- Analytical forward kinematics for a 2D 3-DOF planar robot arm
- Analytical inverse kinematics with target position input
- Interactive visualization using Matplotlib
- Real-time update of robot configuration and trajectories
- Intuitive exploration of joint angles, link lengths, and end-effector position

## Project Structure

```md
```text
Robot-Kinematics-Sim/
│
├── forward.py      # Forward kinematics and visualization
├── inverse.py      # Inverse kinematics with interactive target selection
├── README.md
└── requirements.txt
```
---

## How to run

git clone https://github.com/Ir1222/Robot-Kinematics-Sim.git
cd Robot-Kinematics-Sim


### forward kinematics

```md
python forward.py
```

Execution Flow | 执行流程说明

The robot arm is initialized with predefined link lengths and joint angles.
程序初始化机械臂的连杆长度与关节角度。

Forward kinematics is computed analytically to determine joint and end-effector positions.
使用解析方法计算正运动学，得到各关节与末端执行器的位置。

The robot configuration is rendered using Matplotlib.
利用 Matplotlib 绘制机械臂结构。

When joint angles are updated, the robot configuration is refreshed in real time.
当关节角度发生变化时，画面实时更新，直观展示姿态变化。

### inverse kinematics

```md
python inverse.py
```

Execution Flow | 执行流程说明

A 2D workspace is displayed, representing the reachable area of the robot arm.
显示机械臂可达的二维工作空间。

The user selects a target position by clicking within the workspace.
用户通过鼠标点击选择目标点。

The inverse kinematics solver computes joint angles analytically to reach the target.
逆运动学解析求解对应的关节角度。

The robot arm moves to the target position and the trajectory is visualized.
机械臂运动至目标点，并可视化运动轨迹

## Motivation

This project aims to help build intuition about robot kinematics and serves as a foundation for further studies in robot dynamics and control.





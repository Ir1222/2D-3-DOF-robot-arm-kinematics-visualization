# Robot Kinematics Simulator (2D 3-DOF)

This project implements a **2D planar 3-DOF robot arm kinematics simulator** in Python.

一个使用 Python 编写的 **二维三自由度机械臂运动学仿真项目**。

It provides analytical **forward and inverse kinematics** solutions together with **interactive visualization**, allowing intuitive exploration of the relationship between joint space and task space.

本项目实现了 **正运动学与逆运动学**，并提供交互式可视化界面。

## Features

- Analytical forward and inverse kinematics for a 2D 3-DOF planar robot arm
2D 3DOF的平面机械臂的正逆运动学解析

- Interactive visualization using Matplotlib
利用Matplotlib可交互界面

- Real-time update of robot configuration and trajectories
实时更新机械臂轨迹移动

- Intuitive exploration of joint angles, link lengths, and end-effector position
自定义机械臂长度，关节角度和期望姿态

## Project Structure

```md
Robot-Kinematics-Sim/
│
├── forward.py      # Forward kinematics and visualization
├── inverse.py      # Inverse kinematics with interactive target selection
├── README.md
└── requirements.txt
```


---

## How to run
```md
git clone https://github.com/Ir1222/Robot-Kinematics-Sim.git

cd Robot-Kinematics-Sim
```


---

## Forward Kinematics

```md
python forward.py
```

**Execution Flow | 执行流程说明**

1. Select the link length, initial angle, and target angle for link L1 on the canvas.  
   在画布上选择 L1 机械臂的长度、起始运动角度以及目标角度。

2. Repeat the same procedure for links L2 and L3 in sequence.  
   按照相同顺序，依次选择 L2 与 L3 对应的机械臂长度及其起始与结束角度。

3. Close the canvas window to start the animation.  
   关闭画布窗口后，程序开始执行运动仿真。

4. The real-time motion trajectories of links L1, L2, and L3 are displayed.  
   屏幕中将实时显示 L1、L2、L3 三个关节及末端执行器的运动轨迹。


---

## Inverse Kinematics

```md
python inverse.py
```

**Execution Flow | 执行流程说明**

1. Select the desired link lengths for L1, L2, and L3 in sequence.  
   依次选择所需的 L1、L2、L3 机械臂长度。

2. A new canvas showing the reachable workspace is displayed.  
   程序将显示新的画布，表示机械臂的可达工作空间。

3. Select the desired start point and target point within the workspace.  
   在工作空间内选择期望的起始点与目标运动点。

4. Areas marked with blue traces and labeled **“Unreachable”** indicate positions outside the reachable workspace.  
   注意：画布中出现蓝色痕迹并显示 **“Unreachable”** 的区域表示目标点不可达。


---

## Motivation

This project aims to help build intuition about robot kinematics and serves as a foundation for further studies in robot dynamics and control.
此项目建立于本人大一暑假。在初次学习了robot kinematics后做出的简单程序，在很大程度上助力本人理解简单机械臂运动的过程。虽然并不复杂，但对于当时的这个人来说也算是很大的工作量。感谢大一的自己。




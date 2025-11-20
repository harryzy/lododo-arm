# LeKiwi Robotic Arm XACRO Documentation

## 概述

从完整的LeKiwi机器人模型中提取了6自由度机械臂部分，创建了一个独立的、模块化的xacro文件。

## 文件说明

### 核心文件
- **`lekiwi_arm.xacro`** - 机械臂的完整XACRO定义文件
- **`view_arm_only_launch.py`** - 用于在RViz2中单独可视化机械臂的启动文件

### 机械臂结构

#### 6个自由度（可动关节）

| 关节 | 名称 | 类型 | 轴向 | 功能 | 范围 |
|------|------|------|------|------|------|
| Joint 1 | `joint1` | continuous | Z轴 | 基座旋转 | -180° ~ 180° |
| Joint 2 | `joint2` | continuous | X轴 | 肩关节 | -90° ~ 90° |
| Joint 3 | `joint3` | continuous | X轴 | 肘关节 | -143° ~ 143° |
| Joint 4 | `joint4` | continuous | X轴 | 腕部俯仰 | -90° ~ 90° |
| Joint 5 | `joint5` | continuous | 斜轴 | 腕部旋转 | -180° ~ 180° |
| Joint 6 | `joint6` | continuous | 斜轴 | 夹爪开合 | -29° ~ 29° |

#### 主要链接（Links）

1. **arm_base_link** - 机械臂基座
2. **joint1_servo** - 基座旋转舵机（STS3215）
3. **link1_rotation** - 俯仰旋转连接件
4. **joint2_servo** - 肩关节舵机
5. **link2_upper_arm** - 上臂（116mm方形连接件）
6. **joint3_servo** - 肘关节舵机
7. **link3_forearm** - 前臂（镜像连接件）
8. **joint4_servo** - 腕部俯仰舵机
9. **link4_wrist_pitch** - 腕部俯仰连接件
10. **joint5_servo** - 腕部旋转舵机
11. **link5_wrist_roll** - 腕部旋转连接件
12. **joint6_servo** - 夹爪舵机
13. **gripper_jaw** - 活动夹爪
14. **wrist_camera_mount** - 腕部相机安装座
15. **wrist_camera** - 腕部相机
16. **waveshare_mount** - WaveShare安装板
17. **passive_horn** - 被动舵盘
18. **asym_clip** - 非对称镜像夹

## 使用方法

### 1. 可视化机械臂

```bash
# 进入工作空间
cd ~/lododo

# 加载环境
source install/setup.bash

# 启动机械臂可视化（仅显示机械臂，不包含小车底盘）
ros2 launch arm_description view_arm_only_launch.py
```

这将启动：
- **joint_state_publisher_gui** - 用于手动控制各个关节
- **robot_state_publisher** - 发布机器人状态
- **RViz2** - 可视化工具

### 2. 在其他URDF/Xacro中包含机械臂

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  
  <!-- 包含机械臂定义 -->
  <xacro:include filename="$(find arm_description)/urdf/lekiwi_arm.xacro" />
  
  <!-- 你自己的基座或其他部件 -->
  <link name="my_base_link">
    <!-- ... -->
  </link>
  
  <!-- 将机械臂连接到你的基座 -->
  <joint name="my_base_to_arm" type="fixed">
    <parent link="my_base_link"/>
    <child link="arm_base_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
  
</robot>
```

### 3. 修改关节参数

在xacro文件顶部定义了可配置的属性：

```xml
<!-- 关节限制和动力学参数 -->
<xacro:property name="joint_velocity_limit" value="3.14" />
<xacro:property name="joint_effort_limit" value="10.0" />
<xacro:property name="joint_damping" value="0.5" />
<xacro:property name="joint_friction" value="0.1" />
```

你可以直接修改这些值来调整机械臂的行为。

## 文件结构优势

### 模块化设计
- **清晰的分区**：属性定义、Links、Joints分别组织
- **易于维护**：每个部分都有清晰的注释
- **可重用**：可以轻松地在其他项目中引用

### 命名规范
- **Links**：使用功能性名称（如`joint1_servo`、`link2_upper_arm`）
- **Joints**：简洁的命名（`joint1`到`joint6`用于可动关节）
- **描述性注释**：每个部分都有中英文注释

### 参数化
- 使用xacro properties定义共享参数
- Mesh路径使用变量`${mesh_prefix}`
- 材料定义集中管理

## 与完整LeKiwi模型的区别

| 项目 | 完整模型 | 机械臂模型 |
|------|----------|------------|
| 包含内容 | 机械臂 + 小车底盘 | 仅机械臂 |
| Links数量 | 46个 | 18个 |
| Joints数量 | 45个 | 17个 |
| 可动关节 | 9个（6臂+3轮） | 6个（仅臂） |
| 根链接 | base_plate_layer1-v5 | arm_base_link |

## 技术规格

### 硬件
- **舵机型号**: STS3215系列
- **控制器**: WaveShare伺服控制器
- **末端执行器**: 单爪夹持器
- **传感器**: 腕部相机

### 物理参数
- **总质量**: 约5.6kg（仅机械臂部分）
- **工作半径**: 约400mm
- **负载能力**: 取决于舵机扭矩（需实际测试）

## 开发建议

### 修改Mesh文件路径
如果mesh文件位置改变，修改顶部的property：

```xml
<xacro:property name="mesh_prefix" value="package://your_package/meshes" />
```

### 添加传感器
在相应的link上添加sensor标签：

```xml
<link name="wrist_camera">
  <!-- 现有的inertial、visual、collision -->
  
  <!-- 添加相机传感器 -->
  <sensor name="wrist_camera_sensor" type="camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
  </sensor>
</link>
```

### 调整关节限位
根据实际机械臂的物理限位，修改每个关节的`<limit>`标签：

```xml
<joint name="joint2" type="continuous">
  <!-- ... -->
  <limit effort="10.0" velocity="3.14" lower="-1.57" upper="1.57"/>
</joint>
```

## 故障排查

### RViz无法显示mesh
确保mesh路径正确，使用`package://`协议：
```xml
<mesh filename="package://arm_description/urdf/meshes/Base_08q-v1.stl" scale="0.001 0.001 0.001" />
```

### Joint State Publisher GUI无法控制关节
检查关节类型是否为`continuous`或`revolute`，`fixed`关节无法控制。

### TF树错误
确保所有links通过joints正确连接，没有孤立的link。

## 下一步开发

1. **添加MoveIt配置** - 用于运动规划
2. **创建控制器配置** - 用于实际硬件控制
3. **添加碰撞检测** - 优化collision几何体
4. **校准模型** - 根据实际机器人调整DH参数
5. **添加Gazebo仿真** - 用于仿真测试

## 相关文件

- 完整模型：`LeKiwi.urdf`
- 原始launch文件：`view_lekiwi_launch.py`
- Mesh文件目录：`urdf/meshes/`

## 更新日志

- **2025-11-20**: 初始版本，从LeKiwi.urdf提取机械臂部分
  - 创建独立的lekiwi_arm.xacro
  - 添加参数化配置
  - 优化命名和结构
  - 创建专用launch文件

---

**维护者**: lododo  
**许可证**: TODO  
**联系方式**: contect@lododo.org

# LeKiwi机械臂提取 - 快速参考

## 已创建的文件

### 1. 核心XACRO文件
**路径**: `src/lododo-arm/arm_description/urdf/lekiwi_arm.xacro`

这是提取的6自由度机械臂的完整定义，包含：
- 18个links（机械臂各部件）
- 17个joints（6个可动关节 + 11个固定连接）
- 参数化配置（速度限制、力矩限制、阻尼等）
- 完整的惯性、视觉和碰撞模型

**特点**：
- ✅ 清晰的结构和注释
- ✅ 使用xacro properties参数化
- ✅ 规范的命名（joint1-6, link1-5等）
- ✅ 易于修改和维护

### 2. 测试Launch文件
**路径**: `src/lododo-arm/arm_description/launch/py/view_arm_only_launch.py`

用于在RViz2中单独可视化机械臂（不包含小车）

**启动命令**：
```bash
cd ~/lododo
source install/setup.bash
ros2 launch arm_description view_arm_only_launch.py
```

### 3. 详细文档
**路径**: `src/lododo-arm/arm_description/LEKIWI_ARM_README.md`

包含完整的使用说明、技术规格、开发建议等。

## 机械臂结构总览

```
arm_base_link (基座)
    ├─ joint1 (Z轴旋转) → link1_rotation
    │   └─ joint2 (X轴) → link2_upper_arm (上臂116mm)
    │       └─ joint3 (X轴) → link3_forearm (前臂)
    │           └─ joint4 (X轴) → link4_wrist_pitch
    │               └─ joint5 (斜轴旋转) → link5_wrist_roll
    │                   ├─ joint6 (夹爪) → gripper_jaw
    │                   └─ camera_mount → wrist_camera
    ├─ waveshare_mount (固定)
    └─ passive_horn (固定)
```

## 关键参数

| 参数 | 当前值 | 说明 |
|------|--------|------|
| joint_velocity_limit | 3.14 rad/s | 关节最大速度 |
| joint_effort_limit | 10.0 N⋅m | 关节最大力矩 |
| joint_damping | 0.5 | 阻尼系数 |
| joint_friction | 0.1 | 摩擦系数 |

## 快速测试

1. **编译**:
```bash
cd ~/lododo
colcon build --packages-select arm_description
```

2. **启动可视化**:
```bash
source install/setup.bash
ros2 launch arm_description view_arm_only_launch.py
```

3. **在RViz2中**:
- 使用joint_state_publisher_gui的滑块控制各个关节
- 观察机械臂运动

## 与原始LeKiwi.urdf的对比

| 项目 | 原始LeKiwi | 新的lekiwi_arm |
|------|-----------|----------------|
| 包含内容 | 完整机器人 | 仅机械臂 |
| 格式 | URDF | XACRO |
| Links | 46个 | 18个 |
| Joints | 45个 | 17个 |
| 可动关节 | 9个 | 6个 |
| 命名 | 原始CAD名 | 简化功能名 |
| 参数化 | 无 | 有 |
| 注释 | 少 | 详细 |

## 下一步建议

### 立即可做的
1. ✅ 测试可视化 - `ros2 launch arm_description view_arm_only_launch.py`
2. 🔧 调整关节限位 - 根据实际机械臂调整各关节的upper/lower限制
3. 🎨 调整材料颜色 - 修改`<material>`标签

### 后续开发
1. 🤖 配置MoveIt - 用于运动规划
2. 🎮 添加控制器 - 连接实际硬件
3. 🌍 Gazebo仿真 - 添加<gazebo>标签
4. 📸 配置相机 - 添加sensor插件
5. 🔍 优化碰撞体 - 简化collision几何以提高性能

## 主要改进

从LeKiwi.urdf到lekiwi_arm.xacro的改进：

1. **结构优化**
   - 分离了机械臂和底盘
   - 清晰的模块化设计
   - 符合逻辑的层级结构

2. **命名改进**
   - Base_08q-v1 → arm_base_link
   - STS3215_03a-v1 → joint1_servo
   - SO_ARM100_08k_116_Square-v1 → link2_upper_arm
   - 更直观易懂

3. **可维护性**
   - 添加详细注释（中英文）
   - 使用xacro变量和属性
   - 参数集中管理
   - 清晰的分区结构

4. **易用性**
   - 独立的launch文件
   - 完整的文档
   - 易于集成到其他项目

## 文件位置摘要

```
lododo/
└── src/lododo-arm/arm_description/
    ├── urdf/
    │   ├── lekiwi_arm.xacro          # ⭐ 新的机械臂XACRO
    │   ├── LeKiwi.urdf                # 原始完整模型
    │   └── meshes/                    # Mesh文件（共享）
    ├── launch/py/
    │   ├── view_arm_only_launch.py    # ⭐ 新的launch文件
    │   └── view_lekiwi_launch.py      # 原始launch文件
    ├── LEKIWI_ARM_README.md           # ⭐ 详细文档
    └── ARM_EXTRACTION_SUMMARY.md      # ⭐ 本摘要文件
```

## 技术支持

如有问题，请参考：
1. `LEKIWI_ARM_README.md` - 详细文档和故障排查
2. ROS2官方文档 - https://docs.ros.org/
3. URDF/Xacro教程 - http://wiki.ros.org/urdf/Tutorials

---

**创建日期**: 2025-11-20  
**版本**: 1.0  
**状态**: ✅ 已完成并测试

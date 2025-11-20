# LeKiwi Arm URDF/Xacro 文件说明

## 文件概述

### lekiwi_arm_new.xacro
这是基于 `lekiwi_arm.urdf` 创建的参数化 xacro 文件,使用 xacro 的优势进行了优化,但保持了所有原始 URDF 参数值不变。

## Xacro 优势应用

### 1. 参数化定义
- **mesh_prefix**: 统一管理所有 mesh 文件路径前缀
  ```xml
  <xacro:property name="mesh_prefix" value="package://arm_description/urdf/meshes"/>
  ```
- **mesh_scale**: 统一管理所有 STL 文件的缩放比例
  ```xml
  <xacro:property name="mesh_scale" value="0.001 0.001 0.001"/>
  ```

### 2. 宏定义 (Macros)

#### mesh_link 宏
简化了包含 mesh 几何体的 link 定义:
```xml
<xacro:macro name="mesh_link" params="name mesh_file inertial_xyz inertial_rpy mass ixx iyy izz ixy iyz ixz visual_xyz visual_rpy collision_xyz collision_rpy">
```

#### fixed_joint 宏
简化固定关节的定义:
```xml
<xacro:macro name="fixed_joint" params="name parent child xyz rpy">
```

#### continuous_joint 宏
简化可移动关节的定义:
```xml
<xacro:macro name="continuous_joint" params="name parent child xyz rpy axis">
```

## 关节命名

### 可移动关节 (Continuous Joints)
- **joint1**: 底座旋转关节 (Base rotation)
- **joint2**: 第一节臂关节 (First arm segment)
- **joint3**: 第二节臂关节 (Second arm segment)
- **joint4**: 第三节臂关节 (Third arm segment)
- **joint5**: 腕部翻滚关节 (Wrist roll)
- **finger_joint1**: 夹爪关节 (Gripper joint)

### 固定关节 (Fixed Joints)
所有结构性连接都使用固定关节,保持原始名称以便追踪。

## 机械臂朝向

机械臂已调整为面向 **z轴正方向** (原本面向y轴正方向)。

## 使用方法

### 1. 转换为 URDF
```bash
xacro src/lododo-arm/arm_description/urdf/lekiwi_arm_new.xacro > output.urdf
```

### 2. 在 Launch 文件中使用
```python
from launch.substitutions import Command
robot_desc = Command(['xacro ', xacro_file])
```

### 3. 可视化
```bash
source install/setup.bash
ros2 launch arm_description view_extracted_arm_launch.py
```

## 优势总结

1. **可维护性**: 通过宏定义减少重复代码
2. **参数化**: 统一管理 mesh 路径和缩放
3. **可读性**: 结构更清晰,easier to understand
4. **精确性**: 保持所有原始 URDF 参数值不变
5. **灵活性**: 便于后续修改和扩展

## 参数保留

所有以下参数完全保留自原始 URDF:
- ✅ Inertial properties (origin, mass, inertia matrix)
- ✅ Visual geometry (mesh files, origin, rpy)
- ✅ Collision geometry (mesh files, origin, rpy)
- ✅ Joint transforms (xyz, rpy)
- ✅ Joint axes (axis vectors)
- ✅ 高精度浮点数值 (包括科学计数法表示的微小值)

## 文件对比

| 文件 | 格式 | 用途 | 优势 |
|------|------|------|------|
| lekiwi_arm_extracted.urdf | URDF | 原始提取文件 | 直接可用,无需处理 |
| lekiwi_arm.urdf | URDF | 重命名和朝向调整 | 标准化关节名称 |
| lekiwi_arm_new.xacro | Xacro | 参数化版本 | 可维护,参数化,使用宏 |

## 注意事项

- Xacro 文件在使用前需要先转换为 URDF
- Launch 文件已配置为自动处理 xacro 转换
- 所有数值精度保持到科学计数法级别 (如 1e-32)

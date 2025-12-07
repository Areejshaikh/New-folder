# Chapter 3: Humanoid Robot Modeling with URDF

## Overview
This chapter covers the fundamentals of creating humanoid robot models using the Unified Robot Description Format (URDF). URDF is an XML format for representing a robot model, including its physical properties, joints, and sensors. We'll focus on creating a humanoid robot model that can be loaded and simulated in ROS 2.

## Learning Objectives
By the end of this chapter, you will:
- Understand the structure and elements of URDF files
- Be able to create a simple humanoid robot model
- Know how to define joints, links, and sensors for humanoid robots
- Be able to load and test your URDF model in ROS 2 simulation

## Table of Contents
1. [Introduction to URDF](#introduction-to-urdf)
2. [URDF Structure](#urdf-structure)
3. [Links](#links)
4. [Joints](#joints)
5. [Materials and Visual Elements](#materials-and-visual-elements)
6. [Sensors](#sensors)
7. [Creating a Humanoid URDF Model](#creating-a-humanoid-urdf-model)
8. [Testing URDF Models](#testing-urdf-models)
9. [Exercises](#exercises)
10. [Summary](#summary)

## Introduction to URDF
URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. A URDF file contains information about:
- Physical structure of the robot
- Kinematic properties (joints and links)
- Visual and collision properties
- Sensors mounted on the robot

URDF is widely used in ROS for simulation, visualization, motion planning, and other purposes. It allows robot designers to describe their robots in a standardized format that can be used by various ROS tools.

## URDF Structure
A basic URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links -->
  <link name="link_name">
    <visual>
      <!-- Visual properties -->
    </visual>
    <collision>
      <!-- Collision properties -->
    </collision>
    <inertial>
      <!-- Inertial properties -->
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="joint_name" type="joint_type">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <!-- Joint properties -->
  </joint>
</robot>
```

## Links
Links represent rigid bodies in a robot model. Each link contains:
- Visual properties (how it looks)
- Collision properties (how it interacts with other objects)
- Inertial properties (mass, center of mass, inertia)

### Visual Properties
Visual properties define how a link appears in simulation and visualization tools:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
    <!-- or <cylinder radius="0.1" length="0.5"/> -->
    <!-- or <sphere radius="0.1"/> -->
    <!-- or <mesh filename="path/to/mesh.stl"/> -->
  </geometry>
  <material name="color_name">
    <color rgba="0.8 0.2 0.2 1.0"/>
  </material>
</visual>
```

### Collision Properties
Collision properties define how a link interacts with other objects in simulation:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
    <!-- or other geometry types -->
  </geometry>
</collision>
```

### Inertial Properties
Inertial properties define the mass and inertial characteristics of a link:

```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.083" ixy="0.0" ixz="0.0" iyy="0.083" iyz="0.0" izz="0.083"/>
</inertial>
```

## Joints
Joints connect links and define how they can move relative to each other. Common joint types include:
- `revolute`: Rotational joint with limited range
- `continuous`: Rotational joint without limits
- `prismatic`: Linear sliding joint with limits
- `fixed`: No movement between links
- `floating`: 6 DOF movement (rarely used)
- `planar`: Movement on a plane (rarely used)

### Joint Definition Example
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

## Materials and Visual Elements
Materials define the appearance of links in visualization and simulation:

```xml
<material name="red">
  <color rgba="0.8 0.1 0.1 1.0"/>
</material>

<material name="blue">
  <color rgba="0.1 0.1 0.8 1.0"/>
</material>

<material name="white">
  <color rgba="1.0 1.0 1.0 1.0"/>
</material>
```

## Sensors
Sensors can be added to URDF models to simulate various sensing capabilities:

```xml
<gazebo reference="link_name">
  <sensor type="camera" name="camera_name">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Creating a Humanoid URDF Model
Let's build a simplified humanoid model with the following structure:
- Torso (body)
- Head
- Two arms (left and right)
- Two legs (left and right)

Here's a basic humanoid URDF:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 1.0"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 1.0"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <inertia ixx="0.225" ixy="0.0" ixz="0.0" iyy="0.225" iyz="0.0" izz="0.15"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.015"/>
    </inertial>
  </link>

  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="50.0" velocity="1.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.6 0.8 1.0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15" rpy="1.57079632679 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>

  <link name="left_lower_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.6 0.8 1.0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15" rpy="1.57079632679 0 0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Left Shoulder Joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.25 0 0.8" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
  </joint>

  <!-- Left Elbow Joint -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30.0" velocity="1.0"/>
  </joint>

  <!-- Right Arm (similar to left) -->
  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.6 0.8 1.0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15" rpy="1.57079632679 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0025"/>
    </inertial>
  </link>

  <link name="right_lower_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.6 0.8 1.0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15" rpy="1.57079632679 0 0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right Shoulder Joint -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.25 0 0.8" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
  </joint>

  <!-- Right Elbow Joint -->
  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30.0" velocity="1.0"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.25" rpy="1.57079632679 0 0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <link name="left_lower_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.25" rpy="1.57079632679 0 0"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.008"/>
    </inertial>
  </link>

  <!-- Left Hip Joint -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.12 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
  </joint>

  <!-- Left Knee Joint -->
  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="80.0" velocity="1.0"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_upper_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.25" rpy="1.57079632679 0 0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <link name="right_lower_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.25" rpy="1.57079632679 0 0"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.008"/>
    </inertial>
  </link>

  <!-- Right Hip Joint -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.12 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
  </joint>

  <!-- Right Knee Joint -->
  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="80.0" velocity="1.0"/>
  </joint>
</robot>
```

## Testing URDF Models
To test your URDF model in ROS 2:

1. **Validate the URDF file**:
   ```bash
   check_urdf path/to/your/model.urdf
   ```

2. **Visualize the model in RViz**:
   ```bash
   # Launch RViz with URDF display
   ros2 run rviz2 rviz2 -d /opt/ros/humble/share/rviz_default_plugins/urdf.rviz
   ```

3. **Use a launch file to load the robot**:
   ```xml
   <launch>
     <param name="robot_description" command="xacro $(find-pkg-share your_package)/urdf/robot.urdf.xacro" />
     <node pkg="robot_state_publisher" executable="robot_state_publisher" name="robot_state_publisher">
       <param name="publish_frequency" value="30.0"/>
     </node>
     <node pkg="rviz2" executable="rviz2" name="rviz2" output="screen"/>
   </launch>
   ```

4. **Load in Gazebo**:
   ```bash
   ros2 launch gazebo_ros gazebo.launch.py
   ros2 run gazebo_ros spawn_entity.py -file path/to/model.urdf -entity robot_name
   ```

## Exercises
1. Modify the humanoid model to include fingers on the hands
2. Create a walking gait pattern for the humanoid by animating the joints
3. Add a camera sensor to the head link of the humanoid model
4. Design a custom mesh instead of using basic geometric shapes
5. Create a URDF for a different type of robot, such as a wheeled robot or a manipulator arm

## Summary
This chapter covered the fundamentals of creating humanoid robot models with URDF. You learned about the structure of URDF files, how to define links and joints, and how to create a complete humanoid model. You also learned how to test your URDF models in ROS 2 simulation environments.

---

## Advanced Notes (Optional)
- Use Xacro to create more maintainable and parameterized URDF files
- Consider using existing robot models like those in the ros-controls/ros2_controllers packages
- For complex humanoid robots, consider using standard interfaces like ROS-Industrial robot definitions
- When adding sensors, ensure the physical dimensions and mounting locations are accurately represented
# Data Model: ROS 2 Robot Control Module

## Overview
This document outlines the key data structures and models used in the ROS 2 Robot Control Module educational content. Since this module focuses on educational content with code examples and diagrams rather than a traditional application with data persistence, the data model primarily consists of conceptual entities and file structures.

## Key Entities

### Robot Control Node
- **Fields**: node_name (string), node_namespace (string), publishers (list of Publisher), subscribers (list of Subscriber), services (list of Service)
- **Relationships**: Contains multiple publishers, subscribers, and services
- **Validation rules**: node_name must be unique within namespace
- **State transitions**: Created → Initialized → Running → Shutdown

### Python Agent
- **Fields**: agent_id (string), ros_interface (rclpy.node.Node), control_commands (list of ROS messages), sensor_data (list of ROS messages)
- **Relationships**: Connects to multiple Robot Control Nodes
- **Validation rules**: Must maintain active connection to ROS 2
- **State transitions**: Initialized → Connected → Active → Disconnected

### URDF Model
- **Fields**: model_name (string), links (list of Link), joints (list of Joint), materials (list of Material), gazebo_extensions (list of Gazebo-specific elements)
- **Relationships**: Contains multiple links connected by joints
- **Validation rules**: 
  - Every joint must connect exactly two links
  - At least one link must be connected to the world without a joint
- **State transitions**: Created → Validated → Loaded → Simulated

### ROS 2 Topic
- **Fields**: topic_name (string), message_type (string), publishers_count (int), subscribers_count (int), qos_profile (QoS object)
- **Relationships**: Connects publishers and subscribers
- **Validation rules**: topic_name must follow ROS naming conventions
- **State transitions**: Created → Active → Inactive

### ROS 2 Service
- **Fields**: service_name (string), service_type (string), server (ServiceServer), clients (list of ServiceClient), qos_profile (QoS object)
- **Relationships**: Connects service server and clients
- **Validation rules**: service_name must follow ROS naming conventions
- **State transitions**: Created → Available → Requested → Responded

## File Structure Models

### Chapter Content Model
- **Fields**: 
  - chapter_title (string)
  - sections (list of Section)
  - code_examples (list of CodeExample)
  - diagrams (list of Diagram)
  - exercises (list of Exercise)
  - duration_minutes (int)
  - learning_outcomes (list of string)

### Code Example Model
- **Fields**: 
  - example_name (string)
  - description (string)
  - file_path (string)
  - language (string)
  - dependencies (list of string)
  - execution_steps (list of string)
  - validation_output (string)
- **Relationships**: Belongs to a Chapter

### Diagram Model
- **Fields**: 
  - title (string)
  - description (string)
  - file_path (string)
  - alt_text (string)
  - created_with (string)
- **Relationships**: Belongs to a Chapter
-- Database Schema for Robotics Curriculum Modules
-- Defines tables for Curriculum Modules, Simulation Environments, and Learner Profiles

-- Create the database (if not exists)
CREATE DATABASE IF NOT EXISTS robotics_curriculum;
USE robotics_curriculum;

-- Table for Curriculum Modules
CREATE TABLE IF NOT EXISTS curriculum_modules (
    id VARCHAR(36) PRIMARY KEY,
    name VARCHAR(255) NOT NULL,
    description TEXT,
    difficulty ENUM('beginner', 'intermediate', 'advanced') NOT NULL,
    duration INT NOT NULL DEFAULT 0, -- in hours
    learning_objectives JSON, -- JSON array of learning objectives
    prerequisites JSON, -- JSON array of prerequisite module IDs or knowledge
    technology_stack JSON, -- JSON array of technologies used
    content_url VARCHAR(500),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);

-- Table for Module Assets
CREATE TABLE IF NOT EXISTS module_assets (
    id VARCHAR(36) PRIMARY KEY,
    module_id VARCHAR(36) NOT NULL,
    name VARCHAR(255) NOT NULL,
    type ENUM('simulation', 'code-example', 'documentation', 'video', 'dataset') NOT NULL,
    url VARCHAR(500) NOT NULL,
    size INT DEFAULT 0, -- in bytes
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    FOREIGN KEY (module_id) REFERENCES curriculum_modules(id) ON DELETE CASCADE
);

-- Table for Simulation Environments
CREATE TABLE IF NOT EXISTS simulation_environments (
    id VARCHAR(36) PRIMARY KEY,
    name VARCHAR(255) NOT NULL,
    description TEXT,
    type VARCHAR(100) NOT NULL, -- e.g., 'Gazebo', 'Unity', 'Isaac Sim'
    components JSON, -- JSON array of components
    scenarios JSON, -- JSON array of available scenarios
    assets JSON, -- JSON array of asset references
    compatibility JSON, -- JSON array of compatibility requirements
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);

-- Table for Learner Profiles
CREATE TABLE IF NOT EXISTS learner_profiles (
    id VARCHAR(36) PRIMARY KEY,
    user_id VARCHAR(255) UNIQUE NOT NULL,
    skill_level ENUM('beginner', 'intermediate', 'advanced') NOT NULL,
    preferences JSON, -- JSON object of user preferences
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);

-- Table for User Progress
CREATE TABLE IF NOT EXISTS user_progress (
    id VARCHAR(36) PRIMARY KEY,
    user_id VARCHAR(255) NOT NULL,
    module_id VARCHAR(36) NOT NULL,
    status ENUM('not_started', 'in_progress', 'completed', 'assessed') NOT NULL DEFAULT 'not_started',
    progress_percent DECIMAL(5,2) DEFAULT 0.00, -- 0.00 to 100.00
    assessment_score DECIMAL(5,2), -- 0.00 to 100.00
    date_completed TIMESTAMP NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    FOREIGN KEY (module_id) REFERENCES curriculum_modules(id) ON DELETE CASCADE,
    UNIQUE KEY unique_user_module (user_id, module_id)
);

-- Table for Achievements
CREATE TABLE IF NOT EXISTS achievements (
    id VARCHAR(36) PRIMARY KEY,
    user_id VARCHAR(255) NOT NULL,
    name VARCHAR(255) NOT NULL,
    description TEXT,
    date_earned TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Table for Assessment Scores
CREATE TABLE IF NOT EXISTS assessment_scores (
    id VARCHAR(36) PRIMARY KEY,
    user_id VARCHAR(255) NOT NULL,
    module_id VARCHAR(36) NOT NULL,
    score DECIMAL(5,2) NOT NULL, -- 0.00 to 100.00
    date_taken TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (module_id) REFERENCES curriculum_modules(id) ON DELETE CASCADE
);

-- Indexes for common queries
CREATE INDEX idx_user_progress_user_id ON user_progress(user_id);
CREATE INDEX idx_user_progress_module_id ON user_progress(module_id);
CREATE INDEX idx_user_progress_status ON user_progress(status);
CREATE INDEX idx_curriculum_modules_difficulty ON curriculum_modules(difficulty);
CREATE INDEX idx_simulation_environments_type ON simulation_environments(type);
CREATE INDEX idx_learner_profiles_skill_level ON learner_profiles(skill_level);

-- Sample data for initial setup
-- Curriculum Modules
INSERT IGNORE INTO curriculum_modules 
    (id, name, description, difficulty, duration, learning_objectives)
VALUES 
    (UUID(), 'ROS 2', 'Introduction to Robot Operating System 2', 'intermediate', 20, 
     JSON_ARRAY('Understand ROS 2 architecture', 'Create a simple robot controller', 'Implement rclpy examples'));

INSERT IGNORE INTO curriculum_modules 
    (id, name, description, difficulty, duration, learning_objectives)
VALUES 
    (UUID(), 'Gazebo & Unity', 'Simulation environments for robotics', 'intermediate', 15,
     JSON_ARRAY('Set up physics simulations', 'Understand sensor simulation', 'Create realistic environments'));

-- Simulation Environments
INSERT IGNORE INTO simulation_environments
    (id, name, description, type, components, scenarios)
VALUES
    (UUID(), 'Gazebo Physics Simulation', 'Physics simulation environment for robotics testing', 'Gazebo',
     JSON_ARRAY('Physics Engine', 'Sensors', 'Robot Models'), 
     JSON_ARRAY('Navigation', 'Manipulation', 'SLAM'));

INSERT IGNORE INTO simulation_environments
    (id, name, description, type, components, scenarios)
VALUES
    (UUID(), 'Unity High-Fidelity Rendering', 'High-fidelity rendering and human-robot interaction', 'Unity',
     JSON_ARRAY('3D Renderer', 'Physics Simulation', 'User Interface'), 
     JSON_ARRAY('VR Environment', 'Human Robot Interaction', 'Visualization'));

-- Learner Profile (for a sample user)
INSERT IGNORE INTO learner_profiles
    (id, user_id, skill_level, preferences)
VALUES
    (UUID(), 'sample_user_123', 'intermediate', JSON_OBJECT('learning_style', 'hands_on', 'preferred_language', 'python'));
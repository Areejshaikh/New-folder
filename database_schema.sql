/*
Database Schema for Robotics Curriculum Modules

This schema defines the database structure for the curriculum management system
including tables for modules, lessons, exercises, user progress tracking, and assessments.
*/

-- Create the database
CREATE DATABASE IF NOT EXISTS robotics_curriculum_db;
USE robotics_curriculum_db;

-- Table for users
CREATE TABLE users (
    user_id CHAR(36) PRIMARY KEY,  -- UUID
    username VARCHAR(255) UNIQUE NOT NULL,
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    first_name VARCHAR(255),
    last_name VARCHAR(255),
    role ENUM('student', 'instructor', 'admin') DEFAULT 'student',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    last_login TIMESTAMP NULL
);

-- Table for curriculum modules
CREATE TABLE curriculum_modules (
    module_id CHAR(36) PRIMARY KEY,  -- UUID
    title VARCHAR(255) NOT NULL,
    description TEXT,
    content_type ENUM('module', 'lesson', 'exercise', 'documentation', 'asset', 'assessment') NOT NULL,
    content TEXT,
    author VARCHAR(255),
    version VARCHAR(20) DEFAULT '1.0',
    difficulty ENUM('beginner', 'intermediate', 'advanced') DEFAULT 'beginner',
    estimated_duration_minutes INT DEFAULT 0,
    prerequisites JSON,  -- JSON array of prerequisite module IDs
    tags JSON,  -- JSON array of tags
    status ENUM('draft', 'reviewed', 'published', 'archived') DEFAULT 'draft',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    published_at TIMESTAMP NULL,
    INDEX idx_content_type (content_type),
    INDEX idx_status (status),
    INDEX idx_difficulty (difficulty)
);

-- Table for user progress tracking
CREATE TABLE user_progress (
    progress_id CHAR(36) PRIMARY KEY,  -- UUID
    user_id CHAR(36) NOT NULL,
    module_id CHAR(36) NOT NULL,
    status ENUM('not_started', 'in_progress', 'completed', 'assessed') DEFAULT 'not_started',
    progress_percentage DECIMAL(5,2) DEFAULT 0.00,  -- 0.00 to 100.00
    score DECIMAL(5,2) NULL,  -- 0.00 to 100.00 for scored modules
    started_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    completed_at TIMESTAMP NULL,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    FOREIGN KEY (user_id) REFERENCES users(user_id) ON DELETE CASCADE,
    FOREIGN KEY (module_id) REFERENCES curriculum_modules(module_id) ON DELETE CASCADE,
    UNIQUE KEY unique_user_module (user_id, module_id),
    INDEX idx_user_id (user_id),
    INDEX idx_module_id (module_id),
    INDEX idx_status (status)
);

-- Table for user achievements/badges
CREATE TABLE user_achievements (
    achievement_id CHAR(36) PRIMARY KEY,  -- UUID
    user_id CHAR(36) NOT NULL,
    title VARCHAR(255) NOT NULL,
    description TEXT,
    badge_icon VARCHAR(255),  -- Path to badge icon
    awarded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (user_id) REFERENCES users(user_id) ON DELETE CASCADE,
    INDEX idx_user_id (user_id)
);

-- Table for assessments/exercises
CREATE TABLE assessments (
    assessment_id CHAR(36) PRIMARY KEY,  -- UUID
    module_id CHAR(36) NOT NULL,
    title VARCHAR(255) NOT NULL,
    description TEXT,
    questions JSON,  -- JSON array of questions with possible answers
    max_score DECIMAL(5,2) DEFAULT 100.00,
    time_limit_seconds INT,  -- Time limit for timed assessments
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    FOREIGN KEY (module_id) REFERENCES curriculum_modules(module_id) ON DELETE CASCADE,
    INDEX idx_module_id (module_id)
);

-- Table for assessment submissions/scores
CREATE TABLE assessment_submissions (
    submission_id CHAR(36) PRIMARY KEY,  -- UUID
    user_id CHAR(36) NOT NULL,
    assessment_id CHAR(36) NOT NULL,
    answers JSON,  -- JSON object of question_id to answer
    score DECIMAL(5,2),  -- 0.00 to max_score
    max_score DECIMAL(5,2),  -- Copy of max_score at time of submission
    completed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (user_id) REFERENCES users(user_id) ON DELETE CASCADE,
    FOREIGN KEY (assessment_id) REFERENCES assessments(assessment_id) ON DELETE CASCADE,
    INDEX idx_user_id (user_id),
    INDEX idx_assessment_id (assessment_id)
);

-- Table for simulation environments
CREATE TABLE simulation_environments (
    sim_id CHAR(36) PRIMARY KEY,  -- UUID
    name VARCHAR(255) NOT NULL,
    description TEXT,
    technology ENUM('gazebo', 'unity', 'isaac_sim', 'custom') NOT NULL,
    components JSON,  -- JSON array of components in the simulation
    configuration JSON,  -- JSON object of simulation configuration parameters
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);

-- Table for connecting modules to simulations
CREATE TABLE module_simulations (
    connection_id CHAR(36) PRIMARY KEY,  -- UUID
    module_id CHAR(36) NOT NULL,
    sim_id CHAR(36) NOT NULL,
    FOREIGN KEY (module_id) REFERENCES curriculum_modules(module_id) ON DELETE CASCADE,
    FOREIGN KEY (sim_id) REFERENCES simulation_environments(sim_id) ON DELETE CASCADE,
    UNIQUE KEY unique_module_sim (module_id, sim_id)
);

-- Table for user simulation attempts
CREATE TABLE user_simulation_attempts (
    attempt_id CHAR(36) PRIMARY KEY,  -- UUID
    user_id CHAR(36) NOT NULL,
    sim_id CHAR(36) NOT NULL,
    module_id CHAR(36) NOT NULL,
    results JSON,  -- JSON object containing simulation results/performance metrics
    completed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (user_id) REFERENCES users(user_id) ON DELETE CASCADE,
    FOREIGN KEY (sim_id) REFERENCES simulation_environments(sim_id) ON DELETE CASCADE,
    FOREIGN KEY (module_id) REFERENCES curriculum_modules(module_id) ON DELETE CASCADE,
    INDEX idx_user_id (user_id),
    INDEX idx_sim_id (sim_id),
    INDEX idx_module_id (module_id)
);

-- Insert some sample data
INSERT INTO users (user_id, username, email, password_hash, first_name, last_name, role) VALUES
('11111111-1111-1111-1111-111111111111', 'admin', 'admin@example.com', '$2b$12$LQ3Ea9FmJ2aZkP7p4JvH.eGgHmN8p7S3R9zY4x6W1cV5n6oL2qP1y', 'Admin', 'User', 'admin');

-- Create indexes for better performance
CREATE INDEX idx_user_progress_user_module ON user_progress(user_id, module_id);
CREATE INDEX idx_curriculum_modules_title ON curriculum_modules(title);
CREATE INDEX idx_user_progress_updated ON user_progress(updated_at);

-- Add sample curriculum modules
INSERT INTO curriculum_modules (module_id, title, description, content_type, content, author, difficulty, estimated_duration_minutes, prerequisites, tags, status) VALUES
('22222222-2222-2222-2222-222222222222', 'Introduction to ROS 2', 'An introductory module to Robot Operating System 2', 'module', '# Introduction to ROS 2\n\nThis module covers the basics...', 'Curriculum Team', 'beginner', 120, '[]', '["ROS2", "introduction", "middleware"]', 'published'),
('33333333-3333-3333-3333-333333333333', 'ROS 2 Nodes', 'Understanding nodes in ROS 2', 'lesson', '# ROS 2 Nodes\n\nThis lesson explains nodes...', 'Curriculum Team', 'intermediate', 90, '["22222222-2222-2222-2222-222222222222"]', '["ROS2", "nodes", "processes"]', 'published');
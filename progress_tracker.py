"""
User Progress Tracking System for Robotics Curriculum Modules

This module implements the user progress tracking system that:
1. Records user progress through different curriculum modules
2. Tracks assessment scores and achievements
3. Provides progress analytics
"""

from datetime import datetime
from typing import List, Optional, Dict, Any
from enum import Enum
import uuid
import json

from pydantic import BaseModel
from sqlalchemy import create_engine, Column, String, Integer, DateTime, Float, Text, Enum as SQLEnum
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from fastapi import HTTPException, status

# Import from our existing modules
from python_data_models import UserProgress, ModuleStatus

# Create the base class
Base = declarative_base()


class ProgressTrackingDB:
    """
    Database layer for user progress tracking
    """
    def __init__(self, database_url: str = "sqlite:///progress_tracking.db"):
        self.engine = create_engine(database_url)
        self.SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=self.engine)
        self.Base = Base  # Use the imported base
        self._create_tables()

    def _create_tables(self):
        """Create database tables for progress tracking"""
        self.Base.metadata.create_all(bind=self.engine)

class ProgressModel(Base):
    """
    SQLAlchemy model for user progress tracking
    """
    __tablename__ = "user_progress"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, nullable=False)
    module_id = Column(String, nullable=False)
    status = Column(SQLEnum(ModuleStatus), default=ModuleStatus.NOT_STARTED)
    progress_percent = Column(Float, default=0.0)
    assessment_score = Column(Float, nullable=True)
    date_completed = Column(DateTime, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    def to_dict(self):
        """Convert to dictionary format"""
        return {
            "id": self.id,
            "user_id": self.user_id,
            "module_id": self.module_id,
            "status": self.status.value,
            "progress_percent": self.progress_percent,
            "assessment_score": self.assessment_score,
            "date_completed": self.date_completed.isoformat() if self.date_completed else None,
            "created_at": self.created_at.isoformat(),
            "updated_at": self.updated_at.isoformat()
        }

class AchievementModel(Base):
    """
    SQLAlchemy model for user achievements
    """
    __tablename__ = "achievements"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, nullable=False)
    name = Column(String, nullable=False)
    description = Column(Text)
    date_earned = Column(DateTime, default=datetime.utcnow)
    created_at = Column(DateTime, default=datetime.utcnow)

    def to_dict(self):
        """Convert to dictionary format"""
        return {
            "id": self.id,
            "user_id": self.user_id,
            "name": self.name,
            "description": self.description,
            "date_earned": self.date_earned.isoformat(),
            "created_at": self.created_at.isoformat()
        }

class AssessmentScoreModel(Base):
    """
    SQLAlchemy model for assessment scores
    """
    __tablename__ = "assessment_scores"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, nullable=False)
    module_id = Column(String, nullable=False)
    score = Column(Float, nullable=False)  # 0.0 to 100.0
    date_taken = Column(DateTime, default=datetime.utcnow)
    created_at = Column(DateTime, default=datetime.utcnow)

    def to_dict(self):
        """Convert to dictionary format"""
        return {
            "id": self.id,
            "user_id": self.user_id,
            "module_id": self.module_id,
            "score": self.score,
            "date_taken": self.date_taken.isoformat(),
            "created_at": self.created_at.isoformat()
        }

class ProgressTracker:
    """
    Core logic for tracking user progress
    """

    def __init__(self, db: ProgressTrackingDB):
        self.db = db

    def record_progress(self, user_id: str, module_id: str, progress_percent: float, status: ModuleStatus) -> Dict[str, Any]:
        """
        Record user progress for a specific module
        """
        db = self.db.SessionLocal()
        try:
            # Check if progress record already exists
            existing_progress = db.query(ProgressModel).filter(
                ProgressModel.user_id == user_id,
                ProgressModel.module_id == module_id
            ).first()

            if existing_progress:
                # Update existing record
                existing_progress.progress_percent = progress_percent
                existing_progress.status = status
                existing_progress.updated_at = datetime.utcnow()

                if status == ModuleStatus.COMPLETED and not existing_progress.date_completed:
                    existing_progress.date_completed = datetime.utcnow()
            else:
                # Create new progress record
                progress_record = ProgressModel(
                    user_id=user_id,
                    module_id=module_id,
                    progress_percent=progress_percent,
                    status=status
                )

                if status == ModuleStatus.COMPLETED:
                    progress_record.date_completed = datetime.utcnow()

                db.add(progress_record)

            db.commit()

            # Return the updated record
            record = existing_progress if existing_progress else progress_record
            return record.to_dict()
        except Exception as e:
            db.rollback()
            raise e
        finally:
            db.close()

    def get_user_progress(self, user_id: str) -> Dict[str, Any]:
        """
        Get complete progress information for a user
        """
        db = self.db.SessionLocal()
        try:
            # Get all progress records for user
            progress_records = db.query(ProgressModel).filter(
                ProgressModel.user_id == user_id
            ).all()

            # Get all achievements for user
            achievements = db.query(AchievementModel).filter(
                AchievementModel.user_id == user_id
            ).all()

            # Get all assessment scores for user
            assessment_scores = db.query(AssessmentScoreModel).filter(
                AssessmentScoreModel.user_id == user_id
            ).all()

            # Calculate completion statistics
            modules_completed = [r.module_id for r in progress_records if r.status == ModuleStatus.COMPLETED]
            current_module = None
            for record in progress_records:
                if record.status in [ModuleStatus.IN_PROGRESS, ModuleStatus.ASSESSED]:
                    current_module = record.module_id
                    break

            # Calculate overall progress percentage
            total_modules = len(progress_records)
            if total_modules > 0:
                progress_percent = sum([r.progress_percent for r in progress_records]) / total_modules
            else:
                progress_percent = 0.0

            # Format results
            result = {
                "userId": user_id,
                "modulesCompleted": modules_completed,
                "currentModule": current_module,
                "progressPercent": round(progress_percent, 2),
                "achievements": [a.to_dict() for a in achievements],
                "assessmentScores": [as_.to_dict() for as_ in assessment_scores],
                "progressDetails": [r.to_dict() for r in progress_records]
            }

            return result
        finally:
            db.close()

    def award_achievement(self, user_id: str, name: str, description: str = "") -> Dict[str, Any]:
        """
        Award an achievement to a user
        """
        db = self.db.SessionLocal()
        try:
            achievement = AchievementModel(
                user_id=user_id,
                name=name,
                description=description
            )

            db.add(achievement)
            db.commit()
            db.refresh(achievement)

            return achievement.to_dict()
        finally:
            db.close()

    def record_assessment_score(self, user_id: str, module_id: str, score: float) -> Dict[str, Any]:
        """
        Record an assessment score for a user
        """
        db = self.db.SessionLocal()
        try:
            # Validate score range
            if not 0.0 <= score <= 100.0:
                raise ValueError("Score must be between 0.0 and 100.0")

            assessment_score = AssessmentScoreModel(
                user_id=user_id,
                module_id=module_id,
                score=score
            )

            db.add(assessment_score)
            db.commit()
            db.refresh(assessment_score)

            return assessment_score.to_dict()
        finally:
            db.close()

    def calculate_learning_analytics(self, user_id: str) -> Dict[str, Any]:
        """
        Calculate learning analytics for a user
        """
        db = self.db.SessionLocal()
        try:
            # Get all progress records
            progress_records = db.query(ProgressModel).filter(
                ProgressModel.user_id == user_id
            ).all()

            # Calculate statistics
            modules_started = len([r for r in progress_records])
            modules_completed = len([r for r in progress_records if r.status == ModuleStatus.COMPLETED])
            modules_assessed = len([r for r in progress_records if r.status == ModuleStatus.ASSESSED])

            # Get assessment scores
            scores = db.query(AssessmentScoreModel).filter(
                AssessmentScoreModel.user_id == user_id
            ).all()

            avg_assessment_score = sum([s.score for s in scores]) / len(scores) if scores else 0.0
            highest_score = max([s.score for s in scores]) if scores else 0.0
            lowest_score = min([s.score for s in scores]) if scores else 0.0

            # Get achievements
            achievements = db.query(AchievementModel).filter(
                AchievementModel.user_id == user_id
            ).all()

            # Calculate progress velocity (how fast user is completing modules)
            completion_times = []
            for record in progress_records:
                if record.date_completed:
                    # Calculate time from start to completion
                    # (This is a simplified version - in reality you'd track start time too)
                    completion_times.append(1)  # Placeholder for actual time calculation

            avg_completion_time = sum(completion_times) / len(completion_times) if completion_times else None

            return {
                "userId": user_id,
                "analytics": {
                    "modulesStarted": modules_started,
                    "modulesCompleted": modules_completed,
                    "modulesAssessed": modules_assessed,
                    "completionRate": modules_started and modules_completed / modules_started * 100 or 0,
                    "avgAssessmentScore": round(avg_assessment_score, 2),
                    "highestScore": highest_score,
                    "lowestScore": lowest_score,
                    "totalAchievements": len(achievements),
                    "avgCompletionTime": avg_completion_time
                }
            }
        finally:
            db.close()


# Example usage and testing
if __name__ == "__main__":
    # Initialize the progress tracking system
    progress_db = ProgressTrackingDB()
    tracker = ProgressTracker(progress_db)

    # Example: Record progress for a user
    user_id = "user_123"
    module_id = "mod_ros2_001"

    # Record initial progress
    progress1 = tracker.record_progress(
        user_id=user_id,
        module_id=module_id,
        progress_percent=25.0,
        status=ModuleStatus.IN_PROGRESS
    )
    print("Recorded initial progress:", progress1)

    # Update progress
    progress2 = tracker.record_progress(
        user_id=user_id,
        module_id=module_id,
        progress_percent=75.0,
        status=ModuleStatus.IN_PROGRESS
    )
    print("Updated progress:", progress2)

    # Complete the module
    progress3 = tracker.record_progress(
        user_id=user_id,
        module_id=module_id,
        progress_percent=100.0,
        status=ModuleStatus.COMPLETED
    )
    print("Completed module:", progress3)

    # Record an assessment score
    assessment = tracker.record_assessment_score(
        user_id=user_id,
        module_id=module_id,
        score=85.5
    )
    print("Recorded assessment score:", assessment)

    # Award an achievement
    achievement = tracker.award_achievement(
        user_id=user_id,
        name="First Module Complete",
        description="Successfully completed the first ROS 2 module"
    )
    print("Awarded achievement:", achievement)

    # Get user progress summary
    user_progress = tracker.get_user_progress(user_id)
    print("User progress summary:", user_progress)

    # Get learning analytics
    analytics = tracker.calculate_learning_analytics(user_id)
    print("Learning analytics:", analytics)
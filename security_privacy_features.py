"""
Security and Privacy Features for Robotics Curriculum Platform

This module implements security and privacy features for user data protection
including encryption, data anonymization, access controls, and privacy compliance.
"""

import hashlib
import secrets
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any
from enum import Enum
import re
from dataclasses import dataclass, field
from dataclasses import asdict
import bcrypt
from cryptography.fernet import Fernet
from pydantic import BaseModel


class PrivacyLevel(Enum):
    """Enumeration of privacy levels for user data"""
    PUBLIC = "public"
    SHARED = "shared"  # Visible to instructors/admins
    PRIVATE = "private"  # Only visible to user
    ANONYMIZED = "anonymized"  # Used for analytics without PII


class SecurityRole(Enum):
    """Enumeration of security roles"""
    ADMIN = "admin"
    INSTRUCTOR = "instructor"
    STUDENT = "student"
    GUEST = "guest"


@dataclass
class PrivacyControl:
    """Data class for privacy control settings"""
    user_id: str
    profile_visibility: PrivacyLevel
    progress_sharing: bool  # Allow progress to be shared with instructors
    data_usage_consent: bool  # Consent to use data for improvement
    marketing_consent: bool  # Consent for marketing communications
    created_at: datetime
    updated_at: datetime

    def __post_init__(self):
        if self.profile_visibility is None:
            self.profile_visibility = PrivacyLevel.PRIVATE


class UserDataEncryption:
    """Handles encryption and decryption of sensitive user data"""

    def __init__(self, key: Optional[bytes] = None):
        """
        Initialize the encryption handler

        Args:
            key: Encryption key (if None, generates a new one)
        """
        if key:
            self.key = key
        else:
            # In production, this key should be securely stored and retrieved
            self.key = Fernet.generate_key()

        self.cipher_suite = Fernet(self.key)

    def encrypt_data(self, data: str) -> str:
        """
        Encrypt sensitive data

        Args:
            data: Data string to encrypt

        Returns:
            Encrypted data as URL-safe string
        """
        encrypted_bytes = self.cipher_suite.encrypt(data.encode('utf-8'))
        return encrypted_bytes.decode('utf-8')

    def decrypt_data(self, encrypted_data: str) -> str:
        """
        Decrypt sensitive data

        Args:
            encrypted_data: Encrypted data string

        Returns:
            Decrypted data string
        """
        decrypted_bytes = self.cipher_suite.decrypt(encrypted_data.encode('utf-8'))
        return decrypted_bytes.decode('utf-8')


class AccessControlManager:
    """Manages access control and authorization for the platform"""

    def __init__(self):
        """Initialize access control manager"""
        self.access_rules = {
            "admin": {
                "can_view_any_profile": True,
                "can_edit_any_content": True,
                "can_manage_users": True,
                "can_access_all_data": True
            },
            "instructor": {
                "can_view_any_profile": False,
                "can_edit_any_content": False,
                "can_manage_users": False,
                "can_access_all_data": False,
                "can_view_student_progress": True,
                "can_create_modules": True
            },
            "student": {
                "can_view_any_profile": False,
                "can_edit_any_content": False,
                "can_manage_users": False,
                "can_access_all_data": False,
                "can_view_own_profile": True,
                "can_update_own_profile": True,
                "can_view_own_progress": True
            },
            "guest": {
                "can_view_any_profile": False,
                "can_edit_any_content": False,
                "can_manage_users": False,
                "can_access_all_data": False,
                "can_view_public_modules": True
            }
        }

    def check_permission(self, user_role: SecurityRole, permission: str) -> bool:
        """
        Check if user has a specific permission

        Args:
            user_role: Role of the user
            permission: Permission to check

        Returns:
            True if user has the permission, False otherwise
        """
        role_permissions = self.access_rules.get(user_role.value, {})
        return role_permissions.get(permission, False)

    def get_accessible_resources(self, user_id: str, user_role: SecurityRole) -> Dict[str, List[str]]:
        """
        Get resources accessible to the user based on role

        Args:
            user_id: ID of the user
            user_role: Role of the user

        Returns:
            Dictionary of accessible resources by category
        """
        resources = {
            "own": ["profile", "progress", "settings"],
            "public": ["modules", "resources", "community_content"]
        }

        if user_role == SecurityRole.ADMIN:
            resources["all"] = ["all_users", "all_content", "system_settings"]
        elif user_role == SecurityRole.INSTRUCTOR:
            resources["students"] = ["student_progress", "course_content"]
            resources["own_content"] = ["created_modules", "created_exercises"]
        elif user_role == SecurityRole.STUDENT:
            resources["enrolled"] = ["enrolled_modules", "assignments"]

        return resources


class DataAnonymizer:
    """Provides functionality to anonymize data for analytics and research"""

    @staticmethod
    def anonymize_email(email: str) -> str:
        """
        Anonymize an email address by replacing personal information

        Args:
            email: Email address to anonymize

        Returns:
            Anonymized email address
        """
        if "@" in email:
            local_part, domain = email.split("@", 1)
            if len(local_part) > 2:
                anonymized_local = f"{local_part[0]}{'*' * (len(local_part)-2)}{local_part[-1]}"
            else:
                anonymized_local = "*" * len(local_part)
            return f"{anonymized_local}@{domain}"
        return email  # Return as is if not a valid email format

    @staticmethod
    def anonymize_name(name: str) -> str:
        """
        Anonymize a name by replacing with initials and asterisks

        Args:
            name: Name to anonymize

        Returns:
            Anonymized name
        """
        if not name:
            return name
        if ' ' in name:
            parts = name.split(' ')
            first_part = parts[0]
            last_part = parts[-1] if len(parts) > 1 else ''
            if len(first_part) > 1:
                first_initial = f"{first_part[0]}{'*' * (len(first_part)-1)}"
            else:
                first_initial = "*"
            if len(last_part) > 1:
                last_initial = f"{last_part[0]}{'*' * (len(last_part)-1)}"
            else:
                last_initial = "*"
            return f"{first_initial} {last_initial}"
        else:
            # Single name
            if len(name) > 1:
                return f"{name[0]}{'*' * (len(name)-1)}"
            else:
                return "*"

    @staticmethod
    def anonymize_ip(ip_address: str) -> str:
        """
        Anonymize an IP address by zeroing out the last octet(s)

        Args:
            ip_address: IP address to anonymize

        Returns:
            Anonymized IP address
        """
        if ":" in ip_address:  # IPv6
            # For IPv6, zero out the last 64 bits
            parts = ip_address.split(":")
            if len(parts) >= 4:
                return ":".join(parts[:4]) + ":0000:0000:0000:0000"
        else:  # IPv4
            # Zero out the last octet
            parts = ip_address.split(".")
            if len(parts) == 4:
                return ".".join(parts[:-1]) + ".0"
        return ip_address


class PrivacyManager:
    """Main class to manage privacy and security features"""

    def __init__(self):
        """Initialize the privacy manager"""
        self.encryption_handler = UserDataEncryption()
        self.access_controller = AccessControlManager()
        self.data_anonymizer = DataAnonymizer()
        self.privacy_controls = {}  # user_id -> PrivacyControl mapping
        self.audit_log = []

    def generate_salt(self) -> str:
        """Generate a random salt for password hashing"""
        return secrets.token_hex(32)

    def hash_password(self, password: str, salt: Optional[str] = None) -> tuple:
        """
        Hash a password using bcrypt with optional salt.
        Addresses bcrypt 72-byte password length limitation by truncating if necessary.

        Args:
            password: Plain text password
            salt: Salt for hashing (generated if not provided)

        Returns:
            Tuple of (hashed_password, salt)
        """
        if salt is None:
            salt = self.generate_salt()

        # Combine password and salt before hashing
        salted_password = password + salt

        # Truncate to 72 bytes due to bcrypt limitation
        # This is a security consideration since bcrypt has a 72-byte password length limit
        if len(salted_password) > 72:
            salted_password = salted_password[:72]

        hashed = bcrypt.hashpw(salted_password.encode('utf-8'), bcrypt.gensalt())

        return hashed.decode('utf-8'), salt

    def verify_password(self, password: str, hashed_password: str, salt: str) -> bool:
        """
        Verify a password against its hash

        Args:
            password: Plain text password to verify
            hashed_password: Stored hashed password
            salt: Salt used in hashing

        Returns:
            True if password matches hash, False otherwise
        """
        salted_password = password + salt
        # Truncate to 72 bytes due to bcrypt limitation
        if len(salted_password) > 72:
            salted_password = salted_password[:72]
        return bcrypt.checkpw(salted_password.encode('utf-8'), hashed_password.encode('utf-8'))

    def encrypt_sensitive_field(self, field_value: str) -> str:
        """
        Encrypt a sensitive field value

        Args:
            field_value: Value to encrypt

        Returns:
            Encrypted value
        """
        return self.encryption_handler.encrypt_data(field_value)

    def decrypt_sensitive_field(self, encrypted_value: str) -> str:
        """
        Decrypt an encrypted field value

        Args:
            encrypted_value: Encrypted value to decrypt

        Returns:
            Decrypted value
        """
        return self.encryption_handler.decrypt_data(encrypted_value)

    def setup_user_privacy_controls(self, user_id: str, initial_settings: Optional[dict] = None) -> PrivacyControl:
        """
        Set up privacy controls for a new user

        Args:
            user_id: ID of the user
            initial_settings: Initial privacy settings (optional)

        Returns:
            PrivacyControl object with the user's privacy settings
        """
        if initial_settings is None:
            initial_settings = {}

        privacy_control = PrivacyControl(
            user_id=user_id,
            profile_visibility=initial_settings.get("profile_visibility", PrivacyLevel.PRIVATE),
            progress_sharing=initial_settings.get("progress_sharing", False),
            data_usage_consent=initial_settings.get("data_usage_consent", False),
            marketing_consent=initial_settings.get("marketing_consent", False),
            created_at=datetime.now(),
            updated_at=datetime.now()
        )

        self.privacy_controls[user_id] = privacy_control
        self.audit_log.append({
            "action": "privacy_setup",
            "user_id": user_id,
            "timestamp": datetime.now(),
            "details": f"Privacy controls initialized for user {user_id}"
        })

        return privacy_control

    def update_privacy_controls(self, user_id: str, **updates) -> Optional[PrivacyControl]:
        """
        Update privacy controls for a user

        Args:
            user_id: ID of the user
            **updates: Updates to privacy settings

        Returns:
            Updated PrivacyControl object or None if user not found
        """
        if user_id not in self.privacy_controls:
            return None

        privacy_control = self.privacy_controls[user_id]

        # Update fields based on provided data
        for field, value in updates.items():
            if hasattr(privacy_control, field):
                setattr(privacy_control, field, value)

        privacy_control.updated_at = datetime.now()

        self.audit_log.append({
            "action": "privacy_update",
            "user_id": user_id,
            "timestamp": datetime.now(),
            "details": f"Privacy controls updated for user {user_id}"
        })

        return privacy_control

    def get_allowed_data_access(self, requesting_user_id: str, requesting_role: SecurityRole,
                                target_user_id: str) -> Dict[str, bool]:
        """
        Check what data the requesting user can access from the target user

        Args:
            requesting_user_id: ID of the user requesting access
            requesting_role: Role of the requesting user
            target_user_id: ID of the target user whose data access is checked

        Returns:
            Dictionary indicating which data fields are accessible
        """
        if target_user_id not in self.privacy_controls:
            return {}

        target_privacy = self.privacy_controls[target_user_id]

        # Default access rights
        access_rights = {
            "profile": False,
            "name": False,
            "email": False,
            "progress": False,
            "assessment_scores": False
        }

        # Admins can access all data
        if requesting_role == SecurityRole.ADMIN:
            access_rights = {
                "profile": True,
                "name": True,
                "email": True,
                "progress": True,
                "assessment_scores": True
            }
        elif requesting_role == SecurityRole.INSTRUCTOR:
            # Instructors can see student progress if privacy permits
            access_rights = {
                "name": True,  # Always allow name access for instructors
                "progress": target_privacy.progress_sharing,
                "assessment_scores": target_privacy.progress_sharing
            }
        elif requesting_user_id == target_user_id:
            # Users can always access their own data
            access_rights = {
                "profile": True,
                "name": True,
                "email": True,
                "progress": True,
                "assessment_scores": True
            }
        else:
            # For other users, check profile visibility
            if target_privacy.profile_visibility == PrivacyLevel.PUBLIC:
                access_rights["name"] = True
            elif target_privacy.profile_visibility == PrivacyLevel.SHARED and requesting_role == SecurityRole.INSTRUCTOR:
                access_rights["name"] = True

        return access_rights

    def anonymize_user_data_for_analytics(self, user_data: Dict) -> Dict:
        """
        Anonymize user data for analytics while preserving non-PII information

        Args:
            user_data: Dictionary containing user data

        Returns:
            Dictionary with anonymized user data suitable for analytics
        """
        anonymized_data = user_data.copy()

        # Anonymize personal identifying information
        if "name" in anonymized_data:
            anonymized_data["name"] = self.data_anonymizer.anonymize_name(anonymized_data["name"])

        if "email" in anonymized_data:
            anonymized_data["email"] = self.data_anonymizer.anonymize_email(anonymized_data["email"])

        if "ip_address" in anonymized_data:
            anonymized_data["ip_address"] = self.data_anonymizer.anonymize_ip(anonymized_data["ip_address"])

        # Remove sensitive fields entirely
        sensitive_fields = ["password", "phone", "address", "payment_info"]
        for field in sensitive_fields:
            if field in anonymized_data:
                del anonymized_data[field]

        # Add anonymization marker for audit purposes
        anonymized_data["anonymized_for"] = "analytics"
        anonymized_data["anonymized_at"] = datetime.now()

        return anonymized_data


# Example usage and demonstration
if __name__ == "__main__":
    # Initialize the privacy manager
    privacy_manager = PrivacyManager()

    print("Privacy and Security Manager initialized!")

    # Example: Create a user with privacy controls
    user_id = "user_123"
    privacy_controls = privacy_manager.setup_user_privacy_controls(
        user_id,
        {
            "profile_visibility": PrivacyLevel.SHARED,
            "progress_sharing": True,
            "data_usage_consent": True
        }
    )

    print(f"Privacy controls set up for {user_id}:")
    print(f"  Profile visibility: {privacy_controls.profile_visibility}")
    print(f"  Progress sharing: {privacy_controls.progress_sharing}")
    print(f"  Data usage consent: {privacy_controls.data_usage_consent}")

    # Example: Hash a password
    password = "my_secure_password"
    hashed_password, salt = privacy_manager.hash_password(password)
    print(f"\nPassword hashing example:")
    print(f"  Original: {password}")
    print(f"  Hashed: {hashed_password[:30]}...")  # Showing first 30 chars only
    print(f"  Salt: {salt[:10]}...")  # Showing first 10 chars only

    # Example: Verify password
    is_valid = privacy_manager.verify_password(password, hashed_password, salt)
    print(f"  Password verification: {is_valid}")

    # Example: Encrypt sensitive data
    sensitive_info = "Secret user information"
    encrypted_info = privacy_manager.encrypt_sensitive_field(sensitive_info)
    print(f"\nEncryption example:")
    print(f"  Original: {sensitive_info}")
    print(f"  Encrypted: {encrypted_info[:30]}...")  # Showing first 30 chars only

    # Example: Decrypt sensitive data
    decrypted_info = privacy_manager.decrypt_sensitive_field(encrypted_info)
    print(f"  Decrypted: {decrypted_info}")

    # Example: Access control checks
    admin_role = SecurityRole.ADMIN
    print(f"\nAccess control examples:")
    print(f"  Admin can view any profile: {privacy_manager.access_controller.check_permission(admin_role, 'can_view_any_profile')}")
    print(f"  Admin can edit any content: {privacy_manager.access_controller.check_permission(admin_role, 'can_edit_any_content')}")

    # Example: Anonymize data for analytics
    user_info = {
        "name": "John Doe",
        "email": "john.doe@example.com",
        "ip_address": "192.168.1.100",
        "course_progress": 75.5,
        "assessment_score": 85.0,
        "password": "secure_password"
    }

    anonymized_info = privacy_manager.anonymize_user_data_for_analytics(user_info)
    print(f"\nAnonymization example:")
    print(f"  Original name: {user_info['name']}")
    print(f"  Anonymized name: {anonymized_info['name']}")
    print(f"  Original email: {user_info['email']}")
    print(f"  Anonymized email: {anonymized_info['email']}")
    print(f"  Course progress preserved: 'course_progress' in anonymized data: {'course_progress' in anonymized_info}")

    # Example: Data access based on privacy settings
    instructor_id = "instr_456"
    instructor_role = SecurityRole.INSTRUCTOR
    access_rights = privacy_manager.get_allowed_data_access(
        instructor_id, instructor_role, user_id
    )

    print(f"\nData access example (Instructor accessing student data):")
    for data_type, can_access in access_rights.items():
        print(f"  Can access {data_type}: {can_access}")

    print("\nSecurity and Privacy features implemented successfully!")
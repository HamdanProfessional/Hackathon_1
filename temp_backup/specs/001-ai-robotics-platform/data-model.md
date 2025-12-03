# Data Model

**Date**: 2025-11-30
**Status**: Draft

This document defines the primary data entities for the platform, as identified in the feature specification.

## 1. User
Represents a student on the platform. This entity is managed by the `/auth` service and stored in Neon Postgres.

**Attributes**:
- `user_id` (UUID, Primary Key): Unique identifier for the user.
- `email` (String, Unique): User's email address, used for login.
- `password_hash` (String): Hashed password for the user.
- `hardware_bg` (Enum): The hardware background provided at signup.
  - Possible values: `RTX4090`, `Jetson`, `Laptop`, `Cloud`
- `software_bg` (String): A free-text description of the user's software background.
- `created_at` (Timestamp): Timestamp of account creation.
- `updated_at` (Timestamp): Timestamp of the last profile update.

## 2. SyllabusModule
Represents a top-level module from the `course_syllabus.md` file. This is a logical entity derived from the content structure, not a database model.

**Attributes**:
- `module_id` (String, Slug): A slugified version of the module title (e.g., `ros-2-nervous-system`).
- `title` (String): The full title of the module (e.g., "ROS 2 (Nervous System)").
- `chapters` (List<Chapter>): An ordered list of chapters belonging to this module.

## 3. Chapter
Represents a specific chapter or document within a module. This is a logical entity derived from the content files.

**Attributes**:
- `chapter_id` (String, Slug): A slugified version of the chapter title.
- `title` (String): The full title of the chapter.
- `source_path` (String): The file path to the source Markdown file (e.g., `/docs/en/ros-2/introduction.md`).
- `content` (String): The raw Markdown content of the chapter.

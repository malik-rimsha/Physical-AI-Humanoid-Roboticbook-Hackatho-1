# Data Model: Module 1 – The Robotic Nervous System (ROS 2)

**Feature**: 1-ros2-robotic-nervous-system
**Date**: 2025-12-17

## Entity Definitions

### Educational Content
- **Type**: Documentation entity
- **Fields**:
  - moduleId: string (unique identifier for the module)
  - title: string (module title)
  - description: string (brief description of the module)
  - chapters: array of Chapter objects
  - learningObjectives: array of string (what students will learn)
  - prerequisites: array of string (required knowledge)
  - estimatedDuration: number (in minutes)
- **Relationships**: Contains multiple Chapter entities
- **Validation**: Title and description are required; chapters must have at least one entry

### Chapter
- **Type**: Content entity
- **Fields**:
  - chapterId: string (unique identifier for the chapter)
  - title: string (chapter title)
  - content: string (chapter content in markdown format)
  - objectives: array of string (what students will learn in this chapter)
  - exercises: array of Exercise objects
  - nextChapterId: string (reference to the next chapter)
  - prevChapterId: string (reference to the previous chapter)
- **Relationships**: Belongs to one Educational Content entity; contains multiple Exercise entities
- **Validation**: Title and content are required; objectives must be provided

### Exercise
- **Type**: Assessment entity
- **Fields**:
  - exerciseId: string (unique identifier for the exercise)
  - title: string (exercise title)
  - description: string (exercise instructions)
  - type: string (e.g., "multiple-choice", "coding", "conceptual")
  - difficulty: string (e.g., "beginner", "intermediate", "advanced")
  - solution: string (expected solution or answer)
  - hints: array of string (helpful hints for students)
- **Relationships**: Belongs to one Chapter entity
- **Validation**: Title, description, and solution are required

### Student Progress
- **Type**: Tracking entity
- **Fields**:
  - studentId: string (unique identifier for the student)
  - moduleId: string (reference to the module)
  - chapterProgress: array of ChapterProgress objects
  - completionDate: date (when the module was completed)
  - overallScore: number (percentage score across all chapters)
- **Relationships**: Tracks progress for one Educational Content entity; contains multiple ChapterProgress entities
- **Validation**: studentId and moduleId are required; overallScore must be between 0 and 100

### ChapterProgress
- **Type**: Tracking entity
- **Fields**:
  - chapterId: string (reference to the chapter)
  - completed: boolean (whether the chapter is completed)
  - score: number (percentage score for the chapter)
  - completionDate: date (when the chapter was completed)
  - timeSpent: number (time spent in minutes)
- **Relationships**: Belongs to one Student Progress entity; references one Chapter entity
- **Validation**: chapterId is required; score must be between 0 and 100

## State Transitions

### Chapter States
- `not-started` → `in-progress` (when student begins the chapter)
- `in-progress` → `completed` (when student completes all exercises and assessments)
- `completed` → `reviewed` (when student reviews the content after completion)

### Module States
- `locked` → `available` (when prerequisites are met)
- `available` → `in-progress` (when student starts the first chapter)
- `in-progress` → `completed` (when all chapters are completed)
- `completed` → `certified` (when student achieves required score)

## Data Relationships

```
Educational Content (1) ─── (Many) Chapter (1) ─── (Many) Exercise
      │
      │ (Many) Student Progress (1) ─── (Many) ChapterProgress
```

## Validation Rules

1. **Module-level validation**:
   - All chapters must be completed before module completion
   - Learning objectives must align with chapter content
   - Prerequisites must be validated before module access

2. **Chapter-level validation**:
   - Each chapter must build on previous concepts
   - Exercises must align with chapter objectives
   - Content must be appropriate for target audience

3. **Progress tracking validation**:
   - Students cannot skip chapters in sequence
   - Scores must be calculated correctly from exercise results
   - Progress data must be preserved across sessions
---
id: 1
title: update-project-constitution
stage: constitution
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: smc
command: /sp.constitution
labels: [constitution, setup, governance]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - no tests required for constitution update
---

## Prompt

/sp.constitution

Project: AI-Spec-Driven Book with Embedded RAG Chatbot

Purpose:
Write and deploy a technical book using Docusaurus and Spec-Kit Plus, and embed a RAG chatbot that answers questions strictly from the book's content.

Core Principles:
- Spec-driven execution
- Accuracy and no hallucinations
- Developer-focused clarity
- Reproducibility and modular design

Standards:

Book:
- Framework: Docusaurus
- Deployment: GitHub Pages
- Authored using Claude Code
- Clear structure, correct code examples

Chatbot:
- Architecture: RAG
- SDKs: OpenAI Agents / ChatKit
- Backend: FastAPI
- DB: Neon Serverless Postgres
- Vector DB: Qdrant Cloud (Free Tier)
- Must answer:
  - Full-book questions
  - Selected-text-only questions

RAG Rules:
- Embed only book content
- Retrieval required before generation
- Responses grounded strictly in retrieved text

Constraints:
- Free-tier infrastructure only
- Chatbot embedded in book UI
- No external or fabricated content

Success Criteria:
- Book live on GitHub Pages
- Chatbot answers accurately from book
- Selected-text mode functions correctly
- Project reproducible from repo

## Response snapshot

The constitution file has been successfully updated for the AI-Spec-Driven Book with Embedded RAG Chatbot project. Here's the summary:

**New Version**: 1.0.0 (initial version)
**Bump Rationale**: This is the initial constitution for the project, establishing core principles for the AI-Spec-Driven Book with Embedded RAG Chatbot project.

**Files Updated**:
- `.specify/memory/constitution.md` - Complete rewrite with project-specific principles and standards

**Constitution Overview**:
The new constitution establishes 5 core principles:
1. Spec-Driven Execution - All development follows defined specs with traceability
2. Accuracy and No Hallucinations - RAG chatbot responses must be grounded in book content
3. Developer-Focused Clarity - Clear documentation and correct code examples
4. Reproducibility and Modular Design - Reproducible infrastructure with clear interfaces
5. RAG-First Architecture - Mandatory retrieval before generation for chatbot responses

**Technology Standards**:
- Book: Docusaurus framework, GitHub Pages deployment
- Chatbot: FastAPI backend, Neon Postgres, Qdrant vector DB
- Infrastructure: Free-tier only, embedded in book UI

**Quality Gates**:
- No hallucinations in responses
- Selected-text mode functionality
- Project reproducibility from repo

**Files Flagged for Manual Follow-up**:
- `.specify/templates/plan-template.md` - Constitution Check section needs alignment
- `.specify/templates/spec-template.md` - May need alignment with new principles
- `.specify/templates/tasks-template.md` - May need alignment with new requirements

**Suggested Commit Message**: `docs: add constitution for AI-Spec-Driven Book with RAG Chatbot project`

The constitution is now ready for use in the project and will guide all future development decisions according to the specified principles.

## Outcome

- ✅ Impact: New project constitution created with 5 core principles and technology standards
- 🧪 Tests: no tests required for constitution update
- 📁 Files: .specify/memory/constitution.md updated with project-specific principles
- 🔁 Next prompts: Project constitution is established, ready for spec development
- 🧠 Reflection: Constitution properly captures all user requirements and constraints

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification work
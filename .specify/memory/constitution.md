<!-- SYNC IMPACT REPORT
Version change: N/A -> 1.0.0
Modified principles: N/A (new constitution)
Added sections: All sections (new constitution)
Removed sections: N/A
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->
# AI-Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-Driven Execution
Every feature and implementation starts with a clear specification; All development follows the defined spec with traceability; Requirements must be explicit before implementation begins.

### II. Accuracy and No Hallucinations
All responses from the RAG chatbot must be grounded strictly in retrieved text; No fabricated or external content allowed; Responses must cite specific sources from the book content.

### III. Developer-Focused Clarity
Code examples must be correct and testable; Documentation should be clear and comprehensive; Technical content must be accessible to the target audience.

### IV. Reproducibility and Modular Design
All infrastructure components must be reproducible via code; Modular architecture to enable independent development and testing; Clear interfaces between components.

### V. RAG-First Architecture
All chatbot responses require retrieval from book content before generation; Vector database must contain only book content embeddings; Retrieval-augmented generation is mandatory, not optional.

## Technology Standards

### Book Platform
- Framework: Docusaurus for static site generation
- Deployment: GitHub Pages for hosting
- Authoring: Using Claude Code for content creation
- Structure: Clear organization with correct code examples

### Chatbot Architecture
- Backend: FastAPI for serving
- Database: Neon Serverless Postgres for metadata
- Vector Database: Qdrant Cloud (Free Tier) for embeddings
- SDKs: OpenAI Agents / ChatKit for RAG functionality
- Embedding: Book content only, no external sources

### Infrastructure Constraints
- Free-tier infrastructure only to maintain cost efficiency
- Chatbot must be embedded directly in book UI
- No external content integration beyond book content
- Strict adherence to RAG rules: retrieval required before generation

## Development Workflow

### Implementation Requirements
- Spec-first approach: All features must be specified before implementation
- Code review: All changes must pass review focusing on spec compliance
- Testing: Unit and integration tests for all components
- Validation: RAG responses must be validated against book content

### Quality Gates
- No hallucinations: All chatbot responses must be grounded in book content
- Performance: Response times must meet user expectations
- Accuracy: Content retrieval must be precise and relevant
- Security: No exposure of sensitive data or unauthorized access
- Chatbot answers accurately from book content
- Selected-text mode functions correctly
- Project reproducible from repo

## Governance

All development must adhere to the spec-driven approach; Changes to this constitution require explicit documentation and approval; Code reviews must verify compliance with all principles; Infrastructure changes must maintain free-tier constraint; RAG implementation must strictly follow retrieval-before-generation rules.

**Version**: 1.0.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-17

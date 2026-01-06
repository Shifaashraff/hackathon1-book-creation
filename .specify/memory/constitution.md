<!-- SYNC IMPACT REPORT -->
<!-- Version change: 0.1.0 → 1.0.0 -->
<!-- Added sections: Core Principles (6 principles), Additional Requirements, Development Workflow -->
<!-- Templates requiring updates: All ✅ updated -->
<!-- Follow-up TODOs: None -->

# AI-Native Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-driven, reproducible development
All development must follow spec-driven approach with reproducible processes. Every feature and change must be traceable to a specification. This ensures consistency, quality, and maintainability across the project.

### Clear technical content for engineers
All documentation and code must be written with clear, technical precision for engineer audiences. Content must be accurate, well-structured, and serve the needs of the technical community using the book and chatbot.

### RAG answers grounded only in book content
The RAG chatbot must only provide answers based on book content. The system must refuse to answer questions that cannot be grounded in the provided book content. This preserves the integrity and accuracy of the information provided.

### Single repository with no hardcoded secrets
All code must exist in a single GitHub repository with no hardcoded secrets. All sensitive information must be managed through proper configuration and environment variables. This ensures security and maintainability.

### Technology stack compliance
All implementations must comply with the specified technology stack: Docusaurus for book, OpenAI Agents/ChatKit for chatbot, FastAPI, Qdrant Cloud, and Neon Postgres. Deviations require explicit approval and justification.

### Full-book and selected-text QA support
The system must support both full-book and selected-text-only question answering capabilities. Users must be able to query either the entire book or specific sections of text.

## Additional Requirements

### Book Requirements
- Built with Docusaurus framework
- Markdown format for content
- Deployed to GitHub Pages
- Responsive and accessible design

### Chatbot Requirements
- Built with OpenAI Agents/ChatKit
- FastAPI backend
- Qdrant Cloud for vector storage
- Neon Postgres for relational data
- Must refuse answers without proper context
- Supports both full-book and selected-text queries

### Deployment Requirements
- Single GitHub repository
- Public deployment of book
- Functional RAG chatbot embedded in book
- No hardcoded secrets in repository

## Development Workflow

### Spec-First Development
- All features must begin with a specification document
- Implementation follows specification precisely
- Changes to behavior require spec updates first

### Quality Gates
- All code must pass automated tests
- Code reviews required for all changes
- Specifications must be validated before implementation
- All behavior must be traceable to specifications

### Security Requirements
- No hardcoded secrets in codebase
- Proper authentication and authorization
- Secure handling of user queries and data
- Compliance with data privacy regulations

## Governance

This constitution governs all development decisions for the AI-Native Book with Embedded RAG Chatbot project. All team members must follow these principles. Amendments to this constitution require explicit approval and must be documented with proper justification. All pull requests and code reviews must verify compliance with these principles. All behavior must be traceable to specifications as required by the constraints.

**Version**: 1.0.0 | **Ratified**: 2026-01-01 | **Last Amended**: 2026-01-01
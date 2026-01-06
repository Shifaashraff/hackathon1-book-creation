---
id: 1
title: Update Constitution AI Native Book
stage: constitution
date: 2026-01-01
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: CORE COM
command: /sp.constitution
labels: [constitution, ai-book, rag-chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```
/sp.constitution

Project:
AI-Native Book with Embedded RAG Chatbot

Principles

Spec-driven, reproducible development

Clear technical content for engineers

RAG answers grounded only in book content

Requirements

Book: Docusaurus, Markdown, GitHub Pages

Chatbot: OpenAI Agents/ChatKit, FastAPI, Qdrant Cloud, Neon Postgres

Supports full-book and selected-text-only QA

Single GitHub repo, no hardcoded secrets

Constraints

All behavior traceable to specs

Chatbot must refuse answers without context

Success

Book publicly deployed

RAG chatbot embedded and functional
```

## Response snapshot

Updated the constitution file with specific principles and requirements for the AI-Native Book with Embedded RAG Chatbot project, including core principles, additional requirements, development workflow, and governance sections.

## Outcome

- ✅ Impact: Constitution file updated with project-specific principles and requirements
- 🧪 Tests: No tests required for constitution update
- 📁 Files: Updated .specify/memory/constitution.md with new content
- 🔁 Next prompts: Ready to proceed with spec, plan, and implementation
- 🧠 Reflection: Constitution now properly reflects the specific requirements of the AI-native book project with RAG chatbot

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification based on updated constitution
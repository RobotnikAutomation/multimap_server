---
name: plan-feature
agent: plan
description: Create a requirements-led, read-only feature proposal for the multimap_server package.
argument-hint: "[feature request]"
---

Plan the requested feature only for the `multimap_server` package. Do not edit files or implement the feature.

Read the package `AGENTS.md`, `README.md`, `CMakeLists.txt`, and `package.xml`, then inspect only the source, configuration, launch files, and tests relevant to the request. Confirm paths and existing behavior from the repository; do not rely on stale or untracked examples.

Produce a proposed requirements document, design document, and implementation plan using the corresponding files in `docs/feature-workflow/templates/`:

- Requirements: state observable, testable acceptance criteria, give each criterion a stable ID, and mark whether it is mandatory.
- Design: explain consequential decisions, alternatives considered, rationale, and compatibility effects.
- Plan: reference the requirements and design, map criteria to affected files, and list focused tests, documentation updates, and exact validation commands.

Return the three drafts with suggested package-relative paths under `docs/feature-workflow/<feature-slug>/`. Keep each criterion initially unverified; do not claim it is satisfied before implementation evidence exists.

Call out assumptions, missing information, absent test infrastructure, and validation that cannot yet be established. Ask before changing agreed behavior or expanding scope, especially into `multimap_server_msgs` or another package. Do not create branches, commits, or release-version changes.

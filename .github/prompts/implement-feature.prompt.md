---
name: implement-feature
agent: agent
description: Implement an approved multimap_server feature from its requirements, design, and plan.
argument-hint: "[requirements, design, and plan paths]"
---

Implement an approved feature only within `multimap_server`, using the provided requirements, design, and plan as the authority. If those approved documents or their scope are missing or conflict, stop and ask for clarification rather than inventing acceptance criteria or changing agreed behavior.

Before editing, read the package `AGENTS.md` and the referenced documents, then inspect the relevant implementation and tests. Keep changes surgical. Ask before expanding into another package, changing shared message definitions, changing approved behavior, or adding new test infrastructure or dependencies.

Implement the planned behavior with focused tests and relevant documentation. Use existing package patterns and verify every referenced path. Run the exact relevant validation commands from the plan where possible; report commands not run and why. Do not imply that a build or test passed if it was not run.

For each acceptance criterion, report its ID, status, and supporting evidence. Mark a criterion satisfied only when the evidence supports it. Mark the requirement implemented only if every mandatory criterion passes. Otherwise leave it incomplete and list outstanding mandatory criteria and verification gaps. Do not create branches, commits, or release-version changes.

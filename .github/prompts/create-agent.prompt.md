---
mode: agent
description: Scaffold a new repo-specific agent workflow by creating a slash command and matching prompt file
---

# Create a new Meshtastic agent workflow

Create a new agent workflow for this repository by adding:

- a Claude Code slash command at `.claude/commands/<name>.md`
- a matching Copilot prompt at `.github/prompts/<name>.prompt.md`
- an entry in `.claude/commands/README.md`

## Requirements

- Use the repository's existing command and prompt style from the other agent workflows.
- Include a short description, a clear scope, and concrete steps.
- Mention any safety constraints relevant to the task.
- Reference repo tooling such as PlatformIO, `mcp-server/run-tests.sh`, or helper scripts when appropriate.
- Keep the Claude and Copilot versions aligned.

## Deliverables

1. Create the two files.
2. Update the command index.
3. Summarise the new agent name and the paths created.

---
description: Scaffold a new repo-specific agent workflow (Claude command + Copilot prompt)
argument-hint: <agent-name> [purpose]
---

# `/create-agent` — scaffold a new Meshtastic agent workflow

Create a new agent workflow for this repository by adding a Claude Code slash command and a matching Copilot prompt, then register the command in the command index.

## What to do

1. **Confirm the agent name and purpose.** Use the requested name as the command slug, for example `foo` -> `.claude/commands/foo.md` and `.github/prompts/foo.prompt.md`.
2. **Create the Claude command file.** Add YAML frontmatter plus concise instructions describing:
   - what the agent does
   - which repo tools it should use (for example `pio run`, `./mcp-server/run-tests.sh`, or a helper script)
   - any safety rules or destructive-action cautions
   - how it should present results to the operator
3. **Create the Copilot prompt file.** Mirror the Claude command content in a Copilot-friendly prompt with matching guidance.
4. **Register the new command.** Update `.claude/commands/README.md` with a row for the new workflow.
5. **Keep the two surfaces aligned.** The intent, house rules, and repo-specific guidance should stay in sync.

## Repo-specific expectations

- Follow the Meshtastic firmware conventions in `.github/copilot-instructions.md`.
- Match the tone and structure of the existing commands in `.claude/commands/test.md` and `.claude/commands/diagnose.md`.
- Respect the house rules: avoid destructive writes without operator approval, never speculate about root cause, and keep MCP calls sequential per port.

## Output

- Create the new command and prompt files.
- Update the command index.
- Report the new command name and file paths created.

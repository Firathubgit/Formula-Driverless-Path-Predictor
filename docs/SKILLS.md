# Focused use of the twelve workflow skills

On 2026-09-14 all twelve were copied byte-identically from `~/.codex/skills/` into
`~/.claude/skills/`, so both agents now have them. The frontmatter format is shared, so
`disable-model-invocation` and `argument-hint` carried over unchanged; eight are
user-invocable only and four can also trigger on their own. Codex's own built-ins (Image
Gen, OpenAI Docs, Plugin Creator, Review Agent, Skill Creator, Skill Installer) were not
copied and are not part of these twelve.

The twelve personal skills visible in the local environment were checked on 2026-09-13. They are engineering workflows. They do not provide vehicle validation, certify robotics expertise, install the simulation architecture, or require every session to run twelve procedures. The pasted skill inventory is reference material; use the installed skill's current instructions when its actual task applies.

| Installed skill | Useful project trigger | Concrete output |
| --- | --- | --- |
| `codebase-design` | Designing the planner, plant, telemetry or adapter interface; changing where behavior can be replaced. | A small interface with clear invariants and a cohesive implementation behind it. |
| `domain-modeling` | Resolving terms such as path, trajectory, actual state, observation, grip or replay. | Shared domain vocabulary and recorded architectural decisions. |
| `grilling` | The user explicitly wants to stress-test a consequential design before implementing it. | Resolved constraints and decisions from a focused interview. |
| `grill-with-docs` | The same interview also needs durable terminology and decisions. | Interview results captured in the glossary and ADRs. |
| `prototype` | Comparing rendering approaches or answering a narrow interaction/state question cheaply. | A clearly temporary experiment and a recorded verdict. |
| `tdd` | Test-first work on speed constraints, tracking, delayed commands or replay contracts. | A failing behavioral test, a minimal passing implementation and a useful regression. |
| `improve-codebase-architecture` | An existing implementation has accumulated shallow interfaces or scattered behavior. | A visual review of specific improvement candidates before choosing a refactor. |
| `setup-matt-pocock-skills` | Establishing or changing the issue tracker, triage labels and domain-document layout. | Shared configuration that the issue and domain workflows can actually use. |
| `to-prd` | The settled conversation needs to become a product requirement document. | A requirement brief with scope and acceptance criteria, published only within authorized tracker scope. |
| `to-issues` | A settled milestone needs independently implementable work items. | Vertical slices with observable acceptance criteria and dependencies. |
| `triage` | Real issues or external PRs need reproduction, categorization and an implementable brief. | Evidence and the appropriate tracker state; no invented bugs or unnecessary outreach. |
| `handoff` | A fresh agent or later session must continue substantial unfinished work. | A compact account of decisions, files, verification, open questions and the next concrete action. |

For the starting release, use design vocabulary when shaping an interface and meaningful tests when implementing behavior. Bring in a prototype only when a specific uncertainty warrants it. Use the tracker workflows when there is actual tracker work, and interviews when user judgment is needed. Installing more skills is not a milestone.

A skill's advertised description is used to decide whether it applies; its full instructions are read when selected. This is consistent with Codex's documented progressive disclosure behavior. [Official skills documentation](https://learn.chatgpt.com/docs/build-skills).

## One shared project authority

[AGENTS.md](../AGENTS.md) holds the project invariants. [CLAUDE.md](../CLAUDE.md) imports it with `@AGENTS.md`. Keep architecture and scope in the shared files rather than maintaining competing agent-specific briefs. Codex documents repository `AGENTS.md` discovery; Claude Code documents importing that same file from `CLAUDE.md`. [Codex project instructions](https://learn.chatgpt.com/docs/agent-configuration/agents-md), [Claude Code project memory](https://code.claude.com/docs/en/memory#agentsmd).

The original working recommendation was **Codex locally for implementation, with Claude Code as an independent reviewer at milestone boundaries**, adopting the supplied master brief's workflow rather than a benchmark claim that one product always performs better. Part of its basis was that these workflow skills existed only on the Codex side; that is no longer true, and both agents have implemented and verified milestones here. Keep one agent responsible for edits at a time. Neither agent's agreement replaces a compiler, a reproducible experiment or visual inspection.

A session should read the current brief and status, implement one concrete milestone slice, run the relevant checks, and record what actually passed and what remains. At a milestone boundary, give the reviewer the diff, acceptance criteria, commands and outputs, and request independently reproducible findings. The implementation agent verifies those findings and fixes confirmed defects. Both agents use the same source files and tests.

The complete long-term context lives in `docs/reference/MASTER_PROJECT_PROMPT.txt`; [PROJECT_BRIEF.md](PROJECT_BRIEF.md) is the active scope and [STATUS.md](STATUS.md) records progress. Do not turn the long-term vision into an instruction to implement every roadmap item in one session.

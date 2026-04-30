---
description: Update Easy Trainer user manual based on recent UI/release source changes
argument-hint: "[since=<git-ref>] [files=<path,path>]"
---

Run the **manual-writer** subagent to update the Easy Trainer end-user manual
under `home-next/src/app/docs/_content/ko/`.

Arguments passed: `$ARGUMENTS`

## Routing rules for the subagent

- If `$ARGUMENTS` contains `since=<ref>`, pass that ref to the subagent so it
  diffs `<ref>..HEAD`.
- If `$ARGUMENTS` contains `files=<path1>,<path2>`, pass that list so the
  subagent treats those files as the change set without invoking git.
- If `$ARGUMENTS` is empty, the subagent diffs `HEAD~1..HEAD`.

## Invocation

Use the Agent tool with `subagent_type: manual-writer`. Hand over the
arguments verbatim — the subagent's prompt explains how to interpret them.

After the subagent finishes, verify:

1. The list of changed MDX files reported by the subagent actually has
   git changes (`git status home-next/src/app/docs/_content/ko/`).
2. `npm run lint` and `npx tsc --noEmit` both pass for `home-next/`.
3. Newly inserted `<DocsImage status="missing" />` slots match new rows in
   `home-next/public/docs/SCREENSHOTS.md`.

Do **not** commit or push. Just report the diff to the user so they can review
and commit themselves.

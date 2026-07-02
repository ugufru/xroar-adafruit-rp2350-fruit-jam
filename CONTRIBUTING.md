# Contributing to XRoar on the Adafruit Fruit Jam

This describes how anyone — human or AI assistant — contributes to this
project. Read it before starting work. The same instructions apply whoever you
are; there is no separate track for automated contributors.

## Getting oriented
New here? Read the README and roadmap, skim the open issues, then pick up the
next thing. If you can't orient from the project's own docs, that's a
documentation gap worth surfacing — not a reason to invent context.

## Source of truth
This project's own documentation is canonical. For any question within the
project's scope, the in-repo docs are the authority — consult them before
searching the web. Don't keep project knowledge in private notes or assistant
memory files; if a fact matters, it belongs in the human-readable docs, kept
accurate. When docs are stale or wrong, fix the docs (or file an issue) rather
than routing around them.

## Before you start: file an issue
Work is tracked in `issues.jsonl` at the repo root.
- No substantive work without a matching issue. If none exists, propose one.
- Get the issue reviewed before starting — fairly documented, fairly considered.
- Record deferred alternatives with a revisit trigger ("try this if X"), and
  record rejected or forbidden paths with their rationale, so settled decisions
  aren't quietly relitigated.

## Doing the work
- **Don't invent — ask or verify.** Check any claim against the source first
  (grep the repo); when something can't be verified, ask rather than assert.
- **Build conservatively.** Write the minimum that satisfies the issue; make
  surgical changes that match existing style. Don't scaffold, restyle, or
  redesign unasked, and don't apply codebase reflexes to a repo that isn't one.
- **Stay in bounds.** Work within this project's tree; if a change seems to need
  something outside it, ask first.

## For automated / AI contributors
- **Bounded autonomy.** Proceed on your own through mechanical steps; stop for
  genuine decisions, visual or physical verification, and hard-to-reverse actions.
- **Be careful with side effects.** For repeated, expensive, or outward-facing
  actions (launching apps, hitting external services), act once, observe, then
  iterate — never fire them in a loop.

## Finishing
- An issue isn't `done` until the change is tested and confirmed by a maintainer.
- Resolve related issues before pushing.
- Favor in-repo, diffable, vendor-neutral artifacts so the project stays
  comprehensible and reviewable no matter who contributed.

# Codex token burndown

A ready-to-paste prompt for running a token-burndown session in **Codex**: spend a large
token budget landing verified fixes against the open GitHub Issue backlog, following this
repo's conventions. Codex reads the same canonical guidance as Claude (`AGENTS.md` at the
repo root; `.codex/skills/` symlinks to `.agents/skills/`), so the prompt below leans on
that rather than restating it.

## Model and effort

| Setting | Choice |
| --- | --- |
| Model | `sol` |
| Reasoning effort | **extra high** (first choice) or **ultra** |

Use extra high as the default; step up to ultra if the session is stalling on the harder
issues (SLAM/drift work, anything touching `host/src/roomscan/slam/`). Lower tiers are not
worth it here — the backlog's cheap mechanical items are already fleet-worker territory.

## The prompt

Paste everything in the block below as the opening message of the Codex session.

```text
You are running a token-burndown session in the lidar-roomscanner repo: spend the
available budget landing verified fixes and improvements for open GitHub Issues,
highest impact first. Prefer many small, verified, committed landings over one large
unfinished change.

Ground rules (binding — the repo docs override anything here if they conflict):

1. Before your first edit, read AGENTS.md (repo root) and docs/engineering-practices.md.

2. Every unit of work is issue-anchored. Pick issues via
   `gh issue list --repo hellosamblack/lidar-roomscanner --label work-item` (and
   `--label bug`), working priority/now first, then priority/next, then priority/later.
   Before editing, post a session-start comment on the issue; prefix every commit
   `Refs #NNN`; post an outcome comment when you finish or abandon the issue.

3. Nothing closes on unverified work. Before any `Closes #NNN` or `gh issue close`,
   answer: what would prove this is fixed, and did I actually run it, today, against
   real data? If the proof needs hardware, a capture that does not exist, the network
   path this host cannot exercise, or a human's eyes, the issue STAYS OPEN — label it
   `needs/operator` plus a subtype (capture/network/hardware/eyes/decision) and
   `status/fix-unverified`, and state exactly which check is unrun. A candid
   "not verified on hardware" paragraph inside a closing comment is not a hold.

4. priority/later issues #110–#117, #149–#154 and #132–#135 have design specs under
   docs/superpowers/specs/ — read the governing spec before touching those issues.

5. firmware/vendor/** is read-only reference. Never edit anything under it.

6. Tests run from host/: `cd host && python -m pytest`. The healthy baseline includes
   2 environment skips (pi-bridge nft/mtools). Read the final tally directly — do not
   pipe pytest through tail/head, which masks the exit code. Run the suite before every
   commit that touches host/.

7. Do not bind the sensor device stream or port 8000 — a long-lived roomscan-web
   instance may be running. `pgrep -af roomscan` before starting anything that serves
   or captures; never work around a conflict by changing the port.

8. This checkout is shared with other agent sessions. Run `git status` first; if there
   are uncommitted changes you did not write, leave them alone — never revert, stash,
   or clobber them. Before implementing anything, `git diff` the target files: a prior
   session may have already written the change uncommitted.

9. Docs move with the code: when a doc describes the thing you changed, update it in
   the same commit. Keep commit messages in the repo's conventional style
   (`fix(host-web): … (Refs #NNN)`).

10. Stop when the budget is spent or the priority/now + priority/next queue is
    exhausted. End with a per-issue summary comment: what landed (commit SHAs), what
    is verified and how, and what remains open and why.
```

## Notes

- This is the manual, single-session counterpart to the `issue-fleet` skill /
  `host/tools/fleet_run.py` (which orchestrates parallel Claude workers). Do not run
  both against the backlog at the same time — they will collide on issues and files.
- Run ledger for fleet-style sessions is `docs/fleet-ledger.md`; if the burndown lands
  a substantial batch, add a row there.

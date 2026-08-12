# `fleet_plan` / `fleet_budget` fixtures

Captured once from live sources on 2026-08-12 so the tests are offline and deterministic.
Regenerate only when the shape of an upstream payload changes — not to refresh the contents,
because several tests assert against specific issue numbers in `issues_open.json`.

| File | Source | Trimming |
|---|---|---|
| `issues_open.json` | `gh issue list --repo hellosamblack/lidar-roomscanner --state open --limit 200 --json number,title,labels,body,comments,updatedAt,createdAt,blockedBy,blocking` | 29 of 65 open issues, chosen to cover every footprint-confidence tier, all three priorities, every excludable `status/*`, both schedulable type labels, `data-collection`, firmware + host areas, and several `area/host-web` issues whose seed paths are disjoint but whose expansions overlap. Comment objects keep only `body`, `createdAt`, `author.login`; labels keep only `name`. Bodies and comment bodies are **verbatim** — truncating them would drop the path mentions the fixture exists to carry. |
| `git_log_name_only.txt` | `git log --since=6.months --name-only --pretty=format:'%x00%s'` | Commits with no files (merges) and the 28 tree-wide sweeps of >30 files dropped; they carry no usable co-edit signal and dominated the size. 485 commits kept. |
| `tracked_files.txt` | `git ls-files` | `firmware/vendor/` and `references/datasheets/` removed — 1682 of 2510 tracked files and 75% of the bytes, cited by zero issues, and never a legal edit target (vendor packages are read-only reference). |
| `ccusage_blocks.json` | `npx -y ccusage@latest blocks --json` | Last 12 blocks, enough to span a 5h boundary and the rolling 168h edge. |
| `ccusage_blocks_mangled.json` | derived | Structurally valid JSON with `tokenCounts` → `tokenTotals` and `startTime` → `begins`. Pins the degradation path: a renamed key must yield `verdict: "unknown"`, never a zeroed-out healthy-looking 0%. |
| `transcripts/` | synthetic | Mirrors the `~/.claude/projects/` layout **including the nested `<session>/subagents/agent-*.jsonl` path**. Reference `now` for every test reading this tree is `2026-08-12T03:00:00Z`. |

## What the transcript tree encodes

Three properties, each measured against the real store before being written down:

1. **The nested subagent path.** Subagent spend lives at
   `projects/<proj>/<session-uuid>/subagents/agent-*.jsonl`, one level below the main
   transcripts. A flat `projects/*/*.jsonl` glob misses 100% of it.
2. **Duplicate records.** The same `(message.id, requestId)` pair recurs in the store.
   Summing naively overcounts — measured at **185.8M vs 102.8M weighted tokens** for a single
   5h block, an ~80% overcount. `m1`/`q1` and `s2`/`r2` are duplicated here to pin the dedup.
3. **Rolling vs calendar week.** `m4` sits at `2026-08-06T09:00Z` — ISO week 32, while `now`
   is week 33 — but inside the rolling 168h window, which opens at `2026-08-05T03:00Z`.
   Calendar-week bucketing drops it; a correct rolling window keeps it. This is why
   `fleet_budget` computes the 7-day window itself rather than shelling out to
   `ccusage weekly`, which buckets by calendar week.

Plus a malformed line and a usage-free line, which the reader must skip rather than raise on.

## Verified against ccusage

With dedup on `(message.id, requestId)` and both globs, a native read of the active 5h block
reproduced `ccusage blocks --json` exactly on entry count (790) and on `inputTokens`,
`cacheCreationInputTokens` and `cacheReadInputTokens`. Only `outputTokens` differed, by 27k,
because the measuring session kept writing between the two reads. **ccusage does include
subagent transcripts** — the native reader here is a fallback and a coverage regression check,
not a correction.

# Bug tracker

Defects moved to **GitHub Issues** (2026-08-10): `gh issue list --repo hellosamblack/lidar-roomscanner --label bug`.

File a new one: `gh issue create --repo hellosamblack/lidar-roomscanner --label bug --label area/<area>`
(area labels: `area/host-viewer`, `-panel`, `-sensors`, `-slam`, `-web`, `-transport`, `-tools`,
`-offline`, `-splat`, `area/firmware`, `-eth`, `-scanner-stream`, `-build`, `-host`,
`area/transform-lib`, `area/environment`). Close one: `gh issue close <n> --reason completed`
(or `"not planned"` for a by-design/anomaly/investigated call — add the matching `status/*` label).

All 98 legacy `BUG-NNN` write-ups were migrated verbatim as issue bodies. Titles carry **no** ID
prefix (the `BUG-NNN:` prefixes were stripped 2026-08-11 — they collided with GitHub's own `#NNN`;
the `bug` label denotes the type). Each body keeps a `Legacy ID:` line so GitHub's search still
finds the old ID, and [`docs/issue-migration-map.md`](docs/issue-migration-map.md) is the
authoritative old-ID → issue mapping. `ROADMAP.md`'s "Reference-firmware bugs" section (vendor
package, not GitHub-tracked) is unaffected.

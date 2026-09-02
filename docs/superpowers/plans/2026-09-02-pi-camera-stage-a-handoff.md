# Handoff — Pi-bridge C270 live preview (#226, Stage A) — fleet-20260901-2043

**Written 2026-09-02 08:45 CDT by the fleet orchestrator session.** Pick up from here; do not
re-derive anything in "Verified". Machine handoff (claims, budget anchors, run-state block) is in
`.fleet/fleet-20260901-2043.md` (gitignored, this machine). Governing spec:
`docs/superpowers/specs/2026-08-28-pi-bridge-webcam-datastream-design.md`.

## 1. State of main

| commit | what |
|---|---|
| `f55e268` | host ingest `roomscan.camera` (multipart parser, latest slot, `CameraClient`), `/api/camera/{stream.mjpg,frame.jpg,status}`, `camera` `/ws` state message, browser card, `camera_status`/`camera_snapshot` MCP tools |
| `6a31445` | `docs/mcp-server.md` camera tools |
| `b4ec60a` | Pi: §5.1 characterization (`pi-bridge/tools/cam_characterize*.{py,md}`), `roomscan-cam` daemon + unit + installer/reconcile/status/nft wiring, six review fixes |
| `53ea5d1` | camera card state chip moved into the header row (was `position:absolute` over the title) |
| `1298cd2` | reconcile honours an operator stop/disable/mask (`is-enabled` + `Result`, not `is-active`); status `disabled_by_operator`/`note` |
| `479bb06` | `roomscan-web` SIGTERM bounded while a client holds the MJPEG stream; `host/tools/camera_shutdown_probe.py` |
| `da10b1e` | docs: web-protocol `camera` row + `/api/camera/*` rows, system-architecture "Rig camera", pi-bridge-runbook §6.5 |
| `aaa9b86` | daemon test no longer writes `__pycache__` into the shipped payload tree |

**Deployed:** the payload is on the bridge (`bridge_update` at 00:44 BST 2 Sep), `roomscan-cam.service`
active, 720p30 MJPG, `degraded=false`. `roomscan-web` (user unit) restarted on `479bb06`.

## 2. Verified (live, real hardware — do not repeat)

- Daemon 29.7–29.8 fps at 1280x720 MJPG; `exposure_dynamic_framerate=0` read back; `/status.json`
  `controls_mismatch=null`. Overnight 13 h: 1.47 M frames, 43 error buffers dropped, 0 malformed, 0 restarts.
- Host ingest live at 29.4–29.9 fps, 1280x720 by SOF, `capture_timestamp_valid=true`; 8 reconnects in 13 h
  (wlan0 stalls; each recovers within seconds with a generation bump).
- Browser: real pixels in the card `<img>` (naturalWidth 1280), placeholder hidden, chip geometry 0 px overlap,
  `elementFromPoint` hits the chip. `camera_snapshot` wrote `results/camera/camera-*.jpg` (66 KB).
- **Coexistence gate (§5.1), paired 5-minute windows with the scanner streaming through the bridge:**
  eth0 rx 451.6 pkt/s camera-on vs 450.9 camera-off, 0 drops/errors both; HUD drops 0, gaps 7 vs 6 per window;
  wlan0 tx 2.52 MB/s vs 0.49 MB/s; Pi temp 64–67 °C, throttled 0x0; camera age no monotonic growth. **PASS.**
  `tick_share` not measured: SLAM was not running during the arms.
- Daemon stop/start → host generation bump, card back to Live. `kill -9` self-heals in 5 s.
  Operator `systemctl stop` stays stopped (45 s, no reconcile action), status `warning: null` + `note`.
- `systemctl --user restart roomscan-web` with a browser holding the stream: **3.7 s** (was 90 s SIGKILL).
- Pi→host socket latency (worker's 90 s arms): median 52–55 ms, p95 ~90 ms, p99 273–396 ms.

## 3. Open — why #226 is not closed

1. **Far-side browser-pixel latency (§5.1 hard gate p95 ≤250 ms, target ≤150 ms).** Needs a host-timed
   black/white fiducial on a monitor **in the camera's view**; the C270 sees an equipment case and a wall.
   The probe tool (`/static/fiducial.html` + `host/tools/camera_latency_probe.py` + MCP
   `camera_latency_probe`) was dispatched to the host worker; see §5 for whether it landed.
   Owner action: prop a phone/tablet showing the fiducial page in front of the camera.
2. **Physical unplug/replug close gate (§9 Live Stage A).** Hands on the rig.
3. Low-severity review findings on `479bb06` (gate not cleared at lifespan restart; broad `except` in the
   bounded camera stop; unarmed-gate branch silent) — in the host worker's follow-up, see §5.
4. Stage B (#227) and C (#228) remain `status/blocked` on this.

## 4. How to resume

```
git worktree list                      # issue-226-host2 may still exist with the follow-up / fiducial work
git -C .claude/worktrees/issue-226-host2 log main..HEAD --oneline
cd host && .venv/bin/python -m pytest -q tests/test_camera.py tests/test_camera_routes.py \
    tests/test_camera_shutdown.py tests/test_roomscan_cam.py tests/test_pi_bridge_build.py
```

Land with rebase-in-worktree → narrow tests → `merge --ff-only`; then `bridge_update` (Pi side) and
`systemctl --user restart roomscan-web` (host side); confirm with `curl localhost:8000/api/camera/status`.
Re-run the coexistence arm with `pi-bridge/tools/cam_characterize.py arm --seconds 300` (receiver on the host)
only after a camera/kernel/Wi-Fi change. To take the camera off for a measurement:
`systemctl mask --now roomscan-cam.service` (not `--runtime`: the unit is in /etc).

## 5. In-flight at handoff

Nothing. All worker branches are merged (`0db98d6` closed the low findings) and every worktree is
removed; `git worktree list` should show only the main checkout. **The fiducial latency probe was never
started** — the task message was lost when the worker was killed by the account session limit — so the
next work item on #226 is exactly that: `/static/fiducial.html` (host-timed black/white toggle, served by
`roomscan-web`, shown on a phone in front of the camera), `host/tools/camera_latency_probe.py` (sample
`/api/camera/frame.jpg` or the browser `<img>` via canvas, detect the transitions, report N / median /
p95 / p99 / max, and a null result when no fiducial is in view), an MCP wrapper `camera_latency_probe`,
tests on a synthetic JPEG sequence, and a live null run. The Operator Request posted on #226 assumes
that page exists; write the tool before asking the owner to prop the phone up.

## 6. Learnings for the CI pipeline

Each of these was a defect that shipped past a green suite this run.

1. **Per-test timeouts.** A worktree test run hung >400 s on an in-progress shutdown change (a probe
   subprocess waiting on a server that never exited). CI should run with `pytest-timeout`
   (`--timeout=120` default, longer only on `slow`-marked tests) so a hang fails fast instead of holding the
   runner. Today a hang looks like a slow job.
2. **Run the subprocess/socket probes in CI, not only the fakes.** `host/tests/test_camera_shutdown.py`
   (auto-marked `slow`/`subprocess`) is the only test that could ever see the SIGTERM hang; the 16 green
   lifespan tests could not. Keep the `slow` lane in CI, and treat a new native/socket/subprocess path
   without a probe as a review blocker. Same for `host/tools/browser_reap_probe.py`.
3. **Shipped-tree hygiene must run after the tests that can dirty it.** Importing a payload module wrote
   `pi-bridge/payload/.../__pycache__` that install.sh would have shipped to the Pi; the three payload-hygiene
   tests only fail if they run *after* the daemon tests. `aaa9b86` adds an in-suite guard, but CI should also
   `git status --porcelain --ignored -- pi-bridge/payload` after the run and fail on anything new.
4. **Source-text tests are not tests.** Every reconcile test asserted on the *script's text*, which is why
   a self-heal that undid `systemctl stop` shipped, deployed, and was caught by a human. `1298cd2` makes the
   policy a pure function exercised by tests (`ROOMSCAN_RECONCILE_LIB_ONLY`, a fake `systemctl`). CI rule:
   a test that only greps a shell script's source is a lint, not a behaviour test — reviewers should ask
   for an executed harness for any policy branch.
5. **Rendered-geometry checks for UI cards.** The state chip covered the card title by 10.6 px; no static
   test can see flex/absolute resolution. The host worker's Playwright probe (48 rail-width × state
   combinations, `elementFromPoint` hit test, positive control against pre-fix markup) lives at
   `/tmp/probe226ui.py` on this box and should be landed under `host/tools/` and run in the browser lane.
6. **Mutation-check the assertion, not just the code.** An `exit_s < 5` assertion passed with the route-level
   fix removed because the 3 s backstop stood in for it; the test now asserts `via == "route"`. Where two
   mechanisms bound the same failure, CI tests must name which one fired.
7. **Ruff ratchet per file.** Workers verified "no new findings" by comparing per-file counts against
   `git show main:<file>`; a CI step that fails when a modified file's ruff count rises would make that
   automatic (web.py went 93 → 32 this run by naming two symbols explicitly).
8. **Contract seams between parallel branches need one executed test.** Two workers built the Pi daemon and
   the host client from the same spec and disagreed on a header name and on the status-JSON keys
   (`present` vs `usb_present`); only a grep at the review gate caught it. A tiny contract test that loads the
   daemon's emitted headers/status shape and feeds them to the host parser (both are importable, no
   hardware) would catch this in CI. Not yet written.
9. **Docs gate is load-bearing.** `test_mcp_registry::test_every_tool_is_named_in_the_server_doc` was the
   only red test on the host branch and it was right; keep it, and remember fleet workers cannot fix it (docs
   are orchestrator-owned), so a red run there means "apply the doc delta", not "fix the code".
10. **Live restart timing as a check.** `time systemctl --user restart roomscan-web` with a client attached
    is a one-line release check; the probe automates it without systemd. Consider a post-deploy step.

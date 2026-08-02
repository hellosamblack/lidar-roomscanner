# Responsive 3D magnetometer-calibration feedback

**Status:** ✅ implemented 2026-07-29 (`cf3b243`), with follow-up corrections through 2026-07-31;
see `ROADMAP.md` Phase 6, "3D calibration feedback (Shell & Steering)". **Date:** 2026-07-29.
**Revised 2026-07-30 — the hero camera moved to first-person; see §4.1, which is amended in place.**
**Revised 2026-07-31 — the hero and Steering views are MERGED into one canvas, and the device model is
now the real instrument rather than a board silhouette; see §4.1, amended in place again.**
**Owner ask (verbatim intent):**

> "I want the feedback to be responsive. It should show the current and past orientation of the board
> in 3d space at 30+ hz, with the magnetometer overlaid. Be creative on how you want to represent
> this, the goal is to make it clear to the user what has been done/what needs to be done so they
> don't feel like a crazy person just waving this thing around in the air with no idea what the goal
> and progress is."

The success criterion is **subjective legibility under motion**. While tumbling a handheld board the
user must know at a glance: (a) where they are, (b) where they've been, (c) what's still missing,
(d) how close to done. "Doesn't feel like a crazy person" is the actual requirement; frame rate and
protocol choices below exist only to serve it.

Related: `docs/web-protocol.md` → "Magnetometer sweep + calibration quality" (the existing message),
`docs/coordinate-frames.md` (**binding** — every frame below is one of its four), `BUGS.md` BUG-030,
`host/src/roomscan/magsweep.py`, `host/src/roomscan/static/magcal.js`.

---

## 1. Why this exists (do not re-derive — measured)

BUG-030: the saved calibration is **direction-dependent**. Measured over a tilt sweep, |B| reads
47 µT ceiling-facing and 85 µT horizontal against a fitted `field_ut` of 49.87 µT — heading errors up
to ~90° in exactly the horizontal wall-scanning attitude the scanner is used in. Root cause is an
**incomplete tumble**: dense coverage in one attitude family, none elsewhere. The ellipsoid fit
interpolated a plausible ellipsoid through a cap of the sphere and nothing downstream noticed.

The 2026-07-29 modal already measures this after the fact. What it does not do is make the *missing
part of the sphere* felt in the hand, in the moment. That is this design's whole job: **"what's
missing" must be unmissable, and "how close am I" must be continuous.**

## 2. What exists today, and what is kept unchanged

| Piece | Decision |
|---|---|
| `magsweep.py` — Fibonacci 92-cell lattice, nearest-centre assignment | **Kept verbatim.** Equal-area cells are the reason a coverage % means anything; lat/lon bins vary ~50× in area pole-to-equator and would report "covered" for the exact tumble that caused BUG-030. |
| Binning the **calibrated, body-frame** direction, not the raw one | **Kept, load-bearing.** The rig's hard-iron offset (~65 µT) exceeds the field (~50 µT), so raw directions live in a cone covering barely half the sphere however well you tumble. Guarded by `test_binning_uses_calibrated_not_raw_directions`. The 3D view plots exactly the same quantity `magcal.js` plots today; it changes the *projection*, not the data. |
| Quality thresholds, verdict composition, `limited_by`/`reason` | **Kept verbatim.** |
| Coverage encoded by **shape** (filled vs dashed hollow), not colour | **Kept and extended** (§9). |
| The two 2D Lambert equal-area discs | **Kept as the fallback renderer** (§8.5), not deleted. They are the only view that needs no orientation at all. |
| `display_rotation`, the point-cloud path, `fused_quat()`, anything SLAM reads | **Untouched.** Everything here is presentation-only; §7.4 states the guard. |

## 3. The framing decision

### 3.1 The two framings are one object under two cameras

Coverage is over the **body-frame** magnetic-field direction `d_body = Rᵀ · b_world`. The brief poses:

- **(a)** a sphere rigidly attached to the device, rotating on screen with it, painted where the
  world-fixed field crosses it;
- **(b)** a screen-fixed sphere on which each sample is plotted as its body-frame direction, so the
  "you are here" marker moves while the sphere stays put.

These are **not two data models. They are the same scene rendered from two cameras.** In (b) the
camera is bolted to the device (body-fixed): the shell, the holes, and the device model are all
stationary, and only the field marker moves. In (a) the camera is bolted to the room (world-fixed):
the device and its shell tumble together, and the field marker is the thing that stays put — because
the Earth's field genuinely is fixed in the room.

That reframing matters because it converts an either/or into a camera parameter. In a real 3D engine
switching between them is `camera.position` + a group transform — no second geometry, no second
buffer, no second data path.

### 3.2 What each camera is good at, and the recommendation

| | Body-fixed (b) | World-fixed (a) |
|---|---|---|
| "where am I" | marker moves — reads well | marker fixed, shell moves — reads well |
| "where have I been" | **stable map, countable** | holes tumble; uncountable |
| "what's missing" | **stable, plannable** | you cannot hold a spinning hemisphere of holes in your head |
| "how do I move my hands" | indirect: the marker moves *opposite* to the device, about an axis that isn't drawn | **direct: sweep the shell so the holes pass over the fixed arrow** |
| honesty | exact | exact |

**Recommendation: body-fixed (b) is the hero view; world-fixed (a) survives as a small
"Steering" widget beside it.** Rationale:

1. The failure this UI exists to prevent (BUG-030) is *not knowing what's missing*. That is a
   planning task, and planning against a spinning object is impossible. The stable map wins the
   requirement that pays for the work.
2. (b)'s only real weakness — "how do I translate this into a wrist motion" — is **fully solvable
   without changing camera**, because the required device rotation is exactly computable (§5) and can
   be drawn as a curved arrow on a device model. Once you draw that arrow, (b) loses nothing.
3. (a) still earns a place, small, because a *rotating device model with a ghost target attitude* is
   the single most intuitive "do this with your hands" mechanic there is, and it only works when the
   device visibly moves. At 200 px it costs one extra camera.
4. Bonus, and not a small one: **the hero view needs no orientation data at all.** In the body frame
   the shell is `cell_dirs` verbatim, the marker is `live_dir` verbatim, and the device model is at
   identity. So the hero renders correctly on a session with no stream 9 (§7.3), and it is immune to
   every orientation-noise and gimbal concern in `docs/web-protocol.md`.

**Rejected: (a) as the hero.** Physically honest, and initially attractive because the field vector
being world-fixed is a lovely truth. But a shell whose holes orbit at hand speed is unreadable — the
spike screenshot of this framing (appendix A) makes the case: you can see *that* there is a gap and
never say *where*, and you certainly cannot tell whether it is shrinking.

**Rejected: an unrolled 2D "map that fills in" as the hero.** That is what ships today, and it is
genuinely good at (c) and (d). It is rejected as the hero only because the owner asked for 3D
orientation and because the disc pair cannot show the *device*, which is what carries the "so that's
what I have to do with my hands" moment. It is retained as the fallback renderer, where its
orientation-independence is exactly the property we need.

**Rejected: an anchored 3D device with a shell around it, rendered in a free-orbit camera.** Letting
the user orbit sounds generous; it destroys the one thing that makes (b) work, which is that a hole
is *in the same screen place* it was three seconds ago. The hero camera is fixed (with a slow,
optional auto-orbit off by default; see §11 open question 3).

## 4. The concept — "Shell & Steering"

```
┌─ Magnetometer Calibration ──────────────────────────────────────────────── × ─┐
│                                                                               │
│  ┌────────────────────────────────────────────┐  ┌─────────────────────────┐  │
│  │   COVERAGE SHELL              body-fixed   │  │ PROGRESS                │  │
│  │   (hero, 560 × 560, WebGL)                 │  │  ╭────────╮             │  │
│  │                                            │  │  │  66 %  │  61/92 cells│  │
│  │        ○ ○ ○ ○                             │  │  ╰────────╯             │  │
│  │      ○ ○ ○ ○ ○ ○         ← missing cells   │  │  ├─────┼───┼──────┤     │  │
│  │    ○ ○ ○ ○ ○ ● ●           (hollow, dashed,│  │      60%  85%           │  │
│  │   ○ ○ ○ ● ● ● ● ●           stippled)      │  │                         │  │
│  │   ○ ○ ● ●┌──────┐● ●                       │  │ LIVE FIT (rolling)      │  │
│  │   ○ ● ● ●│device│● ● ●    ← covered cells  │  │  |B| spread   3.1 %  ▲  │  │
│  │    ● ● ● └──────┘ ● ●       (solid; fill = │  │  |B| bias    +0.4 %  ●  │  │
│  │     ● ● ● ↗B ● ● ●          |B| error,     │  │  samples        842  ●  │  │
│  │       ● ●╱ ● ● ●            radius = count)│  │  coverage       66 %  ▲ │  │
│  │        ●╱● ● ●                             │  │                         │  │
│  │        ╱   ⊚  ← next target (double ring)  │  │ B∠g  114.2° ± 0.9°      │  │
│  │       ╱      ⌒ geodesic path               │  │  ▁▂▃▂▁▂▃▂▁▂▃  (60 s)    │  │
│  │      ⊙ head + 3 s comet trail              │  │                         │  │
│  └────────────────────────────────────────────┘  │ SAVED CAL      ✕ BAD    │  │
│                                                  │ CANDIDATE      — none   │  │
│  ┌──────────┐  NEXT: roll ≈ 70° about the        │ map colours: [cur][cand]│  │
│  │ STEERING │  device's long axis, Top → Back.   └─────────────────────────┘  │
│  │ (a) view │  14 cells left in this gap.                                     │
│  │  ◜◝ ghost│  ⟲ ← curved wrist arrow on the model                            │
│  └──────────┘                                                                 │
│                                                                               │
│  [Start] [Stop & Fit]                    [Discard fit] [Clear] [Save & apply] │
└───────────────────────────────────────────────────────────────────────────────┘
```

### 4.1 The shell (hero)

**Frame: SFLP body (X = Up, Y = Right, Z = Forward).** The camera sits on a fixed offset looking at
the origin. Face labels `Top / Bottom / Right / Left / Front / Back` sit at ±X, ±Y, ±Z at radius
1.15 — the same six names `magsweep.FACES` already uses, so the guidance text and the picture name
the same thing.

> **Amended 2026-07-30 (owner):** *"During mag cal, we should render the view from the first person
> perspective of the camera (gravity down always, similar to the fpv world view)."* The camera is now
> parked **behind the device on the boresight** (body −Z, standoff 4.3, fov 40°) looking along it, and
> `camera.up` **tracks −g** instead of being pinned to body Up — the same rule the live view's FPV
> mode applies to the cloud (`web.boresight_view_frame`), here applied to a camera instead of to
> points. Consequences: the shell you are steering through is now the sensor's own field of view;
> screen-down is room-down always; the camera's only motion is a gravity roll, so §3.2's "the hole is
> where it was" property survives (a hole moves only when you roll the board, and then it moves the
> way the room does). The `Front`/`Back` labels are dropped — the boresight axis projects to a point,
> so both would land stacked on the device at screen centre. Roll is eased with a 0.12 s time
> constant on the *camera only*; no mark is ever drawn from a smoothed number.

**Cells.** All 92 lattice cells always drawn, as discs **tangent to the sphere** (oriented by their
own normal, so orientation reads and a back-face disc is edge-on-ish rather than a flat sticker).
One `InstancedMesh` of 92, per-instance matrix + colour.

| channel | encodes | why not colour |
|---|---|---|
| **fill vs outline** | covered vs missing | preserves today's CVD-safe shape encoding, verbatim |
| **outline dash + stipple** | missing | survives the reduced opacity of back-face cells, where a dash alone gets thin |
| **disc radius** | sample count, ramped 1 → 8 samples (thin ring → full disc) | new second non-colour channel: "barely visited" vs "well visited" is a *size*, so a cell grabbed by one stray sample cannot masquerade as solid coverage |
| **fill colour** | mean signed |B| deviation, existing diverging ramp | the one thing colour means (§9) |

Back-hemisphere cells render first at 0.35 opacity with `depthWrite: false`; front cells at full
opacity. The user sees through the shell to the far side, which matters: a gap on the far side must
be visible, not hidden.

> **Amended 2026-07-30 (owner: "the sphere should be translucent for points that are 'behind' the
> camera").** The translucency now keys on **`dir·boresight < 0`** — the cells behind the camera —
> not on distance from the eye. Under the first-person framing above those *are* the near ones, and
> the usual far-is-faint depth cue would be exactly backwards: the near cap covers the entire
> silhouette, so fading the far side would hide the hemisphere you are aiming into. Ghosting the near
> cap (0.26 filled / 0.30 missing, against 0.90 / 0.95) lets you see the aim hemisphere *through* it
> and still find a gap behind you. Implementation note: `InstancedMesh`'s per-instance channel is
> colour, not alpha, so this is a **material** split — four meshes (covered × behind), each cell
> parked at scale 0 in the three that don't apply. The split is static, because the camera is fixed
> in body axes and only rolls. Hue is left untouched by the ghosting (alpha carries depth, so the
> |B| ramp still reads on a cell behind you) — which also retires the old brightness-mix depth cue.
> The device model's fill drops to 0.30 in the hero for the same reason: seen from behind it is
> face-on over the middle of the shell, where the target ring and geodesic live. The Steering widget
> keeps the solid fill — its whole mechanic is landing a solid body inside a wireframe ghost.

**Device model.** A small extruded board silhouette at the origin (NUCLEO outline + a nub for the USB
end + a dot for the ToF aperture), muted ink, wireframe-ish. In the hero (body-fixed) camera it is
**stationary** — which is correct and, importantly, is what makes the shell readable. It is the
anchor that tells you the shell is *yours*, not an abstract globe.

> **Amended 2026-07-31 (owner): the two views are MERGED, and the device is the real one.**
>
> *"Can they be combined? The orientation from the bottom one, with coverage from the top one?"*
> They can. There is now **one canvas** (`#magcal-hero`, unchanged 470 × 340) rendering **one pass**:
> the **Steering framing** (world-fixed `steerCam`, `bodyGroup = T_WORLD_TO_CV·R`, ghost at `R·ΔR`,
> wrist arrow) with the **hero's cell styling** (dashed hollow rings for missing, discs sized by
> sample count, hue = |B| deviation). The faint "steer" cell wash is deleted — with no second view,
> a coverage readout nobody can read is not a trade, it is a loss. `#magcal-steer` and
> `#magcal-steer-note` are gone; `#magcal-guidance` / `#magcal-state-chips` / `#magcal-binning` sit
> under the merged canvas.
>
> **This re-accepts a cost this spec had rejected**, and that rejection is not deleted: a body-fixed
> shell seen from a room-fixed camera has its holes orbit at hand speed, so you can see *that* there
> is a gap and never *where*. The mitigation is that the aiming instrument is no longer the shell —
> it is the **ghost** and the **geodesic leader line**, both world-fixed and therefore steady under
> exactly the motion that makes the shell swim. The shell reverts to being the progress readout. If
> it proves unusable on a real sweep the fallback is the inverse merge (body-fixed camera with the
> ghost drawn into it); same closure condition either way.
>
> **The body-fixed framing survives as the no-orientation fallback.** The old steering pass was gated
> on `have_quat`; a merged view built only on the steering framing would render an **empty canvas**
> on a ToF-only session. So when `have_quat` is false (or `t_world_to_cv` has not arrived) the pass
> falls back to `bodyGroup = identity` + the first-person `heroCam`, which needs no orientation at
> all — `cell_dirs` and `field_dir_body` are already body vectors — and `#magcal-hero-note` says why
> the ghost is missing. `window.__magcal3d.framing` reports `"world"` or `"body"`.
>
> **The near/far translucency split becomes a per-frame quantity.** It was static *because* the
> camera was fixed in body axes; with a room-fixed camera it goes stale immediately. It is recomputed
> from the eye direction expressed in body coordinates (transpose of `bodyGroup`'s rotation — no new
> convention is written client-side), throttled to 10 Hz and skipped below 1.5° of view-direction
> change, and it rewrites the 4 × 92 instance matrices only when a cell actually changes side.
> **Measured**, not assumed (2026-07-31, llvmpipe, `refreshMs`/`behindRecomputes` in
> `window.__magcal3d`, `?magcalstatic=1` for the frozen baseline): **0.062 ms per recompute** under a
> 90 °/s tumble, ≤ 10 Hz ⇒ **≤ 0.062 % of wall clock**; on a stationary rig it fires ~0 times/s and
> the A/B fps difference is below this box's run-to-run noise.
>
> **The device model is the owner's actual instrument**, not a NUCLEO silhouette: a **5.5″ × 3″ ×
> 2.5″ block**, dark grey for the half facing the user, white for the middle quarter, blue for the
> quarter facing away, camera on the blue face. It lives in a new shared module
> `static/devicemodel.js` and is drawn by **both** this view and the Sensors card's orientation gizmo
> (previously an RGB axis triad), so the two cannot teach different shapes. Two consequences worth
> keeping: the block is **8× deeper** than the old board, so its fill is ghosted to 0.30 in *both*
> framings (the Steering pass's old solid 0.85 would blank the middle of the shell — where the
> boresight, target ring and geodesic live); and `MOUNT_ROTATION`, a 180° turn about the boresight,
> encodes that **body +X is the device's BOTTOM** — the old model called it "Up, USB down" and was
> upside down. That constant is derived from the owner's own reading (held normally: World pitch 0°,
> roll 180°, and `triad_roll_deg` is the roll of body +X about the boresight against true vertical)
> and is pinned by `test_static_ui.py`, because a 180° error passes every symmetry check a box
> silhouette can offer.

### 4.2 Trail / history — the two-tier rule

The spaghetti problem has a clean solution: **the covered cells already ARE the long-term history.**
That is what a cell *is*. So the polyline never needs to be long.

- **Comet: the last 3 s** of the body-frame field direction (90 vertices at 30 Hz; 1440 at 480 Hz in
  Phase 3 — both trivial), drawn as a ribbon on the shell surface at r = 1.02.
- **Decay:** opacity and width ramp linearly over the window, from a bright 3 px head to a 0.8 px
  ghost tail. Neutral ink ramp (`#e2e8f0 → #b7c0cf → #8b94a3 → #5b6472`), validated as an ordinal
  ramp (§9). Neutral because hue is spoken for.
- **Head:** a filled dot at r = 1.02 plus a stem to the device centre — this is the "you are here".
- No session-length ribbon by default. A `Show full path` checkbox may draw the whole session
  decimated to ≤ 2000 vertices at 12 % opacity for the "what did I actually do" post-mortem; **off by
  default**, because during collection it is exactly the spaghetti the brief warns about.

Design note to preserve: the polyline's job is the **derivative** (am I moving, which way, how fast);
the cells' job is the **integral** (what have I covered). Keeping those on separate marks is why
neither becomes noise.

### 4.3 Magnetometer overlay

Three marks, all on the same device-centred origin:

1. **Field arrow B** — ink, solid, from origin out through the shell to the head dot. Its *direction*
   is the calibrated body-frame field direction (identical to today's `live_dir`).
2. **Field-magnitude tick** — a short radial tick at the arrow head, drawn **outward** when |B| reads
   high and **inward** when it reads low, length ∝ |dev%| clamped at ±30 %, coloured on the same
   diverging ramp as the cells. So the live |B| error is legible continuously, not only at fit time,
   and it is redundantly encoded (direction + length + colour).
3. **Gravity arrow g** — muted, dashed, from origin along the body-frame down vector. Sourced from the
   **stream-11 SFLP gravity FIFO tag** (0.061 mg/LSB, ~16× finer in tilt than the fp16 quaternion),
   falling back to `Rᵀ·[0,0,−1]` exactly as `sensors.gravity_body_from_imu_raw` /
   `web._…gravity_source` already do. Reuse, do not re-derive.

**The dip arc — the second free diagnostic.** Draw the angle between B and g as an arc between the
two arrows, labelled live: `B∠g 114.2°`. For a *correct* calibration this angle is a **constant of
the location** (90° + magnetic dip): both vectors are fixed in the world frame, so their mutual angle
cannot depend on attitude. If it wobbles as you tumble, the calibration is wrong — and unlike |B| it
is **immune to scale error**, so it catches soft-iron and axis-misalignment faults that a
self-consistent-but-wrong-magnitude calibration would sail past (the ×2.04-bias trap already recorded
in `docs/web-protocol.md`). Ship it as a live number plus a 60 s sparkline with a `± spread` readout.
This is a genuinely new signal, cheap, and it belongs in `magsweep.py` next to `field_consistency`.

### 4.4 The Steering widget (world-fixed camera, ~200 px)

Same scene, room camera (`camera.up = (0,−1,0)`, matching `scene.js`, so "up on screen" is up in the
room). The device model tumbles; B points fixed north-and-down; the shell is drawn faintly.

Its one job is the **ghost mechanic**: a wireframe ghost of the device at the *target attitude*
(§5), plus a curved arrow wrapped around the rotation axis. **Rotate the real board until the solid
model lands inside the ghost.** No text needed, no dip assumption, no hemisphere assumption.

Clicking either view swaps sizes (hero ↔ widget), so a user who prefers (a) can have it big. Default
is body-fixed hero.

## 5. Guidance — the exact wrist rotation

The current hint ("point the Top face toward magnetic north and downward") assumes northern-hemisphere
dip and asks the user to know where north is. Both go away.

**Target selection.** Reuse `magsweep.empty_regions`, which already returns connected components of
empty cells largest-first — chasing the biggest hole is right, chasing the nearest singleton is not.

```
target = argmin over cells c in the largest empty region of  angle(c, d_live)      if |region| >= 3
       = argmin over all empty cells c of  angle(c, d_live)                        otherwise
```

The `>= 3` guard means you are steered toward a hole worth filling, but a nearly-finished sphere with
only scattered singletons still gets pointed at the nearest one.

**The required rotation, exactly.** Let `d` = current body-frame field direction, `t` = target cell
direction. Rotating the device body by `ΔR` (applied in body axes, `R' = R·ΔR`) gives
`d' = ΔRᵀ·d`. We want `d' = t`, so `ΔRᵀ` is the rotation carrying `d → t`, hence

```
axis  n = unit( t × d )        # a BODY axis — drawable on the device model
angle θ = acos( clamp(t · d) ) # the minimal rotation
```

`n` is a body axis, so the curved arrow can be drawn literally around the model, and the ghost
attitude is `R_ghost = R · ΔR` — the minimal-effort representative of the one-parameter family of
attitudes that put the field in the target cell (any further spin about `t` leaves `d` unchanged).

**The text, generated from that:** decompose `n` onto the six body faces and name the dominant pair,
e.g. `Roll ≈ 70° about the device's long axis (Top → Back)`. No dip, no compass, no hemisphere.
Computed server-side (per the "server-side math stays server-side" invariant), shipped in the
`magcal` report as `guidance` + a structured `guidance_axis` `{axis[3], angle_deg, text}` so the
client draws the arrow from numbers rather than parsing prose.

**Geodesic path.** On the hero shell, draw the great-circle arc from `d` to `t` (32 segments), dashed,
at 60 % opacity. It is the path the head dot will actually trace, so following it is self-verifying.

## 6. Progress — continuous, not at fit time

Today the quality numbers only become meaningful after `Stop & Fit`. That is the wrong time: the user
wants to know *while tumbling* whether they can stop.

1. **Coverage ring gauge** around the hero: arc filled = occupied/92, with **tick marks at 60 % and
   85 %** (the existing `COVERAGE_MARGINAL` / `COVERAGE_GOOD` bars) so "how close am I" is a position
   relative to a drawn line, not a naked percentage. Centre label `66 %`, sub-label `61 / 92 cells`.
2. **Rolling provisional fit.** Refit on every report tick while collecting and show the *would-be*
   spread/bias/residual live, labelled `LIVE FIT (rolling)`. This is the single best progress signal:
   the user watches `|B| spread` fall through 5 % → 2 % and coverage climb past 85 %, and stops when
   both are green.
   **Cost is not a concern — measured** (this box, `host/.venv`):
   `fit_ellipsoid` = 0.23 ms @ 300 samples, 0.29 ms @ 1200, 0.90 ms @ 5000, 3.18 ms @ 20 000 (the
   `MAX_SAMPLES` cap). At the 5 Hz report cadence with a 4000-sample decimated subsample the
   broadcaster spends < 4 ms/s. Above that cap, decimate — do not move it off-loop unless a
   measurement says so.
3. **Four verdict chips** (coverage / samples / spread / bias), each `icon + label + colour`, never
   colour alone, always visible during collection rather than only in the post-fit block.
4. **The gap counter** in the guidance line: `14 cells left in this gap` — a countdown the user can
   watch tick down as they move, which is the difference between "waving it around" and "doing a task".

## 7. Protocol and cadence

### 7.1 The split: a fast binary pose channel, a slow JSON truth channel

The existing `magcal` JSON is **4 Hz and 4490 bytes** (measured, 1200-sample session). Pushing it to
30 Hz would be 135 kB/s and 30 × `JSON.parse(4.5 kB)` per second on the UI thread — for data (cell
counts, verdicts, coverage) that changes at human speed. That is the wrong axis to scale. The app
already has the right pattern: high-rate render payloads are **tagged binary**, human-rate state is
**JSON** (`docs/web-protocol.md` § Framing).

**New binary tag 5 — `MAGPOSE`.** Sent to `state.magcal_clients` only (the same per-tab subscriber
set the report already uses — a session with the modal closed everywhere costs exactly nothing), on
the broadcaster's existing `POINT_INTERVAL` (30 Hz) tick.

```
u32  tag = 5
u32  seq
f32[4]  quat            # raw fused body->SFLP-world (w,x,y,z), FULL f32
f32[3]  field_dir_body  # calibrated, AXIS_CONVENTION-applied, unit
f32[3]  gravity_body    # unit, stream-11 SFLP gravity when available
f32     field_ut        # |B| this sample
f32     dev_pct         # signed |B| deviation vs the viewed calibration
f32     dip_deg         # angle(B, g)
i16     live_cell       # -1 = none
i16     filled_cell     # cell newly occupied by THIS sample, else -1
u16     flags
u16     _pad
                        # 68 bytes -> 2.0 kB/s at 30 Hz
```

`flags`: bit0 collecting · bit1 stationary · bit2 mag_anomaly · bit3 have_quat · bit4
provisional_binning · bit5 sample_rejected.

**Why binary, not a small 30 Hz JSON.** Precision. The brief is explicit that orientation must not be
measured off `sensor.rot` (5 dp). f32 binary avoids inventing a second rounding policy and matches
`pack_point_cloud`'s existing precedent. It also keeps the `ws.js` demux uniform.

**`filled_cell` is the trick that keeps the JSON slow.** The 30 Hz channel carries the *delta* (this
sample just lit cell 47), so the client paints a cell solid the instant it fills — the responsiveness
the owner asked for — while the 4–5 Hz JSON remains the *truth* that reconciles counts, deviations,
and verdicts. Server computes `dev_pct` so the client does no calibration math.

### 7.2 Changes to the existing `magcal` JSON

- **Send `cell_dirs` only on `open`.** It is a deterministic constant of `SPHERE_CELLS` and is
  currently re-sent 4×/s. Measured: 4490 B → **1982 B** per report, a 56 % cut.
- Raise the cadence 4 Hz → **5 Hz** (net still ~2× cheaper than today: ~10 kB/s vs ~18 kB/s).
- Add: `guidance_axis {axis[3], angle_deg, text}` (§5), `dip {deg, spread_deg, hist[]}` (§4.3),
  `live_fit {…}` (the rolling provisional fit, §6), `stationary`, `anomalous_count`,
  `t_world_to_cv[9]` (sent once on `open`, so the client composes frames from a server-supplied
  matrix rather than hard-coding sign conventions — §7.3).
- Everything already in the message stays, unchanged. The 2D fallback renderer keeps working off it
  untouched.

### 7.3 Frames — what the client is allowed to compute

Per `docs/coordinate-frames.md`, and per the "server-side math stays server-side" invariant:

- **Hero (body-fixed):** *no transform at all.* `cell_dirs`, `field_dir_body`, `gravity_body` are
  already SFLP-body unit vectors. This is why the hero works with no orientation data.
- **Steering (world-fixed):** a body-frame vector reaches the renderer's world as
  `T_WORLD_TO_CV · R · v_body` (note: **not** the `T_WORLD_TO_CV · R · T_CV_TO_BODY` sandwich — that
  one maps *CV* points; ours are already body points, so the `T_CV_TO_BODY` leg is absent).
  `T_WORLD_TO_CV` is shipped once as `t_world_to_cv[9]` on `open` and applied as a static
  `Object3D.matrix` on the group; the only per-frame client math is `quat → Quaternion` and a slerp.
  No sign/permutation matrix is ever written in JS.

### 7.4 Isolation guarantee

`MAGPOSE` reads `sensor_state.fused_quat()` / `latest_env()` / `latest_imu_raw()` and writes nothing.
Extend `tests/test_magsweep.py::test_magcal_preview_does_not_touch_display_path` to drive a full
open → pose-stream → close cycle and assert `display_rotation`, the packed point-cloud bytes,
`fused_quat()`, and the loaded `mag_cal` are bit-identical. Follow the `protocol-change` skill for the
new tag: `docs/web-protocol.md` table row, `web.py` constant, `ws.js` constant, in lockstep.

## 8. Pose rate and the rendering approach

### 8.1 Decouple render from data — mandatory

Render on `requestAnimationFrame` at display rate (the owner reports the main app at 144 fps), and
**slerp between the last two received poses**, targeting one pose-interval (≈33 ms) of presentation
latency. Rendering 144 fps against 30 Hz data without interpolation shows visible 4-frame stair-step
stutter; this is now the *primary* argument for slerp, not a performance dodge.

Interpolate, do not extrapolate: extrapolating a hand tumble overshoots on direction reversals, and
this view's credibility rests on it never showing motion that did not happen.

### 8.2 The high-rate opportunity: stream 11 + `roomscan.imufusion`

Stream 11 batches the raw LSM6DSV16X FIFO per ToF frame: **GY_NC gyro at 480 Hz** (17.5 mdps/LSB,
16-bit fixed point — *not* fp16), the SFLP gbias estimate, the SFLP gravity vector, and LSM TIMESTAMP
words (21.7 µs/LSB) which are the only usable `dt` source. `host/src/roomscan/imufusion.py` already
integrates exactly this — gyro propagates, gravity corrects tilt, stream 9 anchors yaw — and is
**opt-in and OFF by default** (`SensorState(imu_fusion=None)`).

This is the right long-term answer and I recommend it, with two corrections to how it is usually
framed:

1. **`ImuFusion.update()` currently outputs one quaternion per ToF frame, not 480 per second.** It
   integrates the 480 Hz words internally but only exposes the end-of-batch estimate. So attaching it
   as-is buys *quality* (below the fp16 0.056°/step floor), not *rate*. Getting rate needs a small
   additive `update(..., trace=out)` that appends the per-sample quaternions from `_propagate`'s loop.
2. **Do not attach it to `SensorState`.** Give the magcal session its **own private `ImuFusion`
   instance**, fed `latest_imu_raw()` with `yaw_ref = sensor_state.fused_quat()`. It is then
   non-regressing *by construction*: `fused_quat()` and everything SLAM reads are untouched, the
   global opt-in stays off, and the calibration view becomes the filter's first real consumer without
   betting the reconstruction on it. Drift is bounded because the same anchor `ImuFusion` already
   applies every batch.

Wire it as **tag 6 `MAGPOSE_TRACE`**: `u32 tag · u32 seq · u16 n · u16 flags · f32[4n] quats ·
f32[n] dt_us`. At ~16 samples/frame that is ~340 B/frame → **~10 kB/s**. The client plays the trace
back on its own clock, sampling the sub-frame track by wall time — so at 144 fps the motion is
**measured, not invented**, and the comet trail becomes a true 480 Hz ribbon instead of a chord-per-
sample polyline. Hand tumbling is precisely the fast-motion regime where 30 Hz sampling smears.

**Recommended phasing: slerp in Phase 1, the trace in Phase 3.** Not because the trace is hard, but
because the owner's complaint is about *legibility*, and legibility is independent of pose rate. Ship
the thing that answers the complaint, then make the motion honest. The trace also carries real
prerequisites — a new `ImuFusion` API + tests, the gyro/gravity sign conventions verified on-rig, and
a check that stream 11 is actually enabled on the connected device — none of which should gate the
map, the guidance, or the progress gauge.

Note what the trace does **not** improve: the magnetometer is 30 Hz regardless, so the head dot and
its cell assignment stay 30 Hz. It is the shell/device attitude and the trail that go high-rate —
which is one more argument for the Steering widget existing at all, since that is the view where high-
rate attitude does visible work.

### 8.3 Renderer: a second WebGL context, with a 2D fallback

**Recommendation: a second `THREE.WebGLRenderer` on its own canvas, in a new `magcal3d.js` module,
using the vendored three.js (r160) already in `static/vendor/three` — no new dependency.**

Justification is legibility and simplicity, not frame budget:

- The view needs its own camera, its own world-up, transparency with depth sorting, and instanced
  geometry. Sharing `scene.js`'s single context would mean either scissor/viewport gymnastics against
  a DOM modal (couples the modal's layout to the WebGL canvas — fragile) or `readRenderTargetPixels`
  readback per frame (a needless GPU→CPU stall).
- `slam.js`'s precedent (render into `scene.js`'s context via a passed handle) is right *for slam.js*,
  which draws into the same world with the same camera. This view does neither.
- Two contexts is a non-issue: three.js supports it, and it was verified to work even on the software
  path (appendix A: three extra `webgl2` contexts created without refusal under SwiftShader).

Scene budget: one `InstancedMesh` (92 cell discs) + one instanced outline pass + one `BufferGeometry`
line for the comet + a ~50-tri device model + a handful of arrow/arc lines ≈ **6 draw calls, < 2 k
triangles**. Update per frame is one quaternion on a group and one `setDrawRange` on the comet — no
per-frame allocation, no geometry rebuild. Cell instance colours/scales update on the 5 Hz report
tick, not per frame.

Lifecycle: build lazily on first `open`; stop the rAF loop on `close` (keep the context, so reopening
is instant); `dispose()` on `webglcontextlost` → fall back (below). While the modal is open it fully
occludes the main scene, so add a small additive `scene.setRenderActive(false)` handle and call it —
free, and it stops burning GPU on an invisible view.

### 8.4 Performance argument

The owner's browser is GPU-accelerated and runs the main app at 144 fps; a 6-draw-call, 2 k-triangle
scene is not a meaningful load next to the existing 300 k-point cloud. The binding constraint is
**data rate, not draw cost**, and §8.1/§8.2 address it. No frame-budget contortions are warranted and
none are made.

For the *headless verification* path (SwiftShader, software rasterization) the numbers matter only as
a "does it still work" floor, and they are in appendix A: the fallback 2D renderer sustains 51 fps for
two 360 px views there, and the app page itself caps around 19–20 fps with an *empty* rAF callback —
i.e. under SwiftShader the harness is the bottleneck, not this view. That is a robustness note, not a
design constraint.

### 8.5 Graceful degradation (robustness requirement)

Three failure paths, one destination:

| trigger | behaviour |
|---|---|
| `canvas.getContext('webgl2')` returns null, or three.js throws on construct | fall back to the 2D renderer, log via `window.__diag`, show a one-line note "3D unavailable — showing the flat coverage map" |
| `webglcontextlost` event | same, and do not attempt automatic restore |
| `have_quat` false (ToF-only session, or before the first stream-9 sample) | the **hero still renders** (§3.2 point 4); the **Steering widget** shows a "no orientation — connect the IMU" placeholder instead of a tumbling model |

The fallback is **the existing `magcal.js` Lambert disc pair, kept alive and unmodified** — not a
degraded stub. It needs no orientation, no WebGL, and is already CVD-validated. `magcal.js` keeps
owning the report/quality/actions DOM and delegates *drawing* to whichever renderer is live.

**How a screenshot check confirms the view renders.** Follow `docs/web-ui-testing.md`:

1. `magcal3d.js` publishes a diag line each second:
   `window.__diag('magcal3d: renderer=webgl cells=92 covered=61 frames=143 pose_hz=29.8')`, so
   `web_ui_shot.py`'s `#diag-log` tail is a machine-checkable assertion that the WebGL path *and* the
   pose channel are live — not merely that a canvas exists.
2. Expose `window.__magcal3d = {renderer, frames, cells, covered, lastPoseMs}` for `Runtime.evaluate`.
3. A `?magcal2d=1` query parameter forces the fallback, so **both** paths get screenshotted in one
   run. Fixing the pose from a replay capture makes the shots deterministic enough to eyeball.
4. Screenshot both, `Read` the PNGs, and check the shell/ghost/gauge are drawn — the validator checks
   colour, never layout (dataviz procedure step 7).

## 9. Colour and accessibility

Consulted the `dataviz` skill before choosing any encoding. The governing decision:

> **Hue means exactly one thing in this view: |B| deviation. Everything else is ink, shape, size, or
> motion.**

That is why there is no RGB axis triad on the device model (it would put red and blue on screen
meaning "axis" while red and blue already mean "field too strong / too weak"), why the target
highlight is a double ring + pulse rather than a colour, and why the trail is neutral.

| role | job | encoding | validator result (dark, surface `#16181e`) |
|---|---|---|---|
| |B| deviation | diverging | existing `#60a5fa` ↔ neutral `#64748b` ↔ `#ef4444`, clamped ±30 % | all-pairs CVD ΔE **10.4** (protan), normal-vision ΔE **19.0**, contrast all ≥ 3:1 — **pass**. (Lightness-band / chroma-floor FAILs are the categorical checks; the neutral midpoint is *required* for a diverging ramp — scope note: "categorical palettes only".) |
| trail recency | ordinal | neutral `#e2e8f0 → #b7c0cf → #8b94a3 → #5b6472` | **ALL CHECKS PASS** (`--ordinal`): monotone L, adjacent ΔL ≥ 0.06, light end 2.97:1, hue spread 5° |
| verdicts | status | existing `#10b981` / `#f59e0b` / `#ef4444` | all-pairs CVD ΔE **8.1** (deutan) — at the floor, **legal only with secondary encoding**. The existing `GOOD`/`MARGINAL`/`BAD` text label is that encoding; keep it, and add an icon (`●` / `▲` / `✕`). Never ship the badge as a bare colour swatch. |
| coverage | binary state | **shape**: filled disc vs dashed hollow ring, + **stipple** on missing | not colour |
| sample density | ordinal | **size**: disc radius 1 → 8 samples | not colour |
| next target | attention | **double ring + pulse + geodesic leader line** | not colour |
| anomalous sample | status | **glyph** `✕` + hollow outline, plus a text chip | not colour alone |

**Known collision to fix in Phase 2, flagged rather than hidden:** `#ef4444` currently means both
"critical verdict" and "|B| reads high". The dataviz rule is that status colours are reserved and
never double as a scale pole. They never appear on the same mark, so this is not urgent, but the
diverging warm pole should be re-stepped off the status red and re-validated.

Also carried over: `prefers-reduced-motion` disables the target-ring pulse and the auto-orbit, and
shortens the comet to a static head dot.

## 10. Failure and edge states

| state | detection | what the user sees |
|---|---|---|
| **No calibration saved yet** | `has_current` false → `binning: "provisional"` | Shell renders from the provisional hard-iron estimate, drawn with thin outlines and a `PROVISIONAL GEOMETRY` watermark + "cell positions will settle as the fit improves". Never silently present provisional geometry as truth. |
| **Too few samples to bin at all** | `binning: "raw"` | Shell greyed to a wireframe lattice with "not enough samples to place directions yet — keep tumbling"; the comet still draws (motion feedback is honest even when placement isn't). |
| **Degenerate fit** | `fit_error` set | Shell and coverage unaffected (they don't depend on the fit); banner the reason; `Save` stays disabled; guidance switches to "the fit needs samples spread across more of the sphere — {N} cells in {M} regions still empty". |
| **Device stationary** | `roomscan.motion.coherence` over the recent pose window, or gyro-norm from stream 11 | After ~2 s: a `STATIONARY` chip and "the sphere only fills while the device turns". Samples keep being binned (they are real) but the **cell radius stops growing** past its density cap, so a stationary device cannot inflate one cell into apparent coverage. This is the sibling of the recorded trap where 255 stationary samples scored `std_pct 0.22 %` at a ×2.04 bias. |
| **Anomalous samples** (ferrous object, magnet) | \|B\| deviating > 3σ from the running median, reusing `YawFusion.anomaly_frac`'s notion rather than a new one | Head dot renders as `✕`, the radial tick pins to the clamp, a chip counts them: `12 anomalous samples — move away from metal`. Still binned and still fitted (rejecting them silently would be a hidden policy), but visibly marked. Also fires when the dip arc (§4.3) departs its running median. |
| **No orientation (ToF-only / pre-stream-9)** | `have_quat` false | Hero unaffected; Steering widget shows a placeholder. |
| **WebGL unavailable / context lost** | §8.5 | 2D Lambert disc fallback. |
| **Modal open, no device** (replay with no stream 10) | no env samples arriving | "no magnetometer data on this source" — the modal is a diagnostic; it must say *nothing is arriving* rather than render a convincingly empty sphere. |

## 11. Phased implementation plan

### Phase 1 — the minimum that answers the owner's ask

Everything here is required for the "stop feeling like a crazy person" outcome; nothing here is
polish.

1. `MAGPOSE` binary tag 5 (§7.1) at 30 Hz to `magcal_clients`; `ws.js` demux; `web.py` packer;
   `docs/web-protocol.md` row (via the `protocol-change` skill).
2. `magcal` JSON: drop `cell_dirs` after `open`, 4 → 5 Hz, add `guidance_axis`, `live_fit`,
   `stationary`, `t_world_to_cv`.
3. New `magcal3d.js` (10th ES module): second WebGL context, body-fixed hero — shell of 92 instanced
   cells (fill/outline + radius + diverging fill), device model, comet trail (3 s), B and g arrows,
   target ring + geodesic. **rAF render + slerp** between 30 Hz poses.
4. Steering widget (world-fixed camera, ghost device + curved wrist arrow) from the same scene.
5. Guidance rewrite in `magsweep.py`: `guidance_axis` from `n = unit(t × d)`, `θ = acos(t·d)`;
   region-weighted target selection; **remove the northern-hemisphere dip assumption**.
6. Rolling provisional fit + coverage ring gauge with the 60 %/85 % ticks + the four verdict chips.
7. Fallback path: `?magcal2d=1`, context-loss handling, `have_quat=false` handling, diag hooks.
8. Tests: extend `test_magcal_preview_does_not_touch_display_path` over the pose channel; golden
   `MAGPOSE` byte-layout vector; unit tests for the guidance rotation (round-trip: applying `ΔR`
   moves `d` onto `t` to < 1e-6); headless screenshots of both renderers.

### Phase 2 — the diagnostics that make it a instrument

9. Dip arc `B∠g` + live spread + 60 s sparkline, and `dip_deg` in `magsweep.field_consistency`'s
   sibling. This is a new, scale-immune calibration check (§4.3) — arguably the most valuable single
   item in this document, deferred only because Phase 1 must land the picture first.
10. Anomalous-sample detection + `✕` glyphs + chip.
11. Re-step the diverging warm pole off the status red; re-run `validate_palette.js` (§9).
12. `Show full path` post-mortem ribbon; click-to-swap hero/widget; per-cell hover tooltip in 3D
    (raycast against the `InstancedMesh`).

### Phase 3 — honest high-rate motion

13. `ImuFusion.update(..., trace=out)` (additive) + a **private** `ImuFusion` owned by the magcal
    session (§8.2); on-rig verification of gyro/gravity sign conventions.
14. `MAGPOSE_TRACE` binary tag 6 at ~480 Hz-equivalent; client plays the sub-frame track back on wall
    time; comet becomes a true 480 Hz ribbon.
15. On-rig A/B: tumble the same motion with slerp vs trace, screenshot the comet, and record the
    difference in `docs/iks4a1-stacking.md` next to the orientation-noise pass.

**Acceptance for Phase 1** is a human one, and it should be stated as such: with the modal open and
the board in hand, the owner can (i) name the direction of the largest remaining gap without
hesitating, (ii) reach it by following the arrow without being told where north is, (iii) watch the
gap counter and the coverage ring move as they do it, and (iv) know when to stop without pressing
`Stop & Fit` to find out.

## 12. Open questions for the owner

1. **How much does the "unrolled map" matter to you?** The design keeps the 2D Lambert disc pair only
   as a fallback. It is genuinely the best view for *counting* what is left. Should it be promoted to
   a third always-visible panel (a small strip under the hero), or is that clutter?
2. **Do you want a guided sequence, or a free tumble?** Everything above steers you to the largest
   remaining gap one at a time. A stronger version prescribes a fixed 6-pose recipe ("hold Top up,
   spin slowly; now Right up, spin; …") which guarantees coverage but feels like a wizard. The
   current design is free-form with steering — confirm that is what you want.
3. **Auto-orbit on the hero?** A slow 6°/s camera drift makes the sphere read as a sphere rather than
   a disc of dots, but it costs the "the hole is where it was" stability the whole framing rests on.
   Default off; want a toggle?
4. **Is stream 11 enabled on the device you'll calibrate with?** Phase 3 assumes it; if the connected
   firmware has it off, the gravity arrow silently falls back to the quaternion-derived vector (still
   correct, ~16× coarser in tilt) and the trace phase is moot.
5. **Should anomalous samples be excluded from the fit, or only flagged?** This design flags and
   keeps them (silent rejection is a hidden policy). Excluding them would make a fit near a metal desk
   much better and would also make the tool lie about what it measured.

---

## Appendix A — measurements taken while writing this

All on this host, `host/tools/web_ui_shot.py` (headless Chrome, `--use-angle=swiftshader`), and
`host/.venv`. **These characterize the software-rendering verification path, not the owner's
GPU-accelerated browser.**

**Fallback-renderer cost (standalone spike page, `/tmp/magspike/spike3.html`):**

| configuration | fps |
|---|---|
| twin 360 px 2D views, cells as pre-rendered **sprite atlas** (`drawImage`) | **51.2** |
| twin 360 px 2D views, cells as per-mark `arc` + `fill` + `stroke` | 22.1 |
| single 360 px 2D view, sprite atlas | 58.0 (rAF-capped ≈ 60) |

Same JS work in both sprite/arc rows (JS-side timers reported 0.1–0.35 ms/frame for *both*) — the
2.3× difference is entirely rasterizer state-flush cost from per-mark `globalAlpha`/`setLineDash`/
`fillStyle` changes. **Measurement trap worth recording: the in-page `performance.now()` timers
report the sprite path as *slower* while its frame rate is 2.3× higher, because Canvas2D defers.
Trust fps, not the timers.**

**The live app page under SwiftShader:** whole-page rAF ceiling is **19–20 fps with an empty
callback**, before any magcal drawing exists. Throttling the three.js render 5× (`view_fps` 20 → 4),
hiding `#canvas-container`, and forcing `WEBGL_lose_context` each moved it only to ~17. So under the
headless harness the ceiling is the harness, and no rendering choice in this document can be
evaluated there. This is why §8.4 argues on draw-call count and data rate rather than on measured fps.

**WebGL context availability under SwiftShader:** three additional `webgl2` contexts created
successfully (`ok/ok/ok`) alongside the app's — a second context is safe even on the software path.

**Server-side costs:** `magcal` report = **4490 B** (1200 samples), **1982 B** without `cell_dirs`.
`fit_ellipsoid` = 0.23 / 0.29 / 0.90 / 3.18 ms at 300 / 1200 / 5000 / 20 000 samples.

**Palette validation** (`dataviz/scripts/validate_palette.js`, `--mode dark --surface #16181e`):
diverging poles+midpoint all-pairs CVD ΔE 10.4 / normal 19.0 / contrast pass; ordinal neutral trail
ramp all checks pass; status triple CVD ΔE 8.1 (floor — secondary encoding required, and present).

## Appendix B — rejected alternatives, with reasons

| rejected | why |
|---|---|
| World-fixed camera as the hero | holes orbit at hand speed; you can see *that* there is a gap and never *where*, nor whether it is shrinking (§3.2) |
| Free-orbit camera on the hero | destroys the "the hole is where it was" property the framing rests on |
| Raising the `magcal` JSON to 30 Hz | 135 kB/s and 30 × `JSON.parse(4.5 kB)`/s of UI-thread work for data that changes at human speed; split by rate-of-change instead (§7.1) |
| Sharing `scene.js`'s WebGL context (scissor, or render-target readback) | couples the modal's DOM layout to the WebGL canvas, or adds a per-frame GPU→CPU stall; two contexts is a non-issue (§8.3) |
| 2D canvas as the *primary* renderer | was the safe choice under the wrong assumption that the budget was tight. It is a fine renderer (51 fps even on software) but it cannot draw a convincing device model, tangent discs, or depth-sorted transparency without hand-rolling a rasterizer. Kept as the fallback, where its orientation-independence is a feature |
| Attaching `ImuFusion` to `SensorState` to get high-rate pose | would flip a globally-off, SLAM-affecting filter on for a presentation feature. A private instance is non-regressing by construction (§8.2) |
| Extrapolating pose ahead of the last sample to hide latency | overshoots on direction reversals; this view's credibility rests on never showing motion that did not happen |
| A single opaque "calibration score" | is precisely what let BUG-030 ship; the components stay side by side (existing decision, reaffirmed) |
| Long full-session trail as the default | spaghetti. The covered cells already *are* the long-term history; the polyline's job is the derivative (§4.2) |
| RGB axis triad on the device model | puts red/blue on screen meaning "axis" while red/blue already mean "|B| high/low" (§9) |
| Text guidance of the form "point Top toward magnetic north and downward" | assumes northern-hemisphere dip and that the user knows where north is; the exact body-axis rotation is computable and needs neither (§5) |

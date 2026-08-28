# E1 — Python domain newtypes + Pyright strict vs. the historical bug corpus

Issue #223 · spec `docs/superpowers/specs/2026-08-27-rust-migration-feasibility.md`
("Compile-time correctness payoff", experiment E1) · run 2026-08-27 · pyright 1.1.411.

**Hypothesis:** `NewType` domain types + Pyright catch the historically-shipped
invariant bugs. **Pass threshold ≥ 8/12 rejected; kill < 6/12.**

## Result: 8/12 rejected — PASS (at the threshold)

Reproduce from `host/`: `python tools/spikes/e1_bug_patches/run_all.py`
(each patch is a few lines reintroducing the defect against the real modules).

| Bug | Defect reintroduced | Pyright verdict | Message (first line) |
|---|---|---|---|
| BUG-039 | `graft_yaw(q, gain * (quat_yaw_deg(a) - quat_yaw_deg(b)))` — Euler-yaw difference as the heading error | **REJECTED** | `Argument of type "float" cannot be assigned to parameter "delta_deg" of type "Twist" in function "graft_yaw"` |
| BUG-048 | `absolute_heading` strips ZYX yaw (`graft_yaw(q, -quat_yaw_deg(q))`) then returns a tilt-compensated azimuth as the bearing | **REJECTED** (2) | `Argument of type "float" cannot be assigned to parameter "delta_deg" of type "Twist"`; `Type "float" is not assignable to return type "Bearing"` |
| BUG-051 | `quat_yaw_deg(q)` fed to `YawFusion._absorb_north_bearing` | **REJECTED** | `Argument of type "EulerYaw" cannot be assigned to parameter "target" of type "Bearing" in function "_absorb_north_bearing"` |
| BUG-058 | `tilt_compensated_heading(q, m) - yaw_twist_deg(q)` fed to the same consumer; `graft_yaw(q, -yaw_twist_deg(q))` | **REJECTED** (2) | `Argument of type "float" cannot be assigned to parameter "target" of type "Bearing"`; `... parameter "delta_deg" of type "Twist"` |
| BUG-050 | `elapsed_s(WallTime, ctrl._record_started)` — `time.time()` against a `time.monotonic()` stamp | **REJECTED** | `Argument of type "WallTime" cannot be assigned to parameter "now" of type "HostMono" in function "elapsed_s"` |
| BUG-055 | unitless shell constant returned as `Metres`; `Metres` passed to `Deprojector(max_range_mm=)`; `Millimetres` passed to `TsdfMap(depth_max=)` | **REJECTED** (3) | `Type "float" is not assignable to return type "Metres"`; `Argument of type "Metres" cannot be assigned to parameter "max_range_mm" of type "Millimetres"`; `Argument of type "Millimetres" cannot be assigned to parameter "depth_max" of type "Metres"` |
| BUG-026 | body-frame cloud returned as world-frame | accepted — **not encoded** | frame-tagged arrays (spec item #4) need a generic ndarray wrapper; out of E1 scope |
| BUG-082 | `abs(cur - prev)` on two `Bearing`s (359 → 0 read as 359°) | accepted | `Bearing - Bearing` is a plain `float`; Python cannot forbid `__sub__` on a `NewType`. Only `bearing_delta` discipline guards it (spec predicted this) |
| BUG-083 | `EnvSample.from_wire(payload).pressure_pa` without unwrapping the `None` sentinel | **REJECTED** | `"pressure_pa" is not a known attribute of "None"` |
| BUG-044 | `_state_message(ui)` omitting the optional context args | accepted | defaulted parameters make the omission legal; no type system with default args flags it (Rust-only, as the spec said) |
| BUG-062 | `Mapper(54, 42, icp_mode=cfg.icp_mode, voxel_size=cfg.voxel_size)` — hand-picked subset of `SlamConfig` | accepted | kwarg subsets are legal; no field-forwarding exhaustiveness in Pyright |
| BUG-071 | `resolve_command` match over `CmdName` missing the `STANDBY` arm | **REJECTED** | `Argument of type "Literal[CmdName.STANDBY]" cannot be assigned to parameter "arg" of type "Never" in function "assert_never"` |

Total: **8 rejected / 4 accepted.** Of the 4 accepted, one (BUG-026) was not
attempted, two (BUG-044, BUG-062) are the "no default args / exhaustive struct
destructuring" class the spec already attributed to Rust only, and one (BUG-082)
is the arithmetic-erasure limit of `NewType`.

## Two honest caveats about the 8

* **Escape hatches remain.** `NewType` is erased and arithmetic on it yields the
  base type, so `now - ctrl._record_started` (BUG-050) and `decode_env(payload)[0]`
  (BUG-083) are still accepted; the rejections above fire only when the code goes
  through the typed helper (`elapsed_s`, `EnvSample.from_wire`) or the typed
  consumer (`graft_yaw`, `_absorb_north_bearing`). The guard is therefore "every
  *consumer* of a bearing / clock / length demands the newtype", not "the wrong
  value cannot exist". `bug050.py` and `bug083.py` carry the accepted escape line
  so the limit is visible in the corpus.
* **The rejection lands where the type is demanded, which is one call away from
  the mistake.** BUG-039's error is on `graft_yaw`, not on the `quat_yaw_deg`
  subtraction. That is still the right line to read, but a `Twist(...)` cast on
  it would launder the value — the cast is the review point.

## What was typed (all behaviour-neutral; NewType is erased at runtime)

* `host/src/roomscan/domain.py` (new, strict): `McuMicros`, `LsmTicks`
  (**float**, not int as first proposed — stream 13's frame-ready instant is a
  fractional tick), `HostMono`, `WallTime`, `now_mono/now_wall/elapsed_s/lsm_delta`;
  `Millimetres`, `Metres`, `MM_PER_M`, `mm_to_m/m_to_mm`; `Bearing`, `EulerYaw`,
  `Twist`, `bearing_delta` (wrap to (-180, 180]); frozen `EnvSample` with
  `from_values`/`from_wire -> EnvSample | None`; `IcpMode` (StrEnum), `WireVersion` (IntEnum).
* `sensors.py`: `quat_yaw_deg -> EulerYaw`, `yaw_twist_deg -> Twist`,
  `graft_yaw_error_deg -> Twist`, `graft_yaw(delta_deg: Twist)`,
  `magnetic_north_bearing_deg / boresight_bearing_deg / absolute_heading -> Bearing | None`,
  `YawFusion._absorb_north_bearing(target: Bearing, dt)` split out as the typed consumer.
* `imufusion.py`: `_correct_yaw` grafts `Twist(gain * err)`.
* `sensor_time.py` (strict): `signed_tick_delta`, `TimestampedQuaternionBuffer.add/at`,
  `interpolate_quaternion_at` take `LsmTicks`; `slerp`/`_valid_quat` annotated.
* `protocol.py`: `SUPPORTED_VERSIONS = tuple(int(v) for v in WireVersion)`; `ImuSync.quat_offset_us` wraps its ticks.
* `server/session.py`: `_record_started: HostMono`, `session_message` uses
  `elapsed_s(now_mono(), ...)`; heading motion via `bearing_delta`; `resolve_command`
  now matches a `CmdName` StrEnum exhaustively with `assert_never`.
* `deproject.py` / `slam/tsdf.py`: `max_range_mm: Millimetres`, `depth_max: Metres`,
  `MM_PER_M` at the mm→m boundary. `slam/odometry.py`: mode validated via `IcpMode`.
* `panel.py`: two `graft_yaw` call sites wrap `Twist(...)`.

## Pyright status

* Strict set (`pyrightconfig.json` → `strict`): `domain.py`, `sensor_time.py` —
  **0 errors** (gated by `tests/test_pyright_strict.py`).
* `sensors.py`, `imufusion.py`, `protocol.py` are checked in **basic** mode, not
  strict: each has dozens of untyped `quat`/ndarray parameters and making them
  strict is a separate typing pass, not a few lines. Their newtype signatures
  are what the experiment relies on, and basic mode enforces those.
* Whole-package basic run: **327 errors before, 327 after** (same count as
  `main` — every error the new parameter types introduced was fixed at its
  caller; the remaining 327 are pre-existing and untouched).

## Tests

`tests/test_sensors.py test_protocol.py test_sensor_time.py test_imu_fusion.py
test_deproject.py test_slam_odometry.py test_slam_tsdf.py test_pyright_strict.py
test_protocol_sensors.py test_sensors_e2e.py`: 278 passed, 1 skipped (the
suite's known env skip). `tests/test_web.py`: 591 passed. `test_mcp_registry.py`,
`test_slam_config.py`: green. Ruff on the touched files: 34 findings, all
pre-existing unused imports in `server/session.py` (same 34 on `main`).

## Verdict for the spec

Pass, at exactly the threshold. The pay-off is concentrated in the four
yaw/bearing/twist bugs, the clock split, the unit split and the sentinel — the
six classes the spec ranked (1)–(3) and (6). The four it cannot catch are the
ones the spec already assigned to Rust (no default args, exhaustive struct
destructuring) or to discipline (angle subtraction, frame tags). Recommendation:
adopt — extend the strict list module by module, and make new heading / clock /
length consumers demand the newtype.

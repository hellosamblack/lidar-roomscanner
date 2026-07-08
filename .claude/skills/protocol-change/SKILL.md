---
name: protocol-change
description: Use when changing the binary wire protocol in any way — new field, new stream_id, new frame_type, size/order change, CRC or encoding change. Checklist that keeps spec, firmware C, host Python, and golden vectors in lockstep.
---

# Wire-protocol change checklist

The protocol has four synchronized artifacts. A change lands in **one commit** touching all of them:

1. **Spec** — `docs/protocol.md` (layout tables, semantics, version history section).
2. **Firmware encoder** — `firmware/scanner-stream/Src/rs_protocol.h` / `.c`.
3. **Host decoder** — `host/src/roomscan/protocol.py` (+ `decoder.py` if framing changed).
4. **Golden vectors** — `host/tests/fixtures/*.bin` + the tests that assert exact bytes.

## Rules

- **Layout change ⇒ version bump.** Any change to header size, field order/width, payload encoding, or
  CRC coverage increments `RS_PROTO_VERSION`. Adding a new `stream_id` or `frame_type` value (no layout
  change) does NOT bump the version — decoders must already skip unknown IDs.
- CRC32 stays **last on the wire**, computed over everything before it (header + payload).
- Little-endian, always. New multi-byte fields get explicit offsets documented in the spec table.
- Never reuse a retired `stream_id`/`frame_type` value or repurpose a reserved field without a bump.
- Decoder compatibility: host must keep decoding version N-1 recordings (replay of old captures is a
  feature, not a courtesy) — gate parsing on the header's version byte.

## Procedure

1. Edit `docs/protocol.md`: update the layout table AND append a line to its "Version history" section.
2. Update the C encoder; keep it HAL-free (host-compilable).
3. Update the Python side; regenerate golden vectors with the fixture generator
   (`host/tests/make_fixtures.py`), which builds the bytes independently of `protocol.py`'s parser.
4. `pytest host/tests` — the cross-check tests (parse(golden) == known fields, encode(known) == golden)
   must pass.
5. Flash + run the live viewer against real hardware before declaring done (`firmware-loop` skill):
   0 CRC failures, 0 seq gaps.
6. One commit: `feat(protocol): vN — <what changed>`.

## Red flags

- "I'll update the Python side later" — no; the golden vectors exist to make that impossible to forget.
- Casting a packed C struct straight onto the wire without re-checking the golden bytes after adding a
  field (alignment padding sneaks in silently).
- Bumping the version in code but not in the spec's version-history table (or vice versa).

// app.js — thin composition root.
//
// Constructs the ws.js hub (the single source of truth for server-known state),
// instantiates every feature module against it, then opens the socket. It wires
// nothing else: no rendering, no message parsing, no DOM event handling of its
// own — each module owns its concern and communicates only through the hub
// (§8.3). Reaching here means the ES module graph + `three` import resolved.

import { createHub } from './ws.js';
import { createScene } from './scene.js';
import { createIr } from './ir.js';
import { createHud } from './hud.js';
import { createLog } from './log.js';
import { createControls } from './controls.js';
import { createSensors } from './sensors.js';
import { createMagcal } from './magcal.js';
import { createCapture } from './capture.js';
import { createSlam } from './slam.js';
import { createAdmin } from './admin.js';

const D = (m, l) => { try { window.__diag && window.__diag('app.js: ' + m, l); } catch (e) {} };
D('composition root loaded');

const hub = createHub();

// Order is immaterial — modules only ever talk through the hub — but construct
// receivers before opening the socket so no early message is missed.
createHud(hub);
createLog(hub);
createControls(hub);
createSensors(hub);
createCapture(hub);
createIr(hub);
// Top-bar maintenance actions. Talks to /api/* over fetch, not the hub — it
// takes `hub` only to watch 'conn' so the Restart button can clear its busy
// state when the socket comes back.
createAdmin(hub);
// scene.js returns a handle (Three.js context + follow-camera hooks); slam.js
// renders the SLAM mesh/trajectory into that same scene (web Phase 4).
const sceneApi = createScene(hub);
window.__scene = sceneApi;   // diagnostics only, see scene.js's `controls` comment
createSlam(hub, sceneApi);
// Magnetometer-calibration modal (opened from the Sensors card). It takes the
// scene handle only to PAUSE the main render while the modal occludes it — it
// never draws into scene.js's context (it owns its own; see magcal3d.js §8.3).
createMagcal(hub, sceneApi);

hub.connect();
D('all modules instantiated; socket connecting');

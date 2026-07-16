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
import { createCapture } from './capture.js';
import { createSlam } from './slam.js';

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
// scene.js returns a handle (Three.js context + follow-camera hooks); slam.js
// renders the SLAM mesh/trajectory into that same scene (web Phase 4).
const sceneApi = createScene(hub);
createSlam(hub, sceneApi);

hub.connect();
D('all modules instantiated; socket connecting');

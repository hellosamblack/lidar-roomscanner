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
import { createBrowser } from './browser.js';
import { createSlam } from './slam.js';
import { createSplat } from './splat.js';
import { createAdmin } from './admin.js';
import { createIdle } from './idle.js';
import { createSpotlight } from './spotlight.js';

const D = (m, l) => { try { window.__diag && window.__diag('app.js: ' + m, l); } catch (e) {} };
D('composition root loaded');

const hub = createHub();

// Order is immaterial — modules only ever talk through the hub — but construct
// receivers before opening the socket so no early message is missed.
createHud(hub);
createLog(hub);
createControls(hub);
createSensors(hub);
const sceneApi = createScene(hub);
window.__scene = sceneApi;   // diagnostics only, see scene.js's `controls` comment

// browser.js takes capture.js's handle so the two share ONE rename dialog
// (§12) instead of growing a second copy that drifts from it.
const captureApi = createCapture(hub);
createBrowser(hub, captureApi, sceneApi);
createIr(hub);
// Top-bar maintenance actions. Talks to /api/* over fetch, not the hub — it
// takes `hub` only to watch 'conn' so the Restart button can clear its busy
// state when the socket comes back.
createAdmin(hub);
createSlam(hub, sceneApi);
// Splat source (Live/View/Splat): renders an offline Gaussian-splat
// reconstruction into the SAME scene, drawn by the single render loop.
window.__splat = createSplat(hub, sceneApi);   // diagnostics only (see scene.js `controls`)
// Magnetometer-calibration modal (opened from the Sensors card). It takes the
// scene handle only to PAUSE the main render while the modal occludes it — it
// never draws into scene.js's context (it owns its own; see magcal3d.js §8.3).
createMagcal(hub, sceneApi);
// Parks the tab (stops rendering, tells the server this connection isn't
// actively engaged) after a period of no activity. Takes the scene handle for
// the same reason magcal.js does -- pausing render while occluded/unused is
// pure waste -- see idle.js's own comment for the shared-flag interaction.
window.__idle = createIdle(hub, sceneApi);   // diagnostics only, see scene.js's `controls` comment
// Presentation-only: cursor-follow edge highlight on the chrome cards. No hub,
// no server state — pure DOM, so it's constructed last and wired to nothing.
createSpotlight();

hub.connect();
D('all modules instantiated; socket connecting');

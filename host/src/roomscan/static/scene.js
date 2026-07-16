// scene.js — Three.js scene / camera / OrbitControls / point-cloud geometry.
//
// Extracted verbatim (in behaviour) from the old monolithic app.js: same camera
// pose (0.5,0,-1.5), y-down Open3D CV up vector, Z-forward grid, MAX_POINTS,
// PointsMaterial. Subscribes to "point_cloud" and parses the tag+positions+colors
// layout itself (§6.1). Owns the requestAnimationFrame render loop, measures its
// own VIEW fps (browser paint rate) and publishes it on the hub (~1/s) — this is
// distinct from the device fps the server reports.
//
// Public surface:
//   createScene(hub) -> { resetCamera, THREE, scene, camera,
//                         setPointsVisible(bool), setFollow(bool),
//                         setFollowTarget(eye, center, up) }
// slam.js (web Phase 4) uses the returned handle to add its mesh/trajectory
// group to the same scene and to drive the follow camera (which must coordinate
// with OrbitControls — only one may own the camera per frame).
// Hub events:  subscribes "point_cloud", "reset_camera";  emits "view_fps" (~1/s)

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const D = (m, l) => { try { window.__diag && window.__diag('scene.js: ' + m, l); } catch (e) {} };

const MAX_POINTS = 300000;                 // large buffer for later SLAM maps
const CAM_POS = new THREE.Vector3(0.5, 0, -1.5);
const CAM_TARGET = new THREE.Vector3(0, 0, 1);

export function createScene(hub) {
    D('module loaded; THREE r' + THREE.REVISION);

    const container = document.getElementById('canvas-container');
    if (!container) { D('FATAL #canvas-container not found — scene cannot attach', 'error'); return { resetCamera() {} }; }

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a0f);
    scene.fog = new THREE.FogExp2(0x0a0a0f, 0.1);

    const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 100);
    camera.position.copy(CAM_POS);
    camera.up.set(0, -1, 0);               // Open3D CV convention, y-down

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.target.copy(CAM_TARGET);

    // Subtle grid, oriented to the XY plane for the Z-forward convention.
    const gridHelper = new THREE.GridHelper(10, 20, 0x333333, 0x1a1a1a);
    gridHelper.rotation.x = Math.PI / 2;
    scene.add(gridHelper);

    // Point cloud — position + color attributes, draw range grown per frame.
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
    geometry.setDrawRange(0, 0);
    const material = new THREE.PointsMaterial({ size: 0.025, vertexColors: true, sizeAttenuation: true });
    const points = new THREE.Points(geometry, material);
    scene.add(points);

    // --- SLAM follow camera (web Phase 4) ---------------------------------
    // When follow is on, slam.js pushes an eye/center/up each frame and this
    // loop lerps the camera to it with OrbitControls disabled; when off,
    // OrbitControls owns the camera again. Smoothing mirrors the desktop's
    // _apply_follow_camera (steady when near-stationary, snappy under motion).
    let followOn = false;
    const followEye = new THREE.Vector3();
    const followCenter = new THREE.Vector3();
    const followUp = new THREE.Vector3(0, -1, 0);
    let haveFollowTarget = false;
    function setPointsVisible(v) { points.visible = v; }
    function setFollow(on) {
        followOn = !!on;
        if (!followOn) { controls.enabled = true; }
    }
    function setFollowTarget(eye, center, up) {
        followEye.set(eye[0], eye[1], eye[2]);
        followCenter.set(center[0], center[1], center[2]);
        if (up) followUp.set(up[0], up[1], up[2]);
        haveFollowTarget = true;
    }

    // --- point cloud ingest (§6.1: u32 tag · f32[3N] positions · f32[3N] colors) ---
    hub.on('point_cloud', (buffer) => {
        // Skip the 4-byte tag header; the rest is 6 floats per point.
        const data = new Float32Array(buffer, 4);
        let numPoints = Math.floor(data.length / 6);
        if (numPoints > MAX_POINTS) numPoints = MAX_POINTS;   // clamp to buffer
        if (!window.__gotFrame) { window.__gotFrame = true; D('first point cloud: ' + numPoints + ' pts'); }

        const positions = geometry.attributes.position.array;
        const colors = geometry.attributes.color.array;
        const colorOffset = Math.floor(data.length / 6) * 3;  // colors follow ALL positions in the wire buffer
        const n3 = numPoints * 3;
        for (let i = 0; i < n3; i++) {
            positions[i] = data[i];
            colors[i] = data[colorOffset + i];
        }
        geometry.attributes.position.needsUpdate = true;
        geometry.attributes.color.needsUpdate = true;
        geometry.setDrawRange(0, numPoints);
    });

    function resetCamera() {
        camera.position.copy(CAM_POS);
        controls.target.copy(CAM_TARGET);
        controls.update();
    }
    hub.on('reset_camera', resetCamera);

    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });

    // Render loop + VIEW-fps measurement (browser paint rate, published ~1/s).
    let framesRendered = 0;
    let lastFpsTime = performance.now();
    function animate() {
        requestAnimationFrame(animate);
        if (followOn && haveFollowTarget) {
            controls.enabled = false;
            // Velocity-adaptive lerp: fast when the sensor moves, steady when still.
            const d = camera.position.distanceTo(followEye);
            const alpha = Math.min(1, Math.max(0.12, d / 0.03));
            camera.position.lerp(followEye, alpha);
            controls.target.lerp(followCenter, alpha);
            camera.up.copy(followUp);
            camera.lookAt(controls.target);
        } else {
            controls.update();
        }
        renderer.render(scene, camera);

        framesRendered++;
        const now = performance.now();
        if (now - lastFpsTime >= 1000) {
            hub.emit('view_fps', framesRendered);
            framesRendered = 0;
            lastFpsTime = now;
        }
    }
    animate();

    return { resetCamera, THREE, scene, camera, setPointsVisible, setFollow, setFollowTarget };
}

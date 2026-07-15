import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ---- Scene Setup ----
const container = document.getElementById('canvas-container');
const scene = new THREE.Scene();
// Dark gradient background mimicking the native app
scene.background = new THREE.Color(0x0a0a0f);
// Optional: Add fog for depth
scene.fog = new THREE.FogExp2(0x0a0a0f, 0.1);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 100);
// Position camera to look down the Z axis like the sensor does
camera.position.set(0.5, 0, -1.5);
camera.up.set(0, -1, 0); // Open3D CV convention y-down

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);
container.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.05;
controls.target.set(0, 0, 1);

// Add a subtle grid to ground the space
const gridHelper = new THREE.GridHelper(10, 20, 0x333333, 0x1a1a1a);
gridHelper.rotation.x = Math.PI / 2; // Orient to XY plane for Z-forward convention
scene.add(gridHelper);

// ---- Point Cloud Setup ----
const MAX_POINTS = 300000; // Large buffer for SLAM maps
const geometry = new THREE.BufferGeometry();
// Initialize with empty buffers
geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
geometry.setDrawRange(0, 0);

const material = new THREE.PointsMaterial({
    size: 0.025,
    vertexColors: true,
    sizeAttenuation: true
});

const pointCloud = new THREE.Points(geometry, material);
scene.add(pointCloud);

// ---- Metrics & State ----
let framesRendered = 0;
let lastTime = performance.now();
const fpsEl = document.getElementById('fps-val');
const ptsEl = document.getElementById('pts-val');

// ---- WebSocket ----
let ws = null;
const connText = document.getElementById('conn-text');
const connDot = document.getElementById('conn-dot');

function connect() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    ws = new WebSocket(`${protocol}//${window.location.host}/ws`);
    ws.binaryType = 'arraybuffer';

    ws.onopen = () => {
        connText.textContent = 'Live';
        connDot.classList.add('connected');
    };

    ws.onclose = () => {
        connText.textContent = 'Offline';
        connDot.classList.remove('connected');
        // Reconnect after 2s
        setTimeout(connect, 2000);
    };

    ws.onerror = (err) => {
        console.error('WebSocket Error:', err);
    };

    ws.onmessage = (event) => {
        // Text frames are status/error messages (JSON); binary frames are
        // point data. Without this branch a server error string would be fed
        // into Float32Array as garbage and silently render nothing.
        if (typeof event.data === 'string') {
            try {
                const msg = JSON.parse(event.data);
                if (msg.type === 'error') {
                    connText.textContent = 'Error: ' + msg.message;
                    connDot.classList.remove('connected');
                    console.error('Server error:', msg.message);
                }
            } catch (e) { /* ignore non-JSON text */ }
            return;
        }
        // Payload is Float32Array: [x,y,z,x,y,z... r,g,b,r,g,b...]
        const data = new Float32Array(event.data);
        const numPoints = data.length / 6;
        
        ptsEl.textContent = numPoints.toLocaleString();

        const positions = geometry.attributes.position.array;
        const colors = geometry.attributes.color.array;

        // The python server concatenates positions then colors
        const ptsOffset = 0;
        const colorOffset = numPoints * 3;

        for (let i = 0; i < numPoints * 3; i++) {
            positions[i] = data[ptsOffset + i];
            colors[i] = data[colorOffset + i];
        }

        geometry.attributes.position.needsUpdate = true;
        geometry.attributes.color.needsUpdate = true;
        geometry.setDrawRange(0, numPoints);
    };
}

connect();

// ---- UI Bindings ----
function sendCommand(cmdStr) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ cmd: cmdStr }));
    }
}

document.getElementById('btn-ping').onclick = () => sendCommand('ping');
document.getElementById('btn-calib').onclick = () => sendCommand('calib');
document.getElementById('btn-reinit').onclick = () => sendCommand('reinit');
document.getElementById('btn-uc0').onclick = () => sendCommand('usecase_0');
document.getElementById('btn-uc1').onclick = () => sendCommand('usecase_1');

document.getElementById('btn-reset-cam').onclick = () => {
    camera.position.set(0.5, 0, -1.5);
    controls.target.set(0, 0, 1);
    controls.update();
};

// ---- Render Loop ----
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);

    // FPS calculation
    framesRendered++;
    const now = performance.now();
    if (now - lastTime >= 1000) {
        fpsEl.textContent = framesRendered;
        framesRendered = 0;
        lastTime = now;
    }
}

animate();

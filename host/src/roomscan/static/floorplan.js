// floorplan.js — 2D Top-Down Floorplan view mode and interactive 3D/2D Measure Tool.
//
// Public surface:
//   createFloorplan(hub, sceneApi) -> {
//       setFloorplanMode(bool),
//       toggleFloorplanMode(),
//       setMeasureTool(bool),
//       toggleMeasureTool(),
//       clearMeasurement(),
//       getMeasureDistance(),
//       getBBox()
//   }
//
// Responsibilities:
// 1. Top-down Floorplan View:
//    Aligns camera into a top-down overhead view looking down +Y (Open3D CV
//    gravity vector) centered on current geometry bounding box.
// 2. Interactive Measure Tool:
//    Click point A, click point B to compute Euclidean distance in metres. Renders
//    endpoint markers, dimension line, and midpoint distance sprite in the 3D scene.
// 3. Estimated Room Dimensions:
//    Tracks bounding box extent (X width, Z depth, Y height) from live point clouds
//    and updates #floorplan-dimensions.

import * as THREE from 'three';

const D = (m, l) => { try { window.__diag && window.__diag('floorplan.js: ' + m, l); } catch (e) {} };

export function createFloorplan(hub, sceneApi) {
    D('module loaded');

    const $ = (id) => document.getElementById(id);
    const btnFloorplan = $('btn-view-floorplan');
    const btnMeasureTool = $('btn-measure-tool');
    const measureDistVal = $('measure-distance-val');
    const floorplanDimensionsEl = $('floorplan-dimensions');
    const measureStatusNote = $('measure-status-note');

    const { scene, camera, controls } = sceneApi || {};
    const container = document.getElementById('canvas-container');

    let floorplanActive = false;
    let measureActive = false;
    let currentBBox = null;

    // Measurement geometry group in 3D scene
    const measureGroup = new THREE.Group();
    measureGroup.name = 'floorplan_measure_group';
    if (scene) {
        scene.add(measureGroup);
    }

    // Measurement state:
    // 0 = idle / ready for Point A
    // 1 = Point A set, waiting for Point B
    // 2 = Point A & Point B set, measurement complete
    let measureState = 0;
    let pointA = null;
    let pointB = null;
    let measuredDistance = null;

    // Visual elements for measurement
    const sphereGeom = new THREE.SphereGeometry(0.035, 16, 16);
    const markerMatA = new THREE.MeshBasicMaterial({ color: 0x00ffff, depthTest: false });
    const markerMatB = new THREE.MeshBasicMaterial({ color: 0xffaa00, depthTest: false });
    const markerA = new THREE.Mesh(sphereGeom, markerMatA);
    const markerB = new THREE.Mesh(sphereGeom, markerMatB);
    markerA.visible = false;
    markerB.visible = false;
    measureGroup.add(markerA);
    measureGroup.add(markerB);

    const lineGeom = new THREE.BufferGeometry();
    lineGeom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
    const lineMat = new THREE.LineBasicMaterial({
        color: 0x00ffff,
        linewidth: 2,
        depthTest: false,
        transparent: true,
        opacity: 0.9,
    });
    const measureLine = new THREE.Line(lineGeom, lineMat);
    measureLine.visible = false;
    measureGroup.add(measureLine);

    let textSprite = null;

    function createDistanceSprite(text) {
        const canvas = document.createElement('canvas');
        canvas.width = 256;
        canvas.height = 64;
        const ctx = canvas.getContext('2d');
        if (!ctx) return null;

        ctx.fillStyle = 'rgba(15, 17, 26, 0.88)';
        if (ctx.roundRect) {
            ctx.beginPath();
            ctx.roundRect(4, 4, 248, 56, 8);
            ctx.fill();
        } else {
            ctx.fillRect(4, 4, 248, 56);
        }
        ctx.strokeStyle = '#00ffff';
        ctx.lineWidth = 2;
        ctx.stroke();

        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 22px monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(text, 128, 32);

        const texture = new THREE.CanvasTexture(canvas);
        const spriteMat = new THREE.SpriteMaterial({ map: texture, depthTest: false, transparent: true });
        const sprite = new THREE.Sprite(spriteMat);
        sprite.scale.set(0.65, 0.16, 1);
        return sprite;
    }

    function updateMeasureVisuals() {
        if (textSprite) {
            measureGroup.remove(textSprite);
            if (textSprite.material.map) textSprite.material.map.dispose();
            textSprite.material.dispose();
            textSprite = null;
        }

        if (measureState === 0) {
            markerA.visible = false;
            markerB.visible = false;
            measureLine.visible = false;
            if (measureDistVal) measureDistVal.textContent = '—';
            if (measureStatusNote) measureStatusNote.textContent = 'Click first point in scene to start measurement';
        } else if (measureState === 1 && pointA) {
            markerA.position.copy(pointA);
            markerA.visible = true;
            markerB.visible = false;
            measureLine.visible = false;
            if (measureDistVal) measureDistVal.textContent = '…';
            if (measureStatusNote) measureStatusNote.textContent = 'Point A set. Click second point to measure distance';
        } else if (measureState === 2 && pointA && pointB) {
            markerA.position.copy(pointA);
            markerB.position.copy(pointB);
            markerA.visible = true;
            markerB.visible = true;

            const pos = lineGeom.attributes.position.array;
            pos[0] = pointA.x; pos[1] = pointA.y; pos[2] = pointA.z;
            pos[3] = pointB.x; pos[4] = pointB.y; pos[5] = pointB.z;
            lineGeom.attributes.position.needsUpdate = true;
            measureLine.visible = true;

            const dist = pointA.distanceTo(pointB);
            measuredDistance = dist;
            const distText = `${dist.toFixed(3)} m`;
            if (measureDistVal) measureDistVal.textContent = distText;
            if (measureStatusNote) measureStatusNote.textContent = `Measured ${distText}. Click again to start new measurement`;

            textSprite = createDistanceSprite(distText);
            if (textSprite) {
                const mid = pointA.clone().add(pointB).multiplyScalar(0.5);
                textSprite.position.copy(mid);
                textSprite.position.y -= 0.08;
                measureGroup.add(textSprite);
            }
        }
    }

    function clearMeasurement() {
        measureState = 0;
        pointA = null;
        pointB = null;
        measuredDistance = null;
        updateMeasureVisuals();
    }

    function setMeasureTool(active) {
        measureActive = !!active;
        if (btnMeasureTool) {
            btnMeasureTool.classList.toggle('active', measureActive);
        }
        if (container) {
            container.style.cursor = measureActive ? 'crosshair' : '';
        }
        if (!measureActive) {
            clearMeasurement();
        } else {
            if (measureStatusNote) {
                measureStatusNote.textContent = 'Measure tool active: click two points to measure distance';
            }
        }
        D(`measure tool -> ${measureActive}`);
    }

    function toggleMeasureTool() {
        setMeasureTool(!measureActive);
    }

    // Raycasting for point selection on clicks
    const raycaster = new THREE.Raycaster();
    const mouseNdc = new THREE.Vector2();
    let ptrDownX = 0;
    let ptrDownY = 0;

    function onPointerDown(e) {
        ptrDownX = e.clientX;
        ptrDownY = e.clientY;
    }

    function onPointerUp(e) {
        if (!measureActive || !camera || !scene) return;
        // Ignore drag gestures used for orbit / pan
        const distMoved = Math.hypot(e.clientX - ptrDownX, e.clientY - ptrDownY);
        if (distMoved > 6) return;

        const rect = (container || document.body).getBoundingClientRect();
        mouseNdc.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        mouseNdc.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

        raycaster.setFromCamera(mouseNdc, camera);
        raycaster.params.Points = { threshold: 0.12 };

        // Raycast against scene objects (excluding measurement markers)
        const candidates = [];
        scene.traverse((obj) => {
            if (obj.visible && obj !== measureGroup && !measureGroup.children.includes(obj)) {
                if (obj.isPoints || obj.isMesh) {
                    candidates.push(obj);
                }
            }
        });

        const intersects = raycaster.intersectObjects(candidates, false);
        let pickedPoint = null;

        if (intersects.length > 0) {
            pickedPoint = intersects[0].point.clone();
        } else {
            // Fallback: intersect with horizontal plane at current target height
            const planeY = (controls && controls.target) ? controls.target.y : (currentBBox ? (currentBBox.minY + currentBBox.maxY) / 2 : 0);
            const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -planeY);
            const planeHit = new THREE.Vector3();
            if (raycaster.ray.intersectPlane(plane, planeHit)) {
                pickedPoint = planeHit;
            }
        }

        if (!pickedPoint) return;

        if (measureState === 0 || measureState === 2) {
            pointA = pickedPoint;
            pointB = null;
            measureState = 1;
            updateMeasureVisuals();
        } else if (measureState === 1) {
            pointB = pickedPoint;
            measureState = 2;
            updateMeasureVisuals();
        }
    }

    if (container) {
        container.addEventListener('pointerdown', onPointerDown);
        container.addEventListener('pointerup', onPointerUp);
    }

    // Top-down Floorplan view mode
    function setFloorplanMode(active) {
        floorplanActive = !!active;
        if (btnFloorplan) {
            btnFloorplan.classList.toggle('active', floorplanActive);
        }
        if (!floorplanActive) {
            D('floorplan mode off');
            return;
        }

        if (!camera || !controls) return;

        const targetX = currentBBox ? (currentBBox.minX + currentBBox.maxX) / 2 : (controls.target.x || 0);
        const targetY = currentBBox ? (currentBBox.minY + currentBBox.maxY) / 2 : (controls.target.y || 0);
        const targetZ = currentBBox ? (currentBBox.minZ + currentBBox.maxZ) / 2 : (controls.target.z || 1.0);

        const extentX = currentBBox ? Math.abs(currentBBox.maxX - currentBBox.minX) : 3.0;
        const extentZ = currentBBox ? Math.abs(currentBBox.maxZ - currentBBox.minZ) : 3.0;
        const maxExtent = Math.max(extentX, extentZ, 2.5);
        const distanceAbove = Math.max(3.5, maxExtent * 1.6);

        // In Open3D CV frame, -Y is UP and +Y is DOWN. Looking down along +Y from above:
        camera.position.set(targetX, targetY - distanceAbove, targetZ);
        camera.up.set(0, 0, -1);
        controls.target.set(targetX, targetY, targetZ);
        camera.lookAt(controls.target);
        controls.update();
        camera.updateProjectionMatrix();

        D(`floorplan view set: eye (${targetX.toFixed(2)}, ${(targetY - distanceAbove).toFixed(2)}, ${targetZ.toFixed(2)}) target (${targetX.toFixed(2)}, ${targetY.toFixed(2)}, ${targetZ.toFixed(2)})`);
    }

    function toggleFloorplanMode() {
        setFloorplanMode(!floorplanActive);
    }

    // Button event listeners
    btnFloorplan?.addEventListener('click', toggleFloorplanMode);
    btnMeasureTool?.addEventListener('click', toggleMeasureTool);

    // Update Room Dimensions readout
    function updateRoomDimensions() {
        if (!floorplanDimensionsEl) return;
        if (!currentBBox) {
            floorplanDimensionsEl.textContent = '—';
            return;
        }
        const width = Math.abs(currentBBox.maxX - currentBBox.minX);
        const depth = Math.abs(currentBBox.maxZ - currentBBox.minZ);
        const height = Math.abs(currentBBox.maxY - currentBBox.minY);
        floorplanDimensionsEl.textContent = `${width.toFixed(2)} m × ${depth.toFixed(2)} m (H: ${height.toFixed(2)} m)`;
    }

    // Hub event subscriptions
    hub?.on('point_cloud', (buffer) => {
        const data = new Float32Array(buffer, 4);
        const numPoints = Math.min(Math.floor(data.length / 6), 300000);
        if (numPoints <= 0) return;

        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;
        let minZ = Infinity, maxZ = -Infinity;

        for (let i = 0; i < numPoints; i++) {
            const x = data[i * 3];
            const y = data[i * 3 + 1];
            const z = data[i * 3 + 2];
            if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) {
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
                if (z < minZ) minZ = z;
                if (z > maxZ) maxZ = z;
            }
        }

        if (minX <= maxX && isFinite(minX)) {
            currentBBox = { minX, maxX, minY, maxY, minZ, maxZ };
            updateRoomDimensions();
        }
    });

    hub?.on('stream_reset', () => {
        currentBBox = null;
        updateRoomDimensions();
        clearMeasurement();
    });

    return {
        setFloorplanMode,
        toggleFloorplanMode,
        setMeasureTool,
        toggleMeasureTool,
        clearMeasurement,
        getMeasureDistance: () => measuredDistance,
        getBBox: () => currentBBox,
    };
}

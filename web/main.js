import * as THREE from './vendor/three.module.js';
import { OrbitControls } from './vendor/jsm/controls/OrbitControls.js';

// ===== DOM References =====
const canvas = document.getElementById('viewport');
const connStatus = document.getElementById('conn-status');
const tfStatus = document.getElementById('tf-status');
const mapStatus = document.getElementById('map-status');
const pickStatus = document.getElementById('pick-status');
const robotStatus = document.getElementById('robot-status');
const velDisplay = document.getElementById('vel-display');
const wsUrlInput = document.getElementById('ws-url');
const logList = document.getElementById('log-list');
const logCount = document.getElementById('log-count');

// ===== Event Log =====
const MAX_LOG = 200;
let logEntries = 0;

function log(msg, level = 'info') {
  logEntries++;
  const now = new Date();
  const ts = now.toLocaleTimeString('zh-CN', { hour12: false });
  const el = document.createElement('div');
  el.className = `log-entry ${level}`;
  el.innerHTML = `<span class="ts">${ts}</span>${msg}`;
  logList.insertBefore(el, logList.firstChild);
  logCount.textContent = `(${logEntries})`;
  while (logList.children.length > MAX_LOG) {
    logList.removeChild(logList.lastChild);
  }
}

// ===== Three.js Setup =====
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(canvas.clientWidth, canvas.clientHeight);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0d1b2a);

const camera = new THREE.PerspectiveCamera(60, canvas.clientWidth / canvas.clientHeight, 0.1, 500);
camera.position.set(10, 10, 15);
camera.lookAt(0, 0, 0);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.1;
controls.object.up.set(0, 0, 1);
controls.update();

// Lights
scene.add(new THREE.AmbientLight(0xffffff, 0.6));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(10, 20, 10);
scene.add(dirLight);

// Ground grid
const grid = new THREE.GridHelper(50, 50, 0x2a3a5e, 0x1a2a4e);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

// Axis helper
scene.add(new THREE.AxesHelper(3));

// ===== State =====
let ros = null;
let reconnectTimer = null;
const voxelSize = 0.2;

// Layer groups
const occupiedGroup = new THREE.Group();
const traversableGroup = new THREE.Group();
const preblockedGroup = new THREE.Group();
const riskGroup = new THREE.Group();
const pathGroup = new THREE.Group();
const robotGroup = new THREE.Group();
const markersGroup = new THREE.Group();
scene.add(occupiedGroup, traversableGroup, preblockedGroup, riskGroup, pathGroup, robotGroup, markersGroup);

// Pick targets
let traversablePickMesh = null;
let occupiedPickMesh = null;

// Placement mode
let placementMode = null; // 'start' | 'goal' | null
let dragStart = null;
let dragPlaneZ = 0;
let pointerCaptured = false;

// Navigation state
let hasStart = false;
let hasGoal = false;
let startPoint = null;
let goalPoint = null;

// TF state
const tfState = new Map();
const baseFrameCandidates = ['base_link', 'base_footprint', 'robot_base_link'];

// ROS topics (publishers)
let startTopic = null;
let startPoseTopic = null;
let goalTopic = null;
let goalPoseTopic = null;
let stopNavTopic = null;
let cmdVelTopic = null;
let addVoxelsTopic = null;
let removeVoxelsTopic = null;
let saveMapTopic = null;
let loadMapTopic = null;
let requestMapService = null;

// Joystick state
let joystickActive = false;
let joystickLinearX = 0;
let joystickLinearY = 0;
let joystickAngularZ = 0;
let joystickRepeatTimer = null;
let lastJoystickPublishTime = 0;
const JOYSTICK_MAX_LINEAR = 0.42;
const JOYSTICK_MAX_ANGULAR = 0.45;
const JOYSTICK_DEADBAND = 0.12;
const JOYSTICK_PUBLISH_MS = 80;

// ===== Robot Model =====
function createDogModel() {
  const group = new THREE.Group();
  const bodyMat = new THREE.MeshStandardMaterial({ color: 0x4fc3f7, roughness: 0.5 });
  const legMat = new THREE.MeshStandardMaterial({ color: 0x37474f, roughness: 0.6 });

  // Body
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.25, 0.15), bodyMat);
  body.position.set(0, 0, 0.15);
  group.add(body);

  // Head
  const head = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.12, 0.1), bodyMat);
  head.position.set(0.3, 0, 0.2);
  group.add(head);

  // Legs (4)
  const legGeo = new THREE.BoxGeometry(0.05, 0.05, 0.18);
  const offsets = [[0.18, 0.12], [0.18, -0.12], [-0.18, 0.12], [-0.18, -0.12]];
  for (const [x, y] of offsets) {
    const leg = new THREE.Mesh(legGeo, legMat);
    leg.position.set(x, y, 0.02);
    group.add(leg);
  }

  // Direction arrow
  const arrowGeo = new THREE.ConeGeometry(0.06, 0.15, 8);
  arrowGeo.rotateZ(-Math.PI / 2);
  const arrow = new THREE.Mesh(arrowGeo, new THREE.MeshStandardMaterial({ color: 0xff7043 }));
  arrow.position.set(0.35, 0, 0.25);
  group.add(arrow);

  group.visible = false;
  return group;
}

const dogModel = createDogModel();
robotGroup.add(dogModel);

// ===== Voxel Rendering =====
function clearGroup(group) {
  while (group.children.length) {
    const child = group.children[0];
    group.remove(child);
    if (child.geometry) child.geometry.dispose();
    if (child.material) {
      if (Array.isArray(child.material)) child.material.forEach(m => m.dispose());
      else child.material.dispose();
    }
  }
}

function makeVoxelLayer(points, color, opacity = 1.0) {
  if (!points || points.length === 0) return { group: new THREE.Group(), pickMesh: null };

  const size = voxelSize;
  const geo = new THREE.BoxGeometry(size, size, size);
  const fillMat = new THREE.MeshStandardMaterial({
    color, transparent: opacity < 0.99, opacity, roughness: 0.4, metalness: 0.05
  });
  const edgeMat = new THREE.MeshBasicMaterial({
    color: 0x555555, wireframe: true, transparent: true, opacity: Math.min(1.0, opacity + 0.15)
  });

  const count = points.length;
  const fillMesh = new THREE.InstancedMesh(geo, fillMat, count);
  const edgeMesh = new THREE.InstancedMesh(geo, edgeMat, count);
  fillMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  edgeMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);

  const mat = new THREE.Matrix4();
  for (let i = 0; i < count; i++) {
    const p = points[i];
    mat.makeTranslation(p.x, p.y, p.z);
    fillMesh.setMatrixAt(i, mat);
    edgeMesh.setMatrixAt(i, mat);
  }
  fillMesh.instanceMatrix.needsUpdate = true;
  edgeMesh.instanceMatrix.needsUpdate = true;

  const group = new THREE.Group();
  group.add(fillMesh);
  group.add(edgeMesh);
  return { group, pickMesh: fillMesh };
}

// Map data: collect all chunks, render once, then ignore further updates
let occupiedPointsBuf = [];
let traversablePointsBuf = [];
let preblockedPointsBuf = [];
let occupiedRenderTimer = null;
let traversableRenderTimer = null;
let preblockedRenderTimer = null;
let mapLoaded = { occupied: false, traversable: false, preblocked: false, risk: false };
let mapRepublishing = { occupied: false, traversable: false };
const CHUNK_COLLECT_MS = 800;

// Edit mode state
let editMode = false;
let editBrushSize = 3;
let levelTargetZ = null;
let levelRange = 1.0;
let editDragging = false;
let pendingAddPositions = [];
let pendingRemovePositions = [];
let editFlushTimer = null;
let brushPreviewGroup = null;
let mapZMin = 0;
let mapZMax = 5;

function setOccupiedMarker(msg) {
  if (!msg.points || msg.points.length === 0) return;
  if (msg.id === 0) {
    if (mapLoaded.occupied) mapRepublishing.occupied = true;
    occupiedPointsBuf = [];
  }
  occupiedPointsBuf.push(...msg.points);
  if (occupiedRenderTimer) clearTimeout(occupiedRenderTimer);
  occupiedRenderTimer = setTimeout(() => {
    clearGroup(occupiedGroup);
    const { group, pickMesh } = makeVoxelLayer(occupiedPointsBuf, 0xff7043, 0.92);
    occupiedGroup.add(group);
    occupiedPickMesh = pickMesh;
    if (!mapLoaded.occupied) {
      mapLoaded.occupied = true;
      log(`占据层: ${occupiedPointsBuf.length} 体素`, 'info');
    } else if (mapRepublishing.occupied) {
      mapRepublishing.occupied = false;
      log(`占据层: 已刷新 — ${occupiedPointsBuf.length} 体素`, 'info');
    }
    updateMapProgress();
    autoFrameCamera();
    occupiedRenderTimer = null;
  }, CHUNK_COLLECT_MS);
}

function setTraversableMarker(msg) {
  if (!msg.points || msg.points.length === 0) return;
  if (msg.id === 0) {
    if (mapLoaded.traversable) mapRepublishing.traversable = true;
    traversablePointsBuf = [];
  }
  traversablePointsBuf.push(...msg.points);
  if (traversableRenderTimer) clearTimeout(traversableRenderTimer);
  traversableRenderTimer = setTimeout(() => {
    clearGroup(traversableGroup);
    const { group, pickMesh } = makeVoxelLayer(traversablePointsBuf, 0x00e676, 0.28);
    traversableGroup.add(group);
    traversablePickMesh = pickMesh;
    if (!mapLoaded.traversable) {
      mapLoaded.traversable = true;
      log(`可通行层: ${traversablePointsBuf.length} 体素`, 'info');
    } else if (mapRepublishing.traversable) {
      mapRepublishing.traversable = false;
      log(`可通行层: 已刷新 — ${traversablePointsBuf.length} 体素`, 'info');
    }
    updateMapProgress();
    traversableRenderTimer = null;
  }, CHUNK_COLLECT_MS);
}

function setPreblockedMarker(msg) {
  if (!msg.points || msg.points.length === 0) return;
  if (msg.id === 0) preblockedPointsBuf = [];
  preblockedPointsBuf.push(...msg.points);
  if (preblockedRenderTimer) clearTimeout(preblockedRenderTimer);
  preblockedRenderTimer = setTimeout(() => {
    clearGroup(preblockedGroup);
    const { group } = makeVoxelLayer(preblockedPointsBuf, 0xb388ff, 0.90);
    preblockedGroup.add(group);
    const wasLoaded = mapLoaded.preblocked;
    mapLoaded.preblocked = true;
    log(wasLoaded ? `禁行层: 已刷新 — ${preblockedPointsBuf.length} 体素` : `禁行层: ${preblockedPointsBuf.length} 体素`, 'info');
    updateMapProgress();
    preblockedRenderTimer = null;
  }, CHUNK_COLLECT_MS);
}

function autoFrameCamera() {
  if (occupiedPointsBuf.length === 0) return;
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity, minZ = Infinity, maxZ = -Infinity;
  for (const p of occupiedPointsBuf) {
    if (p.x < minX) minX = p.x;
    if (p.x > maxX) maxX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.y > maxY) maxY = p.y;
    if (p.z < minZ) minZ = p.z;
    if (p.z > maxZ) maxZ = p.z;
  }
  mapZMin = minZ;
  mapZMax = maxZ;
  const cx = (minX + maxX) / 2;
  const cy = (minY + maxY) / 2;
  const span = Math.max(maxX - minX, maxY - minY);
  const dist = span * 0.8;
  camera.position.set(cx + dist * 0.5, cy - dist * 0.5, maxZ + dist * 0.6);
  controls.target.set(cx, cy, 0);
  controls.update();
}

function setRiskCostCloud(msg) {
  clearGroup(riskGroup);
  const points = parsePointCloud2(msg);
  if (!points || points.length === 0) return;
  const isRepublish = mapLoaded.risk;
  mapLoaded.risk = true;

  const size = voxelSize;
  const geo = new THREE.BoxGeometry(size, size, size);

  for (const p of points) {
    const mat = new THREE.MeshStandardMaterial({
      color: 0xffd740, transparent: true, opacity: Math.min(0.9, p.intensity * 0.8 + 0.1),
      roughness: 0.5
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(p.x, p.y, p.z);
    riskGroup.add(mesh);
  }
  log(isRepublish ? `代价层: 已刷新 — ${points.length} 点` : `代价层: ${points.length} 点`, 'info');
  updateMapProgress();
}

function parsePointCloud2(msg) {
  if (!msg.data || msg.data.length === 0) return [];
  const raw = atob(msg.data);
  const buf = new ArrayBuffer(raw.length);
  const view = new Uint8Array(buf);
  for (let i = 0; i < raw.length; i++) view[i] = raw.charCodeAt(i);
  const dv = new DataView(buf);

  const pointStep = msg.point_step;
  const width = msg.width;
  const height = msg.height || 1;
  const count = width * height;
  const littleEndian = !msg.is_bigendian;

  let xOff = 0, yOff = 4, zOff = 8, iOff = 12;
  for (const f of msg.fields) {
    if (f.name === 'x') xOff = f.offset;
    else if (f.name === 'y') yOff = f.offset;
    else if (f.name === 'z') zOff = f.offset;
    else if (f.name === 'intensity') iOff = f.offset;
  }

  const points = [];
  for (let i = 0; i < count; i++) {
    const base = i * pointStep;
    if (base + pointStep > buf.byteLength) break;
    points.push({
      x: dv.getFloat32(base + xOff, littleEndian),
      y: dv.getFloat32(base + yOff, littleEndian),
      z: dv.getFloat32(base + zOff, littleEndian),
      intensity: dv.getFloat32(base + iOff, littleEndian),
    });
  }
  return points;
}

// ===== Map Edit Functions =====
function pickOccupied(event) {
  updatePointer(event);
  raycaster.setFromCamera(pointer, camera);
  if (occupiedPickMesh) {
    const hits = raycaster.intersectObject(occupiedPickMesh);
    if (hits.length > 0) {
      // Use exact voxel center from buffer — the geometric hit point
      // lies on the box surface which may snap to a different octomap key
      const idx = hits[0].instanceId;
      if (idx !== undefined && idx < occupiedPointsBuf.length) {
        return occupiedPointsBuf[idx];
      }
      return hits[0].point.clone();
    }
  }
  return null;
}

// Snap to octomap voxel center: (floor(coord/res) + 0.5) * res
// Matches octomap's OcTreeKey → world coordinate convention
function snapToGrid(v) { return (Math.floor(v / voxelSize) + 0.5) * voxelSize; }

function getBrushPositions(cx, cy, z) {
  const sx = snapToGrid(cx);
  const sy = snapToGrid(cy);
  const sz = snapToGrid(z);
  const half = Math.floor(editBrushSize / 2);
  const positions = [];
  for (let dx = -half; dx <= half; dx++) {
    for (let dy = -half; dy <= half; dy++) {
      positions.push({
        x: sx + dx * voxelSize,
        y: sy + dy * voxelSize,
        z: sz
      });
    }
  }
  return positions;
}

function buildPointCloud2(points) {
  const pointStep = 12;
  const buf = new ArrayBuffer(points.length * pointStep);
  const dv = new DataView(buf);
  for (let i = 0; i < points.length; i++) {
    const off = i * pointStep;
    dv.setFloat32(off, points[i].x, true);
    dv.setFloat32(off + 4, points[i].y, true);
    dv.setFloat32(off + 8, points[i].z, true);
  }
  const bytes = new Uint8Array(buf);
  let binary = '';
  for (let i = 0; i < bytes.length; i++) {
    binary += String.fromCharCode(bytes[i]);
  }
  return {
    header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
    height: 1,
    width: points.length,
    fields: [
      { name: 'x', offset: 0, datatype: 7, count: 1 },
      { name: 'y', offset: 4, datatype: 7, count: 1 },
      { name: 'z', offset: 8, datatype: 7, count: 1 }
    ],
    is_bigendian: false,
    point_step: pointStep,
    row_step: points.length * pointStep,
    data: btoa(binary)
  };
}

function applyLocalEdit(addPositions, removePositions) {
  // Add occupied at target Z
  const existing = new Set(occupiedPointsBuf.map(p => `${p.x.toFixed(4)},${p.y.toFixed(4)},${p.z.toFixed(4)}`));
  const toAdd = addPositions.filter(p => !existing.has(`${p.x.toFixed(4)},${p.y.toFixed(4)},${p.z.toFixed(4)}`));
  if (toAdd.length > 0) occupiedPointsBuf.push(...toAdd);

  // Remove occupied within range of target Z
  const removeSet = new Set(removePositions.map(p => `${p.x.toFixed(4)},${p.y.toFixed(4)},${p.z.toFixed(4)}`));
  if (removeSet.size > 0) {
    occupiedPointsBuf = occupiedPointsBuf.filter(p => !removeSet.has(`${p.x.toFixed(4)},${p.y.toFixed(4)},${p.z.toFixed(4)}`));
  }

  clearGroup(occupiedGroup);
  const { group, pickMesh } = makeVoxelLayer(occupiedPointsBuf, 0xff7043, 0.92);
  occupiedGroup.add(group);
  occupiedPickMesh = pickMesh;
}

function flushEdits() {
  const hasAdds = pendingAddPositions.length > 0 && addVoxelsTopic;
  const hasRemoves = pendingRemovePositions.length > 0 && removeVoxelsTopic;
  if (hasAdds) {
    const msg = buildPointCloud2(pendingAddPositions);
    addVoxelsTopic.publish(new ROSLIB.Message(msg));
    log(`刷平: 添加 ${pendingAddPositions.length} 体素`, 'info');
    pendingAddPositions = [];
  }
  if (hasRemoves) {
    const msg = buildPointCloud2(pendingRemovePositions);
    removeVoxelsTopic.publish(new ROSLIB.Message(msg));
    log(`刷平: 清除 ${pendingRemovePositions.length} 体素`, 'info');
    pendingRemovePositions = [];
  }
  if (editFlushTimer) { clearTimeout(editFlushTimer); editFlushTimer = null; }
}

function applyBrushEdit(event) {
  const hit = pickOccupied(event);
  if (!hit) return;
  if (levelTargetZ === null) {
    levelTargetZ = snapToGrid(hit.z);
    log(`刷平目标 Z=${levelTargetZ.toFixed(2)}m`, 'info');
  }
  const positions = getBrushPositions(hit.x, hit.y, levelTargetZ);

  // Deduplicate pending adds
  const addSet = new Set(pendingAddPositions.map(p => `${p.x.toFixed(4)},${p.y.toFixed(4)},${p.z.toFixed(4)}`));
  const newAdds = positions.filter(p => !addSet.has(`${p.x.toFixed(4)},${p.y.toFixed(4)},${p.z.toFixed(4)}`));

  // Find voxels to remove within range at each brush XY
  const allRemoves = [];
  const rmKeySet = new Set(pendingRemovePositions.map(p => `${p.x.toFixed(4)},${p.y.toFixed(4)},${p.z.toFixed(4)}`));
  for (const bp of positions) {
    for (const p of occupiedPointsBuf) {
      if (p.z < levelTargetZ - levelRange || p.z > levelTargetZ + levelRange) continue;
      const dx = Math.abs(p.x - bp.x);
      const dy = Math.abs(p.y - bp.y);
      if (dx < voxelSize * 0.6 && dy < voxelSize * 0.6) {
        const key = `${p.x.toFixed(4)},${p.y.toFixed(4)},${p.z.toFixed(4)}`;
        if (!rmKeySet.has(key) && key !== `${bp.x.toFixed(4)},${bp.y.toFixed(4)},${bp.z.toFixed(4)}`) {
          allRemoves.push({ x: p.x, y: p.y, z: p.z });
          rmKeySet.add(key);
        }
      }
    }
  }

  if (newAdds.length === 0 && allRemoves.length === 0) return;
  pendingAddPositions.push(...newAdds);
  pendingRemovePositions.push(...allRemoves);
  applyLocalEdit(newAdds, allRemoves);
  if (editFlushTimer) clearTimeout(editFlushTimer);
  editFlushTimer = setTimeout(() => flushEdits(), 200);
}

function updateBrushPreview(event) {
  clearBrushPreview();
  const hit = pickOccupied(event);
  if (!hit) return;
  const z = levelTargetZ !== null ? levelTargetZ : snapToGrid(hit.z);
  const positions = getBrushPositions(hit.x, hit.y, z);
  const size = voxelSize;
  const geo = new THREE.BoxGeometry(size, size, size);
  brushPreviewGroup = new THREE.Group();

  // Green preview for the target Z plane (add)
  const matAdd = new THREE.MeshBasicMaterial({ color: 0x66bb6a, transparent: true, opacity: 0.5 });
  for (const p of positions) {
    const mesh = new THREE.Mesh(geo, matAdd);
    mesh.position.set(p.x, p.y, p.z);
    brushPreviewGroup.add(mesh);
  }

  // Red wireframe for the clearance range (remove zone)
  if (levelTargetZ !== null) {
    const zMin = levelTargetZ - levelRange;
    const zMax = levelTargetZ + levelRange;
    const rangeHeight = zMax - zMin;
    if (rangeHeight > 0) {
      const rangeGeo = new THREE.BoxGeometry(size * 1.05, size * 1.05, rangeHeight);
      const matRange = new THREE.MeshBasicMaterial({ color: 0xef5350, transparent: true, opacity: 0.15, wireframe: true });
      for (const p of positions) {
        const mesh = new THREE.Mesh(rangeGeo, matRange);
        mesh.position.set(p.x, p.y, (zMin + zMax) / 2);
        brushPreviewGroup.add(mesh);
      }
    }
  }

  markersGroup.add(brushPreviewGroup);
}

function clearBrushPreview() {
  if (brushPreviewGroup) {
    markersGroup.remove(brushPreviewGroup);
    brushPreviewGroup.traverse(c => { if (c.geometry) c.geometry.dispose(); if (c.material) c.material.dispose(); });
    brushPreviewGroup = null;
  }
}

function updateLevelRangeUI() {
  const rngVal = document.getElementById('level-range-val');
  if (rngVal) rngVal.textContent = levelRange.toFixed(1);
}

// ===== Path Rendering =====
function setPlannedPath(msg) {
  clearGroup(pathGroup);
  if (!msg.poses || msg.poses.length < 2) return;

  const pts = msg.poses.map(p => new THREE.Vector3(p.pose.position.x, p.pose.position.y, p.pose.position.z));
  const curve = new THREE.CatmullRomCurve3(pts);
  const tubeGeo = new THREE.TubeGeometry(curve, Math.max(20, pts.length * 2), 0.04, 8, false);
  const tubeMat = new THREE.MeshStandardMaterial({
    color: 0x00e5ff,
    emissive: 0x00e5ff,
    emissiveIntensity: 0.5,
    roughness: 0.2
  });
  pathGroup.add(new THREE.Mesh(tubeGeo, tubeMat));

  pickStatus.textContent = `路径: ${msg.poses.length} 点`;
  log(`规划路径: ${msg.poses.length} 个航点, 长度 ${(pts.length > 1 ? curve.getLength().toFixed(1) : '0')} m`, 'ok');
}

// ===== Data Staleness =====
const STALE_TF_MS = 3000;
let lastDataTime = {
  tf: 0,
  get now() { return performance.now(); }
};

function markDataAge(key) { lastDataTime[key] = lastDataTime.now; }

function checkStaleness() {
  const n = lastDataTime.now;

  if (lastDataTime.tf > 0 && n - lastDataTime.tf > STALE_TF_MS) {
    tfState.clear();
    lastDataTime.tf = 0;
    tfFirstReceived = false;
    poseFirstReceived = false;
    updateRobotModel();
    log('TF 数据超时，已清空位姿', 'warn');
  }
}

// ===== TF System =====
let tfFirstReceived = false;
function normalizeFrame(f) { return f.startsWith('/') ? f.slice(1) : f; }

function storeTransform(msg) {
  if (!msg.transforms) return;
  markDataAge('tf');
  for (const t of msg.transforms) {
    const parent = normalizeFrame(t.header.frame_id);
    const child = normalizeFrame(t.child_frame_id);
    tfState.set(`${parent}->${child}`, t.transform);
  }
  if (!tfFirstReceived) {
    tfFirstReceived = true;
    log('TF 坐标树已接收', 'ok');
  }
}

function getTransform(parent, child) {
  return tfState.get(`${parent}->${child}`) || null;
}

function quaternionToYaw(q) {
  const siny = 2 * (q.w * q.z + q.x * q.y);
  const cosy = 1 - 2 * (q.y * q.y + q.z * q.z);
  return Math.atan2(siny, cosy);
}

function resolveRobotPose() {
  // Try direct map -> base
  for (const base of baseFrameCandidates) {
    const tf = getTransform('map', base);
    if (tf) {
      const yaw = quaternionToYaw(tf.rotation);
      return { x: tf.translation.x, y: tf.translation.y, z: tf.translation.z, yaw };
    }
  }

  // Try chained: map -> odom -> base
  const mapOdom = getTransform('map', 'odom');
  if (!mapOdom) return null;

  for (const base of baseFrameCandidates) {
    const odomBase = getTransform('odom', base);
    if (!odomBase) continue;

    const mapYaw = quaternionToYaw(mapOdom.rotation);
    const bx = odomBase.translation.x;
    const by = odomBase.translation.y;
    const x = mapOdom.translation.x + Math.cos(mapYaw) * bx - Math.sin(mapYaw) * by;
    const y = mapOdom.translation.y + Math.sin(mapYaw) * bx + Math.cos(mapYaw) * by;
    const z = mapOdom.translation.z + odomBase.translation.z;
    const yaw = mapYaw + quaternionToYaw(odomBase.rotation);
    return { x, y, z, yaw };
  }

  return null;
}

let poseFirstReceived = false;

function updateRobotModel() {
  const pose = resolveRobotPose();
  if (!pose) {
    dogModel.visible = false;
    robotStatus.className = 'status-indicator offline';
    robotStatus.textContent = '机器人未定位';
    tfStatus.textContent = '无';
    return;
  }

  dogModel.visible = true;
  dogModel.position.set(pose.x, pose.y, pose.z);
  dogModel.rotation.set(0, 0, pose.yaw);
  updateRobotAxes(pose);
  robotStatus.className = 'status-indicator online';
  robotStatus.textContent = '已定位';
  tfStatus.textContent = `(${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, ${pose.z.toFixed(2)}) yaw=${(pose.yaw * 180 / Math.PI).toFixed(0)}°`;
  if (!poseFirstReceived) {
    poseFirstReceived = true;
    log(`机器人已定位: (${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, ${pose.z.toFixed(2)})`, 'ok');
  }

  // Auto-set start from robot pose if not manually set
  if (!hasStart) {
    startPoint = { x: pose.x, y: pose.y, z: pose.z };
  }
}

// ===== Raycasting & Interaction =====
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();

function updatePointer(event) {
  const rect = canvas.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
}

function pickTraversable(event) {
  if (!traversablePickMesh) return null;
  updatePointer(event);
  raycaster.setFromCamera(pointer, camera);
  const hits = raycaster.intersectObject(traversablePickMesh);
  return hits.length > 0 ? hits[0].point.clone() : null;
}

function pickOnPlane(event, planeZ) {
  updatePointer(event);
  raycaster.setFromCamera(pointer, camera);
  const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -planeZ);
  const hit = new THREE.Vector3();
  return raycaster.ray.intersectPlane(plane, hit) ? hit : null;
}

// ===== Placement Event Handlers =====
function onCanvasPointerDown(event) {
  if (editMode && event.button === 0) {
    event.preventDefault();
    controls.enabled = false;
    canvas.setPointerCapture(event.pointerId);
    pointerCaptured = true;
    editDragging = true;
    applyBrushEdit(event);
    return;
  }

  if (!placementMode || event.button !== 0) return;

  const hit = pickTraversable(event);
  if (!hit) return;

  event.preventDefault();
  controls.enabled = false;
  canvas.setPointerCapture(event.pointerId);
  pointerCaptured = true;
  dragStart = hit.clone();
  dragPlaneZ = hit.z;
}

let dragPreviewArrow = null;

function clearDragPreview() {
  if (dragPreviewArrow) {
    markersGroup.remove(dragPreviewArrow);
    dragPreviewArrow.traverse(c => { if (c.geometry) c.geometry.dispose(); if (c.material) c.material.dispose(); });
    dragPreviewArrow = null;
  }
}

function onCanvasPointerMove(event) {
  if (editDragging) {
    applyBrushEdit(event);
    updateBrushPreview(event);
    return;
  }

  if (editMode) {
    updateBrushPreview(event);
    return;
  }

  if (!pointerCaptured || !dragStart) return;

  const endPt = pickOnPlane(event, dragPlaneZ);
  if (!endPt) return;

  const dx = endPt.x - dragStart.x;
  const dy = endPt.y - dragStart.y;
  const dist = Math.sqrt(dx * dx + dy * dy);
  if (dist < 0.05) { clearDragPreview(); return; }

  const yaw = Math.atan2(dy, dx);
  clearDragPreview();

  const arrowLen = Math.min(dist * 1.5, 1.0);
  const arrowGroup = new THREE.Group();
  arrowGroup.position.set(dragStart.x, dragStart.y, dragStart.z + 0.08);

  const arrow = makeArrow(arrowLen, 0xffab40, 0.85);
  arrow.rotation.set(0, 0, yaw);
  arrowGroup.add(arrow);

  markersGroup.add(arrowGroup);
  dragPreviewArrow = arrowGroup;
}

function onCanvasPointerUp(event) {
  if (editDragging) {
    canvas.releasePointerCapture(event.pointerId);
    pointerCaptured = false;
    editDragging = false;
    controls.enabled = true;
    levelTargetZ = null;
    clearBrushPreview();
    flushEdits();
    return;
  }

  if (!pointerCaptured) return;
  canvas.releasePointerCapture(event.pointerId);
  pointerCaptured = false;
  controls.enabled = true;

  const endPt = pickOnPlane(event, dragPlaneZ);
  const origin = dragStart;
  let yaw = 0;

  if (endPt && origin) {
    const dx = endPt.x - origin.x;
    const dy = endPt.y - origin.y;
    if (Math.sqrt(dx * dx + dy * dy) > 0.05) {
      yaw = Math.atan2(dy, dx);
    }
  }

  if (!origin) return;

  if (placementMode === 'start') {
    startPoint = { x: origin.x, y: origin.y, z: origin.z };
    hasStart = true;
    publishStartPose(origin, yaw);
    const yawDeg = (yaw * 180 / Math.PI).toFixed(1);
    pickStatus.textContent = `起点: (${origin.x.toFixed(2)}, ${origin.y.toFixed(2)}) yaw=${yawDeg}°`;
    setMarker('start', origin, 0x66bb6a, yaw);
    log(`设置起点: (${origin.x.toFixed(2)}, ${origin.y.toFixed(2)}, ${origin.z.toFixed(2)}) 航向=${yawDeg}°`, 'info');
  } else if (placementMode === 'goal') {
    goalPoint = { x: origin.x, y: origin.y, z: origin.z };
    hasGoal = true;
    // Auto-publish start from robot pose if not manually set
    if (!hasStart && startPoint) {
      publishStartPoint(startPoint);
      log(`自动起点(里程计): (${startPoint.x.toFixed(2)}, ${startPoint.y.toFixed(2)}, ${startPoint.z.toFixed(2)})`, 'info');
    }
    publishGoalPose(origin, yaw);
    const yawDeg = (yaw * 180 / Math.PI).toFixed(1);
    pickStatus.textContent = `终点: (${origin.x.toFixed(2)}, ${origin.y.toFixed(2)}) yaw=${yawDeg}°`;
    setMarker('goal', origin, 0xef5350, yaw);
    log(`设置终点: (${origin.x.toFixed(2)}, ${origin.y.toFixed(2)}, ${origin.z.toFixed(2)}) 航向=${yawDeg}°`, 'info');
  }

  // Exit placement mode after selection
  clearDragPreview();
  setActivePlacementBtn(null);
  dragStart = null;
}

const namedMarkers = {};

function makeArrow(length, color, opacity = 1.0) {
  // Arrow pointing in +X, lying in XY plane
  const arrow = new THREE.Group();
  const mat = new THREE.MeshStandardMaterial({
    color, emissive: color, emissiveIntensity: 0.5,
    transparent: opacity < 1, opacity
  });

  const shaftLen = length * 0.7;
  const shaftGeo = new THREE.CylinderGeometry(0.025, 0.025, shaftLen, 8);
  shaftGeo.rotateZ(-Math.PI / 2); // lay along +X
  const shaft = new THREE.Mesh(shaftGeo, mat);
  shaft.position.set(shaftLen / 2, 0, 0);
  arrow.add(shaft);

  const coneGeo = new THREE.ConeGeometry(0.07, length * 0.3, 8);
  coneGeo.rotateZ(-Math.PI / 2); // tip toward +X
  const cone = new THREE.Mesh(coneGeo, mat.clone());
  cone.position.set(shaftLen + length * 0.15, 0, 0);
  arrow.add(cone);

  return arrow;
}

function setMarker(name, point, color, yaw = null) {
  if (namedMarkers[name]) {
    markersGroup.remove(namedMarkers[name]);
    namedMarkers[name].traverse(child => {
      if (child.geometry) child.geometry.dispose();
      if (child.material) child.material.dispose();
    });
  }

  const group = new THREE.Group();
  group.position.set(point.x, point.y, point.z + 0.05);
  group.name = name;

  const geo = new THREE.SphereGeometry(0.12, 16, 16);
  const mat = new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.4 });
  group.add(new THREE.Mesh(geo, mat));

  if (yaw !== null) {
    const arrow = makeArrow(0.6, color);
    arrow.rotation.set(0, 0, yaw);
    group.add(arrow);
  }

  markersGroup.add(group);
  namedMarkers[name] = group;
}

// Robot pose visualizer: small XYZ axes (6DOF)
let robotAxesHelper = null;

function updateRobotAxes(pose) {
  if (!robotAxesHelper) {
    robotAxesHelper = new THREE.AxesHelper(0.4);
    robotGroup.add(robotAxesHelper);
  }
  robotAxesHelper.visible = true;
  robotAxesHelper.position.set(pose.x, pose.y, pose.z + 0.02);
  robotAxesHelper.rotation.set(0, 0, pose.yaw);
}

// ===== ROS Publishing =====
function publishStartPose(pt, yaw = 0) {
  if (startTopic) {
    startTopic.publish(new ROSLIB.Message({
      header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
      point: { x: pt.x, y: pt.y, z: pt.z }
    }));
  }
  if (startPoseTopic) {
    startPoseTopic.publish(new ROSLIB.Message({
      header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
      pose: {
        position: { x: pt.x, y: pt.y, z: pt.z },
        orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) }
      }
    }));
  }
}

function publishStartPoint(pt) {
  publishStartPose(pt, 0);
}

function publishGoalPose(pt, yaw) {
  if (goalPoseTopic) {
    goalPoseTopic.publish(new ROSLIB.Message({
      header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
      pose: {
        position: { x: pt.x, y: pt.y, z: pt.z },
        orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) }
      }
    }));
  }
}

function publishStopNavigation() {
  if (!stopNavTopic) return;
  stopNavTopic.publish(new ROSLIB.Message({ data: true }));
  log('导航已停止', 'warn');
}

function publishCmdVel(lx, ly, az) {
  if (!cmdVelTopic) return;
  const now = Date.now();
  if (now - lastJoystickPublishTime < JOYSTICK_PUBLISH_MS) return;
  lastJoystickPublishTime = now;
  cmdVelTopic.publish(new ROSLIB.Message({
    linear: { x: lx, y: ly, z: 0 },
    angular: { x: 0, y: 0, z: az }
  }));
  velDisplay.textContent = `x=${lx.toFixed(2)} y=${ly.toFixed(2)} wz=${az.toFixed(2)}`;
}

// ===== Joystick =====
const joystickPad = document.getElementById('joystick-pad');
const joystickKnob = document.getElementById('joystick-knob');
const rotationSlider = document.getElementById('rotation-slider');

function applySpeedCurve(normalized, maxSpeed) {
  const sign = Math.sign(normalized);
  const mag = Math.abs(normalized);
  if (mag < JOYSTICK_DEADBAND) return 0;
  return sign * ((mag - JOYSTICK_DEADBAND) / (1 - JOYSTICK_DEADBAND)) * maxSpeed;
}

function onJoystickDown(event) {
  event.preventDefault();
  joystickKnob.setPointerCapture(event.pointerId);
  joystickActive = true;
  ensureRepeatTimer();
}

function onJoystickMove(event) {
  if (!joystickActive) return;
  const rect = joystickPad.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  const radius = rect.width / 2;

  let dx = (event.clientX - cx) / radius;
  let dy = -(event.clientY - cy) / radius;
  const dist = Math.sqrt(dx * dx + dy * dy);
  if (dist > 1) { dx /= dist; dy /= dist; }

  // Visual knob position
  joystickKnob.style.left = `${50 + dx * 42}%`;
  joystickKnob.style.top = `${50 - dy * 42}%`;

  joystickLinearX = applySpeedCurve(dy, JOYSTICK_MAX_LINEAR);
  joystickLinearY = applySpeedCurve(-dx, JOYSTICK_MAX_LINEAR);
  publishCmdVel(joystickLinearX, joystickLinearY, joystickAngularZ);
}

function onJoystickUp(event) {
  joystickKnob.releasePointerCapture(event.pointerId);
  joystickActive = false;
  joystickKnob.style.left = '50%';
  joystickKnob.style.top = '50%';
  joystickLinearX = 0;
  joystickLinearY = 0;
  publishCmdVel(0, 0, joystickAngularZ);
  if (joystickAngularZ === 0) clearRepeatTimer();
}

function onRotationInput() {
  const val = Number(rotationSlider.value) / 100;
  joystickAngularZ = applySpeedCurve(val, JOYSTICK_MAX_ANGULAR);
  publishCmdVel(joystickLinearX, joystickLinearY, joystickAngularZ);
  if (joystickAngularZ !== 0) ensureRepeatTimer();
}

function onRotationRelease() {
  rotationSlider.value = 0;
  joystickAngularZ = 0;
  publishCmdVel(joystickLinearX, joystickLinearY, 0);
  if (!joystickActive) clearRepeatTimer();
}

function ensureRepeatTimer() {
  if (joystickRepeatTimer) return;
  joystickRepeatTimer = setInterval(() => {
    if (joystickLinearX === 0 && joystickLinearY === 0 && joystickAngularZ === 0) {
      clearRepeatTimer();
      return;
    }
    lastJoystickPublishTime = 0; // force publish
    publishCmdVel(joystickLinearX, joystickLinearY, joystickAngularZ);
  }, 100);
}

function clearRepeatTimer() {
  if (joystickRepeatTimer) { clearInterval(joystickRepeatTimer); joystickRepeatTimer = null; }
}

// ===== ROSBridge Connection =====
function getDefaultWsUrl() {
  const host = window.location.hostname || 'localhost';
  return `ws://${host}:9090`;
}

function connectRos() {
  const url = wsUrlInput.value || getDefaultWsUrl();
  wsUrlInput.value = url;

  if (ros) { ros.close(); ros = null; }

  ros = new ROSLIB.Ros({ url });

  ros.on('connection', () => {
    connStatus.textContent = '已连接';
    connStatus.style.color = '#66bb6a';
    log('已连接 ROSBridge @ ' + url, 'ok');
    setupTopics();
  });

  ros.on('error', () => {
    connStatus.textContent = '连接错误';
    connStatus.style.color = '#ef5350';
    log('ROSBridge 连接错误', 'err');
  });

  ros.on('close', () => {
    connStatus.textContent = '已断开';
    connStatus.style.color = '#ef5350';
    tfFirstReceived = false;
    poseFirstReceived = false;
    log('ROSBridge 连接断开', 'warn');
    scheduleReconnect();
  });
}

function scheduleReconnect() {
  if (reconnectTimer) return;
  reconnectTimer = setTimeout(() => {
    reconnectTimer = null;
    connectRos();
  }, 2000);
}

function setupTopics() {
  // Publishers
  startTopic = new ROSLIB.Topic({ ros, name: '/start_point', messageType: 'geometry_msgs/PointStamped' });
  startPoseTopic = new ROSLIB.Topic({ ros, name: '/start_pose', messageType: 'geometry_msgs/PoseStamped' });
  goalTopic = new ROSLIB.Topic({ ros, name: '/goal_point', messageType: 'geometry_msgs/PointStamped' });
  goalPoseTopic = new ROSLIB.Topic({ ros, name: '/goal_pose', messageType: 'geometry_msgs/PoseStamped' });
  stopNavTopic = new ROSLIB.Topic({ ros, name: '/stop_navigation', messageType: 'std_msgs/Bool' });
  cmdVelTopic = new ROSLIB.Topic({ ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist' });
  addVoxelsTopic = new ROSLIB.Topic({ ros, name: '/add_occupied_voxels', messageType: 'sensor_msgs/PointCloud2' });
  removeVoxelsTopic = new ROSLIB.Topic({ ros, name: '/remove_occupied_voxels', messageType: 'sensor_msgs/PointCloud2' });
  saveMapTopic = new ROSLIB.Topic({ ros, name: '/save_octomap_path', messageType: 'std_msgs/String' });
  loadMapTopic = new ROSLIB.Topic({ ros, name: '/load_map_file', messageType: 'std_msgs/String' });

  // Subscribers
  new ROSLIB.Topic({ ros, name: '/octomap_occupied_markers', messageType: 'visualization_msgs/Marker' })
    .subscribe(setOccupiedMarker);
  new ROSLIB.Topic({ ros, name: '/traversable_cells_markers', messageType: 'visualization_msgs/Marker' })
    .subscribe(setTraversableMarker);
  new ROSLIB.Topic({ ros, name: '/preblocked_cells_markers', messageType: 'visualization_msgs/Marker' })
    .subscribe(setPreblockedMarker);
  new ROSLIB.Topic({ ros, name: '/risk_cost_cells', messageType: 'sensor_msgs/PointCloud2' })
    .subscribe(setRiskCostCloud);
  new ROSLIB.Topic({ ros, name: '/planned_path', messageType: 'nav_msgs/Path' })
    .subscribe(setPlannedPath);
  new ROSLIB.Topic({ ros, name: '/tf', messageType: 'tf2_msgs/TFMessage' })
    .subscribe(storeTransform);
  new ROSLIB.Topic({ ros, name: '/tf_static', messageType: 'tf2_msgs/TFMessage' })
    .subscribe(storeTransform);

  // Service for manual map fetch
  requestMapService = new ROSLIB.Service({ ros, name: '/request_map', serviceType: 'std_srvs/Trigger' });

  // Auto-fetch map data on connect (with small delay for rosbridge to wire subscriptions)
  setTimeout(() => {
    requestMap();
  }, 500);
}

function updateMapProgress() {
  const parts = [];
  if (mapLoaded.occupied) parts.push(`占据 ${occupiedPointsBuf.length}`);
  if (mapLoaded.traversable) parts.push(`可通行 ${traversablePointsBuf.length}`);
  if (mapLoaded.preblocked) parts.push(`禁行 ${preblockedPointsBuf.length}`);
  if (mapLoaded.risk) parts.push('代价 ✓');
  if (parts.length === 0) {
    mapStatus.textContent = '等待数据...';
  } else if (mapLoaded.occupied && mapLoaded.traversable && mapLoaded.preblocked && mapLoaded.risk) {
    mapStatus.textContent = '获取完成: ' + parts.join(', ');
    log('地图数据全部接收完成', 'ok');
  } else {
    mapStatus.textContent = '接收中: ' + parts.join(', ');
  }
}

function requestMap() {
  if (!requestMapService) return;
  resetMapData();
  mapStatus.textContent = '请求地图...';
  log('请求地图数据...', 'info');
  requestMapService.callService({}, (resp) => {
    if (!resp.success) {
      mapStatus.textContent = '获取失败: ' + resp.message;
      log('地图获取失败: ' + resp.message, 'err');
    }
  });
}

function resetMapData() {
  if (occupiedRenderTimer) { clearTimeout(occupiedRenderTimer); occupiedRenderTimer = null; }
  if (traversableRenderTimer) { clearTimeout(traversableRenderTimer); traversableRenderTimer = null; }
  if (preblockedRenderTimer) { clearTimeout(preblockedRenderTimer); preblockedRenderTimer = null; }
  occupiedPointsBuf = [];
  traversablePointsBuf = [];
  preblockedPointsBuf = [];
  mapLoaded.occupied = false;
  mapLoaded.traversable = false;
  mapLoaded.preblocked = false;
  mapLoaded.risk = false;
  clearGroup(occupiedGroup);
  clearGroup(traversableGroup);
  clearGroup(preblockedGroup);
  clearGroup(riskGroup);
  if (traversablePickMesh) { traversablePickMesh = null; }
  if (occupiedPickMesh) { occupiedPickMesh = null; }
  log('已清空地图缓存', 'info');
}

// ===== Layer Visibility =====
document.getElementById('toggle-occupied').addEventListener('change', e => { occupiedGroup.visible = e.target.checked; });
document.getElementById('toggle-traversable').addEventListener('change', e => { traversableGroup.visible = e.target.checked; });
document.getElementById('toggle-preblocked').addEventListener('change', e => { preblockedGroup.visible = e.target.checked; });
document.getElementById('toggle-risk').addEventListener('change', e => { riskGroup.visible = e.target.checked; });

// Initial visibility
occupiedGroup.visible = false;
riskGroup.visible = false;

// ===== Button Event Wiring =====
function setActivePlacementBtn(mode) {
  if (mode && editMode) {
    editMode = false;
    const btn = document.getElementById('edit-mode-btn');
    btn.classList.remove('active');
    btn.textContent = '编辑地图';
    document.getElementById('edit-controls').hidden = true;
    levelTargetZ = null;
    clearBrushPreview();
    flushEdits();
    // delay reanalysis so ROS subscriber callbacks process pending edits first
    setTimeout(() => requestMap(), 500);
  }
  placementMode = mode;
  document.getElementById('set-start-btn').classList.toggle('active', mode === 'start');
  document.getElementById('set-goal-btn').classList.toggle('active', mode === 'goal');
  pickStatus.textContent = mode ? `点击地图设置${mode === 'start' ? '起点' : '终点'}` : '-';
}

document.getElementById('set-start-btn').addEventListener('click', () => setActivePlacementBtn(placementMode === 'start' ? null : 'start'));
document.getElementById('set-goal-btn').addEventListener('click', () => setActivePlacementBtn(placementMode === 'goal' ? null : 'goal'));

// 开始/恢复导航 → 解除 pathFollower 急停
document.getElementById('navigate-btn').addEventListener('click', () => {
  if (stopNavTopic) stopNavTopic.publish(new ROSLIB.Message({ data: false }));
  pickStatus.textContent = '导航执行中...';
  log('导航已启动/恢复', 'ok');
});

document.getElementById('stop-nav-btn').addEventListener('click', () => {
  publishStopNavigation();
  pickStatus.textContent = '导航已停止';
});

document.getElementById('reset-view-btn').addEventListener('click', () => {
  camera.position.set(10, 10, 15);
  camera.lookAt(0, 0, 0);
  controls.target.set(0, 0, 0);
  controls.update();
});

document.getElementById('connect-btn').addEventListener('click', () => {
  log('手动连接...', 'info');
  connectRos();
});
document.getElementById('fetch-map-btn').addEventListener('click', () => requestMap());

// Canvas interaction
canvas.addEventListener('pointerdown', onCanvasPointerDown);
canvas.addEventListener('pointermove', onCanvasPointerMove);
canvas.addEventListener('pointerup', onCanvasPointerUp);

// Joystick
joystickKnob.addEventListener('pointerdown', onJoystickDown);
joystickKnob.addEventListener('pointermove', onJoystickMove);
joystickKnob.addEventListener('pointerup', onJoystickUp);
joystickKnob.addEventListener('pointercancel', onJoystickUp);
rotationSlider.addEventListener('input', onRotationInput);
rotationSlider.addEventListener('change', onRotationRelease);

// ===== Edit Mode Toggle =====
document.getElementById('edit-mode-btn').addEventListener('click', () => {
  editMode = !editMode;
  const btn = document.getElementById('edit-mode-btn');
  const controls = document.getElementById('edit-controls');
  if (editMode) {
    setActivePlacementBtn(null);
    btn.classList.add('active');
    btn.textContent = '退出编辑';
    controls.hidden = false;
    levelTargetZ = null;
    updateLevelRangeUI();
    log('进入地面刷平模式（点击占据体素设定目标Z）', 'info');
  } else {
    btn.classList.remove('active');
    btn.textContent = '编辑地图';
    controls.hidden = true;
    levelTargetZ = null;
    clearBrushPreview();
    flushEdits();
    // delay reanalysis so ROS subscriber callbacks process pending edits first
    setTimeout(() => requestMap(), 500);
    log('退出地图编辑模式', 'info');
  }
});

document.getElementById('brush-size').addEventListener('change', e => {
  editBrushSize = Number(e.target.value);
  clearBrushPreview();
});

document.getElementById('level-range-slider').addEventListener('input', e => {
  levelRange = parseFloat(e.target.value);
  updateLevelRangeUI();
  clearBrushPreview();
});

// ===== File Browser =====
let saveFBPath = '/home';
let loadFBPath = '/home';
let loadSelectedPath = '';

async function apiCall(url) {
  const resp = await fetch(url);
  if (!resp.ok) {
    const text = await resp.text().catch(() => '');
    const isHtml = text.trimStart().startsWith('<!') || text.trimStart().startsWith('<html');
    const hint = isHtml
      ? '\n\n收到 HTML 响应，确认你用的是 python3 server.py 而非 python3 -m http.server'
      : '';
    throw new Error(`HTTP ${resp.status}${hint}`);
  }
  const ct = resp.headers.get('content-type') || '';
  if (!ct.includes('application/json')) {
    const text = await resp.text().catch(() => '');
    const isHtml = text.trimStart().startsWith('<!') || text.trimStart().startsWith('<html');
    if (isHtml) throw new Error('收到 HTML 而非 JSON。请使用 python3 server.py 而非 python3 -m http.server');
    throw new Error(`非 JSON 响应: ${text.substring(0, 120)}`);
  }
  return resp.json();
}

function showFbError(msg) {
  log(`文件浏览错误: ${msg}`, 'err');
  alert(`文件浏览失败\n${msg}\n\n请确认已停止旧服务，重新运行：\n  cd web && python3 server.py 8080`);
}

function fallbackTextInput(defaultPath, onConfirm) {
  const path = prompt('文件路径:', defaultPath);
  if (path && path.trim()) {
    onConfirm(path.trim());
  }
}

// ---- Save modal ----
async function openSaveFB(initialDir) {
  saveFBPath = initialDir || '/home';
  let data;
  try {
    data = await apiCall(`/api/list?dir=${encodeURIComponent(saveFBPath)}`);
  } catch (err) {
    console.error('openSaveFB failed:', err);
    showFbError(err.message);
    document.getElementById('save-modal').hidden = false;
    document.getElementById('save-fb-list').innerHTML = '';
    document.getElementById('save-fb-path').textContent = saveFBPath + ' (加载失败)';
    return;
  }
  if (data.error) { log(`目录错误: ${data.error}`, 'err'); return; }

  const listEl = document.getElementById('save-fb-list');
  const pathEl = document.getElementById('save-fb-path');

  renderFBList(listEl, pathEl, data, (e, li) => {
    if (e.type === 'dir') {
      saveFBPath = e.path;
      openSaveFB(saveFBPath);
    }
  }, saveFBPath);

  document.getElementById('save-fb-name').value = 'map_edited.bt';
  document.getElementById('save-modal').hidden = false;
}

function renderFBList(listEl, pathEl, data, onSelect, currentPath) {
  listEl.innerHTML = '';
  pathEl.textContent = currentPath || data.path;

  for (const e of data.entries) {
    const li = document.createElement('li');
    const icon = document.createElement('span');
    icon.className = 'fb-icon';
    icon.textContent = e.type === 'dir' ? '\u{1F4C1}' : '\u{1F4C4}';

    const name = document.createElement('span');
    name.className = 'fb-name';
    name.textContent = e.name;

    li.appendChild(icon);
    li.appendChild(name);

    if (e.type === 'file') {
      const size = document.createElement('span');
      size.className = 'fb-size';
      const kb = e.size / 1024;
      size.textContent = kb >= 1024 ? `${(kb / 1024).toFixed(1)} MB` : `${kb.toFixed(1)} KB`;
      li.appendChild(size);
    }

    li.addEventListener('click', () => onSelect(e, li));
    listEl.appendChild(li);
  }
}

document.getElementById('save-fb-up').addEventListener('click', async () => {
  let data;
  try { data = await apiCall(`/api/list?dir=${encodeURIComponent(saveFBPath)}`); }
  catch (err) { showFbError(err.message); return; }
  if (data.parent) {
    saveFBPath = data.parent;
    openSaveFB(saveFBPath);
  }
});

document.getElementById('save-map-btn').addEventListener('click', async () => {
  let roots;
  try { roots = await apiCall('/api/roots'); }
  catch (err) { showFbError(err.message); fallbackTextInput('/tmp/map_edited.bt', p => { if (saveMapTopic) { saveMapTopic.publish(new ROSLIB.Message({ data: p })); log(`保存地图: ${p}`, 'ok'); }}); return; }
  saveFBPath = (roots[1] || roots[0]).path;
  openSaveFB(saveFBPath);
});

document.getElementById('save-cancel').addEventListener('click', () => {
  document.getElementById('save-modal').hidden = true;
});

document.getElementById('save-confirm').addEventListener('click', () => {
  document.getElementById('save-modal').hidden = true;
  const fname = document.getElementById('save-fb-name').value.trim();
  if (!fname) { log('请输入文件名', 'warn'); return; }
  const fnameBT = fname.endsWith('.bt') ? fname : fname + '.bt';
  const fullPath = saveFBPath + '/' + fnameBT;
  if (saveMapTopic) {
    saveMapTopic.publish(new ROSLIB.Message({ data: fullPath }));
    log(`保存地图: ${fullPath}`, 'ok');
  }
});

// ---- Load modal ----
async function openLoadFB(initialDir) {
  loadFBPath = initialDir || '/home';
  let data;
  try {
    data = await apiCall(`/api/list?dir=${encodeURIComponent(loadFBPath)}`);
  } catch (err) {
    console.error('openLoadFB failed:', err);
    showFbError(err.message);
    document.getElementById('load-modal').hidden = false;
    document.getElementById('load-fb-list').innerHTML = '';
    document.getElementById('load-fb-path').textContent = loadFBPath + ' (加载失败)';
    return;
  }
  if (data.error) { log(`目录错误: ${data.error}`, 'err'); return; }

  const listEl = document.getElementById('load-fb-list');
  const pathEl = document.getElementById('load-fb-path');
  const confirmBtn = document.getElementById('load-confirm');
  const selInfo = document.getElementById('load-selected-info');

  renderFBList(listEl, pathEl, data, (e, li) => {
    listEl.querySelectorAll('.selected').forEach(el => el.classList.remove('selected'));

    if (e.type === 'dir') {
      loadFBPath = e.path;
      loadSelectedPath = '';
      confirmBtn.disabled = true;
      selInfo.textContent = '未选择文件';
      openLoadFB(loadFBPath);
    } else {
      li.classList.add('selected');
      loadSelectedPath = e.path;
      confirmBtn.disabled = false;
      selInfo.textContent = e.path;
    }
  }, loadFBPath);

  confirmBtn.disabled = true;
  selInfo.textContent = '未选择文件';
  document.getElementById('load-modal').hidden = false;
}

document.getElementById('load-fb-up').addEventListener('click', async () => {
  let data;
  try { data = await apiCall(`/api/list?dir=${encodeURIComponent(loadFBPath)}`); }
  catch (err) { showFbError(err.message); return; }
  if (data.parent) {
    loadFBPath = data.parent;
    loadSelectedPath = '';
    document.getElementById('load-confirm').disabled = true;
    document.getElementById('load-selected-info').textContent = '未选择文件';
    openLoadFB(loadFBPath);
  }
});

document.getElementById('load-map-btn').addEventListener('click', async () => {
  let roots;
  try { roots = await apiCall('/api/roots'); }
  catch (err) { showFbError(err.message); fallbackTextInput('/home/lenovo/Projects/NavProject/Dog3DNav/src/bringup/maps/map_nav3d.bt', p => { if (!loadMapTopic) { log('ROS 未连接，无法加载地图', 'err'); return; } resetMapData(); loadMapTopic.publish(new ROSLIB.Message({ data: p })); log(`加载地图: ${p}`, 'ok'); setTimeout(() => { if (requestMapService) requestMapService.callService({}, () => {}); }, 5000); }); return; }
  loadFBPath = (roots[1] || roots[0]).path;
  loadSelectedPath = '';
  openLoadFB(loadFBPath);
});

document.getElementById('load-cancel').addEventListener('click', () => {
  document.getElementById('load-modal').hidden = true;
});

document.getElementById('load-confirm').addEventListener('click', () => {
  document.getElementById('load-modal').hidden = true;
  if (!loadSelectedPath) { log('请先选择文件', 'warn'); return; }
  if (!loadMapTopic) { log('ROS 未连接，无法加载地图', 'err'); return; }
  resetMapData();
  loadMapTopic.publish(new ROSLIB.Message({ data: loadSelectedPath }));
  log(`加载地图: ${loadSelectedPath}`, 'ok');
  setTimeout(() => {
    if (requestMapService) requestMapService.callService({}, () => {});
  }, 5000);
});


// ===== Resize =====
function onResize() {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', onResize);

// ===== Animation Loop =====
let robotUpdateCounter = 0;
let stalenessCounter = 0;

function animate() {
  requestAnimationFrame(animate);
  controls.update();

  robotUpdateCounter++;
  if (robotUpdateCounter % 30 === 0) {
    updateRobotModel();
  }

  stalenessCounter++;
  if (stalenessCounter % 120 === 0) {
    checkStaleness();
  }

  renderer.render(scene, camera);
}

// ===== Boot =====
wsUrlInput.value = getDefaultWsUrl();
connectRos();
animate();

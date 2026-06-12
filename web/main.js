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
const navModal = document.getElementById('nav-confirm-modal');
const navConfirmMsg = document.getElementById('nav-confirm-msg');
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

// Placement mode
let placementMode = null; // 'start' | 'goal' | 'navigate' | null
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
let startNavTopic = null;
let stopNavTopic = null;
let cmdVelTopic = null;
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
const CHUNK_COLLECT_MS = 800;

function setOccupiedMarker(msg) {
  if (mapLoaded.occupied) return;
  if (!msg.points || msg.points.length === 0) return;
  if (msg.id === 0) occupiedPointsBuf = [];
  occupiedPointsBuf.push(...msg.points);
  if (occupiedRenderTimer) clearTimeout(occupiedRenderTimer);
  occupiedRenderTimer = setTimeout(() => {
    clearGroup(occupiedGroup);
    const { group } = makeVoxelLayer(occupiedPointsBuf, 0xff7043, 0.92);
    occupiedGroup.add(group);
    mapLoaded.occupied = true;
    log(`占据层: ${occupiedPointsBuf.length} 体素`, 'info');
    updateMapProgress();
    autoFrameCamera();
    occupiedRenderTimer = null;
  }, CHUNK_COLLECT_MS);
}

function setTraversableMarker(msg) {
  if (mapLoaded.traversable) return;
  if (!msg.points || msg.points.length === 0) return;
  if (msg.id === 0) traversablePointsBuf = [];
  traversablePointsBuf.push(...msg.points);
  if (traversableRenderTimer) clearTimeout(traversableRenderTimer);
  traversableRenderTimer = setTimeout(() => {
    clearGroup(traversableGroup);
    const { group, pickMesh } = makeVoxelLayer(traversablePointsBuf, 0x00e676, 0.28);
    traversableGroup.add(group);
    traversablePickMesh = pickMesh;
    mapLoaded.traversable = true;
    log(`可通行层: ${traversablePointsBuf.length} 体素`, 'info');
    updateMapProgress();
    traversableRenderTimer = null;
  }, CHUNK_COLLECT_MS);
}

function setPreblockedMarker(msg) {
  if (mapLoaded.preblocked) return;
  if (!msg.points || msg.points.length === 0) return;
  if (msg.id === 0) preblockedPointsBuf = [];
  preblockedPointsBuf.push(...msg.points);
  if (preblockedRenderTimer) clearTimeout(preblockedRenderTimer);
  preblockedRenderTimer = setTimeout(() => {
    clearGroup(preblockedGroup);
    const { group } = makeVoxelLayer(preblockedPointsBuf, 0xb388ff, 0.90);
    preblockedGroup.add(group);
    mapLoaded.preblocked = true;
    log(`禁行层: ${preblockedPointsBuf.length} 体素`, 'info');
    updateMapProgress();
    preblockedRenderTimer = null;
  }, CHUNK_COLLECT_MS);
}

function autoFrameCamera() {
  if (occupiedPointsBuf.length === 0) return;
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (const p of occupiedPointsBuf) {
    if (p.x < minX) minX = p.x;
    if (p.x > maxX) maxX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.y > maxY) maxY = p.y;
    if (p.z > maxZ) maxZ = p.z;
  }
  const cx = (minX + maxX) / 2;
  const cy = (minY + maxY) / 2;
  const span = Math.max(maxX - minX, maxY - minY);
  const dist = span * 0.8;
  camera.position.set(cx + dist * 0.5, cy - dist * 0.5, maxZ + dist * 0.6);
  controls.target.set(cx, cy, 0);
  controls.update();
}

function setRiskCostCloud(msg) {
  if (mapLoaded.risk) return;
  clearGroup(riskGroup);
  const points = parsePointCloud2(msg);
  if (!points || points.length === 0) return;
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
  log(`代价层: ${points.length} 点`, 'info');
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

  // Show navigation confirm if in navigate mode
  if (placementMode === 'navigate') {
    navConfirmMsg.textContent = `路径包含 ${msg.poses.length} 个航点。是否开始导航？`;
    navModal.hidden = false;
  }

  pickStatus.textContent = `路径: ${msg.poses.length} 点`;
  log(`规划路径: ${msg.poses.length} 个航点, 长度 ${(pts.length > 1 ? curve.getLength().toFixed(1) : '0')} m`, 'ok');
}

// ===== TF System =====
let tfFirstReceived = false;
function normalizeFrame(f) { return f.startsWith('/') ? f.slice(1) : f; }

function storeTransform(msg) {
  if (!msg.transforms) return;
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
  } else if (placementMode === 'goal' || placementMode === 'navigate') {
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
  if (goalTopic) {
    goalTopic.publish(new ROSLIB.Message({
      header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
      point: { x: pt.x, y: pt.y, z: pt.z }
    }));
  }
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

function publishNavigation(go) {
  if (!startNavTopic) return;
  startNavTopic.publish(new ROSLIB.Message({ data: go }));
  if (go) {
    const sp = startPoint || { x: '?', y: '?', z: '?' };
    const gp = goalPoint || { x: '?', y: '?', z: '?' };
    log(`开始导航: (${typeof sp.x === 'number' ? sp.x.toFixed(2) : sp.x}, ${typeof sp.y === 'number' ? sp.y.toFixed(2) : sp.y}) → (${typeof gp.x === 'number' ? gp.x.toFixed(2) : gp.x}, ${typeof gp.y === 'number' ? gp.y.toFixed(2) : gp.y})`, 'ok');
  } else {
    log('仅显示路线，未执行导航', 'info');
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
  startNavTopic = new ROSLIB.Topic({ ros, name: '/start_navigation', messageType: 'std_msgs/Bool' });
  stopNavTopic = new ROSLIB.Topic({ ros, name: '/stop_navigation', messageType: 'std_msgs/Bool' });
  cmdVelTopic = new ROSLIB.Topic({ ros, name: '/web_cmd_vel', messageType: 'geometry_msgs/Twist' });

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
  placementMode = mode;
  document.getElementById('set-start-btn').classList.toggle('active', mode === 'start');
  document.getElementById('set-goal-btn').classList.toggle('active', mode === 'goal');
  document.getElementById('navigate-btn').classList.toggle('active', mode === 'navigate');
  pickStatus.textContent = mode ? `点击地图设置${mode === 'start' ? '起点' : '终点'}` : '-';
}

document.getElementById('set-start-btn').addEventListener('click', () => setActivePlacementBtn(placementMode === 'start' ? null : 'start'));
document.getElementById('set-goal-btn').addEventListener('click', () => setActivePlacementBtn(placementMode === 'goal' ? null : 'goal'));
document.getElementById('navigate-btn').addEventListener('click', () => setActivePlacementBtn(placementMode === 'navigate' ? null : 'navigate'));

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

// Navigation modal
document.getElementById('nav-confirm-go').addEventListener('click', () => {
  navModal.hidden = true;
  publishNavigation(true);
  pickStatus.textContent = '导航执行中...';
});
document.getElementById('nav-confirm-cancel').addEventListener('click', () => {
  navModal.hidden = true;
  publishNavigation(false);
  pickStatus.textContent = '仅显示路线';
});

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

function animate() {
  requestAnimationFrame(animate);
  controls.update();

  robotUpdateCounter++;
  if (robotUpdateCounter % 30 === 0) {
    updateRobotModel();
  }

  renderer.render(scene, camera);
}

// ===== Boot =====
wsUrlInput.value = getDefaultWsUrl();
connectRos();
animate();

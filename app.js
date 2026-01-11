/**
 * Robot Arm Simulator
 * A production-ready 3D robot arm simulator with terminal control interface
 */

// ============================================================================
// CONFIGURATION
// ============================================================================

const CONFIG = {
    // Arm segment lengths (in world units)
    segments: {
        baseHeight: 0.4,
        shoulderLength: 1.5,  // Upper arm (shoulder to elbow)
        elbowLength: 1.5,     // Lower arm (elbow to wrist) - equal to upper arm
        wristLength: 0.4,
        gripperLength: 0.25
    },

    // Joint limits (in degrees)
    limits: {
        base: { min: -180, max: 180 },
        shoulder: { min: -90, max: 90 },
        elbow: { min: -135, max: 135 },
        wrist: { min: -90, max: 90 },
        wristRotate: { min: -180, max: 180 }  // Wrist roll/rotation around arm axis
    },

    // Animation settings
    animation: {
        duration: 1000, // ms
        easing: 'easeInOutCubic'
    },

    // Visual settings
    colors: {
        base: 0x2d3748,
        shoulder: 0x4a5568,
        elbow: 0x718096,
        wrist: 0x90cdf4,
        effector: 0xf56565,
        joint: 0x63b3ed,
        floor: 0x1a202c,
        grid: 0x2d3748
    }
};

// ============================================================================
// GLOBAL STATE
// ============================================================================

let scene, camera, renderer, controls;
let robotArm = {};
let isAnimating = false;
let animationQueue = [];
let commandHistory = [];
let historyIndex = -1;
let multiLineBuffer = [];  // Buffer for multi-line input with Shift+Enter

// Current joint angles (in radians)
const jointAngles = {
    base: 0,
    shoulder: 0,
    elbow: 0,
    wrist: 0,
    wristRotate: 0  // Wrist roll/rotation around arm axis
};

// Target joint angles for animation
const targetAngles = {
    base: 0,
    shoulder: 0,
    elbow: 0,
    wrist: 0,
    wristRotate: 0
};

// Gripper state (0 = closed, 100 = fully open)
let gripperOpenness = 50;  // Start half open
let targetGripperOpenness = 50;
let gripperAnimating = false;

// ============================================================================
// PHYSICS OBJECTS
// ============================================================================

const sceneObjects = [];  // Array of physics objects
let grippedObject = null; // Currently gripped object
let lastFrameTime = performance.now();

class PhysicsObject {
    constructor(mesh, type, size, name) {
        this.mesh = mesh;
        this.type = type;
        this.size = size;  // { x, y, z } bounding dimensions
        this.name = name;
        this.velocity = new THREE.Vector3(0, 0, 0);
        this.isGripped = false;
        this.grippedOffset = null;  // Offset from gripper when gripped
        this.grippedRotation = null; // Rotation relative to gripper when gripped
    }

    getRadius() {
        return Math.max(this.size.x, this.size.z) / 2;
    }

    getHalfHeight() {
        return this.size.y / 2;
    }
}

// Object colors for variety
const OBJECT_COLORS = [
    0xe53e3e, // red
    0xdd6b20, // orange
    0xd69e2e, // yellow
    0x38a169, // green
    0x3182ce, // blue
    0x805ad5, // purple
    0xd53f8c, // pink
    0x319795  // teal
];

let objectCounter = 0;

// ============================================================================
// EASING FUNCTIONS
// ============================================================================

const easingFunctions = {
    linear: t => t,
    easeInQuad: t => t * t,
    easeOutQuad: t => t * (2 - t),
    easeInOutQuad: t => t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t,
    easeInCubic: t => t * t * t,
    easeOutCubic: t => (--t) * t * t + 1,
    easeInOutCubic: t => t < 0.5 ? 4 * t * t * t : (t - 1) * (2 * t - 2) * (2 * t - 2) + 1,
    easeInOutExpo: t => {
        if (t === 0 || t === 1) return t;
        if (t < 0.5) return Math.pow(2, 20 * t - 10) / 2;
        return (2 - Math.pow(2, -20 * t + 10)) / 2;
    }
};

// ============================================================================
// THREE.JS SETUP
// ============================================================================

function initScene() {
    const canvas = document.getElementById('canvas');
    const container = document.getElementById('canvas-container');

    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0d1117);
    scene.fog = new THREE.Fog(0x0d1117, 15, 30);

    // Camera
    camera = new THREE.PerspectiveCamera(
        50,
        container.clientWidth / container.clientHeight,
        0.1,
        100
    );
    camera.position.set(6, 4, 6);
    camera.lookAt(0, 1.5, 0);

    // Renderer
    renderer = new THREE.WebGLRenderer({
        canvas,
        antialias: true,
        alpha: true
    });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.2;

    // Controls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.minDistance = 3;
    controls.maxDistance = 20;
    controls.maxPolarAngle = Math.PI / 2 + 0.1;
    controls.target.set(0, 1.5, 0);

    // Lighting
    setupLighting();

    // Floor
    createFloor();

    // Robot arm
    createRobotArm();

    // Handle resize
    window.addEventListener('resize', onWindowResize);

    // Start animation loop
    animate();
}

function setupLighting() {
    // Ambient light
    const ambient = new THREE.AmbientLight(0x404040, 0.5);
    scene.add(ambient);

    // Main directional light
    const mainLight = new THREE.DirectionalLight(0xffffff, 1);
    mainLight.position.set(5, 10, 5);
    mainLight.castShadow = true;
    mainLight.shadow.mapSize.width = 2048;
    mainLight.shadow.mapSize.height = 2048;
    mainLight.shadow.camera.near = 0.5;
    mainLight.shadow.camera.far = 30;
    mainLight.shadow.camera.left = -10;
    mainLight.shadow.camera.right = 10;
    mainLight.shadow.camera.top = 10;
    mainLight.shadow.camera.bottom = -10;
    mainLight.shadow.bias = -0.0001;
    scene.add(mainLight);

    // Fill light
    const fillLight = new THREE.DirectionalLight(0x4a90d9, 0.3);
    fillLight.position.set(-5, 5, -5);
    scene.add(fillLight);

    // Rim light
    const rimLight = new THREE.DirectionalLight(0xff6b6b, 0.2);
    rimLight.position.set(0, 5, -10);
    scene.add(rimLight);

    // Point light near base
    const pointLight = new THREE.PointLight(0x63b3ed, 0.5, 10);
    pointLight.position.set(0, 0.5, 0);
    scene.add(pointLight);
}

function createFloor() {
    // Floor plane
    const floorGeometry = new THREE.PlaneGeometry(20, 20);
    const floorMaterial = new THREE.MeshStandardMaterial({
        color: CONFIG.colors.floor,
        roughness: 0.8,
        metalness: 0.2
    });
    const floor = new THREE.Mesh(floorGeometry, floorMaterial);
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    scene.add(floor);

    // Create numbered grid
    // 1 unit = 100cm, grid shows every 50cm with numbers every 100cm
    const gridSize = 10; // 10 units = 10 meters total (5m each direction)
    const majorStep = 1; // Major lines every 100cm (1 unit)
    const minorStep = 0.5; // Minor lines every 50cm

    // Minor grid lines (50cm intervals) - lighter
    const minorGridHelper = new THREE.GridHelper(gridSize * 2, gridSize * 2 / minorStep, 0x2d3748, 0x2d3748);
    minorGridHelper.material.opacity = 0.2;
    minorGridHelper.material.transparent = true;
    scene.add(minorGridHelper);

    // Major grid lines (100cm intervals) - brighter
    const majorGridHelper = new THREE.GridHelper(gridSize * 2, gridSize * 2 / majorStep, 0x4a5568, 0x4a5568);
    majorGridHelper.material.opacity = 0.4;
    majorGridHelper.material.transparent = true;
    majorGridHelper.position.y = 0.001;
    scene.add(majorGridHelper);

    // Create text labels for coordinates
    // Position labels slightly above floor so arm can occlude them
    const labelHeight = 0.02;

    // Y-axis labels (forward/back) - along Z in Three.js
    for (let i = -gridSize; i <= gridSize; i += majorStep) {
        if (i === 0) continue; // Skip center
        const labelText = `${i * 100}`;
        const label = createTextLabel(labelText, i >= 0 ? '#7ee787' : '#f0883e');
        label.position.set(-0.15, labelHeight, i);
        scene.add(label);
    }

    // X-axis labels (left/right)
    for (let i = -gridSize; i <= gridSize; i += majorStep) {
        if (i === 0) continue; // Skip center
        const labelText = `${i * 100}`;
        const label = createTextLabel(labelText, i >= 0 ? '#a5d6ff' : '#f0883e');
        label.position.set(i, labelHeight, -0.15);
        scene.add(label);
    }

    // Center marker with "0" label
    const markerGeometry = new THREE.RingGeometry(0.15, 0.18, 32);
    const markerMaterial = new THREE.MeshBasicMaterial({
        color: 0x63b3ed,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.7
    });
    const marker = new THREE.Mesh(markerGeometry, markerMaterial);
    marker.rotation.x = -Math.PI / 2;
    marker.position.y = 0.01;
    scene.add(marker);

    const centerLabel = createTextLabel('0', '#63b3ed', 36);
    centerLabel.position.set(-0.25, labelHeight, -0.25);
    scene.add(centerLabel);

    // Reach circle indicator (approximate working radius)
    const reachRadius = CONFIG.segments.shoulderLength + CONFIG.segments.elbowLength;
    const reachGeometry = new THREE.RingGeometry(reachRadius - 0.02, reachRadius + 0.02, 64);
    const reachMaterial = new THREE.MeshBasicMaterial({
        color: 0x58a6ff,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.3
    });
    const reachCircle = new THREE.Mesh(reachGeometry, reachMaterial);
    reachCircle.rotation.x = -Math.PI / 2;
    reachCircle.position.y = 0.005;
    scene.add(reachCircle);
}

// Helper function to create text labels using canvas
function createTextLabel(text, color = '#ffffff', fontSize = 32) {
    const canvas = document.createElement('canvas');
    const size = 128;
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext('2d');

    ctx.fillStyle = color;
    ctx.font = `bold ${fontSize}px Consolas, Monaco, monospace`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, size / 2, size / 2);

    const texture = new THREE.CanvasTexture(canvas);
    texture.minFilter = THREE.LinearFilter;

    const material = new THREE.SpriteMaterial({
        map: texture,
        transparent: true
    });

    const sprite = new THREE.Sprite(material);
    sprite.scale.set(0.3, 0.3, 1);

    return sprite;
}

// ============================================================================
// ROBOT ARM CREATION
// ============================================================================

function createRobotArm() {
    // Materials
    const baseMaterial = new THREE.MeshStandardMaterial({
        color: CONFIG.colors.base,
        roughness: 0.3,
        metalness: 0.8
    });

    const armMaterial = new THREE.MeshStandardMaterial({
        color: CONFIG.colors.shoulder,
        roughness: 0.4,
        metalness: 0.6
    });

    const jointMaterial = new THREE.MeshStandardMaterial({
        color: CONFIG.colors.joint,
        roughness: 0.3,
        metalness: 0.7,
        emissive: CONFIG.colors.joint,
        emissiveIntensity: 0.1
    });

    const effectorMaterial = new THREE.MeshStandardMaterial({
        color: CONFIG.colors.effector,
        roughness: 0.3,
        metalness: 0.5,
        emissive: CONFIG.colors.effector,
        emissiveIntensity: 0.2
    });

    // Base platform
    const baseGeometry = new THREE.CylinderGeometry(0.6, 0.7, 0.15, 32);
    const basePlatform = new THREE.Mesh(baseGeometry, baseMaterial);
    basePlatform.position.y = 0.075;
    basePlatform.castShadow = true;
    basePlatform.receiveShadow = true;
    scene.add(basePlatform);

    // Base pivot (rotates around Y axis)
    robotArm.basePivot = new THREE.Group();
    robotArm.basePivot.position.y = 0.15;
    scene.add(robotArm.basePivot);

    // Base column
    const columnGeometry = new THREE.CylinderGeometry(0.2, 0.25, CONFIG.segments.baseHeight, 16);
    const column = new THREE.Mesh(columnGeometry, armMaterial);
    column.position.y = CONFIG.segments.baseHeight / 2;
    column.castShadow = true;
    robotArm.basePivot.add(column);

    // Shoulder joint visual
    const shoulderJointGeometry = new THREE.SphereGeometry(0.25, 16, 16);
    const shoulderJoint = new THREE.Mesh(shoulderJointGeometry, jointMaterial);
    shoulderJoint.position.y = CONFIG.segments.baseHeight;
    shoulderJoint.castShadow = true;
    robotArm.basePivot.add(shoulderJoint);

    // Shoulder pivot (rotates around X axis)
    robotArm.shoulderPivot = new THREE.Group();
    robotArm.shoulderPivot.position.y = CONFIG.segments.baseHeight;
    robotArm.basePivot.add(robotArm.shoulderPivot);

    // Upper arm
    const upperArmGeometry = new THREE.BoxGeometry(0.15, CONFIG.segments.shoulderLength, 0.15);
    upperArmGeometry.translate(0, CONFIG.segments.shoulderLength / 2, 0);
    const upperArm = new THREE.Mesh(upperArmGeometry, armMaterial);
    upperArm.castShadow = true;
    robotArm.shoulderPivot.add(upperArm);

    // Elbow joint visual
    const elbowJointGeometry = new THREE.SphereGeometry(0.2, 16, 16);
    const elbowJoint = new THREE.Mesh(elbowJointGeometry, jointMaterial);
    elbowJoint.position.y = CONFIG.segments.shoulderLength;
    elbowJoint.castShadow = true;
    robotArm.shoulderPivot.add(elbowJoint);

    // Elbow pivot
    robotArm.elbowPivot = new THREE.Group();
    robotArm.elbowPivot.position.y = CONFIG.segments.shoulderLength;
    robotArm.shoulderPivot.add(robotArm.elbowPivot);

    // Lower arm
    const lowerArmGeometry = new THREE.BoxGeometry(0.12, CONFIG.segments.elbowLength, 0.12);
    lowerArmGeometry.translate(0, CONFIG.segments.elbowLength / 2, 0);
    const lowerArm = new THREE.Mesh(lowerArmGeometry, new THREE.MeshStandardMaterial({
        color: CONFIG.colors.elbow,
        roughness: 0.4,
        metalness: 0.6
    }));
    lowerArm.castShadow = true;
    robotArm.elbowPivot.add(lowerArm);

    // Wrist joint visual
    const wristJointGeometry = new THREE.SphereGeometry(0.15, 16, 16);
    const wristJoint = new THREE.Mesh(wristJointGeometry, jointMaterial);
    wristJoint.position.y = CONFIG.segments.elbowLength;
    wristJoint.castShadow = true;
    robotArm.elbowPivot.add(wristJoint);

    // Wrist pivot
    robotArm.wristPivot = new THREE.Group();
    robotArm.wristPivot.position.y = CONFIG.segments.elbowLength;
    robotArm.elbowPivot.add(robotArm.wristPivot);

    // Wrist/hand segment
    const wristGeometry = new THREE.BoxGeometry(0.1, CONFIG.segments.wristLength, 0.1);
    wristGeometry.translate(0, CONFIG.segments.wristLength / 2, 0);
    const wrist = new THREE.Mesh(wristGeometry, new THREE.MeshStandardMaterial({
        color: CONFIG.colors.wrist,
        roughness: 0.4,
        metalness: 0.5
    }));
    wrist.castShadow = true;
    robotArm.wristPivot.add(wrist);

    // Wrist rotate pivot (rotates around Y axis - the arm's longitudinal axis)
    robotArm.wristRotatePivot = new THREE.Group();
    robotArm.wristRotatePivot.position.y = CONFIG.segments.wristLength;
    robotArm.wristPivot.add(robotArm.wristRotatePivot);

    // Gripper base (mounting plate)
    const gripperBaseGeometry = new THREE.CylinderGeometry(0.08, 0.1, 0.06, 16);
    const gripperBase = new THREE.Mesh(gripperBaseGeometry, effectorMaterial);
    gripperBase.position.y = 0.03;
    gripperBase.castShadow = true;
    robotArm.wristRotatePivot.add(gripperBase);

    // Gripper clamps (bar style)
    robotArm.gripperGroup = new THREE.Group();
    robotArm.gripperGroup.position.y = 0.06;
    robotArm.wristRotatePivot.add(robotArm.gripperGroup);

    const clampMaterial = new THREE.MeshStandardMaterial({
        color: 0xe53e3e,
        roughness: 0.3,
        metalness: 0.7
    });

    const clampAccentMaterial = new THREE.MeshStandardMaterial({
        color: 0xc53030,
        roughness: 0.4,
        metalness: 0.6
    });

    // Clamp dimensions
    const clampLength = CONFIG.segments.gripperLength * 1.0;  // Vertical length
    const clampWidth = 0.03;   // Thickness (X direction)
    const clampDepth = 0.08;   // Depth (Z direction)

    // Left clamp bar
    const leftClampGroup = new THREE.Group();
    leftClampGroup.position.x = -0.05;  // Initial position

    // Main vertical bar
    const leftBar = new THREE.Mesh(
        new THREE.BoxGeometry(clampWidth, clampLength, clampDepth),
        clampMaterial
    );
    leftBar.position.y = clampLength / 2;
    leftBar.castShadow = true;
    leftClampGroup.add(leftBar);

    // Inner gripping surface (rubber-like pad)
    const leftPad = new THREE.Mesh(
        new THREE.BoxGeometry(0.015, clampLength * 0.7, clampDepth - 0.01),
        clampAccentMaterial
    );
    leftPad.position.set(clampWidth / 2 + 0.007, clampLength * 0.5, 0);
    leftClampGroup.add(leftPad);

    robotArm.gripperGroup.add(leftClampGroup);

    // Right clamp bar (mirror)
    const rightClampGroup = new THREE.Group();
    rightClampGroup.position.x = 0.05;  // Initial position

    // Main vertical bar
    const rightBar = new THREE.Mesh(
        new THREE.BoxGeometry(clampWidth, clampLength, clampDepth),
        clampMaterial
    );
    rightBar.position.y = clampLength / 2;
    rightBar.castShadow = true;
    rightClampGroup.add(rightBar);

    // Inner gripping surface (rubber-like pad)
    const rightPad = new THREE.Mesh(
        new THREE.BoxGeometry(0.015, clampLength * 0.7, clampDepth - 0.01),
        clampAccentMaterial
    );
    rightPad.position.set(-clampWidth / 2 - 0.007, clampLength * 0.5, 0);
    rightClampGroup.add(rightPad);

    robotArm.gripperGroup.add(rightClampGroup);

    // Store clamp references for animation
    robotArm.leftFinger = leftClampGroup;
    robotArm.rightFinger = rightClampGroup;

    // Create end effector position marker (at clamp tips)
    // Clamps extend from y=0 to y=gripperLength, so tip is at gripperLength
    robotArm.endEffectorMarker = new THREE.Object3D();
    robotArm.endEffectorMarker.position.y = CONFIG.segments.gripperLength;
    robotArm.gripperGroup.add(robotArm.endEffectorMarker);

    // Initial position - upright ready stance
    // Arm reaching up and forward with gripper pointing outward
    setJointAngles(180, -20, 90, 45, false);

    // Initialize gripper position (50% open)
    applyGripperOpenness();
}

// ============================================================================
// PHYSICS OBJECTS - CREATION AND MANAGEMENT
// ============================================================================

function createObject(type, x, y, z, sizeMm = 80) {
    // x, y, z are in robotics coordinates (cm): X=left/right, Y=forward, Z=height
    // Convert to Three.js coordinates (meters): X=same, Y=height, Z=forward
    const posX = x / 100;  // cm to meters
    const posY = z / 100;  // Z (height) becomes Y in Three.js
    const posZ = y / 100;  // Y (forward) becomes Z in Three.js

    let geometry, size;
    const objectSize = sizeMm / 1000;  // Convert mm to meters

    switch (type) {
        case 'cylinder':
            geometry = new THREE.CylinderGeometry(objectSize / 2, objectSize / 2, objectSize, 16);
            size = { x: objectSize, y: objectSize, z: objectSize };
            break;
        case 'sphere':
            geometry = new THREE.SphereGeometry(objectSize / 2, 16, 16);
            size = { x: objectSize, y: objectSize, z: objectSize };
            break;
        case 'cube':
        default:
            geometry = new THREE.BoxGeometry(objectSize, objectSize, objectSize);
            size = { x: objectSize, y: objectSize, z: objectSize };
            type = 'cube';
            break;
    }

    const color = OBJECT_COLORS[objectCounter % OBJECT_COLORS.length];
    const material = new THREE.MeshStandardMaterial({
        color: color,
        roughness: 0.4,
        metalness: 0.3
    });

    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    // Position the object (ensure it's above floor)
    const halfHeight = size.y / 2;
    mesh.position.set(posX, Math.max(posY, halfHeight), posZ);

    scene.add(mesh);

    objectCounter++;
    const name = `${type}_${objectCounter}`;

    const physicsObj = new PhysicsObject(mesh, type, size, name);
    sceneObjects.push(physicsObj);

    return physicsObj;
}

function removeObject(obj) {
    const index = sceneObjects.indexOf(obj);
    if (index > -1) {
        scene.remove(obj.mesh);
        obj.mesh.geometry.dispose();
        obj.mesh.material.dispose();
        sceneObjects.splice(index, 1);
        if (grippedObject === obj) {
            grippedObject = null;
        }
    }
}

function removeAllObjects() {
    while (sceneObjects.length > 0) {
        removeObject(sceneObjects[0]);
    }
    objectCounter = 0;
}

// ============================================================================
// PHYSICS UPDATE
// ============================================================================

function getGripperFingerTips() {
    // Update world matrices
    robotArm.leftFinger.updateWorldMatrix(true, false);
    robotArm.rightFinger.updateWorldMatrix(true, false);

    // Create points at finger tip positions (local y = gripperLength)
    const tipY = CONFIG.segments.gripperLength;

    const leftTip = new THREE.Vector3(0, tipY, 0);
    const rightTip = new THREE.Vector3(0, tipY, 0);

    robotArm.leftFinger.localToWorld(leftTip);
    robotArm.rightFinger.localToWorld(rightTip);

    return { left: leftTip, right: rightTip };
}

function getGripperCenter() {
    robotArm.endEffectorMarker.updateWorldMatrix(true, false);
    const center = new THREE.Vector3();
    robotArm.endEffectorMarker.getWorldPosition(center);
    return center;
}

function checkGripperContact(obj) {
    const tips = getGripperFingerTips();
    const objPos = obj.mesh.position;
    const radius = obj.getRadius();

    // Distance from each finger tip to object center
    const leftDist = tips.left.distanceTo(objPos);
    const rightDist = tips.right.distanceTo(objPos);

    // Contact threshold: object radius + finger pad width
    const contactThreshold = radius + 0.025;

    return {
        leftContact: leftDist < contactThreshold,
        rightContact: rightDist < contactThreshold,
        bothContact: leftDist < contactThreshold && rightDist < contactThreshold,
        leftDist: leftDist,
        rightDist: rightDist
    };
}

function applyGripperPush(obj, contact) {
    if (obj.isGripped) return;

    const tips = getGripperFingerTips();
    const objPos = obj.mesh.position;

    // If only one finger touching, push away from that finger
    if (contact.leftContact && !contact.rightContact) {
        const pushDir = objPos.clone().sub(tips.left).normalize();
        pushDir.y = 0;  // Keep push horizontal
        obj.mesh.position.add(pushDir.multiplyScalar(0.015));
        obj.velocity.x = pushDir.x * 0.5;
        obj.velocity.z = pushDir.z * 0.5;
    }
    if (contact.rightContact && !contact.leftContact) {
        const pushDir = objPos.clone().sub(tips.right).normalize();
        pushDir.y = 0;
        obj.mesh.position.add(pushDir.multiplyScalar(0.015));
        obj.velocity.x = pushDir.x * 0.5;
        obj.velocity.z = pushDir.z * 0.5;
    }
}

function gripObject(obj) {
    if (obj.isGripped || grippedObject) return;

    obj.isGripped = true;
    grippedObject = obj;

    // Store offset from gripper center
    const gripperCenter = getGripperCenter();
    obj.grippedOffset = obj.mesh.position.clone().sub(gripperCenter);

    // Store rotation relative to gripper
    const gripperQuat = new THREE.Quaternion();
    robotArm.gripperGroup.getWorldQuaternion(gripperQuat);
    const gripperQuatInv = gripperQuat.clone().invert();
    obj.grippedRotation = obj.mesh.quaternion.clone().premultiply(gripperQuatInv);

    // Stop any velocity
    obj.velocity.set(0, 0, 0);

    terminal.print(`Gripped ${obj.name}`, 'success');
}

function releaseObject() {
    if (!grippedObject) return;

    const obj = grippedObject;
    obj.isGripped = false;
    obj.grippedOffset = null;
    obj.grippedRotation = null;
    grippedObject = null;

    // Give a slight downward velocity when released
    obj.velocity.set(0, -0.1, 0);

    terminal.print(`Released ${obj.name}`, 'info');
}

function updateGrippedObjectPosition(obj) {
    if (!obj.isGripped || !obj.grippedOffset) return;

    // Get gripper world position and rotation
    const gripperCenter = getGripperCenter();
    const gripperQuat = new THREE.Quaternion();
    robotArm.gripperGroup.getWorldQuaternion(gripperQuat);

    // Apply stored offset rotated by current gripper orientation
    const rotatedOffset = obj.grippedOffset.clone().applyQuaternion(gripperQuat);
    obj.mesh.position.copy(gripperCenter).add(rotatedOffset);

    // Update rotation to match gripper
    obj.mesh.quaternion.copy(gripperQuat).multiply(obj.grippedRotation);
}

function updatePhysics(deltaTime) {
    // Cap deltaTime to prevent huge jumps
    deltaTime = Math.min(deltaTime, 0.1);

    for (const obj of sceneObjects) {
        if (obj.isGripped) {
            updateGrippedObjectPosition(obj);
        } else {
            // Apply gravity
            obj.velocity.y -= 9.8 * deltaTime;

            // Apply velocity
            obj.mesh.position.x += obj.velocity.x * deltaTime;
            obj.mesh.position.y += obj.velocity.y * deltaTime;
            obj.mesh.position.z += obj.velocity.z * deltaTime;

            // Floor collision
            const halfHeight = obj.getHalfHeight();
            if (obj.mesh.position.y < halfHeight) {
                obj.mesh.position.y = halfHeight;
                obj.velocity.y = 0;
                // Apply friction
                obj.velocity.x *= 0.9;
                obj.velocity.z *= 0.9;
                if (Math.abs(obj.velocity.x) < 0.001) obj.velocity.x = 0;
                if (Math.abs(obj.velocity.z) < 0.001) obj.velocity.z = 0;
            }

            // Check for gripper contact (push physics)
            const contact = checkGripperContact(obj);
            if (contact.leftContact || contact.rightContact) {
                applyGripperPush(obj, contact);
            }
        }
    }
}

// ============================================================================
// KINEMATICS
// ============================================================================

function setJointAngles(base, shoulder, elbow, wrist, animated = true) {
    // Convert to radians and clamp to limits
    const baseRad = clampAngle(base, CONFIG.limits.base) * Math.PI / 180;
    const shoulderRad = clampAngle(shoulder, CONFIG.limits.shoulder) * Math.PI / 180;
    const elbowRad = clampAngle(elbow, CONFIG.limits.elbow) * Math.PI / 180;
    const wristRad = clampAngle(wrist, CONFIG.limits.wrist) * Math.PI / 180;

    if (animated) {
        targetAngles.base = baseRad;
        targetAngles.shoulder = shoulderRad;
        targetAngles.elbow = elbowRad;
        targetAngles.wrist = wristRad;
        startAnimation();
    } else {
        jointAngles.base = baseRad;
        jointAngles.shoulder = shoulderRad;
        jointAngles.elbow = elbowRad;
        jointAngles.wrist = wristRad;
        applyJointAngles();
    }
}

function clampAngle(angle, limits) {
    return Math.max(limits.min, Math.min(limits.max, angle));
}

function applyJointAngles() {
    robotArm.basePivot.rotation.y = jointAngles.base;
    robotArm.shoulderPivot.rotation.z = jointAngles.shoulder;
    robotArm.elbowPivot.rotation.z = jointAngles.elbow;
    robotArm.wristPivot.rotation.z = jointAngles.wrist;
    robotArm.wristRotatePivot.rotation.y = jointAngles.wristRotate;

    updateUI();
}

// Apply gripper openness to finger positions
// 0 = fully closed (fingers together), 100 = fully open (fingers spread)
function applyGripperOpenness() {
    if (!robotArm.leftFinger || !robotArm.rightFinger) return;

    // Map 0-100 to finger X positions
    // Closed: fingers at ±0.02 (nearly touching)
    // Open: fingers at ±0.12 (spread wide)
    const minOffset = 0.02;
    const maxOffset = 0.12;
    const offset = minOffset + (maxOffset - minOffset) * (gripperOpenness / 100);

    robotArm.leftFinger.position.x = -offset;
    robotArm.rightFinger.position.x = offset;
}

// Set gripper openness with animation
function setGripperOpenness(percent, animated = true) {
    const clampedPercent = Math.max(0, Math.min(100, percent));

    if (animated) {
        targetGripperOpenness = clampedPercent;
        if (!gripperAnimating) {
            gripperAnimating = true;
        }
    } else {
        gripperOpenness = clampedPercent;
        targetGripperOpenness = clampedPercent;
        applyGripperOpenness();
    }
}

function getEndEffectorPosition() {
    // Update world matrix
    robotArm.endEffectorMarker.updateWorldMatrix(true, false);
    const position = new THREE.Vector3();
    robotArm.endEffectorMarker.getWorldPosition(position);
    // Convert from Three.js (Y-up) to robotics convention (Z-up)
    // Return as { x, y, z } where z is height
    return {
        x: position.x,
        y: position.z,
        z: position.y
    };
}

// Inverse Kinematics solver
// Uses robotics convention: X = left/right, Y = forward/backward, Z = height (up)
// Targets the GRIPPER TIP position with gripper pointing straight down
//
// COORDINATE SYSTEM:
// - In Three.js, rotation.z = 0 means arm points straight UP (along +Y)
// - rotation.z = θ tilts the arm θ radians FROM VERTICAL
// - For gripper pointing down: shoulder + elbow + wrist = π (180°)
//
// 2D IK GEOMETRY (in vertical plane after base rotation):
// - r = horizontal distance from base axis
// - h = height relative to shoulder pivot
// - Angles measured from VERTICAL (not horizontal!)
function solveIK(targetX, targetY, targetZ, maxIterations = 50) {
    // Arm segment lengths
    const L1 = CONFIG.segments.shoulderLength;  // Upper arm: 1.5 units
    const L2 = CONFIG.segments.elbowLength;     // Lower arm: 1.5 units
    // L3 = distance from wrist pivot to clamp tips
    const L3 = CONFIG.segments.wristLength + 0.06 + CONFIG.segments.gripperLength;
    const baseHeight = CONFIG.segments.baseHeight + 0.15; // Shoulder pivot height

    const armReach = L1 + L2;

    // Check bounds - don't allow below floor
    if (targetZ < 0) {
        return { success: false, error: 'Target is below the floor (Z must be >= 0)' };
    }

    // Calculate base rotation angle (rotation around vertical axis)
    // The arm extends in local -X direction when shoulder tilts, so we need to
    // rotate base such that local -X points toward the target direction
    // atan2(y, x) gives angle from +X axis, so we use atan2(targetY, targetX)
    let baseAngle = Math.PI - Math.atan2(targetY, targetX);
    // Normalize to -π to π range to stay within joint limits
    while (baseAngle > Math.PI) baseAngle -= 2 * Math.PI;
    while (baseAngle < -Math.PI) baseAngle += 2 * Math.PI;

    // Work in 2D plane (r = horizontal distance, h = height relative to shoulder)
    const r = Math.sqrt(targetX * targetX + targetY * targetY);
    const h = targetZ - baseHeight;

    // For gripper pointing straight down, wrist must be directly above target
    // Wrist height = target height + L3 (gripper hangs down from wrist)
    const wristR = r;                    // Horizontal distance to wrist
    const wristH = h + L3;               // Height of wrist relative to shoulder
    const wristDist = Math.sqrt(wristR * wristR + wristH * wristH);

    // Check reachability
    if (wristDist > armReach * 0.98) {
        return { success: false, error: `Target too far (need ${(wristDist * 100).toFixed(0)}cm, arm reach: ${(armReach * 100).toFixed(0)}cm)` };
    }

    if (wristDist < 0.1) {
        return { success: false, error: 'Target is too close to base' };
    }

    // Solve 2-link IK using law of cosines
    // Internal elbow angle (angle of the triangle at elbow vertex)
    let cosElbowInternal = (L1 * L1 + L2 * L2 - wristDist * wristDist) / (2 * L1 * L2);
    cosElbowInternal = Math.max(-1, Math.min(1, cosElbowInternal));
    const elbowInternalAngle = Math.acos(cosElbowInternal);

    // Angle from VERTICAL to the shoulder->wrist line
    // atan2(horizontal, vertical) gives angle from vertical axis
    const angleToWristFromVertical = Math.atan2(wristR, wristH);

    // Angle at shoulder vertex (between upper arm and shoulder->wrist line)
    let cosAlpha = (L1 * L1 + wristDist * wristDist - L2 * L2) / (2 * L1 * wristDist);
    cosAlpha = Math.max(-1, Math.min(1, cosAlpha));
    const alpha = Math.acos(cosAlpha);

    // For gripper to point STRAIGHT DOWN, cumulative angle must be 180° (π radians)
    // cumulative = shoulder + elbow + wrist = π (all measured from vertical)
    const desiredGripperAngle = Math.PI; // 180° = pointing straight down

    // Get joint limits in radians
    const shoulderLimitMin = CONFIG.limits.shoulder.min * Math.PI / 180;
    const shoulderLimitMax = CONFIG.limits.shoulder.max * Math.PI / 180;
    const elbowLimitMin = CONFIG.limits.elbow.min * Math.PI / 180;
    const elbowLimitMax = CONFIG.limits.elbow.max * Math.PI / 180;
    const wristLimitMin = CONFIG.limits.wrist.min * Math.PI / 180;
    const wristLimitMax = CONFIG.limits.wrist.max * Math.PI / 180;

    // Try both elbow configurations and pick the valid one
    const configs = [];

    // Configuration 1: Elbow-down (elbow below the shoulder-wrist line)
    // Upper arm tilts MORE than the line to wrist
    // Lower arm bends BACK toward vertical, so elbow angle is NEGATIVE
    const shoulderDown = angleToWristFromVertical + alpha;
    const elbowDown = -(Math.PI - elbowInternalAngle);  // Negative bend (back toward vertical)
    const wristDown = desiredGripperAngle - shoulderDown - elbowDown;

    if (shoulderDown >= shoulderLimitMin && shoulderDown <= shoulderLimitMax &&
        elbowDown >= elbowLimitMin && elbowDown <= elbowLimitMax &&
        wristDown >= wristLimitMin && wristDown <= wristLimitMax) {
        configs.push({ shoulder: shoulderDown, elbow: elbowDown, wrist: wristDown, name: 'elbow-down' });
    }

    // Configuration 2: Elbow-up (elbow above the shoulder-wrist line)
    // Upper arm tilts LESS than the line to wrist
    // Lower arm bends FORWARD away from vertical, so elbow angle is POSITIVE
    const shoulderUp = angleToWristFromVertical - alpha;
    const elbowUp = Math.PI - elbowInternalAngle;  // Positive bend (forward)
    const wristUp = desiredGripperAngle - shoulderUp - elbowUp;

    if (shoulderUp >= shoulderLimitMin && shoulderUp <= shoulderLimitMax &&
        elbowUp >= elbowLimitMin && elbowUp <= elbowLimitMax &&
        wristUp >= wristLimitMin && wristUp <= wristLimitMax) {
        configs.push({ shoulder: shoulderUp, elbow: elbowUp, wrist: wristUp, name: 'elbow-up' });
    }

    if (configs.length === 0) {
        return { success: false, error: 'Cannot reach target with gripper pointing down (joint limits)' };
    }

    // Prefer elbow-down for reaching forward/down, elbow-up for reaching up/back
    // Simple heuristic: prefer configuration with smaller total joint movement
    const chosen = configs[0];

    // Convert to degrees
    const result = {
        base: baseAngle * 180 / Math.PI,
        shoulder: chosen.shoulder * 180 / Math.PI,
        elbow: chosen.elbow * 180 / Math.PI,
        wrist: chosen.wrist * 180 / Math.PI
    };

    return { success: true, angles: result };
}

// ============================================================================
// ANIMATION
// ============================================================================

let animationStartTime = 0;
let animationStartAngles = {};

function startAnimation() {
    if (!isAnimating) {
        animationStartTime = performance.now();
        animationStartAngles = { ...jointAngles };
        isAnimating = true;
        document.getElementById('status-animating').classList.add('active');
    }
}

function updateAnimation() {
    if (!isAnimating) return;

    const elapsed = performance.now() - animationStartTime;
    const progress = Math.min(elapsed / CONFIG.animation.duration, 1);
    const easedProgress = easingFunctions[CONFIG.animation.easing](progress);

    // Interpolate all joint angles
    // Base uses lerpAngle for shortest-path rotation (it's circular -180° to 180°)
    jointAngles.base = lerpAngle(animationStartAngles.base, targetAngles.base, easedProgress);
    jointAngles.shoulder = lerp(animationStartAngles.shoulder, targetAngles.shoulder, easedProgress);
    jointAngles.elbow = lerp(animationStartAngles.elbow, targetAngles.elbow, easedProgress);
    jointAngles.wrist = lerp(animationStartAngles.wrist, targetAngles.wrist, easedProgress);
    jointAngles.wristRotate = lerpAngle(animationStartAngles.wristRotate, targetAngles.wristRotate, easedProgress);

    applyJointAngles();

    if (progress >= 1) {
        isAnimating = false;
        document.getElementById('status-animating').classList.remove('active');

        // Process next command in queue if any
        if (animationQueue.length > 0) {
            const nextCommand = animationQueue.shift();
            processCommand(nextCommand);
        }
    }
}

// Update gripper animation (runs independently of joint animation)
function updateGripperAnimation() {
    if (!gripperAnimating) return;

    // Smooth interpolation toward target
    const speed = 0.04;  // Slower gripper movement for realism
    const diff = targetGripperOpenness - gripperOpenness;
    const isClosing = diff < 0;
    const isOpening = diff > 0;

    // Check for grip/release based on direction
    if (isClosing && !grippedObject) {
        // Check if we're about to grip an object
        for (const obj of sceneObjects) {
            if (obj.isGripped) continue;
            const contact = checkGripperContact(obj);
            if (contact.bothContact) {
                // Both fingers touching - grip the object!
                gripObject(obj);
                // Stop closing - gripper stays at current position
                targetGripperOpenness = gripperOpenness;
                gripperAnimating = false;
                applyGripperOpenness();
                return;
            }
        }
    } else if (isOpening && grippedObject) {
        // Opening while gripping - release the object
        releaseObject();
    }

    if (Math.abs(diff) < 0.3) {
        // Close enough, snap to target
        gripperOpenness = targetGripperOpenness;
        gripperAnimating = false;
    } else {
        // Smooth interpolation
        gripperOpenness += diff * speed;
    }

    applyGripperOpenness();
}

function lerp(a, b, t) {
    return a + (b - a) * t;
}

// Shortest path interpolation for angles
function lerpAngle(a, b, t) {
    let diff = b - a;
    while (diff > Math.PI) diff -= 2 * Math.PI;
    while (diff < -Math.PI) diff += 2 * Math.PI;
    let result = a + diff * t;
    // Normalize result to -π to π range
    while (result > Math.PI) result -= 2 * Math.PI;
    while (result < -Math.PI) result += 2 * Math.PI;
    return result;
}

// ============================================================================
// ANIMATION LOOP
// ============================================================================

function animate() {
    requestAnimationFrame(animate);

    // Calculate delta time for physics
    const currentTime = performance.now();
    const deltaTime = (currentTime - lastFrameTime) / 1000;  // Convert to seconds
    lastFrameTime = currentTime;

    updateAnimation();
    updateGripperAnimation();
    updatePhysics(deltaTime);
    controls.update();
    renderer.render(scene, camera);
}

function onWindowResize() {
    const container = document.getElementById('canvas-container');
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
}

// ============================================================================
// UI UPDATES
// ============================================================================

function updateUI() {
    // Update joint angle displays
    document.getElementById('base-angle').textContent =
        (jointAngles.base * 180 / Math.PI).toFixed(1) + '°';
    document.getElementById('shoulder-angle').textContent =
        (jointAngles.shoulder * 180 / Math.PI).toFixed(1) + '°';
    document.getElementById('elbow-angle').textContent =
        (jointAngles.elbow * 180 / Math.PI).toFixed(1) + '°';
    document.getElementById('wrist-angle').textContent =
        (jointAngles.wrist * 180 / Math.PI).toFixed(1) + '°';
    document.getElementById('rotate-angle').textContent =
        (jointAngles.wristRotate * 180 / Math.PI).toFixed(1) + '°';

    // Update end effector position (convert to cm for display)
    const pos = getEndEffectorPosition();
    document.getElementById('end-effector-pos').textContent =
        `X: ${(pos.x * 100).toFixed(0)}  Y: ${(pos.y * 100).toFixed(0)}  Z: ${(pos.z * 100).toFixed(0)} cm`;
}

// ============================================================================
// TERMINAL INTERFACE
// ============================================================================

const terminal = {
    output: null,
    input: null,

    init() {
        this.output = document.getElementById('terminal-output');
        this.input = document.getElementById('terminal-input');

        this.input.addEventListener('keydown', (e) => {
            if (e.key === 'Enter' && e.shiftKey) {
                // Shift+Enter: add current line to buffer, show continuation prompt
                e.preventDefault();
                const line = this.input.value.trim();
                if (line || multiLineBuffer.length > 0) {
                    if (multiLineBuffer.length === 0) {
                        this.print('  (multi-line mode, Enter to execute)', 'info');
                    }
                    multiLineBuffer.push(line);
                    this.print(`  ${multiLineBuffer.length}: ${line || '(empty)'}`, 'info');
                    this.input.value = '';
                    this.input.placeholder = '... (Shift+Enter for more, Enter to run)';
                }
            } else if (e.key === 'Enter') {
                // Regular Enter: execute command(s)
                const command = this.input.value.trim();

                if (multiLineBuffer.length > 0) {
                    // We have buffered lines - add current and execute all
                    if (command) {
                        multiLineBuffer.push(command);
                        this.print(`  ${multiLineBuffer.length}: ${command}`, 'info');
                    }
                    // Execute all buffered lines
                    for (const line of multiLineBuffer) {
                        if (line) this.executeCommand(line);
                    }
                    multiLineBuffer = [];
                    this.input.placeholder = 'Enter command...';
                } else if (command) {
                    this.executeCommand(command);
                    commandHistory.unshift(command);
                    historyIndex = -1;
                }
                this.input.value = '';
            } else if (e.key === 'Escape' && multiLineBuffer.length > 0) {
                // Escape: cancel multi-line mode
                e.preventDefault();
                multiLineBuffer = [];
                this.input.placeholder = 'Enter command...';
                this.print('  (multi-line cancelled)', 'warning');
            } else if (e.key === 'ArrowUp') {
                e.preventDefault();
                if (historyIndex < commandHistory.length - 1) {
                    historyIndex++;
                    this.input.value = commandHistory[historyIndex];
                }
            } else if (e.key === 'ArrowDown') {
                e.preventDefault();
                if (historyIndex > 0) {
                    historyIndex--;
                    this.input.value = commandHistory[historyIndex];
                } else {
                    historyIndex = -1;
                    this.input.value = '';
                }
            }
        });

        // Handle paste events for multi-line input (e.g., pasting entire programs)
        this.input.addEventListener('paste', (e) => {
            const pastedText = e.clipboardData.getData('text');

            // Check if it's multi-line
            if (pastedText.includes('\n')) {
                e.preventDefault();
                const lines = pastedText.split('\n').map(l => l.trim()).filter(l => l);
                for (const line of lines) {
                    this.executeCommand(line);
                }
                this.input.value = '';
            }
            // Single line paste - let default behavior handle it
        });

        // Welcome message
        this.printWelcome();
    },

    printWelcome() {
        this.print('╔════════════════════════════════════════╗', 'info');
        this.print('║     ROBOT ARM SIMULATOR v1.0           ║', 'highlight');
        this.print('║     Terminal Control Interface         ║', 'info');
        this.print('╚════════════════════════════════════════╝', 'info');
        this.print('');
        this.print('Type "help" for available commands.', 'info');
        this.print('');
    },

    print(text, type = '') {
        const line = document.createElement('div');
        line.className = 'output-line';
        if (type) line.classList.add(`output-${type}`);
        line.textContent = text;
        this.output.appendChild(line);
        this.output.scrollTop = this.output.scrollHeight;
    },

    printHTML(html) {
        const line = document.createElement('div');
        line.className = 'output-line';
        line.innerHTML = html;
        this.output.appendChild(line);
        this.output.scrollTop = this.output.scrollHeight;
    },

    executeCommand(command) {
        // Check if we're in program recording mode
        if (programMode) {
            const trimmed = command.trim().toLowerCase();
            // 'end' exits program mode, everything else gets recorded
            if (trimmed === 'end') {
                this.print(`❯ ${command}`, 'command');
                handleEndCommand();
                return;
            }
            // Record the command (preserve original case for display)
            programMode.commands.push(command.trim());
            this.print(`  ${programMode.commands.length}: ${command}`, 'info');
            return;
        }

        this.print(`❯ ${command}`, 'command');

        if (isAnimating) {
            // Queue command if currently animating
            animationQueue.push(command);
            this.print('Command queued (arm is moving)', 'warning');
            return;
        }

        processCommand(command);
    },

    clear() {
        this.output.innerHTML = '';
    }
};

function processCommand(input) {
    const parts = input.toLowerCase().trim().split(/\s+/);
    const command = parts[0];
    const args = parts.slice(1);

    switch (command) {
        case 'help':
            showHelp(args[0]);
            break;

        case 'move':
        case 'm':
            handleMoveCommand(args);
            break;

        case 'goto':
        case 'g':
            handleGotoCommand(args);
            break;

        case 'home':
        case 'h':
            handleHomeCommand();
            break;

        case 'status':
        case 's':
            handleStatusCommand();
            break;

        case 'base':
        case 'b':
            handleJointCommand('base', args);
            break;

        case 'shoulder':
        case 'sh':
            handleJointCommand('shoulder', args);
            break;

        case 'elbow':
        case 'e':
            handleJointCommand('elbow', args);
            break;

        case 'wrist':
        case 'w':
            handleJointCommand('wrist', args);
            break;

        case 'rotate':
        case 'r':
            handleRotateCommand(args);
            break;

        case 'speed':
            handleSpeedCommand(args);
            break;

        case 'clear':
        case 'cls':
            terminal.clear();
            break;

        case 'demo':
            runDemo();
            break;

        case 'reach':
            handleReachCommand();
            break;

        case 'limits':
            showLimits();
            break;

        case 'preset':
        case 'p':
            handlePresetCommand(args);
            break;

        case 'pick':
            handlePickCommand(args);
            break;

        case 'stop':
        case 'x':
            handleStopCommand();
            break;

        case 'save':
            handleSaveCommand(args);
            break;

        case 'load':
        case 'l':
            handleLoadCommand(args);
            break;

        case 'positions':
        case 'pos':
            handlePositionsCommand();
            break;

        case 'delete':
        case 'del':
            handleDeletePositionCommand(args);
            break;

        case 'grip':
        case 'gripper':
            handleGripCommand(args);
            break;

        case 'open':
            handleGripCommand(['100']);
            break;

        case 'close':
            handleGripCommand(['0']);
            break;

        case 'spawn':
        case 'add':
            handleSpawnCommand(args);
            break;

        case 'objects':
        case 'obj':
            handleObjectsCommand();
            break;

        case 'remove':
        case 'rm':
            handleRemoveCommand(args);
            break;

        case 'program':
        case 'prog':
            handleProgramCommand(args);
            break;

        case 'run':
            handleRunCommand(args);
            break;

        case 'end':
            handleEndCommand();
            break;

        case 'wait':
            handleWaitCommand(args);
            break;

        case 'repeat':
            handleRepeatCommand(args);
            break;

        case 'endrepeat':
            handleEndRepeatCommand();
            break;

        default:
            terminal.print(`Unknown command: ${command}`, 'error');
            terminal.print('Type "help" for available commands.', 'info');
    }
}

function showHelp(topic) {
    if (!topic) {
        terminal.print('');
        terminal.print('━━━ AVAILABLE COMMANDS ━━━', 'highlight');
        terminal.print('');
        terminal.print('JOINT CONTROL:', 'warning');
        terminal.print('  move <base> <shoulder> <elbow> <wrist>', 'info');
        terminal.print('    Set all joint angles at once (degrees)');
        terminal.print('');
        terminal.print('  base <angle>     | b <angle>', 'info');
        terminal.print('  shoulder <angle> | sh <angle>', 'info');
        terminal.print('  elbow <angle>    | e <angle>', 'info');
        terminal.print('  wrist <angle>    | w <angle>', 'info');
        terminal.print('  rotate <angle>   | r <angle>  (wrist roll)', 'info');
        terminal.print('    Control individual joints');
        terminal.print('    Use +/- for relative: b +10, e -15, r +45', 'info');
        terminal.print('');
        terminal.print('POSITION CONTROL (Inverse Kinematics):', 'warning');
        terminal.print('  goto <x> <y> <z> | g <x> <y> <z>  (cm)', 'info');
        terminal.print('    Move gripper tip to XYZ position in cm');
        terminal.print('    X=left/right, Y=forward/back, Z=height', 'info');
        terminal.print('');
        terminal.print('PRESETS:', 'warning');
        terminal.print('  preset <name> | p <name>', 'info');
        terminal.print('    Move to named position (home, upright, rest, reach, up, down, left, right)');
        terminal.print('');
        terminal.print('SAVED POSITIONS:', 'warning');
        terminal.print('  save <name>     Save current position with a name', 'info');
        terminal.print('  load <name>     Go to a saved position', 'info');
        terminal.print('  positions       List all saved positions', 'info');
        terminal.print('  delete <name>   Delete a saved position', 'info');
        terminal.print('');
        terminal.print('GRIPPER:', 'warning');
        terminal.print('  grip <0-100>    Set gripper openness (0=closed, 100=open)', 'info');
        terminal.print('  open            Fully open gripper', 'info');
        terminal.print('  close           Fully close gripper', 'info');
        terminal.print('    Use +/- for relative: grip +20, grip -10', 'info');
        terminal.print('');
        terminal.print('OBJECTS:', 'warning');
        terminal.print('  spawn [type] [x y z] [Nmm]  Add object (cube/cylinder/sphere)', 'info');
        terminal.print('  objects               List all objects in scene', 'info');
        terminal.print('  remove <name|all>     Remove object(s)', 'info');
        terminal.print('');
        terminal.print('PROGRAMS:', 'warning');
        terminal.print('  program new <name>    Start recording a program', 'info');
        terminal.print('  program               List all programs', 'info');
        terminal.print('  program show <name>   Display program contents', 'info');
        terminal.print('  program edit <name>   Add commands to existing program', 'info');
        terminal.print('  program delete <name> Delete a program', 'info');
        terminal.print('  run <name>            Execute a program', 'info');
        terminal.print('  program stop          Stop running program', 'info');
        terminal.print('  end                   Finish recording', 'info');
        terminal.print('');
        terminal.print('PROGRAM COMMANDS (inside programs only):', 'warning');
        terminal.print('  wait <ms>             Pause for N milliseconds', 'info');
        terminal.print('  repeat <n>            Start loop (repeat N times)', 'info');
        terminal.print('  endrepeat             End of repeat block', 'info');
        terminal.print('  spawn ...             Add objects during program', 'info');
        terminal.print('  # comment             Comment (ignored)', 'info');
        terminal.print('');
        terminal.print('TIPS:', 'warning');
        terminal.print('  Shift+Enter           Multi-line input mode', 'info');
        terminal.print('  Escape                Cancel multi-line mode', 'info');
        terminal.print('  Paste multi-line      Auto-executes each line', 'info');
        terminal.print('');
        terminal.print('UTILITY:', 'warning');
        terminal.print('  home | h        Reset arm to home position', 'info');
        terminal.print('  status | s      Show current state', 'info');
        terminal.print('  stop | x        Stop movement & clear queue', 'info');
        terminal.print('  speed <ms>      Set animation speed', 'info');
        terminal.print('  limits          Show joint limits', 'info');
        terminal.print('  reach           Show reachable area', 'info');
        terminal.print('  demo            Run demonstration', 'info');
        terminal.print('  clear | cls     Clear terminal', 'info');
        terminal.print('');
        terminal.print('Type "help <command>" for details.', 'info');
    } else {
        showCommandHelp(topic);
    }
}

function showCommandHelp(cmd) {
    const helpTexts = {
        'move': [
            'MOVE - Set all joint angles',
            '',
            'Usage: move <base> <shoulder> <elbow> <wrist>',
            'Alias: m',
            '',
            'Examples:',
            '  move 45 30 -45 0',
            '  m 0 45 45 0'
        ],
        'goto': [
            'GOTO - Move gripper tip to XYZ position (Inverse Kinematics)',
            '',
            'Usage: goto <x> <y> <z>  (coordinates in cm)',
            'Alias: g',
            '',
            'Coordinates (robotics convention, in cm):',
            '  X: Left/Right (negative=left, positive=right)',
            '  Y: Forward/Back (positive=forward from base)',
            '  Z: Height (0=floor, positive=up)',
            '',
            'Examples:',
            '  g 0 200 50    - Forward 200cm, 50cm height',
            '  g 100 150 100 - Right 100cm, forward 150cm, 100cm up',
            '  g 0 250 30    - Straight ahead 250cm, near floor'
        ],
        'base': [
            'BASE - Control base rotation',
            '',
            'Usage: base <angle>',
            'Alias: b',
            'Range: -180° to 180°',
            '',
            'Examples:',
            '  base 90',
            '  b -45'
        ],
        'rotate': [
            'ROTATE - Control wrist roll/rotation',
            '',
            'Usage: rotate <angle>',
            'Alias: r',
            'Range: -180° to 180°',
            '',
            'Rotates the gripper around the arm\'s longitudinal axis.',
            'Useful for orienting objects before gripping.',
            '',
            'Examples:',
            '  rotate 90   - Turn gripper 90° clockwise',
            '  r -45       - Turn gripper 45° counter-clockwise',
            '  r +30       - Relative: add 30° to current rotation'
        ],
        'speed': [
            'SPEED - Set animation duration',
            '',
            'Usage: speed <milliseconds>',
            '',
            'Examples:',
            '  speed 500   (faster)',
            '  speed 2000  (slower)'
        ],
        'spawn': [
            'SPAWN - Add a physics object to the scene',
            '',
            'Usage: spawn [type] [x y z] [Nmm]',
            'Alias: add',
            '',
            'Types: cube (default), cylinder, sphere',
            'Position: x y z in cm (optional, defaults to random)',
            'Size: Nmm where N is 10-500 (optional, defaults to 80mm)',
            '',
            'Objects can be picked up with the gripper.',
            'Gripper must contact object with both fingers to grip.',
            '',
            'Examples:',
            '  spawn              - Random 80mm cube at random position',
            '  spawn sphere       - 80mm sphere at random position',
            '  spawn cube 0 100 0 - 80mm cube at (0, 100, 0) cm',
            '  spawn 50mm         - 50mm cube at random position',
            '  spawn sphere 120mm - 120mm sphere at random position',
            '  spawn cube 0 100 0 150mm - 150mm cube at position'
        ],
        'objects': [
            'OBJECTS - List all objects in the scene',
            '',
            'Usage: objects',
            'Alias: obj',
            '',
            'Shows object names, positions, and grip status.'
        ],
        'remove': [
            'REMOVE - Remove objects from the scene',
            '',
            'Usage: remove <name|all>',
            'Alias: rm',
            '',
            'Examples:',
            '  remove cube_1  - Remove specific object',
            '  remove all     - Remove all objects'
        ],
        'program': [
            'PROGRAM - Create and manage automation programs',
            '',
            'Usage: program [subcommand] [name]',
            'Alias: prog',
            '',
            'Subcommands:',
            '  new <name>     Start recording a new program',
            '  show <name>    Display program contents',
            '  edit <name>    Append commands to existing program',
            '  delete <name>  Remove a program',
            '  stop           Stop currently running program',
            '  (none)         List all programs',
            '',
            'Recording mode:',
            '  After "program new", all commands are recorded.',
            '  Type "end" to save and exit recording mode.',
            '',
            'Special commands (inside programs only):',
            '  wait <ms>      Pause execution for N milliseconds',
            '  repeat <n>     Start a loop that repeats N times',
            '  endrepeat      End of repeat block',
            '  # text         Comment (ignored during execution)',
            '',
            'Example program:',
            '  program new pickplace',
            '  # Spawn a block and pick it up',
            '  spawn cube 100 150 0',
            '  wait 500',
            '  goto 100 150 50',
            '  goto 100 150 5',
            '  close',
            '  wait 500',
            '  goto 100 150 50',
            '  # Move to drop position',
            '  goto -100 150 50',
            '  goto -100 150 5',
            '  open',
            '  wait 500',
            '  home',
            '  end',
            '',
            'Run with: run pickplace',
            '',
            'Tip: Use Shift+Enter to write multi-line programs',
            'directly, or paste an entire program at once.'
        ],
        'run': [
            'RUN - Execute a saved program',
            '',
            'Usage: run <name>',
            '',
            'Executes all commands in the program sequentially.',
            'The program waits for each motion to complete.',
            '',
            'Use "program stop" to abort execution.',
            '',
            'Example:',
            '  run pickplace'
        ]
    };

    const help = helpTexts[cmd] || [`No help available for "${cmd}"`];
    help.forEach(line => terminal.print(line, 'info'));
}

function handleMoveCommand(args) {
    if (args.length < 4) {
        terminal.print('Usage: move <base> <shoulder> <elbow> <wrist>', 'error');
        terminal.print('Example: move 45 30 -45 0', 'info');
        return;
    }

    const base = parseFloat(args[0]);
    const shoulder = parseFloat(args[1]);
    const elbow = parseFloat(args[2]);
    const wrist = parseFloat(args[3]);

    if ([base, shoulder, elbow, wrist].some(isNaN)) {
        terminal.print('Error: All angles must be numbers', 'error');
        return;
    }

    terminal.print(`Moving to: B=${base}° S=${shoulder}° E=${elbow}° W=${wrist}°`, 'success');
    setJointAngles(base, shoulder, elbow, wrist);
}

function handleGotoCommand(args) {
    if (args.length < 3) {
        terminal.print('Usage: goto <x> <y> <z>  (in cm)', 'error');
        terminal.print('Example: goto 0 200 50', 'info');
        return;
    }

    // Parse coordinates in cm
    const xCm = parseFloat(args[0]);
    const yCm = parseFloat(args[1]);
    const zCm = parseFloat(args[2]);

    if ([xCm, yCm, zCm].some(isNaN)) {
        terminal.print('Error: Coordinates must be numbers', 'error');
        return;
    }

    // Convert cm to internal units (1 unit = 100cm)
    const x = xCm / 100;
    const y = yCm / 100;
    const z = zCm / 100;

    terminal.print(`Moving gripper to (${xCm}, ${yCm}, ${zCm}) cm...`, 'info');

    const result = solveIK(x, y, z);

    if (!result.success) {
        terminal.print(`Error: ${result.error}`, 'error');
        return;
    }

    const { base, shoulder, elbow, wrist } = result.angles;
    terminal.print(`Solution: B=${base.toFixed(1)}° S=${shoulder.toFixed(1)}° E=${elbow.toFixed(1)}° W=${wrist.toFixed(1)}°`, 'success');
    setJointAngles(base, shoulder, elbow, wrist);
}

function handleHomeCommand() {
    terminal.print('Moving to home position (ready stance)...', 'success');
    // Return to the starting "ready" position
    setJointAngles(180, -20, 90, 45);
    // Reset wrist rotation
    targetAngles.wristRotate = 0;
    // Reset gripper to default (50% open)
    setGripperOpenness(50);
}

function handleStatusCommand() {
    terminal.print('');
    terminal.print('━━━ ARM STATUS ━━━', 'highlight');
    terminal.print(`Base:     ${(jointAngles.base * 180 / Math.PI).toFixed(1)}°`, 'info');
    terminal.print(`Shoulder: ${(jointAngles.shoulder * 180 / Math.PI).toFixed(1)}°`, 'info');
    terminal.print(`Elbow:    ${(jointAngles.elbow * 180 / Math.PI).toFixed(1)}°`, 'info');
    terminal.print(`Wrist:    ${(jointAngles.wrist * 180 / Math.PI).toFixed(1)}°`, 'info');
    terminal.print(`Rotate:   ${(jointAngles.wristRotate * 180 / Math.PI).toFixed(1)}°`, 'info');
    terminal.print(`Gripper:  ${gripperOpenness.toFixed(0)}% open`, 'info');

    const pos = getEndEffectorPosition();
    // Convert to cm for display
    const xCm = (pos.x * 100).toFixed(0);
    const yCm = (pos.y * 100).toFixed(0);
    const zCm = (pos.z * 100).toFixed(0);
    const reachCm = (Math.sqrt(pos.x*pos.x + pos.y*pos.y) * 100).toFixed(0);

    terminal.print('', 'info');
    terminal.print('━━━ GRIPPER TIP POSITION ━━━', 'highlight');
    terminal.print(`X: ${xCm} cm (left/right)`, 'info');
    terminal.print(`Y: ${yCm} cm (forward/back)`, 'info');
    terminal.print(`Z: ${zCm} cm (height)`, 'info');
    terminal.print('', 'info');
    terminal.print(`Horizontal reach: ${reachCm} cm`, 'info');
    terminal.print('');
}

function handleJointCommand(joint, args) {
    if (args.length < 1) {
        const currentAngle = jointAngles[joint] * 180 / Math.PI;
        terminal.print(`${joint}: ${currentAngle.toFixed(1)}°`, 'info');
        return;
    }

    const input = args[0];
    const currentAngle = jointAngles[joint] * 180 / Math.PI;
    let angle;

    // Support relative adjustments (+/- prefix)
    if (input.startsWith('+') || input.startsWith('-')) {
        const delta = parseFloat(input);
        if (isNaN(delta)) {
            terminal.print('Error: Angle must be a number', 'error');
            return;
        }
        angle = currentAngle + delta;
        terminal.print(`${joint}: ${currentAngle.toFixed(1)}° → ${angle.toFixed(1)}° (${input})`, 'success');
    } else {
        angle = parseFloat(input);
        if (isNaN(angle)) {
            terminal.print('Error: Angle must be a number', 'error');
            return;
        }
        terminal.print(`Setting ${joint} to ${angle}°`, 'success');
    }

    // Get current angles and update the specified joint
    const current = {
        base: jointAngles.base * 180 / Math.PI,
        shoulder: jointAngles.shoulder * 180 / Math.PI,
        elbow: jointAngles.elbow * 180 / Math.PI,
        wrist: jointAngles.wrist * 180 / Math.PI
    };

    current[joint] = angle;

    setJointAngles(current.base, current.shoulder, current.elbow, current.wrist);
}

function handleRotateCommand(args) {
    if (args.length < 1) {
        const currentAngle = jointAngles.wristRotate * 180 / Math.PI;
        terminal.print(`Wrist rotation: ${currentAngle.toFixed(1)}°`, 'info');
        terminal.print('Usage: rotate <angle> | r <angle>', 'info');
        terminal.print('Use +/- for relative: r +45, r -30', 'info');
        return;
    }

    const input = args[0];
    const currentAngle = jointAngles.wristRotate * 180 / Math.PI;
    let angle;

    // Support relative adjustments (+/- prefix)
    if (input.startsWith('+') || input.startsWith('-')) {
        const delta = parseFloat(input);
        if (isNaN(delta)) {
            terminal.print('Error: Angle must be a number', 'error');
            return;
        }
        angle = currentAngle + delta;
        terminal.print(`Wrist rotation: ${currentAngle.toFixed(1)}° → ${angle.toFixed(1)}° (${input})`, 'success');
    } else {
        angle = parseFloat(input);
        if (isNaN(angle)) {
            terminal.print('Error: Angle must be a number', 'error');
            return;
        }
        terminal.print(`Setting wrist rotation to ${angle}°`, 'success');
    }

    // Clamp to limits and convert to radians
    const clampedAngle = clampAngle(angle, CONFIG.limits.wristRotate);
    if (clampedAngle !== angle) {
        terminal.print(`Clamped to limit: ${clampedAngle}°`, 'warning');
    }

    targetAngles.wristRotate = clampedAngle * Math.PI / 180;
    startAnimation();
}

function handleSpeedCommand(args) {
    if (args.length < 1) {
        terminal.print(`Current animation speed: ${CONFIG.animation.duration}ms`, 'info');
        return;
    }

    const speed = parseInt(args[0]);
    if (isNaN(speed) || speed < 100 || speed > 5000) {
        terminal.print('Error: Speed must be between 100 and 5000 ms', 'error');
        return;
    }

    CONFIG.animation.duration = speed;
    terminal.print(`Animation speed set to ${speed}ms`, 'success');
}

function handleReachCommand() {
    const armLength = CONFIG.segments.shoulderLength + CONFIG.segments.elbowLength +
                      CONFIG.segments.wristLength + CONFIG.segments.gripperLength;
    const baseHeight = CONFIG.segments.baseHeight + 0.15;
    terminal.print('');
    terminal.print('━━━ REACHABLE AREA ━━━', 'highlight');
    terminal.print(`Total arm length: ${(armLength * 100).toFixed(0)} cm`, 'info');
    terminal.print(`Max horizontal reach (Y): ~${(armLength * 95).toFixed(0)} cm`, 'info');
    terminal.print(`Max height (Z): ~${((baseHeight + armLength) * 100).toFixed(0)} cm`, 'info');
    terminal.print(`Shoulder height: ${(baseHeight * 100).toFixed(0)} cm`, 'info');
    terminal.print('');
}

function showLimits() {
    terminal.print('');
    terminal.print('━━━ JOINT LIMITS ━━━', 'highlight');
    terminal.print(`Base:     ${CONFIG.limits.base.min}° to ${CONFIG.limits.base.max}°`, 'info');
    terminal.print(`Shoulder: ${CONFIG.limits.shoulder.min}° to ${CONFIG.limits.shoulder.max}°`, 'info');
    terminal.print(`Elbow:    ${CONFIG.limits.elbow.min}° to ${CONFIG.limits.elbow.max}°`, 'info');
    terminal.print(`Wrist:    ${CONFIG.limits.wrist.min}° to ${CONFIG.limits.wrist.max}°`, 'info');
    terminal.print(`Rotate:   ${CONFIG.limits.wristRotate.min}° to ${CONFIG.limits.wristRotate.max}°`, 'info');
    terminal.print('');
}

// Preset positions - realistic robot arm poses
const PRESETS = {
    home: { base: 180, shoulder: -20, elbow: 90, wrist: 45, rotate: 0, desc: 'Home/ready stance (default)' },
    upright: { base: 0, shoulder: 0, elbow: 0, wrist: 0, rotate: 0, desc: 'Fully upright (all zeros)' },
    rest: { base: 0, shoulder: 70, elbow: 110, wrist: -60, rotate: 0, desc: 'Folded rest position' },
    reach: { base: 0, shoulder: 50, elbow: 30, wrist: 10, rotate: 0, desc: 'Forward reach' },
    up: { base: 0, shoulder: -40, elbow: 20, wrist: 60, rotate: 0, desc: 'Reaching up' },
    down: { base: 0, shoulder: 80, elbow: 100, wrist: -90, rotate: 0, desc: 'Reaching to floor' },
    left: { base: -90, shoulder: -20, elbow: 90, wrist: 45, rotate: 0, desc: 'Facing left' },
    right: { base: 90, shoulder: -20, elbow: 90, wrist: 45, rotate: 0, desc: 'Facing right' }
};

// User-saved positions
const savedPositions = {};

// User programs (scripts)
const programs = {};
let programMode = null;  // null or { name: string, commands: [] }
let runningProgram = null;  // null or { name, commands, index, loopStack, paused }

function handlePresetCommand(args) {
    if (args.length < 1) {
        terminal.print('');
        terminal.print('━━━ AVAILABLE PRESETS ━━━', 'highlight');
        for (const [name, preset] of Object.entries(PRESETS)) {
            terminal.print(`  ${name.padEnd(8)} - ${preset.desc}`, 'info');
        }
        terminal.print('');
        terminal.print('Usage: preset <name> | p <name>', 'info');
        return;
    }

    const presetName = args[0].toLowerCase();
    const preset = PRESETS[presetName];

    if (!preset) {
        terminal.print(`Unknown preset: ${presetName}`, 'error');
        terminal.print('Type "preset" to see available presets.', 'info');
        return;
    }

    terminal.print(`Moving to preset: ${presetName} (${preset.desc})`, 'success');

    // Set wrist rotation target before starting animation
    const rotateRad = (preset.rotate !== undefined ? preset.rotate : 0) * Math.PI / 180;
    targetAngles.wristRotate = rotateRad;

    setJointAngles(preset.base, preset.shoulder, preset.elbow, preset.wrist);
}

function handlePickCommand(args) {
    // Simulate a pick and place operation
    terminal.print('Executing pick sequence...', 'warning');

    const sequence = [
        'preset down',
        'e +20',
        'preset ready'
    ];

    for (const cmd of sequence) {
        animationQueue.push(cmd);
    }

    if (!isAnimating && animationQueue.length > 0) {
        processCommand(animationQueue.shift());
    }
}

function handleStopCommand() {
    if (isAnimating || animationQueue.length > 0) {
        isAnimating = false;
        animationQueue = [];
        // Finalize at current position
        targetAngles.base = jointAngles.base;
        targetAngles.shoulder = jointAngles.shoulder;
        targetAngles.elbow = jointAngles.elbow;
        targetAngles.wrist = jointAngles.wrist;
        targetAngles.wristRotate = jointAngles.wristRotate;
        document.getElementById('status-animating').classList.remove('active');
        terminal.print('Movement stopped. Queue cleared.', 'warning');
    } else {
        terminal.print('No movement in progress.', 'info');
    }
}

function handleGripCommand(args) {
    if (args.length < 1) {
        // Show current gripper state
        terminal.print(`Gripper: ${gripperOpenness.toFixed(0)}% open`, 'info');
        terminal.print('Usage: grip <0-100> | open | close', 'info');
        return;
    }

    const input = args[0];

    // Support relative adjustments (+/- prefix)
    let percent;
    if (input.startsWith('+') || input.startsWith('-')) {
        const delta = parseFloat(input);
        if (isNaN(delta)) {
            terminal.print('Error: Value must be a number', 'error');
            return;
        }
        percent = gripperOpenness + delta;
        terminal.print(`Gripper: ${gripperOpenness.toFixed(0)}% → ${Math.max(0, Math.min(100, percent)).toFixed(0)}% (${input})`, 'success');
    } else {
        percent = parseFloat(input);
        if (isNaN(percent)) {
            terminal.print('Error: Value must be a number (0-100)', 'error');
            return;
        }

        if (percent <= 0) {
            terminal.print('Closing gripper...', 'success');
        } else if (percent >= 100) {
            terminal.print('Opening gripper fully...', 'success');
        } else {
            terminal.print(`Setting gripper to ${percent.toFixed(0)}% open...`, 'success');
        }
    }

    setGripperOpenness(percent);
}

// Check if a position is reachable by the arm (in cm)
function isPositionReachable(xCm, yCm, zCm) {
    // Convert cm to internal units (1 unit = 100cm)
    const x = xCm / 100;
    const y = yCm / 100;
    const z = zCm / 100;

    const result = solveIK(x, y, z);
    return result.success;
}

// Get the minimum horizontal reach distance at a given height (in cm)
// This is the inner boundary where the arm cannot reach
function getMinReachAtHeight(heightCm) {
    // Test positions at this height to find minimum reachable distance
    // Start from center and move outward until we find a reachable position
    for (let dist = 0; dist <= 300; dist += 5) {
        if (isPositionReachable(dist, 0, heightCm)) {
            return dist;
        }
    }
    return 300; // Fallback - shouldn't happen
}

function handleSpawnCommand(args) {
    // Parse type (default: cube)
    let type = 'cube';
    let x, y, z;
    let sizeMm = 80; // Default size: 80mm

    if (args.length >= 1 && ['cube', 'cylinder', 'sphere'].includes(args[0])) {
        type = args[0];
        args = args.slice(1);
    }

    // Check for size parameter (e.g., "50mm" or "120mm")
    const sizeArgIndex = args.findIndex(arg => /^\d+mm$/i.test(arg));
    if (sizeArgIndex !== -1) {
        sizeMm = parseInt(args[sizeArgIndex]);
        if (sizeMm < 10 || sizeMm > 500) {
            terminal.print('Error: Size must be between 10mm and 500mm', 'error');
            return;
        }
        args.splice(sizeArgIndex, 1); // Remove size arg from position parsing
    }

    // Parse position (default: random within reach)
    if (args.length >= 3) {
        x = parseFloat(args[0]);
        y = parseFloat(args[1]);
        z = parseFloat(args[2]);

        if (isNaN(x) || isNaN(y) || isNaN(z)) {
            terminal.print('Error: Position must be numbers (x y z in cm)', 'error');
            return;
        }

        // Validate that the position is reachable by the arm
        if (!isPositionReachable(x, y, z)) {
            const horizontalDist = Math.sqrt(x * x + y * y).toFixed(0);
            const maxReach = (CONFIG.segments.shoulderLength + CONFIG.segments.elbowLength) * 100;
            const minReach = getMinReachAtHeight(z);
            terminal.print(`Error: Position (${x.toFixed(0)}, ${y.toFixed(0)}, ${z.toFixed(0)}) cm is not reachable by the arm`, 'error');
            terminal.print(`  Horizontal distance: ${horizontalDist} cm`, 'info');
            terminal.print(`  Reachable range at height ${z.toFixed(0)} cm: ~${minReach}-${maxReach.toFixed(0)} cm`, 'info');
            return;
        }
    } else {
        // Random position within arm reach, ensuring it's actually reachable
        const maxReach = (CONFIG.segments.shoulderLength + CONFIG.segments.elbowLength) * 100;
        let attempts = 0;
        const maxAttempts = 50;

        // Try to find a valid random position
        do {
            const angle = Math.random() * Math.PI * 2;
            // Use a range that's more likely to be reachable (between min reach and 80% of max)
            const minDist = 85; // Approximate minimum horizontal reach
            const maxDist = maxReach * 0.8;
            const distance = minDist + Math.random() * (maxDist - minDist);
            x = Math.cos(angle) * distance;
            y = Math.sin(angle) * distance;
            z = 4 + Math.random() * 30;  // 4-34cm height (object size + some margin, will fall to floor)
            attempts++;
        } while (!isPositionReachable(x, y, z) && attempts < maxAttempts);

        if (attempts >= maxAttempts) {
            terminal.print('Error: Could not find a valid spawn position', 'error');
            return;
        }
    }

    const obj = createObject(type, x, y, z, sizeMm);
    terminal.print(`Spawned ${obj.name} (${sizeMm}mm) at (${x.toFixed(0)}, ${y.toFixed(0)}, ${z.toFixed(0)}) cm`, 'success');
}

function handleObjectsCommand() {
    if (sceneObjects.length === 0) {
        terminal.print('No objects in scene.', 'info');
        terminal.print('Use "spawn" to add objects.', 'info');
        return;
    }

    terminal.print('');
    terminal.print('━━━ SCENE OBJECTS ━━━', 'highlight');
    for (const obj of sceneObjects) {
        // Convert Three.js position to robotics coordinates (cm)
        const x = (obj.mesh.position.x * 100).toFixed(0);
        const y = (obj.mesh.position.z * 100).toFixed(0);  // Z in Three.js is Y in robotics
        const z = (obj.mesh.position.y * 100).toFixed(0);  // Y in Three.js is Z in robotics
        const status = obj.isGripped ? ' [GRIPPED]' : '';
        terminal.print(`  ${obj.name}: (${x}, ${y}, ${z}) cm${status}`, obj.isGripped ? 'success' : 'info');
    }
    terminal.print('');
}

function handleRemoveCommand(args) {
    if (args.length < 1) {
        terminal.print('Usage: remove <name|all>', 'error');
        terminal.print('Type "objects" to see object names.', 'info');
        return;
    }

    const target = args[0].toLowerCase();

    if (target === 'all') {
        const count = sceneObjects.length;
        removeAllObjects();
        terminal.print(`Removed all ${count} object(s).`, 'success');
        return;
    }

    // Find object by name
    const obj = sceneObjects.find(o => o.name.toLowerCase() === target);
    if (!obj) {
        terminal.print(`Error: No object named "${target}"`, 'error');
        terminal.print('Type "objects" to see object names.', 'info');
        return;
    }

    removeObject(obj);
    terminal.print(`Removed ${target}.`, 'success');
}

function handleSaveCommand(args) {
    if (args.length < 1) {
        terminal.print('Usage: save <name>', 'error');
        terminal.print('Example: save pickup1', 'info');
        return;
    }

    const name = args[0].toLowerCase();

    // Don't allow overwriting built-in presets
    if (PRESETS[name]) {
        terminal.print(`Error: "${name}" is a built-in preset and cannot be overwritten.`, 'error');
        return;
    }

    const pos = getEndEffectorPosition();
    savedPositions[name] = {
        base: jointAngles.base * 180 / Math.PI,
        shoulder: jointAngles.shoulder * 180 / Math.PI,
        elbow: jointAngles.elbow * 180 / Math.PI,
        wrist: jointAngles.wrist * 180 / Math.PI,
        wristRotate: jointAngles.wristRotate * 180 / Math.PI,
        gripper: gripperOpenness,
        desc: `Saved position (${(pos.x * 100).toFixed(0)}, ${(pos.y * 100).toFixed(0)}, ${(pos.z * 100).toFixed(0)}) cm`
    };

    terminal.print(`Position saved as "${name}"`, 'success');
    terminal.print(`  B=${savedPositions[name].base.toFixed(1)}° S=${savedPositions[name].shoulder.toFixed(1)}° E=${savedPositions[name].elbow.toFixed(1)}° W=${savedPositions[name].wrist.toFixed(1)}° R=${savedPositions[name].wristRotate.toFixed(1)}° G=${savedPositions[name].gripper.toFixed(0)}%`, 'info');
}

function handleLoadCommand(args) {
    if (args.length < 1) {
        terminal.print('Usage: load <name>', 'error');
        terminal.print('Type "positions" to see saved positions.', 'info');
        return;
    }

    const name = args[0].toLowerCase();
    const position = savedPositions[name];

    if (!position) {
        terminal.print(`Error: No saved position named "${name}"`, 'error');
        terminal.print('Type "positions" to see saved positions.', 'info');
        return;
    }

    terminal.print(`Loading position: ${name}`, 'success');

    // Set wrist rotation target before starting animation
    const rotateRad = position.wristRotate !== undefined
        ? clampAngle(position.wristRotate, CONFIG.limits.wristRotate) * Math.PI / 180
        : 0;
    targetAngles.wristRotate = rotateRad;

    // Animate to position (this will include wristRotate since we set it above)
    setJointAngles(position.base, position.shoulder, position.elbow, position.wrist);

    // Restore gripper state if saved
    if (position.gripper !== undefined) {
        setGripperOpenness(position.gripper);
    }
}

function handlePositionsCommand() {
    const names = Object.keys(savedPositions);

    if (names.length === 0) {
        terminal.print('No saved positions.', 'info');
        terminal.print('Use "save <name>" to save the current position.', 'info');
        return;
    }

    terminal.print('');
    terminal.print('━━━ SAVED POSITIONS ━━━', 'highlight');
    for (const [name, pos] of Object.entries(savedPositions)) {
        terminal.print(`  ${name.padEnd(12)} - ${pos.desc}`, 'info');
    }
    terminal.print('');
    terminal.print('Use "load <name>" to go to a saved position.', 'info');
    terminal.print('Use "delete <name>" to remove a saved position.', 'info');
}

function handleDeletePositionCommand(args) {
    if (args.length < 1) {
        terminal.print('Usage: delete <name>', 'error');
        return;
    }

    const name = args[0].toLowerCase();

    if (!savedPositions[name]) {
        terminal.print(`Error: No saved position named "${name}"`, 'error');
        return;
    }

    delete savedPositions[name];
    terminal.print(`Position "${name}" deleted.`, 'success');
}

// ============================================================================
// PROGRAM/SCRIPT SYSTEM
// ============================================================================

function handleProgramCommand(args) {
    if (args.length < 1) {
        // List all programs
        const names = Object.keys(programs);
        if (names.length === 0) {
            terminal.print('No programs defined.', 'info');
            terminal.print('Use "program new <name>" to create a program.', 'info');
        } else {
            terminal.print('');
            terminal.print('━━━ PROGRAMS ━━━', 'highlight');
            for (const name of names) {
                const prog = programs[name];
                terminal.print(`  ${name.padEnd(12)} (${prog.commands.length} commands)`, 'info');
            }
            terminal.print('');
            terminal.print('Use "program show <name>" to view a program.', 'info');
            terminal.print('Use "run <name>" to execute a program.', 'info');
        }
        return;
    }

    const subcommand = args[0].toLowerCase();

    switch (subcommand) {
        case 'new':
            if (args.length < 2) {
                terminal.print('Usage: program new <name>', 'error');
                return;
            }
            const newName = args[1].toLowerCase();
            if (programs[newName]) {
                terminal.print(`Error: Program "${newName}" already exists. Delete it first.`, 'error');
                return;
            }
            programMode = { name: newName, commands: [] };
            terminal.print('');
            terminal.print(`━━━ RECORDING: ${newName.toUpperCase()} ━━━`, 'warning');
            terminal.print('Enter commands to record. Type "end" to finish.', 'info');
            terminal.print('');
            break;

        case 'show':
        case 'cat':
            if (args.length < 2) {
                terminal.print('Usage: program show <name>', 'error');
                return;
            }
            const showName = args[1].toLowerCase();
            if (!programs[showName]) {
                terminal.print(`Error: Program "${showName}" not found.`, 'error');
                return;
            }
            terminal.print('');
            terminal.print(`━━━ PROGRAM: ${showName.toUpperCase()} ━━━`, 'highlight');
            programs[showName].commands.forEach((cmd, i) => {
                const lineNum = String(i + 1).padStart(3, ' ');
                terminal.print(`${lineNum}: ${cmd}`, cmd.startsWith('#') ? 'info' : '');
            });
            terminal.print('');
            break;

        case 'delete':
        case 'del':
        case 'rm':
            if (args.length < 2) {
                terminal.print('Usage: program delete <name>', 'error');
                return;
            }
            const delName = args[1].toLowerCase();
            if (!programs[delName]) {
                terminal.print(`Error: Program "${delName}" not found.`, 'error');
                return;
            }
            delete programs[delName];
            terminal.print(`Program "${delName}" deleted.`, 'success');
            break;

        case 'edit':
            if (args.length < 2) {
                terminal.print('Usage: program edit <name>', 'error');
                return;
            }
            const editName = args[1].toLowerCase();
            if (!programs[editName]) {
                terminal.print(`Error: Program "${editName}" not found.`, 'error');
                return;
            }
            // Enter edit mode with existing commands
            programMode = { name: editName, commands: [...programs[editName].commands] };
            terminal.print('');
            terminal.print(`━━━ EDITING: ${editName.toUpperCase()} ━━━`, 'warning');
            terminal.print('Current commands:', 'info');
            programMode.commands.forEach((cmd, i) => {
                terminal.print(`  ${i + 1}: ${cmd}`, 'info');
            });
            terminal.print('');
            terminal.print('Enter additional commands. Type "end" to save.', 'info');
            break;

        case 'stop':
            if (runningProgram) {
                terminal.print(`Program "${runningProgram.name}" stopped.`, 'warning');
                runningProgram = null;
            } else {
                terminal.print('No program is running.', 'info');
            }
            break;

        default:
            // Check if it's a program name to show
            if (programs[subcommand]) {
                terminal.print('');
                terminal.print(`━━━ PROGRAM: ${subcommand.toUpperCase()} ━━━`, 'highlight');
                programs[subcommand].commands.forEach((cmd, i) => {
                    const lineNum = String(i + 1).padStart(3, ' ');
                    terminal.print(`${lineNum}: ${cmd}`, cmd.startsWith('#') ? 'info' : '');
                });
                terminal.print('');
            } else {
                terminal.print(`Unknown subcommand: ${subcommand}`, 'error');
                terminal.print('Usage: program [new|show|edit|delete|stop] <name>', 'info');
            }
    }
}

function handleEndCommand() {
    if (programMode) {
        // Save the program
        programs[programMode.name] = {
            commands: programMode.commands,
            created: new Date().toISOString()
        };
        terminal.print('');
        terminal.print(`━━━ PROGRAM SAVED: ${programMode.name.toUpperCase()} ━━━`, 'success');
        terminal.print(`${programMode.commands.length} commands recorded.`, 'info');
        terminal.print(`Use "run ${programMode.name}" to execute.`, 'info');
        terminal.print('');
        programMode = null;
    } else {
        terminal.print('Not currently recording a program.', 'error');
    }
}

function handleRunCommand(args) {
    if (args.length < 1) {
        terminal.print('Usage: run <program_name>', 'error');
        return;
    }

    const name = args[0].toLowerCase();
    if (!programs[name]) {
        terminal.print(`Error: Program "${name}" not found.`, 'error');
        terminal.print('Use "program" to see available programs.', 'info');
        return;
    }

    if (runningProgram) {
        terminal.print(`Error: Program "${runningProgram.name}" is already running.`, 'error');
        terminal.print('Use "program stop" to stop it first.', 'info');
        return;
    }

    terminal.print('');
    terminal.print(`━━━ RUNNING: ${name.toUpperCase()} ━━━`, 'warning');

    runningProgram = {
        name: name,
        commands: [...programs[name].commands],
        index: 0,
        loopStack: []  // Stack of { startIndex, remaining }
    };

    // Start executing
    executeNextProgramCommand();
}

function executeNextProgramCommand() {
    if (!runningProgram) return;

    // Check if program is complete
    if (runningProgram.index >= runningProgram.commands.length) {
        terminal.print('');
        terminal.print(`━━━ PROGRAM COMPLETE: ${runningProgram.name.toUpperCase()} ━━━`, 'success');
        runningProgram = null;
        return;
    }

    const command = runningProgram.commands[runningProgram.index];
    runningProgram.index++;

    // Skip empty lines and comments
    const trimmed = command.trim();
    if (!trimmed || trimmed.startsWith('#')) {
        executeNextProgramCommand();
        return;
    }

    const parts = trimmed.toLowerCase().split(/\s+/);
    const cmd = parts[0];

    // Handle special program-only commands
    if (cmd === 'wait') {
        const ms = parseInt(parts[1]) || 1000;
        terminal.print(`  [wait ${ms}ms]`, 'info');
        setTimeout(() => {
            if (runningProgram) {
                executeNextProgramCommand();
            }
        }, ms);
        return;
    }

    if (cmd === 'repeat') {
        const times = parseInt(parts[1]) || 1;
        runningProgram.loopStack.push({
            startIndex: runningProgram.index,
            remaining: times - 1  // -1 because first iteration is current
        });
        terminal.print(`  [repeat ${times}x]`, 'info');
        executeNextProgramCommand();
        return;
    }

    if (cmd === 'endrepeat') {
        if (runningProgram.loopStack.length > 0) {
            const loop = runningProgram.loopStack[runningProgram.loopStack.length - 1];
            if (loop.remaining > 0) {
                loop.remaining--;
                runningProgram.index = loop.startIndex;
                terminal.print(`  [loop ${loop.remaining + 1} remaining]`, 'info');
            } else {
                runningProgram.loopStack.pop();
            }
        }
        executeNextProgramCommand();
        return;
    }

    // Regular command - print and execute
    terminal.print(`  → ${trimmed}`, 'command');
    processCommand(trimmed);

    // Wait for animation to complete before next command
    if (isAnimating || gripperAnimating) {
        waitForAnimationThenContinue();
    } else {
        // Small delay between instant commands for readability
        setTimeout(() => {
            if (runningProgram) {
                executeNextProgramCommand();
            }
        }, 100);
    }
}

function waitForAnimationThenContinue() {
    if (!runningProgram) return;

    if (isAnimating || gripperAnimating) {
        setTimeout(waitForAnimationThenContinue, 50);
    } else {
        // Small pause after animation completes
        setTimeout(() => {
            if (runningProgram) {
                executeNextProgramCommand();
            }
        }, 100);
    }
}

function handleWaitCommand(args) {
    // Wait only works during program execution
    if (!runningProgram) {
        terminal.print('Note: "wait" only has effect inside programs.', 'info');
        terminal.print('Use "program new <name>" to create a program.', 'info');
    }
}

function handleRepeatCommand(args) {
    if (!runningProgram) {
        terminal.print('Note: "repeat" only has effect inside programs.', 'info');
        terminal.print('Use "program new <name>" to create a program.', 'info');
    }
}

function handleEndRepeatCommand() {
    if (!runningProgram) {
        terminal.print('Note: "endrepeat" only has effect inside programs.', 'info');
    }
}

async function runDemo() {
    terminal.print('Starting demonstration sequence...', 'warning');

    const sequence = [
        'home',
        'move 0 45 45 0',
        'base 90',
        'base -90',
        'base 0',
        'goto 2 2 0',
        'goto 0 3 2',
        'goto -1 1.5 1',
        'home'
    ];

    for (const cmd of sequence) {
        animationQueue.push(cmd);
    }

    // Start first command
    if (!isAnimating && animationQueue.length > 0) {
        processCommand(animationQueue.shift());
    }
}

// ============================================================================
// CAMERA CONTROLS
// ============================================================================

function resetCamera() {
    camera.position.set(6, 4, 6);
    controls.target.set(0, 1.5, 0);
    controls.update();
}

function setTopView() {
    camera.position.set(0, 10, 0.01);
    controls.target.set(0, 0, 0);
    controls.update();
}

function setSideView() {
    camera.position.set(8, 2, 0);
    controls.target.set(0, 2, 0);
    controls.update();
}

function setFrontView() {
    camera.position.set(0, 2, 8);
    controls.target.set(0, 2, 0);
    controls.update();
}

// Make camera functions global for button onclick
window.resetCamera = resetCamera;
window.setTopView = setTopView;
window.setSideView = setSideView;
window.setFrontView = setFrontView;

// ============================================================================
// NOTEBOOK / SCRIPT EDITOR
// ============================================================================

const notebook = {
    modal: null,
    container: null,
    editor: null,
    tabsContainer: null,
    tabs: [],          // Array of { id, name, content, modified, savedName }
    activeTabId: null,
    tabCounter: 0,
    savedScripts: {},  // Loaded from localStorage

    // Window state
    isMinimized: false,
    isMaximized: false,
    savedBounds: null, // For restore after maximize
    isDragging: false,
    dragOffset: { x: 0, y: 0 },

    init() {
        this.modal = document.getElementById('notebook-modal');
        this.container = document.getElementById('notebook-container');
        this.editor = document.getElementById('notebook-editor');
        this.tabsContainer = document.getElementById('notebook-tabs');

        // Load saved scripts from localStorage
        this.loadFromStorage();

        // Create initial tab
        this.newTab();

        // Editor event listeners
        this.editor.addEventListener('input', () => this.onEditorChange());
        this.editor.addEventListener('keydown', (e) => this.onEditorKeydown(e));

        // Setup dragging
        this.setupDragging();

        // Keyboard shortcuts
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && this.modal.classList.contains('open')) {
                if (document.getElementById('save-dialog').classList.contains('open')) {
                    this.hideSave();
                } else if (document.getElementById('load-dialog').classList.contains('open')) {
                    this.hideLoad();
                } else {
                    this.close();
                }
            }
            // Ctrl+S to save
            if (e.ctrlKey && e.key === 's' && this.modal.classList.contains('open')) {
                e.preventDefault();
                this.showSave();
            }
            // Ctrl+Enter to run
            if (e.ctrlKey && e.key === 'Enter' && this.modal.classList.contains('open')) {
                e.preventDefault();
                this.run();
            }
        });

        // Enter key in save dialog
        document.getElementById('save-script-name').addEventListener('keydown', (e) => {
            if (e.key === 'Enter') {
                this.confirmSave();
            }
        });
    },

    setupDragging() {
        const header = document.getElementById('notebook-header');

        header.addEventListener('mousedown', (e) => {
            // Don't drag if clicking buttons
            if (e.target.closest('.notebook-window-btn') || e.target.closest('.notebook-header-btns')) return;
            if (this.isMaximized) return;

            this.isDragging = true;
            const rect = this.modal.getBoundingClientRect();
            this.dragOffset.x = e.clientX - rect.left;
            this.dragOffset.y = e.clientY - rect.top;

            header.style.cursor = 'grabbing';
        });

        document.addEventListener('mousemove', (e) => {
            if (!this.isDragging) return;

            const x = e.clientX - this.dragOffset.x;
            const y = e.clientY - this.dragOffset.y;

            this.modal.style.left = x + 'px';
            this.modal.style.top = y + 'px';
        });

        document.addEventListener('mouseup', () => {
            if (this.isDragging) {
                this.isDragging = false;
                document.getElementById('notebook-header').style.cursor = 'move';
            }
        });
    },

    open() {
        if (!this.modal.classList.contains('open')) {
            // Position in center on first open
            const vw = window.innerWidth;
            const vh = window.innerHeight;
            this.modal.style.left = (vw - 700) / 2 + 'px';
            this.modal.style.top = (vh - 500) / 2 + 'px';
        }
        this.modal.classList.add('open');
        this.isMinimized = false;
        this.container.style.display = 'flex';
        this.editor.focus();
    },

    close() {
        this.modal.classList.remove('open');
        this.hideLoad();
        this.hideSave();
    },

    restore() {
        // Restore to previous size from maximized state
        if (this.isMaximized && this.savedBounds) {
            this.modal.style.left = this.savedBounds.left;
            this.modal.style.top = this.savedBounds.top;
            this.container.style.width = this.savedBounds.width;
            this.container.style.height = this.savedBounds.height;
            this.isMaximized = false;
        }
    },

    maximize() {
        if (this.isMaximized) return; // Already maximized

        // Save current bounds
        this.savedBounds = {
            left: this.modal.style.left,
            top: this.modal.style.top,
            width: this.container.style.width || '700px',
            height: this.container.style.height || '500px'
        };
        // Maximize
        const terminalWidth = document.getElementById('terminal-panel').offsetWidth;
        this.modal.style.left = '20px';
        this.modal.style.top = '20px';
        this.container.style.width = (window.innerWidth - terminalWidth - 40) + 'px';
        this.container.style.height = (window.innerHeight - 40) + 'px';
        this.isMaximized = true;
    },

    // Tab Management
    newTab(name = null, content = '') {
        this.tabCounter++;
        const id = this.tabCounter;
        const tabName = name || `script_${id}`;

        const tab = { id, name: tabName, content, modified: false };
        this.tabs.push(tab);

        this.renderTabs();
        this.switchTab(id);

        return tab;
    },

    switchTab(id) {
        // Save current tab content
        if (this.activeTabId) {
            const currentTab = this.tabs.find(t => t.id === this.activeTabId);
            if (currentTab) {
                currentTab.content = this.editor.value;
            }
        }

        this.activeTabId = id;
        const tab = this.tabs.find(t => t.id === id);
        if (tab) {
            this.editor.value = tab.content;
            this.updateStatus();
        }

        this.renderTabs();
    },

    closeTab(id, event) {
        if (event) event.stopPropagation();

        const index = this.tabs.findIndex(t => t.id === id);
        if (index === -1) return;

        const tab = this.tabs[index];

        // Confirm if modified
        if (tab.modified) {
            if (!confirm(`"${tab.name}" has unsaved changes. Close anyway?`)) {
                return;
            }
        }

        this.tabs.splice(index, 1);

        // If we closed the active tab, switch to another
        if (this.activeTabId === id) {
            if (this.tabs.length > 0) {
                const newIndex = Math.min(index, this.tabs.length - 1);
                this.switchTab(this.tabs[newIndex].id);
            } else {
                // No tabs left, create a new one
                this.newTab();
            }
        } else {
            this.renderTabs();
        }
    },

    renderTabs() {
        // Remove existing tab elements (keep the add button)
        const addBtn = document.getElementById('notebook-tab-add');
        this.tabsContainer.innerHTML = '';

        for (const tab of this.tabs) {
            const tabEl = document.createElement('button');
            tabEl.className = 'notebook-tab' + (tab.id === this.activeTabId ? ' active' : '') + (tab.modified ? ' modified' : '');
            tabEl.onclick = () => this.switchTab(tab.id);

            tabEl.innerHTML = `
                <span class="tab-modified"></span>
                <span class="tab-name">${tab.name}</span>
                <span class="tab-close" onclick="notebook.closeTab(${tab.id}, event)">
                    <svg viewBox="0 0 24 24"><path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z"/></svg>
                </span>
            `;

            this.tabsContainer.appendChild(tabEl);
        }

        this.tabsContainer.appendChild(addBtn);
    },

    onEditorChange() {
        const tab = this.tabs.find(t => t.id === this.activeTabId);
        if (tab) {
            tab.content = this.editor.value;
            if (!tab.modified) {
                tab.modified = true;
                this.renderTabs();
            }
        }
        this.updateStatus();
    },

    onEditorKeydown(e) {
        // Handle Tab key for indentation
        if (e.key === 'Tab') {
            e.preventDefault();
            const start = this.editor.selectionStart;
            const end = this.editor.selectionEnd;
            const value = this.editor.value;

            this.editor.value = value.substring(0, start) + '    ' + value.substring(end);
            this.editor.selectionStart = this.editor.selectionEnd = start + 4;
            this.onEditorChange();
        }
    },

    updateStatus() {
        const content = this.editor.value;
        const lines = content.split('\n').length;
        const chars = content.length;

        document.getElementById('notebook-line-count').textContent = `Lines: ${lines}`;
        document.getElementById('notebook-char-count').textContent = `Chars: ${chars}`;

        const tab = this.tabs.find(t => t.id === this.activeTabId);
        const saveStatus = document.getElementById('notebook-save-status');
        if (tab && tab.savedName && this.savedScripts[tab.savedName]) {
            saveStatus.textContent = tab.modified ? 'Modified' : 'Saved';
            saveStatus.style.color = tab.modified ? '#f0883e' : '#7ee787';
        } else {
            saveStatus.textContent = 'Not saved';
            saveStatus.style.color = '#8b949e';
        }
    },

    // Save/Load
    showSave() {
        const tab = this.tabs.find(t => t.id === this.activeTabId);
        if (!tab) return;

        const input = document.getElementById('save-script-name');
        input.value = tab.savedName || tab.name;
        document.getElementById('save-dialog').classList.add('open');
        input.focus();
        input.select();
    },

    hideSave() {
        document.getElementById('save-dialog').classList.remove('open');
    },

    confirmSave() {
        const tab = this.tabs.find(t => t.id === this.activeTabId);
        if (!tab) return;

        const name = document.getElementById('save-script-name').value.trim();
        if (!name) {
            terminal.print('Error: Please enter a script name', 'error');
            return;
        }

        tab.name = name;
        tab.savedName = name;
        tab.modified = false;

        this.savedScripts[name] = {
            content: this.editor.value,
            savedAt: new Date().toISOString()
        };

        this.saveToStorage();
        this.renderTabs();
        this.updateStatus();
        this.hideSave();

        terminal.print(`Script "${name}" saved.`, 'success');
    },

    showLoad() {
        const dialog = document.getElementById('load-dialog');
        const list = document.getElementById('saved-scripts-list');

        list.innerHTML = '';

        const names = Object.keys(this.savedScripts);
        if (names.length === 0) {
            list.innerHTML = '<div style="padding: 16px; color: #8b949e; text-align: center;">No saved scripts</div>';
        } else {
            for (const name of names) {
                const script = this.savedScripts[name];
                const date = new Date(script.savedAt).toLocaleDateString();

                const item = document.createElement('div');
                item.className = 'saved-script-item';
                item.innerHTML = `
                    <div>
                        <div class="script-name">${name}</div>
                        <div class="script-date">${date}</div>
                    </div>
                    <button class="script-delete" onclick="notebook.deleteScript('${name}', event)" title="Delete">
                        <svg viewBox="0 0 24 24" width="14" height="14" fill="currentColor"><path d="M6 19c0 1.1.9 2 2 2h8c1.1 0 2-.9 2-2V7H6v12zM19 4h-3.5l-1-1h-5l-1 1H5v2h14V4z"/></svg>
                    </button>
                `;
                item.onclick = (e) => {
                    if (!e.target.closest('.script-delete')) {
                        this.load(name);
                        this.hideLoad();
                    }
                };
                list.appendChild(item);
            }
        }

        dialog.classList.add('open');
    },

    hideLoad() {
        document.getElementById('load-dialog').classList.remove('open');
    },

    load(name) {
        const script = this.savedScripts[name];
        if (!script) return;

        // Check if already open in a tab
        const existingTab = this.tabs.find(t => t.savedName === name);
        if (existingTab) {
            this.switchTab(existingTab.id);
            return;
        }

        // Create new tab with this content
        const tab = this.newTab(name, script.content);
        tab.savedName = name;
        tab.modified = false;
        this.renderTabs();
        this.updateStatus();
    },

    deleteScript(name, event) {
        if (event) event.stopPropagation();

        if (!confirm(`Delete script "${name}"?`)) return;

        delete this.savedScripts[name];
        this.saveToStorage();
        this.showLoad(); // Refresh the list

        terminal.print(`Script "${name}" deleted.`, 'warning');
    },

    saveToStorage() {
        try {
            localStorage.setItem('robotarm_scripts', JSON.stringify(this.savedScripts));
        } catch (e) {
            console.error('Failed to save to localStorage:', e);
        }
    },

    loadFromStorage() {
        try {
            const data = localStorage.getItem('robotarm_scripts');
            if (data) {
                this.savedScripts = JSON.parse(data);
            }
        } catch (e) {
            console.error('Failed to load from localStorage:', e);
            this.savedScripts = {};
        }
    },

    // Run the script directly (don't close window)
    run() {
        const content = this.editor.value.trim();
        if (!content) {
            terminal.print('Error: Script is empty', 'error');
            return;
        }

        // Filter out empty lines and comments
        const lines = content.split('\n')
            .map(l => l.trim())
            .filter(l => l && !l.startsWith('#'));

        if (lines.length === 0) {
            terminal.print('Error: No commands to run', 'error');
            return;
        }

        const tab = this.tabs.find(t => t.id === this.activeTabId);
        const name = tab?.savedName || tab?.name || 'script';

        // Don't close the notebook - keep it open!

        // Run the script
        terminal.print('');
        terminal.print(`━━━ RUNNING: ${name.toUpperCase()} ━━━`, 'warning');

        // Execute as a program
        runningProgram = {
            name: name,
            commands: [...lines],
            index: 0,
            loopStack: []
        };

        executeNextProgramCommand();
    }
};

// Make notebook global
window.notebook = notebook;

// ============================================================================
// INITIALIZATION
// ============================================================================

document.addEventListener('DOMContentLoaded', () => {
    initScene();
    terminal.init();
    notebook.init();

    // Focus terminal input
    document.getElementById('terminal-input').focus();

    // Click anywhere on terminal panel to focus input
    document.getElementById('terminal-panel').addEventListener('click', () => {
        document.getElementById('terminal-input').focus();
    });
});

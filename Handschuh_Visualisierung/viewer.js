import * as THREE from './three/build/three.module.js';
import { OrbitControls } from './three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from './three/examples/jsm/loaders/GLTFLoader.js';
import { SkeletonHelper } from './three/src/helpers/SkeletonHelper.js';

console.log("Hallo");

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x222222);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth/window.innerHeight, 0.1, 1000);
camera.position.set(0, 0, 0.3);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);

let skeleton;        // Skeleton des Handskeletts
let handRoot;        // Root-Node der Hand für globale Rotation
let selectedBone;    // Bone für Key-Steuerung
let logArray = [];   // Wird nach File-Input gefüllt

// Licht
const light = new THREE.DirectionalLight(0xffffff, 1);
light.position.set(5,5,5);
scene.add(light);
scene.add(new THREE.AmbientLight(0xffffff, 0.5));

// File-Input: JSON auswählen
document.getElementById('logFileInput').addEventListener('change', (event) => {
    const file = event.target.files[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
        try {
            // Jede Zeile als eigenes JSON-Objekt parsen
            const lines = e.target.result.split(/\r?\n/);
            logArray = lines
                .filter(line => line.trim().length > 0)   // leere Zeilen ignorieren
                .map(line => JSON.parse(line));           // JSON pro Zeile

            console.log("JSONL geladen, Anzahl Posen:", logArray.length);

            if (skeleton && handRoot) {
                playHandLogs(skeleton, handRoot, logArray, 30);
            }
        } catch (err) {
            console.error("Fehler beim Parsen der JSONL:", err);
        }
    };
    reader.readAsText(file);
});

// GLTF laden
const loader = new GLTFLoader();
loader.load('./Hand_eigen.glb', (gltf) => {
    handRoot = gltf.scene;
    handRoot.scale.set(0.1, 0.1, 0.1);
    scene.add(handRoot);

    handRoot.traverse(obj => {
        if (obj.isSkinnedMesh) {
            skeleton = obj.skeleton;

			skeleton.bones.forEach(bone => {
				const worldPos = new THREE.Vector3();
				bone.getWorldPosition(worldPos);
				console.log(`${bone.name}:`, worldPos.toArray());
			});

            // Bones sichtbar machen
            const helper = new SkeletonHelper(obj.skeleton.bones[0]);
            scene.add(helper);

			const axesHelper = new THREE.AxesHelper(0.1); // 0.1 = Länge der Achsen
			scene.add(axesHelper);

            console.log("Bones:", skeleton.bones.map(b => b.name));
        }
    });

    // Falls JSON schon geladen wurde
    if (logArray.length > 0) {
        playHandLogs(skeleton, handRoot, logArray, 30);
    }
});

// Globale Handrotation (BNO-Daten)
function setHandGlobalRotation(handRoot, sensors) {
    if (!sensors || !sensors.euler) return;

    const [roll, pitch, yaw] = sensors.euler.map(deg => deg * Math.PI / 180);

    // Three.js Yaw-Pitch-Roll
    handRoot.rotation.set(pitch, yaw, roll);
}

// Hand-Logs abspielen
function playHandLogs(skeleton, handRoot, logs, interval = 100) {
    let index = 0;
    
    const timer = setInterval(() => {
        if (!skeleton || !handRoot) return;
        if (index >= logs.length) {
            clearInterval(timer);
            return;
        }

        const log = logs[index];

        // 1️⃣ globale Hand-Orientierung
        setHandGlobalRotation(handRoot, log.sensors);

        // 2️⃣ Fingerpose
        setHandPose(skeleton, log.pose);

        index++;
    }, interval);
}

// Fingerpose setzen
function setHandPose(skeleton, poseLog) {
    const fingers = ["thumb", "index", "middle", "ring", "pinky"];
    
    fingers.forEach(fingerName => {
        const joints = poseLog[fingerName]; // Array von 3 oder 4 Vektoren
        if (!joints) return;

        // Bone-Namen passend zu deinem Skeleton
        let boneNames;
        switch(fingerName) {
            case "thumb": boneNames = ["Bone002", "Bone003", "Bone003_end"]; break;
            case "index": boneNames = ["IndexF_lower","IndexF_middle","IndexF_tip"]; break;
            case "middle": boneNames = ["MiddleF_lower","MiddleF_middle","MiddleF_tip"]; break;
            case "ring": boneNames = ["RingF_lower","RingF_middle","RingF_tip"]; break;
            case "pinky": boneNames = ["PinkyF_lower","PinkyF_middle","PinkyF_tip"]; break;
        }

        for (let i = 0; i < boneNames.length; i++) {
            const bone = skeleton.getBoneByName(boneNames[i]);
            if (!bone) continue;

            if (i + 1 >= joints.length) break; // Nur wenn ein nächstes Gelenk existiert

            const parent = new THREE.Vector3(...joints[i]);
            const target = new THREE.Vector3(...joints[i + 1]);
            const dir = new THREE.Vector3().subVectors(target, parent).normalize();

            const quaternion = new THREE.Quaternion().setFromUnitVectors(
                new THREE.Vector3(1, 0, 0), // Default Y-Achse der Bones
                dir
            );
            bone.quaternion.copy(quaternion);
        }
    });
}

// Key-Steuerung einzelner Bones
const ROT_STEP = 0.1;
window.addEventListener('keydown', (event) => {
    if (!selectedBone) return;

    switch(event.key) {
        case "ArrowUp": selectedBone.rotation.x -= ROT_STEP; break;
        case "ArrowDown": selectedBone.rotation.x += ROT_STEP; break;
        case "ArrowLeft": selectedBone.rotation.y -= ROT_STEP; break;
        case "ArrowRight": selectedBone.rotation.y += ROT_STEP; break;
        case "q": selectedBone.rotation.z += ROT_STEP; break;
        case "e": selectedBone.rotation.z -= ROT_STEP; break;
    }
});

// Animationsloop
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
animate();
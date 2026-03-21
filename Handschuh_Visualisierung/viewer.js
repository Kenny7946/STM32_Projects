import * as THREE from './three/build/three.module.js';
import { OrbitControls } from './three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from './three/examples/jsm/loaders/GLTFLoader.js';
import { SkeletonHelper } from './three/src/helpers/SkeletonHelper.js'; // Für Bones

console.log("Hallo");

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x222222);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth/window.innerHeight, 0.1, 1000);
camera.position.set(0, 0, 0.3);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);

const light = new THREE.DirectionalLight(0xffffff, 1);
light.position.set(5,5,5);
scene.add(light);
scene.add(new THREE.AmbientLight(0xffffff, 0.5));

let skeleton;        // Aktuelles Skeleton
let selectedBone;    // Bone, den wir mit Keys steuern

// GLTF laden
const loader = new GLTFLoader();
loader.load('./Hand_eigen.glb', (gltf) => {
    const hand = gltf.scene;
    hand.scale.set(0.1, 0.1, 0.1);
    scene.add(hand);

    hand.traverse(obj => {
        if (obj.isSkinnedMesh) {
            skeleton = obj.skeleton;

            // Bones sichtbar machen
            const helper = new SkeletonHelper(obj.skeleton.bones[0]);
            scene.add(helper);

            console.log("Bones:", skeleton.bones.map(b => b.name));
        }
    });

    // Wähle einen Bone zum Steuern, z.B. ersten Finger
    selectedBone = skeleton.getBoneByName("IndexRoot");  // <--- Name anpassen
    if (!selectedBone) {
        console.warn("Bone nicht gefunden!");
    }
});

// Rotation pro Tastendruck (in Radiant)
const ROT_STEP = 0.1;

// Key-Listener
window.addEventListener('keydown', (event) => {
    if (!selectedBone) return;

    switch(event.key) {
        case "ArrowUp":    // Bone nach oben rotieren
            selectedBone.rotation.x -= ROT_STEP;
            break;
        case "ArrowDown":  // Bone nach unten rotieren
            selectedBone.rotation.x += ROT_STEP;
            break;
        case "ArrowLeft":  // Bone nach links rotieren
            selectedBone.rotation.y -= ROT_STEP;
            break;
        case "ArrowRight": // Bone nach rechts rotieren
            selectedBone.rotation.y += ROT_STEP;
            break;
        case "q":          // Bone um Z positiv
            selectedBone.rotation.z += ROT_STEP;
            break;
        case "e":          // Bone um Z negativ
            selectedBone.rotation.z -= ROT_STEP;
            break;
    }
});

// Animationsloop
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
animate();
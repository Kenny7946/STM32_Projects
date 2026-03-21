import * as THREE from './three/build/three.module.js';
import { OrbitControls } from './three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from './three/examples/jsm/loaders/GLTFLoader.js';
import { SkeletonHelper } from './three/src/helpers/SkeletonHelper.js'; // Für Bones

console.log("Hallo");

// Szene und Kamera
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x222222);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth/window.innerHeight, 0.1, 1000);
camera.position.set(0, 0, 0.3);

// Renderer
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// Controls
const controls = new OrbitControls(camera, renderer.domElement);

// Licht
const light = new THREE.DirectionalLight(0xffffff, 1);
light.position.set(5,5,5);
scene.add(light);
scene.add(new THREE.AmbientLight(0xffffff, 0.5));

// Clock für Animationen
const clock = new THREE.Clock();

// Mixer für Animationen
let mixer;

// Modell laden
const loader = new GLTFLoader();
loader.load('./Hand_eigen.glb', (gltf) => {
    const hand = gltf.scene;
    hand.scale.set(0.1, 0.1, 0.1);
    scene.add(hand);

    hand.traverse(obj => {
        if (obj.isSkinnedMesh) {
            // Bones visualisieren
            const skeletonHelper = new SkeletonHelper(obj.skeleton.bones[0]);
            skeletonHelper.material.linewidth = 2;
            scene.add(skeletonHelper);

            console.log("Bones:", obj.skeleton.bones);
        }
    });

    // Animationen einrichten
    if (gltf.animations && gltf.animations.length > 0) {
        mixer = new THREE.AnimationMixer(hand);
        gltf.animations.forEach(clip => {
            mixer.clipAction(clip).play();
        });
        console.log("Animationen gestartet:", gltf.animations.map(a => a.name));
    }
});

// Animationsloop
function animate() {
    requestAnimationFrame(animate);

    const delta = clock.getDelta();
    if (mixer) mixer.update(delta); // Animation aktualisieren

    controls.update();
    renderer.render(scene, camera);
}
animate();
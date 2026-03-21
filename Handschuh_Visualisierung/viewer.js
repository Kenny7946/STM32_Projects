import * as THREE from './three/three.module.js';
import { OrbitControls } from './three/OrbitControls.js';
import { GLTFLoader } from './three/GLTFLoader.js';

console.log("Hallo");  // MUSS in der Konsole erscheinen

// Szene und Kamera
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x222222);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth/window.innerHeight, 0.1, 1000);
camera.position.set(0, 0, 5);

// Renderer
const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// Controls
const controls = new OrbitControls(camera, renderer.domElement);

// Licht
const light = new THREE.DirectionalLight(0xffffff, 1);
light.position.set(5,5,5);
scene.add(light);
scene.add(new THREE.AmbientLight(0xffffff, 0.5));

// Modell laden
let hand, skeleton;
const loader = new GLTFLoader();
loader.load('./Hand_eigen.glb', (gltf) => {
    console.log("Modell geladen");

    hand = gltf.scene;
    scene.add(hand);

    hand.scale.set(0.1, 0.1, 0.1);

    hand.traverse(obj => {
        if (obj.isSkinnedMesh) {
            skeleton = obj.skeleton;
            console.log("Bones:", skeleton.bones);
        }
    });
});

// Animationsloop
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
animate();
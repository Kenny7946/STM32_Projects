from direct.showbase.ShowBase import ShowBase
from panda3d.core import Point3, NodePath, DirectionalLight, AmbientLight, LineSegs
from direct.actor.Actor import Actor
import numpy as np

class HandViewer(ShowBase):
    def __init__(self, model_path):
        super().__init__()

        self.disableMouse()
        self.camera.setPos(0, 7, 3)
        self.camera.lookAt(0,0,0)

        # Licht
        dlight = DirectionalLight("dlight")
        dlight.setColor((2,2,2,1))
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setHpr(45,-45,0)
        self.render.setLight(dlnp)

        alight = AmbientLight("alight")
        alight.setColor((0.6,0.6,0.6,1))
        self.render.setLight(self.render.attachNewNode(alight))

        # Handmodell laden
        self.hand_mesh = Actor(model_path)
        self.hand_mesh.reparentTo(self.render)

        # Bones kontrollieren
        self.bones = {}
        finger_bone_names = {
            "index": ["IndexF_lower","IndexF_middle","IndexF_tip"],
            "middle": ["MiddleF_lower","MiddleF_middle","MiddleF_tip"],
            "ring": ["RingF_lower","RingF_middle","RingF_tip"],
            "pinky": ["PinkyF_lower","PinkyF_middle","PinkyF_tip"]
        }

        for finger, names in finger_bone_names.items():
            for name in names:
                self.bones[name] = self.hand_mesh.controlJoint(None, 'modelRoot', name)

        # Skelett-Kugeln und Linien erzeugen
        self.bone_spheres = {}
        self.bone_lines = LineSegs()
        self.bone_lines.setThickness(2.0)
        self.bone_lines.setColor(0,1,0,1)  # grün für Linien

        for bone_name, joint in self.bones.items():
            # Kugel für das Bone
            sphere = loader.loadModel("models/misc/sphere")
            sphere.reparentTo(joint)
            sphere.setScale(0.01)
            sphere.setColor(1,0,0,1)  # rot
            self.bone_spheres[bone_name] = sphere

        # Linien zwischen Bones
        for finger, names in finger_bone_names.items():
            for i in range(len(names)-1):
                child = self.bones[names[i+1]]
                parent = self.bones[names[i]]
                # Linie relativ zum render (global)
                self.bone_lines.moveTo(parent.getPos(self.render))
                self.bone_lines.drawTo(child.getPos(self.render))

        # -----------------------------
        # 1️⃣ Mesh-Informationen
        # -----------------------------
        mesh_np = self.hand_mesh.find('**/+GeomNode')
        if not mesh_np.isEmpty():
            min_bound, max_bound = mesh_np.getTightBounds()
            size_vec = max_bound - min_bound
            max_dim = max(size_vec.x, size_vec.y, size_vec.z)
            
            print("=== Mesh Info ===")
            print("Min bound:", min_bound)
            print("Max bound:", max_bound)
            print("Size vector:", size_vec)
            print("Max Dimension:", max_dim)
            print("Mesh Scale:", mesh_np.getScale())

        # -----------------------------
        # 2️⃣ Bone-Informationen
        # -----------------------------
        print("\n=== Bone Info ===")
        for name, joint in self.bones.items():
            # Lokale Position im Actor
            pos_local = joint.getPos()
            # Welt-Position
            pos_world = joint.getPos(self.render)
            print(f"{name}: Local Pos = {pos_local}, World Pos = {pos_world}")



        # NodePath für Linien
        self.skeleton_lines = self.render.attachNewNode(self.bone_lines.create())

        # Kamera-Orbit/Pan-Variablen
        self.orbit_center = Point3(0,0,0)
        self.orbit_distance = 7
        self.orbit_angle_h = 0
        self.orbit_angle_v = 20
        self.mouse_sensitivity = 0.2
        self.pan_sensitivity = 0.005
        self.prev_mouse_x = None
        self.prev_mouse_y = None

        self.accept("wheel_up", self.zoom_in)
        self.accept("wheel_down", self.zoom_out)

        self.taskMgr.add(self.update_task, "update_task")
        self.taskMgr.add(self.camera_task, "camera_task")

    def update_task(self, task):
        t = task.time

        # Beispiel Finger-Bewegung (Index Finger leicht wippen)
        self.bones["IndexF_lower"].setHpr(
            self.bones["IndexF_lower"].getHpr().x,
            self.bones["IndexF_lower"].getHpr().y + 0*np.sin(t),
            self.bones["IndexF_lower"].getHpr().z
        )
        self.bones["IndexF_middle"].setHpr(
            self.bones["IndexF_middle"].getHpr().x,
            self.bones["IndexF_middle"].getHpr().y + 0.008*np.sin(t),
            self.bones["IndexF_middle"].getHpr().z
        )
        self.bones["IndexF_tip"].setHpr(
            self.bones["IndexF_tip"].getHpr().x,
            self.bones["IndexF_tip"].getHpr().y + 0*np.sin(t),
            self.bones["IndexF_tip"].getHpr().z
        )

        # Linien aktualisieren
        self.bone_lines.reset()
        self.bone_lines.setThickness(2.0)
        self.bone_lines.setColor(0,1,0,1)
        finger_bone_names = {
            "index": ["IndexF_lower","IndexF_middle","IndexF_tip"],
            "middle": ["MiddleF_lower","MiddleF_middle","MiddleF_tip"],
            "ring": ["RingF_lower","RingF_middle","RingF_tip"],
            "pinky": ["PinkyF_lower","PinkyF_middle","PinkyF_tip"]
        }
        for finger, names in finger_bone_names.items():
            for i in range(len(names)-1):
                parent = self.bones[names[i]]
                child = self.bones[names[i+1]]
                self.bone_lines.moveTo(parent.getPos(self.render))
                self.bone_lines.drawTo(child.getPos(self.render))

        # Update Linien NodePath
        self.skeleton_lines.removeNode()
        self.skeleton_lines = self.render.attachNewNode(self.bone_lines.create())

        return task.cont

    def camera_task(self, task):
        pointer = base.win.getPointer(0)
        x = pointer.getX()
        y = pointer.getY()

        # Orbit (linke Maustaste)
        if base.mouseWatcherNode.isButtonDown("mouse1"):
            if self.prev_mouse_x is not None and self.prev_mouse_y is not None:
                dx = x - self.prev_mouse_x
                dy = y - self.prev_mouse_y
                self.orbit_angle_h += dx * self.mouse_sensitivity
                self.orbit_angle_v = np.clip(self.orbit_angle_v + dy * self.mouse_sensitivity, -89, 89)
            self.prev_mouse_x = x
            self.prev_mouse_y = y

        # Pan (rechte Maustaste)
        elif base.mouseWatcherNode.isButtonDown("mouse3"):
            if self.prev_mouse_x is not None and self.prev_mouse_y is not None:
                dx = x - self.prev_mouse_x
                dy = y - self.prev_mouse_y
                right = self.camera.getQuat(self.render).getRight()
                up = self.camera.getQuat(self.render).getUp()
                self.orbit_center += -right * dx * self.pan_sensitivity
                self.orbit_center += up * dy * self.pan_sensitivity
            self.prev_mouse_x = x
            self.prev_mouse_y = y
        else:
            self.prev_mouse_x = None
            self.prev_mouse_y = None

        # Kamera-Position berechnen
        rad_h = np.radians(self.orbit_angle_h)
        rad_v = np.radians(self.orbit_angle_v)
        cam_x = self.orbit_center.x + self.orbit_distance * np.cos(rad_v) * np.sin(rad_h)
        cam_y = self.orbit_center.y + self.orbit_distance * np.cos(rad_v) * np.cos(rad_h)
        cam_z = self.orbit_center.z + self.orbit_distance * np.sin(rad_v)
        self.camera.setPos(cam_x, cam_y, cam_z)
        self.camera.lookAt(self.orbit_center)

        return task.cont

    def zoom_in(self):
        self.orbit_distance = max(0.5, self.orbit_distance - 0.3)

    def zoom_out(self):
        self.orbit_distance += 0.3


viewer = HandViewer("model/realistic_hand.glb")
viewer.run()
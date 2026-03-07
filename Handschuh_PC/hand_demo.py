from direct.showbase.ShowBase import ShowBase
from panda3d.core import Point3, NodePath, DirectionalLight, AmbientLight
from direct.actor.Actor import Actor
import numpy as np

example_pose = {
    "index": [np.array([0.02,0,0]), np.array([0.04,0.03,-0.03]),
              np.array([0.03,0.03,-0.05]), np.array([0.01,0.02,-0.06])],
    "middle":[np.array([0.03,0,0]), np.array([0.05,0.02,-0.03]),
              np.array([0.04,0.01,-0.06]), np.array([0.02,0.007,-0.067])],
    "ring":[np.array([0.035,-0.007,0]), np.array([0.057,0,-0.035]),
            np.array([0.046,-0.004,-0.061]), np.array([0.029,-0.011,-0.064])],
    "pinky":[np.array([0.038,-0.027,0]), np.array([0.056,-0.02,-0.028]),
             np.array([0.049,-0.024,-0.046]), np.array([0.032,-0.031,-0.049])]
}

class HandViewer(ShowBase):
    def __init__(self, model_path):
        super().__init__()

        self.disableMouse()
        self.camera.setPos(0, 7, 3)
        self.camera.lookAt(0,0,0)

        # Licht
        dlight = DirectionalLight("dlight")
        dlight.setColor((1,1,1,1))
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setHpr(45,-45,0)
        self.render.setLight(dlnp)

        alight = AmbientLight("alight")
        alight.setColor((0.3,0.3,0.3,1))
        self.render.setLight(self.render.attachNewNode(alight))

        # Handmodell
        self.hand_mesh = Actor(model_path)
        self.hand_mesh.reparentTo(self.render)
        self.hand_mesh.setScale(1)

        # Gelenke
        self.joint_nodes = {}
        for finger, joints in example_pose.items():
            self.joint_nodes[finger] = []
            for i in range(len(joints)):
                np_node = self.render.attachNewNode(f"{finger}_{i}")
                sphere = loader.loadModel("models/misc/sphere")
                sphere.reparentTo(np_node)
                sphere.setScale(0.005)
                sphere.setColor(1,0,0,1)
                self.joint_nodes[finger].append(np_node)

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
        for finger, joints in example_pose.items():
            for i, pos in enumerate(joints):
                self.joint_nodes[finger][i].setPos(Point3(*pos))
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
                # Kamera-Panning relativ zur aktuellen Ansicht
                right = self.camera.getQuat(self.render).getRight()
                up = self.camera.getQuat(self.render).getUp()
                self.orbit_center += -right * dx * self.pan_sensitivity
                self.orbit_center += up * dy * self.pan_sensitivity
            self.prev_mouse_x = x
            self.prev_mouse_y = y
        else:
            # keine Taste gedrückt
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
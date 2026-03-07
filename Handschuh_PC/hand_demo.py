from direct.showbase.ShowBase import ShowBase
from panda3d.core import Point3, NodePath, DirectionalLight, AmbientLight
from panda3d.core import GeomNode
from direct.actor.Actor import Actor
import numpy as np

# Beispielpose (4 Gelenke pro Finger)
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

        # Mesh laden
        self.hand_mesh = Actor(model_path)
        self.hand_mesh.reparentTo(self.render)
        self.hand_mesh.setScale(1)

        # Gelenk-Visualisierung: kleine Kugeln für jeden Joint
        self.joint_nodes = {}
        for finger, joints in example_pose.items():
            self.joint_nodes[finger] = []
            for i in range(len(joints)):
                np_node = self.render.attachNewNode(f"{finger}_{i}")
                sphere = loader.loadModel("models/misc/sphere")  # Panda3D Standardkugel
                sphere.reparentTo(np_node)
                sphere.setScale(0.005)
                sphere.setColor(1,0,0,1)  # rot
                self.joint_nodes[finger].append(np_node)

        self.taskMgr.add(self.update_task, "update_task")

    def update_task(self, task):
        # Gelenke setzen
        for finger, joints in example_pose.items():
            for i, pos in enumerate(joints):
                self.joint_nodes[finger][i].setPos(Point3(*pos))
        return task.cont

# Pfad zu deinem Modell
viewer = HandViewer("model/realistic_hand.glb")
viewer.run()
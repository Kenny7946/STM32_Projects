from direct.showbase.ShowBase import ShowBase
from direct.actor.Actor import Actor
import numpy as np


class HandTFDemo(ShowBase):

    def __init__(self):
        super().__init__()

        # Handmodell laden
        self.hand = Actor("model/hand.glb")
        self.hand.reparentTo(self.render)

        self.camera.setPos(0, 7, 3)
        self.camera.lookAt(0,0,0)

        # alle Joint-Namen
        self.joint_names = [
            "IndexRoot",
            "IndexF_lower",
            "IndexF_middle",
            "IndexF_tip"
        ]

        # NodePaths zu den Joints
        self.joints = {}

        for name in self.joint_names:
            self.joints[name] = self.hand.expose_joint(None, "modelRoot", name)

        # TF dictionary
        self.tf = {}

        # Joint zum Bewegen
        self.index_lower = self.hand.control_joint(None, "modelRoot", "IndexF_lower")

        self.taskMgr.add(self.update_task, "update")


    def compute_tf_tree(self):
        """Berechnet Weltpose aller Joints"""

        for name, joint in self.joints.items():

            pos = joint.getPos(self.render)
            quat = joint.getQuat(self.render)

            self.tf[name] = {
                "pos": pos,
                "quat": quat
            }


    def print_tf(self):
        """Debug Ausgabe ähnlich ROS tf_echo"""

        for name, tf in self.tf.items():

            pos = tf["pos"]
            quat = tf["quat"]

            print(f"{name}")
            print(f"  pos  : {pos}")
            print(f"  quat : {quat}")


    def update_task(self, task):

        t = task.time

        # Finger bewegen
        h, p, r = self.index_lower.getHpr()
        self.index_lower.setHpr(h, p, r + 30 * np.sin(t))

        # TF berechnen
        self.compute_tf_tree()

        # Beispiel: nur Fingerkette ausgeben
        print("\nTF tree:")
        for name in self.joint_names:
            pos = self.tf[name]["pos"]
            print(name, pos)

        return task.cont


app = HandTFDemo()
app.run()
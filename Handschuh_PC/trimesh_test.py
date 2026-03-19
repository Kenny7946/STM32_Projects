import numpy as np
import trimesh
import pyrender
import time

# Szene erstellen
scene = pyrender.Scene()

# einfache Box-Meshes als Platzhalter
palm_mesh = pyrender.Mesh.from_trimesh(trimesh.creation.box(extents=(0.04, 0.04, 0.02)))
finger1_mesh = pyrender.Mesh.from_trimesh(trimesh.creation.box(extents=(0.01, 0.01, 0.03)))
finger2_mesh = pyrender.Mesh.from_trimesh(trimesh.creation.box(extents=(0.01, 0.01, 0.025)))
finger3_mesh = pyrender.Mesh.from_trimesh(trimesh.creation.box(extents=(0.01, 0.01, 0.02)))

# Nodes erstellen
palm_node = pyrender.Node(mesh=palm_mesh)

finger1_node = pyrender.Node(
    mesh=finger1_mesh,
    translation=[0, 0, 0.03]
)

finger2_node = pyrender.Node(
    mesh=finger2_mesh,
    translation=[0, 0, 0.03]
)

finger3_node = pyrender.Node(
    mesh=finger3_mesh,
    translation=[0, 0, 0.025]
)

# Hierarchie aufbauen
scene.add_node(palm_node)
scene.add_node(finger1_node, parent_node=palm_node)
scene.add_node(finger2_node, parent_node=finger1_node)
scene.add_node(finger3_node, parent_node=finger2_node)

# Kamera
camera = pyrender.PerspectiveCamera(yfov=np.pi / 3.0)
cam_pose = np.array([
    [1,0,0,0],
    [0,1,0,-0.1],
    [0,0,1,0.2],
    [0,0,0,1]
])

scene.add(camera, pose=cam_pose)

# Licht
light = pyrender.DirectionalLight(color=[1,1,1], intensity=3)
scene.add(light, pose=cam_pose)

# Viewer starten (läuft in Thread)
viewer = pyrender.Viewer(scene, use_raymond_lighting=True, run_in_thread=True)

angle = 0

while True:

    angle += 0.03

    rot1 = trimesh.transformations.rotation_matrix(angle, [1,0,0])
    rot2 = trimesh.transformations.rotation_matrix(angle*0.8, [1,0,0])
    rot3 = trimesh.transformations.rotation_matrix(angle*0.5, [1,0,0])

    scene.set_pose(finger1_node, rot1)
    scene.set_pose(finger2_node, rot2)
    scene.set_pose(finger3_node, rot3)

    time.sleep(0.016)
import numpy as np
from hand_visualizer import start_viewer

def dummy_pose():
    return {
    "thumb":  [np.array([0,0,0]),
                np.array([0.01,0.03,0.01]),
                np.array([0.02,0.05,0.02])],

    "index":  [np.array([0.02,0,0]),
                np.array([0.02,0.04,0.01]),
                np.array([0.02,0.07,0.02]),
                np.array([0.02,0.09,0.03])],

    "middle": [np.array([0,0,0]),
                np.array([0,0.05,0.01]),
                np.array([0,0.09,0.02]),
                np.array([0,0.12,0.03])],

    "ring":   [np.array([-0.02,0,0]),
                np.array([-0.02,0.04,0.01]),
                np.array([-0.02,0.07,0.02]),
                np.array([-0.02,0.09,0.03])],

    "pinky":  [np.array([-0.04,0,0]),
                np.array([-0.04,0.03,0.01]),
                np.array([-0.04,0.05,0.02]),
                np.array([-0.04,0.07,0.03])]
}

start_viewer(dummy_pose)
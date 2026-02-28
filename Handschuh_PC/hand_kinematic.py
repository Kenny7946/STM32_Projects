import numpy as np
from scipy.spatial.transform import Rotation as R

# -----------------------------
# Kalibrierungswerte (anpassen!)
# -----------------------------
ADC_MIN = [0, 0, 0, 0, 0]
ADC_MAX = [4095, 4095, 4095, 4095, 4095]

# maximale Fingerbeugung
MAX_FLEX_DEG = 90

# Fingersegmentlängen (Meter)
FINGER_LENGTHS = {
    "thumb":  [0.035, 0.025],
    "index":  [0.04, 0.025, 0.02],
    "middle": [0.045, 0.03, 0.02],
    "ring":   [0.043, 0.028, 0.02],
    "pinky":  [0.035, 0.02, 0.018],
}

# Fingerbasen relativ zur Handfläche
FINGER_BASES = {
    "thumb":  np.array([-0.04, 0.02, 0]),
    "index":  np.array([-0.02, 0.03, 0]),
    "middle": np.array([0.0, 0.035, 0]),
    "ring":   np.array([0.02, 0.03, 0]),
    "pinky":  np.array([0.04, 0.025, 0]),
}


def normalize_adc(value, idx):
    v = (value - ADC_MIN[idx]) / (ADC_MAX[idx] - ADC_MIN[idx])
    return np.clip(v, 0, 1)


def adc_to_angle(adc_value, idx):
    norm = normalize_adc(adc_value, idx)
    return np.deg2rad(norm * MAX_FLEX_DEG)


def compute_finger_positions(base_pos, lengths, angle):
    """
    einfache Vorwärtskinematik in einer Ebene
    """
    positions = [base_pos]
    direction = np.array([0, 1, 0])

    current_angle = 0

    for L in lengths:
        current_angle += angle
        rot = R.from_euler('z', current_angle).apply(direction)
        new_pos = positions[-1] + rot * L
        positions.append(new_pos)

    return positions


def compute_hand_pose(sensor_data):
    adc_values = [
        sensor_data["adc0"],
        sensor_data["adc1"],
        sensor_data["adc2"],
        sensor_data["adc3"],
        sensor_data["adc4"],
    ]

    # Handorientierung aus Euler
    roll, pitch, yaw = sensor_data["euler"]
    hand_rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)

    hand_position = np.array([0, 0, 0])

    pose = {}

    for idx, finger in enumerate(FINGER_LENGTHS.keys()):
        angle = adc_to_angle(adc_values[idx], idx)

        base = hand_position + hand_rot.apply(FINGER_BASES[finger])
        joints = compute_finger_positions(base, FINGER_LENGTHS[finger], angle)

        # Orientierung der Hand anwenden
        joints_world = [hand_rot.apply(p - hand_position) + hand_position for p in joints]

        pose[finger] = joints_world

    return pose


# -----------------------------
# Beispiel Test
# -----------------------------
if __name__ == "__main__":
    test_data = {
        "adc0": 400,
        "adc1": 500,
        "adc2": 600,
        "adc3": 450,
        "adc4": 550,
        "euler": [10, 20, 5],
    }

    pose = compute_hand_pose(test_data)

    for finger, joints in pose.items():
        print(f"\n{finger}")
        for j in joints:
            print(j)
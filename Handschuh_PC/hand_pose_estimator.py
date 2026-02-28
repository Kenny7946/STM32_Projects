import numpy as np
from scipy.spatial.transform import Rotation as R


class HandPoseEstimator:
    """
    Berechnet 3D Hand- und Fingerpositionen
    aus ADC-Flexsensoren + IMU Orientierung.
    """

    def __init__(
        self,
        adc_min=None,
        adc_max=None,
        max_flex_deg=90,
        finger_lengths=None,
        finger_bases=None,
    ):
        # -----------------------------
        # Kalibrierung
        # -----------------------------
        self.adc_min = adc_min or [0, 0, 0, 0, 0]
        self.adc_max = adc_max or [4095, 4095, 4095, 4095, 4095]

        self.max_flex_deg = max_flex_deg

        # -----------------------------
        # Anatomisches Modell
        # -----------------------------
        self.finger_lengths = finger_lengths or {
            "thumb":  [0.035, 0.025],
            "index":  [0.04, 0.025, 0.02],
            "middle": [0.045, 0.03, 0.02],
            "ring":   [0.043, 0.028, 0.02],
            "pinky":  [0.035, 0.02, 0.018],
        }

        self.finger_bases = finger_bases or {
            "thumb":  np.array([-0.04, 0.02, 0]),
            "index":  np.array([-0.02, 0.03, 0]),
            "middle": np.array([0.0, 0.035, 0]),
            "ring":   np.array([0.02, 0.03, 0]),
            "pinky":  np.array([0.04, 0.025, 0]),
        }

    # =========================================================
    # ADC Verarbeitung
    # =========================================================

    def normalize_adc(self, value, idx):
        """Normiert ADC auf Bereich 0…1"""
        v = (value - self.adc_min[idx]) / (self.adc_max[idx] - self.adc_min[idx])
        return np.clip(v, 0.0, 1.0)

    def adc_to_angle(self, adc_value, idx):
        """ADC → Beugewinkel (Radiant)"""
        norm = self.normalize_adc(adc_value, idx)
        return np.deg2rad(norm * self.max_flex_deg)

    # =========================================================
    # Fingerkinematik
    # =========================================================

    def compute_finger_positions(self, base_pos, lengths, angle):
        """
        Einfache Vorwärtskinematik in einer Ebene.
        """
        positions = [base_pos]
        direction = np.array([0, 1, 0])
        current_angle = 0

        for L in lengths:
            current_angle += angle
            rot = R.from_euler("z", current_angle).apply(direction)
            new_pos = positions[-1] + rot * L
            positions.append(new_pos)

        return positions

    # =========================================================
    # Orientierung
    # =========================================================

    def _rotation_from_sensor(self, sensor_data):
        """
        Erstellt Rotation aus Euler oder Quaternion.
        """
        if "quat" in sensor_data:
            return R.from_quat(sensor_data["quat"])

        if "euler" in sensor_data:
            roll, pitch, yaw = sensor_data["euler"]
            return R.from_euler("xyz", [roll, pitch, yaw], degrees=True)

        raise ValueError("Sensor data requires 'euler' or 'quat'.")

    # =========================================================
    # Hauptfunktion
    # =========================================================

    def compute_pose(self, sensor_data, hand_position=np.zeros(3)):
        """
        Berechnet die komplette Handpose.

        Returns:
            dict[str, list[np.array]]
            -> Gelenkpositionen pro Finger
        """

        adc_values = [
            sensor_data["adc0"],
            sensor_data["adc1"],
            sensor_data["adc2"],
            sensor_data["adc3"],
            sensor_data["adc4"],
        ]

        hand_rot = self._rotation_from_sensor(sensor_data)

        pose = {}

        for idx, finger in enumerate(self.finger_lengths.keys()):

            angle = self.adc_to_angle(adc_values[idx], idx)

            base = hand_position + hand_rot.apply(self.finger_bases[finger])

            joints = self.compute_finger_positions(
                base,
                self.finger_lengths[finger],
                angle,
            )

            joints_world = [
                hand_rot.apply(p - hand_position) + hand_position
                for p in joints
            ]

            pose[finger] = joints_world

        return pose
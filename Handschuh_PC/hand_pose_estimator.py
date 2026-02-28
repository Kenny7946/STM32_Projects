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
            "middle": np.array([0.0, 0.03, 0]),
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

    def compute_finger_positions(self, base_pos, lengths, angle, plane_normal=np.array([1,0,0])):
        """
        Vorwärtskinematik: Finger lokal in Beugeebene biegen.
        
        base_pos: Startpunkt (Weltkoordinaten)
        lengths: Liste der Segmentlängen
        angle: Beugewinkel pro Segment (Rad)
        plane_normal: Achse, um die jeder Finger lokal rotiert (Finger beugt entlang dieser Achse)
        """
        positions = [base_pos]
        direction = np.array([0, 1, 0])  # Finger zeigt initial entlang Y
        current_rot = R.identity()

        for L in lengths:
            # Rotation um die lokale Beugeachse
            rot = R.from_rotvec(angle * plane_normal)
            current_rot = current_rot * rot
            new_pos = positions[-1] + current_rot.apply(direction) * L
            positions.append(new_pos)

        return positions

    # -----------------------------
    # Handrotation aus Sensor
    # -----------------------------
    def _rotation_from_sensor(self, sensor_data):
        """
        Rotation aus Euler oder Quaternion erstellen.
        BNO055 liefert Roll=X, Pitch=Y, Yaw=Z
        Euler-Winkel werden als INTRINSISCHE Rotationen angewendet (lokale Achsen)
        """
        if "euler" in sensor_data:
            pitch, roll, yaw = sensor_data["euler"]
            return R.from_euler("xyz", [roll, pitch, yaw], degrees=True)

        raise ValueError("Sensor data requires 'euler' or 'quat'.")

    # -----------------------------
    # Hauptfunktion: Pose berechnen
    # -----------------------------
    def compute_pose(self, sensor_data, hand_position=np.zeros(3)):
        """
        Debug: nur ein Finger, keine Beugung, zeigt Handrotation
        """
        adc_values = [
            sensor_data["adc0"],
            sensor_data["adc1"],
            sensor_data["adc2"],
            sensor_data["adc3"],
            sensor_data["adc4"],
        ]
        
        # # Rotation der Hand aus Euler (intrinsisch xyz)
        # hand_rot = self._rotation_from_sensor(sensor_data)

        # # Finger auf Basis setzen
        # angle = self.adc_to_angle(adc_values[2], 2)
        # start = hand_position + hand_rot.apply(self.finger_bases["middle"])
        # joints_local = self.compute_finger_positions(
        #     base_pos=np.zeros(3),  # Start bei 0, wir addieren Handbasis später
        #     lengths=self.finger_lengths["middle"],
        #     angle=angle,
        #     plane_normal=np.array([-1,0,0])
        # )

        # joints_world = [start + hand_rot.apply(j) for j in joints_local]

        # pose = {}
        
        # pose["middle"] = joints_world

        pose = {}

        hand_rot = self._rotation_from_sensor(sensor_data)

        for idx, finger in enumerate(self.finger_lengths.keys()):
            #if finger == "middle":
            angle = self.adc_to_angle(adc_values[idx], idx)

            start = hand_position + hand_rot.apply(self.finger_bases[finger])

            joints_local = self.compute_finger_positions(
                base_pos=np.zeros(3),  # Start bei 0, wir addieren Handbasis später
                lengths=self.finger_lengths[finger],
                angle=angle,
                plane_normal=np.array([-1,0,0]))

            joints_world = [start + hand_rot.apply(j) for j in joints_local]

            pose[finger] = joints_world

        return pose
    
from hand_pose_estimator import HandPoseEstimator

estimator = HandPoseEstimator()

sensor_data = {
    "adc0": 2100,
    "adc1": 1800,
    "adc2": 1500,
    "adc3": 2200,
    "adc4": 2600,
    "euler": [10, 15, 5],
}

pose = estimator.compute_pose(sensor_data)

for finger, joints in pose.items():
    print(finger)
    for j in joints:
        print(j)
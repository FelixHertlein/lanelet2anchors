import numpy as np

from lanelet2anchors.anchor_tools.lanelet_matching import VehiclePose

NUSC_ORIGIN_MAP = {
    "boston-seaport": [42.336849169438615, -71.05785369873047],
    "singapore-onenorth": [1.2882100868743724, 103.78475189208984],
    "singapore-hollandvillage": [1.2993652317780957, 103.78217697143555],
    "singapore-queenstown": [1.2782562240223188, 103.76741409301758],
}


def load_agent_from_sample(nusc, sample_token, width, length):
    sample_record = nusc.get("sample", sample_token)
    sample_data_record = nusc.get("sample_data", sample_record["data"]["LIDAR_TOP"])
    pose_record = nusc.get("ego_pose", sample_data_record["ego_pose_token"])
    timestamp = pose_record["timestamp"]
    translation = np.array(pose_record["translation"])
    rotation = pose_record["rotation"]
    vehicle_pose = VehiclePose.from_nusc(
        translation[0], translation[1], rotation, width, length
    )
    return vehicle_pose, timestamp

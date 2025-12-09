# ruff: noqa: D103

import pickle
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from handeye_4dof import Calibrator4DOF, robot_pose_selector
from scipy.spatial.transform import Rotation

np.set_printoptions(suppress=True)


def main() -> None:
    print("Hi from handeye_4dof.")

    example_data_directory = Path(get_package_share_directory("handeye_4dof_ros")) / "example_data"
    pose_samples_path = example_data_directory / "pose_samples.pkl"
    with pose_samples_path.open("rb") as f:
        try:
            base_to_hand, camera_to_marker = pickle.load(f)  # noqa: S301
        except UnicodeDecodeError:
            # python 2 to python 3 pickle in case sampling was done in ROS
            base_to_hand, camera_to_marker = pickle.load(f, encoding="latin1")  # noqa: S301

    # Obtain optimal motions as dual quaternions.
    motions = robot_pose_selector(camera_to_marker, base_to_hand)

    # Initialize calibrator with precomputed motions.
    cb = Calibrator4DOF(motions)

    # Our camera and end effector z-axes are antiparallel so we apply a 180deg x-axis rotation.
    dq_x = cb.calibrate(antiparallel_screw_axes=True)

    # Hand to Camera TF obtained from handeye calibration.
    ca_hand_to_camera = np.linalg.inv(dq_x.as_transform())

    # Hand to Camera TF obtained from post nonlinear refinement.
    nl_hand_to_camera = cb.nonlinear_refinement(camera_to_marker, base_to_hand, ca_hand_to_camera)

    ca_rotation = np.rad2deg(Rotation.from_matrix(ca_hand_to_camera[:3, :3]).as_euler("xyz"))
    nl_rotation = np.rad2deg(Rotation.from_matrix(nl_hand_to_camera[:3, :3]).as_euler("xyz"))

    # Ground Truth Hand to Camera
    gt_translation = [-0.456, -0.037, -0.112]
    gt_rotation = [180, 0, 0]

    # NOTE: (1) Ground Truth itself may be inaccurate (manually measured).
    #       (2) z-translation is an invalid number.
    np.set_printoptions(precision=5)
    print("Hand to Camera Transform Comparisons")
    print(f"Translations: Calibration  {ca_hand_to_camera[:3, -1]}")
    print(f"              Nonlinear    {nl_hand_to_camera[:3, -1]}")
    print(f"              Ground Truth {gt_translation}")
    print(f"Rotations:    Calibration  {ca_rotation}")
    print(f"              Nonlinear    {nl_rotation}")
    print(f"              Ground Truth {gt_rotation}")


if __name__ == "__main__":
    main()

import pickle
from pathlib import Path

import rclpy
import rclpy.time
from rclpy.node import Node

import numpy as np
import std_srvs.srv
from geometry_msgs.msg import Transform, TransformStamped
from handeye_4dof import Calibrator4DOF
from scipy.spatial.transform import Rotation
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener

np.set_printoptions(suppress=True)

SAMPLES_DIRECTORY = Path("/builder/samples")


def tf_to_homogenous_matrix(transform: Transform) -> np.ndarray:
    """Convert a TF2 transform to a homogenous transformation matrix."""
    # Quaternion to rotation matrix
    rot_matrix = Rotation.from_quat(
        [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
    ).as_matrix()

    # Translation vector as column vector
    translation_vector = np.array([[transform.translation.x, transform.translation.y, transform.translation.z]]).T

    # Construct homogenous array
    homogenous_matrix = np.eye(4)
    homogenous_matrix[:3, :3] = rot_matrix
    homogenous_matrix[:3, -1] = translation_vector

    return homogenous_matrix


def homogenous_matrix_to_tf(matrix: np.ndarray) -> Transform:
    """Convert a homogenous transformation matrix to a TF2 transform."""
    transform = Transform()

    rot_quat = Rotation.from_matrix(matrix[:3, :3]).as_quat(canonical=False)
    transform.rotation.x = rot_quat[0]
    transform.rotation.y = rot_quat[1]
    transform.rotation.z = rot_quat[2]
    transform.rotation.w = rot_quat[3]

    transform.translation.x = matrix[0, -1]
    transform.translation.y = matrix[1, -1]
    transform.translation.z = matrix[2, -1]

    return transform


class Calibrator4DofNode(Node):
    """Calibration node for 4-DoF robots."""

    def __init__(self) -> None:
        """Initialize the node."""
        super().__init__("calibrator_4dof")

        # Declare and acquire parameters
        self.init_parameters()

        # Declare services
        self.srv_update_handeye = self.create_service(std_srvs.srv.SetBool, "/update_handeye", self.cb_update_handeye)

        # TF2 initialization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Arrays for internal storage
        self.transform_pairs: list[tuple[np.ndarray, np.ndarray]] = []

    def init_parameters(self) -> None:
        """Initialize all parameters and store the results internally."""
        # TF frame names
        self.frame_robot_base = self.declare_parameter("tf.robot.base", "base_link").get_parameter_value().string_value
        self.frame_robot_ee = self.declare_parameter("tf.robot.ee", "link_6").get_parameter_value().string_value
        self.frame_sensor_camera = (
            self.declare_parameter("tf.sensor.camera", "camera0_color_optical_frame").get_parameter_value().string_value
        )
        self.frame_sensor_pattern = (
            self.declare_parameter("tf.sensor.pattern", "tf_pattern").get_parameter_value().string_value
        )

        self.frame_result = self.declare_parameter("tf.result", "tf_result").get_parameter_value().string_value

        # Calibration configuration
        self.eye_in_hand = self.declare_parameter("eye_in_hand", False).get_parameter_value().bool_value
        self.min_samples_stored = self.declare_parameter("min_stored", 3).get_parameter_value().integer_value

    def lookup_transform(
        self, target_frame: str, source_frame: str, time: rclpy.time.Time | None = None
    ) -> TransformStamped:
        """Look up a transformation in TF2."""
        if time is None:
            time = rclpy.time.Time()

        return self.tf_buffer.lookup_transform(target_frame, source_frame, time)

    def cb_update_handeye(
        self,
        req: std_srvs.srv.SetBool.Request,
        res: std_srvs.srv.SetBool.Response,
    ) -> std_srvs.srv.SetBool.Response:
        """Service callback to take a sample and calculate calibration if enough samples are collected."""
        try:
            # Camera <-> Pattern
            tf_sensor_c2p = self.lookup_transform(self.frame_sensor_pattern, self.frame_sensor_camera)
            tf_sensor_p2c = self.lookup_transform(self.frame_sensor_camera, self.frame_sensor_pattern)
            # Robot base <-> end effector
            tf_robot_b2e = self.lookup_transform(self.frame_robot_ee, self.frame_robot_base)
            tf_robot_e2b = self.lookup_transform(self.frame_robot_base, self.frame_robot_ee)
        except TransformException as ex:
            err_msg = f"Could not find a required transform, reason: {ex}"
            self.get_logger().error(err_msg)
            res.message = err_msg
            res.success = False
            return res

        # TODO: Check if transforms are too old

        # Store transformation internally
        if self.eye_in_hand:
            sensor_tf_matrix = tf_to_homogenous_matrix(tf_sensor_c2p.transform)
            robot_tf_matrix = tf_to_homogenous_matrix(tf_robot_b2e.transform)
        else:
            sensor_tf_matrix = tf_to_homogenous_matrix(tf_sensor_p2c.transform)
            robot_tf_matrix = tf_to_homogenous_matrix(tf_robot_e2b.transform)
        self.transform_pairs.append((sensor_tf_matrix, robot_tf_matrix))

        # Check if there are enough samples to compute a calibration
        if len(self.transform_pairs) < self.min_samples_stored:
            self.save_samples_to_disk()
            # Successfully saved transformations, but did not compute a calibration since there are not enough samples
            res.message = f"Saved: {len(self.transform_pairs)} samples"
            res.success = True
            return res

        # There are enough samples to try to compute a calibration
        calibrator = Calibrator4DOF(motions=self.transform_pairs)
        calibration_tf_matrix = calibrator.calibrate().as_transform()

        calibration_tf_msg = TransformStamped()
        calibration_tf_msg.header.stamp = self.get_clock().now().to_msg()

        if self.eye_in_hand:
            calibration_tf_msg.transform = homogenous_matrix_to_tf(np.linalg.inv(calibration_tf_matrix))
            calibration_tf_msg.header.frame_id = self.frame_robot_ee
            calibration_tf_msg.child_frame_id = self.frame_sensor_camera
        else:
            calibration_tf_msg.transform = homogenous_matrix_to_tf(calibration_tf_matrix)
            calibration_tf_msg.header.frame_id = self.frame_robot_base
            calibration_tf_msg.child_frame_id = self.frame_sensor_camera

        self.tf_broadcaster.sendTransform(calibration_tf_msg)

        self.save_samples_to_disk()
        res.message = f"Saved and solved: {len(self.transform_pairs)} samples"
        res.success = True
        return res

    def save_samples_to_disk(self, samples_dir: Path = SAMPLES_DIRECTORY) -> None:
        """Save samples to the disk."""
        samples_dir.mkdir(parents=True, exist_ok=True)  # Ensure samples dir exists
        filepath = samples_dir / "samples.yaml"
        with filepath.open("wb") as f:
            pickle.dump(self.transform_pairs, f)


def main(args: list[str] | None = None) -> None:
    """Start the node."""
    # Initialize ROS and the node
    rclpy.init(args=args)
    node = Calibrator4DofNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Gracefully shut down when a Ctrl+C is pressed
        pass

    finally:
        # Clean up the node
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

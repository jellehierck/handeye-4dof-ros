# ruff: noqa: INP001

from pathlib import Path

from setuptools import find_packages, setup

package_name = "handeye_4dof_ros"
package_base_path = Path(__file__).resolve().parent

setup(
    # Package information.
    name=package_name,
    version="0.0.0",
    description="Handeye calibration for 4DOF manipulators using dual quaternions.",
    license="MIT",
    maintainer="TODO",
    maintainer_email="TODO",
    # List pure Python dependencies (no ROS packages)
    install_requires=[
        "setuptools",
        # TODO: add symver to these packages
        # TODO: Match to requirements.txt
        "numpy",
        "scipy",
        "sympy",
    ],
    # Which Python packages to include in (and exclude from) this ROS package
    packages=[
        # Include the pure Python handeye_4dof source files
        *find_packages(where=str(package_base_path / "src"), include=["handeye_4dof"]),
        # TODO: Include the handeye_4dof_ros source files
        # TODO: Exclude test files
    ],
    # Register the programs to run to be recognized as executables by the ROS 2 command line tools
    entry_points={
        "console_scripts": ["calibrator_4dof = handeye_4dof_ros.calibrator_4dof:main"],
    },
    # Register data files to install the package correctly
    data_files=list(
        {
            # package.xml to let colcon recognize this package as ROS package
            str(Path("share") / package_name): [str(package_base_path / "package.xml")],
            # Marker file to let ROS recognize this Python package
            str(Path("share") / "ament_index" / "resource_index" / "packages"): [
                str(package_base_path / "resource" / package_name)
            ],
            # Launch files
            str(Path("share") / package_name / "launch"): [str(path) for path in Path("launch").glob("**/*.launch.*")],
        }.items()
    ),
    # Test configurations
    tests_require=["pytest"],
)

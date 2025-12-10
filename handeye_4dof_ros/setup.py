# ruff: noqa: INP001

from pathlib import Path

from setuptools import find_packages, setup

package_name = "handeye_4dof_ros"
package_base_path = Path(__file__).resolve().parent
relative_base_path = Path()


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
        "numpy~=1.26",
        "scipy~=1.11",
        "sympy~=1.12",
    ],
    # Which Python packages to include in the ROS package
    packages=[
        "handeye_4dof_ros",  # ROS wrapper package
        "handeye_4dof",  # Original handeye_4dof package (pure python)
    ],
    # Point setuptools to the paths where it can find the packages' source files, relative to this setup.py script
    package_dir={
        "handeye_4dof": "../src/handeye_4dof",
        "handeye_4dof_ros": "./handeye_4dof_ros",
    },
    # Register the programs to run to be recognized as executables by the ROS 2 command line tools
    entry_points={
        "console_scripts": [
            "example = handeye_4dof_ros.example:main",
            "calibrator = handeye_4dof_ros.calibrator:main",
        ],
    },
    # Register data files to install the package correctly (these paths must be relative)
    data_files=list(
        {
            # package.xml to let colcon recognize this package as ROS package
            str(Path("share") / package_name): [str(Path() / "package.xml")],
            # Marker file to let ROS recognize this Python package
            str(Path("share") / "ament_index" / "resource_index" / "packages"): [
                str(Path() / "resource" / package_name)
            ],
            # Find all launch files inside the launch/ directory
            str(Path("share") / package_name / "launch"): [str(path) for path in Path("launch").glob("**/*.launch.*")],
            # Find all rviz configuration files inside the rviz/ directory
            str(Path("share") / package_name / "rviz"): [str(path) for path in Path("rviz").glob("**/*.rviz*")],
            # Add the example data files to the ROS share
            str(Path("share") / package_name / "example_data"): [
                str(path) for path in Path("../example_data").glob("*.pkl")
            ],
        }.items()
    ),
    # Test configurations
    tests_require=["pytest"],
)

from glob import glob
import os

from setuptools import find_packages, setup


package_name = "wuji_hand_debug"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Local Wuji Hand Debug Environment",
    maintainer_email="local@example.invalid",
    description="Safety-gated ROS 2 monitoring and control GUI for one Wuji Hand.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "wuji_hand_control_gui = wuji_hand_debug.control_gui:main",
        ],
    },
)

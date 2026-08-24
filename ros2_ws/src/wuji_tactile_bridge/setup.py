from glob import glob
from setuptools import find_packages, setup


package_name = "wuji_tactile_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config/poses", glob("config/poses/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="local Wuji workspace",
    maintainer_email="oem@localhost",
    description="Read-only paired tactile plus integrated, safety-gated Wuji Hand control.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "wuji_tactile_bridge = wuji_tactile_bridge.tactile_bridge:main",
            "wuji_tactile_heatmap = wuji_tactile_bridge.tactile_heatmap:main",
            "wuji_live_hand_control = wuji_tactile_bridge.live_hand_control:main",
        ],
    },
)

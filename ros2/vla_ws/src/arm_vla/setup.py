import os
from glob import glob
from setuptools import setup

package_name = "arm_vla"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Samer",
    maintainer_email="samerhaddadin06@gmail.com",
    description="3-DOF vision-language-action pipeline.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vision_spatial_node = arm_vla.vision_spatial_node:main",
            "vla_commander_node  = arm_vla.vla_commander_node:main",
            "calibrate_extrinsics = arm_vla.calibrate_extrinsics:main",
        ],
    },
)

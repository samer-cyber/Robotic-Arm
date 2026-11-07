from setuptools import setup

package_name = "armlink_py"

setup(
    name=package_name,
    version="0.1.0",
    # armlink is vendored in by sync.sh from host/armlink so that the ROS
    # workspace is self-contained inside the WSL filesystem. Building against
    # /mnt/d works but is slow and permission-fragile.
    packages=["armlink"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Samer",
    maintainer_email="samerhaddadin06@gmail.com",
    description="Host driver for the arm controller, packaged for ROS 2.",
    license="MIT",
)

from glob import glob
import os

from setuptools import find_packages, setup

package_name = "lego_spike_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch")),
        ),
        (
            os.path.join("share", "package_name", "mindstorms"),
            glob(os.path.join("mindstorms", "*")),
        ),
        # (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Chris Iverach-Brereton",
    maintainer_email="civerachb@clearpathrobotics.com",
    description="Serial-based interface for communicating with the Lego Mindstorms RI + Lego Spike Prime hub",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "serial_interface = lego_spike_interface:main",
        ],
    },
)

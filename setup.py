import os
from glob import glob

from setuptools import setup

package_name = "BinPickingCellControl"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    package_dir={"": "include"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Anas Shahzad",
    maintainer_email="anasshahzad1996@gmail.com",
    description="A ROS 2 package with bin-picking logic",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "barcode_scanner = BinPickingCellControl.barcode_scanner:main",
            "door_handle = BinPickingCellControl.door_handle:main",
            "emergency_button = BinPickingCellControl.emergency_button:main",
        ],
    },
)

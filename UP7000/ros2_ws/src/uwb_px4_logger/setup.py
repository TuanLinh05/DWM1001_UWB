from glob import glob

from setuptools import setup

package_name = "uwb_px4_logger"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, package_name + ".vendor"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vu Tuan Linh",
    maintainer_email="vutlinh.work@gmail.com",
    description="UP4000 onboard logger: UWB TAG relay over WiFi and PX4 MAVLink/ULog logging",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "uwb_relay = uwb_px4_logger.uwb_relay_node:main",
            "px4_logger = uwb_px4_logger.px4_logger_node:main",
            "up4000_status = uwb_px4_logger.up4000_status_node:main",
        ],
    },
)

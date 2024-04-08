from setuptools import find_packages, setup
import os
from glob import glob

package_name = "robot_description_echo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("scripts/*.*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xy",
    maintainer_email="1023515576@qq.com",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["robot_description_echo = robot_description_echo.robot_description_echo:main"],
    },
)

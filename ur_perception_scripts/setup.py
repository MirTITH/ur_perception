from setuptools import find_packages, setup

package_name = "ur_perception_scripts"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yang Xie",
    maintainer_email="1023515576@qq.com",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_description_echo = ur_perception_scripts.robot_description_echo:main",
        ],
    },
)

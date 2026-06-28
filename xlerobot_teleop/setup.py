from setuptools import find_packages, setup


package_name = "xlerobot_teleop"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="XLeRobot maintainers",
    maintainer_email="raibek885@users.noreply.github.com",
    description="Gamepad and leader-follower teleoperation nodes for XLeRobot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joy_teleop = xlerobot_teleop.joy_teleop_node:main",
            "leader_follower = xlerobot_teleop.leader_follower_node:main",
            "trajectory_macro = xlerobot_teleop.trajectory_macro_node:main",
        ],
    },
)

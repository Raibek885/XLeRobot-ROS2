from setuptools import find_packages, setup


package_name = "xlerobot_driver"


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
    description="ROS 2 bridge for the XLeRobot LeRobot hardware API.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "xlerobot_driver = xlerobot_driver.driver_node:main",
            "xlerobot_calibrate = xlerobot_driver.calibrate:main",
        ],
    },
)

from setuptools import setup

package_name = "motor"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ductrung1802",
    maintainer_email="trung.lyduc18@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drive_motor = motor.drive_motor:main",
            "teleop_key = motor.teleop_key:main",
        ]
    },
)

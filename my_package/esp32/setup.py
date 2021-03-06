from setuptools import setup

package_name = "esp32"

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
            "esp32 = esp32.esp32_node:main",
            "motor_controller = esp32.motor_controller_node:main",
        ]
    },
)

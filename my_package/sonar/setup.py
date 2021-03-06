from setuptools import setup

package_name = "sonar"

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
            "sonar = sonar.SonarNode:main",
            "sonar1 = sonar.sonar1:main",
            "sonar2 = sonar.sonar2:main",
            "sonar3 = sonar.sonar3:main",
            "sonar4 = sonar.sonar4:main",
            "sonar5 = sonar.sonar5:main",
        ]
    },
)

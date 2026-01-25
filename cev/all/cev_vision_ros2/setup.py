from setuptools import find_packages, setup

package_name = "vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/config", ["config/occupancy.yaml"]),
        (f"share/{package_name}/launch", ["launch/launch.py", "launch/bag.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Utku Melemetci",
    maintainer_email="55263178+Yey007@users.noreply.github.com",
    description="Vision code for mini-cars.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "occupancy_transformer = vision.occupancy_transformer:main"
        ],
    },
)

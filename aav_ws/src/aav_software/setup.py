from setuptools import find_packages, setup

package_name = "aav_software"

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
    maintainer="carter",
    maintainer_email="carterhawkins@vt.edu",
    description="Software for AAV Drone",
    license="GPL-3.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "manavs_magic_code = aav_software.manavs_magic_code:main",
            "location_logger = aav_software.location_logger:main",
            "topic_converter_for_drone = aav_software.topic_converter_for_drone:main",
            "topic_converter_for_simulation = aav_software.topic_converter_for_simulation:main",
            "object_alignment_controller = aav_software.object_alignment_controller:main",
        ],
    },
)

from glob import glob
from setuptools import setup

setup(
    name="bit_standalone",
    version="0.1.0",
    packages=["bit_standalone"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/bit_standalone"]),
        ("share/bit_standalone", ["package.xml"]),
        ("share/bit_standalone/launch", glob("launch/*.launch.py")),
        ("share/bit_standalone/config", glob("config/*.yaml") + glob("config/*.json") + glob("config/*.rviz")),
        ("share/bit_standalone/behavior_tree", glob("behavior_tree/*.xml")),
        ("share/bit_standalone/map", glob("map/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="standalone maintainers",
    maintainer_email="maintainer@example.com",
    description="Independent single MID360 perception bringup.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "base_odometry = bit_standalone.base_odometry:main",
        "motion_preview = bit_standalone.motion_preview:main",
        "can_preview = bit_standalone.can_preview:main",
        "can_bridge = bit_standalone.can_bridge:main",
    ]},
)

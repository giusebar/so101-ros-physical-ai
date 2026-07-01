from glob import glob

from setuptools import find_packages, setup

package_name = "so101_depth_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Giuseppe Barbieri",
    maintainer_email="giuseppe.barbieri@canonical.com",
    description="CPU Depth Anything V2 (Small) depth visualization demo for SO-101 (ONNX Runtime, no NVIDIA required)",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "depth_anything_node = so101_depth_demo.depth_anything_node:main",
            "depth_display_node = so101_depth_demo.depth_display_node:main",
            "depth_proximity_node = so101_depth_demo.depth_proximity_node:main",
            "test_image_publisher = so101_depth_demo.test_image_publisher:main",
        ],
    },
)

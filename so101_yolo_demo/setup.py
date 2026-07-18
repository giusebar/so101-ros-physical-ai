from glob import glob

from setuptools import find_packages, setup

package_name = "so101_yolo_demo"

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
    description="CPU YOLOv8n object/person detection demo for SO-101 (ONNX Runtime, no NVIDIA required)",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "yolo_detect_node = so101_yolo_demo.yolo_detect_node:main",
            "test_image_publisher = so101_yolo_demo.test_image_publisher:main",
        ],
    },
)

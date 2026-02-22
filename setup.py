from setuptools import setup

package_name = "ros2_rtsp"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/rtsp_multi.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nils",
    maintainer_email="nils@example.com",
    description="Multi-camera RTSP capture node for ROS 2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rtsp_multi = ros2_rtsp.rtsp_multi_node:main",
            "audio_monitor = ros2_rtsp.audio_monitor_node:main",
        ],
    },
)

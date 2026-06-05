from setuptools import find_packages, setup

package_name = "r2d3_humble_bridge"

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
    maintainer="Sam Mathew",
    maintainer_email="semathew@andrew.cmu.edu",
    description="rm_ros_interfaces <-> std_msgs bridge sitting between "
                "r2d3_model (Py 3.11) and the Isaac sim_adapter (Py 3.12).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "r2d3_humble_bridge = r2d3_humble_bridge.bridge:main",
        ],
    },
)

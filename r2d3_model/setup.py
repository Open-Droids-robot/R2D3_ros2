from setuptools import find_packages, setup

package_name = "r2d3_model"

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
    description="R2D3 V1 Isaac Sim participant model template (Lifecycle Node).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "r2d3_model = r2d3_model.r2d3_model:main",
        ],
    },
)

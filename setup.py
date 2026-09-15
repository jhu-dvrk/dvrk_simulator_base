from pathlib import Path

from setuptools import find_packages, setup


package_name = "dvrk_simulator_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/share/arms", [
            str(path) for path in sorted(Path("share/arms").glob("*.yaml"))
        ]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="dVRK maintainers",
    maintainer_email="support@intusurg.com",
    description="Simulator-independent CRTK behavior and contracts for dVRK simulators.",
    license="Apache-2.0",
)

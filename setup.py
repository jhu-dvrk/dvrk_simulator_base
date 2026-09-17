from pathlib import Path

from setuptools import find_packages, setup


package_name = "dvrk_simulator_base"
asset_root = Path("share/assets")


def asset_data_files():
    """Install shared assets while preserving their relative directories."""
    files = []
    for path in sorted(asset_root.rglob("*")):
        if not path.is_file():
            continue
        relative_parent = path.parent.relative_to(asset_root)
        destination = f"share/{package_name}/share/assets"
        if relative_parent != Path("."):
            destination += f"/{relative_parent.as_posix()}"
        files.append((destination, [str(path)]))
    return files

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
    ] + asset_data_files(),
    install_requires=["setuptools", "numpy", "PyYAML"],
    entry_points={
        "console_scripts": [
            "generate_cart_frames = dvrk_simulator_base.cart_frames:main",
            "cart_frame_editor = dvrk_simulator_base.cart_frame_editor:main",
        ],
    },
    zip_safe=True,
    maintainer="dVRK maintainers",
    maintainer_email="support@intusurg.com",
    description="Simulator-independent CRTK behavior and contracts for dVRK simulators.",
    license="Apache-2.0",
)

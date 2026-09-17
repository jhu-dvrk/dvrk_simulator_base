#!/usr/bin/env python3
"""Generate URDF-ready visual and convex collision meshes from triangular_block.scad."""

from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parent
MESHES = ROOT / "meshes"


def stl_to_obj(source: Path, destination: Path) -> None:
    """Convert an OpenSCAD ASCII STL to a compact Wavefront OBJ."""
    vertices: list[tuple[float, float, float]] = []
    indices: dict[tuple[float, float, float], int] = {}
    faces: list[tuple[int, int, int]] = []
    facet: list[int] = []

    for line in source.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if not fields or fields[0] != "vertex":
            continue
        vertex = tuple(float(value) for value in fields[1:4])
        if vertex not in indices:
            indices[vertex] = len(vertices) + 1
            vertices.append(vertex)
        facet.append(indices[vertex])
        if len(facet) == 3:
            faces.append(tuple(facet))
            facet = []

    if not faces:
        raise RuntimeError(f"OpenSCAD produced no facets in {source}")

    lines = [
        "# Generated from ../triangular_block.scad; do not edit by hand.",
        "o triangular_block",
    ]
    lines.extend(f"v {x:.9g} {y:.9g} {z:.9g}" for x, y, z in vertices)
    lines.extend(f"f {a} {b} {c}" for a, b, c in faces)
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    try:
        import pybullet
    except ImportError as error:
        raise SystemExit("pybullet is required to generate the collision mesh") from error

    MESHES.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="dvrk-triangular-block-") as temp_name:
        temp = Path(temp_name)
        stl = temp / "triangular_block.stl"
        log = temp / "vhacd.log"
        subprocess.run(
            [
                "openscad",
                "--export-format",
                "asciistl",
                "-o",
                str(stl),
                str(ROOT / "triangular_block.scad"),
            ],
            check=True,
        )
        stl_to_obj(stl, MESHES / "triangular_block.obj")
        pybullet.vhacd(
            str(MESHES / "triangular_block.obj"),
            str(MESHES / "triangular_block_vhacd.obj"),
            str(log),
            resolution=1_000_000,
            concavity=0.0001,
            alpha=0.04,
            beta=0.05,
            gamma=0.001,
            minVolumePerCH=0.000001,
            maxNumVerticesPerCH=64,
        )


if __name__ == "__main__":
    main()

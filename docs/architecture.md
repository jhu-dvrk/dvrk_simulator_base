# Architecture decisions

## Backend-owned URDF parsing

`dvrk_simulator_base` does not parse URDF or create a URDF semantic manifest.
Each derived simulator owns the parser or importer needed by its engine and
must validate mappings by configured joint and link name rather than index.

The base package owns the configured semantic names, ordered CRTK joints,
limits, home state, and backend protocols. This keeps XML, mimic-joint, mesh,
and engine-import details outside the shared command and ROS layers.

Model discovery and Xacro expansion are not decided here. They may become
shared source-preparation utilities later without making the base responsible
for interpreting URDF structure.

## Temporary source duplication

`dvrk_isaac_sim` remains unchanged until `dvrk_pybullet` is operational enough
to validate the shared abstractions. Common modules are temporarily copied and
tested in `dvrk_simulator_base`; Isaac migration and duplicate removal happen
after that milestone.

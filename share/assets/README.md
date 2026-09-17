# Shared simulator assets

These simple URDF primitives are simulator-neutral scene assets maintained by
`dvrk_simulator_base`.  They have explicit metric dimensions, collision
geometry, and inertial properties.  The base package installs them but does
not parse or load URDFs; each simulator backend resolves and loads the assets.

All assets are Apache-2.0.

- `table/table.urdf`: 600 x 400 mm work surface.
- `tray/tray.urdf`: 220 x 160 mm open tray.
- `block/block.urdf`: 24 x 16 x 10 mm graspable training block.
- `cube/cube.urdf`: 20 mm graspable calibration cube.

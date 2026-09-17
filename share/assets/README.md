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
- `peg_board/peg_board.urdf`: 150 x 110 x 5 mm plate with 48 vertical pegs (20 mm tall, 4 mm diameter on a 20 x 20 mm grid with 5 mm border margin).
- `ring/ring.urdf`: Torus ring (15 mm outer diameter, 3 mm section diameter, 9 mm inner hole) with VHACD convex collision mesh.

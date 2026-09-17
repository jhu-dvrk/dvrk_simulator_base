// Parametric hollow triangular training block.
// Units are metres because this geometry is consumed directly by URDF.
$fn = 48;

outer_radius = 0.010;
hole_radius = 0.0048;
depth = 0.0145;

difference() {
    linear_extrude(height = depth, center = true, convexity = 4)
        polygon(points = [
            [outer_radius, 0],
            [-outer_radius / 2, sqrt(3) * outer_radius / 2],
            [-outer_radius / 2, -sqrt(3) * outer_radius / 2]
        ]);

    // Extend the cutter beyond both faces to avoid coincident CSG surfaces.
    cylinder(h = depth + 0.002, r = hole_radius, center = true);
}

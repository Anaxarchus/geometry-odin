package tests

import "core:math"
import "core:testing"
import "../sphere"

@(test)
test_sphere_measures :: proc(t: ^testing.T) {
	testing.expect(t, feq(sphere.volume(f32(2)), 4.0 / 3.0 * math.PI * 8), "volume")
	testing.expect(t, feq(sphere.surface_area(f32(2)), 4 * math.PI * 4), "surface_area")
}

@(test)
test_sphere_point_queries :: proc(t: ^testing.T) {
	o := [3]f32{0, 0, 0}

	testing.expect(
		t,
		veq(sphere.project_to_boundary([3]f32{4, 0, 0}, o, 2), [3]f32{2, 0, 0}),
		"project outside",
	)
	testing.expect(
		t,
		veq(sphere.project_to_boundary(o, o, 2), [3]f32{2, 0, 0}),
		"project center",
	)

	testing.expect(
		t,
		veq(sphere.clamp_point([3]f32{4, 0, 0}, o, 2), [3]f32{2, 0, 0}),
		"clamp out",
	)
	testing.expect(
		t,
		veq(sphere.clamp_point([3]f32{1, 0, 0}, o, 2), [3]f32{1, 0, 0}),
		"clamp in",
	)

	testing.expect(t, sphere.has_point([3]f32{1, 0, 0}, o, 2), "has_point inside")
	testing.expect(t, !sphere.has_point([3]f32{3, 0, 0}, o, 2), "has_point outside")
}

@(test)
test_sphere_parametric :: proc(t: ^testing.T) {
	o := [3]f32{0, 0, 0}

	// equator (phi = PI/2) reduces to the circle in the xy-plane
	testing.expect(
		t,
		veq(sphere.point_at(o, 2, 0, math.PI / 2), [3]f32{2, 0, 0}),
		"point_at equator",
	)
	// north pole (phi = 0) is +z
	testing.expect(
		t,
		veq(sphere.point_at(o, 2, 0, 0), [3]f32{0, 0, 2}),
		"point_at pole",
	)

	testing.expect(
		t,
		veq(sphere.normal_at(f32(0), f32(math.PI / 2)), [3]f32{1, 0, 0}),
		"normal_at equator",
	)

	theta, phi := sphere.angles_of([3]f32{5, 0, 0}, o)
	testing.expect(t, feq(theta, 0) && feq(phi, math.PI / 2), "angles_of +x")

	theta2, phi2 := sphere.angles_of([3]f32{0, 0, 5}, o)
	testing.expect(t, feq(theta2, 0) && feq(phi2, 0), "angles_of +z")
}

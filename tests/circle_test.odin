package tests

import "core:math"
import "core:testing"
import "../circle"

@(test)
test_circle_measures :: proc(t: ^testing.T) {
	testing.expect(t, feq(circle.circle_area(f32(2)), math.PI * 4), "area")
	testing.expect(t, feq(circle.circle_circumference(f32(2)), math.TAU * 2), "circumference")
}

@(test)
test_circle_point_queries :: proc(t: ^testing.T) {
	o := [2]f32{0, 0}

	testing.expect(
		t,
		veq(circle.circle_project_to_boundary([2]f32{4, 0}, o, 2), [2]f32{2, 0}),
		"project outside",
	)
	// center is degenerate: falls back to +x
	testing.expect(
		t,
		veq(circle.circle_project_to_boundary(o, o, 2), [2]f32{2, 0}),
		"project center",
	)

	testing.expect(
		t,
		veq(circle.circle_clamp_point([2]f32{4, 0}, o, 2), [2]f32{2, 0}),
		"clamp out",
	)
	testing.expect(
		t,
		veq(circle.circle_clamp_point([2]f32{1, 0}, o, 2), [2]f32{1, 0}),
		"clamp in",
	)

	testing.expect(t, circle.circle_has_point([2]f32{1, 0}, o, 2), "has_point inside")
	testing.expect(t, circle.circle_has_point([2]f32{2, 0}, o, 2), "has_point boundary")
	testing.expect(t, !circle.circle_has_point([2]f32{3, 0}, o, 2), "has_point outside")
}

@(test)
test_circle_parametric :: proc(t: ^testing.T) {
	o := [2]f32{0, 0}
	testing.expect(t, veq(circle.circle_point_at(o, 2, 0), [2]f32{2, 0}), "point_at 0")
	testing.expect(
		t,
		veq(circle.circle_point_at(o, 2, math.PI / 2), [2]f32{0, 2}),
		"point_at 90",
	)
	testing.expect(t, veq(circle.circle_tangent_at(f32(0)), [2]f32{0, 1}), "tangent_at 0")
	testing.expect(t, veq(circle.circle_normal_at(f32(0)), [2]f32{1, 0}), "normal_at 0")
	testing.expect(t, feq(circle.circle_angle_of([2]f32{0, 5}, o), math.PI / 2), "angle_of")
}

@(test)
test_circle_from_contour :: proc(t: ^testing.T) {
	// diamond: distances to centroid are 2, 2, 1, 1
	contour := [][2]f32{{-2, 0}, {2, 0}, {0, 1}, {0, -1}}
	o_min, r_min := circle.circle_from_contour(contour, .Min)
	testing.expect(t, veq(o_min, [2]f32{0, 0}), "centroid")
	testing.expect(t, feq(r_min, 1), "fit Min")

	_, r_max := circle.circle_from_contour(contour, .Max)
	testing.expect(t, feq(r_max, 2), "fit Max")

	_, r_avg := circle.circle_from_contour(contour, .Average)
	testing.expect(t, feq(r_avg, 1.5), "fit Average")
}

@(test)
test_circle_to_contour :: proc(t: ^testing.T) {
	pts := circle.circle_to_contour([2]f32{0, 0}, 1, 4)
	defer delete(pts)
	testing.expect(t, len(pts) == 4, "segment count")
	testing.expect(t, veq(pts[0], [2]f32{1, 0}), "vertex 0")
	testing.expect(t, veq(pts[1], [2]f32{0, 1}), "vertex 1")
	testing.expect(t, veq(pts[2], [2]f32{-1, 0}), "vertex 2")
	testing.expect(t, veq(pts[3], [2]f32{0, -1}), "vertex 3")
}

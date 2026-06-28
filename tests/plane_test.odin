package tests

import "core:testing"
import "../plane"

@(test)
test_plane_from_points :: proc(t: ^testing.T) {
	// xy-plane through origin, normal +z
	p := plane.from_points([3]f32{0, 0, 0}, [3]f32{1, 0, 0}, [3]f32{0, 1, 0})
	testing.expect(t, veq(plane.normal(p), [3]f32{0, 0, 1}), "normal")
	testing.expect(t, feq(p.w, 0), "offset")

	// xy-plane raised to z = 2
	p2 := plane.from_points([3]f32{0, 0, 2}, [3]f32{1, 0, 2}, [3]f32{0, 1, 2})
	testing.expect(t, veq(plane.normal(p2), [3]f32{0, 0, 1}), "normal raised")
	testing.expect(t, feq(p2.w, 2), "offset raised")
}

@(test)
test_plane_distance_project :: proc(t: ^testing.T) {
	p := [4]f32{0, 0, 1, 0} // xy-plane, normal +z

	testing.expect(t, feq(plane.distance(p, [3]f32{0, 0, 5}), 5), "distance above")
	testing.expect(t, feq(plane.distance(p, [3]f32{0, 0, -3}), -3), "distance below")
	testing.expect(t, feq(plane.distance(p, [3]f32{5, 5, 0}), 0), "distance on")

	testing.expect(
		t,
		veq(plane.project(p, [3]f32{3, 4, 5}), [3]f32{3, 4, 0}),
		"project",
	)

	p2 := [4]f32{0, 0, 1, 2} // z = 2
	testing.expect(t, feq(plane.distance(p2, [3]f32{0, 0, 5}), 3), "distance offset")
	testing.expect(
		t,
		veq(plane.project(p2, [3]f32{1, 1, 5}), [3]f32{1, 1, 2}),
		"project offset",
	)
}

@(test)
test_plane_predicates :: proc(t: ^testing.T) {
	p := [4]f32{0, 0, 1, 0}
	testing.expect(t, plane.is_above(p, [3]f32{0, 0, 1}), "above true")
	testing.expect(t, !plane.is_above(p, [3]f32{0, 0, -1}), "above false")
	testing.expect(t, !plane.is_above(p, [3]f32{0, 0, 0}), "above on-plane false")

	testing.expect(t, plane.is_equal_approx(p, [4]f32{0, 0, 1, 0}), "equal true")
	testing.expect(t, !plane.is_equal_approx(p, [4]f32{0, 1, 0, 0}), "equal false")
}

@(test)
test_plane_f64 :: proc(t: ^testing.T) {
	p := plane.from_points([3]f64{0, 0, 0}, [3]f64{1, 0, 0}, [3]f64{0, 1, 0})
	testing.expect(t, veq(plane.normal(p), [3]f64{0, 0, 1}), "f64 normal")
	testing.expect(t, feq(plane.distance(p, [3]f64{0, 0, 7}), 7.0), "f64 distance")
}

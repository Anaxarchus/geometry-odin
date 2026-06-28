package tests

import "core:math"
import "core:math/linalg"
import "core:testing"
import "../line"

// ---------------------------------------------------------------------------
// Casting & direction
// ---------------------------------------------------------------------------

@(test)
test_line_cast_to :: proc(t: ^testing.T) {
	li := [2][2]int{{1, 2}, {3, 4}}
	lf := line.cast_to(li, f32)
	testing.expect(t, veq(lf[0], [2]f32{1, 2}), "cast endpoint 0")
	testing.expect(t, veq(lf[1], [2]f32{3, 4}), "cast endpoint 1")
}

@(test)
test_line_direction :: proc(t: ^testing.T) {
	// raw (non-normalized) direction, all scalars
	lf := [2][2]f32{{1, 2}, {4, 6}}
	testing.expect(t, veq(line.direction(lf), [2]f32{3, 4}), "direction float")

	li := [2][2]int{{1, 2}, {4, 6}}
	testing.expect(t, line.direction(li) == [2]int{3, 4}, "direction int exact")
}

@(test)
test_line_direction_normal :: proc(t: ^testing.T) {
	// 3-4-5 → unit direction (0.6, 0.8)
	lf := [2][2]f32{{0, 0}, {3, 4}}
	testing.expect(t, veq(line.direction_normal(lf), [2]f32{0.6, 0.8}), "dir normal float")

	li := [2][2]int{{0, 0}, {3, 4}}
	testing.expect(t, veq(line.direction_normal(li), [2]f64{0.6, 0.8}), "dir normal int")

	// degenerate segment normalizes to zero (normalize0)
	deg := [2][2]f32{{5, 5}, {5, 5}}
	testing.expect(t, veq(line.direction_normal(deg), [2]f32{0, 0}), "dir normal degenerate")
}

@(test)
test_line_normal :: proc(t: ^testing.T) {
	// normal must be unit length and perpendicular to the direction (sign-agnostic)
	lf := [2][2]f32{{0, 0}, {3, 4}}
	n := line.normal(lf)
	d := line.direction_normal(lf)
	testing.expect(t, feq(linalg.length(n), 1), "normal is unit length")
	testing.expect(t, feq(linalg.dot(n, d), 0), "normal perpendicular to direction")

	li := [2][2]int{{0, 0}, {3, 4}}
	ni := line.normal(li)
	di := line.direction_normal(li)
	testing.expect(t, feq(linalg.length(ni), 1), "normal int unit length")
	testing.expect(t, feq(linalg.dot(ni, di), 0), "normal int perpendicular")
}

// ---------------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------------

@(test)
test_line_center :: proc(t: ^testing.T) {
	lf := [2][2]f32{{0, 0}, {4, 6}}
	testing.expect(t, veq(line.center(lf), [2]f32{2, 3}), "center float")

	li := [2][2]int{{0, 0}, {4, 6}}
	testing.expect(t, line.center(li) == [2]int{2, 3}, "center int exact")

	// 3D
	l3 := [2][3]f32{{0, 0, 0}, {2, 4, 8}}
	testing.expect(t, veq(line.center(l3), [3]f32{1, 2, 4}), "center 3d")
}

@(test)
test_line_point_at :: proc(t: ^testing.T) {
	lf := [2][2]f32{{0, 0}, {10, 0}}
	testing.expect(t, veq(line.point_at(lf, 0), [2]f32{0, 0}), "point_at t=0")
	testing.expect(t, veq(line.point_at(lf, 1), [2]f32{10, 0}), "point_at t=1")
	testing.expect(t, veq(line.point_at(lf, 0.5), [2]f32{5, 0}), "point_at t=0.5")
	testing.expect(t, veq(line.point_at(lf, 0.25), [2]f32{2.5, 0}), "point_at t=0.25")
	// extrapolation beyond the segment
	testing.expect(t, veq(line.point_at(lf, 1.5), [2]f32{15, 0}), "point_at t=1.5")
	testing.expect(t, veq(line.point_at(lf, -0.5), [2]f32{-5, 0}), "point_at t=-0.5")
}

// ---------------------------------------------------------------------------
// Projection
// ---------------------------------------------------------------------------

@(test)
test_line_project_parameter :: proc(t: ^testing.T) {
	lf := [2][2]f32{{0, 0}, {10, 0}}
	testing.expect(t, feq(line.project_parameter(lf, [2]f32{3, 5}), 0.3), "param interior")
	testing.expect(t, feq(line.project_parameter(lf, [2]f32{-2, 1}), -0.2), "param before")
	testing.expect(t, feq(line.project_parameter(lf, [2]f32{12, 9}), 1.2), "param after")

	// degenerate segment → 0 (guarded divide-by-zero)
	deg := [2][2]f32{{5, 5}, {5, 5}}
	testing.expect(t, feq(line.project_parameter(deg, [2]f32{9, 9}), 0), "param degenerate")
}

@(test)
test_line_project_infinite :: proc(t: ^testing.T) {
	lf := [2][2]f32{{0, 0}, {10, 0}}
	// projects onto the infinite line, so it may land outside the segment
	testing.expect(t, veq(line.project_infinite(lf, [2]f32{3, 5}), [2]f32{3, 0}), "interior")
	testing.expect(t, veq(line.project_infinite(lf, [2]f32{-4, 2}), [2]f32{-4, 0}), "before")
	testing.expect(t, veq(line.project_infinite(lf, [2]f32{14, 7}), [2]f32{14, 0}), "after")

	li := [2][2]int{{0, 0}, {10, 0}}
	testing.expect(t, veq(line.project_infinite(li, [2]int{3, 5}), [2]f64{3, 0}), "int interior")
}

@(test)
test_line_project :: proc(t: ^testing.T) {
	lf := [2][2]f32{{0, 0}, {10, 0}}
	// clamped to the segment endpoints
	testing.expect(t, veq(line.project(lf, [2]f32{3, 5}), [2]f32{3, 0}), "interior")
	testing.expect(t, veq(line.project(lf, [2]f32{-5, 2}), [2]f32{0, 0}), "clamp to start")
	testing.expect(t, veq(line.project(lf, [2]f32{15, 2}), [2]f32{10, 0}), "clamp to end")

	li := [2][2]int{{0, 0}, {10, 0}}
	testing.expect(t, veq(line.project(li, [2]int{15, 2}), [2]f64{10, 0}), "int clamp to end")
}

// ---------------------------------------------------------------------------
// Length
// ---------------------------------------------------------------------------

@(test)
test_line_length :: proc(t: ^testing.T) {
	lf := [2][2]f32{{0, 0}, {3, 4}}
	testing.expect(t, feq(line.length2(lf), 25), "length2 float")
	testing.expect(t, feq(line.length(lf), 5), "length float")

	li := [2][2]int{{0, 0}, {3, 4}}
	testing.expect(t, feq(line.length2(li), 25.0), "length2 int → f64")
	testing.expect(t, feq(line.length(li), 5.0), "length int → f64")

	// 3D length
	l3 := [2][3]f32{{0, 0, 0}, {2, 3, 6}} // 2-3-6 → 7
	testing.expect(t, feq(line.length(l3), 7), "length 3d")
}

// ---------------------------------------------------------------------------
// Point distance
// ---------------------------------------------------------------------------

@(test)
test_line_distance :: proc(t: ^testing.T) {
	lf := [2][2]f32{{0, 0}, {10, 0}}
	// perpendicular within the span
	testing.expect(t, feq(line.distance2(lf, [2]f32{3, 5}), 25), "distance2 interior")
	testing.expect(t, feq(line.distance(lf, [2]f32{3, 5}), 5), "distance interior")
	// past the end → measured to the endpoint
	testing.expect(t, feq(line.distance(lf, [2]f32{13, 4}), 5), "distance past end")
	testing.expect(t, feq(line.distance(lf, [2]f32{-3, 4}), 5), "distance before start")

	li := [2][2]int{{0, 0}, {10, 0}}
	testing.expect(t, feq(line.distance(li, [2]int{3, 5}), 5.0), "distance int → f64")
}

@(test)
test_line_distance_infinite :: proc(t: ^testing.T) {
	lf := [2][2]f32{{0, 0}, {10, 0}}
	// perpendicular distance to the infinite x-axis, ignoring the endpoints
	testing.expect(t, feq(line.distance2_infinite(lf, [2]f32{15, 4}), 16), "dist2 inf past end")
	testing.expect(t, feq(line.distance_infinite(lf, [2]f32{15, 4}), 4), "dist inf past end")
	testing.expect(t, feq(line.distance_infinite(lf, [2]f32{-7, 3}), 3), "dist inf before start")

	li := [2][2]int{{0, 0}, {10, 0}}
	testing.expect(t, feq(line.distance_infinite(li, [2]int{15, 4}), 4.0), "dist inf int → f64")
}

// ---------------------------------------------------------------------------
// Bounds
// ---------------------------------------------------------------------------

@(test)
test_line_bounds :: proc(t: ^testing.T) {
	li := [2][2]int{{4, 6}, {1, 2}}
	b := line.bounds(li)
	testing.expect(t, b[0] == [2]int{1, 2}, "bounds min")
	testing.expect(t, b[1] == [2]int{4, 6}, "bounds max")

	// mixed signs and 3D
	l3 := [2][3]f32{{3, -1, 0}, {-2, 5, -4}}
	b3 := line.bounds(l3)
	testing.expect(t, veq(b3[0], [3]f32{-2, -1, -4}), "bounds 3d min")
	testing.expect(t, veq(b3[1], [3]f32{3, 5, 0}), "bounds 3d max")
}

// ---------------------------------------------------------------------------
// has_point
// ---------------------------------------------------------------------------

@(test)
test_line_has_point :: proc(t: ^testing.T) {
	lf := [2][2]f32{{0, 0}, {10, 0}}
	testing.expect(t, line.has_point(lf, [2]f32{5, 0}), "on segment")
	testing.expect(t, line.has_point(lf, [2]f32{0, 0}), "on endpoint")
	testing.expect(t, !line.has_point(lf, [2]f32{5, 0.5}), "off segment")
	testing.expect(t, !line.has_point(lf, [2]f32{15, 0}), "past end, on the line")
	testing.expect(t, !line.has_point(lf, [2]f32{-2, 0}), "before start, on the line")

	// custom epsilon admits a near point that the default rejects
	near := [2]f32{5, 0.0005}
	testing.expect(t, !line.has_point(lf, near), "near point rejected by default eps")
	testing.expect(t, line.has_point(lf, near, 0.001), "near point accepted by custom eps")

	li := [2][2]int{{0, 0}, {10, 0}}
	testing.expect(t, line.has_point(li, [2]int{5, 0}), "int on segment")
	testing.expect(t, !line.has_point(li, [2]int{5, 2}), "int off segment")
}

// ---------------------------------------------------------------------------
// side & angle (2D)
// ---------------------------------------------------------------------------

@(test)
test_line_side :: proc(t: ^testing.T) {
	lf := [2][2]f32{{0, 0}, {10, 0}} // pointing +x
	testing.expect(t, line.side(lf, [2]f32{5, 3}) > 0, "left is positive")
	testing.expect(t, line.side(lf, [2]f32{5, -3}) < 0, "right is negative")
	testing.expect(t, feq(line.side(lf, [2]f32{5, 0}), 0), "on line is zero")
	testing.expect(t, feq(line.side(lf, [2]f32{5, 3}), 30), "exact cross value")

	li := [2][2]int{{0, 0}, {10, 0}}
	testing.expect(t, feq(line.side(li, [2]int{5, 3}), 30.0), "int side → f64")
}

@(test)
test_line_angle :: proc(t: ^testing.T) {
	testing.expect(t, feq(line.angle([2][2]f32{{0, 0}, {1, 0}}), 0), "angle +x")
	testing.expect(t, feq(line.angle([2][2]f32{{0, 0}, {1, 1}}), math.PI / 4), "angle 45deg")
	testing.expect(t, feq(line.angle([2][2]f32{{0, 0}, {0, 1}}), math.PI / 2), "angle +y")
	testing.expect(t, feq(line.angle([2][2]f32{{0, 0}, {-1, 0}}), math.PI), "angle -x")

	li := [2][2]int{{0, 0}, {0, 1}}
	testing.expect(t, feq(line.angle(li), math.PI / 2), "int angle → f64")
}

// ---------------------------------------------------------------------------
// Equality
// ---------------------------------------------------------------------------

@(test)
test_line_equal_approx :: proc(t: ^testing.T) {
	a := [2][2]f32{{0, 0}, {10, 0}}
	testing.expect(t, line.equal_approx(a, [2][2]f32{{0, 0}, {10, 0}}), "identical")
	testing.expect(t, !line.equal_approx(a, [2][2]f32{{0, 0}, {10, 1}}), "different endpoint")

	// epsilon controls tolerance
	near := [2][2]f32{{0, 0}, {10, 0.0005}}
	testing.expect(t, !line.equal_approx(a, near), "near rejected by default eps")
	testing.expect(t, line.equal_approx(a, near, 0.001), "near accepted by custom eps")

	// order sensitivity
	rev := [2][2]f32{{10, 0}, {0, 0}}
	testing.expect(t, !line.equal_approx(a, rev), "reversed not equal (ordered)")
	testing.expect(t, line.equal_approx_unordered(a, rev), "reversed equal (unordered)")
	testing.expect(t, line.equal_approx_unordered(a, a), "identical (unordered)")

	li := [2][2]int{{0, 0}, {10, 0}}
	testing.expect(t, line.equal_approx(li, [2][2]int{{0, 0}, {10, 0}}), "int identical")
	testing.expect(t, line.equal_approx_unordered(li, [2][2]int{{10, 0}, {0, 0}}), "int unordered")
}

// ---------------------------------------------------------------------------
// Transforms
// ---------------------------------------------------------------------------

@(test)
test_line_reverse :: proc(t: ^testing.T) {
	li := [2][2]int{{1, 2}, {3, 4}}
	r := line.reverse(li)
	testing.expect(t, r[0] == [2]int{3, 4} && r[1] == [2]int{1, 2}, "reverse swaps endpoints")
}

@(test)
test_line_translate :: proc(t: ^testing.T) {
	li := [2][2]int{{0, 0}, {2, 2}}
	r := line.translate(li, [2]int{1, 1})
	testing.expect(t, r[0] == [2]int{1, 1} && r[1] == [2]int{3, 3}, "translate int")

	l3 := [2][3]f32{{0, 0, 0}, {1, 1, 1}}
	r3 := line.translate(l3, [3]f32{2, 0, -1})
	testing.expect(t, veq(r3[0], [3]f32{2, 0, -1}), "translate 3d start")
	testing.expect(t, veq(r3[1], [3]f32{3, 1, 0}), "translate 3d end")
}

@(test)
test_line_scale :: proc(t: ^testing.T) {
	// scale about the center; center stays put, length scales by factor
	lf := [2][2]f32{{0, 0}, {4, 0}}
	r2 := line.scale(lf, 2)
	testing.expect(t, veq(r2[0], [2]f32{-2, 0}), "scale x2 start")
	testing.expect(t, veq(r2[1], [2]f32{6, 0}), "scale x2 end")

	r_half := line.scale(lf, 0.5)
	testing.expect(t, veq(r_half[0], [2]f32{1, 0}), "scale x0.5 start")
	testing.expect(t, veq(r_half[1], [2]f32{3, 0}), "scale x0.5 end")
	testing.expect(t, veq(line.center(r_half), [2]f32{2, 0}), "scale keeps center")
}

@(test)
test_line_set_length :: proc(t: ^testing.T) {
	// anchored at l[0], rescaled along its direction
	lf := [2][2]f32{{0, 0}, {3, 4}} // length 5, dir (0.6, 0.8)
	r := line.set_length(lf, 10, .Begin)
	testing.expect(t, veq(r[0], [2]f32{0, 0}), "set_length keeps start")
	testing.expect(t, veq(r[1], [2]f32{6, 8}), "set_length scales end")
	testing.expect(t, feq(line.length(r), 10), "set_length produces target length")

	li := [2][2]int{{0, 0}, {0, 5}}
	ri := line.set_length(li, 10, .Begin) // → f64
	testing.expect(t, veq(ri[1], [2]f64{0, 10}), "set_length int → f64")
}

@(test)
test_line_rotate :: proc(t: ^testing.T) {
	// 90deg about the center turns a horizontal segment vertical
	lf := [2][2]f32{{0, 0}, {2, 0}}
	r := line.rotate(lf, math.PI / 2)
	testing.expect(t, veq(r[0], [2]f32{1, -1}), "rotate start")
	testing.expect(t, veq(r[1], [2]f32{1, 1}), "rotate end")
	testing.expect(t, feq(line.length(r), 2), "rotate preserves length")
	testing.expect(t, veq(line.center(r), [2]f32{1, 0}), "rotate preserves center")
}

// ---------------------------------------------------------------------------
// Closest approach (segment ↔ segment)
// ---------------------------------------------------------------------------

@(test)
test_line_closest_parallel :: proc(t: ^testing.T) {
	a := [2][2]f32{{0, 0}, {10, 0}}
	b := [2][2]f32{{0, 5}, {10, 5}}
	testing.expect(t, feq(line.closest_distance2(a, b), 25), "parallel dist2")
	testing.expect(t, feq(line.closest_distance(a, b), 5), "parallel dist")
}

@(test)
test_line_closest_crossing :: proc(t: ^testing.T) {
	// two crossing segments in 2D meet at (2,2) → distance 0
	a := [2][2]f32{{0, 0}, {4, 4}}
	b := [2][2]f32{{0, 4}, {4, 0}}
	c1, c2 := line.closest_points(a, b)
	testing.expect(t, veq(c1, [2]f32{2, 2}), "crossing point on a")
	testing.expect(t, veq(c2, [2]f32{2, 2}), "crossing point on b")
	testing.expect(t, feq(line.closest_distance(a, b), 0), "crossing distance 0")
}

@(test)
test_line_closest_skew_3d :: proc(t: ^testing.T) {
	// x-axis segment at z=0 and a y-direction segment at x=0,z=1
	a := [2][3]f32{{0, 0, 0}, {1, 0, 0}}
	b := [2][3]f32{{0, 0, 1}, {0, 1, 1}}
	c1, c2 := line.closest_points(a, b)
	testing.expect(t, veq(c1, [3]f32{0, 0, 0}), "skew closest on a")
	testing.expect(t, veq(c2, [3]f32{0, 0, 1}), "skew closest on b")
	testing.expect(t, feq(line.closest_distance(a, b), 1), "skew distance")
}

@(test)
test_line_closest_degenerate :: proc(t: ^testing.T) {
	// point-to-point and point-to-segment degenerate cases
	pa := [2][2]f32{{2, 2}, {2, 2}}
	pb := [2][2]f32{{5, 6}, {5, 6}}
	testing.expect(t, feq(line.closest_distance(pa, pb), 5), "point-point distance")

	seg := [2][2]f32{{0, 0}, {10, 0}}
	testing.expect(t, feq(line.closest_distance(pa, seg), 2), "point-segment distance")

	li := [2][2]int{{0, 0}, {10, 0}}
	lj := [2][2]int{{0, 5}, {10, 5}}
	testing.expect(t, feq(line.closest_distance(li, lj), 5.0), "int closest → f64")
}

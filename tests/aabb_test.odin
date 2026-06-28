package tests

import "core:testing"
import "../aabb"

@(test)
test_aabb_constructors :: proc(t: ^testing.T) {
	a := aabb.from_center_size([3]f32{1, 2, 3}, [3]f32{2, 4, 6})
	testing.expect(t, veq(a[0], [3]f32{0, 0, 0}), "from_center_size min")
	testing.expect(t, veq(a[1], [3]f32{2, 4, 6}), "from_center_size max")

	pts := [][3]f32{{1, 1, 1}, {-2, 3, 0}, {4, -1, 5}}
	b := aabb.from_points(pts)
	testing.expect(t, veq(b[0], [3]f32{-2, -1, 0}), "from_points min")
	testing.expect(t, veq(b[1], [3]f32{4, 3, 5}), "from_points max")
}

@(test)
test_aabb_accessors :: proc(t: ^testing.T) {
	a := [2][3]f32{{0, 0, 0}, {2, 4, 6}}
	testing.expect(t, veq(aabb.size(a), [3]f32{2, 4, 6}), "size")
	testing.expect(t, feq(aabb.volume(a), 48), "volume")
	testing.expect(t, feq(aabb.surface_area(a), 88), "surface_area")
	testing.expect(t, veq(aabb.center(a), [3]f32{1, 2, 3}), "center")

	testing.expect(
		t,
		veq(aabb.corner(a, .Min_X_Min_Y_Min_Z), [3]f32{0, 0, 0}),
		"corner min",
	)
	testing.expect(
		t,
		veq(aabb.corner(a, .Max_X_Max_Y_Max_Z), [3]f32{2, 4, 6}),
		"corner max",
	)
	testing.expect(
		t,
		veq(aabb.corner(a, .Max_X_Min_Y_Min_Z), [3]f32{2, 0, 0}),
		"corner mixed",
	)

	corners := aabb.corners(a)
	testing.expect(t, veq(corners[0], [3]f32{0, 0, 0}) && veq(corners[7], [3]f32{2, 4, 6}), "corners")

	nx := aabb.face(a, .Neg_X)
	testing.expect(t, veq(nx[0], [3]f32{0, 0, 0}) && veq(nx[1], [3]f32{0, 4, 6}), "face Neg_X")
	px := aabb.face(a, .Pos_X)
	testing.expect(t, veq(px[0], [3]f32{2, 0, 0}) && veq(px[1], [3]f32{2, 4, 6}), "face Pos_X")
}

@(test)
test_aabb_queries :: proc(t: ^testing.T) {
	a := [2][3]f32{{0, 0, 0}, {2, 4, 6}}
	testing.expect(t, aabb.has_point(a, [3]f32{1, 2, 3}), "has_point inside")
	testing.expect(t, !aabb.has_point(a, [3]f32{3, 0, 0}), "has_point outside")

	testing.expect(
		t,
		aabb.contains_aabb(a, [2][3]f32{{0.5, 0.5, 0.5}, {1, 1, 1}}),
		"contains true",
	)

	b := [2][3]f32{{1, 1, 1}, {3, 5, 7}}
	far := [2][3]f32{{3, 5, 7}, {4, 6, 8}}
	testing.expect(t, aabb.intersects(a, b), "intersects true")
	testing.expect(t, !aabb.intersects(a, far), "intersects false")

	ov, ok := aabb.intersection(a, b)
	testing.expect(t, ok, "intersection ok")
	testing.expect(
		t,
		veq(ov[0], [3]f32{1, 1, 1}) && veq(ov[1], [3]f32{2, 4, 6}),
		"intersection box",
	)

	m := aabb.merge(a, b)
	testing.expect(t, veq(m[0], [3]f32{0, 0, 0}) && veq(m[1], [3]f32{3, 5, 7}), "merge")

	v, c := aabb.nearest_vertex(a, [3]f32{-1, -1, -1})
	testing.expect(t, veq(v, [3]f32{0, 0, 0}) && c == .Min_X_Min_Y_Min_Z, "nearest_vertex")

	testing.expect(t, aabb.nearest_face(a, [3]f32{1, 2, -0.5}) == .Neg_Z, "nearest_face")

	testing.expect(
		t,
		veq(aabb.project_to_boundary(a, [3]f32{1, 2, 3}), [3]f32{0, 2, 3}),
		"project center",
	)

	testing.expect(t, veq(aabb.clamp_point(a, [3]f32{7, 7, 7}), [3]f32{2, 4, 6}), "clamp out")
	testing.expect(t, veq(aabb.clamp_point(a, [3]f32{-1, 2, 3}), [3]f32{0, 2, 3}), "clamp in")
}

@(test)
test_aabb_transforms :: proc(t: ^testing.T) {
	a := [2][3]f32{{0, 0, 0}, {2, 4, 6}}

	tr := aabb.translate(a, [3]f32{1, 1, 1})
	testing.expect(t, veq(tr[0], [3]f32{1, 1, 1}) && veq(tr[1], [3]f32{3, 5, 7}), "translate")

	sc := aabb.scale(a, [3]f32{2, 2, 2})
	testing.expect(t, veq(sc[0], [3]f32{-1, -2, -3}) && veq(sc[1], [3]f32{3, 6, 9}), "scale")

	gr := aabb.grow(a, [3]f32{1, 1, 1})
	testing.expect(t, veq(gr[0], [3]f32{-1, -1, -1}) && veq(gr[1], [3]f32{3, 5, 7}), "grow")

	gt := aabb.grow_to(a, [3]f32{3, 5, 7})
	testing.expect(t, veq(gt[0], [3]f32{0, 0, 0}) && veq(gt[1], [3]f32{3, 5, 7}), "grow_to")

	cr := aabb.clamp_aabb([2][3]f32{{10, 10, 10}, {11, 12, 13}}, a)
	testing.expect(t, veq(cr[0], [3]f32{1, 2, 3}) && veq(cr[1], [3]f32{2, 4, 6}), "clamp_aabb")
}

@(test)
test_aabb_partitioning :: proc(t: ^testing.T) {
	a := [2][3]f32{{0, 0, 0}, {2, 4, 6}}

	lo, hi := aabb.split_x(a, 1)
	testing.expect(t, veq(lo[1], [3]f32{1, 4, 6}) && veq(hi[0], [3]f32{1, 0, 0}), "split_x")

	loy, hiy := aabb.split_y(a, 2)
	testing.expect(t, veq(loy[1], [3]f32{2, 2, 6}) && veq(hiy[0], [3]f32{0, 2, 0}), "split_y")

	loz, hiz := aabb.split_z(a, 3)
	testing.expect(t, veq(loz[1], [3]f32{2, 4, 3}) && veq(hiz[0], [3]f32{0, 0, 3}), "split_z")

	oct := aabb.subdivide(a)
	testing.expect(
		t,
		veq(oct[0][0], [3]f32{0, 0, 0}) && veq(oct[0][1], [3]f32{1, 2, 3}),
		"subdivide octant 0",
	)
	testing.expect(
		t,
		veq(oct[7][0], [3]f32{1, 2, 3}) && veq(oct[7][1], [3]f32{2, 4, 6}),
		"subdivide octant 7",
	)
}

@(test)
test_aabb_integer :: proc(t: ^testing.T) {
	ai := [2][3]int{{0, 0, 0}, {2, 4, 6}}
	testing.expect(t, aabb.size(ai) == [3]int{2, 4, 6}, "int size")
	testing.expect(t, aabb.volume(ai) == 48, "int volume")
	testing.expect(t, aabb.has_point(ai, [3]int{1, 1, 1}), "int has_point")
	testing.expect(t, !aabb.has_point(ai, [3]int{9, 9, 9}), "int has_point out")
}

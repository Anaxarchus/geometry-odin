package tests

import "core:testing"
import "../rect"

@(test)
test_rect_constructors :: proc(t: ^testing.T) {
	r := rect.rect_from_center_size([2]f32{2, 1}, [2]f32{4, 2})
	testing.expect(t, veq(r[0], [2]f32{0, 0}), "from_center_size min")
	testing.expect(t, veq(r[1], [2]f32{4, 2}), "from_center_size max")

	pts := [][2]f32{{1, 1}, {-2, 3}, {4, -1}}
	b := rect.rect_from_points(pts)
	testing.expect(t, veq(b[0], [2]f32{-2, -1}), "from_points min")
	testing.expect(t, veq(b[1], [2]f32{4, 3}), "from_points max")
}

@(test)
test_rect_accessors :: proc(t: ^testing.T) {
	r := [2][2]f32{{0, 0}, {4, 2}}
	testing.expect(t, veq(rect.rect_get_size(r), [2]f32{4, 2}), "size")
	testing.expect(t, feq(rect.rect_get_area(r), 8), "area")
	testing.expect(t, feq(rect.rect_get_perimeter(r), 12), "perimeter")
	testing.expect(t, feq(rect.rect_get_aspect(r), 2.0), "aspect")
	testing.expect(t, veq(rect.rect_get_center(r), [2]f32{2, 1}), "center")

	testing.expect(t, veq(rect.rect_get_corner(r, .Top_Left), [2]f32{0, 0}), "corner TL")
	testing.expect(t, veq(rect.rect_get_corner(r, .Top_Right), [2]f32{4, 0}), "corner TR")
	testing.expect(t, veq(rect.rect_get_corner(r, .Bottom_Left), [2]f32{0, 2}), "corner BL")
	testing.expect(t, veq(rect.rect_get_corner(r, .Bottom_Right), [2]f32{4, 2}), "corner BR")

	corners := rect.rect_get_corners(r)
	testing.expect(t, veq(corners[0], [2]f32{0, 0}) && veq(corners[3], [2]f32{4, 2}), "corners")

	// Left edge: bottom-left to top-left
	le := rect.rect_get_edge(r, .Left)
	testing.expect(t, veq(le[0], [2]f32{0, 2}) && veq(le[1], [2]f32{0, 0}), "edge Left")
}

@(test)
test_rect_queries :: proc(t: ^testing.T) {
	r := [2][2]f32{{0, 0}, {4, 2}}
	testing.expect(t, rect.rect_has_point(r, [2]f32{2, 1}), "has_point inside")
	testing.expect(t, rect.rect_has_point(r, [2]f32{0, 0}), "has_point boundary")
	testing.expect(t, !rect.rect_has_point(r, [2]f32{5, 5}), "has_point outside")

	testing.expect(
		t,
		rect.rect_contains_rect(r, [2][2]f32{{1, 0.5}, {3, 1.5}}),
		"contains_rect true",
	)
	testing.expect(
		t,
		!rect.rect_contains_rect(r, [2][2]f32{{-1, 0}, {3, 1}}),
		"contains_rect false",
	)

	a := [2][2]f32{{0, 0}, {2, 2}}
	b := [2][2]f32{{1, 1}, {3, 3}}
	far := [2][2]f32{{3, 3}, {4, 4}}
	testing.expect(t, rect.rect_intersects(a, b), "intersects true")
	testing.expect(t, !rect.rect_intersects(a, far), "intersects false")

	ov, ok := rect.rect_intersection(a, b)
	testing.expect(t, ok, "intersection ok")
	testing.expect(t, veq(ov[0], [2]f32{1, 1}) && veq(ov[1], [2]f32{2, 2}), "intersection rect")
	_, ok2 := rect.rect_intersection(a, far)
	testing.expect(t, !ok2, "intersection none")

	m := rect.rect_merge(a, b)
	testing.expect(t, veq(m[0], [2]f32{0, 0}) && veq(m[1], [2]f32{3, 3}), "merge")

	v, c := rect.rect_get_nearest_vertex(r, [2]f32{-1, -1})
	testing.expect(t, veq(v, [2]f32{0, 0}) && c == .Top_Left, "nearest_vertex")

	testing.expect(t, rect.rect_get_nearest_edge(r, [2]f32{2, -1}) == .Top, "nearest_edge")

	// center projects to the nearest edge (tie resolves to Top)
	testing.expect(
		t,
		veq(rect.rect_project_to_boundary(r, [2]f32{2, 1}), [2]f32{2, 0}),
		"project center",
	)
	// outside point projects onto the right edge
	testing.expect(
		t,
		veq(rect.rect_project_to_boundary(r, [2]f32{5, 1}), [2]f32{4, 1}),
		"project outside",
	)

	testing.expect(t, veq(rect.rect_clamp_point(r, [2]f32{5, 5}), [2]f32{4, 2}), "clamp out")
	testing.expect(t, veq(rect.rect_clamp_point(r, [2]f32{2, 1}), [2]f32{2, 1}), "clamp in")
}

@(test)
test_rect_transforms :: proc(t: ^testing.T) {
	r := [2][2]f32{{0, 0}, {4, 2}}

	tr := rect.rect_translate(r, [2]f32{1, 1})
	testing.expect(t, veq(tr[0], [2]f32{1, 1}) && veq(tr[1], [2]f32{5, 3}), "translate")

	sc := rect.rect_scale(r, [2]f32{2, 2})
	testing.expect(t, veq(sc[0], [2]f32{-2, -1}) && veq(sc[1], [2]f32{6, 3}), "scale")

	gr := rect.rect_grow(r, [2]f32{1, 1})
	testing.expect(t, veq(gr[0], [2]f32{-1, -1}) && veq(gr[1], [2]f32{5, 3}), "grow")

	gs := rect.rect_grow_sides(r, 1, 0, 0, 0)
	testing.expect(t, veq(gs[0], [2]f32{-1, 0}) && veq(gs[1], [2]f32{4, 2}), "grow_sides")

	gt := rect.rect_grow_to(r, [2]f32{6, 6})
	testing.expect(t, veq(gt[0], [2]f32{0, 0}) && veq(gt[1], [2]f32{6, 6}), "grow_to")

	// shrink-and-translate a far rect into bounds
	cr := rect.rect_clamp_rect([2][2]f32{{10, 10}, {12, 11}}, r)
	testing.expect(t, veq(cr[0], [2]f32{2, 1}) && veq(cr[1], [2]f32{4, 2}), "clamp_rect")
}

@(test)
test_rect_partitioning :: proc(t: ^testing.T) {
	r := [2][2]f32{{0, 0}, {4, 2}}

	l, rr := rect.rect_split_x(r, 2)
	testing.expect(t, veq(l[1], [2]f32{2, 2}) && veq(rr[0], [2]f32{2, 0}), "split_x")

	tp, bm := rect.rect_split_y(r, 1)
	testing.expect(t, veq(tp[1], [2]f32{4, 1}) && veq(bm[0], [2]f32{0, 1}), "split_y")

	lf, _ := rect.rect_split_x_frac(r, 0.25)
	testing.expect(t, veq(lf[1], [2]f32{1, 2}), "split_x_frac")

	sl, rem := rect.rect_cut(r, .Left, 1)
	testing.expect(t, veq(sl[1], [2]f32{1, 2}), "cut slice")
	testing.expect(t, veq(rem[0], [2]f32{1, 0}), "cut remainder")

	slr, _ := rect.rect_cut(r, .Right, 1)
	testing.expect(t, veq(slr[0], [2]f32{3, 0}), "cut right")

	slf, _ := rect.rect_cut_frac(r, .Left, 0.25)
	testing.expect(t, veq(slf[1], [2]f32{1, 2}), "cut_frac")

	cols := rect.rect_divide_x(r, 2, 0)
	defer delete(cols)
	testing.expect(t, len(cols) == 2, "divide count")
	testing.expect(t, veq(cols[0][1], [2]f32{2, 2}) && veq(cols[1][0], [2]f32{2, 0}), "divide_x")

	stack, _, re := rect.rect_stack_x(r, 2, 1, 0)
	defer delete(stack)
	testing.expect(t, len(stack) == 2, "stack count")
	testing.expect(t, veq(stack[0][1], [2]f32{1, 2}), "stack first")
	testing.expect(t, veq(stack[1][0], [2]f32{1, 0}) && veq(stack[1][1], [2]f32{2, 2}), "stack second")
	testing.expect(t, veq(re[0], [2]f32{2, 0}) && veq(re[1], [2]f32{4, 2}), "stack remainder_end")

	tris := rect.rect_triangulate(r)
	testing.expect(t, veq(tris[0][0], [2]f32{0, 0}), "triangulate v0")
	testing.expect(t, veq(tris[0][2], [2]f32{4, 2}), "triangulate v2")
}

@(test)
test_rect_integer :: proc(t: ^testing.T) {
	ri := [2][2]int{{0, 0}, {4, 2}}
	testing.expect(t, rect.rect_get_size(ri) == [2]int{4, 2}, "int size")
	testing.expect(t, rect.rect_get_area(ri) == 8, "int area")
	testing.expect(t, rect.rect_has_point(ri, [2]int{1, 1}), "int has_point")
	testing.expect(t, !rect.rect_has_point(ri, [2]int{9, 9}), "int has_point out")

	m := rect.rect_merge(ri, [2][2]int{{-1, -1}, {1, 1}})
	testing.expect(t, m == [2][2]int{{-1, -1}, {4, 2}}, "int merge")
}

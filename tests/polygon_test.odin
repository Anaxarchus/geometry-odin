package tests

import "core:testing"
import "../polygon"

// --- local helpers -------------------------------------------------------

@(private = "file")
contour_set_area :: proc(cs: [][][2]f32) -> f32 {
	total: f32
	for c in cs {
		total += polygon.area(c)
	}
	return total
}

@(private = "file")
free_contours :: proc(cs: [][][2]f32) {
	for c in cs {
		delete(c)
	}
	delete(cs)
}

@(private = "file")
tri_area :: proc(tri: [3][2]f32) -> f32 {
	a := tri[0]
	b := tri[1]
	c := tri[2]
	return 0.5 * abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y))
}

@(private = "file")
near :: proc(a, b: f32) -> bool {
	return abs(a - b) < 0.01
}

// --- analytic ops --------------------------------------------------------

@(test)
test_polygon_area_perimeter :: proc(t: ^testing.T) {
	sq := [][2]f32{{0, 0}, {2, 0}, {2, 2}, {0, 2}}
	testing.expect(t, feq(polygon.area(sq), 4), "area")
	testing.expect(t, feq(polygon.perimeter(sq), 8), "perimeter")
}

@(test)
test_polygon_winding :: proc(t: ^testing.T) {
	ccw := [][2]f32{{0, 0}, {2, 0}, {2, 2}, {0, 2}}
	cw := [][2]f32{{0, 2}, {2, 2}, {2, 0}, {0, 0}}
	testing.expect(t, !polygon.is_clockwise(ccw), "ccw not clockwise")
	testing.expect(t, polygon.is_clockwise(cw), "cw clockwise")
}

@(test)
test_polygon_convexity :: proc(t: ^testing.T) {
	sq := [][2]f32{{0, 0}, {2, 0}, {2, 2}, {0, 2}}
	l := [][2]f32{{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}}
	testing.expect(t, polygon.is_convex(sq), "square convex")
	testing.expect(t, !polygon.is_convex(l), "L not convex")
	// the L-shape has area 3 (2x2 minus a 1x1 corner)
	testing.expect(t, feq(polygon.area(l), 3), "L area")
}

@(test)
test_polygon_point_in :: proc(t: ^testing.T) {
	sq := [][2]f32{{0, 0}, {2, 0}, {2, 2}, {0, 2}}
	testing.expect(t, polygon.has_point(sq, [2]f32{1, 1}), "even-odd inside")
	testing.expect(
		t,
		polygon.has_point(sq, [2]f32{1, 1}, .Winding_Number),
		"winding inside",
	)
	testing.expect(t, !polygon.has_point(sq, [2]f32{3, 3}), "outside")

	// concave: the notch corner is outside, the base is inside
	l := [][2]f32{{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}}
	testing.expect(t, polygon.has_point(l, [2]f32{0.5, 0.5}), "L base inside")
	testing.expect(t, !polygon.has_point(l, [2]f32{1.5, 1.5}), "L notch outside")
}

@(test)
test_polygon_min_max_edges :: proc(t: ^testing.T) {
	sq := [][2]f32{{0, 0}, {2, 0}, {2, 2}, {0, 2}}
	mm := polygon.min_max(sq)
	testing.expect(t, veq(mm[0], [2]f32{0, 0}) && veq(mm[1], [2]f32{2, 2}), "min_max")

	edges := polygon.edges(sq)
	defer delete(edges)
	testing.expect(t, len(edges) == 4, "edge count")
	testing.expect(t, veq(edges[0][0], [2]f32{0, 0}) && veq(edges[0][1], [2]f32{2, 0}), "edge 0")
	// closing edge wraps last -> first
	testing.expect(t, veq(edges[3][0], [2]f32{0, 2}) && veq(edges[3][1], [2]f32{0, 0}), "closing edge")
}

@(test)
test_polygon_simplify :: proc(t: ^testing.T) {
	// (1,0) is collinear on the bottom edge and should be removed
	poly := [][2]f32{{0, 0}, {1, 0}, {2, 0}, {2, 2}, {0, 2}}
	out := polygon.simplify(poly, 0.001)
	defer delete(out)
	testing.expect(t, len(out) == 4, "collinear point removed")
	testing.expect(t, feq(polygon.area(out), 4), "area preserved")
}

@(test)
test_polygon_decimate :: proc(t: ^testing.T) {
	// (1, 0.001) is a tiny near-straight bump; a 0.1 rad threshold drops it
	poly := [][2]f32{{0, 0}, {1, 0.001}, {2, 0}, {2, 2}, {0, 2}}
	out := polygon.decimate(poly, 0.1)
	defer delete(out)
	testing.expect(t, len(out) == 4, "near-straight vertex removed")
}

// --- libtess2-backed ops -------------------------------------------------

@(test)
test_polygon_triangulate :: proc(t: ^testing.T) {
	sq := [][2]f32{{0, 0}, {2, 0}, {2, 2}, {0, 2}}
	tris := polygon.triangulate(sq)
	defer delete(tris)
	defer free_all(context.temp_allocator)

	testing.expect(t, len(tris) >= 1, "produced triangles")
	total: f32
	for tri in tris {
		total += tri_area(tri)
	}
	testing.expect(t, near(total, 4), "triangulated area preserved")

	// earcut backend should agree on total area
	fast := polygon.triangulate(sq, .Fast)
	defer delete(fast)
	ftotal: f32
	for tri in fast {
		ftotal += tri_area(tri)
	}
	testing.expect(t, near(ftotal, 4), "fast (earcut) area preserved")
}

@(test)
test_polygon_booleans :: proc(t: ^testing.T) {
	a := [][2]f32{{0, 0}, {2, 0}, {2, 2}, {0, 2}} // [0,2]^2
	b := [][2]f32{{1, 1}, {3, 1}, {3, 3}, {1, 3}} // [1,3]^2, overlap [1,2]^2 = 1
	defer free_all(context.temp_allocator)

	u := polygon.union_polygon([][][2]f32{a, b})
	defer free_contours(u)
	testing.expect(t, near(contour_set_area(u), 7), "union area 7")

	x := polygon.intersect([][][2]f32{a, b})
	defer free_contours(x)
	testing.expect(t, near(contour_set_area(x), 1), "intersect area 1")

	d := polygon.difference([][][2]f32{a, b})
	defer free_contours(d)
	testing.expect(t, near(contour_set_area(d), 3), "difference area 3")

	xr := polygon.xor([][][2]f32{a, b})
	defer free_contours(xr)
	testing.expect(t, near(contour_set_area(xr), 6), "xor area 6")
}

@(test)
test_polygon_offset :: proc(t: ^testing.T) {
	sq := [][2]f32{{0, 0}, {2, 0}, {2, 2}, {0, 2}}
	off := polygon.offset(sq, 0.5, .Miter)
	defer free_contours(off)
	defer free_all(context.temp_allocator)

	testing.expect(t, len(off) > 0, "offset non-empty")
	testing.expect(t, contour_set_area(off) > 0, "offset area positive")
}

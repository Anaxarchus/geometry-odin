package rect

import "base:intrinsics"
import "core:math/linalg"

Rect_Corner :: enum {
	Top_Left,
	Top_Right,
	Bottom_Left,
	Bottom_Right,
}

Rect_Edge :: enum {
	Left,
	Right,
	Top,
	Bottom,
}

Rect_Alignment :: enum {
	Begin,
	Center,
	End,
}

@(private)
is_scalar :: intrinsics.type_is_ordered_numeric

// builds a rect from a center point and full size
from_center_size :: proc(center, size: [2]$T) -> [2][2]T where is_scalar(T) {
	hs := size / 2
	return {center - hs, center + hs}
}

// returns the axis-aligned bounding box of the given points
from_points :: proc(points: [][2]$T) -> [2][2]T where is_scalar(T) {
	result := [2][2]T{points[0], points[0]}
	for v in points[1:] {
		result[0] = linalg.min(result[0], v)
		result[1] = linalg.max(result[1], v)
	}
	return result
}

// returns the size (width, height) of the rect
size :: #force_inline proc "contextless" (r: [2][2]$T) -> [2]T where is_scalar(T) {
	return r[1] - r[0]
}

// returns the area of the rect
area :: #force_inline proc "contextless" (r: [2][2]$T) -> T where is_scalar(T) {
	size := size(r)
	return size.x * size.y
}

// returns the perimeter of the rect
perimeter :: #force_inline proc "contextless" (r: [2][2]$T) -> T where is_scalar(T) {
	size := size(r)
	return 2 * (size.x + size.y)
}

// returns the aspect ratio of the rect (width / height)
aspect :: #force_inline proc "contextless" (r: [2][2]$T) -> f64 where is_scalar(T) {
	size := size(r)
	return f64(size.x) / f64(size.y)
}

// returns the center point of the rect
center :: #force_inline proc "contextless" (r: [2][2]$T) -> [2]T where is_scalar(T) {
	return (r[0] + r[1]) / 2
}

// returns the position of a single named corner
corner :: #force_inline proc "contextless" (
	r: [2][2]$T,
	corner: Rect_Corner,
) -> [2]T where is_scalar(T) {
	switch corner {
	case .Top_Left:
		return r[0]
	case .Top_Right:
		return [2]T{r[1].x, r[0].y}
	case .Bottom_Left:
		return [2]T{r[0].x, r[1].y}
	case .Bottom_Right:
		return r[1]
	}
	return {}
}

// returns all four corners in Rect_Corner order (Top_Left, Top_Right, Bottom_Left, Bottom_Right)
corners :: #force_inline proc "contextless" (r: [2][2]$T) -> [4][2]T where is_scalar(T) {
	return {r[0], [2]T{r[1].x, r[0].y}, [2]T{r[0].x, r[1].y}, r[1]}
}

// returns the two endpoints of a named edge
edge :: #force_inline proc "contextless" (
	r: [2][2]$T,
	edge: Rect_Edge,
) -> [2][2]T where is_scalar(T) {
	switch edge {

	// bottom-left to top-left
	case .Left:
		return {{r[0].x, r[1].y}, r[0]}

	// top-right to bottom-right
	case .Right:
		return {{r[1].x, r[0].y}, r[1]}

	// top-left to top-right
	case .Top:
		return {r[0], {r[1].x, r[0].y}}

	// bottom-right to bottom-left
	case .Bottom:
		return {r[1], {r[0].x, r[1].y}}

	}
	return {}
}

// returns true if a rect contains a given point
has_point :: #force_inline proc "contextless" (
	r: [2][2]$T,
	p: [2]T,
) -> bool where is_scalar(T) {
	return p.x >= r[0].x && p.x <= r[1].x && p.y >= r[0].y && p.y <= r[1].y
}

// returns true if `outer` fully encloses `inner`
contains_rect :: #force_inline proc "contextless" (
	outer, inner: [2][2]$T,
) -> bool where is_scalar(T) {
	return(
		inner[0].x >= outer[0].x &&
		inner[0].y >= outer[0].y &&
		inner[1].x <= outer[1].x &&
		inner[1].y <= outer[1].y \
	)
}

// returns true if the two rects overlap
intersects :: #force_inline proc "contextless" (a, b: [2][2]$T) -> bool where is_scalar(T) {
	return a[0].x <= b[1].x && a[1].x >= b[0].x && a[0].y <= b[1].y && a[1].y >= b[0].y
}

// returns the overlapping rect; ok is false when they don't intersect
intersection :: #force_inline proc "contextless" (
	a, b: [2][2]$T,
) -> (overlap: [2][2]T, ok: bool) where is_scalar(T) {
	lo := linalg.max(a[0], b[0])
	hi := linalg.min(a[1], b[1])
	if lo.x > hi.x || lo.y > hi.y {
		return {}, false
	}
	return {lo, hi}, true
}

// returns a new rect enclosing two given rects (their union / bounding box)
merge :: #force_inline proc "contextless" (a, b: [2][2]$T) -> [2][2]T where is_scalar(T) {
	return {linalg.min(a[0], b[0]), linalg.max(a[1], b[1])}
}

// returns the vertex of the rect closest to the given point, and which corner it is
nearest_vertex :: proc "contextless" (
	r: [2][2]$T,
	p: [2]T,
) -> (vertex: [2]T, corner: Rect_Corner) where is_scalar(T) {
	corners := corners(r)
	best := 0
	d := corners[0] - p
	best_dist := d.x * d.x + d.y * d.y
	for i in 1 ..< 4 {
		d = corners[i] - p
		dist := d.x * d.x + d.y * d.y
		if dist < best_dist {
			best_dist = dist
			best = i
		}
	}
	return corners[best], Rect_Corner(best)
}

// returns the edge of the rect closest to the given point
nearest_edge :: proc "contextless" (
	r: [2][2]$T,
	p: [2]T,
) -> Rect_Edge where is_scalar(T) {
	dl := abs(p.x - r[0].x)
	dr := abs(p.x - r[1].x)
	dt := abs(p.y - r[0].y)
	db := abs(p.y - r[1].y)
	edge := Rect_Edge.Left
	best := dl
	if dr < best {best = dr;edge = .Right}
	if dt < best {best = dt;edge = .Top}
	if db < best {best = db;edge = .Bottom}
	return edge
}

// returns the nearest point on the boundary to the given point (snaps even from inside)
project_to_boundary :: proc "contextless" (r: [2][2]$T, p: [2]T) -> [2]T where is_scalar(T) {
	c := clamp_point(r, p)
	dl := abs(c.x - r[0].x)
	dr := abs(r[1].x - c.x)
	dt := abs(c.y - r[0].y)
	db := abs(r[1].y - c.y)
	m := min(dl, dr, dt, db)
	result := c
	switch {
	case m == dl:
		result.x = r[0].x
	case m == dr:
		result.x = r[1].x
	case m == dt:
		result.y = r[0].y
	case:
		result.y = r[1].y
	}
	return result
}

// clamps a point into the rect; interior points are returned unchanged
clamp_point :: #force_inline proc "contextless" (
	r: [2][2]$T,
	p: [2]T,
) -> [2]T where is_scalar(T) {
	return {clamp(p.x, r[0].x, r[1].x), clamp(p.y, r[0].y, r[1].y)}
}

// moves the rect by an offset without resizing
translate :: #force_inline proc "contextless" (
	r: [2][2]$T,
	offset: [2]T,
) -> [2][2]T where is_scalar(T) {
	return {r[0] + offset, r[1] + offset}
}

// scales the rect about its center by a per-axis factor
scale :: #force_inline proc "contextless" (
	r: [2][2]$T,
	factor: [2]T,
) -> [2][2]T where is_scalar(T) {
	c := center(r)
	hs := size(r) / 2 * factor
	return {c - hs, c + hs}
}

// grows a rect outward on all sides by a given vector (width grows by 2*amount.x)
grow :: #force_inline proc "contextless" (
	r: [2][2]$T,
	amount: [2]T,
) -> [2][2]T where is_scalar(T) {
	return {r[0] - amount, r[1] + amount}
}

// grows a rect by the given amount per side
grow_sides :: #force_inline proc "contextless" (
	r: [2][2]$T,
	left: T,
	right: T,
	top: T,
	bottom: T,
) -> [2][2]T where is_scalar(T) {
	return {{r[0].x - left, r[0].y - top}, {r[1].x + right, r[1].y + bottom}}
}

// grows a rect to enclose a given point
grow_to :: #force_inline proc "contextless" (
	r: [2][2]$T,
	p: [2]T,
) -> [2][2]T where is_scalar(T) {
	return {linalg.min(r[0], p), linalg.max(r[1], p)}
}

// constrains a rect to sit fully inside `bounds` (translating, and shrinking if needed)
clamp_rect :: proc "contextless" (
	r: [2][2]$T,
	bounds: [2][2]T,
) -> [2][2]T where is_scalar(T) {
	size := linalg.min(size(r), size(bounds))
	lo := r[0]
	lo = linalg.min(lo, bounds[1] - size)
	lo = linalg.max(lo, bounds[0])
	return {lo, lo + size}
}

// splits a rect into two new rects at the given x coordinate
split_x :: #force_inline proc "contextless" (
	r: [2][2]$T,
	x: T,
) -> (left, right: [2][2]T) where is_scalar(T) {
	left = {r[0], {x, r[1].y}}
	right = {{x, r[0].y}, r[1]}
	return
}

// splits a rect into two new rects at the given y coordinate
split_y :: #force_inline proc "contextless" (
	r: [2][2]$T,
	y: T,
) -> (top, bottom: [2][2]T) where is_scalar(T) {
	top = {r[0], {r[1].x, y}}
	bottom = {{r[0].x, y}, r[1]}
	return
}

// splits a rect into two at a fraction t in [0, 1] along the x axis
split_x_frac :: #force_inline proc "contextless" (
	r: [2][2]$T,
	t: f32,
) -> (left, right: [2][2]T) where is_scalar(T) {
	x := r[0].x + T(f64(size(r).x) * f64(t))
	return split_x(r, x)
}

// splits a rect into two at a fraction t in [0, 1] along the y axis
split_y_frac :: #force_inline proc "contextless" (
	r: [2][2]$T,
	t: f32,
) -> (top, bottom: [2][2]T) where is_scalar(T) {
	y := r[0].y + T(f64(size(r).y) * f64(t))
	return split_y(r, y)
}

// cuts a fixed amount off one edge, returning the slice and what remains.
cut :: proc "contextless" (
	r: [2][2]$T,
	edge: Rect_Edge,
	amount: T,
) -> (slice, remainder: [2][2]T) where is_scalar(T) {
	switch edge {
	case .Left:
		x := r[0].x + amount
		slice = {r[0], {x, r[1].y}}
		remainder = {{x, r[0].y}, r[1]}
	case .Right:
		x := r[1].x - amount
		slice = {{x, r[0].y}, r[1]}
		remainder = {r[0], {x, r[1].y}}
	case .Top:
		y := r[0].y + amount
		slice = {r[0], {r[1].x, y}}
		remainder = {{r[0].x, y}, r[1]}
	case .Bottom:
		y := r[1].y - amount
		slice = {{r[0].x, y}, r[1]}
		remainder = {r[0], {r[1].x, y}}
	}
	return
}

// cuts a fraction t in [0, 1] off one edge, returning the slice and what remains
cut_frac :: #force_inline proc "contextless" (
	r: [2][2]$T,
	edge: Rect_Edge,
	t: f32,
) -> (slice, remainder: [2][2]T) where is_scalar(T) {
	size := size(r)
	amount: T
	switch edge {
	case .Left, .Right:
		amount = T(f64(size.x) * f64(t))
	case .Top, .Bottom:
		amount = T(f64(size.y) * f64(t))
	}
	return cut(r, edge, amount)
}

// divides a rect into N equal rects along the x axis, with optional gap between them
divide_x :: proc(
	r: [2][2]$T,
	n: int,
	gap: T,
	allocator := context.allocator,
) -> [][2][2]T where is_scalar(T) {
	result := make([][2][2]T, n, allocator)
	size := size(r)
	w := (size.x - gap * T(n - 1)) / T(n)
	for i in 0 ..< n {
		x0 := r[0].x + (w + gap) * T(i)
		result[i] = {{x0, r[0].y}, {x0 + w, r[1].y}}
	}
	return result
}

// divides a rect into N equal rects along the y axis, with optional gap between them
divide_y :: proc(
	r: [2][2]$T,
	n: int,
	gap: T,
	allocator := context.allocator,
) -> [][2][2]T where is_scalar(T) {
	result := make([][2][2]T, n, allocator)
	size := size(r)
	h := (size.y - gap * T(n - 1)) / T(n)
	for i in 0 ..< n {
		y0 := r[0].y + (h + gap) * T(i)
		result[i] = {{r[0].x, y0}, {r[1].x, y0 + h}}
	}
	return result
}

// portions a rect into N slices of fixed width x along the x axis, returning the
// stack and the leftover. align positions the stack within r (see remainder note below).
stack_x :: proc(
	r: [2][2]$T,
	n: int,
	x: T,
	gap: T,
	align: Rect_Alignment = .Begin,
	allocator := context.allocator,
) -> (
	stack: [][2][2]T,
	remainder_begin, remainder_end: [2][2]T,
) where is_scalar(T) {
	stack = make([][2][2]T, n, allocator)
	total := x * T(n) + gap * T(n - 1)
	leftover := size(r).x - total
	pad: T
	switch align {
	case .Begin:
		pad = 0
	case .End:
		pad = leftover
	case .Center:
		pad = leftover / 2
	}
	start := r[0].x + pad
	for i in 0 ..< n {
		x0 := start + (x + gap) * T(i)
		stack[i] = {{x0, r[0].y}, {x0 + x, r[1].y}}
	}
	stack_end := start + total
	remainder_begin = {r[0], {start, r[1].y}}
	remainder_end = {{stack_end, r[0].y}, r[1]}
	return
}

// portions a rect into N slices of fixed height y along the y axis, returning the
// stack and the leftover. align positions the stack within r.
// remainder semantics for stack_x/y:
//   .Begin  -> stack sits at the min edge; remainder is the trailing leftover (contiguous)
//   .End    -> stack sits at the max edge; remainder is the leading  leftover (contiguous)
//   .Center -> leftover is split evenly both sides; remainder is the trailing half
//              (the leading pad is the same width, so the caller can reconstruct it)
// Use .Begin/.End when you intend to chain off the remainder.
stack_y :: proc(
	r: [2][2]$T,
	n: int,
	y: T,
	gap: T,
	align: Rect_Alignment = .Begin,
	allocator := context.allocator,
) -> (
	stack: [][2][2]T,
	remainder_begin, remainder_end: [2][2]T,
) where is_scalar(T) {
	stack = make([][2][2]T, n, allocator)
	total := y * T(n) + gap * T(n - 1)
	leftover := size(r).y - total
	pad: T
	switch align {
	case .Begin:
		pad = 0
	case .End:
		pad = leftover
	case .Center:
		pad = leftover / 2
	}
	start := r[0].y + pad
	for i in 0 ..< n {
		y0 := start + (y + gap) * T(i)
		stack[i] = {{r[0].x, y0}, {r[1].x, y0 + y}}
	}
	stack_end := start + total
	remainder_begin = {r[0], {r[1].x, start}}
	remainder_end = {{r[0].x, stack_end}, r[1]}
	return
}

// returns the two triangles forming the rect (each as three vertices, consistent winding)
triangulate :: #force_inline proc "contextless" (
	r: [2][2]$T,
) -> [2][3][2]T where is_scalar(T) {
	tl := r[0]
	tr := [2]T{r[1].x, r[0].y}
	br := r[1]
	bl := [2]T{r[0].x, r[1].y}
	return {{tl, tr, br}, {tl, br, bl}}
}

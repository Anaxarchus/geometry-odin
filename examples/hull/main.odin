package main

// Convex hull builder. Drag a cloud of points around; the hull is computed by
// gift-wrapping (Jarvis march) with `line.side` as the orientation test, and the
// resulting contour is then run through the rest of the library. Nearly every
// line here is a geometry call -- raylib only opens the window, reads the mouse
// and draws.
//
//   line     -> side (the wrap test) + distance / project / normal / center /
//               angle (the mouse probe)
//   polygon  -> is_convex, is_clockwise, area, perimeter, min_max, edges,
//               triangulate, has_point
//   rect     -> center / size / area / corners on the hull's bounding box
//               (polygon.min_max returns a rect)
//   circle   -> from_contour (centroid fit) + has_point (the point picker)
//   triangle -> centroid + area (summed to cross-check polygon.area)
//
// Controls:
//   left click / drag : grab a point, or drop a new one on empty space
//   right click       : delete the nearest point
//   R                 : scatter a fresh random cloud
//   N                 : toggle per-edge normals
//
// run: odin run examples/hull

import rl "vendor:raylib"
import "core:fmt"
import "core:math/linalg"
import "../../circle"
import "../../line"
import "../../polygon"
import "../../rect"
import "../../triangle"

WIDTH :: 1000
HEIGHT :: 720
GRAB :: 12.0 // pick radius (px) for grabbing / deleting points

BG :: rl.Color{22, 24, 30, 255}
GRID :: rl.Color{60, 64, 78, 255}

v2 :: proc(p: [2]f32) -> rl.Vector2 {
	return rl.Vector2(p)
}

scatter :: proc(points: ^[dynamic][2]f32, n: int) {
	clear(points)
	for _ in 0 ..< n {
		x := f32(rl.GetRandomValue(140, WIDTH - 140))
		y := f32(rl.GetRandomValue(120, HEIGHT - 160))
		append(points, [2]f32{x, y})
	}
}

draw_points :: proc(points: [][2]f32, dragging: int) {
	for p, i in points {
		col := i == dragging ? rl.YELLOW : rl.RAYWHITE
		rl.DrawCircleV(v2(p), 4, col)
	}
}

// Gift-wrapping (Jarvis march): start at the leftmost point and repeatedly pick
// the next vertex so that every other point lies to one side of the directed
// edge. `line.side` gives that orientation test; ties (collinear points) are
// broken by keeping the farther one so collinear hull edges stay maximal.
gift_wrap :: proc(points: [][2]f32, allocator := context.allocator) -> [][2]f32 {
	n := len(points)
	if n < 3 {
		out := make([][2]f32, n, allocator)
		copy(out, points)
		return out
	}

	// leftmost point (lowest x, then lowest y) is guaranteed on the hull
	start := 0
	for i in 1 ..< n {
		if points[i].x < points[start].x ||
		   (points[i].x == points[start].x && points[i].y < points[start].y) {
			start = i
		}
	}

	hull := make([dynamic][2]f32, 0, n, allocator)
	current := start
	for {
		append(&hull, points[current])
		next := (current + 1) % n
		for cand in 0 ..< n {
			if cand == current || cand == next {
				continue
			}
			edge := [2][2]f32{points[current], points[next]}
			s := line.side(edge, points[cand])
			if s > 0 {
				// cand is left of current->next: a more extreme wrap
				next = cand
			} else if s == 0 {
				// collinear: keep whichever is farther from current
				dc := points[cand] - points[current]
				dn := points[next] - points[current]
				if linalg.dot(dc, dc) > linalg.dot(dn, dn) {
					next = cand
				}
			}
		}
		current = next
		if current == start || len(hull) > n {
			break
		}
	}
	return hull[:]
}

main :: proc() {
	rl.InitWindow(WIDTH, HEIGHT, "geometry-odin: convex hull")
	rl.SetTargetFPS(60)

	points := make([dynamic][2]f32, 0, 64)
	defer delete(points)
	scatter(&points, 12)

	dragging := -1
	show_normals := false

	for !rl.WindowShouldClose() {
		mp := rl.GetMousePosition()
		m := [2]f32{mp.x, mp.y}

		// ---- input ----
		if rl.IsKeyPressed(.R) {scatter(&points, 12)}
		if rl.IsKeyPressed(.N) {show_normals = !show_normals}

		if rl.IsMouseButtonPressed(.LEFT) {
			dragging = -1
			for p, i in points {
				if circle.has_point(m, p, GRAB) { 	// circle pick test
					dragging = i
					break
				}
			}
			if dragging == -1 {
				append(&points, m)
				dragging = len(points) - 1
			}
		}
		if rl.IsMouseButtonDown(.LEFT) && dragging >= 0 {
			points[dragging] = m
		}
		if rl.IsMouseButtonReleased(.LEFT) {
			dragging = -1
		}
		if rl.IsMouseButtonPressed(.RIGHT) && len(points) > 3 {
			nearest := -1
			best: f32 = GRAB * GRAB
			for p, i in points {
				d := p - m
				if linalg.dot(d, d) < best {
					best = linalg.dot(d, d)
					nearest = i
				}
			}
			if nearest >= 0 {
				unordered_remove(&points, nearest)
			}
		}

		// ---- geometry ----
		hull := gift_wrap(points[:], context.temp_allocator)

		rl.BeginDrawing()
		rl.ClearBackground(BG)

		if len(hull) >= 3 {
			// --- polygon analysis ---
			area := polygon.area(hull)
			perim := polygon.perimeter(hull)
			convex := polygon.is_convex(hull)
			cw := polygon.is_clockwise(hull)
			mm := polygon.min_max(hull) // {min, max} == a rect

			// --- bounding rect via the rect package ---
			bbcenter := rect.center(mm)
			bbsize := rect.size(mm)
			bbarea := rect.area(mm)
			corners := rect.corners(mm)

			// --- centroid-fit circles from the circle package ---
			oc_max, r_max := circle.from_contour(hull, .Max)
			oc_avg, r_avg := circle.from_contour(hull, .Average)

			// --- triangulate; sum triangle areas to cross-check the polygon area ---
			tris := polygon.triangulate(hull, .Robust, context.temp_allocator)
			tri_area: f32 = 0
			for t in tris {
				tri_area += triangle.area(t)
			}

			// --- mouse probe: containment + nearest hull edge ---
			edges := polygon.edges(hull, context.temp_allocator)
			inside := polygon.has_point(hull, m, .Winding_Number)
			ne := 0
			best: f32 = 1e30
			for e, i in edges {
				d := line.distance(e, m)
				if d < best {
					best = d
					ne = i
				}
			}
			pe := edges[ne]
			proj := line.project(pe, m)
			sd := line.side(pe, m)
			ec := line.center(pe)
			en := line.normal(pe)
			ea := line.angle(pe)

			// ---- draw ----
			// bounding box
			rl.DrawRectangleLinesEx(rl.Rectangle{mm[0].x, mm[0].y, bbsize.x, bbsize.y}, 1, GRID)
			for c in corners {
				rl.DrawCircleV(v2(c), 2, GRID)
			}

			// fitted circles
			rl.DrawCircleLinesV(v2(oc_max), r_max, rl.Fade(rl.SKYBLUE, 0.45))
			rl.DrawCircleLinesV(v2(oc_avg), r_avg, rl.Fade(rl.ORANGE, 0.40))
			rl.DrawCircleV(v2(oc_max), 3, rl.SKYBLUE)

			// triangulated fill + per-triangle centroids
			for t in tris {
				a := v2(t[0]);b := v2(t[1]);c := v2(t[2])
				rl.DrawTriangle(a, b, c, rl.Fade(rl.DARKBLUE, 0.25))
				rl.DrawTriangle(a, c, b, rl.Fade(rl.DARKBLUE, 0.25)) // both windings
				rl.DrawCircleV(v2(triangle.centroid(t)), 1.5, rl.Fade(rl.RAYWHITE, 0.5))
			}

			// the points themselves (above the fill, below the outline)
			draw_points(points[:], dragging)

			// hull outline + optional edge normals
			for e in edges {
				rl.DrawLineEx(v2(e[0]), v2(e[1]), 2, rl.SKYBLUE)
				if show_normals {
					c := line.center(e)
					rl.DrawLineEx(v2(c), v2(c + line.normal(e) * 16), 1, rl.Fade(rl.LIME, 0.7))
				}
			}

			// probe: nearest edge highlighted, projection + that edge's normal
			rl.DrawLineEx(v2(pe[0]), v2(pe[1]), 3, rl.RED)
			rl.DrawLineEx(v2(m), v2(proj), 1, rl.Fade(rl.RAYWHITE, 0.6))
			rl.DrawLineEx(v2(ec), v2(ec + en * 26), 2, rl.RED)
			rl.DrawCircleV(v2(proj), 4, rl.RED)
			rl.DrawCircleV(v2(m), 5, inside ? rl.GREEN : rl.GRAY)

			// --- HUD ---
			rl.DrawText(fmt.ctprintf("points %d   hull vertices %d", len(points), len(hull)), 12, 12, 20, rl.RAYWHITE)
			rl.DrawText(fmt.ctprintf("area %.0f    perimeter %.0f", area, perim), 12, 38, 18, rl.LIGHTGRAY)
			rl.DrawText(fmt.ctprintf("convex %v    clockwise (screen y-down) %v", convex, cw), 12, 60, 18, rl.LIGHTGRAY)
			rl.DrawText(fmt.ctprintf("bbox %.0f x %.0f  (area %.0f)   center %.0f, %.0f", bbsize.x, bbsize.y, bbarea, bbcenter.x, bbcenter.y), 12, 82, 18, rl.LIGHTGRAY)
			rl.DrawText(fmt.ctprintf("triangles %d   sum tri area %.0f  (polygon %.0f)", len(tris), tri_area, area), 12, 104, 18, rl.LIGHTGRAY)
			rl.DrawText(fmt.ctprintf("mouse inside %v   nearest edge dist %.1f   side %+.0f   edge angle %.2f", inside, best, sd, ea), 12, 126, 18, rl.LIGHTGRAY)
		} else {
			draw_points(points[:], dragging)
			rl.DrawText("add at least 3 points (left click)", 12, 12, 20, rl.RAYWHITE)
		}

		rl.DrawText("L add/drag   right-click delete   R scatter   N normals", 12, HEIGHT - 28, 18, rl.GRAY)
		rl.EndDrawing()

		free_all(context.temp_allocator)
	}

	rl.CloseWindow()
}

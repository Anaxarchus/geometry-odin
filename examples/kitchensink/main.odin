package main

// A "kitchen sink" touching several primitive packages at once: rect (panel
// layout via cut/grow, plus point queries), circle (containment + boundary
// projection), and line (project / distance / rotate). The mouse is a probe;
// each obstacle reports its nearest point and whether the probe is inside.

import rl "vendor:raylib"
import "core:fmt"
import "../../circle"
import "../../line"
import "../../rect"

WIDTH :: 1000
HEIGHT :: 720

v2 :: proc(p: [2]f32) -> rl.Vector2 {
	return rl.Vector2(p)
}

to_rec :: proc(r: [2][2]f32) -> rl.Rectangle {
	return {r[0].x, r[0].y, r[1].x - r[0].x, r[1].y - r[0].y}
}

main :: proc() {
	rl.InitWindow(WIDTH, HEIGHT, "geometry-odin: kitchen sink")
	rl.SetTargetFPS(60)

	// static obstacles
	circ_o := [2]f32{300, 430}
	circ_r := f32(95)
	box := [2][2]f32{{600, 330}, {840, 540}}

	for !rl.WindowShouldClose() {
		t := f32(rl.GetTime())
		mp := rl.GetMousePosition()
		p := [2]f32{mp.x, mp.y}

		// a slowly rotating segment "wall" (line.rotate about its center)
		wall_c := [2]f32{500, 210}
		wall := [2][2]f32{wall_c + {-170, 0}, wall_c + {170, 0}}
		wall = line.rotate(wall, t * 0.5)

		// rect layout: carve a header strip off an inset root
		root := rect.grow([2][2]f32{{0, 0}, {WIDTH, HEIGHT}}, [2]f32{-12, -12})
		header, _ := rect.cut(root, .Top, 44)

		// queries against each obstacle
		seg_proj := line.project(wall, p)
		seg_dist := line.distance(wall, p)

		in_circle := circle.has_point(p, circ_o, circ_r)
		circ_near := circle.project_to_boundary(p, circ_o, circ_r)

		in_box := rect.has_point(box, p)
		box_near := rect.project_to_boundary(box, p)

		rl.BeginDrawing()
		rl.ClearBackground({24, 26, 32, 255})

		// header panel
		hr := to_rec(header)
		rl.DrawRectangleRec(hr, {44, 50, 64, 255})
		rl.DrawRectangleLinesEx(hr, 2, rl.SKYBLUE)
		rl.DrawText(
			"kitchen sink: rect + circle + line  (move the mouse)",
			i32(header[0].x) + 10,
			i32(header[0].y) + 12,
			20,
			rl.RAYWHITE,
		)

		// circle obstacle + nearest boundary point
		rl.DrawCircleLines(
			i32(circ_o.x),
			i32(circ_o.y),
			circ_r,
			rl.GREEN if in_circle else rl.GRAY,
		)
		rl.DrawLineEx(v2(p), v2(circ_near), 1, rl.Fade(rl.GREEN, 0.5))
		rl.DrawCircleV(v2(circ_near), 4, rl.GREEN)

		// rect obstacle + nearest boundary point
		rl.DrawRectangleLinesEx(to_rec(box), 2, rl.ORANGE if in_box else rl.GRAY)
		rl.DrawLineEx(v2(p), v2(box_near), 1, rl.Fade(rl.ORANGE, 0.5))
		rl.DrawCircleV(v2(box_near), 4, rl.ORANGE)

		// line wall + nearest point on the segment
		rl.DrawLineEx(v2(wall[0]), v2(wall[1]), 3, rl.VIOLET)
		rl.DrawLineEx(v2(p), v2(seg_proj), 1, rl.Fade(rl.VIOLET, 0.6))
		rl.DrawCircleV(v2(seg_proj), 4, rl.VIOLET)

		// the probe
		rl.DrawCircleV(v2(p), 6, rl.YELLOW)

		// readouts
		y := i32(HEIGHT - 96)
		rl.DrawText(fmt.ctprintf("wall distance: %.1f px", seg_dist), 12, y, 18, rl.LIGHTGRAY)
		rl.DrawText(fmt.ctprintf("inside circle: %v", in_circle), 12, y + 24, 18, rl.LIGHTGRAY)
		rl.DrawText(fmt.ctprintf("inside rect:   %v", in_box), 12, y + 48, 18, rl.LIGHTGRAY)

		rl.EndDrawing()
		free_all(context.temp_allocator)
	}

	rl.CloseWindow()
}

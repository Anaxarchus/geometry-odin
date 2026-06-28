package main

// An interactive tour of the `line` segment API. A polyline path is drawn and
// the mouse is a query point. For the nearest segment we show line.project (the
// nearest point), line.distance, line.side (which half-plane the mouse is in)
// and line.has_point. Each segment draws its line.normal at its line.center. A
// spinning "blade" (line.rotate) reports its line.closest_distance to the path.

import rl "vendor:raylib"
import "core:fmt"
import "core:math"
import "../../line"

WIDTH :: 1000
HEIGHT :: 720

v2 :: proc(p: [2]f32) -> rl.Vector2 {
	return rl.Vector2(p)
}

main :: proc() {
	rl.InitWindow(WIDTH, HEIGHT, "geometry-odin: line API")
	rl.SetTargetFPS(60)

	// a polyline path, stored as individual segments
	pts := [][2]f32{{120, 240}, {300, 150}, {470, 330}, {640, 190}, {820, 370}}
	segs: [4][2][2]f32
	for i in 0 ..< len(segs) {
		segs[i] = {pts[i], pts[i + 1]}
	}

	for !rl.WindowShouldClose() {
		t := f32(rl.GetTime())
		mp := rl.GetMousePosition()
		p := [2]f32{mp.x, mp.y}

		// nearest path segment to the mouse
		nearest := 0
		best: f32 = math.F32_MAX
		for s, i in segs {
			d := line.distance(s, p)
			if d < best {
				best = d
				nearest = i
			}
		}
		ns := segs[nearest]
		proj := line.project(ns, p)
		side := line.side(ns, p)
		on_seg := line.has_point(ns, p, 6) // within 6 px of the segment

		// a spinning blade segment; report its closest approach to the path
		blade_c := [2]f32{500, 560}
		blade := [2][2]f32{blade_c + {-95, 0}, blade_c + {95, 0}}
		blade = line.rotate(blade, t)
		bc1, bc2 := line.closest_points(blade, ns)
		blade_gap := line.closest_distance(blade, ns)

		rl.BeginDrawing()
		rl.ClearBackground({22, 24, 30, 255})

		// every segment, plus its outward normal drawn from the center
		for s, i in segs {
			col := rl.SKYBLUE if i == nearest else rl.DARKGRAY
			rl.DrawLineEx(v2(s[0]), v2(s[1]), 3, col)
			c := line.center(s)
			n := line.normal(s)
			rl.DrawLineEx(v2(c), v2(c + n * 26), 2, rl.Fade(rl.GRAY, 0.7))
		}

		// mouse projection onto the nearest segment
		side_col := rl.LIME if side >= 0 else rl.ORANGE
		rl.DrawLineEx(v2(p), v2(proj), 1, rl.Fade(rl.RAYWHITE, 0.5))
		rl.DrawCircleV(v2(proj), 5, rl.SKYBLUE)
		rl.DrawCircleV(v2(p), 6, rl.YELLOW if on_seg else side_col)

		// spinning blade + closest-approach connector
		rl.DrawLineEx(v2(blade[0]), v2(blade[1]), 3, rl.VIOLET)
		rl.DrawCircleV(v2(line.center(blade)), 3, rl.VIOLET)
		rl.DrawLineEx(v2(bc1), v2(bc2), 1, rl.Fade(rl.RED, 0.8))
		rl.DrawCircleV(v2(bc1), 4, rl.RED)
		rl.DrawCircleV(v2(bc2), 4, rl.RED)

		// readouts for the nearest segment
		ang := line.angle(ns) * 180 / math.PI
		rl.DrawText("line API: nearest path segment in blue", 12, 12, 20, rl.RAYWHITE)
		rl.DrawText(fmt.ctprintf("distance to segment: %.1f px", best), 12, 42, 18, rl.LIGHTGRAY)
		rl.DrawText(
			fmt.ctprintf("segment length: %.1f   angle: %.1f deg", line.length(ns), ang),
			12,
			66,
			18,
			rl.LIGHTGRAY,
		)
		rl.DrawText(
			fmt.ctprintf("mouse side: %s (%.0f)", "left" if side >= 0 else "right", side),
			12,
			90,
			18,
			rl.LIGHTGRAY,
		)
		rl.DrawText(
			fmt.ctprintf("blade closest gap: %.1f px", blade_gap),
			12,
			114,
			18,
			rl.LIGHTGRAY,
		)
		rl.DrawText(
			"yellow dot = mouse lies on the segment (line.has_point)",
			12,
			HEIGHT - 30,
			16,
			rl.DARKGRAY,
		)

		rl.EndDrawing()
		free_all(context.temp_allocator)
	}

	rl.CloseWindow()
}

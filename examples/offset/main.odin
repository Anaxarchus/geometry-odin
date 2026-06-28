package main

// Polygon offsetting. A star is offset outward and inward by an animated
// distance; press 1/2/3 to switch the join style (miter / round / bevel).

import rl "vendor:raylib"
import "core:fmt"
import "core:math"
import "../../polygon"

WIDTH :: 960
HEIGHT :: 720

make_star :: proc(
	center: [2]f32,
	outer, inner: f32,
	points: int,
	allocator := context.allocator,
) -> [][2]f32 {
	n := points * 2
	out := make([][2]f32, n, allocator)
	for i in 0 ..< n {
		ang := f32(i) * (2 * math.PI / f32(n))
		r := outer if i % 2 == 0 else inner
		out[i] = center + [2]f32{math.cos(ang), math.sin(ang)} * r
	}
	return out
}

draw_outline :: proc(c: [][2]f32, thick: f32, color: rl.Color) {
	n := len(c)
	for i in 0 ..< n {
		rl.DrawLineEx(rl.Vector2(c[i]), rl.Vector2(c[(i + 1) % n]), thick, color)
	}
}

draw_contours :: proc(cs: [][][2]f32, color: rl.Color) {
	for c in cs {
		draw_outline(c, 2, color)
	}
}

main :: proc() {
	rl.InitWindow(WIDTH, HEIGHT, "geometry-odin: polygon offset")
	rl.SetTargetFPS(60)

	join := polygon.Polygon_Join_Type.Round
	join_name := "Round"

	for !rl.WindowShouldClose() {
		if rl.IsKeyPressed(.ONE) {join = .Miter;join_name = "Miter"}
		if rl.IsKeyPressed(.TWO) {join = .Round;join_name = "Round"}
		if rl.IsKeyPressed(.THREE) {join = .Bevel;join_name = "Bevel"}

		// oscillate the offset distance over time
		d := 10 + 35 * (0.5 + 0.5 * math.sin(f32(rl.GetTime())))

		star := make_star([2]f32{WIDTH / 2, HEIGHT / 2}, 130, 55, 6, context.temp_allocator)
		outward := polygon.offset(star, d, join, 2, 6, context.temp_allocator)
		inward := polygon.offset(star, -d, join, 2, 6, context.temp_allocator)

		rl.BeginDrawing()
		rl.ClearBackground(rl.RAYWHITE)

		draw_contours(outward, rl.SKYBLUE)
		draw_contours(inward, rl.ORANGE)
		draw_outline(star, 3, rl.DARKGRAY)

		rl.DrawText(fmt.ctprintf("Join: %v   offset: %.1f", join_name, d), 12, 12, 22, rl.DARKGRAY)
		rl.DrawText("press 1/2/3 for miter / round / bevel", 12, 40, 18, rl.GRAY)
		rl.DrawText("blue = outward   orange = inward", 12, 64, 18, rl.GRAY)
		rl.EndDrawing()

		free_all(context.temp_allocator)
	}

	rl.CloseWindow()
}

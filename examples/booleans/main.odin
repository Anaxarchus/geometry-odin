package main

// Polygon boolean operations. One disk is fixed, the other follows the mouse.
// Press 1-4 to switch operation. The result is filled via triangulation and
// outlined.

import rl "vendor:raylib"
import "core:fmt"
import "../../circle"
import "../../polygon"

WIDTH :: 960
HEIGHT :: 720

draw_outline :: proc(c: [][2]f32, color: rl.Color) {
	n := len(c)
	for i in 0 ..< n {
		a := c[i]
		b := c[(i + 1) % n]
		rl.DrawLineEx(rl.Vector2(a), rl.Vector2(b), 2, color)
	}
}

fill_contour :: proc(c: [][2]f32, color: rl.Color) {
	tris := polygon.triangulate(c, allocator = context.temp_allocator)
	for tri in tris {
		p0 := rl.Vector2(tri[0])
		p1 := rl.Vector2(tri[1])
		p2 := rl.Vector2(tri[2])
		// draw both windings so culling never hides a triangle
		rl.DrawTriangle(p0, p1, p2, color)
		rl.DrawTriangle(p0, p2, p1, color)
	}
}

main :: proc() {
	rl.InitWindow(WIDTH, HEIGHT, "geometry-odin: polygon booleans")
	rl.SetTargetFPS(60)

	op := 0
	names := []string{"Union", "Intersect", "Difference (A - B)", "XOR"}

	for !rl.WindowShouldClose() {
		if rl.IsKeyPressed(.ONE) {op = 0}
		if rl.IsKeyPressed(.TWO) {op = 1}
		if rl.IsKeyPressed(.THREE) {op = 2}
		if rl.IsKeyPressed(.FOUR) {op = 3}

		mouse := rl.GetMousePosition()

		a := circle.to_contour([2]f32{WIDTH / 2 - 70, HEIGHT / 2}, 150, 64, context.temp_allocator)
		b := circle.to_contour([2]f32{mouse.x, mouse.y}, 120, 64, context.temp_allocator)
		set := [][][2]f32{a, b}

		result: [][][2]f32
		switch op {
		case 0:
			result = polygon.union_polygon(set, context.temp_allocator)
		case 1:
			result = polygon.intersect(set, context.temp_allocator)
		case 2:
			result = polygon.difference(set, context.temp_allocator)
		case 3:
			result = polygon.xor(set, context.temp_allocator)
		}

		rl.BeginDrawing()
		rl.ClearBackground(rl.RAYWHITE)

		draw_outline(a, rl.LIGHTGRAY)
		draw_outline(b, rl.LIGHTGRAY)

		fill := rl.Fade(rl.SKYBLUE, 0.5)
		for c in result {
			fill_contour(c, fill)
			draw_outline(c, rl.DARKBLUE)
		}

		rl.DrawText(fmt.ctprintf("Operation: %v", names[op]), 12, 12, 22, rl.DARKGRAY)
		rl.DrawText("press 1-4 to switch  |  move the mouse", 12, 40, 18, rl.GRAY)
		rl.EndDrawing()

		free_all(context.temp_allocator)
	}

	rl.CloseWindow()
}

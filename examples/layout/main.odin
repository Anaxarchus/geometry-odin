package main

// A responsive dashboard built entirely from the rect partitioning API:
// rect_grow (inset), rect_cut (header/sidebar/footer), rect_divide (card grid),
// rect_stack (sidebar buttons). Resize the window to see it reflow.

import rl "vendor:raylib"
import "core:fmt"
import "../../rect"

to_rec :: proc(r: [2][2]f32) -> rl.Rectangle {
	return {r[0].x, r[0].y, r[1].x - r[0].x, r[1].y - r[0].y}
}

panel :: proc(r: [2][2]f32, fill, border: rl.Color, label: cstring) {
	rec := to_rec(r)
	rl.DrawRectangleRec(rec, fill)
	rl.DrawRectangleLinesEx(rec, 2, border)
	rl.DrawText(label, i32(r[0].x) + 10, i32(r[0].y) + 8, 18, border)
}

main :: proc() {
	rl.SetConfigFlags({.WINDOW_RESIZABLE})
	rl.InitWindow(1000, 720, "geometry-odin: rect layout")
	rl.SetTargetFPS(60)

	for !rl.WindowShouldClose() {
		root := [2][2]f32{{0, 0}, {f32(rl.GetScreenWidth()), f32(rl.GetScreenHeight())}}
		root = rect.rect_grow(root, [2]f32{-12, -12}) // inset

		header, rest := rect.rect_cut(root, .Top, 56)
		footer, mid := rect.rect_cut(rest, .Bottom, 36)
		sidebar, content := rect.rect_cut(mid, .Left, 210)

		// shave a small gap around the inner regions
		content = rect.rect_grow(content, [2]f32{-6, -6})
		sidebar = rect.rect_grow(sidebar, [2]f32{-6, -6})

		rl.BeginDrawing()
		rl.ClearBackground({28, 30, 38, 255})

		panel(header, {52, 58, 74, 255}, rl.SKYBLUE, "Header")
		panel(footer, {52, 58, 74, 255}, rl.GRAY, "Footer / status bar")

		// sidebar: a stack of fixed-height buttons
		buttons, _, _ := rect.rect_stack_y(sidebar, 6, 44, 8, .Begin, context.temp_allocator)
		panel(sidebar, {40, 44, 56, 255}, rl.DARKGRAY, "")
		for btn, i in buttons {
			panel(btn, {66, 72, 92, 255}, rl.LIGHTGRAY, fmt.ctprintf("Nav %d", i + 1))
		}

		// content: a 3 x 2 card grid
		cols := rect.rect_divide_x(content, 3, 10, context.temp_allocator)
		for col in cols {
			cells := rect.rect_divide_y(col, 2, 10, context.temp_allocator)
			for cell in cells {
				panel(cell, {44, 48, 60, 255}, rl.SKYBLUE, "Card")
			}
		}

		rl.EndDrawing()
		free_all(context.temp_allocator)
	}

	rl.CloseWindow()
}

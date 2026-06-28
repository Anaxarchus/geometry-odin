package main

import "core:math"
import "core:math/linalg"
import "core:fmt"
import rl "vendor:raylib"
import "../../mesh"
import "../../rect"

WIDTH  :: 1280
HEIGHT :: 720

Edit_Mode :: enum {
	Vertex,
	Edge,
	Face,
}

App_State :: struct {
	canvas:      mesh.Mesh,
	rl_mesh:     rl.Mesh,
	rl_model:    rl.Model,
	is_dirty:    bool,

	// --- Selection State ---
	active_mode: Edit_Mode,

	// --- Arc Rotate Camera Properties ---
	cam_target:  [3]f32,
	cam_radius:  f32,
	cam_yaw:     f32, 
	cam_pitch:   f32,
}

sync_mesh_to_raylib :: proc(state: ^App_State) {
	if state.rl_mesh.vboId != nil {
		rl.UnloadMesh(state.rl_mesh)
	}

	baked := mesh.bake(state.canvas, context.temp_allocator)

	state.rl_mesh = {}
	state.rl_mesh.vertexCount   = i32(len(baked.vertices))
	state.rl_mesh.triangleCount = i32(len(baked.indices) / 3)

	v_size := size_of(f32) * 3 * len(baked.vertices)
	n_size := size_of(f32) * 3 * len(baked.vertices)
	c_size := size_of(u8)  * 4 * len(baked.vertices)
	i_size := size_of(u16) * len(baked.indices)

	state.rl_mesh.vertices = (^f32)(rl.MemAlloc(u32(v_size)))
	state.rl_mesh.normals  = (^f32)(rl.MemAlloc(u32(n_size)))
	state.rl_mesh.colors   = (^u8)(rl.MemAlloc(u32(c_size)))
	state.rl_mesh.indices  = (^u16)(rl.MemAlloc(u32(i_size)))

	out_vertices := ([^][3]f32)(state.rl_mesh.vertices)[:len(baked.vertices)]
	out_normals  := ([^][3]f32)(state.rl_mesh.normals)[:len(baked.vertices)]
	out_colors   := ([^][4]u8)(state.rl_mesh.colors)[:len(baked.vertices)]
	out_indices  := ([^]u16)(state.rl_mesh.indices)[:len(baked.indices)]

	light_dir  := linalg.normalize([3]f32{0.3, 1.0, 0.4})
	base_color := [3]f32{0.75, 0.77, 0.82}

	for i in 0..<len(baked.vertices) {
		out_vertices[i] = baked.vertices[i].position
		out_normals[i]  = baked.vertices[i].normal

		dot := linalg.dot(baked.vertices[i].normal, light_dir)
		intensity := clamp((dot * 0.35) + 0.6, 0.25, 0.95)

		out_colors[i] = [4]u8{
			u8(base_color.x * intensity * 255.0),
			u8(base_color.y * intensity * 255.0),
			u8(base_color.z * intensity * 255.0),
			255,
		}
	}
	
	for idx, i in baked.indices {
		out_indices[i] = u16(idx)
	}

	rl.UploadMesh(&state.rl_mesh, false)
	state.rl_model.meshes[0] = state.rl_mesh
	state.is_dirty = false
}

update_camera_orbit :: proc(state: ^App_State, camera: ^rl.Camera3D, bounds_3d: [2][2]int) {
	// Only parse camera panning/orbit loops if mouse is hovering inside the 3D viewport region
	m_pos := rl.GetMousePosition()
	p := [2]int{int(m_pos.x), int(m_pos.y)}
	
	if rect.has_point(bounds_3d, p) {
		if rl.IsMouseButtonDown(.RIGHT) {
			mouse_delta := rl.GetMouseDelta()
			state.cam_yaw   -= mouse_delta.x * 0.005
			state.cam_pitch += mouse_delta.y * 0.005
			state.cam_pitch = clamp(state.cam_pitch, math.to_radians(f32(-89.0)), math.to_radians(f32(89.0)))
		}

		wheel := rl.GetMouseWheelMove()
		if wheel != 0 {
			state.cam_radius -= wheel * 0.5
			if state.cam_radius < 1.0 do state.cam_radius = 1.0
		}
	}

	offset: [3]f32
	offset.x = state.cam_radius * math.cos(state.cam_pitch) * math.sin(state.cam_yaw)
	offset.y = state.cam_radius * math.sin(state.cam_pitch)
	offset.z = state.cam_radius * math.cos(state.cam_pitch) * math.cos(state.cam_yaw)

	camera.target   = state.cam_target
	camera.position = state.cam_target + offset
}

// Immediate-mode UI button helper mapped directly to our layout structures
draw_ui_button :: proc(box: [2][2]int, text: string, is_active: bool) -> bool {
	rl_box := rl.Rectangle{f32(box[0].x), f32(box[0].y), f32(box[1].x - box[0].x), f32(box[1].y - box[0].y)}
	m_pos := rl.GetMousePosition()
	is_hovered := rect.has_point(box, [2]int{int(m_pos.x), int(m_pos.y)})

	// Determine palette states based on execution context
	bg_color := rl.Color{45, 45, 48, 255}
	if is_active     do bg_color = rl.Color{0, 122, 204, 255} // Active blue
	else if is_hovered do bg_color = rl.Color{62, 62, 66, 255}

	rl.DrawRectangleRec(rl_box, bg_color)
	rl.DrawRectangleLines(i32(rl_box.x), i32(rl_box.y), i32(rl_box.width), i32(rl_box.height), rl.Color{30, 30, 30, 255})

	c_str := fmt.ctprintf("%s", text)
	font_size := i32(14)
	text_w := rl.MeasureText(c_str, font_size)
	
	tx := i32(rl_box.x) + (i32(rl_box.width) - text_w) / 2
	ty := i32(rl_box.y) + (i32(rl_box.height) - font_size) / 2
	rl.DrawText(c_str, tx, ty, font_size, rl.WHITE)

	return is_hovered && rl.IsMouseButtonPressed(.LEFT)
}

main :: proc() {
	rl.InitWindow(WIDTH, HEIGHT, "geometry-odin: 3D Modeling Studio")
	rl.SetTargetFPS(60)

	state: App_State
	state.cam_target  = {0.0, 0.0, 0.0}
	state.cam_radius  = 6.0
	state.cam_yaw     = math.to_radians(f32(45.0))
	state.cam_pitch   = math.to_radians(f32(30.0))
	state.active_mode = .Face

	// Primary mesh construction loop
	state.canvas   = mesh.from_box({2.0, 2.0, 2.0})
	state.rl_model = rl.LoadModelFromMesh(state.rl_mesh)
	state.is_dirty = true

	camera := rl.Camera3D{
		up         = {0.0, 1.0, 0.0},
		fovy       = 45.0,
		projection = .PERSPECTIVE,
	}

	for !rl.WindowShouldClose() {
		// --- Immediate Mode UI Layout Hierarchy ---
		window_rect := [2][2]int{{0, 0}, {WIDTH, HEIGHT}}

		// Slice layout into Main Work Area and Sidebar
		sidebar_box, main_workspace := rect.cut(window_rect, .Left, 240)
		
		// Divide Workspace into a Top Control Bar and the 3D Viewport
		top_bar_box, viewport_box   := rect.cut(main_workspace, .Top, 48)

		// Process camera positioning vectors limited by viewport bounds
		update_camera_orbit(&state, &camera, viewport_box)

		if state.is_dirty {
			sync_mesh_to_raylib(&state)
			free_all(context.temp_allocator)
		}

		rl.BeginDrawing()
		rl.ClearBackground(rl.Color{28, 28, 28, 255}) // Dark Studio Canvas Background

		// 1. --- Render 3D Viewport Region ---
		// Set scissor boundaries to keep rendering clean and confined
		rl.BeginScissorMode(i32(viewport_box[0].x), i32(viewport_box[0].y), i32(viewport_box[1].x - viewport_box[0].x), i32(viewport_box[1].y - viewport_box[0].y))
			rl.BeginMode3D(camera)
				rl.DrawModel(state.rl_model, {0,0,0}, 1.0, rl.WHITE)
				
				// Dynamically tint our edge wireframe configuration colors to align with our edit mode selection
				wire_color := rl.Color{50, 200, 50, 255} // Default face/edge green overlay
				if state.active_mode == .Vertex do wire_color = rl.Color{230, 140, 10, 255} // Orange vertices emphasis
				
				rl.DrawModelWires(state.rl_model, {0,0,0}, 1.0, wire_color)
				rl.DrawGrid(10, 1.0)
			rl.EndMode3D()
		rl.EndScissorMode()

		// 2. --- Render Sidebar Options Container ---
		rl.DrawRectangle(i32(sidebar_box[0].x), i32(sidebar_box[0].y), i32(sidebar_box[1].x - sidebar_box[0].x), i32(sidebar_box[1].y - sidebar_box[0].y), rl.Color{37, 37, 38, 255})
		rl.DrawLine(i32(sidebar_box[1].x), 0, i32(sidebar_box[1].x), HEIGHT, rl.Color{45, 45, 48, 255})
		
		// Cut out inner margins inside sidebar to organize button groupings
		sidebar_inner, _ := rect.cut(sidebar_box, .Top, 16)
		sidebar_inner, _  = rect.cut(sidebar_inner, .Left, 12)
		sidebar_inner, _  = rect.cut(sidebar_inner, .Right, 12)

		rl.DrawText("PRIMITIVES", i32(sidebar_inner[0].x), i32(sidebar_inner[0].y), 12, rl.Color{150, 150, 150, 255})
		
		// Stack primitive creation triggers vertically
		_, prim_stack := rect.cut(sidebar_inner, .Top, 24)
		prim_buttons, _, _ := rect.stack_y(prim_stack, 3, 36, 8)

		if draw_ui_button(prim_buttons[0], "Reset to Cube", false) {
			mesh.destroy_mesh(&state.canvas)
			state.canvas = mesh.from_box({2.0, 2.0, 2.0})
			state.is_dirty = true
		}
		if draw_ui_button(prim_buttons[1], "Spawn Sphere (UI)", false) {
			// Stub placeholder callback
		}
		if draw_ui_button(prim_buttons[2], "Spawn Capsule (UI)", false) {
			// Stub placeholder callback
		}

		// 3. --- Render Top Horizontal Toolbar Container ---
		rl.DrawRectangle(i32(top_bar_box[0].x), i32(top_bar_box[0].y), i32(top_bar_box[1].x - top_bar_box[0].x), i32(top_bar_box[1].y - top_bar_box[0].y), rl.Color{30, 30, 30, 255})
		rl.DrawLine(i32(top_bar_box[0].x), i32(top_bar_box[1].y), i32(top_bar_box[1].x), i32(top_bar_box[1].y), rl.Color{45, 45, 48, 255})

		// Use stack_x horizontally inside the toolbar to space select modes out
		_, toolbar_inner := rect.cut(top_bar_box, .Left, 16)
		mode_stack, _, _ := rect.stack_x(toolbar_inner, 3, 110, 6, .Begin)

		if draw_ui_button(mode_stack[0], "Vertex Mode", state.active_mode == .Vertex) do state.active_mode = .Vertex
		if draw_ui_button(mode_stack[1], "Edge Mode",   state.active_mode == .Edge)   do state.active_mode = .Edge
		if draw_ui_button(mode_stack[2], "Face Mode",   state.active_mode == .Face)   do state.active_mode = .Face

		// Print context diagnostics at bottom corner bounds
		rl.DrawFPS(WIDTH - 80, 14)
		rl.DrawText("Hold RMB to Orbit Canvas. Scroll Wheel to zoom viewports.", i32(main_workspace[0].x) + 20, HEIGHT - 28, 14, rl.Color{130, 130, 130, 255})

		rl.EndDrawing()
	}

	rl.UnloadModel(state.rl_model)
	if state.rl_mesh.vboId != nil {
		rl.UnloadMesh(state.rl_mesh)
	}
	mesh.destroy_mesh(&state.canvas)
	rl.CloseWindow()
}
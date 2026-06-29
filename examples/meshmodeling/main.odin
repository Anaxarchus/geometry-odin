package main

import "core:math"
import "core:math/linalg"
import "core:fmt"
import rl "vendor:raylib"
import "../../mesh"
import "../../rect"

WIDTH  :: 1280
HEIGHT :: 720

INSET_DISTANCE   :: 0.3
EXTRUDE_DISTANCE :: 0.6
GIZMO_LEN        :: f32(1.3)
GIZMO_PICK_PX    :: f32(12.0)

Edit_Mode :: enum {
	Vertex,
	Edge,
	Face,
}

Selection :: struct {
	active: bool,
	index:  int, // vertex / half-edge / face index, interpreted per active_mode
}

App_State :: struct {
	canvas:      mesh.Mesh,
	rl_mesh:     rl.Mesh,
	rl_model:    rl.Model,
	is_dirty:    bool,

	// --- Selection State ---
	active_mode: Edit_Mode,
	prev_mode:   Edit_Mode,
	selection:   Selection,

	// --- Gizmo Drag State ---
	dragging:    bool,
	drag_axis:   int,      // 0 = X, 1 = Y, 2 = Z
	drag_origin: rl.Vector3,
	drag_dir:    rl.Vector3,
	drag_s:      f32,      // last parameter along the drag axis

	// --- Arc Rotate Camera Properties ---
	cam_target:  [3]f32,
	cam_radius:  f32,
	cam_yaw:     f32,
	cam_pitch:   f32,
}

v3 :: #force_inline proc(p: [3]f64) -> rl.Vector3 {
	return {f32(p.x), f32(p.y), f32(p.z)}
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

// --- Selection helpers ----------------------------------------------------

selection_centroid :: proc(state: ^App_State) -> (rl.Vector3, bool) {
	if !state.selection.active do return {}, false
	idx := state.selection.index
	switch state.active_mode {
	case .Vertex:
		return v3(state.canvas.vertices[idx].position), true
	case .Edge:
		v0, v1 := mesh.edge_endpoints(&state.canvas, idx)
		mid := (state.canvas.vertices[v0].position + state.canvas.vertices[v1].position) * 0.5
		return v3(mid), true
	case .Face:
		return v3(mesh.face_centroid(&state.canvas, idx)), true
	}
	return {}, false
}

mouse_ray_f64 :: proc(camera: rl.Camera3D) -> (origin, dir: [3]f64) {
	ray := rl.GetScreenToWorldRay(rl.GetMousePosition(), camera)
	origin = {f64(ray.position.x), f64(ray.position.y), f64(ray.position.z)}
	dir    = {f64(ray.direction.x), f64(ray.direction.y), f64(ray.direction.z)}
	return
}

pick_element :: proc(state: ^App_State, camera: rl.Camera3D) {
	origin, dir := mouse_ray_f64(camera)
	switch state.active_mode {
	case .Vertex:
		if idx, ok := mesh.ray_pick_vertex(&state.canvas, origin, dir); ok {
			state.selection = {active = true, index = idx}
		}
	case .Edge:
		if idx, ok := mesh.ray_pick_edge(&state.canvas, origin, dir); ok {
			state.selection = {active = true, index = idx}
		}
	case .Face:
		if idx, _, ok := mesh.ray_pick_face(&state.canvas, origin, dir); ok {
			state.selection = {active = true, index = idx}
		}
	}
}

// --- Gizmo math -----------------------------------------------------------

dist_point_seg2d :: proc(p, a, b: rl.Vector2) -> f32 {
	ab := b - a
	ap := p - a
	denom := linalg.dot(ab, ab)
	t := denom > 1e-6 ? clamp(linalg.dot(ap, ab) / denom, 0, 1) : 0
	closest := a + ab * t
	return linalg.length(p - closest)
}

axis_dir :: proc(i: int) -> rl.Vector3 {
	switch i {
	case 0: return {1, 0, 0}
	case 1: return {0, 1, 0}
	case 2: return {0, 0, 1}
	}
	return {}
}

// Returns the gizmo axis (0/1/2) nearest the mouse in screen space, or -1.
pick_gizmo_axis :: proc(c: rl.Vector3, camera: rl.Camera3D, mouse: rl.Vector2) -> int {
	c2 := rl.GetWorldToScreen(c, camera)
	best := -1
	best_d := GIZMO_PICK_PX
	for i in 0..<3 {
		tip := rl.GetWorldToScreen(c + axis_dir(i) * GIZMO_LEN, camera)
		d := dist_point_seg2d(mouse, c2, tip)
		if d < best_d {
			best_d = d
			best = i
		}
	}
	return best
}

// Parameter along the axis line (origin + dir*s) of the point closest to the ray.
closest_axis_s :: proc(ray: rl.Ray, origin, dir: rl.Vector3) -> f32 {
	u := dir
	v := ray.direction
	w0 := origin - ray.position
	a := linalg.dot(u, u)
	b := linalg.dot(u, v)
	c := linalg.dot(v, v)
	d := linalg.dot(u, w0)
	e := linalg.dot(v, w0)
	denom := a * c - b * b
	if abs(denom) < 1e-6 do return 0
	return (b * e - c * d) / denom
}

apply_translation :: proc(state: ^App_State, delta: rl.Vector3) {
	dv := [3]f64{f64(delta.x), f64(delta.y), f64(delta.z)}
	idx := state.selection.index
	switch state.active_mode {
	case .Vertex: mesh.translate_vertex(&state.canvas, idx, dv)
	case .Edge:   mesh.translate_edge(&state.canvas, idx, dv)
	case .Face:   mesh.translate_face(&state.canvas, idx, dv)
	}
	state.is_dirty = true
}

// --- Selection rendering --------------------------------------------------

draw_selection :: proc(state: ^App_State) {
	if !state.selection.active do return
	idx := state.selection.index
	hl := rl.Color{255, 200, 40, 255}

	switch state.active_mode {
	case .Vertex:
		rl.DrawSphere(v3(state.canvas.vertices[idx].position), 0.09, hl)
	case .Edge:
		v0, v1 := mesh.edge_endpoints(&state.canvas, idx)
		rl.DrawCylinderEx(v3(state.canvas.vertices[v0].position), v3(state.canvas.vertices[v1].position), 0.03, 0.03, 8, hl)
	case .Face:
		verts := mesh.face_vertices(&state.canvas, idx, context.temp_allocator)
		fill := rl.Color{255, 200, 40, 90}
		// Translucent fan fill (drawn both windings so it shows from either side)
		for i in 1..<len(verts) - 1 {
			p0 := v3(state.canvas.vertices[verts[0]].position)
			p1 := v3(state.canvas.vertices[verts[i]].position)
			p2 := v3(state.canvas.vertices[verts[i + 1]].position)
			rl.DrawTriangle3D(p0, p1, p2, fill)
			rl.DrawTriangle3D(p0, p2, p1, fill)
		}
		// Bright boundary outline
		for i in 0..<len(verts) {
			a := v3(state.canvas.vertices[verts[i]].position)
			b := v3(state.canvas.vertices[verts[(i + 1) % len(verts)]].position)
			rl.DrawLine3D(a, b, hl)
		}
	}
}

draw_gizmo :: proc(state: ^App_State, c: rl.Vector3) {
	colors := [3]rl.Color{{220, 70, 70, 255}, {70, 210, 90, 255}, {70, 130, 230, 255}}
	for i in 0..<3 {
		tip := c + axis_dir(i) * GIZMO_LEN
		r: f32 = (state.dragging && state.drag_axis == i) ? 0.05 : 0.025
		rl.DrawCylinderEx(c, tip, r, r, 8, colors[i])
		rl.DrawSphere(tip, 0.07, colors[i])
	}
}

// Immediate-mode UI button helper mapped directly to our layout structures
draw_ui_button :: proc(box: [2][2]int, text: string, is_active: bool, enabled := true) -> bool {
	rl_box := rl.Rectangle{f32(box[0].x), f32(box[0].y), f32(box[1].x - box[0].x), f32(box[1].y - box[0].y)}
	m_pos := rl.GetMousePosition()
	is_hovered := enabled && rect.has_point(box, [2]int{int(m_pos.x), int(m_pos.y)})

	// Determine palette states based on execution context
	bg_color := rl.Color{45, 45, 48, 255}
	if !enabled        do bg_color = rl.Color{34, 34, 36, 255}
	else if is_active  do bg_color = rl.Color{0, 122, 204, 255} // Active blue
	else if is_hovered do bg_color = rl.Color{62, 62, 66, 255}

	rl.DrawRectangleRec(rl_box, bg_color)
	rl.DrawRectangleLines(i32(rl_box.x), i32(rl_box.y), i32(rl_box.width), i32(rl_box.height), rl.Color{30, 30, 30, 255})

	c_str := fmt.ctprintf("%s", text)
	font_size := i32(14)
	text_w := rl.MeasureText(c_str, font_size)

	tx := i32(rl_box.x) + (i32(rl_box.width) - text_w) / 2
	ty := i32(rl_box.y) + (i32(rl_box.height) - font_size) / 2
	text_col := enabled ? rl.WHITE : rl.Color{110, 110, 110, 255}
	rl.DrawText(c_str, tx, ty, font_size, text_col)

	return enabled && is_hovered && rl.IsMouseButtonPressed(.LEFT)
}

main :: proc() {
	rl.InitWindow(WIDTH, HEIGHT, "geometry-odin: 3D Modeling")
	rl.SetTargetFPS(60)

	state: App_State
	state.cam_target  = {0.0, 0.0, 0.0}
	state.cam_radius  = 6.0
	state.cam_yaw     = math.to_radians(f32(45.0))
	state.cam_pitch   = math.to_radians(f32(30.0))
	state.active_mode = .Face
	state.prev_mode   = .Face

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

		// Clearing selection when the edit mode changes (indices are mode-specific)
		if state.active_mode != state.prev_mode {
			state.selection.active = false
			state.dragging = false
			state.prev_mode = state.active_mode
		}

		// --- Viewport interaction (select / gizmo drag) ---
		m_pos := rl.GetMousePosition()
		in_viewport := rect.has_point(viewport_box, [2]int{int(m_pos.x), int(m_pos.y)})

		if rl.IsMouseButtonPressed(.LEFT) && in_viewport && !rl.IsMouseButtonDown(.RIGHT) {
			handled := false
			if c, ok := selection_centroid(&state); ok {
				axis := pick_gizmo_axis(c, camera, m_pos)
				if axis >= 0 {
					ray := rl.GetScreenToWorldRay(m_pos, camera)
					state.dragging    = true
					state.drag_axis   = axis
					state.drag_origin = c
					state.drag_dir    = axis_dir(axis)
					state.drag_s      = closest_axis_s(ray, c, axis_dir(axis))
					handled = true
				}
			}
			if !handled {
				pick_element(&state, camera)
			}
		}

		if state.dragging {
			if rl.IsMouseButtonDown(.LEFT) {
				ray := rl.GetScreenToWorldRay(m_pos, camera)
				s := closest_axis_s(ray, state.drag_origin, state.drag_dir)
				delta := state.drag_dir * (s - state.drag_s)
				state.drag_s = s
				if delta.x != 0 || delta.y != 0 || delta.z != 0 {
					apply_translation(&state, delta)
				}
			} else {
				state.dragging = false
			}
		}

		if state.is_dirty {
			sync_mesh_to_raylib(&state)
		}

		rl.BeginDrawing()
		rl.ClearBackground(rl.Color{28, 28, 28, 255}) // Dark Studio Canvas Background

		// 1. --- Render 3D Viewport Region ---
		rl.BeginScissorMode(i32(viewport_box[0].x), i32(viewport_box[0].y), i32(viewport_box[1].x - viewport_box[0].x), i32(viewport_box[1].y - viewport_box[0].y))
			rl.BeginMode3D(camera)
				rl.DrawModel(state.rl_model, {0,0,0}, 1.0, rl.WHITE)

				// Dynamically tint our edge wireframe configuration colors to align with our edit mode selection
				wire_color := rl.Color{50, 200, 50, 255} // Default face/edge green overlay
				if state.active_mode == .Vertex do wire_color = rl.Color{230, 140, 10, 255} // Orange vertices emphasis

				rl.DrawModelWires(state.rl_model, {0,0,0}, 1.0, wire_color)
				rl.DrawGrid(10, 1.0)

				draw_selection(&state)
				if c, ok := selection_centroid(&state); ok {
					draw_gizmo(&state, c)
				}
			rl.EndMode3D()
		rl.EndScissorMode()

		// 2. --- Render Sidebar Options Container ---
		rl.DrawRectangle(i32(sidebar_box[0].x), i32(sidebar_box[0].y), i32(sidebar_box[1].x - sidebar_box[0].x), i32(sidebar_box[1].y - sidebar_box[0].y), rl.Color{37, 37, 38, 255})
		rl.DrawLine(i32(sidebar_box[1].x), 0, i32(sidebar_box[1].x), HEIGHT, rl.Color{45, 45, 48, 255})

		// Cut inner margins inside sidebar (use remainders) to organize groupings
		_, sidebar_inner := rect.cut(sidebar_box, .Top, 16)   // drop top margin
		_, sidebar_inner  = rect.cut(sidebar_inner, .Left, 12) // drop left margin
		_, sidebar_inner  = rect.cut(sidebar_inner, .Right, 12) // drop right margin (keep the left remainder)

		rl.DrawText("PRIMITIVES", i32(sidebar_inner[0].x), i32(sidebar_inner[0].y), 12, rl.Color{150, 150, 150, 255})

		// Stack primitive creation triggers vertically
		_, prim_stack := rect.cut(sidebar_inner, .Top, 24)
		prim_buttons, _, prim_rest := rect.stack_y(prim_stack, 1, 36, 8, allocator = context.temp_allocator)

		if draw_ui_button(prim_buttons[0], "Reset to Cube", false) {
			mesh.destroy_mesh(&state.canvas)
			state.canvas = mesh.from_box({2.0, 2.0, 2.0})
			state.selection.active = false
			state.dragging = false
			state.is_dirty = true
		}

		// --- Operations section (context-sensitive on the active mode) ---
		_, ops_area := rect.cut(prim_rest, .Top, 24)
		rl.DrawText("OPERATIONS", i32(ops_area[0].x), i32(ops_area[0].y), 12, rl.Color{150, 150, 150, 255})
		_, ops_stack := rect.cut(ops_area, .Top, 24)
		op_buttons, _, _ := rect.stack_y(ops_stack, 4, 36, 8, allocator = context.temp_allocator)

		has_sel := state.selection.active

		#partial switch state.active_mode {
		case .Face:
			if draw_ui_button(op_buttons[0], "Subdivide Face", false, has_sel) {
				mesh.subdivide_face(&state.canvas, state.selection.index)
				state.selection.active = false
				state.is_dirty = true
			}
			if draw_ui_button(op_buttons[1], "Inset Face", false, has_sel) {
				mesh.inset_face(&state.canvas, state.selection.index, INSET_DISTANCE)
				state.selection.active = false
				state.is_dirty = true
			}
			if draw_ui_button(op_buttons[2], "Extrude Face", false, has_sel) {
				n := mesh.face_normal(&state.canvas, state.selection.index)
				mesh.extrude_face(&state.canvas, state.selection.index, n * EXTRUDE_DISTANCE)
				state.selection.active = false
				state.is_dirty = true
			}
		case .Edge:
			if draw_ui_button(op_buttons[0], "Subdivide Edge", false, has_sel) {
				mesh.subdivide_edge(&state.canvas, state.selection.index)
				state.selection.active = false
				state.is_dirty = true
			}
		case .Vertex:
			rl.DrawText("Drag the gizmo to move.", i32(ops_area[0].x), i32(op_buttons[0][0].y) + 8, 12, rl.Color{120, 120, 120, 255})
		}

		// 3. --- Render Top Horizontal Toolbar Container ---
		rl.DrawRectangle(i32(top_bar_box[0].x), i32(top_bar_box[0].y), i32(top_bar_box[1].x - top_bar_box[0].x), i32(top_bar_box[1].y - top_bar_box[0].y), rl.Color{30, 30, 30, 255})
		rl.DrawLine(i32(top_bar_box[0].x), i32(top_bar_box[1].y), i32(top_bar_box[1].x), i32(top_bar_box[1].y), rl.Color{45, 45, 48, 255})

		// Use stack_x horizontally inside the toolbar to space select modes out
		_, toolbar_inner := rect.cut(top_bar_box, .Left, 16)
		mode_stack, _, _ := rect.stack_x(toolbar_inner, 3, 110, 6, .Begin, context.temp_allocator)

		if draw_ui_button(mode_stack[0], "Vertex Mode", state.active_mode == .Vertex) do state.active_mode = .Vertex
		if draw_ui_button(mode_stack[1], "Edge Mode",   state.active_mode == .Edge)   do state.active_mode = .Edge
		if draw_ui_button(mode_stack[2], "Face Mode",   state.active_mode == .Face)   do state.active_mode = .Face

		// Print context diagnostics at bottom corner bounds
		rl.DrawFPS(WIDTH - 80, 14)
		rl.DrawText("LMB select / drag gizmo. Hold RMB to orbit. Scroll to zoom.", i32(main_workspace[0].x) + 20, HEIGHT - 28, 14, rl.Color{130, 130, 130, 255})

		rl.EndDrawing()

		free_all(context.temp_allocator)
	}

	rl.UnloadModel(state.rl_model)
	if state.rl_mesh.vboId != nil {
		rl.UnloadMesh(state.rl_mesh)
	}
	mesh.destroy_mesh(&state.canvas)
	rl.CloseWindow()
}

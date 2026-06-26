package libtess2

// High-level, Odin-native wrapper around the libtess2 port (package
// libtess2_port). This mirrors the ergonomic binding API that polygon.odin was
// originally written against (begin/add/end + tesselate_*), but is backed
// entirely by the pure-Odin port rather than the C library.

import lt2 "src"

// Winding rules. Alias of the port's enum, so values pass straight through to
// the tesselator without conversion.
Winding_Rule :: lt2.WindingRule

// A tessellation context. Owns an underlying tesselator handle.
Context :: struct {
	handle:      ^lt2.Tesselator,
	vertex_size: int,
}

// begin creates a new tessellation context. `reverse_contours` flips the
// orientation of every added contour (maps to TESS_REVERSE_CONTOURS).
begin :: proc(vertex_size := 2, reverse_contours := false) -> (ctx: Context, ok: bool) {
	handle := lt2.tessNewTess(nil)
	if handle == nil {
		return {}, false
	}
	if reverse_contours {
		lt2.tessSetOption(handle, .Reverse_Contours, 1)
	}
	ctx.handle      = handle
	ctx.vertex_size = vertex_size
	return ctx, true
}

// add appends a 2D contour to the context. Returns false if the tesselator is
// in an error state (e.g. out of memory or invalid input).
add :: proc(ctx: Context, contour: [][2]f64) -> bool {
	if ctx.handle == nil {
		return false
	}
	lt2.tessAddContour(ctx.handle, 2, raw_data(contour), size_of([2]f64), i32(len(contour)))
	return ctx.handle.status == .Ok
}

// end releases the context's resources.
end :: proc(ctx: Context) {
	if ctx.handle != nil {
		lt2.tessDeleteTess(ctx.handle)
	}
}

// tesselate_boundary_contours runs the tessellation in boundary-extraction mode
// and returns the resulting closed contours. The caller owns the result and
// should free it with delete_contours.
tesselate_boundary_contours :: proc(ctx: ^Context, winding: Winding_Rule, allocator := context.allocator) -> [][][2]f64 {
	if ctx == nil || ctx.handle == nil {
		return nil
	}
	h := ctx.handle
	ok := lt2.tessTesselate(h, winding, .Boundary_Contours, 0, i32(ctx.vertex_size), nil)
	if !ok {
		return nil
	}

	ec := int(h.elementCount)
	if ec == 0 {
		return nil
	}
	verts := h.vertices  // [count*2] f64
	elems := h.elements  // [ec*2] Index: (startVertex, vertexCount) per contour

	result := make([][][2]f64, ec, allocator)
	for i in 0 ..< ec {
		start := int(elems[i*2])
		count := int(elems[i*2 + 1])
		contour := make([][2]f64, count, allocator)
		for j in 0 ..< count {
			vi := (start + j) * 2
			contour[j] = {verts[vi], verts[vi + 1]}
		}
		result[i] = contour
	}
	return result
}

// tesselate_polygons triangulates the added contours and returns a flat list of
// triangles. poly_size is forwarded to the tesselator; only triangles
// (poly_size == 3) are returned here. Any unused/degenerate slot is zeroed.
tesselate_polygons :: proc(ctx: ^Context, winding: Winding_Rule, poly_size := 3, allocator := context.allocator) -> [][3][2]f64 {
	if ctx == nil || ctx.handle == nil {
		return nil
	}
	h := ctx.handle
	ok := lt2.tessTesselate(h, winding, .Polygons, i32(poly_size), i32(ctx.vertex_size), nil)
	if !ok {
		return nil
	}

	ec := int(h.elementCount)
	if ec == 0 {
		return nil
	}
	verts := h.vertices
	elems := h.elements

	result := make([][3][2]f64, ec, allocator)
	for i in 0 ..< ec {
		tri: [3][2]f64
		for j in 0 ..< 3 {
			idx := int(elems[i*poly_size + j])
			if idx == int(lt2.TESS_UNDEF) {
				tri[j] = {}
			} else {
				tri[j] = {verts[idx*2], verts[idx*2 + 1]}
			}
		}
		result[i] = tri
	}
	return result
}

// delete_contours frees a contour set returned by tesselate_boundary_contours.
delete_contours :: proc(contours: [][][2]f64, allocator := context.allocator) {
	for c in contours {
		delete(c, allocator)
	}
	delete(contours, allocator)
}
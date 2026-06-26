package libtess2_port

Real  :: f64
Index :: i32

TESS_UNDEF :: ~Index(0)

TESS_MAX_VALID_INPUT_VALUE :: Real(1 << 51)
TESS_MIN_VALID_INPUT_VALUE :: -TESS_MAX_VALID_INPUT_VALUE

WindingRule :: enum i32 {
	Odd,
	Nonzero,
	Positive,
	Negative,
	Abs_Geq_Two,
}

ElementType :: enum i32 {
	Polygons,
	Connected_Polygons,
	Boundary_Contours,
}

Option :: enum i32 {
	Constrained_Delaunay_Triangulation,
	Reverse_Contours,
}

Status :: enum i32 {
	Ok,
	Out_Of_Memory,
	Invalid_Input,
}

Alloc :: struct {
	memalloc:             proc(userData: rawptr, size: u32) -> rawptr,
	memrealloc:           proc(userData: rawptr, ptr: rawptr, size: u32) -> rawptr,
	memfree:              proc(userData: rawptr, ptr: rawptr),
	userData:             rawptr,
	meshEdgeBucketSize:   i32,
	meshVertexBucketSize: i32,
	meshFaceBucketSize:   i32,
	dictNodeBucketSize:   i32,
	regionBucketSize:     i32,
	extraVertices:        i32,
}

Tesselator :: struct {
	mesh:   ^Mesh,
	status: Status,

	normal: [3]Real,
	sUnit:  [3]Real,
	tUnit:  [3]Real,

	bmin: [2]Real,
	bmax: [2]Real,

	processCDT:       i32,
	reverseContours:  i32,
	windingRule:      WindingRule,

	dict:        ^Dict,
	pq:          ^PriorityQ,
	event:       ^Vertex,
	regionPool:  ^BucketAlloc,

	vertexIndexCounter: Index,

	vertices:      []Real,
	vertexIndices: []Index,
	vertexCount:   i32,
	elements:      []Index,
	elementCount:  i32,

	alloc: Alloc,
}

// --- Public API ---

tess_new :: proc(alloc: ^Alloc = nil) -> ^Tesselator {
	return tessNewTess(alloc)
}

tess_delete :: proc(tess: ^Tesselator) {
	tessDeleteTess(tess)
}

tess_add_contour :: proc(tess: ^Tesselator, size: i32, vertices: rawptr, stride: i32, count: i32) {
	tessAddContour(tess, size, vertices, stride, count)
}

tess_set_option :: proc(tess: ^Tesselator, option: Option, value: i32) {
	tessSetOption(tess, option, value)
}

tess_tesselate :: proc(
	tess: ^Tesselator,
	winding_rule: WindingRule,
	element_type: ElementType,
	poly_size: i32,
	vertex_size: i32,
	normal: []Real,
) -> bool {
	return tessTesselate(tess, winding_rule, element_type, poly_size, vertex_size, normal)
}

tess_get_vertex_count :: proc(tess: ^Tesselator) -> i32 {
	return tess.vertexCount
}

tess_get_vertices :: proc(tess: ^Tesselator) -> []Real {
	return tess.vertices
}

tess_get_vertex_indices :: proc(tess: ^Tesselator) -> []Index {
	return tess.vertexIndices
}

tess_get_element_count :: proc(tess: ^Tesselator) -> i32 {
	return tess.elementCount
}

tess_get_elements :: proc(tess: ^Tesselator) -> []Index {
	return tess.elements
}

tess_get_status :: proc(tess: ^Tesselator) -> Status {
	return tess.status
}

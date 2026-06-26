package libtess2_port

import "core:mem"

// ---- Default allocator (pure Odin, backed by context.allocator) ----
//
// The public Alloc interface mirrors C: memrealloc/memfree receive only a
// pointer, not its size. To support that on top of Odin's allocator we prepend
// a small header storing each allocation's size, so realloc/free can recover
// it. The header is 16 bytes, which preserves 16-byte alignment of the pointer
// handed back to callers.

@(private="file")
ALLOC_HEADER :: 16

@(private="file")
default_alloc_proc :: proc(userData: rawptr, size: u32) -> rawptr {
	ptr, err := mem.alloc(int(size) + ALLOC_HEADER)
	if err != nil || ptr == nil { return nil }
	(^uint)(ptr)^ = uint(size)
	return rawptr(uintptr(ptr) + ALLOC_HEADER)
}

@(private="file")
default_realloc_proc :: proc(userData: rawptr, ptr: rawptr, size: u32) -> rawptr {
	if ptr == nil { return default_alloc_proc(userData, size) }
	base    := uintptr(ptr) - ALLOC_HEADER
	oldSize := (^uint)(rawptr(base))^
	newPtr  := default_alloc_proc(userData, size)
	if newPtr == nil { return nil }
	mem.copy(newPtr, ptr, min(int(oldSize), int(size)))
	default_free_proc(userData, ptr)
	return newPtr
}

@(private="file")
default_free_proc :: proc(userData: rawptr, ptr: rawptr) {
	if ptr == nil { return }
	mem.free(rawptr(uintptr(ptr) - ALLOC_HEADER))
}

@(private="file")
default_alloc := Alloc{
	memalloc   = default_alloc_proc,
	memrealloc = default_realloc_proc,
	memfree    = default_free_proc,
}

// ---- Utility ----

@(private="file")
longAxis :: proc(v: [3]Real) -> int {
	i := 0
	if abs(v[1]) > abs(v[0]) { i = 1 }
	if abs(v[2]) > abs(v[i]) { i = 2 }
	return i
}

@(private="file")
shortAxis :: proc(v: [3]Real) -> int {
	i := 0
	if abs(v[1]) < abs(v[0]) { i = 1 }
	if abs(v[2]) < abs(v[i]) { i = 2 }
	return i
}

@(private="file")
dot :: proc(u, v: [3]Real) -> Real {
	return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]
}

@(private="file")
computeNormal :: proc(tess: ^Tesselator, norm: ^[3]Real) {
	vHead := &tess.mesh.vHead
	v     := vHead.next
	if v == vHead {
		norm[0] = 0; norm[1] = 0; norm[2] = 1
		return
	}

	maxVal: [3]Real; minVal: [3]Real
	maxVert: [3]^Vertex; minVert: [3]^Vertex
	for i in 0..<3 {
		c := v.coords[i]
		minVal[i] = c; minVert[i] = v
		maxVal[i] = c; maxVert[i] = v
	}

	for v = vHead.next; v != vHead; v = v.next {
		for i in 0..<3 {
			c := v.coords[i]
			if c < minVal[i] { minVal[i] = c; minVert[i] = v }
			if c > maxVal[i] { maxVal[i] = c; maxVert[i] = v }
		}
	}

	i := 0
	if maxVal[1] - minVal[1] > maxVal[0] - minVal[0] { i = 1 }
	if maxVal[2] - minVal[2] > maxVal[i] - minVal[i] { i = 2 }
	if minVal[i] >= maxVal[i] {
		norm[0] = 0; norm[1] = 0; norm[2] = 1
		return
	}

	maxLen2 := Real(0)
	v1 := minVert[i]; v2 := maxVert[i]
	d1 := [3]Real{v1.coords[0]-v2.coords[0], v1.coords[1]-v2.coords[1], v1.coords[2]-v2.coords[2]}

	for v = vHead.next; v != vHead; v = v.next {
		d2 := [3]Real{v.coords[0]-v2.coords[0], v.coords[1]-v2.coords[1], v.coords[2]-v2.coords[2]}
		tNorm := [3]Real{
			d1[1]*d2[2] - d1[2]*d2[1],
			d1[2]*d2[0] - d1[0]*d2[2],
			d1[0]*d2[1] - d1[1]*d2[0],
		}
		tLen2 := tNorm[0]*tNorm[0] + tNorm[1]*tNorm[1] + tNorm[2]*tNorm[2]
		if tLen2 > maxLen2 {
			maxLen2 = tLen2
			norm^ = tNorm
		}
	}

	if maxLen2 <= 0 {
		norm[0] = 0; norm[1] = 0; norm[2] = 0
		norm[shortAxis(d1)] = 1
	}
}

@(private="file")
checkOrientation :: proc(tess: ^Tesselator) {
	fHead := &tess.mesh.fHead
	vHead := &tess.mesh.vHead
	area  := Real(0)
	for f := fHead.next; f != fHead; f = f.next {
		e := f.anEdge
		if e.winding <= 0 { continue }
		for {
			area += (e.Org.s - Dst(e).s) * (e.Org.t + Dst(e).t)
			e = e.Lnext
			if e == f.anEdge { break }
		}
	}
	if area < 0 {
		for v := vHead.next; v != vHead; v = v.next {
			v.t = -v.t
		}
		tess.tUnit[0] = -tess.tUnit[0]
		tess.tUnit[1] = -tess.tUnit[1]
		tess.tUnit[2] = -tess.tUnit[2]
	}
}

S_UNIT_X :: Real(1.0)
S_UNIT_Y :: Real(0.0)

tessProjectPolygon :: proc(tess: ^Tesselator) {
	vHead := &tess.mesh.vHead
	norm  := tess.normal
	computedNormal := false

	if norm[0] == 0 && norm[1] == 0 && norm[2] == 0 {
		computeNormal(tess, &norm)
		computedNormal = true
	}

	i := longAxis(norm)
	tess.sUnit[i]     = 0
	tess.sUnit[(i+1)%3] = S_UNIT_X
	tess.sUnit[(i+2)%3] = S_UNIT_Y

	tess.tUnit[i]       = 0
	tess.tUnit[(i+1)%3] = norm[i] > 0 ? -S_UNIT_Y : S_UNIT_Y
	tess.tUnit[(i+2)%3] = norm[i] > 0 ?  S_UNIT_X : -S_UNIT_X

	sUnit := tess.sUnit; tUnit := tess.tUnit
	for v := vHead.next; v != vHead; v = v.next {
		v.s = dot(v.coords, sUnit)
		v.t = dot(v.coords, tUnit)
	}
	if computedNormal { checkOrientation(tess) }

	first := true
	for v := vHead.next; v != vHead; v = v.next {
		if first {
			tess.bmin[0] = v.s; tess.bmax[0] = v.s
			tess.bmin[1] = v.t; tess.bmax[1] = v.t
			first = false
		} else {
			if v.s < tess.bmin[0] { tess.bmin[0] = v.s }
			if v.s > tess.bmax[0] { tess.bmax[0] = v.s }
			if v.t < tess.bmin[1] { tess.bmin[1] = v.t }
			if v.t > tess.bmax[1] { tess.bmax[1] = v.t }
		}
	}
}

// ---- Mono-region tessellation ----

tessMeshTessellateMonoRegion :: proc(mesh: ^Mesh, face: ^Face) -> bool {
	up := face.anEdge
	assert(up.Lnext != up && up.Lnext.Lnext != up)

	for VertLeq(Dst(up), up.Org) { up = Lprev(up) }
	for VertLeq(up.Org, Dst(up)) { up = up.Lnext }
	lo := Lprev(up)

	for up.Lnext != lo {
		if VertLeq(Dst(up), lo.Org) {
			for lo.Lnext != up && (EdgeGoesLeft(lo.Lnext) ||
				EdgeSign(lo.Org, Dst(lo), Dst(lo.Lnext)) <= 0) {
				tmp := tessMeshConnect(mesh, lo.Lnext, lo)
				if tmp == nil { return false }
				lo = tmp.Sym
			}
			lo = Lprev(lo)
		} else {
			for lo.Lnext != up && (EdgeGoesRight(Lprev(up)) ||
				EdgeSign(Dst(up), up.Org, Lprev(up).Org) >= 0) {
				tmp := tessMeshConnect(mesh, up, Lprev(up))
				if tmp == nil { return false }
				up = tmp.Sym
			}
			up = up.Lnext
		}
	}

	assert(lo.Lnext != up)
	for lo.Lnext.Lnext != up {
		tmp := tessMeshConnect(mesh, lo.Lnext, lo)
		if tmp == nil { return false }
		lo = tmp.Sym
	}
	return true
}

tessMeshTessellateInterior :: proc(mesh: ^Mesh) -> bool {
	f := mesh.fHead.next
	for f != &mesh.fHead {
		next := f.next
		if f.inside != 0 {
			if !tessMeshTessellateMonoRegion(mesh, f) { return false }
		}
		f = next
	}
	return true
}

// ---- CDT (Constrained Delaunay) ----

EdgeStackNode :: struct {
	edge: ^HalfEdge,
	next: ^EdgeStackNode,
}

EdgeStack :: struct {
	top:        ^EdgeStackNode,
	nodeBucket: ^BucketAlloc,
}

@(private="file")
stackInit :: proc(stack: ^EdgeStack, alloc: ^Alloc) -> bool {
	stack.top        = nil
	stack.nodeBucket = createBucketAlloc(alloc, "CDT nodes", size_of(EdgeStackNode), 512)
	return stack.nodeBucket != nil
}

@(private="file")
stackDelete :: proc(stack: ^EdgeStack) {
	deleteBucketAlloc(stack.nodeBucket)
}

@(private="file")
stackEmpty :: proc(stack: ^EdgeStack) -> bool { return stack.top == nil }

@(private="file")
stackPush :: proc(stack: ^EdgeStack, e: ^HalfEdge) {
	node := (^EdgeStackNode)(bucketAlloc(stack.nodeBucket))
	if node == nil { return }
	node.edge  = e
	node.next  = stack.top
	stack.top  = node
}

@(private="file")
stackPop :: proc(stack: ^EdgeStack) -> ^HalfEdge {
	node := stack.top
	if node == nil { return nil }
	stack.top = node.next
	e := node.edge
	bucketFree(stack.nodeBucket, node)
	return e
}

tessMeshRefineDelaunay :: proc(mesh: ^Mesh, alloc: ^Alloc) {
	stack: EdgeStack
	if !stackInit(&stack, alloc) { return }
	defer stackDelete(&stack)

	maxFaces := 0
	for f := mesh.fHead.next; f != &mesh.fHead; f = f.next {
		if f.inside != 0 {
			e := f.anEdge
			for {
				e.mark = EdgeIsInternal(e) ? 1 : 0
				if e.mark != 0 && e.Sym.mark == 0 { stackPush(&stack, e) }
				e = e.Lnext
				if e == f.anEdge { break }
			}
			maxFaces += 1
		}
	}

	maxIter := maxFaces * maxFaces
	iter    := 0
	for !stackEmpty(&stack) && iter < maxIter {
		e := stackPop(&stack)
		e.mark = 0; e.Sym.mark = 0
		if !tesedgeIsLocallyDelaunay(e) {
			edges := [4]^HalfEdge{e.Lnext, Lprev(e), e.Sym.Lnext, Lprev(e.Sym)}
			tessMeshFlipEdge(mesh, e)
			for i in 0..<4 {
				if edges[i].mark == 0 && EdgeIsInternal(edges[i]) {
					edges[i].mark = 1; edges[i].Sym.mark = 1
					stackPush(&stack, edges[i])
				}
			}
		}
		iter += 1
	}
}

// ---- Output ----

@(private="file")
getNeighbourFace :: proc(edge: ^HalfEdge) -> Index {
	if Rface(edge) == nil        { return TESS_UNDEF }
	if Rface(edge).inside == 0   { return TESS_UNDEF }
	return Rface(edge).n
}

@(private="file")
outputPolymesh :: proc(tess: ^Tesselator, mesh: ^Mesh, elementType: ElementType, polySize: int, vertexSize: int) {
	if polySize > 3 {
		if !tessMeshMergeConvexFaces(mesh, polySize) {
			tess.status = .Out_Of_Memory
			return
		}
	}

	// Mark unused vertices
	for v := mesh.vHead.next; v != &mesh.vHead; v = v.next {
		v.n = TESS_UNDEF
	}

	maxVertexCount := Index(0)
	maxFaceCount   := Index(0)

	for f := mesh.fHead.next; f != &mesh.fHead; f = f.next {
		f.n = TESS_UNDEF
		if f.inside == 0 { continue }
		edge := f.anEdge
		faceVerts := 0
		for {
			v := edge.Org
			if v.n == TESS_UNDEF {
				v.n = maxVertexCount
				maxVertexCount += 1
			}
			faceVerts += 1
			edge = edge.Lnext
			if edge == f.anEdge { break }
		}
		assert(faceVerts <= polySize)
		f.n = maxFaceCount
		maxFaceCount += 1
	}

	tess.elementCount = i32(maxFaceCount)
	allocCount := maxFaceCount
	if elementType == .Connected_Polygons { allocCount *= 2 }
	raw := tess.alloc.memalloc(tess.alloc.userData, u32(allocCount) * u32(polySize) * size_of(Index))
	if raw == nil { tess.status = .Out_Of_Memory; return }
	tess.elements = (([^]Index)(raw))[:int(allocCount) * polySize]

	tess.vertexCount = i32(maxVertexCount)
	raw = tess.alloc.memalloc(tess.alloc.userData, u32(maxVertexCount) * u32(vertexSize) * size_of(Real))
	if raw == nil { tess.status = .Out_Of_Memory; return }
	tess.vertices = (([^]Real)(raw))[:int(maxVertexCount) * vertexSize]

	raw = tess.alloc.memalloc(tess.alloc.userData, u32(maxVertexCount) * size_of(Index))
	if raw == nil { tess.status = .Out_Of_Memory; return }
	tess.vertexIndices = (([^]Index)(raw))[:int(maxVertexCount)]

	// Fill vertices
	for v := mesh.vHead.next; v != &mesh.vHead; v = v.next {
		if v.n != TESS_UNDEF {
			base := int(v.n) * vertexSize
			tess.vertices[base]   = v.coords[0]
			tess.vertices[base+1] = v.coords[1]
			if vertexSize > 2 { tess.vertices[base+2] = v.coords[2] }
			tess.vertexIndices[v.n] = v.idx
		}
	}

	// Fill elements
	eIdx := 0
	for f := mesh.fHead.next; f != &mesh.fHead; f = f.next {
		if f.inside == 0 { continue }
		edge      := f.anEdge
		faceVerts := 0
		for {
			tess.elements[eIdx] = edge.Org.n
			eIdx += 1; faceVerts += 1
			edge = edge.Lnext
			if edge == f.anEdge { break }
		}
		for i := faceVerts; i < polySize; i += 1 {
			tess.elements[eIdx] = TESS_UNDEF; eIdx += 1
		}
		if elementType == .Connected_Polygons {
			edge = f.anEdge
			faceVerts2 := 0
			for {
				tess.elements[eIdx] = getNeighbourFace(edge)
				eIdx += 1; faceVerts2 += 1
				edge = edge.Lnext
				if edge == f.anEdge { break }
			}
			for i := faceVerts2; i < polySize; i += 1 {
				tess.elements[eIdx] = TESS_UNDEF; eIdx += 1
			}
		}
	}
}

@(private="file")
outputContours :: proc(tess: ^Tesselator, mesh: ^Mesh, vertexSize: int) {
	tess.vertexCount  = 0
	tess.elementCount = 0

	for f := mesh.fHead.next; f != &mesh.fHead; f = f.next {
		if f.inside == 0 { continue }
		start := f.anEdge; edge := start
		for {
			tess.vertexCount += 1
			edge = edge.Lnext
			if edge == start { break }
		}
		tess.elementCount += 1
	}

	raw := tess.alloc.memalloc(tess.alloc.userData, u32(tess.elementCount) * 2 * size_of(Index))
	if raw == nil { tess.status = .Out_Of_Memory; return }
	tess.elements = (([^]Index)(raw))[:int(tess.elementCount) * 2]

	raw = tess.alloc.memalloc(tess.alloc.userData, u32(tess.vertexCount) * u32(vertexSize) * size_of(Real))
	if raw == nil { tess.status = .Out_Of_Memory; return }
	tess.vertices = (([^]Real)(raw))[:int(tess.vertexCount) * vertexSize]

	raw = tess.alloc.memalloc(tess.alloc.userData, u32(tess.vertexCount) * size_of(Index))
	if raw == nil { tess.status = .Out_Of_Memory; return }
	tess.vertexIndices = (([^]Index)(raw))[:int(tess.vertexCount)]

	vIdx := 0; eIdx := 0; startVert := 0
	for f := mesh.fHead.next; f != &mesh.fHead; f = f.next {
		if f.inside == 0 { continue }
		vertCount := 0
		start := f.anEdge; edge := start
		for {
			base := vIdx * vertexSize
			tess.vertices[base]   = edge.Org.coords[0]
			tess.vertices[base+1] = edge.Org.coords[1]
			if vertexSize > 2 { tess.vertices[base+2] = edge.Org.coords[2] }
			tess.vertexIndices[vIdx] = edge.Org.idx
			vIdx += 1; vertCount += 1
			edge = edge.Lnext
			if edge == start { break }
		}
		tess.elements[eIdx]   = Index(startVert)
		tess.elements[eIdx+1] = Index(vertCount)
		eIdx += 2; startVert += vertCount
	}
}

@(private="file")
isValidCoord :: proc(coord: Real) -> bool {
	return coord <= TESS_MAX_VALID_INPUT_VALUE && coord >= TESS_MIN_VALID_INPUT_VALUE
}

// ---- Public API implementation ----

tessNewTess :: proc(alloc_: ^Alloc) -> ^Tesselator {
	alloc := alloc_
	if alloc == nil { alloc = &default_alloc }

	tess := (^Tesselator)(alloc.memalloc(alloc.userData, size_of(Tesselator)))
	if tess == nil { return nil }

	tess.alloc = alloc^
	a := &tess.alloc
	if a.meshEdgeBucketSize   == 0 { a.meshEdgeBucketSize   = 512 }
	if a.meshVertexBucketSize == 0 { a.meshVertexBucketSize = 512 }
	if a.meshFaceBucketSize   == 0 { a.meshFaceBucketSize   = 256 }
	if a.dictNodeBucketSize   == 0 { a.dictNodeBucketSize   = 512 }
	if a.regionBucketSize     == 0 { a.regionBucketSize     = 256 }

	tess.normal        = {0, 0, 0}
	tess.bmin          = {0, 0}
	tess.bmax          = {0, 0}
	tess.reverseContours = 0
	tess.windingRule   = .Odd
	tess.processCDT    = 0

	if a.regionBucketSize < 16   { a.regionBucketSize = 16 }
	if a.regionBucketSize > 4096 { a.regionBucketSize = 4096 }
	tess.regionPool = createBucketAlloc(a, "Regions", size_of(ActiveRegion), u32(a.regionBucketSize))

	tess.mesh               = nil
	tess.status             = .Ok
	tess.vertexIndexCounter = 0
	tess.vertices           = nil
	tess.vertexIndices      = nil
	tess.vertexCount        = 0
	tess.elements           = nil
	tess.elementCount       = 0
	return tess
}

tessDeleteTess :: proc(tess: ^Tesselator) {
	alloc := tess.alloc
	deleteBucketAlloc(tess.regionPool)
	if tess.mesh != nil {
		tessMeshDeleteMesh(&alloc, tess.mesh)
		tess.mesh = nil
	}
	if tess.vertices      != nil { alloc.memfree(alloc.userData, raw_data(tess.vertices)) }
	if tess.vertexIndices != nil { alloc.memfree(alloc.userData, raw_data(tess.vertexIndices)) }
	if tess.elements      != nil { alloc.memfree(alloc.userData, raw_data(tess.elements)) }
	alloc.memfree(alloc.userData, tess)
}

tessAddContour :: proc(tess: ^Tesselator, size_: i32, vertices: rawptr, stride: i32, numVertices: i32) {
	size := size_
	if size < 2 { size = 2 }
	if size > 3 { size = 3 }

	if tess.mesh == nil {
		tess.mesh = tessMeshNewMesh(&tess.alloc)
	}
	if tess.mesh == nil {
		tess.status = .Out_Of_Memory
		return
	}

	src := uintptr(vertices)
	e: ^HalfEdge

	for i in 0..<numVertices {
		coords := (^[3]Real)(src)
		src += uintptr(stride)

		if !isValidCoord(coords[0]) || !isValidCoord(coords[1]) ||
		   (size > 2 && !isValidCoord(coords[2])) {
			tess.status = .Invalid_Input
			return
		}

		if e == nil {
			e = tessMeshMakeEdge(tess.mesh)
			if e == nil { tess.status = .Out_Of_Memory; return }
			if !tessMeshSplice(tess.mesh, e, e.Sym) { tess.status = .Out_Of_Memory; return }
		} else {
			if tessMeshSplitEdge(tess.mesh, e) == nil { tess.status = .Out_Of_Memory; return }
			e = e.Lnext
		}

		e.Org.coords[0] = coords[0]
		e.Org.coords[1] = coords[1]
		e.Org.coords[2] = size > 2 ? coords[2] : 0
		e.Org.idx       = tess.vertexIndexCounter
		tess.vertexIndexCounter += 1

		e.winding     = tess.reverseContours != 0 ? -1 : 1
		e.Sym.winding = tess.reverseContours != 0 ?  1 : -1
	}
}

tessSetOption :: proc(tess: ^Tesselator, option: Option, value: i32) {
	switch option {
	case .Constrained_Delaunay_Triangulation:
		tess.processCDT = value > 0 ? 1 : 0
	case .Reverse_Contours:
		tess.reverseContours = value > 0 ? 1 : 0
	}
}

tessTesselate :: proc(tess: ^Tesselator, windingRule: WindingRule, elementType: ElementType,
                      polySize: i32, vertexSize_: i32, normal: []Real) -> bool {
	vertexSize  := int(vertexSize_)
	if vertexSize < 2 { vertexSize = 2 }
	if vertexSize > 3 { vertexSize = 3 }

	// Free previous output
	a := &tess.alloc
	if tess.vertices      != nil { a.memfree(a.userData, raw_data(tess.vertices));      tess.vertices = nil }
	if tess.elements      != nil { a.memfree(a.userData, raw_data(tess.elements));      tess.elements = nil }
	if tess.vertexIndices != nil { a.memfree(a.userData, raw_data(tess.vertexIndices)); tess.vertexIndices = nil }

	tess.vertexIndexCounter = 0

	if normal != nil {
		tess.normal[0] = normal[0]
		tess.normal[1] = normal[1]
		tess.normal[2] = normal[2]
	}
	tess.windingRule = windingRule

	if tess.status != .Ok || tess.mesh == nil {
		return false
	}

	tessProjectPolygon(tess)

	if !tessComputeInterior(tess) {
		tess.status = .Out_Of_Memory
		return false
	}

	mesh := tess.mesh
	rc   := true
	if elementType == .Boundary_Contours {
		rc = tessMeshSetWindingNumber(mesh, 1, true)
	} else {
		rc = tessMeshTessellateInterior(mesh)
		if rc && tess.processCDT != 0 {
			tessMeshRefineDelaunay(mesh, &tess.alloc)
		}
	}
	if !rc {
		tess.status = .Out_Of_Memory
		return false
	}

	tessMeshCheckMesh(mesh)

	if elementType == .Boundary_Contours {
		outputContours(tess, mesh, vertexSize)
	} else {
		outputPolymesh(tess, mesh, elementType, int(polySize), vertexSize)
	}

	tessMeshDeleteMesh(&tess.alloc, mesh)
	tess.mesh = nil

	return tess.status == .Ok
}

tessMeshSetWindingNumber :: proc(mesh: ^Mesh, value: i32, keepOnlyBoundary: bool) -> bool {
	e := mesh.eHead.next
	for e != &mesh.eHead {
		eNext := e.next
		if Rface(e).inside != e.Lface.inside {
			e.winding = e.Lface.inside != 0 ? value : -value
		} else {
			if !keepOnlyBoundary {
				e.winding = 0
			} else {
				if !tessMeshDelete(mesh, e) { return false }
			}
		}
		e = eNext
	}
	return true
}

tessMeshDiscardExterior :: proc(mesh: ^Mesh) {
	f := mesh.fHead.next
	for f != &mesh.fHead {
		next := f.next
		if f.inside == 0 { tessMeshZapFace(mesh, f) }
		f = next
	}
}

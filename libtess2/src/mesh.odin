package libtess2_port

// Mesh types

Vertex :: struct {
	next:     ^Vertex,
	prev:     ^Vertex,
	anEdge:   ^HalfEdge,
	coords:   [3]Real,
	s, t:     Real,
	pqHandle: PQhandle,
	n:        Index,
	idx:      Index,
}

Face :: struct {
	next:    ^Face,
	prev:    ^Face,
	anEdge:  ^HalfEdge,
	trail:   ^Face,
	n:       Index,
	marked:  i8,
	inside:  i8,
}

HalfEdge :: struct {
	next:         ^HalfEdge,
	Sym:          ^HalfEdge,
	Onext:        ^HalfEdge,
	Lnext:        ^HalfEdge,
	Org:          ^Vertex,
	Lface:        ^Face,
	activeRegion: ^ActiveRegion,
	winding:      i32,
	mark:         i32,
}

// EdgePair — allocated as a unit so pointer comparison can distinguish the two halves
EdgePair :: struct {
	e:    HalfEdge,
	eSym: HalfEdge,
}

Mesh :: struct {
	vHead:    Vertex,
	fHead:    Face,
	eHead:    HalfEdge,
	eHeadSym: HalfEdge,

	edgeBucket:   ^BucketAlloc,
	vertexBucket: ^BucketAlloc,
	faceBucket:   ^BucketAlloc,
}

// ---- Internal helpers ----

@(private="file")
makeEdge :: proc(mesh: ^Mesh, eNext_: ^HalfEdge) -> ^HalfEdge {
	eNext := eNext_
	pair  := (^EdgePair)(bucketAlloc(mesh.edgeBucket))
	if pair == nil { return nil }

	e    := &pair.e
	eSym := &pair.eSym

	// Ensure eNext points to the first of the pair (lower address)
	if uintptr(eNext.Sym) < uintptr(eNext) { eNext = eNext.Sym }

	ePrev        := eNext.Sym.next
	eSym.next    = ePrev
	ePrev.Sym.next = e
	e.next       = eNext
	eNext.Sym.next = eSym

	e.Sym = eSym; e.Onext = e; e.Lnext = eSym
	e.Org = nil;  e.Lface = nil; e.winding = 0; e.activeRegion = nil; e.mark = 0

	eSym.Sym = e; eSym.Onext = eSym; eSym.Lnext = e
	eSym.Org = nil; eSym.Lface = nil; eSym.winding = 0; eSym.activeRegion = nil; eSym.mark = 0

	return e
}

@(private="file")
splice :: proc(a, b: ^HalfEdge) {
	aOnext := a.Onext
	bOnext := b.Onext
	aOnext.Sym.Lnext = b
	bOnext.Sym.Lnext = a
	a.Onext = bOnext
	b.Onext = aOnext
}

@(private="file")
makeVertex :: proc(newVertex: ^Vertex, eOrig: ^HalfEdge, vNext: ^Vertex) {
	vNew  := newVertex
	vPrev := vNext.prev
	vNew.prev  = vPrev
	vPrev.next = vNew
	vNew.next  = vNext
	vNext.prev = vNew
	vNew.anEdge = eOrig
	e := eOrig
	for {
		e.Org = vNew
		e = e.Onext
		if e == eOrig { break }
	}
}

@(private="file")
makeFace :: proc(newFace: ^Face, eOrig: ^HalfEdge, fNext: ^Face) {
	fNew  := newFace
	fPrev := fNext.prev
	fNew.prev  = fPrev
	fPrev.next = fNew
	fNew.next  = fNext
	fNext.prev = fNew
	fNew.anEdge = eOrig
	fNew.trail  = nil
	fNew.marked = 0
	fNew.inside = fNext.inside
	e := eOrig
	for {
		e.Lface = fNew
		e = e.Lnext
		if e == eOrig { break }
	}
}

@(private="file")
killEdge :: proc(mesh: ^Mesh, eDel_: ^HalfEdge) {
	eDel := eDel_
	if uintptr(eDel.Sym) < uintptr(eDel) { eDel = eDel.Sym }
	eNext  := eDel.next
	ePrev  := eDel.Sym.next
	eNext.Sym.next = ePrev
	ePrev.Sym.next = eNext
	bucketFree(mesh.edgeBucket, eDel)
}

@(private="file")
killVertex :: proc(mesh: ^Mesh, vDel: ^Vertex, newOrg: ^Vertex) {
	eStart := vDel.anEdge
	e := eStart
	for {
		e.Org = newOrg
		e = e.Onext
		if e == eStart { break }
	}
	vPrev := vDel.prev; vNext := vDel.next
	vNext.prev = vPrev; vPrev.next = vNext
	bucketFree(mesh.vertexBucket, vDel)
}

@(private="file")
killFace :: proc(mesh: ^Mesh, fDel: ^Face, newLface: ^Face) {
	eStart := fDel.anEdge
	e := eStart
	for {
		e.Lface = newLface
		e = e.Lnext
		if e == eStart { break }
	}
	fPrev := fDel.prev; fNext := fDel.next
	fNext.prev = fPrev; fPrev.next = fNext
	bucketFree(mesh.faceBucket, fDel)
}

// ---- Public mesh operations ----

tessMeshMakeEdge :: proc(mesh: ^Mesh) -> ^HalfEdge {
	newV1 := (^Vertex)(bucketAlloc(mesh.vertexBucket))
	newV2 := (^Vertex)(bucketAlloc(mesh.vertexBucket))
	newF  := (^Face)(bucketAlloc(mesh.faceBucket))

	if newV1 == nil || newV2 == nil || newF == nil {
		if newV1 != nil { bucketFree(mesh.vertexBucket, newV1) }
		if newV2 != nil { bucketFree(mesh.vertexBucket, newV2) }
		if newF  != nil { bucketFree(mesh.faceBucket,   newF) }
		return nil
	}

	e := makeEdge(mesh, &mesh.eHead)
	if e == nil { return nil }

	makeVertex(newV1, e,     &mesh.vHead)
	makeVertex(newV2, e.Sym, &mesh.vHead)
	makeFace(newF, e, &mesh.fHead)
	return e
}

tessMeshSplice :: proc(mesh: ^Mesh, eOrg, eDst: ^HalfEdge) -> bool {
	if eOrg == eDst { return true }

	joiningVertices := false
	joiningLoops    := false

	if eDst.Org != eOrg.Org {
		joiningVertices = true
		killVertex(mesh, eDst.Org, eOrg.Org)
	}
	if eDst.Lface != eOrg.Lface {
		joiningLoops = true
		killFace(mesh, eDst.Lface, eOrg.Lface)
	}

	splice(eDst, eOrg)

	if !joiningVertices {
		newVertex := (^Vertex)(bucketAlloc(mesh.vertexBucket))
		if newVertex == nil { return false }
		makeVertex(newVertex, eDst, eOrg.Org)
		eOrg.Org.anEdge = eOrg
	}
	if !joiningLoops {
		newFace := (^Face)(bucketAlloc(mesh.faceBucket))
		if newFace == nil { return false }
		makeFace(newFace, eDst, eOrg.Lface)
		eOrg.Lface.anEdge = eOrg
	}
	return true
}

tessMeshDelete :: proc(mesh: ^Mesh, eDel: ^HalfEdge) -> bool {
	eDelSym     := eDel.Sym
	joiningLoops := false

	if eDel.Lface != Rface(eDel) {
		joiningLoops = true
		killFace(mesh, eDel.Lface, Rface(eDel))
	}

	if eDel.Onext == eDel {
		killVertex(mesh, eDel.Org, nil)
	} else {
		Rface(eDel).anEdge = Oprev(eDel)
		eDel.Org.anEdge    = eDel.Onext
		splice(eDel, Oprev(eDel))
		if !joiningLoops {
			newFace := (^Face)(bucketAlloc(mesh.faceBucket))
			if newFace == nil { return false }
			makeFace(newFace, eDel, eDel.Lface)
		}
	}

	if eDelSym.Onext == eDelSym {
		killVertex(mesh, eDelSym.Org, nil)
		killFace(mesh, eDelSym.Lface, nil)
	} else {
		eDel.Lface.anEdge  = Oprev(eDelSym)
		eDelSym.Org.anEdge = eDelSym.Onext
		splice(eDelSym, Oprev(eDelSym))
	}

	killEdge(mesh, eDel)
	return true
}

tessMeshAddEdgeVertex :: proc(mesh: ^Mesh, eOrg: ^HalfEdge) -> ^HalfEdge {
	eNew := makeEdge(mesh, eOrg)
	if eNew == nil { return nil }
	eNewSym := eNew.Sym

	splice(eNew, eOrg.Lnext)

	eNew.Org = Dst(eOrg)
	newVertex := (^Vertex)(bucketAlloc(mesh.vertexBucket))
	if newVertex == nil || eNew.Org == nil { return nil }
	makeVertex(newVertex, eNewSym, eNew.Org)
	eNew.Lface    = eOrg.Lface
	eNewSym.Lface = eOrg.Lface
	return eNew
}

tessMeshSplitEdge :: proc(mesh: ^Mesh, eOrg: ^HalfEdge) -> ^HalfEdge {
	tmp := tessMeshAddEdgeVertex(mesh, eOrg)
	if tmp == nil { return nil }
	eNew := tmp.Sym

	splice(eOrg.Sym, Oprev(eOrg.Sym))
	splice(eOrg.Sym, eNew)

	eOrg.Sym.Org = eNew.Org
	eNew.Sym.Org.anEdge = eNew.Sym
	eNew.Sym.Lface = Rface(eOrg)
	eNew.winding       = eOrg.winding
	eNew.Sym.winding   = eOrg.Sym.winding
	return eNew
}

tessMeshConnect :: proc(mesh: ^Mesh, eOrg, eDst: ^HalfEdge) -> ^HalfEdge {
	eNew := makeEdge(mesh, eOrg)
	if eNew == nil { return nil }
	eNewSym := eNew.Sym

	joiningLoops := false
	if eDst.Lface != eOrg.Lface {
		joiningLoops = true
		killFace(mesh, eDst.Lface, eOrg.Lface)
	}

	splice(eNew, eOrg.Lnext)
	splice(eNewSym, eDst)

	eNew.Org    = Dst(eOrg)
	eNewSym.Org = eDst.Org
	eNew.Lface    = eOrg.Lface
	eNewSym.Lface = eOrg.Lface
	eOrg.Lface.anEdge = eNewSym

	if !joiningLoops {
		newFace := (^Face)(bucketAlloc(mesh.faceBucket))
		if newFace == nil { return nil }
		makeFace(newFace, eNew, eOrg.Lface)
	}
	return eNew
}

tessMeshZapFace :: proc(mesh: ^Mesh, fZap: ^Face) {
	eStart := fZap.anEdge
	eNext  := eStart.Lnext
	for {
		e     := eNext
		eNext  = e.Lnext
		e.Lface = nil
		if Rface(e) == nil {
			if e.Onext == e {
				killVertex(mesh, e.Org, nil)
			} else {
				e.Org.anEdge = e.Onext
				splice(e, Oprev(e))
			}
			eSym := e.Sym
			if eSym.Onext == eSym {
				killVertex(mesh, eSym.Org, nil)
			} else {
				eSym.Org.anEdge = eSym.Onext
				splice(eSym, Oprev(eSym))
			}
			killEdge(mesh, e)
		}
		if e == eStart { break }
	}
	fPrev := fZap.prev; fNext := fZap.next
	fNext.prev = fPrev; fPrev.next = fNext
	bucketFree(mesh.faceBucket, fZap)
}

tessMeshNewMesh :: proc(alloc: ^Alloc) -> ^Mesh {
	mesh := (^Mesh)(alloc.memalloc(alloc.userData, size_of(Mesh)))
	if mesh == nil { return nil }

	if alloc.meshEdgeBucketSize < 16   { alloc.meshEdgeBucketSize = 16 }
	if alloc.meshEdgeBucketSize > 4096 { alloc.meshEdgeBucketSize = 4096 }
	if alloc.meshVertexBucketSize < 16   { alloc.meshVertexBucketSize = 16 }
	if alloc.meshVertexBucketSize > 4096 { alloc.meshVertexBucketSize = 4096 }
	if alloc.meshFaceBucketSize < 16   { alloc.meshFaceBucketSize = 16 }
	if alloc.meshFaceBucketSize > 4096 { alloc.meshFaceBucketSize = 4096 }

	mesh.edgeBucket   = createBucketAlloc(alloc, "Mesh Edges",    size_of(EdgePair), u32(alloc.meshEdgeBucketSize))
	mesh.vertexBucket = createBucketAlloc(alloc, "Mesh Vertices", size_of(Vertex),   u32(alloc.meshVertexBucketSize))
	mesh.faceBucket   = createBucketAlloc(alloc, "Mesh Faces",    size_of(Face),     u32(alloc.meshFaceBucketSize))

	v    := &mesh.vHead
	f    := &mesh.fHead
	e    := &mesh.eHead
	eSym := &mesh.eHeadSym

	v.next = v; v.prev = v; v.anEdge = nil

	f.next = f; f.prev = f; f.anEdge = nil
	f.trail = nil; f.marked = 0; f.inside = 0

	e.next = e; e.Sym = eSym; e.Onext = nil; e.Lnext = nil
	e.Org = nil; e.Lface = nil; e.winding = 0; e.activeRegion = nil

	eSym.next = eSym; eSym.Sym = e; eSym.Onext = nil; eSym.Lnext = nil
	eSym.Org = nil; eSym.Lface = nil; eSym.winding = 0; eSym.activeRegion = nil

	return mesh
}

tessMeshUnion :: proc(alloc: ^Alloc, mesh1, mesh2: ^Mesh) -> ^Mesh {
	f1 := &mesh1.fHead; v1 := &mesh1.vHead; e1 := &mesh1.eHead
	f2 := &mesh2.fHead; v2 := &mesh2.vHead; e2 := &mesh2.eHead

	if f2.next != f2 {
		f1.prev.next = f2.next; f2.next.prev = f1.prev
		f2.prev.next = f1;      f1.prev = f2.prev
	}
	if v2.next != v2 {
		v1.prev.next = v2.next; v2.next.prev = v1.prev
		v2.prev.next = v1;      v1.prev = v2.prev
	}
	if e2.next != e2 {
		e1.Sym.next.Sym.next = e2.next;   e2.next.Sym.next = e1.Sym.next
		e2.Sym.next.Sym.next = e1;        e1.Sym.next = e2.Sym.next
	}

	alloc.memfree(alloc.userData, mesh2)
	return mesh1
}

@(private="file")
countFaceVerts :: proc(f: ^Face) -> int {
	eCur := f.anEdge
	n    := 0
	for {
		n += 1
		eCur = eCur.Lnext
		if eCur == f.anEdge { break }
	}
	return n
}

tessMeshMergeConvexFaces :: proc(mesh: ^Mesh, maxVertsPerFace: int) -> bool {
	eHead := &mesh.eHead
	e     := eHead.next
	for e != eHead {
		eNext := e.next
		eSym  := e.Sym
		if eSym == nil { e = eNext; continue }
		if e.Lface == nil || e.Lface.inside == 0 { e = eNext; continue }
		if eSym.Lface == nil || eSym.Lface.inside == 0 { e = eNext; continue }

		leftNv  := countFaceVerts(e.Lface)
		rightNv := countFaceVerts(eSym.Lface)
		if leftNv + rightNv - 2 > maxVertsPerFace { e = eNext; continue }

		va := Lprev(e).Org
		vb := e.Org
		vc := Dst(Lnext(e.Sym))
		vd := Lprev(e.Sym).Org
		ve := e.Sym.Org
		vf := Dst(e.Lnext)

		if VertCCW(va, vb, vc) && VertCCW(vd, ve, vf) {
			if e == eNext || e == eNext.Sym { eNext = eNext.next }
			if !tessMeshDelete(mesh, e) { return false }
		}
		e = eNext
	}
	return true
}

// Lnext helper for readability inside this file only
@(private="file")
Lnext :: #force_inline proc(e: ^HalfEdge) -> ^HalfEdge { return e.Lnext }

tessMeshFlipEdge :: proc(mesh: ^Mesh, edge: ^HalfEdge) {
	a0 := edge
	a1 := a0.Lnext
	a2 := a1.Lnext
	b0 := edge.Sym
	b1 := b0.Lnext
	b2 := b1.Lnext

	aOrg := a0.Org; aOpp := a2.Org
	bOrg := b0.Org; bOpp := b2.Org

	fa := a0.Lface; fb := b0.Lface

	assert(EdgeIsInternal(edge))
	assert(a2.Lnext == a0)
	assert(b2.Lnext == b0)

	a0.Org = bOpp; a0.Onext = b1.Sym
	b0.Org = aOpp; b0.Onext = a1.Sym
	a2.Onext = b0; b2.Onext = a0
	b1.Onext = a2.Sym; a1.Onext = b2.Sym

	a0.Lnext = a2; a2.Lnext = b1; b1.Lnext = a0
	b0.Lnext = b2; b2.Lnext = a1; a1.Lnext = b0

	a1.Lface = fb; b1.Lface = fa
	fa.anEdge = a0; fb.anEdge = b0

	if aOrg.anEdge == a0 { aOrg.anEdge = b1 }
	if bOrg.anEdge == b0 { bOrg.anEdge = a1 }
}

tessMeshDeleteMesh :: proc(alloc: ^Alloc, mesh: ^Mesh) {
	deleteBucketAlloc(mesh.edgeBucket)
	deleteBucketAlloc(mesh.vertexBucket)
	deleteBucketAlloc(mesh.faceBucket)
	alloc.memfree(alloc.userData, mesh)
}

when !ODIN_DEBUG {
	tessMeshCheckMesh :: proc(mesh: ^Mesh) {}
} else {
	tessMeshCheckMesh :: proc(mesh: ^Mesh) {
		fHead := &mesh.fHead
		vHead := &mesh.vHead
		eHead := &mesh.eHead

		fPrev := fHead
		for {
			f := fPrev.next
			if f == fHead { break }
			assert(f.prev == fPrev)
			e := f.anEdge
			for {
				assert(e.Sym != e)
				assert(e.Sym.Sym == e)
				assert(e.Lnext.Onext.Sym == e)
				assert(e.Onext.Sym.Lnext == e)
				assert(e.Lface == f)
				e = e.Lnext
				if e == f.anEdge { break }
			}
			fPrev = f
		}
		assert(fPrev.next == fHead && fPrev.anEdge == nil)

		vPrev := vHead
		for {
			v := vPrev.next
			if v == vHead { break }
			assert(v.prev == vPrev)
			e := v.anEdge
			for {
				assert(e.Sym != e)
				assert(e.Sym.Sym == e)
				assert(e.Lnext.Onext.Sym == e)
				assert(e.Onext.Sym.Lnext == e)
				assert(e.Org == v)
				e = e.Onext
				if e == v.anEdge { break }
			}
			vPrev = v
		}
		assert(vPrev.next == vHead && vPrev.anEdge == nil)

		ePrev := eHead
		for {
			e := ePrev.next
			if e == eHead { break }
			assert(e.Sym.next == ePrev.Sym)
			assert(e.Sym != e)
			assert(e.Sym.Sym == e)
			assert(e.Org != nil)
			assert(Dst(e) != nil)
			assert(e.Lnext.Onext.Sym == e)
			assert(e.Onext.Sym.Lnext == e)
			ePrev = e
		}
		assert(ePrev.next == eHead &&
			ePrev.next.Sym == &mesh.eHeadSym &&
			ePrev.next.Sym.Sym == ePrev.next &&
			ePrev.next.Org == nil && Dst(ePrev.next) == nil &&
			ePrev.next.Lface == nil && Rface(ePrev.next) == nil)
	}
}

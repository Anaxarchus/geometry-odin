package libtess2_port

import "core:math"

ActiveRegion :: struct {
	eUp:          ^HalfEdge,
	nodeUp:       ^DictNode,
	windingNumber: i32,
	inside:       i32,
	sentinel:     i32,
	dirty:        i32,
	fixUpperEdge: i32,
}

RegionBelow :: #force_inline proc(r: ^ActiveRegion) -> ^ActiveRegion {
	return (^ActiveRegion)(dictKey(dictPred(r.nodeUp)))
}
RegionAbove :: #force_inline proc(r: ^ActiveRegion) -> ^ActiveRegion {
	return (^ActiveRegion)(dictKey(dictSucc(r.nodeUp)))
}

@(private="file")
edgeLeq :: proc(tess: ^Tesselator, reg1, reg2: ^ActiveRegion) -> bool {
	event := tess.event
	e1 := reg1.eUp
	e2 := reg2.eUp

	if e1.Sym.Org == event {
		if e2.Sym.Org == event {
			if VertLeq(e1.Org, e2.Org) {
				return EdgeSign(Dst(e2), e1.Org, e2.Org) <= 0
			}
			return EdgeSign(Dst(e1), e2.Org, e1.Org) >= 0
		}
		return EdgeSign(Dst(e2), event, e2.Org) <= 0
	}
	if e2.Sym.Org == event {
		return EdgeSign(Dst(e1), event, e1.Org) >= 0
	}
	t1 := EdgeEval(Dst(e1), event, e1.Org)
	t2 := EdgeEval(Dst(e2), event, e2.Org)
	return t1 >= t2
}

@(private="file")
deleteRegion :: proc(tess: ^Tesselator, reg: ^ActiveRegion) {
	if reg.fixUpperEdge != 0 {
	}
	reg.eUp.activeRegion = nil
	dictDelete(tess.dict, reg.nodeUp)
	bucketFree(tess.regionPool, reg)
}

@(private="file")
fixUpperEdge :: proc(tess: ^Tesselator, reg: ^ActiveRegion, newEdge: ^HalfEdge) -> bool {
	if !tessMeshDelete(tess.mesh, reg.eUp) { return false }
	reg.fixUpperEdge      = 0
	reg.eUp               = newEdge
	newEdge.activeRegion  = reg
	return true
}

@(private="file")
topLeftRegion :: proc(tess: ^Tesselator, reg_: ^ActiveRegion) -> (^ActiveRegion, bool) {
	reg := reg_
	org := reg.eUp.Org
	for {
		reg = RegionAbove(reg)
		if reg.eUp.Org != org { break }
	}
	if reg.fixUpperEdge != 0 {
		e := tessMeshConnect(tess.mesh, RegionBelow(reg).eUp.Sym, reg.eUp.Lnext)
		if e == nil { return nil, false }
		if !fixUpperEdge(tess, reg, e) { return nil, false }
		reg = RegionAbove(reg)
	}
	return reg, true
}

@(private="file")
topRightRegion :: proc(reg_: ^ActiveRegion) -> ^ActiveRegion {
	reg := reg_
	dst := Dst(reg.eUp)
	for {
		reg = RegionAbove(reg)
		if Dst(reg.eUp) != dst { break }
	}
	return reg
}

@(private="file")
addRegionBelow :: proc(tess: ^Tesselator, regAbove: ^ActiveRegion, eNewUp: ^HalfEdge) -> (^ActiveRegion, bool) {
	regNew := (^ActiveRegion)(bucketAlloc(tess.regionPool))
	if regNew == nil { return nil, false }
	regNew.eUp          = eNewUp
	regNew.nodeUp       = dictInsertBefore(tess.dict, regAbove.nodeUp, regNew)
	if regNew.nodeUp == nil { return nil, false }
	regNew.fixUpperEdge = 0
	regNew.sentinel     = 0
	regNew.dirty        = 0
	eNewUp.activeRegion = regNew
	return regNew, true
}

@(private="file")
isWindingInside :: proc(tess: ^Tesselator, n: i32) -> bool {
	switch tess.windingRule {
	case .Odd:         return (n & 1) != 0
	case .Nonzero:     return n != 0
	case .Positive:    return n > 0
	case .Negative:    return n < 0
	case .Abs_Geq_Two: return n >= 2 || n <= -2
	}
	return false
}

@(private="file")
computeWinding :: proc(tess: ^Tesselator, reg: ^ActiveRegion) {
	reg.windingNumber = RegionAbove(reg).windingNumber + reg.eUp.winding
	reg.inside = isWindingInside(tess, reg.windingNumber) ? 1 : 0
}

@(private="file")
finishRegion :: proc(tess: ^Tesselator, reg: ^ActiveRegion) {
	e := reg.eUp
	f := e.Lface
	f.inside = i8(reg.inside)
	f.anEdge = e
	deleteRegion(tess, reg)
}

@(private="file")
finishLeftRegions :: proc(tess: ^Tesselator, regFirst, regLast: ^ActiveRegion) -> (^HalfEdge, bool) {
	regPrev := regFirst
	ePrev   := regFirst.eUp
	for regPrev != regLast {
		regPrev.fixUpperEdge = 0
		reg := RegionBelow(regPrev)
		e   := reg.eUp
		if e.Org != ePrev.Org {
			if reg.fixUpperEdge == 0 {
				finishRegion(tess, regPrev)
				break
			}
			e2 := tessMeshConnect(tess.mesh, Lprev(ePrev), e.Sym)
			if e2 == nil { return nil, false }
			if !fixUpperEdge(tess, reg, e2) { return nil, false }
			e = e2  // update local e after fixUpperEdge
		}
		if ePrev.Onext != reg.eUp {
			if !tessMeshSplice(tess.mesh, Oprev(reg.eUp), reg.eUp) { return nil, false }
			if !tessMeshSplice(tess.mesh, ePrev, reg.eUp)           { return nil, false }
		}
		finishRegion(tess, regPrev)
		ePrev  = reg.eUp
		regPrev = reg
	}
	return ePrev, true
}

@(private="file")
addRightEdges :: proc(tess: ^Tesselator, regUp: ^ActiveRegion,
                      eFirst, eLast, eTopLeft: ^HalfEdge, cleanUp: bool) -> bool {
	eTopLeft := eTopLeft
	firstTime := true

	e := eFirst
	for {
		_, ok := addRegionBelow(tess, regUp, e.Sym)
		if !ok { return false }
		e = e.Onext
		if e == eLast { break }
	}

	if eTopLeft == nil {
		eTopLeft = RegionBelow(regUp).eUp.Sym.Onext  // Rprev(RegionBelow(regUp).eUp)
	}
	regPrev := regUp
	ePrev   := eTopLeft
	for {
		reg := RegionBelow(regPrev)
		e   = reg.eUp.Sym
		if e.Org != ePrev.Org { break }

		if e.Onext != ePrev {
			if !tessMeshSplice(tess.mesh, Oprev(e), e) { return false }
			if !tessMeshSplice(tess.mesh, Oprev(ePrev), e) { return false }
		}
		reg.windingNumber = regPrev.windingNumber - e.winding
		reg.inside        = isWindingInside(tess, reg.windingNumber) ? 1 : 0

		regPrev.dirty = 1
		if !firstTime {
			action, ok := checkForRightSplice(tess, regPrev)
			if !ok { return false }
			if action {
				AddWinding(e, ePrev)
				deleteRegion(tess, regPrev)
				if !tessMeshDelete(tess.mesh, ePrev) { return false }
			}
		}
		firstTime = false
		regPrev   = reg
		ePrev     = e
	}
	regPrev.dirty = 1

	if cleanUp {
		if !walkDirtyRegions(tess, regPrev) { return false }
	}
	return true
}

@(private="file")
reg_winding_below :: proc(reg: ^ActiveRegion) -> i32 {
	return RegionBelow(reg).windingNumber
}

@(private="file")
spliceMergeVertices :: proc(tess: ^Tesselator, e1, e2: ^HalfEdge) -> bool {
	return tessMeshSplice(tess.mesh, e1, e2)
}

@(private="file")
vertexWeights :: proc(isect, org, dst: ^Vertex, weights: []Real) {
	t1 := VertL1dist(org, isect)
	t2 := VertL1dist(dst, isect)
	weights[0] = Real(0.5) * t2 / (t1 + t2)
	weights[1] = Real(0.5) * t1 / (t1 + t2)
	isect.coords[0] += weights[0]*org.coords[0] + weights[1]*dst.coords[0]
	isect.coords[1] += weights[0]*org.coords[1] + weights[1]*dst.coords[1]
	isect.coords[2] += weights[0]*org.coords[2] + weights[1]*dst.coords[2]
}

@(private="file")
getIntersectData :: proc(tess: ^Tesselator, isect, orgUp, dstUp, orgLo, dstLo: ^Vertex) {
	weights: [4]Real
	isect.coords[0] = 0; isect.coords[1] = 0; isect.coords[2] = 0
	isect.idx = TESS_UNDEF
	vertexWeights(isect, orgUp, dstUp, weights[0:2])
	vertexWeights(isect, orgLo, dstLo, weights[2:4])
}

@(private="file")
checkForRightSplice :: proc(tess: ^Tesselator, regUp: ^ActiveRegion) -> (action: bool, ok: bool) {
	regLo := RegionBelow(regUp)
	eUp   := regUp.eUp
	eLo   := regLo.eUp

	if VertLeq(eUp.Org, eLo.Org) {
		if EdgeSign(Dst(eLo), eUp.Org, eLo.Org) > 0 { return false, true }
		if !VertEq(eUp.Org, eLo.Org) {
			if tessMeshSplitEdge(tess.mesh, eLo.Sym) == nil { return false, false }
			if !tessMeshSplice(tess.mesh, eUp, Oprev(eLo)) { return false, false }
			regUp.dirty = 1; regLo.dirty = 1
		} else if eUp.Org != eLo.Org {
			pqDelete(tess.pq, eUp.Org.pqHandle)
			if !spliceMergeVertices(tess, Oprev(eLo), eUp) { return false, false }
		}
	} else {
		if EdgeSign(Dst(eUp), eLo.Org, eUp.Org) < 0 { return false, true }
		regUp.dirty = 1
		above := RegionAbove(regUp)
		if above != nil { above.dirty = 1 }
		if tessMeshSplitEdge(tess.mesh, eUp.Sym) == nil { return false, false }
		if !tessMeshSplice(tess.mesh, Oprev(eLo), eUp) { return false, false }
	}
	return true, true
}

@(private="file")
checkForLeftSplice :: proc(tess: ^Tesselator, regUp: ^ActiveRegion) -> (action: bool, ok: bool) {
	regLo := RegionBelow(regUp)
	eUp   := regUp.eUp
	eLo   := regLo.eUp


	if VertLeq(Dst(eUp), Dst(eLo)) {
		if EdgeSign(Dst(eUp), Dst(eLo), eUp.Org) < 0 { return false, true }
		regUp.dirty = 1
		above := RegionAbove(regUp)
		if above != nil { above.dirty = 1 }
		e := tessMeshSplitEdge(tess.mesh, eUp)
		if e == nil { return false, false }
		if !tessMeshSplice(tess.mesh, eLo.Sym, e) { return false, false }
		e.Lface.inside = i8(regUp.inside)
	} else {
		if EdgeSign(Dst(eLo), Dst(eUp), eLo.Org) > 0 { return false, true }
		regUp.dirty = 1; regLo.dirty = 1
		e := tessMeshSplitEdge(tess.mesh, eLo)
		if e == nil { return false, false }
		if !tessMeshSplice(tess.mesh, eUp.Lnext, eLo.Sym) { return false, false }
		e.Sym.Lface.inside = i8(regUp.inside)  // e->Rface->inside
	}
	return true, true
}

@(private="file")
checkForIntersect :: proc(tess: ^Tesselator, regUp_: ^ActiveRegion) -> (action: bool, ok: bool) {
	regUp := regUp_
	regLo := RegionBelow(regUp)
	eUp   := regUp.eUp
	eLo   := regLo.eUp
	orgUp := eUp.Org; orgLo := eLo.Org
	dstUp := Dst(eUp); dstLo := Dst(eLo)


	if orgUp == orgLo { return false, true }

	tMinUp := min(orgUp.t, dstUp.t)
	tMaxLo := max(orgLo.t, dstLo.t)
	if tMinUp > tMaxLo { return false, true }

	if VertLeq(orgUp, orgLo) {
		if EdgeSign(dstLo, orgUp, orgLo) > 0 { return false, true }
	} else {
		if EdgeSign(dstUp, orgLo, orgUp) < 0 { return false, true }
	}

	isect: Vertex
	tesedgeIntersect(dstUp, orgUp, dstLo, orgLo, &isect)


	if VertLeq(&isect, tess.event) {
		isect.s = tess.event.s
		isect.t = tess.event.t
	}
	orgMin := VertLeq(orgUp, orgLo) ? orgUp : orgLo
	if VertLeq(orgMin, &isect) {
		isect.s = orgMin.s
		isect.t = orgMin.t
	}

	if VertEq(&isect, orgUp) || VertEq(&isect, orgLo) {
		_, cok := checkForRightSplice(tess, regUp)
		if !cok { return false, false }
		return false, true
	}

	if (!VertEq(dstUp, tess.event) && EdgeSign(dstUp, tess.event, &isect) >= 0) ||
	   (!VertEq(dstLo, tess.event) && EdgeSign(dstLo, tess.event, &isect) <= 0) {
		if dstLo == tess.event {
			if tessMeshSplitEdge(tess.mesh, eUp.Sym) == nil { return false, false }
			if !tessMeshSplice(tess.mesh, eLo.Sym, eUp) { return false, false }
			regUp2, rOk := topLeftRegion(tess, regUp)
			if !rOk { return false, false }
			regUp = regUp2
			eUp   = RegionBelow(regUp).eUp
			_, fOk := finishLeftRegions(tess, RegionBelow(regUp), regLo)
			if !fOk { return false, false }
			if !addRightEdges(tess, regUp, Oprev(eUp), eUp, eUp, true) { return false, false }
			return true, true
		}
		if dstUp == tess.event {
			if tessMeshSplitEdge(tess.mesh, eLo.Sym) == nil { return false, false }
			if !tessMeshSplice(tess.mesh, eUp.Lnext, Oprev(eLo)) { return false, false }
			regLo  = regUp
			regUp  = topRightRegion(regUp)
			e     := RegionBelow(regUp).eUp.Sym.Onext  // Rprev
			regLo.eUp = Oprev(eLo)
			eLo2, fOk := finishLeftRegions(tess, regLo, nil)
			if !fOk { return false, false }
			if !addRightEdges(tess, regUp, eLo2.Onext, Rprev(eUp), e, true) { return false, false }
			return true, true
		}
		if EdgeSign(dstUp, tess.event, &isect) >= 0 {
			RegionAbove(regUp).dirty = 1; regUp.dirty = 1
			if tessMeshSplitEdge(tess.mesh, eUp.Sym) == nil { return false, false }
			eUp.Org.s = tess.event.s; eUp.Org.t = tess.event.t
		}
		if EdgeSign(dstLo, tess.event, &isect) <= 0 {
			regUp.dirty = 1; regLo.dirty = 1
			if tessMeshSplitEdge(tess.mesh, eLo.Sym) == nil { return false, false }
			eLo.Org.s = tess.event.s; eLo.Org.t = tess.event.t
		}
		return false, true
	}

	// General case
	if tessMeshSplitEdge(tess.mesh, eUp.Sym) == nil { return false, false }
	if tessMeshSplitEdge(tess.mesh, eLo.Sym) == nil { return false, false }
	if !tessMeshSplice(tess.mesh, Oprev(eLo), eUp) { return false, false }
	eUp.Org.s = isect.s; eUp.Org.t = isect.t
	eUp.Org.pqHandle = pqInsert(&tess.alloc, tess.pq, eUp.Org)
	if eUp.Org.pqHandle == INV_HANDLE {
		pqDeletePriorityQ(&tess.alloc, tess.pq)
		tess.pq = nil
		return false, false
	}
	getIntersectData(tess, eUp.Org, orgUp, dstUp, orgLo, dstLo)
	RegionAbove(regUp).dirty = 1; regUp.dirty = 1; regLo.dirty = 1
	return false, true
}

@(private="file")
walkDirtyRegions :: proc(tess: ^Tesselator, regUp_: ^ActiveRegion) -> bool {
	regUp := regUp_
	regLo := RegionBelow(regUp)

	for {
		for regLo.dirty != 0 {
			regUp = regLo
			regLo = RegionBelow(regLo)
		}
		if regUp.dirty == 0 {
			regLo = regUp
			regUp  = RegionAbove(regUp)
			if regUp == nil || regUp.dirty == 0 {
				return true
			}
		}
		regUp.dirty = 0
		eUp  := regUp.eUp
		eLo  := regLo.eUp

		if Dst(eUp) != Dst(eLo) {
			action, ok := checkForLeftSplice(tess, regUp)
			if !ok { return false }
			if action {
				if regLo.fixUpperEdge != 0 {
					deleteRegion(tess, regLo)
					if !tessMeshDelete(tess.mesh, eLo) { return false }
					regLo = RegionBelow(regUp)
					eLo   = regLo.eUp
				} else if regUp.fixUpperEdge != 0 {
					deleteRegion(tess, regUp)
					if !tessMeshDelete(tess.mesh, eUp) { return false }
					regUp = RegionAbove(regLo)
					eUp   = regUp.eUp
				}
			}
		}
		if eUp.Org != eLo.Org {
			if Dst(eUp) != Dst(eLo) &&
				regUp.fixUpperEdge == 0 && regLo.fixUpperEdge == 0 &&
				(Dst(eUp) == tess.event || Dst(eLo) == tess.event) {
				action, ok := checkForIntersect(tess, regUp)
				if !ok { return false }
				if action { return true }
			} else {
				_, ok := checkForRightSplice(tess, regUp)
				if !ok { return false }
			}
		}
		if eUp.Org == eLo.Org && Dst(eUp) == Dst(eLo) {
			AddWinding(eLo, eUp)
			deleteRegion(tess, regUp)
			if !tessMeshDelete(tess.mesh, eUp) { return false }
			regUp = RegionAbove(regLo)
		}
	}
}

@(private="file")
connectRightVertex :: proc(tess: ^Tesselator, regUp_: ^ActiveRegion, eBottomLeft_: ^HalfEdge) -> bool {
	regUp      := regUp_
	eBottomLeft := eBottomLeft_
	eTopLeft := eBottomLeft.Onext
	regLo    := RegionBelow(regUp)
	eUp      := regUp.eUp
	eLo      := regLo.eUp
	degenerate := false

	if Dst(eUp) != Dst(eLo) {
		_, ok := checkForIntersect(tess, regUp)
		if !ok { return false }
	}

	if VertEq(eUp.Org, tess.event) {
		if !tessMeshSplice(tess.mesh, Oprev(eTopLeft), eUp) { return false }
		r, ok := topLeftRegion(tess, regUp)
		if !ok { return false }
		regUp    = r
		eTopLeft = RegionBelow(regUp).eUp
		_, fOk := finishLeftRegions(tess, RegionBelow(regUp), regLo)
		if !fOk { return false }
		degenerate = true
	}
	if VertEq(eLo.Org, tess.event) {
		if !tessMeshSplice(tess.mesh, eBottomLeft, Oprev(eLo)) { return false }
		ebl, fOk := finishLeftRegions(tess, regLo, nil)
		if !fOk { return false }
		eBottomLeft = ebl
		degenerate = true
	}
	if degenerate {
		return addRightEdges(tess, regUp, eBottomLeft.Onext, eTopLeft, eTopLeft, true)
	}

	eNew: ^HalfEdge
	if VertLeq(eLo.Org, eUp.Org) {
		eNew = Oprev(eLo)
	} else {
		eNew = eUp
	}
	eNew = tessMeshConnect(tess.mesh, Lprev(eBottomLeft), eNew)
	if eNew == nil { return false }

	if !addRightEdges(tess, regUp, eNew, eNew.Onext, eNew.Onext, false) { return false }
	eNew.Sym.activeRegion.fixUpperEdge = 1
	return walkDirtyRegions(tess, regUp)
}

@(private="file")
connectLeftDegenerate :: proc(tess: ^Tesselator, regUp_: ^ActiveRegion, vEvent: ^Vertex) -> bool {
	regUp := regUp_
	e := regUp.eUp
	if VertEq(e.Org, vEvent) {
		return spliceMergeVertices(tess, e, vEvent.anEdge)
	}

	if !VertEq(Dst(e), vEvent) {
		if tessMeshSplitEdge(tess.mesh, e.Sym) == nil { return false }
		if regUp.fixUpperEdge != 0 {
			if !tessMeshDelete(tess.mesh, e.Onext) { return false }
			regUp.fixUpperEdge = 0
		}
		if !tessMeshSplice(tess.mesh, vEvent.anEdge, e) { return false }
		return sweepEvent(tess, vEvent)
	}

	// vEvent coincides with e->Dst
	regUp = topRightRegion(regUp)
	reg   := RegionBelow(regUp)
	eTopRight := reg.eUp.Sym
	eTopLeft  := eTopRight.Onext
	eLast     := eTopLeft
	if reg.fixUpperEdge != 0 {
		deleteRegion(tess, reg)
		if !tessMeshDelete(tess.mesh, eTopRight) { return false }
		eTopRight = Oprev(eTopLeft)
	}
	if !tessMeshSplice(tess.mesh, vEvent.anEdge, eTopRight) { return false }
	if !EdgeGoesLeft(eTopLeft) { eTopLeft = nil }
	return addRightEdges(tess, regUp, eTopRight.Onext, eLast, eTopLeft, true)
}

@(private="file")
connectLeftVertex :: proc(tess: ^Tesselator, vEvent: ^Vertex) -> bool {
	tmp: ActiveRegion
	tmp.eUp = vEvent.anEdge.Sym
	regUp  := (^ActiveRegion)(dictKey(dictSearch(tess.dict, &tmp)))
	regLo  := RegionBelow(regUp)
	if regLo == nil { return true }
	eUp    := regUp.eUp
	eLo    := regLo.eUp

	if EdgeSign(Dst(eUp), vEvent, eUp.Org) == 0 {
		return connectLeftDegenerate(tess, regUp, vEvent)
	}

	reg: ^ActiveRegion
	if VertLeq(Dst(eLo), Dst(eUp)) {
		reg = regUp
	} else {
		reg = regLo
	}

	if regUp.inside != 0 || reg.fixUpperEdge != 0 {
		eNew: ^HalfEdge
		if reg == regUp {
			eNew = tessMeshConnect(tess.mesh, vEvent.anEdge.Sym, eUp.Lnext)
			if eNew == nil { return false }
		} else {
			tmp := tessMeshConnect(tess.mesh, Dnext(eLo), vEvent.anEdge)
			if tmp == nil { return false }
			eNew = tmp.Sym
		}
		if reg.fixUpperEdge != 0 {
			if !fixUpperEdge(tess, reg, eNew) { return false }
		} else {
			r, ok := addRegionBelow(tess, regUp, eNew)
			if !ok { return false }
			computeWinding(tess, r)
		}
		return sweepEvent(tess, vEvent)
	}

	return addRightEdges(tess, regUp, vEvent.anEdge, vEvent.anEdge, nil, true)
}

@(private="file")
sweepEvent :: proc(tess: ^Tesselator, vEvent: ^Vertex) -> bool {
	tess.event = vEvent

	e := vEvent.anEdge
	for e.activeRegion == nil {
		e = e.Onext
		if e == vEvent.anEdge {
			return connectLeftVertex(tess, vEvent)
		}
	}

	regUp, ok := topLeftRegion(tess, e.activeRegion)
	if !ok { return false }
	reg       := RegionBelow(regUp)
	eTopLeft  := reg.eUp
	eBottomLeft, fOk := finishLeftRegions(tess, reg, nil)
	if !fOk { return false }

	if eBottomLeft.Onext == eTopLeft {
		return connectRightVertex(tess, regUp, eBottomLeft)
	}
	return addRightEdges(tess, regUp, eBottomLeft.Onext, eTopLeft, eTopLeft, true)
}

@(private="file")
addSentinel :: proc(tess: ^Tesselator, smin, smax, t: Real) -> bool {
	reg := (^ActiveRegion)(bucketAlloc(tess.regionPool))
	if reg == nil { return false }

	e := tessMeshMakeEdge(tess.mesh)
	if e == nil { return false }

	e.Org.s = smax; e.Org.t = t
	e.Sym.Org.s = smin; e.Sym.Org.t = t
	tess.event = e.Sym.Org

	reg.eUp          = e
	reg.windingNumber = 0
	reg.inside        = 0
	reg.fixUpperEdge  = 0
	reg.sentinel      = 1
	reg.dirty         = 0
	reg.nodeUp        = dictInsert(tess.dict, reg)
	if reg.nodeUp == nil { return false }
	return true
}

@(private="file")
initEdgeDict :: proc(tess: ^Tesselator) -> bool {
	tess.dict = dictNewDict(&tess.alloc, tess, edgeLeq)
	if tess.dict == nil { return false }

	w := (tess.bmax[0] - tess.bmin[0]) + Real(0.01)
	h := (tess.bmax[1] - tess.bmin[1]) + Real(0.01)

	smin := tess.bmin[0] - w; smax := tess.bmax[0] + w
	tmin := tess.bmin[1] - h; tmax := tess.bmax[1] + h

	if !addSentinel(tess, smin, smax, tmin) { return false }
	if !addSentinel(tess, smin, smax, tmax) { return false }
	return true
}

@(private="file")
doneEdgeDict :: proc(tess: ^Tesselator) {
	for {
		reg := (^ActiveRegion)(dictKey(dictMin(tess.dict)))
		if reg == nil { break }
		deleteRegion(tess, reg)
	}
	dictDeleteDict(&tess.alloc, tess.dict)
}

@(private="file")
removeDegenerateEdges :: proc(tess: ^Tesselator) -> bool {
	eHead := &tess.mesh.eHead
	e     := eHead.next
	for e != eHead {
		eNext  := e.next
		eLnext := e.Lnext
		if VertEq(e.Org, Dst(e)) && e.Lnext.Lnext != e {
			if !spliceMergeVertices(tess, eLnext, e) { return false }
			if !tessMeshDelete(tess.mesh, e) { return false }
			e      = eLnext
			eLnext = e.Lnext
		}
		if eLnext.Lnext == e {
			if eLnext != e {
				if eLnext == eNext || eLnext == eNext.Sym { eNext = eNext.next }
				if !tessMeshDelete(tess.mesh, eLnext) { return false }
			}
			if e == eNext || e == eNext.Sym { eNext = eNext.next }
			if !tessMeshDelete(tess.mesh, e) { return false }
		}
		e = eNext
	}
	return true
}

@(private="file")
initPriorityQ :: proc(tess: ^Tesselator) -> bool {
	vHead := &tess.mesh.vHead
	vertexCount := i32(0)
	v := vHead.next
	for v != vHead { vertexCount += 1; v = v.next }
	vertexCount += max(i32(8), tess.alloc.extraVertices)

	pq := pqNewPriorityQ(&tess.alloc, vertexCount)
	if pq == nil { return false }
	tess.pq = pq

	v = vHead.next
	for v != vHead {
		v.pqHandle = pqInsert(&tess.alloc, pq, v)
		if v.pqHandle == INV_HANDLE { break }
		v = v.next
	}
	if v != vHead || !pqInit(&tess.alloc, pq) {
		pqDeletePriorityQ(&tess.alloc, tess.pq)
		tess.pq = nil
		return false
	}
	return true
}

@(private="file")
donePriorityQ :: proc(tess: ^Tesselator) {
	pqDeletePriorityQ(&tess.alloc, tess.pq)
}

@(private="file")
removeDegenerateFaces :: proc(tess: ^Tesselator, mesh: ^Mesh) -> bool {
	f := mesh.fHead.next
	for f != &mesh.fHead {
		fNext := f.next
		e := f.anEdge
		if e.Lnext.Lnext == e {
			AddWinding(e.Onext, e)
			if !tessMeshDelete(tess.mesh, e) { return false }
		}
		f = fNext
	}
	return true
}

tessComputeInterior :: proc(tess: ^Tesselator) -> bool {
	if !removeDegenerateEdges(tess) { return false }
	if !initPriorityQ(tess)        { return false }
	if !initEdgeDict(tess)         { return false }

	for {
		v := (^Vertex)(pqExtractMin(tess.pq))
		if v == nil { break }
		for {
			vNext := (^Vertex)(pqMinimum(tess.pq))
			if vNext == nil || !VertEq(vNext, v) { break }
			vNext = (^Vertex)(pqExtractMin(tess.pq))
			if !spliceMergeVertices(tess, v.anEdge, vNext.anEdge) { return false }
		}
		if !sweepEvent(tess, v) { return false }
	}

	tess.event = ((^ActiveRegion)(dictKey(dictMin(tess.dict)))).eUp.Org
	doneEdgeDict(tess)
	donePriorityQ(tess)

	if !removeDegenerateFaces(tess, tess.mesh) { return false }
	tessMeshCheckMesh(tess.mesh)
	return true
}

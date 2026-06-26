package libtess2_port

import "core:math"

// Navigation helpers — expand C macros that act as virtual fields on HalfEdge.
// Read-only accessors (left-hand-side assignments must be done by expanding manually).
Rface :: #force_inline proc(e: ^HalfEdge) -> ^Face     { return e.Sym.Lface }
Dst   :: #force_inline proc(e: ^HalfEdge) -> ^Vertex   { return e.Sym.Org }
Oprev :: #force_inline proc(e: ^HalfEdge) -> ^HalfEdge { return e.Sym.Lnext }
Lprev :: #force_inline proc(e: ^HalfEdge) -> ^HalfEdge { return e.Onext.Sym }
Dprev :: #force_inline proc(e: ^HalfEdge) -> ^HalfEdge { return e.Lnext.Sym }
Rprev :: #force_inline proc(e: ^HalfEdge) -> ^HalfEdge { return e.Sym.Onext }
Dnext :: #force_inline proc(e: ^HalfEdge) -> ^HalfEdge { return Rprev(e).Sym }
Rnext :: #force_inline proc(e: ^HalfEdge) -> ^HalfEdge { return Oprev(e).Sym }

// Vertex ordering on the sweep plane
VertEq :: #force_inline proc(u, v: ^Vertex) -> bool {
	return u.s == v.s && u.t == v.t
}

VertLeq :: #force_inline proc(u, v: ^Vertex) -> bool {
	return u.s < v.s || (u.s == v.s && u.t <= v.t)
}

TransLeq :: #force_inline proc(u, v: ^Vertex) -> bool {
	return u.t < v.t || (u.t == v.t && u.s <= v.s)
}

EdgeGoesLeft  :: #force_inline proc(e: ^HalfEdge) -> bool { return VertLeq(Dst(e), e.Org) }
EdgeGoesRight :: #force_inline proc(e: ^HalfEdge) -> bool { return VertLeq(e.Org, Dst(e)) }
EdgeIsInternal :: #force_inline proc(e: ^HalfEdge) -> bool {
	return Rface(e) != nil && Rface(e).inside != 0
}

VertL1dist :: #force_inline proc(u, v: ^Vertex) -> Real {
	return abs(u.s - v.s) + abs(u.t - v.t)
}

VertCCW :: #force_inline proc(u, v, w: ^Vertex) -> bool {
	return tesvertCCW(u, v, w)
}

AddWinding :: #force_inline proc(eDst, eSrc: ^HalfEdge) {
	eDst.winding     += eSrc.winding
	eDst.Sym.winding += eSrc.Sym.winding
}

// Public geometry functions

vertLeq :: proc(u, v: ^Vertex) -> bool {
	return VertLeq(u, v)
}

tesedgeEval :: proc(u, v, w: ^Vertex) -> Real {
	gapL := v.s - u.s
	gapR := w.s - v.s
	if gapL + gapR > 0 {
		if gapL < gapR {
			return (v.t - u.t) + (u.t - w.t) * (gapL / (gapL + gapR))
		} else {
			return (v.t - w.t) + (w.t - u.t) * (gapR / (gapL + gapR))
		}
	}
	return 0
}

tesedgeSign :: proc(u, v, w: ^Vertex) -> Real {
	gapL := v.s - u.s
	gapR := w.s - v.s
	if gapL + gapR > 0 {
		result := (v.t - w.t) * gapL + (v.t - u.t) * gapR
		return math.is_nan(result) ? 0 : result
	}
	return 0
}

testransEval :: proc(u, v, w: ^Vertex) -> Real {
	gapL := v.t - u.t
	gapR := w.t - v.t
	if gapL + gapR > 0 {
		if gapL < gapR {
			return (v.s - u.s) + (u.s - w.s) * (gapL / (gapL + gapR))
		} else {
			return (v.s - w.s) + (w.s - u.s) * (gapR / (gapL + gapR))
		}
	}
	return 0
}

testransSign :: proc(u, v, w: ^Vertex) -> Real {
	gapL := v.t - u.t
	gapR := w.t - v.t
	if gapL + gapR > 0 {
		return (v.s - w.s) * gapL + (v.s - u.s) * gapR
	}
	return 0
}

tesvertCCW :: proc(u, v, w: ^Vertex) -> bool {
	return (u.s*(v.t - w.t) + v.s*(w.t - u.t) + w.s*(u.t - v.t)) >= 0
}

@(private="file")
realInterpolate :: proc(a_, x: Real, b_, y: Real) -> Real {
	a := a_ < 0 ? Real(0) : a_
	b := b_ < 0 ? Real(0) : b_
	if a <= b {
		if b == 0 { return x / 2 + y / 2 }
		return x + (y - x) * (a / (a + b))
	}
	return y + (x - y) * (b / (a + b))
}

tesedgeIntersect :: proc(o1_, d1_, o2_, d2_: ^Vertex, v: ^Vertex) {
	o1 := o1_; d1 := d1_; o2 := o2_; d2 := d2_

	if !VertLeq(o1, d1) { o1, d1 = d1, o1 }
	if !VertLeq(o2, d2) { o2, d2 = d2, o2 }
	if !VertLeq(o1, o2) { o1, o2 = o2, o1; d1, d2 = d2, d1 }

	if !VertLeq(o2, d1) {
		v.s = o2.s / 2 + d1.s / 2
	} else if VertLeq(d1, d2) {
		z1 := tesedgeEval(o1, o2, d1)
		z2 := tesedgeEval(o2, d1, d2)
		if z1 + z2 < 0 { z1 = -z1; z2 = -z2 }
		v.s = realInterpolate(z1, o2.s, z2, d1.s)
	} else {
		z1 := tesedgeSign(o1, o2, d1)
		z2 := -tesedgeSign(o1, d2, d1)
		if z1 + z2 < 0 { z1 = -z1; z2 = -z2 }
		v.s = realInterpolate(z1, o2.s, z2, d2.s)
	}

	// Repeat for t
	if !TransLeq(o1, d1) { o1, d1 = d1, o1 }
	if !TransLeq(o2, d2) { o2, d2 = d2, o2 }
	if !TransLeq(o1, o2) { o1, o2 = o2, o1; d1, d2 = d2, d1 }

	if !TransLeq(o2, d1) {
		v.t = o2.t / 2 + d1.t / 2
	} else if TransLeq(d1, d2) {
		z1 := testransEval(o1, o2, d1)
		z2 := testransEval(o2, d1, d2)
		if z1 + z2 < 0 { z1 = -z1; z2 = -z2 }
		v.t = realInterpolate(z1, o2.t, z2, d1.t)
	} else {
		z1 := testransSign(o1, o2, d1)
		z2 := -testransSign(o1, d2, d1)
		if z1 + z2 < 0 { z1 = -z1; z2 = -z2 }
		v.t = realInterpolate(z1, o2.t, z2, d2.t)
	}
}

@(private="file")
inCircle :: proc(v, v0, v1, v2: ^Vertex) -> Real {
	adx := v0.s - v.s; ady := v0.t - v.t
	bdx := v1.s - v.s; bdy := v1.t - v.t
	cdx := v2.s - v.s; cdy := v2.t - v.t
	abdet := adx*bdy - bdx*ady
	bcdet := bdx*cdy - cdx*bdy
	cadet := cdx*ady - adx*cdy
	alift := adx*adx + ady*ady
	blift := bdx*bdx + bdy*bdy
	clift := cdx*cdx + cdy*cdy
	return alift*bcdet + blift*cadet + clift*abdet
}

tesedgeIsLocallyDelaunay :: proc(e: ^HalfEdge) -> bool {
	return inCircle(e.Sym.Lnext.Lnext.Org, e.Lnext.Org, e.Lnext.Lnext.Org, e.Org) < 0
}

// Macro aliases matching geom.h usage in sweep/tess
EdgeEval  :: tesedgeEval
EdgeSign  :: tesedgeSign
TransEval :: testransEval
TransSign :: testransSign

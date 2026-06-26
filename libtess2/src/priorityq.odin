package libtess2_port

PQkey    :: rawptr
PQhandle :: i32

INV_HANDLE :: PQhandle(0x0fffffff)

PQnode :: struct {
	handle: PQhandle,
}

PQhandleElem :: struct {
	key:  PQkey,
	node: PQhandle,
}

PriorityQHeap :: struct {
	nodes:       []PQnode,
	handles:     []PQhandleElem,
	size:        i32,
	max:         i32,
	freeList:    PQhandle,
	initialized: bool,
}

PriorityQ :: struct {
	heap:        ^PriorityQHeap,
	keys:        []PQkey,
	order:       []^PQkey,
	size:        i32,
	max:         i32,
	initialized: bool,
}

// Inlined comparator — avoids indirect call overhead, matches C's LEQ macro
@(private="file")
pq_leq :: #force_inline proc(x, y: PQkey) -> bool {
	return vertLeq((^Vertex)(x), (^Vertex)(y))
}

// ---- Heap internals ----

@(private="file")
floatDown :: proc(pq: ^PriorityQHeap, curr_: i32) {
	curr  := curr_
	n     := pq.nodes
	h     := pq.handles
	hCurr := n[curr].handle
	for {
		child := curr << 1
		if child < pq.size && pq_leq(h[n[child+1].handle].key, h[n[child].handle].key) {
			child += 1
		}
		assert(child <= pq.max)
		hChild := n[child].handle
		if child > pq.size || pq_leq(h[hCurr].key, h[hChild].key) {
			n[curr].handle = hCurr
			h[hCurr].node  = curr
			break
		}
		n[curr].handle  = hChild
		h[hChild].node  = curr
		curr = child
	}
}

@(private="file")
floatUp :: proc(pq: ^PriorityQHeap, curr_: i32) {
	curr   := curr_
	n      := pq.nodes
	h      := pq.handles
	hCurr  := n[curr].handle
	for {
		parent  := curr >> 1
		hParent := n[parent].handle
		if parent == 0 || pq_leq(h[hParent].key, h[hCurr].key) {
			n[curr].handle  = hCurr
			h[hCurr].node   = curr
			break
		}
		n[curr].handle   = hParent
		h[hParent].node  = curr
		curr = parent
	}
}

// ---- Heap public ----

pqHeapNewPriorityQ :: proc(alloc: ^Alloc, size: i32) -> ^PriorityQHeap {
	pq := (^PriorityQHeap)(alloc.memalloc(alloc.userData, size_of(PriorityQHeap)))
	if pq == nil { return nil }

	raw_nodes := alloc.memalloc(alloc.userData, u32((size + 1) * size_of(PQnode)))
	if raw_nodes == nil {
		alloc.memfree(alloc.userData, pq)
		return nil
	}
	raw_handles := alloc.memalloc(alloc.userData, u32((size + 1) * size_of(PQhandleElem)))
	if raw_handles == nil {
		alloc.memfree(alloc.userData, raw_nodes)
		alloc.memfree(alloc.userData, pq)
		return nil
	}

	pq.nodes            = (([^]PQnode)(raw_nodes))[:size + 1]
	pq.handles          = (([^]PQhandleElem)(raw_handles))[:size + 1]
	pq.size             = 0
	pq.max              = size
	pq.freeList         = 0
	pq.initialized      = false
	pq.nodes[1].handle  = 1
	pq.handles[1].key   = nil
	return pq
}

pqHeapDeletePriorityQ :: proc(alloc: ^Alloc, pq: ^PriorityQHeap) {
	alloc.memfree(alloc.userData, raw_data(pq.handles))
	alloc.memfree(alloc.userData, raw_data(pq.nodes))
	alloc.memfree(alloc.userData, pq)
}

pqHeapInit :: proc(pq: ^PriorityQHeap) {
	for i := pq.size; i >= 1; i -= 1 {
		floatDown(pq, i)
	}
	pq.initialized = true
}

pqHeapMinimum :: #force_inline proc(pq: ^PriorityQHeap) -> PQkey {
	return pq.handles[pq.nodes[1].handle].key
}

pqHeapIsEmpty :: #force_inline proc(pq: ^PriorityQHeap) -> bool {
	return pq.size == 0
}

pqHeapInsert :: proc(alloc: ^Alloc, pq: ^PriorityQHeap, keyNew: PQkey) -> PQhandle {
	pq.size += 1
	curr := pq.size
	if (curr * 2) > pq.max {
		if alloc.memrealloc == nil { return INV_HANDLE }
		save_nodes   := pq.nodes
		save_handles := pq.handles
		pq.max <<= 1
		raw := alloc.memrealloc(alloc.userData, raw_data(pq.nodes), u32((pq.max + 1) * size_of(PQnode)))
		if raw == nil { pq.nodes = save_nodes; return INV_HANDLE }
		pq.nodes = (([^]PQnode)(raw))[:pq.max + 1]
		raw = alloc.memrealloc(alloc.userData, raw_data(pq.handles), u32((pq.max + 1) * size_of(PQhandleElem)))
		if raw == nil { pq.handles = save_handles; return INV_HANDLE }
		pq.handles = (([^]PQhandleElem)(raw))[:pq.max + 1]
	}

	free: PQhandle
	if pq.freeList == 0 {
		free = PQhandle(curr)
	} else {
		free = pq.freeList
		pq.freeList = pq.handles[free].node
	}

	pq.nodes[curr].handle  = free
	pq.handles[free].node  = curr
	pq.handles[free].key   = keyNew

	if pq.initialized { floatUp(pq, curr) }
	assert(free != INV_HANDLE)
	return free
}

pqHeapExtractMin :: proc(pq: ^PriorityQHeap) -> PQkey {
	n    := pq.nodes
	h    := pq.handles
	hMin := n[1].handle
	min  := h[hMin].key

	if pq.size > 0 {
		n[1].handle         = n[pq.size].handle
		h[n[1].handle].node = 1
		h[hMin].key         = nil
		h[hMin].node        = pq.freeList
		pq.freeList         = hMin
		pq.size -= 1
		if pq.size > 0 { floatDown(pq, 1) }
	}
	return min
}

pqHeapDelete :: proc(pq: ^PriorityQHeap, hCurr: PQhandle) {
	n    := pq.nodes
	h    := pq.handles
	assert(hCurr >= 1 && hCurr <= PQhandle(pq.max) && h[hCurr].key != nil)
	curr := h[hCurr].node
	n[curr].handle         = n[pq.size].handle
	h[n[curr].handle].node = curr
	pq.size -= 1
	if curr <= pq.size {
		if curr <= 1 || pq_leq(h[n[curr>>1].handle].key, h[n[curr].handle].key) {
			floatDown(pq, curr)
		} else {
			floatUp(pq, curr)
		}
	}
	h[hCurr].key  = nil
	h[hCurr].node = pq.freeList
	pq.freeList    = hCurr
}

// ---- Sort (initial-insert phase + heap) ----

pqNewPriorityQ :: proc(alloc: ^Alloc, size: i32) -> ^PriorityQ {
	pq := (^PriorityQ)(alloc.memalloc(alloc.userData, size_of(PriorityQ)))
	if pq == nil { return nil }

	pq.heap = pqHeapNewPriorityQ(alloc, size)
	if pq.heap == nil {
		alloc.memfree(alloc.userData, pq)
		return nil
	}

	raw_keys := alloc.memalloc(alloc.userData, u32(size) * size_of(PQkey))
	if raw_keys == nil {
		pqHeapDeletePriorityQ(alloc, pq.heap)
		alloc.memfree(alloc.userData, pq)
		return nil
	}

	pq.keys        = (([^]PQkey)(raw_keys))[:size]
	pq.size        = 0
	pq.max         = size
	pq.initialized = false
	pq.order       = nil
	return pq
}

pqDeletePriorityQ :: proc(alloc: ^Alloc, pq: ^PriorityQ) {
	assert(pq != nil)
	if pq.heap  != nil { pqHeapDeletePriorityQ(alloc, pq.heap) }
	if pq.order != nil { alloc.memfree(alloc.userData, raw_data(pq.order)) }
	if pq.keys  != nil { alloc.memfree(alloc.userData, raw_data(pq.keys)) }
	alloc.memfree(alloc.userData, pq)
}

pqInit :: proc(alloc: ^Alloc, pq: ^PriorityQ) -> bool {
	raw_order := alloc.memalloc(alloc.userData, u32(pq.size + 1) * size_of(^PQkey))
	if raw_order == nil { return false }
	pq.order = (([^]^PQkey)(raw_order))[:pq.size + 1]

	// Fill indirect pointers into keys array
	for i in 0..<int(pq.size) {
		pq.order[i] = &pq.keys[i]
	}

	// Iterative randomized quicksort — sort DESCENDING so min is at the end
	seed := u32(2016473283)
	QSFrame :: struct { p, r: int }
	stack: [50]QSFrame
	top := 0
	stack[top] = {0, int(pq.size) - 1}
	top += 1

	for top > 0 {
		top -= 1
		p := stack[top].p
		r := stack[top].r

		for r > p + 10 {
			seed = seed * 1539415821 + 1
			idx := p + int(seed % u32(r - p + 1))
			piv := pq.order[idx]
			pq.order[idx] = pq.order[p]
			pq.order[p]   = piv
			i := p - 1
			j := r + 1
			for {
				// advance i while order[i] > piv  (GT)
				i += 1
				for !pq_leq(pq.order[i]^, piv^) { i += 1 }
				// advance j while order[j] < piv  (LT)
				j -= 1
				for !pq_leq(piv^, pq.order[j]^) { j -= 1 }
				if i >= j { break }
				pq.order[i], pq.order[j] = pq.order[j], pq.order[i]
			}
			// Standard Hoare partition: we break *before* swapping when the
			// indices cross, so (unlike the C reference's do-while form) there
			// is no final in-loop swap to undo here.
			if i - p < r - j {
				stack[top] = {j + 1, r}; top += 1
				r = i - 1
			} else {
				stack[top] = {p, i - 1}; top += 1
				p = j + 1
			}
		}
		// Insertion sort for small segments
		for i := p + 1; i <= r; i += 1 {
			piv := pq.order[i]
			j   := i
			// move j left while order[j-1] < piv  (LT)
			for j > p && !pq_leq(piv^, pq.order[j-1]^) {
				pq.order[j] = pq.order[j-1]
				j -= 1
			}
			pq.order[j] = piv
		}
	}

	pq.max         = pq.size
	pq.initialized = true
	pqHeapInit(pq.heap)
	return true
}

pqInsert :: proc(alloc: ^Alloc, pq: ^PriorityQ, keyNew: PQkey) -> PQhandle {
	if pq.initialized {
		return pqHeapInsert(alloc, pq.heap, keyNew)
	}
	curr := pq.size
	pq.size += 1
	if pq.size >= pq.max {
		if alloc.memrealloc == nil { return INV_HANDLE }
		save_keys := pq.keys
		pq.max <<= 1
		raw := alloc.memrealloc(alloc.userData, raw_data(pq.keys), u32(pq.max) * size_of(PQkey))
		if raw == nil { pq.keys = save_keys; return INV_HANDLE }
		pq.keys = (([^]PQkey)(raw))[:pq.max]
	}
	assert(curr != INV_HANDLE)
	pq.keys[curr] = keyNew
	return -(curr + 1)
}

pqExtractMin :: proc(pq: ^PriorityQ) -> PQkey {
	if pq.size == 0 {
		return pqHeapExtractMin(pq.heap)
	}
	sortMin := pq.order[pq.size - 1]^
	if !pqHeapIsEmpty(pq.heap) {
		heapMin := pqHeapMinimum(pq.heap)
		if pq_leq(heapMin, sortMin) {
			return pqHeapExtractMin(pq.heap)
		}
	}
	for {
		pq.size -= 1
		if pq.size <= 0 || pq.order[pq.size - 1]^ != nil { break }
	}
	return sortMin
}

pqMinimum :: proc(pq: ^PriorityQ) -> PQkey {
	if pq.size == 0 {
		return pqHeapMinimum(pq.heap)
	}
	sortMin := pq.order[pq.size - 1]^
	if !pqHeapIsEmpty(pq.heap) {
		heapMin := pqHeapMinimum(pq.heap)
		if pq_leq(heapMin, sortMin) {
			return heapMin
		}
	}
	return sortMin
}

pqIsEmpty :: proc(pq: ^PriorityQ) -> bool {
	return pq.size == 0 && pqHeapIsEmpty(pq.heap)
}

pqDelete :: proc(pq: ^PriorityQ, curr_: PQhandle) {
	if curr_ >= 0 {
		pqHeapDelete(pq.heap, curr_)
		return
	}
	curr := -(curr_ + 1)
	assert(curr < PQhandle(pq.max) && pq.keys[curr] != nil)
	pq.keys[curr] = nil
	for pq.size > 0 && pq.order[pq.size - 1]^ == nil {
		pq.size -= 1
	}
}

package libtess2_port

DictNode :: struct {
	key:  ^ActiveRegion,
	next: ^DictNode,
	prev: ^DictNode,
}

Dict :: struct {
	head:     DictNode,
	frame:    rawptr,
	nodePool: ^BucketAlloc,
	leq:      proc(frame: ^Tesselator, key1: ^ActiveRegion, key2: ^ActiveRegion) -> bool,
}

dictKey  :: #force_inline proc(n: ^DictNode) -> ^ActiveRegion { return n.key }
dictSucc :: #force_inline proc(n: ^DictNode) -> ^DictNode     { return n.next }
dictPred :: #force_inline proc(n: ^DictNode) -> ^DictNode     { return n.prev }
dictMin  :: #force_inline proc(d: ^Dict) -> ^DictNode         { return d.head.next }
dictMax  :: #force_inline proc(d: ^Dict) -> ^DictNode         { return d.head.prev }

dictInsert :: #force_inline proc(d: ^Dict, k: ^ActiveRegion) -> ^DictNode {
	return dictInsertBefore(d, &d.head, k)
}

dictNewDict :: proc(alloc: ^Alloc, frame: rawptr, leq: proc(^Tesselator, ^ActiveRegion, ^ActiveRegion) -> bool) -> ^Dict {
	dict := (^Dict)(alloc.memalloc(alloc.userData, size_of(Dict)))
	if dict == nil { return nil }

	head := &dict.head
	head.key  = nil
	head.next = head
	head.prev = head

	dict.frame = frame
	dict.leq   = leq

	if alloc.dictNodeBucketSize < 16   { alloc.dictNodeBucketSize = 16 }
	if alloc.dictNodeBucketSize > 4096 { alloc.dictNodeBucketSize = 4096 }
	dict.nodePool = createBucketAlloc(alloc, "Dict", size_of(DictNode), u32(alloc.dictNodeBucketSize))

	return dict
}

dictDeleteDict :: proc(alloc: ^Alloc, dict: ^Dict) {
	deleteBucketAlloc(dict.nodePool)
	alloc.memfree(alloc.userData, dict)
}

dictInsertBefore :: proc(dict: ^Dict, node: ^DictNode, key: ^ActiveRegion) -> ^DictNode {
	node := node
	for {
		node = node.prev
		if node.key == nil || dict.leq((^Tesselator)(dict.frame), node.key, key) { break }
	}

	newNode := (^DictNode)(bucketAlloc(dict.nodePool))
	if newNode == nil { return nil }

	newNode.key       = key
	newNode.next      = node.next
	node.next.prev    = newNode
	newNode.prev      = node
	node.next         = newNode

	return newNode
}

dictDelete :: proc(dict: ^Dict, node: ^DictNode) {
	node.next.prev = node.prev
	node.prev.next = node.next
	bucketFree(dict.nodePool, node)
}

dictSearch :: proc(dict: ^Dict, key: ^ActiveRegion) -> ^DictNode {
	node := &dict.head
	for {
		node = node.next
		if node.key == nil || dict.leq((^Tesselator)(dict.frame), key, node.key) { break }
	}
	return node
}

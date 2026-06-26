package libtess2_port

Bucket :: struct {
	next: ^Bucket,
}

BucketAlloc :: struct {
	freelist:   rawptr,
	buckets:    ^Bucket,
	itemSize:   u32,
	bucketSize: u32,
	name:       cstring,
	alloc:      ^Alloc,
}

@(private)
createBucket :: proc(ba: ^BucketAlloc) -> bool {
	size := u32(size_of(Bucket)) + ba.itemSize * ba.bucketSize
	bucket := (^Bucket)(ba.alloc.memalloc(ba.alloc.userData, size))
	if bucket == nil {
		return false
	}
	bucket.next = nil

	bucket.next = ba.buckets
	ba.buckets = bucket

	freelist := ba.freelist
	head := uintptr(bucket) + size_of(Bucket)
	it := head + uintptr(ba.itemSize * ba.bucketSize)
	for {
		it -= uintptr(ba.itemSize)
		(^rawptr)(it)^ = freelist
		freelist = rawptr(it)
		if it == head { break }
	}
	ba.freelist = rawptr(it)
	return true
}

@(private)
nextFreeItem :: proc(ba: ^BucketAlloc) -> rawptr {
	return (^rawptr)(ba.freelist)^
}

createBucketAlloc :: proc(alloc: ^Alloc, name: cstring, itemSize: u32, bucketSize: u32) -> ^BucketAlloc {
	ba := (^BucketAlloc)(alloc.memalloc(alloc.userData, size_of(BucketAlloc)))
	if ba == nil { return nil }

	ba.alloc      = alloc
	ba.name       = name
	ba.itemSize   = max(itemSize, u32(size_of(rawptr)))
	ba.bucketSize = bucketSize
	ba.freelist   = nil
	ba.buckets    = nil

	if !createBucket(ba) {
		alloc.memfree(alloc.userData, ba)
		return nil
	}
	return ba
}

bucketAlloc :: proc(ba: ^BucketAlloc) -> rawptr {
	if ba.freelist == nil || nextFreeItem(ba) == nil {
		if !createBucket(ba) {
			return nil
		}
	}
	it := ba.freelist
	ba.freelist = nextFreeItem(ba)
	return it
}

bucketFree :: proc(ba: ^BucketAlloc, ptr: rawptr) {
	(^rawptr)(ptr)^ = ba.freelist
	ba.freelist = ptr
}

deleteBucketAlloc :: proc(ba: ^BucketAlloc) {
	alloc  := ba.alloc
	bucket := ba.buckets
	for bucket != nil {
		next := bucket.next
		alloc.memfree(alloc.userData, bucket)
		bucket = next
	}
	ba.freelist = nil
	ba.buckets  = nil
	alloc.memfree(alloc.userData, ba)
}

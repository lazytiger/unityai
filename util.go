package unityai

import (
	"unsafe"
)

func Assert(v bool) {
	if !v {
		panic("Assert failed")
	}
}

var navMeshTileSize = unsafe.Sizeof(NavMeshTile{})

func GetTileIndex(begin, end *NavMeshTile) int32 {
	a := uintptr(unsafe.Pointer(begin))
	b := uintptr(unsafe.Pointer(end))
	return int32((b - a) / navMeshTileSize)
}

type TileFreeList struct {
	m_Data     []NavMeshTile
	m_NextFree uint32
}

const (
	kNullLinkId uint32 = 0xffffffff
)

func (this *TileFreeList) Init() {
	this.m_NextFree = kNullLinkId
}

func (this *TileFreeList) Alloc() uint32 {
	if this.m_NextFree == kNullLinkId {
		s := 2 * this.Capacity()
		if s == 0 {
			s = 4
		}
		this.Grow(s)
	}
	id := this.m_NextFree
	this.m_NextFree = this.m_Data[id].next
	this.m_Data[id].next = 0
	return id
}

func (this *TileFreeList) Grow(s uint32) {
	capacity := this.Capacity()
	if s < capacity || this.m_NextFree != kNullLinkId {
		return
	}

	for i := uint32(len(this.m_Data)); i < s; i++ {
		this.m_Data = append(this.m_Data, *NewNavMeshTile())
		this.m_Data[i].next = i + 1
	}
	this.m_Data[s-1].next = kNullLinkId
	this.m_NextFree = capacity
}

func (this *TileFreeList) Release(id uint32) {
	this.m_Data[id].next = this.m_NextFree
	this.m_NextFree = id
}

func (this *TileFreeList) Capacity() uint32 {
	return uint32(len(this.m_Data))
}

func (this *TileFreeList) Get(i uint32) *NavMeshTile {
	return &this.m_Data[i]
}

type LinkFreeList struct {
	m_Data     []NavMeshLink
	m_NextFree uint32
}

func (this *LinkFreeList) Alloc() uint32 {
	if this.m_NextFree == kNullLinkId {
		s := 2 * this.Capacity()
		if s == 0 {
			s = 4
		}
		this.Grow(s)
	}
	id := this.m_NextFree
	this.m_NextFree = this.m_Data[id].next
	this.m_Data[id].next = 0
	return id
}

func (this *LinkFreeList) Release(id uint32) {
	this.m_Data[id].next = this.m_NextFree
	this.m_NextFree = id
}

func (this *LinkFreeList) Capacity() uint32 {
	return uint32(len(this.m_Data))
}

func (this *LinkFreeList) Get(i uint32) *NavMeshLink {
	return &this.m_Data[i]
}

func (this *LinkFreeList) Init() {
	this.m_NextFree = kNullLinkId
}

func (this *LinkFreeList) Grow(s uint32) {
	capacity := this.Capacity()
	if s < capacity || this.m_NextFree != kNullLinkId {
		return
	}

	for i := uint32(len(this.m_Data)); i < s; i++ {
		this.m_Data = append(this.m_Data, NavMeshLink{})
		this.m_Data[i].next = i + 1
	}
	this.m_Data[s-1].next = kNullLinkId
	this.m_NextFree = capacity
}

type OffMeshFreeList struct {
	m_Data     []OffMeshConnection
	m_NextFree uint32
}

func (this *OffMeshFreeList) Alloc() uint32 {
	if this.m_NextFree == kNullLinkId {
		s := 2 * this.Capacity()
		if s == 0 {
			s = 4
		}
		this.Grow(s)
	}
	id := this.m_NextFree
	this.m_NextFree = this.m_Data[id].next
	this.m_Data[id].next = 0
	return id
}

func (this *OffMeshFreeList) Release(id uint32) {
	this.m_Data[id].next = this.m_NextFree
	this.m_NextFree = id
}

func (this *OffMeshFreeList) Capacity() uint32 {
	return uint32(len(this.m_Data))
}

func (this *OffMeshFreeList) Get(i uint32) *OffMeshConnection {
	return &this.m_Data[i]
}

func (this *OffMeshFreeList) Init() {
	this.m_NextFree = kNullLinkId
}

func (this *OffMeshFreeList) Grow(s uint32) {
	capacity := this.Capacity()
	if s < capacity || this.m_NextFree != kNullLinkId {
		return
	}

	for i := uint32(len(this.m_Data)); i < s; i++ {
		this.m_Data = append(this.m_Data, *NewOffMeshConnection())
		this.m_Data[i].next = i + 1
	}
	this.m_Data[s-1].next = kNullLinkId
	this.m_NextFree = capacity
}

func NextIndex(index, modulus int32) int32 {
	Assert(index < modulus)
	next := index + 1
	if next == modulus {
		return 0
	} else {
		return next
	}
}

func PrevIndex(index, modulus int32) int32 {
	if index == 0 {
		return modulus - 1
	} else {
		return index - 1
	}
}

func TriArea2D(a, b, c Vector3f) float32 {
	abx := b.x - a.x
	abz := b.z - a.z
	acx := c.x - a.x
	acz := c.z - a.z
	return acx*abz - abx*acz
}

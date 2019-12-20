package unityai

import (
	"unsafe"
)

type NavMeshNodeFlags uint8

const (
	kNew    NavMeshNodeFlags = 0x00
	kOpen   NavMeshNodeFlags = 0x01
	kClosed NavMeshNodeFlags = 0x02
)

type NavMeshNode struct {
	pos   Vector3f         // Position of the node.
	cost  float32          // Cost from previous node to current node.
	total float32          // Cost up to the node.
	pidx  uint32           // Index to parent node.
	flags NavMeshNodeFlags // NavMeshNode flags new/open/closed.
	id    NavMeshPolyRef   // Polygon ref the node corresponds to.
}

type NavMeshNodeIndex uint16

const (
	kNavMeshNodeNullIndex NavMeshNodeIndex = 0xffff
)

type NavMeshNodePool struct {
	m_MaxNavMeshNodes  int32
	m_HashSize         int32
	m_NavMeshNodeCount int32
	m_NavMeshNodes     []NavMeshNode
	m_First            []NavMeshNodeIndex
	m_Next             []NavMeshNodeIndex
}

var navMeshNodeSize = unsafe.Sizeof(NavMeshNode{})

func GetNavMeshNodeIndex(start, end *NavMeshNode) int32 {
	nstart := uintptr(unsafe.Pointer(start))
	nend := uintptr(unsafe.Pointer(end))
	return int32((nend - nstart) / navMeshNodeSize)
}

func (this *NavMeshNodePool) GetNodeIdx(node *NavMeshNode) uint32 {
	if node == nil {
		return 0
	}
	idx := GetNavMeshNodeIndex(&this.m_NavMeshNodes[0], node) + 1
	Assert(idx <= this.m_MaxNavMeshNodes)
	return uint32(idx)
}

func (this *NavMeshNodePool) GetNodeAtIdx(idx uint32) *NavMeshNode {
	Assert(idx <= uint32(this.m_MaxNavMeshNodes))
	if idx == 0 {
		return nil
	}
	return &this.m_NavMeshNodes[idx-1]
}

func (this *NavMeshNodePool) GetHashSize() int32 {
	return this.m_HashSize
}

func (this *NavMeshNodePool) GetFirst(bucket int32) NavMeshNodeIndex {
	return this.m_First[bucket]
}

func (this *NavMeshNodePool) GetNext(i int32) NavMeshNodeIndex {
	return this.m_Next[i]
}

type NavMeshNodeQueue struct {
	m_Heap []*NavMeshNode
	m_Size int32
}

func (this *NavMeshNodeQueue) empty() bool {
	return this.m_Size == 0
}

func (this *NavMeshNodeQueue) Clear() {
	this.m_Size = 0
}

func (this *NavMeshNodeQueue) Pop() *NavMeshNode {
	result := this.m_Heap[0]
	this.m_Size--
	this.TrickleDown(0, this.m_Heap[this.m_Size])
	return result
}

func (this *NavMeshNodeQueue) Push(node *NavMeshNode) {
	this.m_Size++
	this.BubbleUp(this.m_Size-1, node)
}

func (this *NavMeshNodeQueue) Modify(node *NavMeshNode) {
	for i := int32(0); i < this.m_Size; i++ {
		if this.m_Heap[i] == node {
			this.BubbleUp(i, node)
			return
		}
	}
}

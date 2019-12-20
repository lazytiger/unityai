package unityai

func HashRef(a NavMeshPolyRef) uint32 {
	// Thomas Wang's 64bit to 32bit.
	a = (^a) + (a << 18) // key = (key << 18) - key - 1;
	a = a ^ (a >> 31)
	a = a * 21 // key = (key + (key << 2)) + (key << 4);
	a = a ^ (a >> 11)
	a = a + (a << 6)
	a = a ^ (a >> 22)
	return uint32(a)
}

//////////////////////////////////////////////////////////////////////////////////////////
func NewNavMeshNodePool(maxNavMeshNodes, hashSize int32) *NavMeshNodePool {
	Assert(hashSize != 0)
	Assert(IsPowerOfTwo(hashSize))
	pool := new(NavMeshNodePool)
	pool.m_MaxNavMeshNodes = maxNavMeshNodes
	pool.m_HashSize = hashSize
	pool.m_NavMeshNodeCount = 0
	pool.m_NavMeshNodes = make([]NavMeshNode, maxNavMeshNodes, maxNavMeshNodes)
	pool.m_First = make([]NavMeshNodeIndex, hashSize, hashSize)
	for i := range pool.m_First {
		pool.m_First[i] = kNavMeshNodeNullIndex
	}
	pool.m_Next = make([]NavMeshNodeIndex, maxNavMeshNodes, maxNavMeshNodes)
	for i := range pool.m_Next {
		pool.m_Next[i] = kNavMeshNodeNullIndex
	}
	return pool
}

func (this *NavMeshNodePool) Clear() {
	for i := range this.m_First {
		this.m_First[i] = kNavMeshNodeNullIndex
	}
	this.m_NavMeshNodeCount = 0
}

func (this *NavMeshNodePool) FindNavMeshNode(id NavMeshPolyRef) *NavMeshNode {
	bucket := HashRef(id) & uint32(this.m_HashSize-1)
	i := this.m_First[bucket]
	for i != kNavMeshNodeNullIndex {
		if this.m_NavMeshNodes[i].id == id {

			return &this.m_NavMeshNodes[i]
		}
		i = this.m_Next[i]
	}
	return nil
}

func (this *NavMeshNodePool) GetNode(id NavMeshPolyRef) *NavMeshNode {
	bucket := HashRef(id) & uint32(this.m_HashSize-1)
	i := this.m_First[bucket]
	var node *NavMeshNode
	for i != kNavMeshNodeNullIndex {
		if this.m_NavMeshNodes[i].id == id {
			return &this.m_NavMeshNodes[i]
		}
		i = this.m_Next[i]
	}

	if this.m_NavMeshNodeCount >= this.m_MaxNavMeshNodes {
		return nil
	}

	i = NavMeshNodeIndex(this.m_NavMeshNodeCount)
	this.m_NavMeshNodeCount++

	// Init node
	node = &this.m_NavMeshNodes[i]
	node.pidx = 0
	node.cost = 0
	node.total = 0
	node.id = id
	node.flags = kNew

	this.m_Next[i] = this.m_First[bucket]
	this.m_First[bucket] = i
	return node
}

//////////////////////////////////////////////////////////////////////////////////////////
func NewNavMeshNodeQueue(n int32) *NavMeshNodeQueue {
	queue := new(NavMeshNodeQueue)
	queue.m_Heap = make([]*NavMeshNode, n+1, n+1)
	return queue
}

func (this *NavMeshNodeQueue) BubbleUp(i int32, node *NavMeshNode) {
	parent := (i - 1) / 2
	// note: (index > 0) means there is a parent
	for (i > 0) && (this.m_Heap[parent].total > node.total) {
		this.m_Heap[i] = this.m_Heap[parent]
		i = parent
		parent = (i - 1) / 2
	}
	this.m_Heap[i] = node
}

func (this *NavMeshNodeQueue) TrickleDown(i int32, node *NavMeshNode) {
	child := (i * 2) + 1
	for child < this.m_Size {
		if ((child + 1) < this.m_Size) && (this.m_Heap[child].total > this.m_Heap[child+1].total) {
			child++
		}
		this.m_Heap[i] = this.m_Heap[child]
		i = child
		child = (i * 2) + 1
	}
	this.BubbleUp(i, node)
}

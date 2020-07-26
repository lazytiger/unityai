package unityai

func hash(x, y, z int, BucketCount int) int {
	h1 := 0x8da6b343 // Large multiplicative constants;
	h2 := 0xd8163841 // here arbitrarily chosen primes
	h3 := 0xcb1ab31f
	n := h1*x + h2*y + h3*z
	return n & (BucketCount - 1)
}
func NewVertexWelder(BucketCount int, verts *[]Vector3f, weldThr float32) *VertexWelder {
	welder := &VertexWelder{
		m_weldThr:   weldThr,
		m_verts:     verts,
		m_next:      nil,
		m_first:     make([]int, BucketCount),
		BucketCount: BucketCount,
	}

	Assert(weldThr > 0.0)
	for i := 0; i < BucketCount; i++ {
		welder.m_first[i] = -1
	}
	return welder
}

type VertexWelder struct {
	m_weldThr   float32
	m_verts     *[]Vector3f
	m_next      []int
	m_first     []int
	BucketCount int
}

func (this *VertexWelder) GetCellSize() float32 {
	return this.m_weldThr * 10.0
}

func (this *VertexWelder) SetVertexArray(verts *[]Vector3f) {
	Assert(len(*verts) == 0)
	this.m_verts = verts
}

func (this *VertexWelder) Reset() {
	for i := 0; i < this.BucketCount; i++ {
		this.m_first[i] = -1
	}
	this.m_next = this.m_next[:0]
}

func (this *VertexWelder) Push(pt Vector3f) int {
	cellSize := this.GetCellSize()
	// Add vertex
	x := FloorfToInt(pt.x / cellSize)
	y := FloorfToInt(pt.y / cellSize)
	z := FloorfToInt(pt.z / cellSize)
	h := hash(int(x), int(y), int(z), this.BucketCount)
	*this.m_verts = append(*this.m_verts, pt)
	this.m_next = append(this.m_next, -1)
	// The external array must be kept in sync with m_next.
	// Only this method should add stuff to verts when using the welder.
	Assert(len(*this.m_verts) == len(this.m_next))
	idx := len(*this.m_verts) - 1
	this.m_next[idx] = this.m_first[h]
	this.m_first[h] = idx
	return idx
}

func (this *VertexWelder) AddUnique(pt Vector3f) int {
	verts := *this.m_verts

	// Try to find vertex
	weldThr := this.m_weldThr
	cellSize := this.GetCellSize()
	minx := FloorfToInt((pt.x - weldThr) / cellSize)
	maxx := FloorfToInt((pt.x + weldThr) / cellSize)
	miny := FloorfToInt((pt.y - weldThr) / cellSize)
	maxy := FloorfToInt((pt.y + weldThr) / cellSize)
	minz := FloorfToInt((pt.z - weldThr) / cellSize)
	maxz := FloorfToInt((pt.z + weldThr) / cellSize)
	bestIndex := -1
	bestDistSq := weldThr * weldThr
	for z := minz; z <= maxz; z++ {
		for y := miny; y <= maxy; y++ {
			for x := minx; x <= maxx; x++ {
				h := hash(int(x), int(y), int(z), this.BucketCount)
				for i := this.m_first[h]; i != -1; i = this.m_next[i] {
					distSq := SqrMagnitude(verts[i].Sub(pt))
					if distSq < bestDistSq {
						bestDistSq = distSq
						bestIndex = i
					}
				}
			}
		}
	}

	if bestIndex != -1 {

		return bestIndex
	}

	return this.Push(pt)
}

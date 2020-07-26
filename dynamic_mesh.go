package unityai

const kNumVerts = 6

type PolyStatus uint8

const (
	kOriginalPolygon  PolyStatus = iota
	kGeneratedPolygon            = 1
)

type Poly struct {
	m_Neighbours  [kNumVerts]uint16
	m_VertexIDs   [kNumVerts]uint16
	m_VertexCount uint8
	m_Status      PolyStatus
}

type DataType int32

type Hull []Plane
func (this *Hull) emplace_back_uninitialized() *Plane {
	*this = append(*this, Plane{})
	return &(*this)[len(*this)-1]
}

type HullContainer []Hull
type Polygon []Vector3f

func (this *Polygon) resize_uninitialized(size int) {
	if cap(*this) >= size {
		*this = (*this)[:size]
	} else {
		*this = append(*this, make([]Vector3f, size-len(*this))...)
	}
}

func (this *Polygon) emplace_back_uninitialized() *Vector3f {
	*this = append(*this, Vector3f{})
	return &(*this)[len(*this)-1]
}

func (this *Polygon) push_back(vert Vector3f) {
	*this = append(*this, vert)
}

func (this *Polygon) clone() Polygon {
	ret := make([]Vector3f, len(*this))
	copy(ret, *this)
	return ret
}

type PolygonContainer []Polygon

func (this *PolygonContainer) clear() {
	*this = (*this)[:0]
}

func (this *PolygonContainer) push_back(tri Polygon) {
	*this = append(*this, tri)
}

func (this *PolygonContainer) erase(index int) {
	if index == len(*this)-1 {
		*this = (*this)[:len(*this)-1]
	} else if index == 0 {
		*this = (*this)[1:]
	} else {
		tmp := make([]Polygon, len(*this)-1)
		copy(tmp[:index], (*this)[:index])
		copy(tmp[index:], (*this)[index+1:])
		*this = tmp
	}
}

type DetailHull struct {
	hull     Hull
	polysIds []int // Polygon ids to carve
}

type DetailHullContainer []DetailHull

func NewDetailHull() *DetailHull {
	return &DetailHull{}
}

type Edge struct {
	v1, v2, p1, p2, c1, c2 uint16
}

type EdgeList []Edge

func (this *EdgeList) resize_uninitialized(size int) {
	if cap(*this) > size {
		*this = (*this)[:size]
	} else {
		*this = append(*this, make([]Edge, size-len(*this))...)
	}
}

type PolyList []Poly

func (l *PolyList) resize_uninitialized(size int) {
	if cap(*l) >= size {
		*l = (*l)[:size]
	} else {
		*l = append(*l, make([]Poly, size-len(*l))...)
	}
}

func (this *PolyList) erase(index int) {
	if index == len(*this)-1 {
		*this = (*this)[:len(*this)-1]
	} else if index == 0 {
		*this = (*this)[1:]
	} else {
		tmp := make([]Poly, len(*this)-1)
		copy(tmp[:index], (*this)[:index])
		copy(tmp[index:], (*this)[index+1:])
		*this = tmp
	}
}

type DynamicMesh struct {
	m_Polygons    PolyList
	m_Vertices    []Vector3f
	m_Data        []DataType
	m_Welder      *VertexWelder //BucketCount=2048
	m_QuantFactor float32
}

func (this *DynamicMesh) PolyCount() int {
	return len(this.m_Polygons)
}

func (this *DynamicMesh) VertCount() int {
	return len(this.m_Vertices)
}

func (this *DynamicMesh) GetVertex(i int) Vector3f {
	return this.m_Vertices[i]
}

func (this *DynamicMesh) SetVertex(i int, v Vector3f) {
	this.m_Vertices[i] = v
}

func (this *DynamicMesh) GetPoly(i int) *Poly {
	return &this.m_Polygons[i]
}

func (this *DynamicMesh) GetData(i int) *DataType {
	return &this.m_Data[i]
}

func NewDynamicMesh(quantFactor float32) *DynamicMesh {
	mesh := &DynamicMesh{
		m_QuantFactor: quantFactor,
		m_Welder:      NewVertexWelder(2048, nil, quantFactor),
	}
	mesh.m_Welder.SetVertexArray(&mesh.m_Vertices)
	return mesh
}

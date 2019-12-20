package unityai

const (
	kMaxAreas int32 = 32
)

type QueryFilter struct {
	m_AreaCost     [kMaxAreas]float32 // Array storing cost per area type, used by default implementation.
	m_IncludeFlags uint32             // Include poly flags, used by default implementation.
	m_TypeID       int32              // Which type of navmesh is considered
}

func NewQueryFilter() *QueryFilter {
	var filter QueryFilter
	filter.m_IncludeFlags = 0xffffffff
	filter.m_TypeID = -1
	for i := range filter.m_AreaCost {
		filter.m_AreaCost[i] = 1.0
	}
	return &filter
}

func (this *QueryFilter) PassFilter(flags uint32) bool {
	return (flags & this.m_IncludeFlags) != 0
}

func (this *QueryFilter) Set(typeID int32, includeFlags uint32, costs []float32) {
	this.m_TypeID = typeID
	this.m_IncludeFlags = includeFlags
	if costs != nil {
		copy(this.m_AreaCost[:], costs)
	}
}

// Getters and setters for the default implementation data.
func (this *QueryFilter) GetAreaCost(i int32) float32 {
	Assert(i < kMaxAreas)
	return this.m_AreaCost[i]
}

func (this *QueryFilter) SetAreaCost(i int32, cost float32) {
	this.m_AreaCost[i] = cost
}

func (this *QueryFilter) SetAreaCosts(areaCosts []float32) {
	copy(this.m_AreaCost[:], areaCosts)
}

func (this *QueryFilter) GetIncludeFlags() uint32 {
	return this.m_IncludeFlags
}

func (this *QueryFilter) SetIncludeFlags(flags uint32) {
	this.m_IncludeFlags = flags
}

func (this *QueryFilter) SetTypeID(typeID int32) {
	this.m_TypeID = typeID
}

func (this *QueryFilter) GetTypeID() int32 {
	return this.m_TypeID
}

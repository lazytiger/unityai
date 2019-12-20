package unityai

type NavMeshPath struct {
	m_timeStamp      uint32
	m_status         NavMeshPathStatus
	m_polygons       []NavMeshPolyRef
	m_sourcePosition Vector3f
	m_targetPosition Vector3f
	m_Size           int32
}

type NavMeshPathStatus int32

const (
	kPathComplete NavMeshPathStatus = 0
	kPathPartial  NavMeshPathStatus = 1
	kPathInvalid  NavMeshPathStatus = 2
)

func (this *NavMeshPath) GetSourcePosition() Vector3f {
	return this.m_sourcePosition
}

func (this *NavMeshPath) SetSourcePosition(sourcePosition Vector3f) {
	this.m_sourcePosition = sourcePosition
}

func (this *NavMeshPath) GetTargetPosition() Vector3f {
	return this.m_targetPosition
}

func (this *NavMeshPath) SetTargetPosition(targetPosition Vector3f) {
	this.m_targetPosition = targetPosition
}

func (this *NavMeshPath) GetPolygonCount() int32 {
	return this.m_Size
}

func (this *NavMeshPath) GetPolygonCapacity() int32 {
	return int32(len(this.m_polygons))
}

func (this *NavMeshPath) SetPolygonCount(polygonCount int32) {
	if this.GetPolygonCapacity() < polygonCount {
		this.ReservePolygons(polygonCount)
	}
	this.m_Size = polygonCount
}

func (this *NavMeshPath) GetPolygonPath() []NavMeshPolyRef {
	return this.m_polygons
}

func (this *NavMeshPath) GetPolygonData() []NavMeshPolyRef {
	return this.m_polygons[:this.m_Size]
}

func (this *NavMeshPath) GetStatus() NavMeshPathStatus {
	return this.m_status
}

func (this *NavMeshPath) IsComplete() bool {
	return this.m_status == kPathComplete
}

func (this *NavMeshPath) IsPartial() bool {
	return this.m_status == kPathPartial
}

func (this *NavMeshPath) IsInvalid() bool {
	return this.m_status == kPathInvalid
}

func (this *NavMeshPath) SetStatus(status NavMeshPathStatus) {
	this.m_status = status
}

func (this *NavMeshPath) SetTimeStamp(timeStamp uint32) {
	this.m_timeStamp = timeStamp
}

func NewNavMeshPath() *NavMeshPath {
	var path NavMeshPath
	path.m_status = kPathInvalid
	return &path
}

func (this *NavMeshPath) ReservePolygons(size int32) {
	kSizeInc := int32(32)
	capNum := ((size + kSizeInc - 1) / kSizeInc) * kSizeInc
	if cap(this.m_polygons) < int(capNum) {
		d := make([]NavMeshPolyRef, capNum, capNum)
		copy(d, this.m_polygons)
		this.m_polygons = d
	}
}

package unityai

type NavMeshObstacleShape int32

const (
	kObstacleShapeCapsule NavMeshObstacleShape = iota
	kObstacleShapeBox
)

type NavMeshCarveShape struct {
	shape   NavMeshObstacleShape // NavMeshObstacleShape
	center  Vector3f
	extents Vector3f
	xAxis   Vector3f
	yAxis   Vector3f
	zAxis   Vector3f
	bounds  MinMaxAABB
}

func (this *NavMeshCarveShape) GetBounds() MinMaxAABB {
	return this.bounds
}

type CarveData struct {
	m_SurfaceID int32
	m_TileIndex int32
	m_Position  Vector3f
	m_Rotation  Quaternionf
	m_Shapes    []NavMeshCarveShape
}

type Shapes []NavMeshCarveShape

func (this Shapes) Len() int {
	return len(this)
}

func (this Shapes) Less(i, j int) bool {
	lhs := this[i]
	rhs := this[j]
	if lhs.center.x < rhs.center.x {
		return true
	}
	if rhs.center.x < lhs.center.x {
		return false
	}

	// rhs.x == lhs.x
	if lhs.center.z < rhs.center.z {
		return true
	}
	if rhs.center.z < lhs.center.z {
		return false
	}

	// rhs.x == lhs.x && rhs.z == lhs.z
	if lhs.center.y < rhs.center.y {
		return true
	}
	if rhs.center.y < lhs.center.y {
		return false
	}

	// all same - favour the biggest
	return SqrMagnitude(lhs.extents) > SqrMagnitude(rhs.extents)
}

func (this Shapes) Swap(i, j int) {
	this[i], this[j] = this[j], this[i]
}

func NewCarveData(surfaceID int32, tileIndex int32) *CarveData {
	return &CarveData{
		m_SurfaceID: surfaceID,
		m_TileIndex: tileIndex,
	}
}

func (this *CarveData) AddShape(shape NavMeshCarveShape) {
	this.m_Shapes = append(this.m_Shapes, shape)
}

func (this *CarveData) Empty() bool {
	return len(this.m_Shapes) == 0
}

type NavMeshObstacle struct {
	m_VersionStamp int32
	m_Shape        NavMeshObstacleShape
	m_Extents      Vector3f
	m_Scale        Vector3f
	m_Center       Vector3f
	m_Position     Vector3f
	m_Rotation     Quaternionf
}

func NewNavMeshObstacle(shape NavMeshObstacleShape, position, scale Vector3f, rotation Quaternionf) *NavMeshObstacle {
	obs := &NavMeshObstacle{
		m_VersionStamp: 0,
		m_Shape:        shape,
		m_Extents:      NewVector3f(0.5, 0.5, 0.5),
		m_Center:       NewVector3f(0, 0, 0),
		m_Scale:        scale,
		m_Rotation:     rotation,
		m_Position:     position,
	}
	return obs
}

func (this *NavMeshObstacle) SetSize(size Vector3f) {
	this.SetExtents(size.Mulf(0.5))
}

func (this *NavMeshObstacle) SetCenter(center Vector3f) {
	this.m_Center = center
}

func (this *NavMeshObstacle) GetVersionStamp() int32 {
	return this.m_VersionStamp
}

func (this *NavMeshObstacle) GetCarveShape(shape *NavMeshCarveShape) {
	shape.shape = this.m_Shape
	shape.extents = this.GetWorldExtents()
	this.GetWorldCenterAndAxes(&shape.center, &shape.xAxis, &shape.yAxis, &shape.zAxis)
	var worldExtents Vector3f
	if this.m_Shape == kObstacleShapeCapsule {
		CalcCapsuleWorldExtents(&worldExtents, shape.extents, shape.xAxis, shape.yAxis, shape.zAxis)
	} else {
		CalcBoxWorldExtents(&worldExtents, shape.extents, shape.xAxis, shape.yAxis, shape.zAxis)
	}
	shape.bounds = NewMinMaxAABB(shape.center.Sub(worldExtents), shape.center.Add(worldExtents))
}

func (this *NavMeshObstacle) GetWorldExtents() Vector3f {
	var matrix Matrix4x4f
	matrix.SetTRS(this.m_Position, this.m_Rotation, this.m_Scale)
	absScale := matrix.GetLossyScale()
	if this.m_Shape == kObstacleShapeCapsule {
		scaledRadius := this.m_Extents.x * FloatMax(absScale.x, absScale.z)
		scaledHeight := this.m_Extents.y * absScale.y
		return NewVector3f(scaledRadius, scaledHeight, scaledRadius)
	} else {
		return ScaleVector3f(this.m_Extents, absScale)
	}
}

func (this *NavMeshObstacle) GetWorldCenterAndAxes(center, xAxis, yAxis, zAxis *Vector3f) {
	//const Transform& transform = GetComponent<Transform>();

	//center = transform.TransformPoint(m_Center);
	//Quaternionf rotation = transform.GetRotation();
	var matrix Matrix4x4f
	matrix.SetTRS(this.m_Position, this.m_Rotation, this.m_Scale)
	*center = matrix.MultiplyPoint3(this.m_Center)

	var rotationMatrix Matrix3x3f
	QuaternionToMatrix3(this.m_Rotation, &rotationMatrix)
	*xAxis = rotationMatrix.GetAxisX()
	*yAxis = rotationMatrix.GetAxisY()
	*zAxis = rotationMatrix.GetAxisZ()
}

func (this *NavMeshObstacle) SetExtents(mulf Vector3f) {
	this.m_Extents = mulf
}

func EnsurePositive(value float32) float32 {
	return FloatMax(0.00001, value)
}

func (this *NavMeshObstacle) SetHeight(value float32) {
	positiveHeight := EnsurePositive(value)
	newExtents := NewVector3f(this.m_Extents.x, positiveHeight*0.5, this.m_Extents.z)
	this.SetExtents(newExtents)
}

func (this *NavMeshObstacle) SetRadius(value float32) {
	positiveRadius := EnsurePositive(value)
	newExtents := NewVector3f(positiveRadius, this.m_Extents.y, positiveRadius)
	this.SetExtents(newExtents)
}

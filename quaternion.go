package unityai

import "math"

type Quaternionf struct {
	x, y, z, w float32
}

func NewQuaternionf(x, y, z, w float32) Quaternionf {
	return Quaternionf{x, y, z, w}
}

func (this *Quaternionf) X() float32 {
	return this.x
}
func (this *Quaternionf) Y() float32 {
	return this.y
}
func (this *Quaternionf) Z() float32 {
	return this.z
}
func (this *Quaternionf) W() float32 {
	return this.w
}
func (this *Quaternionf) Add(that Quaternionf) Quaternionf {
	return Quaternionf{
		this.x + that.x,
		this.y + that.y,
		this.z + that.z,
		this.w + that.w,
	}
}

func (this *Quaternionf) Sub(that Quaternionf) Quaternionf {
	return Quaternionf{
		this.x - that.x,
		this.y - that.y,
		this.z - that.z,
		this.w - that.w,
	}
}

func (lhs Quaternionf) Mul(rhs Quaternionf) Quaternionf {
	return NewQuaternionf(
		lhs.w*rhs.x+lhs.x*rhs.w+lhs.y*rhs.z-lhs.z*rhs.y,
		lhs.w*rhs.y+lhs.y*rhs.w+lhs.z*rhs.x-lhs.x*rhs.z,
		lhs.w*rhs.z+lhs.z*rhs.w+lhs.x*rhs.y-lhs.y*rhs.x,
		lhs.w*rhs.w-lhs.x*rhs.x-lhs.y*rhs.y-lhs.z*rhs.z)
}

func QuaternionToMatrix3(q Quaternionf, m *Matrix3x3f) {
	// Precalculate coordinate products
	x := q.x * 2.0
	y := q.y * 2.0
	z := q.z * 2.0
	xx := q.x * x
	yy := q.y * y
	zz := q.z * z
	xy := q.x * y
	xz := q.x * z
	yz := q.y * z
	wx := q.w * x
	wy := q.w * y
	wz := q.w * z

	// Calculate 3x3 matrix from orthonormal basis
	m.m_Data[0] = 1.0 - (yy + zz)
	m.m_Data[1] = xy + wz
	m.m_Data[2] = xz - wy
	m.m_Data[3] = xy - wz
	m.m_Data[4] = 1.0 - (xx + zz)
	m.m_Data[5] = yz + wx
	m.m_Data[6] = xz + wy
	m.m_Data[7] = yz - wx
	m.m_Data[8] = 1.0 - (xx + yy)
}

func QuaternionToMatrix4(q Quaternionf, m *Matrix4x4f) {
	// Precalculate coordinate products
	x := q.x * 2.0
	y := q.y * 2.0
	z := q.z * 2.0
	xx := q.x * x
	yy := q.y * y
	zz := q.z * z
	xy := q.x * y
	xz := q.x * z
	yz := q.y * z
	wx := q.w * x
	wy := q.w * y
	wz := q.w * z

	// Calculate 3x3 matrix from orthonormal basis
	m.m_Data[0] = 1.0 - (yy + zz)
	m.m_Data[1] = xy + wz
	m.m_Data[2] = xz - wy
	m.m_Data[3] = 0.0
	m.m_Data[4] = xy - wz
	m.m_Data[5] = 1.0 - (xx + zz)
	m.m_Data[6] = yz + wx
	m.m_Data[7] = 0.0
	m.m_Data[8] = xz + wy
	m.m_Data[9] = yz - wx
	m.m_Data[10] = 1.0 - (xx + yy)
	m.m_Data[11] = 0.0
	m.m_Data[12] = 0.0
	m.m_Data[13] = 0.0
	m.m_Data[14] = 0.0
	m.m_Data[15] = 1.0
}

func InverseQuaternion(q Quaternionf) Quaternionf {
	var ret Quaternionf
	ret.x = -q.x
	ret.y = -q.y
	ret.z = -q.z
	ret.w = q.w
	return ret
}

type RotationOrder int

const (
	kOrderXYZ RotationOrder = iota
	kOrderXZY
	kOrderYZX
	kOrderYXZ
	kOrderZXY
	kOrderZYX
	OrderUnity RotationOrder = kOrderZXY
)

func EulerToQuaternionUnity(eulerAngle Vector3f) Quaternionf {
	return EulerToQuaternion(eulerAngle.Mulf(math.Pi/180.0), kOrderZXY)
}

func EulerToQuaternion(someEulerAngles Vector3f, order RotationOrder) Quaternionf {
	cX := math.Cos(float64(someEulerAngles.x / 2.0))
	sX := math.Sin(float64(someEulerAngles.x / 2.0))
	cY := math.Cos(float64(someEulerAngles.y / 2.0))
	sY := math.Sin(float64(someEulerAngles.y / 2.0))
	cZ := math.Cos(float64(someEulerAngles.z / 2.0))
	sZ := math.Sin(float64(someEulerAngles.z / 2.0))
	qX := NewQuaternionf(float32(sX), 0.0, 0.0, float32(cX))
	qY := NewQuaternionf(0.0, float32(sY), 0.0, float32(cY))
	qZ := NewQuaternionf(0.0, 0.0, float32(sZ), float32(cZ))
	var ret Quaternionf

	switch order {
	case kOrderZYX:
		CreateQuaternionFromAxisQuaternions(qX, qY, qZ, &ret)
	case kOrderYZX:
		CreateQuaternionFromAxisQuaternions(qX, qZ, qY, &ret)
	case kOrderXZY:
		CreateQuaternionFromAxisQuaternions(qY, qZ, qX, &ret)
	case kOrderZXY:
		CreateQuaternionFromAxisQuaternions(qY, qX, qZ, &ret)
	case kOrderYXZ:
		CreateQuaternionFromAxisQuaternions(qZ, qX, qY, &ret)
	case kOrderXYZ:
		CreateQuaternionFromAxisQuaternions(qZ, qY, qX, &ret)
	}

	return ret
}

func CreateQuaternionFromAxisQuaternions(q1 Quaternionf, q2 Quaternionf, q3 Quaternionf, result *Quaternionf) {
	*result = q1.Mul(q2).Mul(q3)
}

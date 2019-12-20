package unityai

type Quaternionf struct {
	x, y, z, w float32
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

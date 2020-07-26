package unityai

type Matrix4x4f struct {
	m_Data [16]float32
}

func (this *Matrix4x4f) SetTR(pos Vector3f, q Quaternionf) {
	QuaternionToMatrix4(q, this)
	this.m_Data[12] = pos.x
	this.m_Data[13] = pos.y
	this.m_Data[14] = pos.z
}

func (this *Matrix4x4f) SetTRS(pos Vector3f, q Quaternionf, s Vector3f) {
	QuaternionToMatrix4(q, this)
	this.m_Data[0] *= s.x
	this.m_Data[1] *= s.x
	this.m_Data[2] *= s.x
	this.m_Data[4] *= s.y
	this.m_Data[5] *= s.y
	this.m_Data[6] *= s.y
	this.m_Data[8] *= s.z
	this.m_Data[9] *= s.z
	this.m_Data[10] *= s.z
	this.m_Data[12] = pos.x
	this.m_Data[13] = pos.y
	this.m_Data[14] = pos.z
}

func (this *Matrix4x4f) GetLossyScale() Vector3f {
	var result Vector3f
	result.x = Magnitude(this.GetAxisX())
	result.y = Magnitude(this.GetAxisY())
	result.z = Magnitude(this.GetAxisZ())
	return result
}

func (this *Matrix4x4f) SetTRInverse(pos Vector3f, q Quaternionf) {
	QuaternionToMatrix4(InverseQuaternion(q), this)
	var v Vector3f
	v.x = -pos.x
	v.y = -pos.y
	v.z = -pos.z
	this.Translate(v)
}

func (this *Matrix4x4f) Get(row, col int) float32 {
	return this.m_Data[row+col*4]
}

func (this *Matrix4x4f) Set(row, col int, data float32) {
	this.m_Data[row+col*4] = data
}

func (this *Matrix4x4f) Translate(inTrans Vector3f) {
	d := this.Get(0, 0)*inTrans.x + this.Get(0, 1)*inTrans.y + this.Get(0, 2)*inTrans.z + this.Get(0, 3)
	this.Set(0, 3, d)
	d = this.Get(1, 0)*inTrans.x + this.Get(1, 1)*inTrans.y + this.Get(1, 2)*inTrans.z + this.Get(1, 3)
	this.Set(1, 3, d)
	d = this.Get(2, 0)*inTrans.x + this.Get(2, 1)*inTrans.y + this.Get(2, 2)*inTrans.z + this.Get(2, 3)
	this.Set(2, 3, d)
	d = this.Get(3, 0)*inTrans.x + this.Get(3, 1)*inTrans.y + this.Get(3, 2)*inTrans.z + this.Get(3, 3)
	this.Set(3, 3, d)
}

func (this *Matrix4x4f) MultiplyPoint3(v Vector3f) Vector3f {
	var res Vector3f
	res.x = this.m_Data[0]*v.x + this.m_Data[4]*v.y + this.m_Data[8]*v.z + this.m_Data[12]
	res.y = this.m_Data[1]*v.x + this.m_Data[5]*v.y + this.m_Data[9]*v.z + this.m_Data[13]
	res.z = this.m_Data[2]*v.x + this.m_Data[6]*v.y + this.m_Data[10]*v.z + this.m_Data[14]
	return res
}

func (this *Matrix4x4f) MultiplyVector3(v Vector3f) Vector3f {
	var res Vector3f
	res.x = this.m_Data[0]*v.x + this.m_Data[4]*v.y + this.m_Data[8]*v.z
	res.y = this.m_Data[1]*v.x + this.m_Data[5]*v.y + this.m_Data[9]*v.z
	res.z = this.m_Data[2]*v.x + this.m_Data[6]*v.y + this.m_Data[10]*v.z
	return res
}

func (this *Matrix4x4f) Scale(scale Vector3f) {
	this.m_Data[0+0*4] *= scale.x
	this.m_Data[1+0*4] *= scale.x
	this.m_Data[2+0*4] *= scale.x
	this.m_Data[3+0*4] *= scale.x
	this.m_Data[0+1*4] *= scale.y
	this.m_Data[1+1*4] *= scale.y
	this.m_Data[2+1*4] *= scale.y
	this.m_Data[3+1*4] *= scale.y
	this.m_Data[0+2*4] *= scale.z
	this.m_Data[1+2*4] *= scale.z
	this.m_Data[2+2*4] *= scale.z
	this.m_Data[3+2*4] *= scale.z
}

func (this *Matrix4x4f) GetAxisX() Vector3f {
	return NewVector3f(this.Get(0, 0), this.Get(1, 0), this.Get(2, 0))
}

func (this *Matrix4x4f) GetAxisY() Vector3f {
	return NewVector3f(this.Get(0, 1), this.Get(1, 1), this.Get(2, 1))
}

func (this *Matrix4x4f) GetAxisZ() Vector3f {
	return NewVector3f(this.Get(0, 2), this.Get(1, 2), this.Get(2, 2))
}

type Matrix3x3f struct {
	m_Data [9]float32
}

func (this *Matrix3x3f) Get(row, column int) float32 {
	return this.m_Data[row+(column*3)]
}
func (this *Matrix3x3f) MultiplyPoint3(v Vector3f) Vector3f {
	return this.MultiplyVector3(v)
}

func (this *Matrix3x3f) MultiplyVector3(v Vector3f) Vector3f {
	var res Vector3f
	res.x = this.m_Data[0]*v.x + this.m_Data[3]*v.y + this.m_Data[6]*v.z
	res.y = this.m_Data[1]*v.x + this.m_Data[4]*v.y + this.m_Data[7]*v.z
	res.z = this.m_Data[2]*v.x + this.m_Data[5]*v.y + this.m_Data[8]*v.z
	return res
}

func (this *Matrix3x3f) GetColumn(i int) Vector3f {
	return NewVector3f(this.Get(0, i), this.Get(1, i), this.Get(2, i))
}

func (this *Matrix3x3f) GetAxisX() Vector3f {
	return NewVector3f(this.Get(0, 0), this.Get(1, 0), this.Get(2, 0))
}

func (this *Matrix3x3f) GetAxisY() Vector3f {
	return NewVector3f(this.Get(0, 1), this.Get(1, 1), this.Get(2, 1))
}

func (this *Matrix3x3f) GetAxisZ() Vector3f {
	return NewVector3f(this.Get(0, 2), this.Get(1, 2), this.Get(2, 2))
}
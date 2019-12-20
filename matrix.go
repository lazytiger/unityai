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

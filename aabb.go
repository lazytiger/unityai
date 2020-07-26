package unityai

import (
	"math"
)

type AABB struct {
	m_Center Vector3f
	m_Extent Vector3f
}

func NewAABBFromMinMax(data MinMaxAABB) AABB {
	return AABB{
		m_Center: data.m_Min.Add(data.m_Max).Mulf(0.5),
		m_Extent: data.m_Max.Sub(data.m_Min).Mulf(0.5),
	}
}

func (this *AABB) SetCenterAndExtent(center, extent Vector3f) {
	this.m_Center = center
	this.m_Extent = extent
}

func (this *AABB) IsInside(inPoint Vector3f) bool {
	if inPoint.x < this.m_Center.x-this.m_Extent.x {
		return false
	}
	if inPoint.x > this.m_Center.x+this.m_Extent.x {
		return false
	}

	if inPoint.y < this.m_Center.y-this.m_Extent.y {
		return false
	}
	if inPoint.y > this.m_Center.y+this.m_Extent.y {
		return false
	}

	if inPoint.z < this.m_Center.z-this.m_Extent.z {
		return false
	}
	if inPoint.z > this.m_Center.z+this.m_Extent.z {
		return false
	}

	return true
}

type MinMaxAABB struct {
	m_Min Vector3f
	m_Max Vector3f
}

func NewMinMaxAABB(min, max Vector3f) MinMaxAABB {
	return MinMaxAABB{m_Max: max, m_Min: min}
}

func NewMinMaxAABBFromAABB(inAABB AABB) MinMaxAABB {
	return MinMaxAABB{
		m_Min: inAABB.m_Center.Sub(inAABB.m_Extent),
		m_Max: inAABB.m_Center.Add(inAABB.m_Extent),
	}
}

func (this *MinMaxAABB) CalculateVertices(outVertices []Vector3f) {
	outVertices[0].Set(this.m_Min.x, this.m_Min.y, this.m_Min.z)
	outVertices[1].Set(this.m_Max.x, this.m_Min.y, this.m_Min.z)
	outVertices[2].Set(this.m_Max.x, this.m_Max.y, this.m_Min.z)
	outVertices[3].Set(this.m_Min.x, this.m_Max.y, this.m_Min.z)
	outVertices[4].Set(this.m_Min.x, this.m_Min.y, this.m_Max.z)
	outVertices[5].Set(this.m_Max.x, this.m_Min.y, this.m_Max.z)
	outVertices[6].Set(this.m_Max.x, this.m_Max.y, this.m_Max.z)
	outVertices[7].Set(this.m_Min.x, this.m_Max.y, this.m_Max.z)
}

func (this *MinMaxAABB) Contains(p Vector3f)bool {
	a := p.Sub(this.m_Min)
	b := this.m_Max.Sub(p)
	return a.x > 0 && a.y >0 && a.z > 0 && b.x > 0 && b.y >0 && b.z > 0
}

func (this *MinMaxAABB) Init() {
	this.m_Min = Vector3f{math.MaxFloat32, math.MaxFloat32, math.MaxFloat32}
	this.m_Max = Vector3f{-math.MaxFloat32, -math.MaxFloat32, -math.MaxFloat32}
}
func (this *MinMaxAABB) EncapsulateV(inPoint Vector3f) {
	this.m_Min = MinVector3f(this.m_Min, inPoint)
	this.m_Max = MaxVector3f(this.m_Max, inPoint)
}

func (this *MinMaxAABB) Encapsulate(other MinMaxAABB) {
	this.m_Min = MinVector3f(this.m_Min, other.m_Min)
	this.m_Max = MaxVector3f(this.m_Max, other.m_Max)
}

func TransformAABBSlow(aabb MinMaxAABB, transform Matrix4x4f, result *MinMaxAABB) {
	var v [8]Vector3f
	aabb.CalculateVertices(v[:])
	result.Init()
	for i := 0; i < 8; i++ {
		point := transform.MultiplyPoint3(v[i])
		result.EncapsulateV(point)
	}
}

func IntersectionAABBAABB(a, b MinMaxAABB, outBoxIntersect *MinMaxAABB) bool {
	if !IntersectAABBAABB(a, b) {
		return false
	}
	outBoxIntersect.m_Min.x = FloatMax(a.m_Min.x, b.m_Min.x)
	outBoxIntersect.m_Max.x = FloatMin(a.m_Max.x, b.m_Max.x)
	outBoxIntersect.m_Min.y = FloatMax(a.m_Min.y, b.m_Min.y)
	outBoxIntersect.m_Max.y = FloatMin(a.m_Max.y, b.m_Max.y)
	outBoxIntersect.m_Min.z = FloatMax(a.m_Min.z, b.m_Min.z)
	outBoxIntersect.m_Max.z = FloatMin(a.m_Max.z, b.m_Max.z)
	return true
}

func IntersectAABBAABB(a, b MinMaxAABB) bool {
	if a.m_Min.x > b.m_Max.x {
		return false
	}
	if a.m_Max.x < b.m_Min.x {
		return false
	}
	if a.m_Min.y > b.m_Max.y {
		return false
	}
	if a.m_Max.y < b.m_Min.y {
		return false
	}
	if a.m_Min.z > b.m_Max.z {
		return false
	}
	if a.m_Max.z < b.m_Min.z {
		return false
	}
	return true
}

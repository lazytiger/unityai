package unityai

type Plane struct {
	normal   Vector3f
	distance float32
}

func (p *Plane) SetNormalAndPosition(normal Vector3f, position Vector3f) {
	p.normal = normal
	p.distance = -DotVector3f(normal, position)
}

func (this *Plane) GetDistanceToPoint(inPt Vector3f) float32 {
	return DotVector3f(this.normal, inPt) + this.distance
}

func NewPlane(normal, position Vector3f) Plane {
	plane := Plane{}
	plane.SetNormalAndPosition(normal, position)
	return plane
}

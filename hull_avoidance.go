package unityai

import "sort"

type Vertex2Array []Vector2f

func (this Vertex2Array) Len() int {
	return len(this)
}

func (this Vertex2Array) Less(i, j int) bool {
	a := this[i]
	b := this[j]
	return a.x < b.x || (a.x == b.x && a.y < b.y)
}

func (this Vertex2Array) Swap(i, j int) {
	this[i], this[j] = this[j], this[i]
}

func (this *Vertex2Array) resize_uninitialized(size int) {
	if cap(*this) >= size {
		*this = (*this)[:size]
	} else {
		*this = append(*this, make([]Vector2f, size-len(*this))...)
	}
}
func (this *Vertex2Array) empty() bool {
	return len(*this) == 0
}

func (this *Vertex2Array) pop_back() {
	*this = (*this)[:this.Len()-1]
}

func (this *Vertex2Array) push_back(point Vector2f) {
	*this = append(*this, point)
}

func (this *Vertex2Array) erase(index int) {
	if index == len(*this)-1 {
		*this = (*this)[:len(*this)-1]
	} else if index == 0 {
		*this = (*this)[1:]
	} else {
		tmp := make([]Vector2f, len(*this)-1)
		copy(tmp[:index], (*this)[:index])
		copy(tmp[index:], (*this)[index+1:])
		*this = tmp
	}
}

func CalculatePointSide(l0, l1, point Vector2f) float32 {
	return (l1.y-l0.y)*(point.x-l0.x) - (l1.x-l0.x)*(point.y-l0.y)
}

func CalculateConvexHull(hull *Vertex2Array, points *Vertex2Array) {
	// TODO : prune (near) duplicate points before calculating hull
	hull.resize_uninitialized(0)
	if points.empty() {
		return
	}

	sort.Sort(points)

	// Andrews monotone chain
	for i := 0; i < points.Len(); i++ {
		for hull.Len() >= 2 && CalculatePointSide((*hull)[hull.Len()-2], (*hull)[hull.Len()-1], (*points)[i]) <= 0 {
			hull.pop_back()
		}
		hull.push_back((*points)[i])
	}

	for i, j := points.Len()-2, hull.Len()+1; i >= 0; i-- {
		for hull.Len() >= j && CalculatePointSide((*hull)[hull.Len()-2], (*hull)[hull.Len()-1], (*points)[i]) <= 0 {
			hull.pop_back()
		}
		hull.push_back((*points)[i])
	}
	hull.pop_back()
}

func FitCapsuleToExtents(radius, height *float32, capsuleExtents Vector3f) {
	r := FloatMax(capsuleExtents.x, capsuleExtents.z)
	*radius = r
	*height = FloatMax(0.0, capsuleExtents.y-r)
}

func CalcCapsuleWorldExtents(worldExtents *Vector3f, localExtents, xAxis, yAxis, zAxis Vector3f) {
	var radius, height float32
	FitCapsuleToExtents(&radius, &height, localExtents)
	*worldExtents = AbsVector3f(yAxis).Mulf(height).Add(NewVector3f(radius, radius, radius))
}

func CalcBoxWorldExtents(worldExtents *Vector3f, localExtents, xAxis, yAxis, zAxis Vector3f) {
	*worldExtents = AbsVector3f(xAxis).Mulf(localExtents.x).Add(AbsVector3f(yAxis).Mulf(localExtents.y)).Add(AbsVector3f(zAxis).Mulf(localExtents.z))
}

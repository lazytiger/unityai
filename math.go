package unityai

import (
	"math"
	"math/rand"
)

func SqrDistancePointSegment(t *float32, pt Vector3f, s1 Vector3f, s2 Vector3f) float32 {
	ds := s2.Sub(s1)
	dp := pt.Sub(s1)
	den := DotVector3f(ds, ds)
	if den == 0 {
		*t = 0
		return DotVector3f(dp, dp)
	}
	tt := DotVector3f(ds, dp) / den
	tt = FloatClamp(tt, 0.0, 1.0)
	v := ds.Mulf(tt).Sub(dp)
	*t = tt
	return DotVector3f(v, v)
}

func SqrDistance(a, b Vector3f) float32 {
	return SqrMagnitudeV(b.Sub(a))
}

func SqrMagnitudeV(inV Vector3f) float32 {
	return DotVector3f(inV, inV)
}

func SqrMagnitudeQ(inV Quaternionf) float32 {
	return DotQuaternionf(inV, inV)
}

func FloatMin(l, r float32) float32 {
	if l > r {
		return r
	} else {
		return l
	}
}

func FloatMax(l, r float32) float32 {
	if l > r {
		return l
	} else {
		return r
	}
}

func FloatAbs(d float32) float32 {
	return float32(math.Abs(float64(d)))
}

func Sqr(x float32) float32 {
	return x * x
}

func FloatClamp(v, min, max float32) float32 {
	if v < min {
		return min
	} else if v > max {
		return max
	} else {
		return v
	}
}

var kBiggestFloatSmallerThanOne float32 = 0.99999994

func FloorfToInt(f float32) int32 {
	if f > 0 {
		return int32(f)
	} else {
		return int32(f - kBiggestFloatSmallerThanOne)
	}
}

func RotateExtents(extents Vector3f, rotation Matrix3x3f) Vector3f {
	var newExtents Vector3f
	for i := 0; i < 3; i++ {
		d := FloatAbs(rotation.Get(i, 0)*extents.x) + FloatAbs(rotation.Get(i, 1)*extents.y) + FloatAbs(rotation.Get(i, 2)*extents.z)
		newExtents.SetData(i, d)
	}
	return newExtents
}

func InverseTransformAABB(aabb AABB, position Vector3f, rotation Quaternionf, result *AABB) {
	var m Matrix3x3f
	QuaternionToMatrix3(InverseQuaternion(rotation), &m)
	extents := RotateExtents(aabb.m_Extent, m)
	center := aabb.m_Center.Sub(position)
	center = m.MultiplyPoint3(center)
	result.SetCenterAndExtent(center, extents)
}

func OverlapBounds(amin, amax, bmin, bmax Vector3f) bool {
	overlap := true
	if amin.x > bmax.x || amax.x < bmin.x {
		overlap = false
	}
	if amin.y > bmax.y || amax.y < bmin.y {
		overlap = false
	}
	if amin.z > bmax.z || amax.z < bmin.z {
		overlap = false
	}
	return overlap
}

func Align4(value uintptr) uint32 {
	alignment := uintptr(4)
	return uint32((value + (alignment - 1)) & ^(alignment - 1))
}

func TransformAABB(aabb AABB, position Vector3f, rotation Quaternionf, result *AABB) {
	var m Matrix3x3f
	QuaternionToMatrix3(rotation, &m)
	extents := RotateExtents(aabb.m_Extent, m)
	center := m.MultiplyPoint3(aabb.m_Center)
	center = center.Add(position)
	result.SetCenterAndExtent(center, extents)
}

func CompareApproximatelyV(inV0, inV1 Vector3f, inMaxDist float32) bool {
	return SqrMagnitudeV(inV1.Sub(inV0)) <= inMaxDist*inMaxDist
}

func CompareApproximatelyQ(q1, q2 Quaternionf, epsilon float32) bool {
	//return SqrMagnitude (q1 - q2) < epsilon * epsilon;
	return (SqrMagnitudeQ(q1.Sub(q2)) <= epsilon*epsilon) || (SqrMagnitudeQ(q1.Add(q2)) <= epsilon*epsilon)
	//return Abs (Dot (q1, q2)) > (1 - epsilon * epsilon);
}

func SqrDistancePointSegment2D(t *float32, pt, s1, s2 Vector3f) float32 {
	dsx := s2.x - s1.x
	dsz := s2.z - s1.z
	dpx := pt.x - s1.x
	dpz := pt.z - s1.z
	den := dsx*dsx + dsz*dsz
	if den == 0 {
		*t = 0
		return dpx*dpx + dpz*dpz
	}
	tt := (dsx*dpx + dsz*dpz) / den
	tt = FloatClamp(tt, 0.0, 1.0)
	x := tt*dsx - dpx
	z := tt*dsz - dpz
	*t = tt
	return x*x + z*z
}

func IntersectSegmentPoly2D(tmin, tmax *float32, segMin, segMax *int32, p0, p1 Vector3f, verts []Vector3f, nverts int32) bool {
	EPS := float32(0.00000001)
	ttmin := float32(0)
	ttmax := float32(1)
	tSegMin := int32(-1)
	tSegMax := int32(-1)
	dir := p1.Sub(p0)

	i := int32(0)
	for j := nverts - 1; i < nverts; j, i = i, i+1 {
		edge := verts[i].Sub(verts[j])
		diff := p0.Sub(verts[j])
		n := Perp2D(edge, diff)
		d := Perp2D(dir, edge)
		if FloatAbs(d) < EPS {
			// S is nearly parallel to this edge
			if n < 0 {
				break
			} else {
				continue
			}
		}
		t := n / d
		if d < 0 {
			// segment S is entering across this edge
			if t > ttmin {
				ttmin = t
				tSegMin = j
				// S enters after leaving polygon
				if ttmin > ttmax {
					break
				}
			}
		} else {
			// segment S is leaving across this edge
			if t < ttmax {
				ttmax = t
				tSegMax = j
				// S leaves before entering polygon
				if ttmax < ttmin {
					break
				}
			}
		}
	}
	*tmin = ttmin
	*tmax = ttmax
	*segMin = tSegMin
	*segMax = tSegMax
	return i == nverts
}

const kEpsilon float32 = 0.000001
const FLT_EPSILON float32 = 1.192092896e-07

func IsPowerOfTwo(mask int32) bool {
	return (mask & (mask - 1)) == 0
}

func Sqrt(v float32) float32 {
	return float32(math.Sqrt(float64(v)))
}

func NextPowerOfTwo(v uint32) uint32 {
	v -= 1
	v |= v >> 16
	v |= v >> 8
	v |= v >> 4
	v |= v >> 2
	v |= v >> 1
	return v + 1
}

func LerpFloat32(from, to, t float32) float32 {
	return to*t + from*(1.0-t)
}

func RotateVectorByQuat(lhs Quaternionf, rhs Vector3f) Vector3f {
	//  Matrix3x3f m;
	//  QuaternionToMatrix (lhs, &m);
	//  Vector3f restest = m.MultiplyVector3 (rhs);
	x := lhs.x * 2.0
	y := lhs.y * 2.0
	z := lhs.z * 2.0
	xx := lhs.x * x
	yy := lhs.y * y
	zz := lhs.z * z
	xy := lhs.x * y
	xz := lhs.x * z
	yz := lhs.y * z
	wx := lhs.w * x
	wy := lhs.w * y
	wz := lhs.w * z
	var res Vector3f
	res.x = (1.0-(yy+zz))*rhs.x + (xy-wz)*rhs.y + (xz+wy)*rhs.z
	res.y = (xy+wz)*rhs.x + (1.0-(xx+zz))*rhs.y + (yz-wx)*rhs.z
	res.z = (xz-wy)*rhs.x + (yz+wx)*rhs.y + (1.0-(xx+yy))*rhs.z

	//  Assert (CompareApproximately (restest, res));
	return res
}

func ClosestHeightPointTriangle(h *float32, p, a, b, c Vector3f) bool {
	v0 := c.Sub(a)
	v1 := b.Sub(a)
	v2 := p.Sub(a)
	dot00 := Dot2D(v0, v0)
	dot01 := Dot2D(v0, v1)
	dot02 := Dot2D(v0, v2)
	dot11 := Dot2D(v1, v1)
	dot12 := Dot2D(v1, v2)

	// Compute barycentric coordinates
	invDenom := 1.0 / (dot00*dot11 - dot01*dot01)
	u := (dot11*dot02 - dot01*dot12) * invDenom
	v := (dot00*dot12 - dot01*dot02) * invDenom

	// The (sloppy) epsilon is needed to allow to get height of points which
	// are interpolated along the edges of the triangles.
	EPS := float32(1e-4)

	// If point lies inside the triangle, return interpolated ycoord.
	if u >= -EPS && v >= -EPS && (u+v) <= 1.0+EPS {
		*h = a.y + v0.y*u + v1.y*v
		return true
	}

	return false
}

// calculate xz plan area of the triangle
// https://www.mathopenref.com/coordtrianglearea.html
func TriangleAreaXZ(pa, pb, pc Vector3f) float32 {
	aa := pa.x * (pb.z - pc.z)
	ab := pb.x * (pc.z - pa.z)
	ac := pc.x * (pa.z - pb.z)
	area := (aa + ab + ac) / 2
	if area < 0 {
		area = -area
	}
	return area
}

// Returns a random point in a convex polygon.
// Adapted from Graphics Gems article.
func RandomPointInConvexPoly(pts []Vector3f, npts int) Vector3f {
	areasum := float32(0.0)
	areas := make([]float32, npts)
	for i := 2; i < npts; i++ {
		areas[i] = TriangleAreaXZ(pts[0], pts[i-1], pts[i])
		areasum += areas[i]
	}
	// Find sub triangle weighted by area.
	targtSum := rand.Float32() * areasum
	var searchSum float32
	tri := npts - 1
	for i := 2; i < npts; i++ {
		searchSum += areas[i]
		if searchSum >= targtSum {
			tri = i
			break
		}
	}

	// Find random point in triangle
	// https://adamswaab.wordpress.com/2009/12/11/random-point-in-a-triangle-barycentric-coordinates/
	r := rand.Float32()
	s := rand.Float32()
	if r+s > 1 {
		r = 1 - r
		s = 1 - s
	}

	pa := pts[0]
	pb := pts[tri-1]
	pc := pts[tri]
	ab := pb.Sub(pa)
	ac := pc.Sub(pa)

	return pa.Add(ab.Mulf(r).Add(ac.Mulf(s)))
}

func SqrDistance2D(a, b Vector3f) float32 {
	c := b.Sub(a)
	c.y = 0.0
	return SqrMagnitude(c)
}

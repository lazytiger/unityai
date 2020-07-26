package unityai

import (
	"fmt"
	"math"
)

type Vector3f struct {
	x, y, z float32
}

var Vector3_One = Vector3f{1, 1, 1}
var Vector2_One = Vector2f{1, 1}

func NewVector3f(x, y, z float32) Vector3f {
	var v Vector3f
	v.Set(x, y, z)
	return v
}

func (this *Vector3f) Set(x, y, z float32) {
	this.x = x
	this.y = y
	this.z = z
}

func (this *Vector3f) SetData(i int, data float32) {
	switch i {
	case 0:
		this.x = data
	case 1:
		this.y = data
	case 2:
		this.z = data
	default:
		panic(fmt.Errorf("invalid vector index:%d", i))
	}
}

func (this *Vector3f) GetData(i int) float32 {
	switch i {
	case 0:
		return this.x
	case 1:
		return this.y
	case 2:
		return this.z
	default:
		panic(fmt.Errorf("invalid vector index:%d", i))
	}
}

func (this Vector3f) Sub(data Vector3f) Vector3f {
	return Vector3f{this.x - data.x, this.y - data.y, this.z - data.z}
}

func (this Vector3f) Add(data Vector3f) Vector3f {
	return Vector3f{this.x + data.x, this.y + data.y, this.z + data.z}
}

func (this Vector3f) Mulf(data float32) Vector3f {
	return Vector3f{this.x * data, this.y * data, this.z * data}
}

func (this Vector3f) Mulv(data Vector3f) Vector3f {
	return Vector3f{this.x * data.x, this.y * data.y, this.z * data.z}
}

func (this Vector3f) Div(v float32) Vector3f {
	return Vector3f{this.x / v, this.y / v, this.z / v}
}

func (this Vector3f) Neg() Vector3f {
	return Vector3f{-this.x, -this.y, -this.z}
}

func (this *Vector3f) SetZero() {
	this.x = 0
	this.y = 0
	this.z = 0
}

func ScaleVector3f(l, r Vector3f) Vector3f {
	return Vector3f{l.x * r.x, l.y * r.y, l.z * r.z}
}

func MinVector3f(l, r Vector3f) Vector3f {
	return Vector3f{FloatMin(l.x, r.x), FloatMin(l.y, r.y), FloatMin(l.z, r.z)}
}

func MaxVector3f(l, r Vector3f) Vector3f {
	return Vector3f{FloatMax(l.x, r.x), FloatMax(l.y, r.y), FloatMax(l.z, r.z)}
}

func DotVector3f(l, r Vector3f) float32 {
	return l.x*r.x + l.y*r.y + l.z*r.z
}

func Dot2D(a, b Vector3f) float32 {
	return a.x*b.x + a.z*b.z
}

func DotQuaternionf(l, r Quaternionf) float32 {
	return l.x*r.x + l.y*r.y + l.z*r.z + l.w*r.w
}

func LerpVector3f(from, to Vector3f, t float32) Vector3f {
	return to.Mulf(t).Add(from.Mulf(1.0 - t))
}

func Perp2D(u, v Vector3f) float32 {
	return u.z*v.x - u.x*v.z
}

func Magnitude(inV Vector3f) float32 {
	return float32(math.Sqrt(float64(DotVector3f(inV, inV))))
}

func Magnitude2(inV Vector2f) float32 {
	return float32(math.Sqrt(float64(DotVector2f(inV, inV))))
}

func SqrMagnitude(inV Vector3f) float32 {
	return DotVector3f(inV, inV)
}

func Distance(a, b Vector3f) float32 {
	return Magnitude(b.Sub(a))
}

func Cross(lhs, rhs Vector3f) Vector3f {
	return Vector3f{
		lhs.y*rhs.z - lhs.z*rhs.y,
		lhs.z*rhs.x - lhs.x*rhs.z,
		lhs.x*rhs.y - lhs.y*rhs.x,
	}
}

func AbsVector3f(v Vector3f) Vector3f {
	return NewVector3f(FloatAbs(v.x), FloatAbs(v.y), FloatAbs(v.z))
}

func NormalizeSafe(inV, defaultV Vector3f) Vector3f {
	mag := Magnitude(inV)
	if mag > kEpsilon {
		return inV.Div(mag)
	} else {
		return defaultV
	}
}

func NormalizeSafe2(inV, defaultV Vector2f) Vector2f {
	mag := Magnitude2(inV)
	if mag > kEpsilon {
		return inV.Div(mag)
	} else {
		return defaultV
	}
}

func CompareApproximately(inV0, inV1 Vector3f, inMaxDist float32) bool {
	return SqrMagnitude(inV1.Sub(inV0)) <= inMaxDist*inMaxDist
}

func Normalize(inV Vector3f) Vector3f {
	mag := Magnitude(inV)
	return inV.Div(mag)
}

type Vector2f struct {
	x, y float32
}

func NewVector2f(x, y float32) Vector2f {
	return Vector2f{x, y}
}
func MinVector2f(l, r Vector2f) Vector2f {
	return Vector2f{FloatMin(l.x, r.x), FloatMin(l.y, r.y)}
}
func DotVector2f(l, r Vector2f) float32 {
	return l.x*r.x + l.y*r.y
}

func MaxVector2f(l, r Vector2f) Vector2f {
	return Vector2f{FloatMax(l.x, r.x), FloatMax(l.y, r.y)}
}
func (this Vector2f) Sub(data Vector2f) Vector2f {
	return Vector2f{this.x - data.x, this.y - data.y}
}

func (this Vector2f) Div(v float32) Vector2f {
	return Vector2f{this.x / v, this.y / v}
}

func (this Vector2f) Mulf(data float32) Vector2f {
	return Vector2f{this.x * data, this.y * data}
}

func (this Vector2f) Add(data Vector2f) Vector2f {
	return Vector2f{this.x + data.x, this.y + data.y}
}

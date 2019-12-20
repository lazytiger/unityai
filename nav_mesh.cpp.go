package unityai

import (
	"math"
	"reflect"
	"sort"
	"unsafe"
)

type PortalArea struct {
	min, max float32
}

func GetWorldTileBounds(tile NavMeshTile) MinMaxAABB {
	Assert(tile.header != nil)
	tileBounds := NewMinMaxAABB(tile.header.bmin, tile.header.bmax)
	if tile.transformed != 0 {
		var mat Matrix4x4f
		mat.SetTR(tile.position, tile.rotation)
		TransformAABBSlow(tileBounds, mat, &tileBounds)
	}
	return tileBounds
}

func OverlapSlabs(amin Vector2f, amax Vector2f, bmin Vector2f, bmax Vector2f, px float32, py float32) bool {
	// Check for horizontal overlap.
	// The segment is shrunken a little so that slabs which touch
	// at end points are not connected.
	minx := FloatMax(amin.x+px, bmin.x+px)
	maxx := FloatMin(amax.x-px, bmax.x-px)
	if minx > maxx {
		return false
	}

	// Check vertical overlap.
	ad := (amax.y - amin.y) / (amax.x - amin.x)
	ak := amin.y - ad*amin.x
	bd := (bmax.y - bmin.y) / (bmax.x - bmin.x)
	bk := bmin.y - bd*bmin.x
	aminy := ad*minx + ak
	amaxy := ad*maxx + ak
	bminy := bd*minx + bk
	bmaxy := bd*maxx + bk
	dmin := bminy - aminy
	dmax := bmaxy - amaxy

	// Crossing segments always overlap.
	if dmin*dmax < 0 {
		return true
	}

	// Check for overlap at endpoints.
	thr := Sqr(py * 2)
	if dmin*dmin <= thr || dmax*dmax <= thr {
		return true
	}
	return false
}

func OverlapDetailSlabs(aslabs []Vector2f, acount int32, bslabs []Vector2f, bcount int32, px float32, py float32) bool {
	for i := int32(0); i < acount-1; i++ {
		amin := aslabs[i]
		amax := aslabs[i+1]
		for j := int32(0); j < bcount-1; j++ {
			bmin := bslabs[j]
			bmax := bslabs[j+1]
			if OverlapSlabs(amin, amax, bmin, bmax, px, py) {
				return true
			}
		}
	}
	return false
}

func GetSlabCoord(va Vector3f, side int32) float32 {
	if side == 0 || side == 4 {
		return va.x
	} else if side == 2 || side == 6 {
		return va.z
	}
	return 0
}

func MakeDetailEdgeSlabs(slabs []Vector2f, pts []Vector3f, npts int32, side int32) {
	va := pts[0]
	vb := pts[npts-1]
	if side == 0 || side == 4 {
		if va.z < vb.z {
			for i := int32(0); i < npts; i++ {
				slabs[i].x = pts[i].z
				slabs[i].y = pts[i].y
			}
		} else {
			for i := int32(0); i < npts; i++ {
				slabs[i].x = pts[npts-1-i].z
				slabs[i].y = pts[npts-1-i].y
			}
		}
	} else if side == 2 || side == 6 {
		if va.x < vb.x {
			for i := int32(0); i < npts; i++ {
				slabs[i].x = pts[i].x
				slabs[i].y = pts[i].y
			}
		} else {
			for i := int32(0); i < npts; i++ {
				slabs[i].x = pts[npts-1-i].x
				slabs[i].y = pts[npts-1-i].y
			}
		}
	} else {
		// fix warning of uninitialized bmin, bmax
		Assert(false)
		for i := int32(0); i < npts; i++ {
			slabs[i].x = 0
			slabs[i].y = 0
		}
	}
}

func OppositeTile(side int32) int32 {
	return (side + 4) & 0x7
}

func NeighbourLocation(x *int32, y *int32, side int32) {
	nx := *x
	ny := *y
	switch side {
	case 0:
		nx++
	case 1:
		nx++
		ny++
	case 2:
		ny++
	case 3:
		nx--
		ny++
	case 4:
		nx--
	case 5:
		nx--
		ny--
	case 6:
		ny--
	case 7:
		nx++
		ny--
	}
	*x = nx
	*y = ny
}

// It's assumed the triangle (a,b,c) is not degenerate - otherwise division by zero might occur
func ClosestPtPointTriangle(p Vector3f, a Vector3f, b Vector3f, c Vector3f) Vector3f {
	// Check if P in vertex region outside A
	ab := b.Sub(a)
	ac := c.Sub(a)
	ap := p.Sub(a)
	d1 := DotVector3f(ab, ap)
	d2 := DotVector3f(ac, ap)
	if d1 <= 0.0 && d2 <= 0.0 {
		// barycentric coordinates (1,0,0)
		return a
	}

	// Check if P in vertex region outside B
	bp := p.Sub(b)
	d3 := DotVector3f(ab, bp)
	d4 := DotVector3f(ac, bp)
	if d3 >= 0.0 && d4 <= d3 {
		// barycentric coordinates (0,1,0)
		return b
	}

	// Check if P in edge region of AB, if so return projection of P onto AB
	vc := d1*d4 - d3*d2
	if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
		// barycentric coordinates (1-v,v,0)
		v := d1 / (d1 - d3)
		return a.Add(ab.Mulf(v))
	}

	// Check if P in vertex region outside C
	cp := p.Sub(c)
	d5 := DotVector3f(ab, cp)
	d6 := DotVector3f(ac, cp)
	if d6 >= 0.0 && d5 <= d6 {
		// barycentric coordinates (0,0,1)
		return c
	}

	// Check if P in edge region of AC, if so return projection of P onto AC
	vb := d5*d2 - d1*d6
	if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
		// barycentric coordinates (1-w,0,w)
		w := d2 / (d2 - d6)
		return a.Add(ac.Mulf(w))
	}

	// Check if P in edge region of BC, if so return projection of P onto BC
	va := d3*d6 - d5*d4
	if va <= 0.0 && (d4-d3) >= 0.0 && (d5-d6) >= 0.0 {
		// barycentric coordinates (0,1-w,w)
		w := (d4 - d3) / ((d4 - d3) + (d5 - d6))
		return LerpVector3f(b, c, w)
	}

	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	denom := 1.0 / (va + vb + vc)
	v := vb * denom
	w := vc * denom
	return a.Add(ab.Mulf(v)).Add(ac.Mulf(w))
}

func OverlapQuantBounds(amin []uint16, amax []uint16, bmin []uint16, bmax []uint16) bool {
	overlap := true
	if amin[0] > bmax[0] || amax[0] < bmin[0] {
		overlap = false
	}
	if amin[1] > bmax[1] || amax[1] < bmin[1] {
		overlap = false
	}
	if amin[2] > bmax[2] || amax[2] < bmin[2] {
		overlap = false
	}
	return overlap
}

//////////////////////////////////////////////////////////////////////////////////////////

func NewNavMesh() *NavMesh {
	mesh := &NavMesh{
		m_firstOffMeshConnection: kNavMeshNullLink,
		m_timeStamp:              1,
		m_SurfaceIDToData:        make(map[int32]*SurfaceData),
	}
	mesh.m_tiles.Init()
	mesh.m_links.Init()
	mesh.m_offMeshConnections.Init()
	return mesh
}

// Creates a surface - set of tiles sharing some data
// Reserves room for tiles belonging for a given surface.
// Params:
//  tileCount - (in) number of tiles to reserve memory for.
//  settings - (in) settings for tiles belonging to this surface.
//
func (this *NavMesh) CreateSurface(reserveTileCount int, settings NavMeshBuildSettings, pos Vector3f, rot Quaternionf) int32 {
	// Find an unused and non-zero id
	var id int32
	for {
		id = NexSurfaceId()
		if _, ok := this.m_SurfaceIDToData[id]; ok {
			continue
		} else {
			break
		}
	}
	data := NewSurfaceData()
	data.m_Settings = settings
	data.m_Position = pos
	data.m_Rotation = rot
	data.m_TileLUT = make(map[TileLoc]uint32, reserveTileCount)
	this.m_SurfaceIDToData[id] = data
	return id
}

// Removes a surface
// Params:
//  surfaceID - (in) id of surface to remove.
func (this *NavMesh) RemoveSurface(surfaceID int32) {
	delete(this.m_SurfaceIDToData, surfaceID)
}

func (this *NavMesh) GetSurfaceSettings(surfaceID int32) *NavMeshBuildSettings {
	data, ok := this.m_SurfaceIDToData[surfaceID]
	if ok {
		return &data.m_Settings
	} else {
		return nil
	}
}

func (this *NavMesh) SetSurfaceSettings(surfaceID int32, settings NavMeshBuildSettings) {
	data := this.m_SurfaceIDToData[surfaceID]
	data.m_Settings = settings
	this.m_SurfaceIDToData[surfaceID] = data
}

func (this *NavMesh) GetSurfaceTransform(surfaceID int32, pos *Vector3f, rot *Quaternionf) NavMeshStatus {
	data, ok := this.m_SurfaceIDToData[surfaceID]
	if !ok {
		return kNavMeshFailure
	}

	*pos = data.m_Position
	*rot = data.m_Rotation
	return kNavMeshSuccess
}

const kMaxPoints = 16

// Returns all polygons in neighbour tile based on portal defined by the segment.
func (this *NavMesh) FindConnectingPolys(fromPoints []Vector3f, fromPointCount int32,
	tile *NavMeshTile, side int32, con []NavMeshPolyRef, conarea []PortalArea,
	maxcon int32, portalHeight float32) int32 {
	if tile == nil {
		return 0
	}
	if fromPointCount < 2 {
		return 0
	}
	portalHalfHeight := 0.5 * portalHeight // Slab check is symmetrical (kinda like radius).
	var fromSlabs [kMaxPoints]Vector2f
	MakeDetailEdgeSlabs(fromSlabs[:], fromPoints, fromPointCount, side)
	fromMin := fromSlabs[0]
	fromMax := fromSlabs[fromPointCount-1]
	fromCoord := GetSlabCoord(fromPoints[0], side)
	var toPoints [kMaxPoints]Vector3f
	var toSlabs [kMaxPoints]Vector2f
	var toPointCount int32 = 0

	// Remove links pointing to 'side' and compact the links array.
	m := kNavMeshExtLink | uint16(side)
	n := int32(0)
	base := this.GetPolyRefBase(tile)
	for i := int32(0); i < tile.header.polyCount; i++ {
		poly := &tile.polys[i]
		nv := poly.vertCount
		for j := uint8(0); j < nv; j++ {
			// Skip edges which do not point to the right side.
			if poly.neis[j] != m {
				continue
			}

			// Segments are not close enough along the border.
			toCoord := GetSlabCoord(tile.verts[poly.verts[j]], side)
			if FloatAbs(fromCoord-toCoord) > 0.01 {
				continue
			}

			this.GetPolyEdgeDetailPoints(tile, i, int32(j), toPoints[:], &toPointCount, kMaxPoints)
			if toPointCount == 0 {
				continue
			}
			MakeDetailEdgeSlabs(toSlabs[:], toPoints[:], toPointCount, side)
			toMin := toSlabs[0]
			toMax := toSlabs[toPointCount-1]

			// Check if the segments touch.
			if !OverlapDetailSlabs(fromSlabs[:], fromPointCount, toSlabs[:], toPointCount, 0.01, portalHalfHeight) {
				continue
			}

			// Add return value.
			if n < maxcon {
				conarea[n].min = FloatMax(fromMin.x, toMin.x)
				conarea[n].max = FloatMin(fromMax.x, toMax.x)
				con[n] = base | EncodeBasePolyId(kPolyTypeGround, uint32(i))
				n++
			}
			break
		}
	}
	return n
}

func (this *NavMesh) RemoveLinkBetween(from NavMeshPolyRef, to NavMeshPolyRef) {
	if DecodePolyIdType(from) == kPolyTypeOffMeshConnection {
		con := this.GetOffMeshConnectionUnsafe(from)
		if con != nil {
			// Remove link from connect polygon.
			k := con.firstLink
			pk := kNavMeshNullLink
			for k != kNavMeshNullLink {
				if this.m_links.Get(k).ref == to {
					nk := this.m_links.Get(k).next
					if pk == kNavMeshNullLink {
						con.firstLink = nk
					} else {
						Assert(pk != nk)
						this.m_links.Get(pk).next = nk
					}
					con.endPoints[this.m_links.Get(k).edge].tileRef = 0
					this.m_links.Release(k)
					break
				}
				pk = k
				k = this.m_links.Get(k).next
			}
		}
	} else {
		var neiTile *NavMeshTile
		var neiPoly *NavMeshPoly
		if NavMeshStatusSucceed(this.GetTileAndPolyByRef(from, &neiTile, &neiPoly)) {
			// Remove link from connect polygon.
			ip := GetPolyIndex(neiTile, neiPoly)
			k := uint32(neiTile.polyLinks[ip])
			pk := kNavMeshNullLink
			for k != kNavMeshNullLink {
				if this.m_links.Get(k).ref == to {
					nk := this.m_links.Get(k).next
					if pk == kNavMeshNullLink {
						neiTile.polyLinks[ip] = nk
					} else {
						Assert(pk != nk)
						this.m_links.Get(pk).next = nk
					}
					this.m_links.Release(k)
					break
				}
				pk = k
				k = this.m_links.Get(k).next
			}
		}
	}
}

// Removes all internal and external links from a tile.
func (this *NavMesh) UnconnectLinks(tile *NavMeshTile) {
	Assert(tile != nil)
	Assert(tile.header != nil)
	Assert(tile.polyLinks != nil)
	base := this.GetPolyRefBase(tile)
	for i := int32(0); i < tile.header.polyCount; i++ {
		polyRef := base | EncodeBasePolyId(kPolyTypeGround, uint32(i))
		j := tile.polyLinks[i]
		for j != kNavMeshNullLink {
			next := this.m_links.Get(j).next

			// Remove link to this polygon
			this.RemoveLinkBetween(this.m_links.Get(j).ref, polyRef)
			this.m_links.Release(j)
			j = next
		}
		tile.polyLinks[i] = kNavMeshNullLink
	}
}

type EdgePointSample struct {
	t  float32
	pt Vector3f
}

type EdgePointSamples []EdgePointSample

var _ sort.Interface = EdgePointSamples{}

func (this EdgePointSamples) Len() int {
	return len(this)
}

func (this EdgePointSamples) Less(i, j int) bool {
	return this[i].t < this[j].t
}

func (this EdgePointSamples) Swap(i, j int) {
	this[i], this[j] = this[j], this[i]
}

const kMaxSamples = 64

// Returns polygon edge points based on detail mesh.
// If 'maxPoints' is less than the number of points on the edge, the edge is simplified.
// Params:
//  tile - (in) pointer to tile where the polygon lies.
//  poly - (in) polygon index
//  edge - (in) edge index
//  points - (out) array to store the edge points (maxPoints*3).
//  pointCount - (out) number of points stored.
//  maxPoints - (in) max number of points that can fit into points, must be >= 2.
func (this *NavMesh) GetPolyEdgeDetailPoints(tile *NavMeshTile, p int32, edge int32,
	points []Vector3f, pointCount *int32, maxPoints int32) NavMeshStatus {
	Assert(maxPoints > 2)
	poly := &tile.polys[p]
	pd := &tile.detailMeshes[p]
	ndv := pd.vertCount
	vertCount := poly.vertCount
	v0 := tile.verts[poly.verts[edge]]
	v1 := tile.verts[poly.verts[NextIndex(edge, int32(vertCount))]]
	var samples [kMaxSamples]EdgePointSample

	// Add first edge vertex
	samples[0].t = 0.0
	samples[0].pt = v0
	sampleCount := int32(1)

	// Collect detail vertices close to the edge.
	// adds up to kMaxSamples-2 detail vertices.
	if ndv != 0 {
		EPS := float32(0.0001)
		dvx := v1.x - v0.x
		dvz := v1.z - v0.z
		lenSq := dvx*dvx + dvz*dvz
		if lenSq < EPS {
			*pointCount = 0
			return kNavMeshFailure
		}

		distanceThr := Sqr(0.01)
		nrm := 1.0 / lenSq
		dverts := tile.detailVerts[pd.vertBase:]
		for i := 0; i < int(ndv); i++ {
			dv := dverts[i]
			dpx := dv.x - v0.x
			dpz := dv.z - v0.z
			t := FloatClamp((dvx*dpx+dvz*dpz)*nrm, 0.0, 1.0)

			// Skip points projected to end points of edge
			if t < EPS || t > (1.0-EPS) {
				continue
			}

			// Skip points not close to edge
			x := t*dvx - dpx
			z := t*dvz - dpz
			distSqr := x*x + z*z
			if distSqr > distanceThr {
				continue
			}

			// Accept and add point.
			samples[sampleCount].t = t
			samples[sampleCount].pt = dv
			sampleCount++

			// Leave room for last vertex
			if sampleCount == kMaxSamples-1 {
				break
			}
		}

		// Order the collected detail vertices
		sort.Sort(EdgePointSamples(samples[1:sampleCount]))
	}

	// Add last edge vertex
	samples[sampleCount].t = 1.0
	samples[sampleCount].pt = v1
	sampleCount++

	// Simplify if needed.
	for sampleCount > maxPoints {
		// Find
		removeIdx := int32(-1)
		minDist := float32(math.MaxFloat32)
		for i := int32(1); i < sampleCount-1; i++ {
			var t float32
			d := SqrDistancePointSegment(&t, samples[i].pt, samples[i-1].pt, samples[i+1].pt)
			if d < minDist {
				minDist = d
				removeIdx = i
			}
		}
		// Remove removeIdx
		for i := removeIdx; i < sampleCount-1; i++ {
			samples[i] = samples[i+1]
		}
		sampleCount--
	}

	for i := int32(0); i < sampleCount; i++ {
		points[i] = samples[i].pt
	}
	*pointCount = sampleCount
	return kNavMeshSuccess
}

// Builds external polygon links for a tile.
func (this *NavMesh) ConnectExtLinks(tile *NavMeshTile, target *NavMeshTile, side int32, portalHeight float32) {
	if tile == nil {
		return
	}

	// Unity specific: Do not connect tiles from different surfaces.
	// We expect each navmesh surface to be separate and connected to each other only via off-mesh connections.
	if tile.surfaceID != target.surfaceID {
		return
	}

	// Connect border links.
	for i := int32(0); i < tile.header.polyCount; i++ {
		poly := &tile.polys[i]
		nv := poly.vertCount
		for j := uint8(0); j < nv; j++ {
			// Skip non-portal edges.
			if (poly.neis[j] & kNavMeshExtLink) == 0 {
				continue
			}

			dir := poly.neis[j] & 0xff
			if side != -1 && dir != uint16(side) {
				continue
			}

			// Create new links
			va := tile.verts[poly.verts[j]]
			vb := tile.verts[poly.verts[NextIndex(int32(j), int32(nv))]]
			var points [kMaxPoints]Vector3f
			var pointCount = int32(0)
			this.GetPolyEdgeDetailPoints(tile, i, int32(j), points[:], &pointCount, kMaxPoints)
			var nei [4]NavMeshPolyRef
			var neia [4]PortalArea
			nnei := this.FindConnectingPolys(points[:], pointCount, target, OppositeTile(int32(dir)), nei[:], neia[:], 4, portalHeight)
			for k := int32(0); k < nnei; k++ {
				idx := this.m_links.Alloc()
				if idx != kNavMeshNullLink {
					link := this.m_links.Get(idx)
					link.ref = nei[k]
					link.edge = j
					link.side = uint8(dir)
					Assert(idx != tile.polyLinks[i])
					link.next = tile.polyLinks[i]
					tile.polyLinks[i] = idx

					// Compress portal limits to a byte value.
					if dir == 0 || dir == 4 {
						tmin := (neia[k].min - va.z) / (vb.z - va.z)
						tmax := (neia[k].max - va.z) / (vb.z - va.z)
						if tmin > tmax {
							tmin, tmax = tmax, tmin
						}
						link.bmin = uint8(FloatClamp(tmin, 0.0, 1.0) * 255.0)
						link.bmax = uint8(FloatClamp(tmax, 0.0, 1.0) * 255.0)
					} else if dir == 2 || dir == 6 {
						tmin := (neia[k].min - va.x) / (vb.x - va.x)
						tmax := (neia[k].max - va.x) / (vb.x - va.x)
						if tmin > tmax {
							tmin, tmax = tmax, tmin
						}
						link.bmin = uint8(FloatClamp(tmin, 0.0, 1.0) * 255.0)
						link.bmax = uint8(FloatClamp(tmax, 0.0, 1.0) * 255.0)
					}
				}
			}
		}
	}
}

// Builds internal polygons links for a tile.
func (this *NavMesh) ConnectIntLinks(tile *NavMeshTile) {
	Assert(tile != nil)
	Assert(tile.header != nil)
	polyCount := tile.header.polyCount

	// Allocate and nullify per poly links
	Assert(tile.polyLinks == nil)
	tile.polyLinks = make([]uint32, polyCount)
	for i := int32(0); i < polyCount; i++ {
		tile.polyLinks[i] = kNavMeshNullLink
	}

	base := this.GetPolyRefBase(tile)
	for i := int32(0); i < polyCount; i++ {
		poly := &tile.polys[i]

		// Build edge links backwards so that the links will be
		// in the linked list from lowest index to highest.
		for j := int32(poly.vertCount) - 1; j >= 0; j-- {
			// Skip hard and non-internal edges.
			if poly.neis[j] == 0 || (poly.neis[j]&kNavMeshExtLink) != 0 {
				continue
			}

			idx := this.m_links.Alloc()
			if idx != kNavMeshNullLink {
				link := this.m_links.Get(idx)
				link.ref = base | EncodeBasePolyId(kPolyTypeGround, uint32(poly.neis[j]-1))
				link.edge = uint8(j)
				link.side = 0xff
				link.bmin = 0
				link.bmax = 0
				// Add to linked list.
				Assert(idx != tile.polyLinks[i])
				link.next = tile.polyLinks[i]
				tile.polyLinks[i] = idx
			}
		}
	}
}

// Returns closest point on polygon.
func (this *NavMesh) ClosestPointOnPolyInTileLocal(tile *NavMeshTile, poly *NavMeshPoly, pos Vector3f, closest *Vector3f) {
	ip := GetPolyIndex(tile, poly)
	pd := &tile.detailMeshes[ip]
	closestDistSqr := float32(math.MaxFloat32)
	for j := 0; j < int(pd.triCount); j++ {
		t := tile.detailTris[(int(pd.triBase)+j)*4:]
		var v [3]Vector3f
		for k := 0; k < 3; k++ {
			if uint8(t[k]) < poly.vertCount {
				v[k] = tile.verts[poly.verts[t[k]]]
			} else {
				v[k] = tile.detailVerts[uint8(pd.vertBase)+(uint8(t[k])-poly.vertCount)]
			}
		}

		pt := ClosestPtPointTriangle(pos, v[0], v[1], v[2])
		d := SqrDistance(pos, pt)
		if d < closestDistSqr {
			*closest = pt
			closestDistSqr = d
		}
	}
}

type NearestQuery struct {
	m_NavMesh     *NavMesh
	m_PolyRef     NavMeshPolyRef
	m_DistanceSqr float32
	m_Center      Vector3f
	m_Point       Vector3f
}

func NewNearestQuery(navmesh *NavMesh, center Vector3f) NearestQuery {
	return NearestQuery{
		navmesh, 0, math.MaxFloat32, center, Vector3f{0, 0, 0},
	}
}

func (this *NearestQuery) ProcessPolygons(tile *NavMeshTile, polyRefs []NavMeshPolyRef, polys []*NavMeshPoly, itemCount int) {
	localPosition := WorldToTile(tile, this.m_Center)
	// Find nearest polygon amongst the nearby polygons.
	for i := 0; i < itemCount; i++ {
		ref := polyRefs[i]
		poly := polys[i]
		var closestPtPoly Vector3f
		this.m_NavMesh.ClosestPointOnPolyInTileLocal(tile, poly, localPosition, &closestPtPoly)
		d := SqrDistance(localPosition, closestPtPoly)
		if d < this.m_DistanceSqr {
			this.m_Point = closestPtPoly
			this.m_DistanceSqr = d
			this.m_PolyRef = ref
		}
	}
}

// Find nearest polygon within a tile.
func (this *NavMesh) FindNearestPoly(typeID int32, center Vector3f, extents Vector3f, nearestPt *Vector3f) NavMeshPolyRef {

	// Get nearby polygons from proximity grid.
	nearest := NewNearestQuery(this, center)
	this.QueryPolygons(typeID, center, extents, &nearest)
	if nearest.m_PolyRef == 0 {
		return 0
	}
	if nearestPt != nil {
		tile := this.GetTileByRef(NavMeshTileRef(nearest.m_PolyRef))
		if tile != nil {
			*nearestPt = TileToWorld(tile, nearest.m_Point)
		}
	}

	return nearest.m_PolyRef
}

// Returns polygons which overlap the query box.
// Params:
//  typeID - (in) query surfaces with this id. -1 means query all surfaces.
//  center[3] - (in) the center of the search box.
//  extents[3] - (in) the extents of the search box.
//  callback - (in/out) pointer to callback interface to batch-process the results, see NavMeshProcessCallback.
func (this *NavMesh) QueryPolygons(typeID int32, cen Vector3f, ext Vector3f, callback NavMeshProcessCallback) {
	bounds := NewMinMaxAABB(cen.Sub(ext), cen.Add(ext))
	for _, surf := range this.m_SurfaceIDToData {
		if typeID != -1 && typeID != surf.m_Settings.agentTypeID {
			continue
		}

		var overlap MinMaxAABB
		if !IntersectionAABBAABB(bounds, surf.m_WorldBounds, &overlap) {
			continue
		}

		var localBounds AABB
		InverseTransformAABB(NewAABBFromMinMax(overlap), surf.m_Position, surf.m_Rotation, &localBounds)
		invTileSize := 1.0 / (float32(surf.m_Settings.tileSize) * surf.m_Settings.cellSize)
		iminx := FloorfToInt((localBounds.m_Center.x - localBounds.m_Extent.x) * invTileSize)
		imaxx := FloorfToInt((localBounds.m_Center.x + localBounds.m_Extent.x) * invTileSize)
		jminz := FloorfToInt((localBounds.m_Center.z - localBounds.m_Extent.z) * invTileSize)
		jmaxz := FloorfToInt((localBounds.m_Center.z + localBounds.m_Extent.z) * invTileSize)
		for i := iminx; i <= imaxx; i++ {
			for j := jminz; j <= jmaxz; j++ {
				key := TileLoc{i, j}
				tileIndex, ok := surf.m_TileLUT[key]
				if !ok {
					continue
				}
				this.QueryPolygonsInTile(this.m_tiles.Get(tileIndex), localBounds.m_Center, localBounds.m_Extent, callback)
			}
		}
	}
}

const kBufferSize = 32

// Queries polygons within a tile.
func (this *NavMesh) QueryPolygonsInTile(tile *NavMeshTile, center Vector3f, extents Vector3f, callback NavMeshProcessCallback) {
	var polyRefs [kBufferSize]NavMeshPolyRef
	var polys [kBufferSize]*NavMeshPoly
	qmin := center.Sub(extents)
	qmax := center.Add(extents)
	if tile.bvTree != nil {
		offset := int32(0)
		node := &tile.bvTree[offset]
		// The final node is garbage as we allocate 2*polyCount but need one less for the tree.
		// Here we make sure the end node is not reached by subtracting 1
		end := &tile.bvTree[tile.header.bvNodeCount-1]
		tbmin := tile.header.bmin
		tbmax := tile.header.bmax
		qfac := tile.header.bvQuantFactor

		// Calculate quantized box
		var bmin, bmax [3]uint16
		// FloatClamp query box to world box.
		minx := FloatClamp(qmin.x, tbmin.x, tbmax.x) - tbmin.x
		miny := FloatClamp(qmin.y, tbmin.y, tbmax.y) - tbmin.y
		minz := FloatClamp(qmin.z, tbmin.z, tbmax.z) - tbmin.z
		maxx := FloatClamp(qmax.x, tbmin.x, tbmax.x) - tbmin.x
		maxy := FloatClamp(qmax.y, tbmin.y, tbmax.y) - tbmin.y
		maxz := FloatClamp(qmax.z, tbmin.z, tbmax.z) - tbmin.z
		// Quantize
		bmin[0] = uint16(qfac*minx) & 0xfffe
		bmin[1] = uint16(qfac*miny) & 0xfffe
		bmin[2] = uint16(qfac*minz) & 0xfffe
		bmax[0] = uint16(qfac*maxx+1) | 1
		bmax[1] = uint16(qfac*maxy+1) | 1
		bmax[2] = uint16(qfac*maxz+1) | 1

		// Traverse tree
		base := this.GetPolyRefBase(tile)
		n := 0
		for uintptr(unsafe.Pointer(node)) < uintptr(unsafe.Pointer(end)) {
			overlap := OverlapQuantBounds(bmin[:], bmax[:], node.bmin[:], node.bmax[:])
			isLeafNode := node.i >= 0
			if isLeafNode && overlap {
				if n >= kBufferSize {
					callback.ProcessPolygons(tile, polyRefs[:], polys[:], n)
					n = 0
				}
				polyRefs[n] = base | EncodeBasePolyId(kPolyTypeGround, uint32(node.i))
				polys[n] = &tile.polys[node.i]
				n++
			}

			if overlap || isLeafNode {
				offset++
				node = &tile.bvTree[offset]
			} else {
				escapeIndex := -node.i
				offset += escapeIndex
				node = &tile.bvTree[offset]
			}
		}
		if n > 0 {
			callback.ProcessPolygons(tile, polyRefs[:], polys[:], n)
		}
	} else {
		var bmin, bmax Vector3f
		n := 0
		base := this.GetPolyRefBase(tile)
		for i := int32(0); i < tile.header.polyCount; i++ {
			// Calc polygon bounds.
			p := &tile.polys[i]
			bmax = tile.verts[p.verts[0]]
			bmin = bmax
			for j := uint8(1); j < p.vertCount; j++ {
				v := tile.verts[p.verts[j]]
				bmin = MinVector3f(bmin, v)
				bmax = MaxVector3f(bmax, v)
			}
			if OverlapBounds(qmin, qmax, bmin, bmax) {
				if n >= kBufferSize {
					callback.ProcessPolygons(tile, polyRefs[:], polys[:], n)
					n = 0
				}
				polyRefs[n] = base | EncodeBasePolyId(kPolyTypeGround, uint32(i))
				polys[n] = p
				n++
			}
		}
		if n > 0 {
			callback.ProcessPolygons(tile, polyRefs[:], polys[:], n)
		}
	}
}

// Adds new tile into the navmesh.
// The add will fail if the data is in wrong format,
// there is not enough tiles left, or if there is a tile already at the location.
// Params:
//  data - (in) Data of the new tile mesh.
//  dataSize - (in) Data size of the new tile mesh.
//  flags - (in) Tile flags, see NavMeshTileFlags.
//  surfaceID - (in) surface identifier for the tile.
//  result - (out,optional) tile ref if the tile was successfully added.
func (this *NavMesh) AddTile(data []byte, dataSize int32, flags NavMeshTileFlags, surfaceID int32, result *NavMeshTileRef) NavMeshStatus {
	// Make sure the data is in right format.
	header := (*NavMeshDataHeader)(unsafe.Pointer(&(data[0])))
	if header.magic != kNavMeshMagic {
		return kNavMeshFailure | kNavMeshWrongMagic
	}
	if header.version != kNavMeshVersion {
		return kNavMeshFailure | kNavMeshWrongVersion
	}

	// Patch header pointers.
	headerSize := Align4(unsafe.Sizeof(NavMeshDataHeader{}))
	vertsSize := Align4(unsafe.Sizeof(Vector3f{}) * uintptr(header.vertCount))
	polysSize := Align4(unsafe.Sizeof(NavMeshPoly{}) * uintptr(header.polyCount))
	detailMeshesSize := Align4(unsafe.Sizeof(NavMeshPolyDetail{}) * uintptr(header.detailMeshCount))
	detailVertsSize := Align4(unsafe.Sizeof(Vector3f{}) * uintptr(header.detailVertCount))
	detailTrisSize := Align4(unsafe.Sizeof(NavMeshPolyDetailIndex(0)) * 4 * uintptr(header.detailTriCount))
	bvtreeSize := Align4(unsafe.Sizeof(NavMeshBVNode{}) * uintptr(header.bvNodeCount))
	d := headerSize

	var verts []Vector3f
	sliceHeader := (*reflect.SliceHeader)(unsafe.Pointer(&(verts)))
	sliceHeader.Cap = int(header.vertCount)
	sliceHeader.Len = int(header.vertCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(data[d])))
	d += vertsSize

	var polys []NavMeshPoly
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(polys)))
	sliceHeader.Cap = int(header.polyCount)
	sliceHeader.Len = int(header.polyCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(data[d])))
	d += polysSize

	var detailMeshes []NavMeshPolyDetail
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(detailMeshes)))
	sliceHeader.Cap = int(header.detailMeshCount)
	sliceHeader.Len = int(header.detailMeshCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(data[d])))
	d += detailMeshesSize

	var detailVerts []Vector3f
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(detailVerts)))
	sliceHeader.Cap = int(header.detailVertCount)
	sliceHeader.Len = int(header.detailVertCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(data[d])))
	d += detailVertsSize

	var detailTris []NavMeshPolyDetailIndex
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(detailTris)))
	sliceHeader.Cap = int(header.detailTriCount * 4)
	sliceHeader.Len = int(header.detailTriCount * 4)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(data[d])))
	d += detailTrisSize

	var bvTree []NavMeshBVNode
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(bvTree)))
	sliceHeader.Cap = int(header.bvNodeCount)
	sliceHeader.Len = int(header.bvNodeCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(data[d])))
	d += bvtreeSize

	if d != uint32(dataSize) {
		println(d, dataSize)
		return kNavMeshFailure
	}

	// Allocate a tile.
	newTileIndex := this.m_tiles.Alloc()

	// Make sure we could allocate a tile.
	if newTileIndex == kNavMeshNullLink {
		return kNavMeshFailure | kNavMeshOutOfMemory
	}
	_, ok := this.m_SurfaceIDToData[surfaceID]
	Assert(ok)
	tile := this.m_tiles.Get(newTileIndex)

	// Insert tile into the position lut.
	surfaceData := this.m_SurfaceIDToData[surfaceID]
	lut := surfaceData.m_TileLUT
	lut[TileLoc{header.x, header.y}] = newTileIndex

	position := surfaceData.m_Position
	rotation := surfaceData.m_Rotation

	// Consider: Post process all these
	var tileWorldBounds AABB
	tilebounds := NewMinMaxAABB(header.bmin, header.bmax)
	TransformAABB(NewAABBFromMinMax(tilebounds), position, rotation, &tileWorldBounds)
	surfaceData.m_WorldBounds.Encapsulate(NewMinMaxAABBFromAABB(tileWorldBounds))

	// Patch header pointers.
	tile.verts = verts
	tile.polys = polys
	tile.detailMeshes = detailMeshes
	tile.detailVerts = detailVerts
	tile.detailTris = detailTris
	tile.bvTree = bvTree

	// If there are no items in the bvtree, reset the tree pointer.
	if bvtreeSize == 0 {
		tile.bvTree = nil
	}

	// Init tile.
	tile.surfaceID = surfaceID
	tile.header = header
	tile.polyLinks = nil
	tile.data = data
	tile.dataSize = dataSize
	tile.flags = int32(flags)
	epsilon := float32(0.00001)
	if (CompareApproximatelyV(Vector3f{0, 0, 0}, position, epsilon) &&
		CompareApproximatelyQ(Quaternionf{0, 0, 0, 1}, rotation, epsilon)) {
		tile.rotation = Quaternionf{0, 0, 0, 1}
		tile.position = Vector3f{0, 0, 0}
		tile.transformed = 0
	} else {
		tile.rotation = rotation
		tile.position = position
		tile.transformed = 1
	}

	this.ConnectIntLinks(tile)

	// Create connections to the 4 nearest neighbour tiles.
	portalHeight := surfaceData.m_Settings.cellSize
	for i := int32(0); i < 8; i += 2 {
		x := tile.header.x
		y := tile.header.y
		NeighbourLocation(&x, &y, i)
		tileIndex, ok := lut[TileLoc{x, y}]
		if !ok {
			continue
		}
		ntile := this.m_tiles.Get(tileIndex)
		this.ConnectExtLinks(tile, ntile, i, portalHeight)
		this.ConnectExtLinks(ntile, tile, OppositeTile(i), portalHeight)
	}

	this.ConnectOffMeshConnectionsToTile(tile)
	if result != nil {
		*result = this.GetTileRef(tile)
	}
	return kNavMeshSuccess
}

// Returns tile based on references.
func (this *NavMesh) GetTileByRef(ref NavMeshTileRef) *NavMeshTile {
	if ref == 0 {
		return nil
	}
	tileIndex := DecodePolyIdTile(NavMeshPolyRef(ref))
	tileSalt := DecodePolyIdSalt(NavMeshPolyRef(ref))
	if tileIndex >= uint32(this.m_tiles.Capacity()) {
		return nil
	}
	tile := this.m_tiles.Get(tileIndex)
	if tile.salt != tileSalt {
		return nil
	}
	return tile
}

// Returns pointer to tile and polygon pointed by the polygon reference.
// Params:
//  ref - (in) reference to a polygon.
//  tile - (out) pointer to the tile containing the polygon.
//  poly - (out) pointer to the polygon.
func (this *NavMesh) GetTileAndPolyByRef(ref NavMeshPolyRef, tile **NavMeshTile, poly **NavMeshPoly) NavMeshStatus {
	if ref == 0 {
		return kNavMeshFailure
	}

	var salt, it, typ, ip uint32
	DecodePolyId(&salt, &it, &typ, &ip, ref)
	if it >= this.m_tiles.Capacity() {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	if this.m_tiles.Get(it).salt != salt || this.m_tiles.Get(it).header == nil {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	if typ == kPolyTypeOffMeshConnection {
		*tile = nil
		*poly = nil
	} else {
		if int32(ip) >= this.m_tiles.Get(it).header.polyCount {
			return kNavMeshFailure | kNavMeshInvalidParam
		}
		*tile = this.m_tiles.Get(it)
		*poly = &this.m_tiles.Get(it).polys[ip]
	}
	return kNavMeshSuccess
}

// Returns pointer to tile and polygon pointed by the polygon reference.
// Note: this function does not check if 'ref' s valid, and is thus faster. Use only with valid refs!
// Params:
//  ref - (in) reference to a polygon.
//  tile - (out) pointer to the tile containing the polygon.
//  poly - (out) pointer to the polygon.
func (this *NavMesh) GetTileAndPolyByRefUnsafe(ref NavMeshPolyRef, tile **NavMeshTile, poly **NavMeshPoly) {
	var salt, it, typ, ip uint32
	DecodePolyId(&salt, &it, &typ, &ip, ref)
	if typ == kPolyTypeOffMeshConnection {
		*tile = nil
		*poly = nil
	} else {
		*tile = this.m_tiles.Get(it)
		*poly = &this.m_tiles.Get(it).polys[ip]
	}
}

// Returns true if polygon reference points to valid data.
func (this *NavMesh) IsValidPolyRef(ref NavMeshPolyRef) bool {
	if ref == 0 {
		return false
	}
	var salt, it, typ, ip uint32
	DecodePolyId(&salt, &it, &typ, &ip, ref)
	if typ == kPolyTypeOffMeshConnection {
		if ip >= this.m_offMeshConnections.Capacity() {
			return false
		}
		if this.m_offMeshConnections.Get(ip).salt != salt {
			return false
		}
	} else {
		if it >= this.m_tiles.Capacity() {
			return false
		}
		if this.m_tiles.Get(it).salt != salt || this.m_tiles.Get(it).header == nil {
			return false
		}
		if int32(ip) >= this.m_tiles.Get(it).header.polyCount {
			return false
		}
	}
	return true
}

// Returns the agentTypeId that is allowed to use the polygon.
func (this *NavMesh) GetAgentTypeIdForPolyRef(ref NavMeshPolyRef) int32 {
	if ref == 0 {
		return 0
	}

	var salt, it, typ, ip uint32
	DecodePolyId(&salt, &it, &typ, &ip, ref)
	if typ == kPolyTypeOffMeshConnection {
		if ip >= this.m_offMeshConnections.Capacity() {
			return kInvalidAgentTypeID
		}

		if this.m_offMeshConnections.Get(ip).salt != salt {
			return kInvalidAgentTypeID
		}

		return this.m_offMeshConnections.Get(ip).agentTypeID
	} else {
		if it >= this.m_tiles.Capacity() {
			return kInvalidAgentTypeID
		}

		if this.m_tiles.Get(it).salt != salt || this.m_tiles.Get(it).header == nil {
			return kInvalidAgentTypeID
		}

		if int32(ip) >= this.m_tiles.Get(it).header.polyCount {
			return kInvalidAgentTypeID
		}

		return int32(this.m_tiles.Get(it).header.agentTypeId)
	}
}

// Removes specified tile.
// Params:
//  ref - (in) Reference to the tile to remove.
//  surfaceID - (in) surface identifier for the tile
//  data - (out) Data associated with deleted tile.
//  dataSize - (out) Size of the data associated with deleted tile.
func (this *NavMesh) RemoveTile(ref NavMeshTileRef, surfaceID int32, data *[]byte, dataSize *int32) NavMeshStatus {
	if ref == 0 {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	tileIndex := DecodePolyIdTile(NavMeshPolyRef(ref))
	tileSalt := DecodePolyIdSalt(NavMeshPolyRef(ref))
	if tileIndex >= this.m_tiles.Capacity() {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	tile := this.m_tiles.Get(tileIndex)
	if tile.salt != tileSalt {
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	// Remove tile from hash lookup.
	surfaceData, ok := this.m_SurfaceIDToData[surfaceID]
	Assert(ok)
	lut := surfaceData.m_TileLUT
	delete(lut, TileLoc{tile.header.x, tile.header.y})

	// Remove links to connected tiles.
	this.UnconnectLinks(tile)
	// Remove off-mesh connections connected to the tile.
	// unconnectLinks() unconnects most of them, but one-directional
	// links may not be unconnected.
	this.UnconnectOffMeshConnectionsToTile(ref)

	// Reset tile.
	if tile.flags&int32(kTileFreeData) != 0 {
		// We own the data - it's ok to cast away const and free it.
		if data != nil {
			*data = nil
		}
		if dataSize != nil {
			*dataSize = 0
		}
	} else {
		if data != nil {
			*data = tile.data
		}
		if dataSize != nil {
			*dataSize = tile.dataSize
		}
	}
	tile.data = nil
	tile.dataSize = 0
	tile.polyLinks = nil
	tile.header = nil
	tile.flags = 0
	tile.polys = nil
	tile.verts = nil
	tile.detailMeshes = nil
	tile.detailVerts = nil
	tile.detailTris = nil
	tile.bvTree = nil

	// Update salt, salt should never be zero.
	tile.salt = (tile.salt + 1) & ((1 << kPolyRefSaltBits) - 1)
	if tile.salt == 0 {
		tile.salt++
	}

	// Add to free list.
	this.m_tiles.Release(tileIndex)
	return kNavMeshSuccess
}

// Returns tile references of a tile based on tile pointer.
func (this *NavMesh) GetTileRef(tile *NavMeshTile) NavMeshTileRef {
	if tile == nil {
		return 0
	}
	it := GetTileIndex(this.m_tiles.Get(0), tile)
	return NavMeshTileRef(EncodePolyId(tile.salt, uint32(it), 0, 0))
}

// Returns base poly id for specified tile, polygon refs can be deducted from this.
func (this *NavMesh) GetPolyRefBase(tile *NavMeshTile) NavMeshPolyRef {
	if tile == nil {
		return NavMeshPolyRef(0)
	}

	it := GetTileIndex(this.m_tiles.Get(0), tile)
	return EncodePolyId(tile.salt, uint32(it), 0, 0)
}

// Returns start and end location of an off-mesh link polygon.
// As seen when approached from prevRef.
// Returns start and end location of an off-mesh link polygon.
// Params:
//  prevRef - (in) ref to the polygon before the link (used to select direction).
//  polyRef - (in) ref to the off-mesh link polygon.
//  startPos[3] - (out) start point of the link.
//  endPos[3] - (out) end point of the link.
// Returns: true if link is found.
func (this *NavMesh) GetOffMeshConnectionEndPoints(prevRef NavMeshPolyRef, offMeshPolyRef NavMeshPolyRef, startPos *Vector3f, endPos *Vector3f) NavMeshStatus {
	// Make sure that the current poly is indeed off-mesh link.
	if DecodePolyIdType(offMeshPolyRef) != kPolyTypeOffMeshConnection {

		return kNavMeshFailure | kNavMeshInvalidParam
	}

	con := this.GetOffMeshConnection(offMeshPolyRef)
	if con == nil {
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	// Find link which leads to previous polygon to figure out which way we are traversing the off-mesh connection.
	var foundLink *NavMeshLink
	for link := this.GetLink(con.firstLink); link != nil; link = this.GetNextLink(link) {
		if link.ref == prevRef {
			foundLink = link
			break
		}
	}
	if foundLink == nil {
		return kNavMeshFailure
	}

	// OffMesh connection can only have edge 0 or 1
	Assert(foundLink.edge == 0 || foundLink.edge == 1)

	// Set endpoints based on direction
	if foundLink.edge == 0 {
		*startPos = con.endPoints[0].pos
		*endPos = con.endPoints[1].pos
		return kNavMeshSuccess
	} else if foundLink.edge == 1 {
		*startPos = con.endPoints[1].pos
		*endPos = con.endPoints[0].pos
		return kNavMeshSuccess
	}

	return kNavMeshFailure
}

func (this *NavMesh) GetNearestOffMeshConnectionEndPoints(prevRef NavMeshPolyRef, offMeshPolyRef NavMeshPolyRef, nextRef NavMeshPolyRef, currentPos Vector3f, startPos *Vector3f, endPos *Vector3f) NavMeshStatus {
	// Make sure that the current poly is indeed off-mesh link.
	if DecodePolyIdType(offMeshPolyRef) != kPolyTypeOffMeshConnection {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	con := this.GetOffMeshConnection(offMeshPolyRef)
	if con == nil {
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	// Find link which leads to previous polygon to figure out which way we are traversing the off-mesh connection.
	var prevLink *NavMeshLink
	for link := this.GetLink(con.firstLink); link != nil; link = this.GetNextLink(link) {
		if link.ref == prevRef {
			prevLink = link
			break
		}
	}
	if prevLink == nil {
		return kNavMeshFailure
	}

	// Find link which leads to next polygon to figure out which way we are traversing the off-mesh connection.
	var nextLink *NavMeshLink
	for link := this.GetLink(con.firstLink); link != nil; link = this.GetNextLink(link) {
		if link.ref == nextRef {
			nextLink = link
			break
		}
	}
	if nextLink == nil {
		return kNavMeshFailure
	}

	// OffMesh connection can only have edge 0 or 1
	Assert(prevLink.edge == 0 || prevLink.edge == 1)
	if con.width > 0.0 {
		// For wide connections we try to go across at the same relative locations.
		// I.e. of the nearest point along the entry segment is at t=0.3, then we try
		// to choose the same proportional location at the exit segment too.
		// We clamp the entry and exit locations to the links. This is done since
		// the connection can be linked to many polygons at the destination.
		// Note:    It should be possible to constrain the link traversal during the search
		//          so that we only exit at links which overlap the input range. That way
		//          we could have the proportional locations to match at entry and exit.
		var startP, startQ, endP, endQ Vector3f
		if prevLink.edge == 0 {
			startP = con.endPoints[0].mapped[0]
			startQ = con.endPoints[0].mapped[1]
			endP = con.endPoints[1].mapped[0]
			endQ = con.endPoints[1].mapped[1]
		} else if prevLink.edge == 1 {
			startP = con.endPoints[1].mapped[0]
			startQ = con.endPoints[1].mapped[1]
			endP = con.endPoints[0].mapped[0]
			endQ = con.endPoints[0].mapped[1]
		} else {
			startP = Vector3f{0, 0, 0}
			startQ = Vector3f{0, 0, 0}
			endP = Vector3f{0, 0, 0}
			endQ = Vector3f{0, 0, 0}
		}

		// Find proportional location on entry segment.
		t0 := float32(0.0)
		SqrDistancePointSegment2D(&t0, currentPos, startP, startQ)
		t1 := 1.0 - t0 // Exit segment points are in reverse.

		// Clamp both factors to link range.
		s := float32(1.0) / 255.0
		startTmin := FloatMax(float32(prevLink.bmin)*s, 0.0)
		startTmax := FloatMin(float32(prevLink.bmax)*s, 1.0)
		endTmin := FloatMax(float32(nextLink.bmin)*s, 0.0)
		endTmax := FloatMin(float32(nextLink.bmax)*s, 1.0)
		t0 = FloatClamp(t0, startTmin, startTmax)
		t1 = FloatClamp(t1, endTmin, endTmax)
		*startPos = LerpVector3f(startP, startQ, t0)
		*endPos = LerpVector3f(endP, endQ, t1)
		return kNavMeshSuccess
	} else {
		// For point-to-point connections we use the end points provided by the user.
		// Set endpoints based on direction
		if prevLink.edge == 0 {
			*startPos = con.endPoints[0].pos
			*endPos = con.endPoints[1].pos
		} else if prevLink.edge == 1 {
			*startPos = con.endPoints[1].pos
			*endPos = con.endPoints[0].pos
		}

		return kNavMeshSuccess
	}
}

// polyRef - polyRef of the offmeshlink polygon
// costOverride - use this as cost OffMeshLink cost modifier instead of what is specified by areas.
func (this *NavMesh) SetOffMeshConnectionCostModifier(ref NavMeshPolyRef, costOverride float32) NavMeshStatus {
	if DecodePolyIdType(ref) != kPolyTypeOffMeshConnection {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	con := this.GetOffMeshConnectionUnsafe(ref)
	if con == nil {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	if costOverride >= 0.0 {
		con.costModifier = costOverride
	} else {
		con.costModifier = -1.0
	}
	this.BumpTimeStamp()
	return kNavMeshSuccess
}

func (this *NavMesh) SetOffMeshConnectionFlags(ref NavMeshPolyRef, flags uint32) NavMeshStatus {
	if DecodePolyIdType(ref) != kPolyTypeOffMeshConnection {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	con := this.GetOffMeshConnectionUnsafe(ref)
	if con == nil {
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	con.flags = flags
	this.BumpTimeStamp()
	return kNavMeshSuccess
}

func (this *NavMesh) GetOffMeshConnectionUserID(ref NavMeshPolyRef, userID *int32) NavMeshStatus {
	if DecodePolyIdType(ref) != kPolyTypeOffMeshConnection {
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	con := this.GetOffMeshConnection(ref)
	if con == nil {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	*userID = con.userID
	return kNavMeshSuccess
}

func (this *NavMesh) SetOffMeshConnectionUserID(ref NavMeshPolyRef, userID int32) NavMeshStatus {
	if DecodePolyIdType(ref) != kPolyTypeOffMeshConnection {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	con := this.GetOffMeshConnectionUnsafe(ref)
	if con == nil {
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	con.userID = userID
	return kNavMeshSuccess
}

func (this *NavMesh) GetOffMeshConnection(ref NavMeshPolyRef) *OffMeshConnection {
	if DecodePolyIdType(ref) != kPolyTypeOffMeshConnection {

		return nil
	}
	index := DecodePolyIdPoly(ref)
	if index >= this.m_offMeshConnections.Capacity() {
		return nil
	}
	salt := DecodePolyIdSalt(ref)
	if salt != this.m_offMeshConnections.Get(index).salt {
		return nil
	}
	return this.m_offMeshConnections.Get(index)
}

func (this *NavMesh) GetOffMeshConnectionUnsafe(ref NavMeshPolyRef) *OffMeshConnection {
	if DecodePolyIdType(ref) != kPolyTypeOffMeshConnection {
		return nil
	}

	index := DecodePolyIdPoly(ref)
	if index >= this.m_offMeshConnections.Capacity() {
		return nil
	}

	salt := DecodePolyIdSalt(ref)
	if salt != this.m_offMeshConnections.Get(index).salt {
		return nil
	}

	return this.m_offMeshConnections.Get(index)
}

func TestPointInCylinder(point Vector3f, center Vector3f, halfHeight float32, radius float32) bool {
	if Sqr(point.x-center.x)+Sqr(point.z-center.z) > Sqr(radius) {
		return false
	}
	return FloatAbs(point.y-center.y) <= halfHeight
}

func (this *NavMesh) ConnectOffMeshConnectionsToTile(tile *NavMeshTile) {
	tileBounds := GetWorldTileBounds(*tile)
	agentTypeID := tile.header.agentTypeId
	settings := this.m_SurfaceIDToData[tile.surfaceID].m_Settings
	for i := this.m_firstOffMeshConnection; i != kNavMeshNullLink; i = this.m_offMeshConnections.Get(i).next {
		con := this.m_offMeshConnections.Get(i)
		if con.agentTypeID != int32(agentTypeID) {
			continue
		}
		// TODO: it might be possible that adding a new tile allows the end point to be mapped closer.
		//      if (con.endPoints[0].tileRef && con.endPoints[1].tileRef)
		//          continue;
		if IntersectAABBAABB(con.bounds, tileBounds) {
			this.ConnectOffMeshConnection(i, settings.agentRadius, settings.agentClimb)
		}
	}
}

type OverlapQuery struct {
	m_NavMesh       *NavMesh
	m_Start         Vector3f
	m_End           Vector3f
	m_Height        float32
	m_Polys         []NavMeshPolyRef
	m_OverlapMinMax []float32
	m_PolyCount     int
	m_MaxPolys      int
}

func NewOverlapQuery(
	navmesh *NavMesh, start Vector3f, end Vector3f, height float32, polys []NavMeshPolyRef, overlapMinMax []float32, maxPolys int) OverlapQuery {
	return OverlapQuery{
		m_NavMesh: navmesh, m_Start: start, m_End: end, m_Height: height,
		m_Polys: polys, m_OverlapMinMax: overlapMinMax, m_PolyCount: 0, m_MaxPolys: maxPolys,
	}
}

func (this *OverlapQuery) ProcessPolygons(tile *NavMeshTile, polyRefs []NavMeshPolyRef, polys []*NavMeshPoly, itemCount int) {
	localStart := WorldToTile(tile, this.m_Start)
	localEnd := WorldToTile(tile, this.m_End)

	// Find nearest polygon amongst the nearby polygons.
	for i := 0; i < itemCount; i++ {
		ref := polyRefs[i]
		var verts [kNavMeshVertsPerPoly]Vector3f
		nverts := this.m_NavMesh.GetPolyGeometry(ref, verts[:], nil, 0)
		if nverts == 0 {
			continue
		}

		var tmin, tmax float32
		var segMin, segMax int32
		if !IntersectSegmentPoly2D(&tmin, &tmax, &segMin, &segMax, localStart, localEnd, verts[:], nverts) {
			continue
		}

		tmin = FloatClamp(tmin, 0, 1)
		tmax = FloatClamp(tmax, 0, 1)
		if tmin >= tmax {
			continue
		}

		if this.m_PolyCount >= this.m_MaxPolys {
			return
		}

		this.m_Polys[this.m_PolyCount] = ref
		this.m_OverlapMinMax[this.m_PolyCount*2+0] = tmin
		this.m_OverlapMinMax[this.m_PolyCount*2+1] = tmax
		this.m_PolyCount++
	}
}

func (this *NavMesh) FindPolygonsOverlappingSegment(typeID int32, pa Vector3f, pb Vector3f, height float32, polys []NavMeshPolyRef, overlapMinMax []float32, polyCount *int, maxPolys int) {

	bmin := MinVector3f(pa, pb)
	bmax := MaxVector3f(pa, pb)
	bmin.y -= height
	bmax.y += height

	// Get nearby polygons from proximity grid.
	overlap := NewOverlapQuery(this, pa, pb, height, polys, overlapMinMax, maxPolys)
	this.QueryPolygons(typeID, bmax.Add(bmin).Mulf(0.5), bmax.Sub(bmin).Mulf(0.5), &overlap)
	*polyCount = overlap.m_PolyCount
}

const kMaxResults = 32

func (this *NavMesh) ConnectOffMeshConnection(index uint32, connectRadius float32, connectHeight float32) {
	con := this.m_offMeshConnections.Get(index)
	conRef := EncodeLinkId(con.salt, index)
	if con.width > 0.0 {
		// Segment-to-segment connection

		this.UnconnectOffMeshConnection(index)
		for i := 0; i < 2; i++ {
			var extent Vector3f
			if i == 0 {
				extent = NormalizeSafe(con.axisX, Vector3f{0, 0, 0})
			} else {
				extent = NormalizeSafe(con.axisX, Vector3f{0, 0, 0}).Mulf(con.width * 0.5).Neg()
			}
			con.endPoints[i].mapped[0] = con.endPoints[i].pos.Sub(extent)
			con.endPoints[i].mapped[1] = con.endPoints[i].pos.Add(extent)
			var polys [kMaxResults]NavMeshPolyRef
			var overlapMinMax [kMaxResults * 2]float32
			count := 0
			this.FindPolygonsOverlappingSegment(con.agentTypeID, con.endPoints[i].mapped[0], con.endPoints[i].mapped[1], connectHeight, polys[:], overlapMinMax[:], &count, kMaxResults)
			for j := 0; j < count; j++ {
				mappedRef := polys[j]
				if mappedRef == 0 {
					continue
				}
				var mappedTile *NavMeshTile
				var mappedPoly *NavMeshPoly
				if NavMeshStatusFailed(this.GetTileAndPolyByRef(mappedRef, &mappedTile, &mappedPoly)) {
					continue
				}

				con.endPoints[i].tileRef = NavMeshTileRef(mappedRef)
				tmin := overlapMinMax[j*2+0] * 255.0
				tmax := overlapMinMax[j*2+1] * 255.0
				if tmin >= tmax {
					continue
				}

				// Link off-mesh connection to target poly.
				idx := this.m_links.Alloc()
				if idx != kNavMeshNullLink {
					link := this.m_links.Get(idx)
					link.ref = mappedRef
					link.edge = uint8(i)
					link.side = 0xff
					link.bmin = uint8(tmin)
					link.bmax = uint8(tmax)
					// Add to linked list.
					Assert(idx != con.firstLink)
					link.next = con.firstLink
					con.firstLink = idx
				}

				// Start end-point is always connect back to off-mesh connection,
				// Destination end-point only if it is bidirectional link.
				if i == 0 || (i == 1 && con.linkDirection&byte(kLinkDirectionTwoWay) != 0) {
					// Link target poly to off-mesh connection.
					idx := this.m_links.Alloc()
					if idx != kNavMeshNullLink {
						link := this.m_links.Get(idx)
						link.ref = conRef
						link.edge = uint8(i)
						link.side = 0xff
						link.bmin = uint8(tmin)
						link.bmax = uint8(tmax)

						// Add to linked list.
						ip := GetPolyIndex(mappedTile, mappedPoly)
						Assert(idx != mappedTile.polyLinks[ip])
						link.next = mappedTile.polyLinks[ip]
						mappedTile.polyLinks[ip] = idx
					}
				}
			}
		}
	} else {
		// Point-to-point connection
		ext := Vector3f{connectRadius, connectHeight, connectRadius}
		for i := 0; i < 2; i++ {
			if con.endPoints[i].tileRef != 0 {
				continue
			}
			searchPos := con.endPoints[i].pos
			var mappedPos Vector3f
			mappedRef := this.FindNearestPoly(con.agentTypeID, searchPos, ext, &mappedPos)
			if mappedRef == 0 {
				continue
			}
			if !TestPointInCylinder(mappedPos, searchPos, connectHeight, connectRadius) {
				continue
			}

			var mappedTile *NavMeshTile
			var mappedPoly *NavMeshPoly
			if NavMeshStatusFailed(this.GetTileAndPolyByRef(mappedRef, &mappedTile, &mappedPoly)) {
				continue
			}

			con.endPoints[i].mapped[0] = mappedPos
			con.endPoints[i].mapped[1] = mappedPos
			con.endPoints[i].tileRef = NavMeshTileRef(mappedRef)

			// Link off-mesh connection to target poly.
			idx := this.m_links.Alloc()
			if idx != kNavMeshNullLink {
				link := this.m_links.Get(idx)
				link.ref = mappedRef
				link.edge = uint8(i)
				link.side = 0xff
				link.bmin = 0
				link.bmax = 0
				// Add to linked list.
				Assert(idx != con.firstLink)
				link.next = con.firstLink
				con.firstLink = idx
			}

			// Start end-point is always connect back to off-mesh connection,
			// Destination end-point only if it is bidirectional link.
			if i == 0 || (i == 1 && con.linkDirection&byte(kLinkDirectionTwoWay) != 0) {
				// Link target poly to off-mesh connection.
				idx := this.m_links.Alloc()
				if idx != kNavMeshNullLink {
					link := this.m_links.Get(idx)
					link.ref = conRef
					link.edge = uint8(i)
					link.side = 0xff
					link.bmin = 0
					link.bmax = 0

					// Add to linked list.
					ip := GetPolyIndex(mappedTile, mappedPoly)
					Assert(idx != mappedTile.polyLinks[ip])
					link.next = mappedTile.polyLinks[ip]
					mappedTile.polyLinks[ip] = idx
				}
			}
		}
	}
}

func (this *NavMesh) AddOffMeshConnection(params *OffMeshConnectionParams, connectRadius float32, connectHeight float32) NavMeshPolyRef {
	index := this.m_offMeshConnections.Alloc()
	if index >= uint32(kPolyRefPolyMask) {
		this.m_offMeshConnections.Release(index)
		return 0
	}
	con := this.m_offMeshConnections.Get(index)

	// Retain salt.
	salt := con.salt
	*con = OffMeshConnection{}
	con.salt = salt

	// Add to active off-mesh connection list.
	con.next = this.m_firstOffMeshConnection
	this.m_firstOffMeshConnection = index

	// fill out connection struct
	con.endPoints[0].pos = params.startPos
	con.endPoints[1].pos = params.endPos
	dir := NormalizeSafe(params.endPos.Sub(params.startPos), Vector3f{0, 0, 0})
	if Magnitude(dir) < 0.00001 {
		dir = Vector3f{0, 0, 1}
	}
	con.axisY = params.up
	con.axisX = Cross(con.axisY, dir)
	con.axisZ = Cross(con.axisX, con.axisY)
	con.width = params.width
	con.costModifier = params.costModifier
	con.linkDirection = params.linkDirection
	con.flags = params.flags
	con.area = params.area
	con.linkType = params.linkType
	con.userID = params.userID
	con.agentTypeID = params.agentTypeID
	con.bounds.Init()
	if con.width > 0.0 {
		extent := NormalizeSafe(con.axisX, Vector3f{0, 0, 0}).Mulf(con.width * 0.5)
		con.bounds.EncapsulateV(con.endPoints[0].pos.Sub(extent))
		con.bounds.EncapsulateV(con.endPoints[0].pos.Add(extent))
		con.bounds.EncapsulateV(con.endPoints[1].pos.Sub(extent))
		con.bounds.EncapsulateV(con.endPoints[1].pos.Add(extent))
	} else {
		con.bounds.EncapsulateV(con.endPoints[0].pos)
		con.bounds.EncapsulateV(con.endPoints[1].pos)
	}

	con.firstLink = kNavMeshNullLink
	dynRef := EncodeLinkId(con.salt, index)

	// Connect
	this.ConnectOffMeshConnection(index, connectRadius, connectHeight)
	this.BumpTimeStamp()
	return dynRef
}

func (this *NavMesh) UnconnectOffMeshConnectionsToTile(ref NavMeshTileRef) {
	tileIndex := DecodePolyIdTile(NavMeshPolyRef(ref))
	for i := this.m_firstOffMeshConnection; i != kNavMeshNullLink; i = this.m_offMeshConnections.Get(i).next {
		con := this.m_offMeshConnections.Get(i)
		ref := EncodeLinkId(con.salt, i)
		for j := 0; j < 2; j++ {
			if con.endPoints[j].tileRef == 0 {
				continue
			}

			// Remove links associated with the tile.
			k := con.firstLink
			for k != kNavMeshNullLink {
				next := this.m_links.Get(k).next
				targetTileIndex := DecodePolyIdTile(NavMeshPolyRef(this.m_links.Get(k).ref))
				if targetTileIndex == tileIndex {
					this.RemoveLinkBetween(ref, this.m_links.Get(k).ref)
					this.RemoveLinkBetween(this.m_links.Get(k).ref, ref)
				}
				k = next
			}
		}
	}
}

func (this *NavMesh) UnconnectOffMeshConnection(index uint32) {
	// Remove links
	con := this.m_offMeshConnections.Get(index)
	ref := EncodeLinkId(con.salt, index)
	i := con.firstLink
	for i != kNavMeshNullLink {
		next := this.m_links.Get(i).next
		this.RemoveLinkBetween(this.m_links.Get(i).ref, ref)
		this.m_links.Release(i)
		i = next
	}
	con.firstLink = kNavMeshNullLink
}

func (this *NavMesh) RemoveOffMeshConnection(ref NavMeshPolyRef) NavMeshStatus {
	if DecodePolyIdType(ref) != kPolyTypeOffMeshConnection {
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	index := DecodePolyIdPoly(ref)
	if index >= this.m_offMeshConnections.Capacity() {
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	salt := DecodePolyIdSalt(ref)
	if salt != this.m_offMeshConnections.Get(index).salt {
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	// Remove links
	this.UnconnectOffMeshConnection(index)

	// Find previous offmesh link to be able to remove from the list.
	i := this.m_firstOffMeshConnection
	prev := kNavMeshNullLink
	for i != kNavMeshNullLink {
		if i == index {
			break
		}
		prev = i
		i = this.m_offMeshConnections.Get(i).next
	}
	// Remove from linked list
	next := this.m_offMeshConnections.Get(index).next
	if prev == kNavMeshNullLink {
		this.m_firstOffMeshConnection = next
	} else {
		this.m_offMeshConnections.Get(prev).next = next
	}

	// Bump salt to distinguish deleted connections.
	this.m_offMeshConnections.Get(index).salt++
	if this.m_offMeshConnections.Get(index).salt == 0 {

		this.m_offMeshConnections.Get(index).salt = 1
	}

	this.m_offMeshConnections.Release(index)
	this.BumpTimeStamp()
	return kNavMeshSuccess
}

func (this *NavMesh) GetFirstLink(ref NavMeshPolyRef) *NavMeshLink {
	if DecodePolyIdType(ref) == kPolyTypeOffMeshConnection {
		con := this.GetOffMeshConnection(ref)
		if con == nil {
			return nil
		}
		if con.firstLink == kNavMeshNullLink {
			return nil
		}
		return this.m_links.Get(con.firstLink)
	} else {
		var tile *NavMeshTile
		var poly *NavMeshPoly
		if NavMeshStatusFailed(this.GetTileAndPolyByRef(ref, &tile, &poly)) {
			return nil
		}

		ip := GetPolyIndex(tile, poly)
		firstLink := tile.polyLinks[ip]
		if firstLink == kNavMeshNullLink {
			return nil
		}
		return this.m_links.Get(firstLink)
	}
}

func (this *NavMesh) GetFirstOffMeshConnection() *OffMeshConnection {
	if this.m_firstOffMeshConnection == kNavMeshNullLink {
		return nil
	}
	return this.m_offMeshConnections.Get(this.m_firstOffMeshConnection)
}

func (this *NavMesh) GetNextOffMeshConnection(data *OffMeshConnection) *OffMeshConnection {
	if data == nil {
		return nil
	}
	if data.next == kNavMeshNullLink {
		return nil
	}
	Assert(data.next < this.m_offMeshConnections.Capacity())
	return this.m_offMeshConnections.Get(data.next)
}

func (this *NavMesh) GetPolyFlags(ref NavMeshPolyRef) uint32 {
	if DecodePolyIdType(ref) == kPolyTypeOffMeshConnection {
		con := this.GetOffMeshConnection(ref)
		if con == nil {
			return 0
		}
		return con.flags
	} else {
		var tile *NavMeshTile
		var poly *NavMeshPoly
		if NavMeshStatusFailed(this.GetTileAndPolyByRef(ref, &tile, &poly)) {
			return 0
		}
		return poly.flags
	}
}

func (this *NavMesh) GetPolyArea(ref NavMeshPolyRef) byte {
	if DecodePolyIdType(ref) == kPolyTypeOffMeshConnection {
		con := this.GetOffMeshConnection(ref)
		if con == nil {
			return 0
		}
		return con.area
	} else {
		var tile *NavMeshTile
		var poly *NavMeshPoly
		if NavMeshStatusFailed(this.GetTileAndPolyByRef(ref, &tile, &poly)) {
			return 0
		}
		return poly.area
	}
}

func (this *NavMesh) GetPolyFlagsAndArea(ref NavMeshPolyRef, flags *uint32, area *byte) {
	if flags != nil {
		*flags = 0
	}
	if area != nil {
		*area = 0
	}

	if DecodePolyIdType(ref) == kPolyTypeOffMeshConnection {
		con := this.GetOffMeshConnection(ref)
		if con == nil {
			return
		}
		if flags != nil {
			*flags = con.flags
		}
		if area != nil {
			*area = con.area
		}
	} else {
		var tile *NavMeshTile
		var poly *NavMeshPoly
		if NavMeshStatusFailed(this.GetTileAndPolyByRef(ref, &tile, &poly)) {

			return
		}
		if flags != nil {
			*flags = poly.flags
		}
		if area != nil {
			*area = poly.area
		}
	}
}

// Returns vertices and neighbours of polygon pointed to by the polygon reference.
// Note: this function returns 0 if ref is invalid
// Params:
//  ref - (in) reference to a polygon.
//  verts - (out, optional) pointer to polygon vertices, must have a capacity of at least kNavMeshVertsPerPoly.
//  neighbours - (out, optional) pointer to the polygon neighbours, must have a capacity of at least kNavMeshVertsPerPoly*maxNeisPerEdge.
//  maxNeisPerEdge - (int) maximum number of neighbours stored in 'neighbours' per edge.
func (this *NavMesh) GetPolyGeometry(ref NavMeshPolyRef, verts []Vector3f, neighbours []NavMeshPolyRef, maxNeisPerEdge int32) int32 {
	if DecodePolyIdType(ref) == kPolyTypeOffMeshConnection {
		// TODO: should we return off-mesh link geometry?
		return 0
	} else {
		var tile *NavMeshTile
		var poly *NavMeshPoly
		if NavMeshStatusFailed(this.GetTileAndPolyByRef(ref, &tile, &poly)) {

			return 0
		}
		nverts := poly.vertCount

		// Copy vertices
		if verts != nil {
			for i := uint8(0); i < nverts; i++ {
				verts[i] = tile.verts[poly.verts[i]]
			}
		}

		// Copy neighbours
		if neighbours != nil {
			for i := int32(0); i < int32(nverts)*maxNeisPerEdge; i++ {
				neighbours[i] = 0
			}

			ip := GetPolyIndex(tile, poly)
			firstLink := tile.polyLinks[ip]
			for link := this.GetLink(firstLink); link != nil; link = this.GetNextLink(link) {
				// Only accept polygon connections.
				if DecodePolyIdType(link.ref) == kPolyTypeOffMeshConnection {
					continue
				}
				index := link.edge
				if index < nverts {
					// find empty slot
					for j := int32(0); j < maxNeisPerEdge; j++ {
						if neighbours[int32(index)*maxNeisPerEdge+j] == 0 {
							neighbours[int32(index)*maxNeisPerEdge+j] = link.ref
							break
						}
					}
				}
			}
		}
		return int32(nverts)
	}
}

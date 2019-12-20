package unityai

import (
	"math"
)

const (
	H_SCALE float32 = 0.999 // Search heuristic scale.
)

func GetCostModifier(navmesh *NavMesh, filter *QueryFilter, ref NavMeshPolyRef) float32 {
	if DecodePolyIdType(ref) == kPolyTypeOffMeshConnection {
		con := navmesh.GetOffMeshConnection(ref)
		cost := con.costModifier
		if cost != -1.0 {
			return cost
		}
		return filter.GetAreaCost(int32(con.area))
	} else {
		var tile *NavMeshTile
		var poly *NavMeshPoly
		navmesh.GetTileAndPolyByRef(ref, &tile, &poly)
		return filter.GetAreaCost(int32(poly.area))
	}
	Assert(false) // bad
	return float32(math.MaxFloat32)
}

func SqrDistancePointPolyEdge(pt Vector3f, verts []Vector3f, nverts int32, ed, et []float32) bool {
	// TODO: Replace pnpoly with triArea2D tests?
	c := false
	for i, j := int32(0), nverts-1; i < nverts; j, i = i, i+1 {
		vi := verts[i]
		vj := verts[j]
		if ((vi.z > pt.z) != (vj.z > pt.z)) && (pt.x < (vj.x-vi.x)*(pt.z-vi.z)/(vj.z-vi.z)+vi.x) {
			c = !c
		}
		ed[j] = SqrDistancePointSegment2D(&et[j], pt, vj, vi)
	}
	return c
}

func ProjectPoly(rmin, rmax *float32, axis Vector3f, poly []Vector3f, npoly int32) {
	var min, max float32
	max = DotVector3f(axis, poly[0])
	min = max
	for i := int32(1); i < npoly; i++ {
		d := DotVector3f(axis, poly[i])
		min = FloatMin(min, d)
		max = FloatMax(max, d)
	}
	*rmin = min
	*rmax = max
}

func OverlapRange(amin, amax, bmin, bmax, eps float32) bool {
	if (amin+eps) > bmax || (amax-eps) < bmin {
		return false
	} else {
		return true
	}
}

func OverlapPolyPoly2D(polya []Vector3f, npolya int32, polyb []Vector3f, npolyb int32) bool {
	eps := float32(1e-4)
	for i, j := int32(0), npolya-1; i < npolya; j, i = i, i+1 {
		va := polya[j]
		vb := polya[i]
		n := Vector3f{vb.z - va.z, 0, va.x - vb.x}
		var amin, amax, bmin, bmax float32
		ProjectPoly(&amin, &amax, n, polya, npolya)
		ProjectPoly(&bmin, &bmax, n, polyb, npolyb)
		if !OverlapRange(amin, amax, bmin, bmax, eps) {
			// Found separating axis
			return false
		}
	}
	for i, j := int32(0), npolyb-1; i < npolyb; j, i = i, i+1 {
		va := polyb[j]
		vb := polyb[i]
		n := Vector3f{vb.z - va.z, 0, -va.x - vb.x}
		var amin, amax, bmin, bmax float32
		ProjectPoly(&amin, &amax, n, polya, npolya)
		ProjectPoly(&bmin, &bmax, n, polyb, npolyb)
		if !OverlapRange(amin, amax, bmin, bmax, eps) {
			// Found separating axis
			return false
		}
	}
	return true
}

func PointInPolygon2D(pt Vector3f, verts []Vector3f, nverts int32) bool {
	// Assume clockwise and convex
	prev := verts[nverts-1]
	for i := int32(0); i < nverts; i++ {
		curr := verts[i]
		dlx := curr.x - prev.x
		dlz := curr.z - prev.z
		dpx := pt.x - prev.x
		dpz := pt.z - prev.z
		if dpx*dlz < dpz*dlx {
			return false
		}
		prev = curr
	}
	return true
}

func NewNavMeshQuery(navmesh *NavMesh, maxNodes int32) *NavMeshQuery {
	query := new(NavMeshQuery)
	query.InitPools(navmesh, maxNodes)
	return query
}

// Initializes the nav mesh query.
// Params:
//  nav - (in) pointer to navigation mesh data.
//  maxNodes - (in) Maximum number of search nodes to use (max 65536).
func (this *NavMeshQuery) InitPools(nav *NavMesh, maxNodes int32) NavMeshStatus {
	Assert(this.m_TinyNodePool == nil)
	Assert(this.m_NodePool == nil)
	Assert(this.m_OpenList == nil)
	this.m_NavMesh = nav
	this.m_TinyNodePool = NewNavMeshNodePool(64, 32)
	if this.m_TinyNodePool == nil {
		return kNavMeshFailure | kNavMeshOutOfMemory
	}
	if maxNodes == 0 {
		return kNavMeshSuccess
	}
	this.m_NodePool = NewNavMeshNodePool(maxNodes, int32(NextPowerOfTwo(uint32(maxNodes)/4)))
	if this.m_NodePool == nil {
		return kNavMeshFailure | kNavMeshOutOfMemory
	}
	this.m_OpenList = NewNavMeshNodeQueue(maxNodes)
	if this.m_OpenList == nil {
		return kNavMeshFailure | kNavMeshOutOfMemory
	}
	return kNavMeshSuccess
}

// Returns closest point on navigation polygon.
// Uses detail polygons to find the closest point to the navigation polygon surface.
// Params:
//  ref - (in) ref to the polygon.
//  pos[3] - (in) the point to check.
//  closest[3] - (out) closest point.
// Returns: true if closest point found.
func (this *NavMeshQuery) ClosestPointOnPoly(ref NavMeshPolyRef, pos Vector3f, closest *Vector3f) NavMeshStatus {
	Assert(this.m_NavMesh != nil)
	if DecodePolyIdType(ref) == kPolyTypeOffMeshConnection {
		con := this.m_NavMesh.GetOffMeshConnection(ref)
		if con == nil {
			return kNavMeshFailure | kNavMeshInvalidParam
		}
		if con.endPoints[0].tileRef == 0 || con.endPoints[1].tileRef == 0 {
			return kNavMeshFailure
		}

		// TODO: Handle segment
		d0 := SqrDistance(con.endPoints[0].mapped[0], pos)
		d1 := SqrDistance(con.endPoints[1].mapped[0], pos)
		if d0 < d1 {
			*closest = con.endPoints[0].mapped[0]
		} else {
			*closest = con.endPoints[1].mapped[0]
		}
		return kNavMeshSuccess
	} else {
		var tile *NavMeshTile
		var poly *NavMeshPoly
		if NavMeshStatusFailed(this.m_NavMesh.GetTileAndPolyByRef(ref, &tile, &poly)) {
			return kNavMeshFailure | kNavMeshInvalidParam
		}
		if tile == nil {
			return kNavMeshFailure | kNavMeshInvalidParam
		}
		localPos := WorldToTile(tile, pos)
		var localClosest Vector3f
		this.ClosestPointOnPolyInTileLocal(tile, poly, ref, localPos, &localClosest)
		*closest = TileToWorld(tile, localClosest)
	}

	return kNavMeshSuccess
}

func ProjectPointToPoly2DLocal(pos Vector3f, poly *NavMeshPoly, tile *NavMeshTile, projectedPos *Vector3f) bool {
	Assert(poly != nil)
	Assert(tile != nil)
	var verts [kNavMeshVertsPerPoly]Vector3f
	var edged [kNavMeshVertsPerPoly]float32
	var edget [kNavMeshVertsPerPoly]float32
	nv := poly.vertCount
	for i := uint8(0); i < nv; i++ {
		verts[i] = tile.verts[poly.verts[i]]
	}

	inside := SqrDistancePointPolyEdge(pos, verts[:], int32(nv), edged[:], edget[:])
	if inside {
		// Point is inside the polygon, return the point.
		*projectedPos = pos
	} else {
		// Point is outside the polygon, clamp to nearest edge.
		dmin := float32(math.MaxFloat32)
		imin := int32(-1)
		for i := uint8(0); i < nv; i++ {
			if edged[i] < dmin {
				dmin = edged[i]
				imin = int32(i)
			}
		}
		Assert(imin < int32(nv))
		var iminnext int32
		if imin+1 == int32(nv) {
			iminnext = 0
		} else {
			iminnext = imin + 1
		}
		va := verts[imin]
		vb := verts[iminnext]
		*projectedPos = LerpVector3f(va, vb, edget[imin])
		//points that are on (or very close to) an edge are also considered to be inside the polygon
		inside = dmin < FLT_EPSILON
	}
	return inside
}

func (this *NavMeshQuery) GetUpAxis(ref NavMeshPolyRef, up *Vector3f) NavMeshStatus {
	if DecodePolyIdType(ref) == kPolyTypeOffMeshConnection {
		con := this.m_NavMesh.GetOffMeshConnection(ref)
		if con != nil {
			*up = con.axisY
			return kNavMeshSuccess
		}
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	yAxis := Vector3f{0.0, 1.0, 0.0}
	tile := this.m_NavMesh.GetTileByRef(NavMeshTileRef(ref))
	if tile != nil {
		*up = RotateVectorByQuat(tile.rotation, yAxis)
		return kNavMeshSuccess
	}

	return kNavMeshFailure | kNavMeshInvalidParam
}

// Local space input/output
// Calculate the closest point on a polygon
// Returns true unless the 2D projected point is outside the polygon
func (this *NavMeshQuery) ClosestPointOnPolyInTileLocal(tile *NavMeshTile, poly *NavMeshPoly, ref NavMeshPolyRef,
	pos Vector3f, closest *Vector3f) bool {
	inside := ProjectPointToPoly2DLocal(pos, poly, tile, closest)
	// Find height at the location.
	this.GetPolyHeightLocal(ref, *closest, &closest.y)
	return inside
}

// Returns closest point on navigation polygon boundary.
// Uses the navigation polygon boundary to snap the point to poly boundary
// if it is outside the polygon. Much faster than ClosestPointToPoly. Does not affect height.
// Params:
//  ref - (in) ref to the polygon.
//  pos[3] - (in) the point to check.
//  closest[3] - (out) closest point.
// Returns: true if closest point found.
func (this *NavMeshQuery) ClosestPointOnPolyBoundary(ref NavMeshPolyRef, pos Vector3f, closest *Vector3f) NavMeshStatus {
	Assert(this.m_NavMesh != nil)
	if DecodePolyIdType(ref) == kPolyTypeOffMeshConnection {
		con := this.m_NavMesh.GetOffMeshConnection(ref)
		if con == nil {
			return kNavMeshFailure | kNavMeshInvalidParam
		}

		if con.endPoints[0].tileRef == 0 || con.endPoints[1].tileRef == 0 {
			return kNavMeshFailure
		}

		// TODO: Handle segment
		d0 := SqrDistance(con.endPoints[0].mapped[0], pos)
		d1 := SqrDistance(con.endPoints[1].mapped[0], pos)
		if d0 < d1 {

			*closest = con.endPoints[0].mapped[0]
		} else {
			*closest = con.endPoints[1].mapped[0]
		}

		return kNavMeshSuccess
	}

	var tile *NavMeshTile
	var poly *NavMeshPoly
	if NavMeshStatusFailed(this.m_NavMesh.GetTileAndPolyByRef(ref, &tile, &poly)) {
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	localPos := WorldToTile(tile, pos)
	var localProjPos Vector3f
	ProjectPointToPoly2DLocal(localPos, poly, tile, &localProjPos)
	*closest = TileToWorld(tile, localProjPos)
	return kNavMeshSuccess
}

// Project point height to detail triangle and accept immediately if the projected point is inside triangle
func ProjectToPolyDetail(tile *NavMeshTile, poly *NavMeshPoly, pos Vector3f, height *float32) bool {
	ip := GetPolyIndex(tile, poly)
	pd := &tile.detailMeshes[ip]
	for j := uint32(0); j < uint32(pd.triCount); j++ {
		t := tile.detailTris[(pd.triBase+j)*4:]
		var v [3]Vector3f
		for k := 0; k < 3; k++ {
			if uint8(t[k]) < poly.vertCount {
				v[k] = tile.verts[poly.verts[t[k]]]

			} else {
				v[k] = tile.detailVerts[pd.vertBase+(uint32(t[k])-uint32(poly.vertCount))]
			}
		}
		var h float32
		if ClosestHeightPointTriangle(&h, pos, v[0], v[1], v[2]) {
			*height = h
			return true
		}
	}
	return false
}

// In the plane of projection find the height of the closest point on the edge of the detail triangles
func ProjectToPolyDetailEdge(tile *NavMeshTile, poly *NavMeshPoly, pos Vector3f) float32 {
	ip := GetPolyIndex(tile, poly)
	pd := &tile.detailMeshes[ip]
	dmin := float32(math.MaxFloat32)
	h := float32(math.MaxFloat32)
	for j := uint32(0); j < uint32(pd.triCount); j++ {
		t := tile.detailTris[(pd.triBase+j)*4:]
		var v [3]Vector3f
		for k := 0; k < 3; k++ {
			if uint8(t[k]) < poly.vertCount {
				v[k] = tile.verts[poly.verts[t[k]]]

			} else {
				v[k] = tile.detailVerts[pd.vertBase+(uint32(t[k])-uint32(poly.vertCount))]
			}
		}

		for kp, k := 2, 0; k < 3; kp, k = k, k+1 {
			var tt float32
			d := SqrDistancePointSegment2D(&tt, pos, v[kp], v[k])
			if d < dmin {
				dmin = d
				h = LerpFloat32(v[kp].y, v[k].y, tt)
			}
		}
	}
	return h
}

// Returns height of the polygon at specified location.
// Params:
//  ref - (in) ref to the polygon.
//  pos[3] - (in) the point where to locate the height.
//  height - (out) height at the location.
// Returns: true if over polygon.
func (this *NavMeshQuery) GetPolyHeightLocal(ref NavMeshPolyRef, pos Vector3f, height *float32) NavMeshStatus {
	Assert(this.m_NavMesh != nil)
	Assert(height != nil)
	if DecodePolyIdType(ref) == kPolyTypeOffMeshConnection {
		con := this.m_NavMesh.GetOffMeshConnection(ref)
		if con != nil {
			// TODO: handle segment
			v0 := con.endPoints[0].mapped[0]
			v1 := con.endPoints[1].mapped[0]
			d0 := Distance(pos, v0)
			d1 := Distance(pos, v1)
			u := d0 / (d0 + d1)
			*height = LerpFloat32(v0.y, v1.y, u)
			return kNavMeshSuccess
		}
	} else {
		var tile *NavMeshTile
		var poly *NavMeshPoly
		if NavMeshStatusSucceed(this.m_NavMesh.GetTileAndPolyByRef(ref, &tile, &poly)) {
			// Most calls should terminate here
			if ProjectToPolyDetail(tile, poly, pos, height) {
				return kNavMeshSuccess
			}

			// Only rarely should this be executed - so we allow a second lookup of triangles
			*height = ProjectToPolyDetailEdge(tile, poly, pos)
			return kNavMeshSuccess
		}
	}

	return kNavMeshFailure | kNavMeshInvalidParam
}

// Project point to polygon along the local up-axis
func (this *NavMeshQuery) ProjectToPoly(projPos *Vector3f, ref NavMeshPolyRef, pos Vector3f) NavMeshStatus {
	Assert(this.m_NavMesh != nil)
	Assert(projPos != nil)
	*projPos = pos
	if DecodePolyIdType(ref) == kPolyTypeOffMeshConnection {
		con := this.m_NavMesh.GetOffMeshConnection(ref)
		if con != nil {
			v0 := LerpVector3f(con.endPoints[0].mapped[0], con.endPoints[0].mapped[1], 0.5)
			v1 := LerpVector3f(con.endPoints[1].mapped[0], con.endPoints[1].mapped[1], 0.5)
			d0 := Distance(pos, v0)
			d1 := Distance(pos, v1)
			u := d0 / (d0 + d1)
			*projPos = LerpVector3f(v0, v1, u)
			return kNavMeshSuccess
		}
	} else {
		var tile *NavMeshTile
		var poly *NavMeshPoly
		if NavMeshStatusSucceed(this.m_NavMesh.GetTileAndPolyByRef(ref, &tile, &poly)) {
			// Most calls should terminate here
			var localHeight float32
			localPos := WorldToTile(tile, pos)
			if ProjectToPolyDetail(tile, poly, localPos, &localHeight) {
				worldPos := TileToWorld(tile, Vector3f{localPos.x, localHeight, localPos.z})
				*projPos = worldPos
				return kNavMeshSuccess
			}

			// Only rarely should this be executed - so we allow a second lookup of triangles
			localHeight = ProjectToPolyDetailEdge(tile, poly, localPos)
			worldPos := TileToWorld(tile, Vector3f{localPos.x, localHeight, localPos.z})
			*projPos = worldPos
			return kNavMeshSuccess
		}
	}

	return kNavMeshFailure | kNavMeshInvalidParam
}

type NearestQuery1 struct {
	m_NavMeshMeshQuery *NavMeshQuery
	m_Filter           *QueryFilter
	m_Center           Vector3f

	m_DistanceSqr [2]float32
	m_PolyRef     [2]NavMeshPolyRef
	m_LocalPoint  [2]Vector3f
}

func NewNearestQuery1(navMeshQuery *NavMeshQuery, filter *QueryFilter, center Vector3f) *NearestQuery1 {
	query := new(NearestQuery1)
	query.m_NavMeshMeshQuery = navMeshQuery
	query.m_Filter = filter
	query.m_Center = center
	query.m_DistanceSqr[0] = math.MaxFloat32
	query.m_DistanceSqr[1] = math.MaxFloat32
	query.m_PolyRef[0] = 0
	query.m_PolyRef[1] = 0
	// It's strictly not necessary to initialise the positions.
	// The user should be checking the corresponding polygon reference before reading the position
	query.m_LocalPoint[0].SetZero()
	query.m_LocalPoint[1].SetZero()
	return query
}

func (this *NearestQuery1) ProcessPolygons(tile *NavMeshTile, polyRefs []NavMeshPolyRef, polys []*NavMeshPoly, itemCount int) {
	localPosition := WorldToTile(tile, this.m_Center)

	for item := 0; item < itemCount; item++ {
		ref := polyRefs[item]
		poly := polys[item]
		if !this.m_Filter.PassFilter(poly.flags) {
			continue
		}

		var closestPtPoly Vector3f
		var inside int32
		if this.m_NavMeshMeshQuery.ClosestPointOnPolyInTileLocal(tile, poly, ref, localPosition, &closestPtPoly) {
			inside = 1
		} else {
			inside = 0
		}

		// Keep track of the closest position. Calculating the squared vertical and 3D distances.
		// The 3D distance is used for comparing points projected outside the polygon - otherwise using the vertical distance.
		var d [2]float32
		d[0] = SqrMagnitude(localPosition.Sub(closestPtPoly))
		d[1] = Sqr(localPosition.y - closestPtPoly.y)

		if d[inside] < this.m_DistanceSqr[inside] {
			this.m_LocalPoint[inside] = closestPtPoly
			this.m_DistanceSqr[inside] = d[inside]
			this.m_PolyRef[inside] = ref
		}
	}
}

// Finds the nearest navigation polygon around the center location.
// Params:
//  center[3] - (in) The center of the search box.
//  extents[3] - (in) The extents of the search box.
//  filter - (in) path polygon filter.
//  nearestRef - (out) Reference to the nearest polygon.
//  nearestPt[3] - (out, opt) The nearest point on found polygon, null if not needed.
func (this *NavMeshQuery) FindNearestPoly(center Vector3f, extents Vector3f, filter *QueryFilter,
	nearestRef *NavMeshPolyRef, nearestPt *Vector3f) {
	Assert(this.m_NavMesh != nil)
	Assert(nearestRef != nil)
	Assert(nearestPt != nil)

	// Get the nearest polygons and projected points
	nearest := NewNearestQuery1(this, filter, center)
	this.m_NavMesh.QueryPolygons(filter.GetTypeID(), center, extents, nearest)

	// Check if any of the closest points found (outside/inside) is within the query volume
	// Prefer inside points (inside == 1)

	bounds := AABB{center, extents}
	var point Vector3f
	var ref NavMeshPolyRef = 0
	for inside := 0; inside < 2; inside++ {
		tile := this.m_NavMesh.GetTileByRef(NavMeshTileRef(nearest.m_PolyRef[inside]))
		if tile != nil {
			worldPoint := TileToWorld(tile, nearest.m_LocalPoint[inside])
			if bounds.IsInside(worldPoint) {
				ref = nearest.m_PolyRef[inside]
				point = worldPoint
			}
		}
	}

	if nearestRef != nil {
		*nearestRef = ref
	}

	if nearestPt != nil && ref != 0 {
		*nearestPt = point
	}
}

type PolygonQuery struct {
	m_NavMeshMeshQuery *NavMeshQuery
	m_Filter           *QueryFilter
	m_Results          []NavMeshPolyRef
	m_MaxResults       int32
	m_ResultCount      int32
}

func NewPolygonQuery(query *NavMeshQuery, filter *QueryFilter, polys []NavMeshPolyRef, maxPolys int32) *PolygonQuery {
	pQuery := new(PolygonQuery)
	pQuery.m_NavMeshMeshQuery = query
	pQuery.m_Filter = filter
	pQuery.m_Results = polys
	pQuery.m_MaxResults = maxPolys
	pQuery.m_ResultCount = 0
	return pQuery
}

func (this *PolygonQuery) ProcessPolygons(title *NavMeshTile, polyRefs []NavMeshPolyRef, polys []*NavMeshPoly, itemCount int) {
	// Find nearest polygon amongst the nearby polygons.
	for i := 0; i < itemCount; i++ {
		if !this.m_Filter.PassFilter(polys[i].flags) {
			continue
		}

		if this.m_ResultCount < this.m_MaxResults {

			this.m_Results[this.m_ResultCount] = polyRefs[i]
			this.m_ResultCount++
		}
	}
}

// Returns polygons which overlap the query box.
// Params:
//  center[3] - (in) the center of the search box.
//  extents[3] - (in) the extents of the search box.
//  filter - (in) path polygon filter.
//  polys - (out) array holding the search result.
//  polyCount - (out) Number of polygons in search result array.
//  maxPolys - (in) The max number of polygons the polys array can hold.
func (this *NavMeshQuery) QueryPolygons(center Vector3f, extents Vector3f, filter *QueryFilter,
	polys []NavMeshPolyRef, polyCount *int32, maxPolys int32) {

	polygons := NewPolygonQuery(this, filter, polys, maxPolys)
	this.m_NavMesh.QueryPolygons(filter.GetTypeID(), center, extents, polygons)

	if polyCount != nil {
		*polyCount = polygons.m_ResultCount
	}
}

func (this *NavMeshQuery) InitSlicedFindPath(startRef NavMeshPolyRef, endRef NavMeshPolyRef, startPos Vector3f,
	endPos Vector3f, agentType int32, areaMask int32, costs [kAreaCount]float32) NavMeshStatus {
	if this.m_NavMesh == nil {
		return kNavMeshFailure
	}
	filter := NewQueryFilter()
	filter.Set(agentType, uint32(areaMask), costs[:])
	return this.InitSlicedFindPath2(startRef, endRef, startPos, endPos, filter)
}

// Initializes sliced path find query.
// Note 1: calling any other NavMeshQuery method before calling FindPathEnd()
// may results in corrupted data!
// Note 2: The pointer to filter is store, and used in subsequent
// calls to UpdateSlicedFindPath().
// Params:
//  startRef - (in) ref to path start polygon.
//  endRef - (in) ref to path end polygon.
//  startPos[3] - (in) Path start location.
//  endPos[3] - (in) Path end location.
//  filter - (in) path polygon filter.
func (this *NavMeshQuery) InitSlicedFindPath2(startRef NavMeshPolyRef, endRef NavMeshPolyRef, startPos Vector3f,
	endPos Vector3f, filter *QueryFilter) NavMeshStatus {
	Assert(this.m_NavMesh != nil)
	Assert(this.m_NodePool != nil)
	Assert(this.m_OpenList != nil)

	// Init path state.
	this.m_QueryData = NavMeshQueryData{}
	this.m_QueryData.status = kNavMeshFailure
	this.m_QueryData.startRef = startRef
	this.m_QueryData.endRef = endRef
	this.m_QueryData.startPos = startPos
	this.m_QueryData.endPos = endPos
	this.m_QueryData.filter = filter
	if startRef == 0 || endRef == 0 {
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	// Validate input
	if !this.m_NavMesh.IsValidPolyRef(startRef) || !this.m_NavMesh.IsValidPolyRef(endRef) {
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	// Validate first poly
	if !this.m_QueryData.filter.PassFilter(this.m_NavMesh.GetPolyFlags(startRef)) {
		return kNavMeshFailure
	}
	if startRef == endRef {
		this.m_QueryData.status = kNavMeshSuccess
		return kNavMeshSuccess
	}

	this.m_NodePool.Clear()
	this.m_OpenList.Clear()
	var endPosProjStart Vector3f
	this.ClosestPointOnPoly(startRef, endPos, &endPosProjStart)
	startHeuristic := Distance(endPosProjStart, endPos) * H_SCALE
	startNode := this.m_NodePool.GetNode(startRef)
	startNode.pos = startPos
	startNode.pidx = 0
	startNode.cost = 0
	startNode.total = startHeuristic
	startNode.id = startRef
	startNode.flags = kOpen
	this.m_OpenList.Push(startNode)
	this.m_QueryData.status = kNavMeshInProgress
	this.m_QueryData.lastBestNode = startNode
	this.m_QueryData.lastBestNodeCost = startHeuristic
	return this.m_QueryData.status
}

// Updates sliced path find query.
// Params:
//  maxIter - (in) Max number of iterations to update.
//  doneIters - (out,opt) Number of iterations done during the update.
// Returns: Path query state.
func (this *NavMeshQuery) UpdateSlicedFindPath(maxIter int32, doneIters *int32) NavMeshStatus {
	Assert(this.m_NavMesh != nil)
	Assert(this.m_NodePool != nil)
	Assert(this.m_OpenList != nil)
	if !NavMeshStatusInProgress(this.m_QueryData.status) {
		if doneIters != nil {
			*doneIters = 0
		}
		return this.m_QueryData.status
	}

	// Make sure the request is still valid.
	if !this.m_NavMesh.IsValidPolyRef(this.m_QueryData.startRef) || !this.m_NavMesh.IsValidPolyRef(this.m_QueryData.endRef) {
		this.m_QueryData.status = kNavMeshFailure
		if doneIters != nil {
			*doneIters = 0
		}
		return kNavMeshFailure
	}

	iter := int32(0)
	for iter < maxIter && !this.m_OpenList.empty() {
		iter++

		// Remove node from open list and put it in closed list.
		bestNode := this.m_OpenList.Pop()
		bestNode.flags &= ^kOpen
		bestNode.flags |= kClosed

		// Reached the goal, stop searching.
		if bestNode.id == this.m_QueryData.endRef {
			this.m_QueryData.lastBestNode = bestNode
			details := this.m_QueryData.status & kNavMeshStatusDetailMask
			this.m_QueryData.status = kNavMeshSuccess | details
			if doneIters != nil {
				*doneIters = iter
			}
			return this.m_QueryData.status
		}

		// Get and verify best poly ref.
		bestRef := bestNode.id
		if !this.m_NavMesh.IsValidPolyRef(bestRef) {
			// Things changed during the sliced query, fail.
			this.m_QueryData.status = kNavMeshFailure
			if doneIters != nil {
				*doneIters = iter
			}
			return kNavMeshFailure
		}

		// Get and verify parent poly ref.
		parentRef := NavMeshPolyRef(0)
		if bestNode.pidx != 0 {
			// Things changed during the sliced query, fail.
			parentRef = this.m_NodePool.GetNodeAtIdx(bestNode.pidx).id
			if !this.m_NavMesh.IsValidPolyRef(parentRef) {
				this.m_QueryData.status = kNavMeshFailure
				if doneIters != nil {
					*doneIters = iter
				}
				return kNavMeshFailure
			}
		}

		for link := this.m_NavMesh.GetFirstLink(bestRef); link != nil; link = this.m_NavMesh.GetNextLink(link) {
			this.UpdateNeighbourLink(link, parentRef, bestRef, bestNode)
		}
	}

	// Exhausted all nodes, but could not find path.
	if this.m_OpenList.empty() {
		details := this.m_QueryData.status & kNavMeshStatusDetailMask
		this.m_QueryData.status = kNavMeshSuccess | details
	}

	if doneIters != nil {
		*doneIters = iter
	}
	return this.m_QueryData.status
}

func (this *NavMeshQuery) UpdateNeighbourLink(link *NavMeshLink, parentRef NavMeshPolyRef, bestRef NavMeshPolyRef, bestNode *NavMeshNode) {
	Assert(this.m_OpenList != nil)
	Assert(this.m_NodePool != nil)
	neighbourRef := link.ref

	// Skip invalid ids and do not expand back to where we came from.
	if neighbourRef == 0 || neighbourRef == parentRef {
		return
	}

	if !this.m_QueryData.filter.PassFilter(this.m_NavMesh.GetPolyFlags(neighbourRef)) {
		return
	}

	neighbourNode := this.m_NodePool.GetNode(neighbourRef)
	if neighbourNode == nil {
		this.m_QueryData.status |= kNavMeshOutOfNodes
		return
	}

	// If the node is visited the first time, calculate node position.
	if neighbourNode.flags == kNew {
		var left, right Vector3f
		if NavMeshStatusFailed(this.GetPortalPoints(bestRef, neighbourRef, &left, &right)) {
			return
		}

		dl := right.Sub(left)
		ddl := DotVector3f(dl, dl)
		if ddl > 0.0001 {
			dp := bestNode.pos.Sub(left)
			pdot := DotVector3f(dl, dp) / ddl
			pdot = FloatClamp(pdot, 0.05, 0.95) // Avoid degenerating to vertex

			dp = left.Add(dl.Mulf(pdot))
			neighbourNode.pos = dp
		} else {
			neighbourNode.pos = left
		}
	}

	// Calculate cost and heuristic.
	heuristic := float32(0.0)
	cost := bestNode.cost + Distance(bestNode.pos, neighbourNode.pos)*GetCostModifier(this.m_NavMesh, this.m_QueryData.filter, bestRef)

	// Special case for last node.
	if neighbourRef == this.m_QueryData.endRef {
		cost += Distance(neighbourNode.pos, this.m_QueryData.endPos) * GetCostModifier(this.m_NavMesh, this.m_QueryData.filter, neighbourRef)
	} else {
		var endPosProjNeighbour Vector3f
		if NavMeshStatusFailed(this.ClosestPointOnPolyBoundary(neighbourRef, this.m_QueryData.endPos, &endPosProjNeighbour)) {
			return
		}
		heuristic = Distance(endPosProjNeighbour, this.m_QueryData.endPos) * H_SCALE
	}

	// Update nearest node to target so far.
	if heuristic < this.m_QueryData.lastBestNodeCost {
		this.m_QueryData.lastBestNodeCost = heuristic
		this.m_QueryData.lastBestNode = neighbourNode
	}

	total := cost + heuristic

	// The node is already in open list and the new result is worse, skip.
	if (neighbourNode.flags&kOpen) != 0 && total >= neighbourNode.total {
		return
	}
	// The node is already visited and process, and the new result is worse, skip.
	if (neighbourNode.flags&kClosed) != 0 && total >= neighbourNode.total {
		return
	}

	// Add or update the node.
	neighbourNode.pidx = this.m_NodePool.GetNodeIdx(bestNode)
	neighbourNode.id = neighbourRef
	neighbourNode.flags &= ^kClosed
	neighbourNode.cost = cost
	neighbourNode.total = total
	if (neighbourNode.flags & kOpen) != 0 {
		// Already in open, update node location.
		this.m_OpenList.Modify(neighbourNode)
	} else {
		// Put the node in open list.
		neighbourNode.flags |= kOpen
		this.m_OpenList.Push(neighbourNode)
	}
}

// Finalizes sliced path find query and returns found path.
//  pathCount - (out) Number of polygons in search result.
func (this *NavMeshQuery) FinalizeSlicedFindPath(pathCount *int32) NavMeshStatus {
	Assert(this.m_NodePool != nil)
	*pathCount = 0
	if NavMeshStatusFailed(this.m_QueryData.status) {
		// Reset query.
		this.m_QueryData = NavMeshQueryData{}
		return kNavMeshFailure
	}

	n := int32(0)
	if this.m_QueryData.startRef == this.m_QueryData.endRef {
		// Special case: the search starts and ends at same poly.
		n = 1
	} else {
		// Reverse the path.
		Assert(this.m_QueryData.lastBestNode != nil)
		if this.m_QueryData.lastBestNode.id != this.m_QueryData.endRef {
			this.m_QueryData.status |= kNavMeshPartialResult
		}

		var prev *NavMeshNode
		node := this.m_QueryData.lastBestNode
		for {
			next := this.m_NodePool.GetNodeAtIdx(node.pidx)
			node.pidx = this.m_NodePool.GetNodeIdx(prev)
			prev = node
			node = next
			n++
			if node == nil {
				break
			}
		}

		this.m_QueryData.startNode = prev
	}

	details := this.m_QueryData.status & kNavMeshStatusDetailMask
	*pathCount = n
	return kNavMeshSuccess | details
}

// Finalizes partial sliced path find query and returns path to the furthest
// polygon on the existing path that was visited during the search.
//  existing - (out) Array of polygons in the existing path.
//  existingSize - (out) Number of polygons in existing path array.
//  pathCount - (out) Number of polygons in search result.
func (this *NavMeshQuery) FinalizeSlicedFindPathPartial(pathCount *int32, existing []NavMeshPolyRef,
	existingSize int32) NavMeshStatus {
	*pathCount = 0
	if existingSize == 0 {
		return kNavMeshFailure
	}

	if NavMeshStatusFailed(this.m_QueryData.status) {
		// Reset query.
		this.m_QueryData = NavMeshQueryData{}
		return kNavMeshFailure
	}

	n := int32(0)
	if this.m_QueryData.startRef == this.m_QueryData.endRef {
		// Special case: the search starts and ends at same poly.
		n = 1
	} else {
		// Find furthest existing node that was visited.
		var prev, node *NavMeshNode
		for i := existingSize - 1; i >= 0; i-- {
			node = this.m_NodePool.FindNavMeshNode(existing[i])
			if node == nil {
				break
			}
		}

		if node == nil {
			return kNavMeshFailure
		}

		// Reverse the path.
		for {
			next := this.m_NodePool.GetNodeAtIdx(node.pidx)
			node.pidx = this.m_NodePool.GetNodeIdx(prev)
			prev = node
			node = next
			n++
			if node == nil {
				break
			}
		}

		this.m_QueryData.startNode = prev
	}

	details := this.m_QueryData.status & kNavMeshStatusDetailMask
	*pathCount = n
	return kNavMeshSuccess | details
}

// Returns found path, must call finalizeSlicedFindPath() or finalizeSlicedFindPathPartial() before calling this function.
//  path - (out) array holding the search result.
//  pathCount - (out) Number of polygons in search result array.
//  maxPath - (in) The max number of polygons the path array can hold.
func (this *NavMeshQuery) GetPath(path []NavMeshPolyRef, pathCount *int32, maxPath int32) NavMeshStatus {
	if NavMeshStatusFailed(this.m_QueryData.status) {
		// Reset query.
		this.m_QueryData = NavMeshQueryData{}
		return kNavMeshFailure
	}

	n := int32(0)
	if this.m_QueryData.startRef == this.m_QueryData.endRef {
		// Special case: the search starts and ends at same poly.
		path[n] = this.m_QueryData.startRef
		n++
	} else {
		// startNode is updated when the path is reversed, must have it to continue.
		if this.m_QueryData.startNode == nil {
			return kNavMeshFailure
		}
		// Store path
		node := this.m_QueryData.startNode
		for {
			path[n] = node.id
			n++
			if n >= maxPath {
				this.m_QueryData.status |= kNavMeshBufferTooSmall
				break
			}
			node = this.m_NodePool.GetNodeAtIdx(node.pidx)
			if node == nil {
				break
			}
		}
	}

	details := this.m_QueryData.status & kNavMeshStatusDetailMask

	// Reset query.
	this.m_QueryData = NavMeshQueryData{}
	*pathCount = n
	return kNavMeshSuccess | details
}

// returns the Closest Point of Approach (CPA) on segment p for two line segments (p,q).
// note: returns segment midpoint in the degenerate case.
func SegmentSegmentCPA(p0 Vector3f, p1 Vector3f, q0 Vector3f, q1 Vector3f) Vector3f {
	u := p1.Sub(p0)
	v := q1.Sub(q0)
	w0 := p0.Sub(q0)

	a := DotVector3f(u, u)
	b := DotVector3f(u, v)
	c := DotVector3f(v, v)
	d := DotVector3f(u, w0)
	e := DotVector3f(v, w0)
	den := a*c - b*b
	if den == 0 {
		return p0.Add(p1).Mulf(0.5)
	}

	t := (b*e - c*d) / den
	return LerpVector3f(p0, p1, FloatClamp(t, 0, 1))
}

// Retrace portals between corners register if type of polygon changes
func (this *NavMeshQuery) RetracePortals(startIndex int32, endIndex int32,
	path []NavMeshPolyRef, n int32, termPos Vector3f, straightPath []Vector3f,
	straightPathFlags []byte, straightPathRefs []NavMeshPolyRef, maxStraightPath int32) int32 {
	Assert(n < maxStraightPath)
	Assert(startIndex <= endIndex)
	for k := startIndex; k < endIndex-1; k++ {
		type1 := DecodePolyIdType(path[k])
		type2 := DecodePolyIdType(path[k+1])
		if type1 != type2 {
			var l, r Vector3f
			status := this.GetPortalPoints(path[k], path[k+1], &l, &r)
			Assert(status == kNavMeshSuccess) // Expect path elements k, k+1 to be verified
			cpa := SegmentSegmentCPA(l, r, straightPath[n-1], termPos)
			straightPath[n] = cpa
			straightPathRefs[n] = path[k+1]
			if type2 == kPolyTypeOffMeshConnection {

				straightPathFlags[n] = byte(kStraightPathOffMeshConnection)
			} else {

				straightPathFlags[n] = 0
			}
			n++
			if n == maxStraightPath {
				return maxStraightPath
			}
		}
	}
	straightPath[n] = termPos
	straightPathRefs[n] = path[endIndex]
	if DecodePolyIdType(path[endIndex]) == kPolyTypeOffMeshConnection {

		straightPathFlags[n] = byte(kStraightPathOffMeshConnection)
	} else {

		straightPathFlags[n] = 0
	}
	n++
	return n
}

func WorldToTileSafe(tile *NavMeshTile, p Vector3f) Vector3f {
	if tile != nil {
		return WorldToTile(tile, p)
	} else {
		return p
	}
}

func TileToWorldSafe(tile *NavMeshTile, p Vector3f) Vector3f {
	if tile != nil {
		return TileToWorld(tile, p)
	} else {
		return p
	}
}

// Finds a straight path from start to end locations within the corridor
// described by the path polygons.
// Start and end locations will be clamped on the corridor.
// The returned polygon references are point to polygon which was entered when
// a path point was added. For the end point, zero will be returned. This allows
// to match for example off-mesh link points to their representative polygons.
// Params:
//  startPos[3] - (in) Path start location.
//  endPo[3] - (in) Path end location.
//  path - (in) Array of connected polygons describing the corridor.
//  pathSize - (in) Number of polygons in path array.
//  straightPath - (out) Points describing the straight path.
//  straightPathFlags - (out, opt) Flags describing each point type, see NavMeshStraightPathFlags.
//  straightPathRefs - (out, opt) References to polygons at point locations.
//  straightPathCount - (out) Number of points in the path.
//  maxStraightPath - (in) The max number of points the straight path array can hold. Must be at least 1
func (this *NavMeshQuery) FindStraightPath(startPos Vector3f, endPos Vector3f,
	path []NavMeshPolyRef, pathSize int32, straightPath []Vector3f,
	straightPathFlags []byte, straightPathRefs []NavMeshPolyRef,
	straightPathCount *int32, maxStraightPath int32) NavMeshStatus {
	Assert(this.m_NavMesh != nil)
	Assert(maxStraightPath > 1)
	Assert(pathSize > 0)
	Assert(straightPath != nil)
	Assert(straightPathRefs != nil)
	Assert(straightPathFlags != nil)
	if path[0] == 0 || !this.m_NavMesh.IsValidPolyRef(path[0]) {
		*straightPathCount = 0
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	straightPath[0] = startPos
	straightPathRefs[0] = path[0]
	straightPathFlags[0] = byte(kStraightPathStart)
	apexIndex := int32(0)
	n := int32(1)
	if pathSize > 1 {
		startTile := this.m_NavMesh.GetTileByRef(NavMeshTileRef(path[0]))
		apex := WorldToTileSafe(startTile, startPos)
		left := Vector3f{0.0, 0.0, 0.0}
		right := Vector3f{0.0, 0.0, 0.0}
		leftIndex := int32(-1)
		rightIndex := int32(-1)
		for i := int32(1); i <= pathSize; i++ {
			tile := this.m_NavMesh.GetTileByRef(NavMeshTileRef(path[apexIndex]))
			var vl, vr Vector3f
			if i == pathSize {
				vr = WorldToTileSafe(tile, endPos)
				vl = vr
			} else {
				if NavMeshStatusFailed(this.GetPortalPoints(path[i-1], path[i], &vl, &vr)) {
					return kNavMeshFailure | kNavMeshInvalidParam
				}
				Assert(this.m_NavMesh.IsValidPolyRef(path[i-1]))
				Assert(this.m_NavMesh.IsValidPolyRef(path[i]))
				vl = WorldToTileSafe(tile, vl)
				vr = WorldToTileSafe(tile, vr)
			}

			vl = vl.Sub(apex)
			vr = vr.Sub(apex)

			// Ensure left/right ordering
			if Perp2D(vl, vr) < 0 {
				vl, vr = vr, vl
			}

			// Terminate funnel by turning
			if Perp2D(left, vr) < 0 {
				termPos := TileToWorldSafe(tile, apex.Add(left))
				n = this.RetracePortals(apexIndex, leftIndex, path, n, termPos, straightPath, straightPathFlags, straightPathRefs, maxStraightPath)
				if n == maxStraightPath {
					*straightPathCount = n
					return kNavMeshBufferTooSmall | kNavMeshSuccess
				}

				apex = WorldToTileSafe(tile, termPos)
				left.SetZero()
				right.SetZero()
				i, apexIndex = leftIndex, leftIndex
				continue
			}
			if Perp2D(right, vl) > 0 {
				termPos := TileToWorldSafe(tile, apex.Add(right))
				n = this.RetracePortals(apexIndex, rightIndex, path, n, termPos, straightPath, straightPathFlags, straightPathRefs, maxStraightPath)
				if n == maxStraightPath {
					*straightPathCount = n
					return kNavMeshBufferTooSmall | kNavMeshSuccess
				}

				apex = WorldToTileSafe(tile, termPos)
				left.SetZero()
				right.SetZero()
				i, apexIndex = rightIndex, rightIndex
				continue
			}
			// Consider: additional termination test - based on changing up-vector in frame of reference

			// Narrow funnel
			if Perp2D(left, vl) >= 0 {
				left = vl
				leftIndex = i
			}
			if Perp2D(right, vr) <= 0 {
				right = vr
				rightIndex = i
			}
		}
	}

	// Remove the the next to last if duplicate point - e.g. start and end positions are the same
	// (in which case we have get a single point)
	if n > 0 && (straightPath[n-1] == endPos) {
		n--
	}
	n = this.RetracePortals(apexIndex, pathSize-1, path, n, endPos, straightPath, straightPathFlags, straightPathRefs, maxStraightPath)
	if n == maxStraightPath {
		*straightPathCount = n
		return kNavMeshBufferTooSmall | kNavMeshSuccess
	}

	// Fix flag for final path point
	straightPathFlags[n-1] = byte(kStraightPathEnd)
	*straightPathCount = n
	return kNavMeshSuccess
}

// Moves from startPos to endPos constrained to the navmesh.
// If the endPos is reachable, the resultPos will be endPos,
// or else the resultPos will be the nearest point in navmesh.
// Note: The resulting point is not projected to the ground, use GetPolyHeight() to get height.
// Note: The algorithm is optimized for small delta movement and small number of polygons.
// Params:
//  startRef - (in) ref to the polygon where startPos lies.
//  startPos[3] - (in) start position of the mover.
//  endPos[3] - (in) desired end position of the mover.
//  filter - (in) path polygon filter.
//  resultPos[3] - (out) new position of the mover.
//  visited - (out) array of visited polygons.
//  visitedCount - (out) Number of entries in the visited array.
//  maxVisitedSize - (in) max number of polygons in the visited array.
func (this *NavMeshQuery) MoveAlongSurface(startRef NavMeshPolyRef, startPos Vector3f,
	endPos Vector3f, filter *QueryFilter, resultPos *Vector3f, visited []NavMeshPolyRef,
	visitedCount *int32, maxVisitedSize int32) NavMeshStatus {
	return this.MoveAlongSurface2(startRef, startPos, endPos, filter, resultPos, visited, visitedCount, maxVisitedSize, this.m_TinyNodePool)
}

const kMaxStack = 48

func (this *NavMeshQuery) MoveAlongSurface2(startRef NavMeshPolyRef, startPos Vector3f,
	endPos Vector3f, filter *QueryFilter, resultPos *Vector3f, visited []NavMeshPolyRef,
	visitedCount *int32, maxVisitedSize int32, nodePool *NavMeshNodePool) NavMeshStatus {
	Assert(this.m_NavMesh != nil)
	Assert(nodePool != nil)
	*visitedCount = 0

	// Validate input
	if startRef == 0 {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	if !this.m_NavMesh.IsValidPolyRef(startRef) {
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	// Test start poly vs filter
	if !filter.PassFilter(this.m_NavMesh.GetPolyFlags(startRef)) {
		return kNavMeshFailure
	}
	status := kNavMeshSuccess

	// Here we assume all connected tiles are have the same world xform
	startTile := this.m_NavMesh.GetTileByRef(NavMeshTileRef(startRef))
	localStart := WorldToTile(startTile, startPos)
	localEnd := WorldToTile(startTile, endPos)
	var stack [kMaxStack]*NavMeshNode
	nstack := int32(0)
	nodePool.Clear()
	startNode := nodePool.GetNode(startRef)
	startNode.pidx = 0
	startNode.cost = 0
	startNode.total = 0
	startNode.id = startRef
	startNode.flags = kClosed
	stack[nstack] = startNode
	nstack++

	bestPos := localStart
	bestDist := float32(math.MaxFloat32)
	var bestNode *NavMeshNode

	// Search constraints
	searchPos := LerpVector3f(localStart, localEnd, 0.5)
	searchRadSqr := Sqr(Distance(localStart, localEnd)/2.0 + 0.001)
	var verts [kNavMeshVertsPerPoly]Vector3f
	var neighbours [kNavMeshVertsPerPoly * kMaxNeis]NavMeshPolyRef
	for nstack != 0 {
		// Pop front.
		curNode := stack[0]
		for i := int32(0); i < nstack-1; i++ {
			stack[i] = stack[i+1]
		}
		nstack--

		// Get poly and tile.
		// The API input has been checked already, skip checking internal data.
		curRef := curNode.id

		// Collect vertices.
		nverts := this.m_NavMesh.GetPolyGeometry(curRef, verts[:], neighbours[:], kMaxNeis)
		if nverts == 0 {
			continue
		}

		// If target is inside the poly, stop search.
		if PointInPolygon2D(localEnd, verts[:], nverts) {
			bestNode = curNode
			bestDist = 0
			bestPos = localEnd
			break
		}

		// Find wall edges and find nearest point inside the walls.
		var i, j int32
		for j = int32(nverts - 1); i < nverts; i++ {
			// Find links to neighbours.
			nneis := int32(0)
			neis := neighbours[j*kMaxNeis:]
			for k := int32(0); k < kMaxNeis; k++ {
				neiRef := neis[k]
				if neiRef == 0 {
					break
				}

				// Skip if no node can be allocated.
				neighbourNode := nodePool.GetNode(neiRef)
				if neighbourNode == nil {
					j = i
					continue
				}

				// Skip if already visited.
				if (neighbourNode.flags & kClosed) != 0 {
					j = i
					continue
				}

				// Do not advance if the polygon is excluded by the filter.
				if !filter.PassFilter(this.m_NavMesh.GetPolyFlags(neiRef)) {
					j = i
					continue
				}

				// Skip the link if it is too far from search constraint.
				// TODO: Maybe should use GetPortalPoints(), but this one is way faster.
				vj := verts[j]
				vi := verts[i]
				var tseg float32
				distSqr := SqrDistancePointSegment2D(&tseg, searchPos, vj, vi)
				if distSqr > searchRadSqr {
					j = i
					continue
				}

				// Mark as the node as visited and push to queue.
				if nstack < kMaxStack {
					neighbourNode.pidx = nodePool.GetNodeIdx(curNode)
					neighbourNode.flags |= kClosed
					stack[nstack] = neighbourNode
					nstack++
				}
				nneis++
			}

			if nneis == 0 {
				// Wall edge, calculate distance.
				vj := verts[j]
				vi := verts[i]
				var tseg float32
				distSqr := SqrDistancePointSegment2D(&tseg, localEnd, vj, vi)
				if distSqr < bestDist {
					// Update nearest distance.
					bestPos = LerpVector3f(vj, vi, tseg)
					bestDist = distSqr
					bestNode = curNode
				}
			}
			j = i
		}
	}

	n := int32(0)
	if bestNode != nil {
		// Reverse the path.
		var prev *NavMeshNode
		node := bestNode
		for {
			next := nodePool.GetNodeAtIdx(node.pidx)
			node.pidx = nodePool.GetNodeIdx(prev)
			prev = node
			node = next
			if node == nil {
				break
			}
		}

		// Store result
		node = prev
		for {
			visited[n] = node.id
			n++
			if n >= maxVisitedSize {
				status |= kNavMeshBufferTooSmall
				break
			}
			node = nodePool.GetNodeAtIdx(node.pidx)
			if node == nil {
				break
			}
		}
	}

	*resultPos = TileToWorld(startTile, bestPos)
	*visitedCount = n
	return status
}

// Returns portal points between two polygons.
func (this *NavMeshQuery) GetPortalPoints(from NavMeshPolyRef, to NavMeshPolyRef, left *Vector3f, right *Vector3f) NavMeshStatus {
	Assert(this.m_NavMesh != nil)

	// Handle ground.offmesh and offmesh.ground
	fromOffMeshCon := DecodePolyIdType(from) == uint32(kPolyTypeOffMeshConnection)
	toOffMeshCon := DecodePolyIdType(to) == uint32(kPolyTypeOffMeshConnection)
	if fromOffMeshCon || toOffMeshCon {
		for link := this.m_NavMesh.GetFirstLink(from); link != nil; link = this.m_NavMesh.GetNextLink(link) {
			if link.ref == to {
				iv := link.edge
				Assert(link.edge == 0 || link.edge == 1)
				var v0, v1 Vector3f
				if fromOffMeshCon {
					con := this.m_NavMesh.GetOffMeshConnection(from)
					v0 = con.endPoints[iv].mapped[0]
					v1 = con.endPoints[iv].mapped[1]
				} else if toOffMeshCon {
					con := this.m_NavMesh.GetOffMeshConnection(to)
					v0 = con.endPoints[iv].mapped[0]
					v1 = con.endPoints[iv].mapped[1]
				} else {
					Assert(false)
				}

				// Unpack portal limits.
				if link.bmin != 0 || link.bmax != 255 {
					s := float32(1.0 / 255.0)
					tmin := FloatMax(float32(link.bmin)*s, 0.0)
					tmax := FloatMin(float32(link.bmax)*s, 1.0)
					*left = LerpVector3f(v0, v1, tmin)
					*right = LerpVector3f(v0, v1, tmax)
				} else {
					*left = v0
					*right = v1
				}

				return kNavMeshSuccess
			}
		}
		return kNavMeshFailure
	}

	// Handle ground.ground

	var fromTile *NavMeshTile
	var fromPoly *NavMeshPoly
	if NavMeshStatusFailed(this.m_NavMesh.GetTileAndPolyByRef(from, &fromTile, &fromPoly)) {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	var toTile *NavMeshTile
	var toPoly *NavMeshPoly
	if NavMeshStatusFailed(this.m_NavMesh.GetTileAndPolyByRef(to, &toTile, &toPoly)) {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	ip := GetPolyIndex(fromTile, fromPoly)
	firstLink := fromTile.polyLinks[ip]
	var link *NavMeshLink
	for flink := this.m_NavMesh.GetLink(firstLink); flink != nil; flink = this.m_NavMesh.GetNextLink(flink) {
		if flink.ref == to {
			link = flink
			break
		}
	}
	if link == nil {
		return kNavMeshFailure | kNavMeshInvalidParam
	}

	// Find portal vertices.
	Assert(link.edge < fromPoly.vertCount)
	var nextEdge int32
	if link.edge+1 == fromPoly.vertCount {
		nextEdge = 0
	} else {
		nextEdge = int32(link.edge) + 1
	}
	v0 := fromPoly.verts[link.edge]
	v1 := fromPoly.verts[nextEdge]
	*left = fromTile.verts[v0]
	*right = fromTile.verts[v1]

	// If the link is at tile boundary, clamp the vertices to
	// the link width.
	if link.side != 0xff {
		// Unpack portal limits.
		if link.bmin != 0 || link.bmax != 255 {
			s := float32(1.0 / 255.0)
			tmin := FloatMax(float32(link.bmin)*s, 0.0)
			tmax := FloatMin(float32(link.bmax)*s, 1.0)
			*left = LerpVector3f(fromTile.verts[v0], fromTile.verts[v1], tmin)
			*right = LerpVector3f(fromTile.verts[v0], fromTile.verts[v1], tmax)
		}
	}

	*right = TileToWorld(fromTile, *right)
	*left = TileToWorld(fromTile, *left)
	return kNavMeshSuccess
}

// Casts 'walkability' ray along the navmesh surface from startPos towards the endPos.
// Params:
//  startRef - (in) ref to the polygon where the start lies.
//  startPos[3] - (in) start position of the query.
//  endPos[3] - (in) end position of the query.
//  filter - (in) path polygon filter.
//  result - (out) raycast result data.
//  path - (out,opt) visited path polygons.
//  pathCount - (out,opt) Number of polygons visited.
//  maxPath - (in) max number of polygons in the path array.
func (this *NavMeshQuery) Raycast(startRef NavMeshPolyRef, startPos Vector3f, endPos Vector3f, filter *QueryFilter,
	result *NavMeshRaycastResult, path []NavMeshPolyRef, pathCount *int32, maxPath int32) NavMeshStatus {
	Assert(this.m_NavMesh != nil)

	// Clear output
	*result = NavMeshRaycastResult{}
	if pathCount != nil {
		*pathCount = 0
	}

	// Validate input
	startTile := this.m_NavMesh.GetTileByRef(NavMeshTileRef(startRef))
	if startTile == nil {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	localStartPos := WorldToTile(startTile, startPos)
	localEndPos := WorldToTile(startTile, endPos)
	curRef := startRef
	lastPoly := startRef
	var verts [kNavMeshVertsPerPoly]Vector3f
	t := float32(0)
	costScale := float32(0)
	n := int32(0)
	status := kNavMeshSuccess
	for curRef != 0 {
		// Cast ray against current polygon.
		nverts := this.m_NavMesh.GetPolyGeometry(curRef, verts[:], nil, 0)
		if nverts == 0 {
			break
		}
		var tmin, tmax float32
		var segMin, segMax int32
		if !IntersectSegmentPoly2D(&tmin, &tmax, &segMin, &segMax, localStartPos, localEndPos, verts[:], nverts) && segMax != -1 {
			// Could not hit the polygon, keep the old t and report hit.

			// In some cases, when 'startPos' is collinear to an edge between two polygons, 'tmax' becomes a tiny
			// negative number and we would break here - causing the raycast to terminate prematurely.
			// We prevent that by ensuring tmax > 0 for the first polygon.
			if curRef != startRef || tmax > 0 {
				break
			}
		}

		// Keep track of furthest t so far.
		if tmax > t {
			dt := tmax - t
			costScale += dt * float32(filter.GetAreaCost(int32(this.m_NavMesh.GetPolyArea(curRef))))
			t = tmax
		}

		lastPoly = curRef

		// Store visited polygons.
		if n < maxPath {
			path[n] = curRef
			n++

		} else {
			status |= kNavMeshBufferTooSmall
		}

		// Ray end is completely inside the polygon.
		if segMax == -1 {
			t = 1.0
			break
		}

		// Follow neighbours.
		var nextRef NavMeshPolyRef

		for link := this.m_NavMesh.GetFirstLink(curRef); link != nil; link = this.m_NavMesh.GetNextLink(link) {
			// Skip off-mesh connections.
			if DecodePolyIdType(link.ref) == kPolyTypeOffMeshConnection {
				continue
			}

			// Find link which contains this edge.
			if int32(link.edge) != segMax {
				continue
			}

			// If the link is internal, accept and just return the ref.
			if link.side == 0xff {
				nextRef = link.ref
				break
			}

			// If the link is at tile boundary,
			// Check if the link spans the whole edge, and accept.
			if link.bmin == 0 && link.bmax == 255 {
				nextRef = link.ref
				break
			}

			// Check for partial edge links.
			Assert(int32(link.edge) < nverts)
			var nextEdge int32
			if int32(link.edge)+1 == nverts {
				nextEdge = 0
			} else {
				nextEdge = int32(link.edge) + 1
			}
			left := verts[link.edge]
			right := verts[nextEdge]

			// Check that the intersection lies inside the link portal.
			if link.side == 0 || link.side == 4 {
				// Calculate link size.
				s := float32(1.0 / 255.0)
				lmin := left.z + (right.z-left.z)*float32(link.bmin)*s
				lmax := left.z + (right.z-left.z)*float32(link.bmax)*s
				if lmin > lmax {
					lmin, lmax = lmax, lmin
				}

				// Find Z intersection.
				z := localStartPos.z + (localEndPos.z-localStartPos.z)*tmax
				if z >= lmin && z <= lmax {
					nextRef = link.ref
					break
				}
			} else if link.side == 2 || link.side == 6 {
				// Calculate link size.
				s := float32(1.0 / 255.0)
				lmin := left.x + (right.x-left.x)*float32(link.bmin)*s
				lmax := left.x + (right.x-left.x)*float32(link.bmax)*s
				if lmin > lmax {
					lmin, lmax = lmax, lmin
				}

				// Find X intersection.
				x := localStartPos.x + (localEndPos.x-localStartPos.x)*tmax
				if x >= lmin && x <= lmax {
					nextRef = link.ref
					break
				}
			}
		}

		if nextRef == 0 || !filter.PassFilter(this.m_NavMesh.GetPolyFlags(nextRef)) {
			// No accessible neighbour, we hit a wall.

			// Calculate hit normal.
			a := segMax
			b := int32(0)
			if segMax+1 < nverts {
				b = segMax + 1
			}
			va := verts[a]
			vb := verts[b]
			dx := vb.x - va.x
			dz := vb.z - va.z
			lenSq := dx*dx + dz*dz
			if lenSq > 0 {
				s := float32(1.0 / math.Sqrt(float64(lenSq)))
				result.normal.x = dz * s
				result.normal.y = 0.0
				result.normal.z = -dx * s
			}

			// Advance ref - it allows us to obtain the blocking poly ref.
			curRef = nextRef
			break
		}

		// No hit, advance to neighbour polygon.
		curRef = nextRef
	}

	result.t = t
	result.totalCost = costScale * Distance(startPos, endPos)
	result.lastPoly = lastPoly
	result.hitPoly = curRef
	result.normal = TileToWorldVector(startTile, result.normal)
	if pathCount != nil {
		*pathCount = n
	}
	return status
}

// Finds non-overlapping local neighbourhood around center location.
// Note: The algorithm is optimized for small query radius and small number of polygons.
// Params:
//  startRef - (in) ref to the polygon where the search starts.
//  centerPos[3] - (in) center if the query circle.
//  radius - (in) radius of the query circle.
//  filter - (in) path polygon filter.
//  resultRef - (out) refs to the polygons touched by the circle.
//  resultParent - (out, opt) parent of each result polygon.
//  resultCount - (out) number of results.
//  maxResult - (int) maximum capacity of search results.
func (this *NavMeshQuery) FindLocalNeighbourhood(startRef NavMeshPolyRef, centerPos Vector3f,
	radius float32, filter *QueryFilter, resultRef []NavMeshPolyRef, resultParent []NavMeshPolyRef,
	resultCount *int32, maxResult int32) NavMeshStatus {
	Assert(this.m_NavMesh != nil)
	Assert(this.m_TinyNodePool != nil)
	*resultCount = 0

	// Validate input
	if startRef == 0 || !this.m_NavMesh.IsValidPolyRef(startRef) {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	var stack [kMaxStack]*NavMeshNode
	nstack := 0
	this.m_TinyNodePool.Clear()
	startNode := this.m_TinyNodePool.GetNode(startRef)
	startNode.pidx = 0
	startNode.id = startRef
	startNode.flags = kClosed
	stack[nstack] = startNode
	nstack++

	radiusSqr := Sqr(radius)
	var pa [kNavMeshVertsPerPoly]Vector3f
	var pb [kNavMeshVertsPerPoly]Vector3f
	status := kNavMeshSuccess
	n := int32(0)
	if n < maxResult {
		resultRef[n] = startNode.id
		if resultParent != nil {
			resultParent[n] = 0
		}
		n++
	} else {
		status |= kNavMeshBufferTooSmall
	}

	for nstack != 0 {
		// Pop front.
		curNode := stack[0]
		for i := 0; i < nstack-1; i++ {
			stack[i] = stack[i+1]
		}
		nstack--

		// Get poly and tile.
		// The API input has been checked already, skip checking internal data.
		curRef := curNode.id
		for link := this.m_NavMesh.GetFirstLink(curRef); link != nil; link = this.m_NavMesh.GetNextLink(link) {
			neighbourRef := link.ref

			// Skip off-mesh connections.
			if DecodePolyIdType(neighbourRef) == kPolyTypeOffMeshConnection {
				continue
			}

			// Skip invalid neighbours.
			if neighbourRef == 0 {
				continue
			}

			// Skip if cannot alloca more nodes.
			neighbourNode := this.m_TinyNodePool.GetNode(neighbourRef)
			if neighbourNode == nil {
				continue
			}
			// Skip visited.
			if (neighbourNode.flags & kClosed) != 0 {
				continue
			}

			// Expand to neighbour
			var neighbourTile *NavMeshTile
			var neighbourPoly *NavMeshPoly
			this.m_NavMesh.GetTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly)

			// Do not advance if the polygon is excluded by the filter.
			if !filter.PassFilter(neighbourPoly.flags) {
				continue
			}

			// Find edge and calc distance to the edge.
			var va, vb Vector3f
			if this.GetPortalPoints(curRef, neighbourRef, &va, &vb) == 0 {
				continue
			}

			// If the circle is not touching the next polygon, skip it.
			var tseg float32
			distSqr := SqrDistancePointSegment2D(&tseg, centerPos, va, vb)
			if distSqr > radiusSqr {
				continue
			}

			// Mark node visited, this is done before the overlap test so that
			// we will not visit the poly again if the test fails.
			neighbourNode.flags |= kClosed
			neighbourNode.pidx = this.m_TinyNodePool.GetNodeIdx(curNode)

			// Check that the polygon does not collide with existing polygons.

			// Collect vertices of the neighbour poly.
			npa := this.m_NavMesh.GetPolyGeometry(neighbourRef, pa[:], nil, 0)
			overlap := false
			for j := int32(0); j < n; j++ {
				pastRef := resultRef[j]
				// Connected polys do not overlap.
				connected := false
				for clink := this.m_NavMesh.GetFirstLink(curRef); clink != nil; clink = this.m_NavMesh.GetNextLink(clink) {
					if clink.ref == pastRef {
						connected = true
						break
					}
				}
				if connected {
					continue
				}
				// Get vertices and test overlap
				npb := this.m_NavMesh.GetPolyGeometry(pastRef, pb[:], nil, 0)
				if OverlapPolyPoly2D(pa[:], npa, pb[:], npb) {
					overlap = true
					break
				}
			}
			if overlap {
				continue
			}

			// This poly is fine, store and advance to the poly.
			if n < maxResult {
				resultRef[n] = neighbourRef
				if resultParent != nil {
					resultParent[n] = curRef
				}
				n++
			} else {
				status |= kNavMeshBufferTooSmall
			}

			if nstack < kMaxStack {
				stack[nstack] = neighbourNode
				nstack++
			}
		}
	}

	*resultCount = n
	return status
}

type SegInterval struct {
	ref        NavMeshPolyRef
	tmin, tmax int16
}

func InsertInterval(ints []SegInterval, nints int32, maxInts int32, tmin int16, tmax int16, ref NavMeshPolyRef) int32 {
	if nints+1 > maxInts {
		return maxInts
	}
	// Find insertion point.
	idx := int32(0)
	for idx < nints {
		if tmax <= ints[idx].tmin {
			break
		}
		idx++
	}
	// Move current results.
	if nints-idx != 0 {
		for i := nints - idx - 1; i >= 0; i-- {
			ints[nints+idx+1+i] = ints[nints+idx+i]
		}
	}
	// Store
	ints[idx].ref = ref
	ints[idx].tmin = tmin
	ints[idx].tmax = tmax
	return nints + 1
}

const MAX_INTERVAL = 16

// Returns wall segments of specified polygon.
// If 'segmentRefs' is specified, both the wall and portal segments are returned.
// Wall segments will have null (0) polyref, and portal segments store the polygon they lead to.
// Params:
//  ref - (in) ref to the polygon.
//  filter - (in) path polygon filter.
//  segmentVerts[2*maxSegments] - (out) wall segments (2 endpoints per segment).
//  segmentRefs[maxSegments] - (out,opt) reference to a neighbour.
//  segmentCount - (out) number of wall segments.
//  maxSegments - (in) max number of segments that can be stored in 'segments'.
func (this *NavMeshQuery) GetPolyWallSegments(ref NavMeshPolyRef, filter *QueryFilter,
	segmentVerts []Vector3f, segmentRefs []NavMeshPolyRef, segmentCount *int32, maxSegments int32) NavMeshStatus {
	Assert(this.m_NavMesh != nil)
	if DecodePolyIdType(ref) == kPolyTypeOffMeshConnection {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	var tile *NavMeshTile
	var poly *NavMeshPoly
	if NavMeshStatusFailed(this.m_NavMesh.GetTileAndPolyByRef(ref, &tile, &poly)) {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	n := int32(0)
	var ints [MAX_INTERVAL]SegInterval
	storePortals := segmentRefs != nil
	status := kNavMeshSuccess
	for i, j := uint8(0), poly.vertCount-1; i < poly.vertCount; j, i = i, i+1 {
		// Skip non-solid edges.
		nints := int32(0)
		if (poly.neis[j] & kNavMeshExtLink) != 0 {
			// Tile border.
			ip := GetPolyIndex(tile, poly)
			firstLink := tile.polyLinks[ip]
			for link := this.m_NavMesh.GetLink(firstLink); link != nil; link = this.m_NavMesh.GetNextLink(link) {
				if link.edge == j {
					if link.ref != 0 {
						var neiTile *NavMeshTile
						var neiPoly *NavMeshPoly
						this.m_NavMesh.GetTileAndPolyByRefUnsafe(link.ref, &neiTile, &neiPoly)
						if neiPoly != nil && filter.PassFilter(neiPoly.flags) {
							nints = InsertInterval(ints[:], nints, MAX_INTERVAL, int16(link.bmin), int16(link.bmax), link.ref)
						}
					}
				}
			}
		} else {
			// Internal edge
			var ref NavMeshPolyRef
			if (poly.neis[j]) != 0 {
				idx := uint32(poly.neis[j] - 1)
				ref = this.m_NavMesh.GetPolyRefBase(tile) | EncodeBasePolyId(kPolyTypeGround, idx)
				if !storePortals && !filter.PassFilter(tile.polys[idx].flags) {
					ref = 0
				}
			}

			// If the edge leads to another polygon and portals are not stored, skip.
			if ref != 0 && !storePortals {
				continue
			}
			if n < maxSegments {
				vj := tile.verts[poly.verts[j]]
				vi := tile.verts[poly.verts[i]]
				segmentVerts[2*n+0] = vj
				segmentVerts[2*n+1] = vi
				if segmentRefs != nil {
					segmentRefs[n] = ref
				}
				n++
			} else {
				status |= kNavMeshBufferTooSmall
			}

			continue
		}

		// Add sentinels
		nints = InsertInterval(ints[:], nints, MAX_INTERVAL, -1, 0, 0)
		nints = InsertInterval(ints[:], nints, MAX_INTERVAL, 255, 256, 0)

		// Store segments.
		vj := tile.verts[poly.verts[j]]
		vi := tile.verts[poly.verts[i]]
		for k := int32(1); k < nints; k++ {
			// Portal segment.
			if storePortals && ints[k].ref != 0 {
				tmin := float32(ints[k].tmin) / 255.0
				tmax := float32(ints[k].tmax) / 255.0
				if n < maxSegments {
					segmentVerts[2*n+0] = LerpVector3f(vj, vi, tmin)
					segmentVerts[2*n+1] = LerpVector3f(vj, vi, tmax)
					if segmentRefs != nil {
						segmentRefs[n] = ints[k].ref
					}
					n++
				} else {
					status |= kNavMeshBufferTooSmall
				}
			}

			// Wall segment.
			imin := ints[k-1].tmax
			imax := ints[k].tmin
			if imin != imax {
				tmin := float32(imin) / 255.0
				tmax := float32(imax) / 255.0
				if n < maxSegments {
					segmentVerts[2*n+0] = LerpVector3f(vj, vi, tmin)
					segmentVerts[2*n+1] = LerpVector3f(vj, vi, tmax)
					if segmentRefs != nil {
						segmentRefs[n] = 0
					}
					n++
				} else {
					status |= kNavMeshBufferTooSmall
				}
			}
		}
	}

	*segmentCount = n
	return status
}

/// Finds the distance from the specified position to the nearest polygon wall.
///  @param[in]         startRef           The reference id of the polygon containing @p centerPos.
///  @param[in]         centerPos          The center of the search circle. [(x, y, z)]
///  @param[in]         filter             The polygon filter to apply to the query.
///  @param[out]        hitDist            The distance to the nearest wall from @p centerPos.
///  @param[out]        hitPos             The nearest position on the wall that was hit. [(x, y, z)]
///  @param[out]        hitNormal          The normalized ray formed from the wall point to the
///                                        source point. [(x, y, z)]
/// @returns The status flags for the query.
func (this *NavMeshQuery) FindDistanceToWall(startRef NavMeshPolyRef, centerPos Vector3f,
	filter *QueryFilter, hitDist *float32, hitPos *Vector3f, hitNormal *Vector3f,
	hitFlags *uint32) NavMeshStatus {
	Assert(this.m_NavMesh != nil)
	Assert(this.m_NodePool != nil)
	Assert(this.m_OpenList != nil)

	// Validate input
	startTile := this.m_NavMesh.GetTileByRef(NavMeshTileRef(startRef))
	if startTile == nil {
		return kNavMeshFailure | kNavMeshInvalidParam
	}
	localCenterPos := WorldToTile(startTile, centerPos)
	this.m_NodePool.Clear()
	this.m_OpenList.Clear()
	startNode := this.m_NodePool.GetNode(startRef)
	startNode.pos = localCenterPos
	startNode.pidx = 0
	startNode.cost = 0
	startNode.total = 0
	startNode.id = startRef
	startNode.flags = kOpen
	this.m_OpenList.Push(startNode)
	radiusSqr := float32(math.MaxFloat32)
	status := kNavMeshSuccess
	var verts [kNavMeshVertsPerPoly]Vector3f
	var neighbours [kNavMeshVertsPerPoly * kMaxNeis]NavMeshPolyRef
	edgeNormal := Vector3f{0.0, 0.0, 0.0}
	for !this.m_OpenList.empty() {
		bestNode := this.m_OpenList.Pop()
		bestNode.flags &= ^kOpen
		bestNode.flags |= kClosed

		// Get poly and tile.

		// The API input has been checked already, skip checking internal data.
		bestRef := bestNode.id

		// Get parent poly and tile.
		var parentRef NavMeshPolyRef = 0
		if bestNode.pidx != 0 {
			parentRef = this.m_NodePool.GetNodeAtIdx(bestNode.pidx).id
		}

		// Collect vertices.
		nverts := this.m_NavMesh.GetPolyGeometry(bestRef, verts[:], neighbours[:], kMaxNeis)
		if nverts == 0 {
			continue
		}

		// Hit test walls, pruning the search distance too.
		for i, j := int32(0), nverts-1; i < nverts; j, i = i, i+1 {
			// Skip non-solid edges.
			solid := true
			neis := neighbours[j*kMaxNeis:]
			for k := int32(0); k < kMaxNeis; k++ {
				if neis[k] == 0 {
					break
				}
				if filter.PassFilter(this.m_NavMesh.GetPolyFlags(neis[k])) {
					// Edge is passable.
					solid = false
					break
				}
			}
			if !solid {
				continue
			}

			// Calc distance to the edge.
			vj := verts[j]
			vi := verts[i]
			var tseg float32
			distSqr := SqrDistancePointSegment2D(&tseg, localCenterPos, vj, vi)

			// Edge is too far, skip.
			if distSqr > radiusSqr {
				continue
			}

			// Hit wall, update radius.
			radiusSqr = distSqr
			if hitFlags != nil {
				*hitFlags = this.m_NavMesh.GetPolyFlags(bestRef)
			}

			// Calculate hit pos.
			*hitPos = LerpVector3f(vj, vi, tseg)

			// Calculate the 2D edge normal
			edgeNormal.x = vi.z - vj.z
			edgeNormal.z = vj.x - vi.x
		}

		// Expand search
		for i, j := int32(0), nverts-1; i < nverts; j, i = i, i+1 {
			// Skip non-solid edges.
			neis := neighbours[j*kMaxNeis:]
			for k := int32(0); k < kMaxNeis; k++ {
				if neis[k] == 0 {
					break
				}
				neighbourRef := neis[k]
				// Skip invalid neighbours and do not follow back to parent.
				if neighbourRef == 0 || neighbourRef == parentRef {
					continue
				}
				// Skip off-mesh connections.
				if DecodePolyIdType(neighbourRef) == kPolyTypeOffMeshConnection {
					continue
				}
				// Calculate distance to the edge.
				va := verts[j]
				vb := verts[i]
				var tseg float32
				distSqr := SqrDistancePointSegment2D(&tseg, localCenterPos, va, vb)

				// If the circle is not touching the next polygon, skip it.
				if distSqr > radiusSqr {
					continue
				}
				if !filter.PassFilter(this.m_NavMesh.GetPolyFlags(neighbourRef)) {
					continue
				}
				neighbourNode := this.m_NodePool.GetNode(neighbourRef)
				if neighbourNode == nil {
					status |= kNavMeshOutOfNodes
					continue
				}

				if (neighbourNode.flags & kClosed) != 0 {
					continue
				}

				// Cost
				if neighbourNode.flags == kNew {
					neighbourNode.pos = LerpVector3f(va, vb, 0.5)
				}

				total := bestNode.total + Distance(bestNode.pos, neighbourNode.pos)

				// The node is already in open list and the new result is worse, skip.
				if (neighbourNode.flags&kOpen) != 0 && total >= neighbourNode.total {
					continue
				}
				neighbourNode.id = neighbourRef
				neighbourNode.flags = neighbourNode.flags & ^kClosed
				neighbourNode.pidx = this.m_NodePool.GetNodeIdx(bestNode)
				neighbourNode.total = total
				if (neighbourNode.flags & kOpen) != 0 {
					this.m_OpenList.Modify(neighbourNode)
				} else {
					neighbourNode.flags |= kOpen
					this.m_OpenList.Push(neighbourNode)
				}
			}
		}
	}

	// Calculate the 2D hit normal
	// Use the direction to the hit position to ensure smooth behaviour across concave corner points.
	// Use the navmesh edge normal when position mapped onto/outside the edge.
	direction := localCenterPos.Sub(*hitPos)
	dot2d := direction.x*edgeNormal.x + direction.z*edgeNormal.z
	if dot2d > 1e-4 {
		direction.y = 0.0
		*hitNormal = Normalize(direction)
	} else {
		*hitNormal = NormalizeSafe(edgeNormal, Vector3f{0, 0, 0})
	}

	*hitDist = Sqrt(radiusSqr)
	*hitNormal = TileToWorldVector(startTile, *hitNormal)
	*hitPos = TileToWorld(startTile, *hitPos)
	return status
}

// Returns true if poly reference ins in closed list.
func (this *NavMeshQuery) IsInClosedList(ref NavMeshPolyRef) bool {
	if this.m_NodePool == nil {
		return false
	}

	node := this.m_NodePool.FindNavMeshNode(ref)
	return node != nil && node.flags&kClosed != 0
}

package unityai

import (
	"fmt"
	"math"
	"math/rand"

	"github.com/lazytiger/unityai/format"
)

const kNodePoolSize int32 = 4096

type NavMeshHit struct {
	position Vector3f
	normal   Vector3f
	distance float32
	mask     uint32
	hit      bool
}

func (this *NavMeshHit) GetPosition() Vector3f {
	return this.position
}

func (this *NavMeshHit) GetNormal() Vector3f {
	return this.normal
}

func (this *NavMeshHit) GetDistance() float32 {
	return this.distance
}

func (this *NavMeshHit) GetMask() uint32 {
	return this.mask
}

func (this *NavMeshHit) Hit() bool {
	return this.hit
}

type NavMeshManager struct {
	m_NavMesh                *NavMesh
	m_NavMeshQuery           *NavMeshQuery
	m_Filter                 *QueryFilter
	m_LegacyQueryExtents     Vector3f
	m_LegacyLinkQueryExtents Vector3f
}

func NewManagerFromData(nvData *format.NavMeshData) (*NavMeshManager, error) {
	data := NewDataFromFormat(nvData)
	manager := NewNavMeshManager()
	err := manager.LoadData(data)
	return manager, err
}

func NewNavMeshManager() *NavMeshManager {
	navMesh := NewNavMesh()
	query := NewNavMeshQuery(navMesh, kNodePoolSize)
	return &NavMeshManager{
		navMesh,
		query,
		NewQueryFilter(),
		Vector3f{0, 0, 0},
		Vector3f{0, 0, 0},
	}
}

func (this *NavMeshManager) Clone() *NavMeshManager {
	filter := *this.m_Filter
	return &NavMeshManager{
		this.m_NavMesh, NewNavMeshQuery(this.m_NavMesh, kNodePoolSize), &filter,
		this.m_LegacyQueryExtents, this.m_LegacyLinkQueryExtents,
	}
}

func (this *NavMeshManager) LoadData(nvData *NavMeshData) error {
	position := nvData.GetPosition()
	rotation := nvData.GetRotation()
	settings := nvData.GetNavMeshBuildSettings()

	this.m_Filter.SetAreaCosts(nvData.GetFilterAreaCosts())
	this.m_LegacyQueryExtents = Vector3f{settings.agentRadius, settings.agentHeight, settings.agentRadius}
	this.m_LegacyLinkQueryExtents = Vector3f{settings.agentRadius, settings.agentClimb, settings.agentRadius}
	tiles := nvData.GetNavMeshTiles()
	surfaceID := this.m_NavMesh.CreateSurface(len(tiles), settings, position, rotation)
	for i := range tiles {
		tile := tiles[i]
		data := tile.GetData()
		if len(data) == 0 {
			continue
		}
		var ref NavMeshTileRef
		status := this.m_NavMesh.AddTile(data, int32(len(data)), kTileLeakData, surfaceID, &ref)
		if NavMeshStatusFailed(status) {
			if NavMeshStatusDetail(status, kNavMeshWrongMagic) || NavMeshStatusDetail(status, kNavMeshWrongVersion) {
				return fmt.Errorf("Loading NavMesh failed - wrong format. Please rebake the NavMesh")
			} else if NavMeshStatusDetail(status, kNavMeshOutOfMemory) {
				return fmt.Errorf("Loading NavMesh failed - out of memory")
			} else {
				return fmt.Errorf("Loading NavMesh tile #%d failed. Error code:%x\n", i, status)
			}
		}
	}

	offMeshLinks := nvData.GetOffMeshLinks()
	for i := range offMeshLinks {
		data := offMeshLinks[i]
		var conn OffMeshConnectionParams
		conn.startPos = data.m_Start
		conn.endPos = data.m_End
		conn.up = Vector3f{0, 1, 0}
		conn.width = 0.0
		conn.costModifier = -1.0
		conn.linkDirection = data.m_LinkDirection
		conn.flags = 1 << data.m_Area
		conn.area = data.m_Area
		conn.linkType = data.m_LinkType
		conn.userID = 0
		conn.agentTypeID = nvData.GetAgentTypeId()
		ref := this.m_NavMesh.AddOffMeshConnection(&conn, settings.agentRadius, settings.agentClimb)
		if ref == 0 {
			return fmt.Errorf("add offmeshlink failed")
		}
	}

	return nil
}

func InvalidateNavMeshHit(hit *NavMeshHit) {
	inf := float32(math.MaxFloat32)
	hit.position = Vector3f{inf, inf, inf}
	hit.normal.SetZero()
	hit.distance = inf
	hit.mask = 0
	hit.hit = false
}

func (this *NavMeshManager) Raycast(hit *NavMeshHit, sourcePosition, targetPosition Vector3f) bool {
	var mappedPolyRef NavMeshPolyRef
	var mappedPosition Vector3f
	ext := this.m_LegacyQueryExtents
	if !this.MapPosition(&mappedPolyRef, &mappedPosition, sourcePosition, ext) {
		InvalidateNavMeshHit(hit)
		return false
	}

	var result NavMeshRaycastResult
	status := this.m_NavMeshQuery.Raycast(mappedPolyRef, mappedPosition, targetPosition,
		this.m_Filter, &result, nil, nil, 0)
	if NavMeshStatusFailed(status) {
		InvalidateNavMeshHit(hit)
		return false
	}

	lpos := LerpVector3f(mappedPosition, targetPosition, result.t)
	var pos Vector3f
	this.m_NavMeshQuery.ProjectToPoly(&pos, result.lastPoly, lpos)
	blocked := result.t < 1.0
	hit.position = pos
	hit.normal = result.normal
	hit.distance = Magnitude(hit.position.Sub(sourcePosition))
	hit.mask = this.m_NavMesh.GetPolyFlags(result.hitPoly)
	hit.hit = blocked
	return blocked
}

func (this *NavMeshManager) DistanceToEdge(hit *NavMeshHit, sourcePosition Vector3f) bool {
	var mappedPolyRef NavMeshPolyRef
	var mappedPosition Vector3f
	ext := this.m_LegacyQueryExtents
	if !this.MapPosition(&mappedPolyRef, &mappedPosition, sourcePosition, ext) {
		InvalidateNavMeshHit(hit)
		return false
	}

	mask := uint32(0)
	status := this.m_NavMeshQuery.FindDistanceToWall(mappedPolyRef, mappedPosition, this.m_Filter,
		&hit.distance, &hit.position, &hit.normal,
		&mask)
	hit.mask = mask
	if NavMeshStatusFailed(status) {
		InvalidateNavMeshHit(hit)
		return false
	}
	hit.hit = true
	return true
}

func (this *NavMeshManager) SamplePosition(hit *NavMeshHit, sourcePosition Vector3f, maxDistance float32) bool {
	var mappedPolyRef NavMeshPolyRef
	var mappedPosition Vector3f
	extents := Vector3f{maxDistance, maxDistance, maxDistance}
	if !this.MapPosition(&mappedPolyRef, &mappedPosition, sourcePosition, extents) {
		InvalidateNavMeshHit(hit)
		return false
	}

	distance := Magnitude(mappedPosition.Sub(sourcePosition))
	if distance > maxDistance {
		InvalidateNavMeshHit(hit)
		return false
	}

	hit.position = mappedPosition
	hit.normal.SetZero()
	hit.distance = distance
	hit.mask = this.m_NavMesh.GetPolyFlags(mappedPolyRef)
	hit.hit = true
	return true
}

func (this *NavMeshManager) MapPosition(mappedPolyRef *NavMeshPolyRef, mappedPosition *Vector3f, position, extents Vector3f) bool {
	if this.m_NavMeshQuery == nil {
		return false
	}
	this.m_NavMeshQuery.FindNearestPoly(position, extents, this.m_Filter, mappedPolyRef, mappedPosition)
	return *mappedPolyRef != 0
}

func (this *NavMeshManager) CalculatePolygonPath(path *NavMeshPath, sourcePosition, targetPosition Vector3f, maxIter int32) int32 {
	path.SetTimeStamp(0)
	path.SetPolygonCount(0)
	path.SetStatus(kPathInvalid)
	if this.m_NavMeshQuery == nil {
		return 0
	}

	var targetMappedPos, sourceMappedPos Vector3f
	query := this.m_NavMeshQuery
	ext := this.m_LegacyQueryExtents

	var targetPolyRef NavMeshPolyRef
	query.FindNearestPoly(targetPosition, ext, this.m_Filter, &targetPolyRef, &targetMappedPos)
	if targetPolyRef == 0 {
		return 0
	}

	var sourcePolyRef NavMeshPolyRef
	query.FindNearestPoly(sourcePosition, ext, this.m_Filter, &sourcePolyRef, &sourceMappedPos)
	if sourcePolyRef == 0 {
		return 0
	}

	polygonCount := int32(0)

	status := query.InitSlicedFindPath2(sourcePolyRef, targetPolyRef, sourceMappedPos, targetMappedPos, this.m_Filter)
	if !NavMeshStatusFailed(status) {
		status = query.UpdateSlicedFindPath(maxIter, nil)
	}
	if !NavMeshStatusFailed(status) {
		status = query.FinalizeSlicedFindPath(&polygonCount)
	}
	path.ReservePolygons(polygonCount)
	if !NavMeshStatusFailed(status) {
		status = query.GetPath(path.GetPolygonPath(), &polygonCount, path.GetPolygonCapacity())
	}

	path.SetTimeStamp(this.m_NavMesh.GetTimeStamp())
	path.SetPolygonCount(polygonCount)
	path.SetSourcePosition(sourceMappedPos)
	path.SetTargetPosition(targetMappedPos)
	if NavMeshStatusFailed(status) || polygonCount == 0 {
		path.SetStatus(kPathInvalid)
		return 0
	}

	if NavMeshStatusDetail(status, kNavMeshPartialResult) {
		// when path is partial we project the target position
		// to the last polygon in the path.

		polygonPath := path.GetPolygonPath()
		lastPolyRef := polygonPath[polygonCount-1]
		var partialTargetPos Vector3f
		status := query.ClosestPointOnPoly(lastPolyRef, targetMappedPos, &partialTargetPos)
		if NavMeshStatusFailed(status) {
			path.SetStatus(kPathInvalid)
			return 0
		}

		path.SetStatus(kPathPartial)
		path.SetTargetPosition(partialTargetPos)

		// If the pathfinding req ran out of nodes - we mark the path as stale
		// i.e. outdating the path by settings the time-stamp to 0
		if NavMeshStatusDetail(status, kNavMeshOutOfNodes) {
			path.SetTimeStamp(0)
		}
	} else {
		path.SetStatus(kPathComplete)
	}

	return polygonCount
}

func (this *NavMeshManager) CalculatePathCorners(corners []Vector3f, maxCorners int32, path *NavMeshPath) int32 {
	if this.m_NavMeshQuery == nil || corners == nil || maxCorners < 2 || path.GetPolygonCount() < 1 {
		return 0
	}

	cornerCount := int32(0)
	var result NavMeshStatus
	sourcePos := path.GetSourcePosition()
	targetPos := path.GetTargetPosition()
	refs := make([]NavMeshPolyRef, maxCorners)
	flags := make([]uint8, maxCorners)
	query := this.m_NavMeshQuery
	result = query.FindStraightPath(sourcePos, targetPos,
		path.GetPolygonPath(), path.GetPolygonCount(),
		corners, flags, refs, &cornerCount, maxCorners)
	Assert(cornerCount <= maxCorners)
	if NavMeshStatusFailed(result) {
		return 0
	}
	return cornerCount
}

//TODO not fully tested
func (this *NavMeshManager) WalkableBetween(source, target Vector3f) bool {
	var sourceRef, targetRef NavMeshPolyRef
	this.m_NavMeshQuery.FindNearestPoly(source, this.m_LegacyQueryExtents, this.m_Filter, &sourceRef, &source)
	if sourceRef == 0 {
		return false
	}

	this.m_NavMeshQuery.FindNearestPoly(target, this.m_LegacyQueryExtents, this.m_Filter, &targetRef, &target)
	if targetRef == 0 {
		return false
	}

	visited := make(map[NavMeshPolyRef]bool)
	openList := make([]NavMeshPolyRef, 1, 10)
	openList[0] = sourceRef
	for count := 1; count > 0; count = len(openList) {
		ref := openList[count-1]
		visited[ref] = true
		openList = openList[:count-1]
		for link := this.m_NavMesh.GetFirstLink(ref); link != nil; link = this.m_NavMesh.GetNextLink(link) {
			if link.ref == targetRef {
				return true
			}
			if !visited[link.ref] {
				openList = append(openList, link.ref)
			}
		}
	}
	return false
}

func (this *NavMeshManager) CalculatePath(source, target Vector3f, maxIter int32) ([]Vector3f, bool) {
	path := NewNavMeshPath()
	if this.CalculatePolygonPath(path, source, target, maxIter) <= 0 {
		return []Vector3f{}, true
	}
	cornerCount := path.GetPolygonCount() + 2
	corners := make([]Vector3f, cornerCount)
	cornerCount = this.CalculatePathCorners(corners, cornerCount, path)
	return corners[:cornerCount], path.m_status == kPathPartial
}

func (this *NavMeshManager) GetNavMesh() *NavMesh {
	return this.m_NavMesh
}

func (this *NavMeshManager) GetNavMeshQuery() *NavMeshQuery {
	return this.m_NavMeshQuery
}

func (this *NavMeshManager) GetFilter() *QueryFilter {
	return this.m_Filter
}

func (this *NavMeshManager) FindNearestPoly(center Vector3f, nearestRef *NavMeshPolyRef, nearestPt *Vector3f) {
	this.m_NavMeshQuery.FindNearestPoly(center, this.m_LegacyQueryExtents,
		this.m_Filter, nearestRef, nearestPt)
}

func (this *NavMeshManager) MoveAlongSurface(startRef NavMeshPolyRef, startPos Vector3f, endPos Vector3f, resultPos *Vector3f, visited []NavMeshPolyRef, visitedCount *int32, maxVisitedSize int32) NavMeshStatus {
	status := this.m_NavMeshQuery.MoveAlongSurface(startRef, startPos, endPos, this.m_Filter, resultPos, visited, visitedCount, maxVisitedSize)
	if !NavMeshStatusSucceed(status) {
		return status
	}

	var resultRef = startRef
	for index, viRef := range visited {
		if viRef == 0 && index > 0 {
			resultRef = visited[index-1]
			break
		}
	}

	var height float32
	status = this.m_NavMeshQuery.GetPolyHeightLocal(resultRef, *resultPos, &height)
	if !NavMeshStatusSucceed(status) {
		return status
	}

	resultPos.SetData(1, height)
	return status
}

func (this *NavMeshManager) FindRandomPointInCircle(center Vector3f, radius float32) (Vector3f, error) {
	var result Vector3f
	var centerRef NavMeshPolyRef
	this.m_NavMeshQuery.FindNearestPoly(center, this.m_LegacyQueryExtents, this.m_Filter, &centerRef, &result)
	if centerRef == 0 {
		return result, fmt.Errorf("center:%v not found in navmesh", center)
	}
	var resultRef [32]NavMeshPolyRef
	var refCount int32
	status := this.m_NavMeshQuery.FindLocalNeighbourhood(centerRef, center, radius, this.m_Filter, resultRef[:], nil, &refCount, 32)
	if !NavMeshStatusSucceed(status) {
		return result, fmt.Errorf("find local neighbourhood failed")
	}

	//calculate all the polygon area, randomed with weight as area
	var verts [kNavMeshVertsPerPoly]Vector3f
	weights := make([]float32, refCount)
	var areaSum float32
	for i := int32(0); i < refCount; i++ {
		vertCount := this.m_NavMesh.GetPolyGeometry(resultRef[i], verts[:], nil, 0)
		var polyArea float32
		for i := int32(2); i < vertCount; i++ {
			va := verts[0]
			vb := verts[i-1]
			vc := verts[i-2]
			polyArea += TriangleAreaXZ(va, vb, vc)
		}
		weights[i] = polyArea
		areaSum += polyArea
	}
	targetArea := rand.Float32() * areaSum
	var searchArea float32
	var targetPolyRef NavMeshPolyRef
	for i := int32(0); i < refCount; i++ {
		searchArea += weights[i]
		if searchArea >= targetArea {
			targetPolyRef = resultRef[i]
			break
		}
	}
	//find the targeted polygon, random a point in the polygon
	vertCount := this.m_NavMesh.GetPolyGeometry(targetPolyRef, verts[:], nil, 0)
	result = RandomPointInConvexPoly(verts[:], int(vertCount))
	result = TileToWorld(this.m_NavMesh.GetTileByRef(NavMeshTileRef(targetPolyRef)), result)

	//fix height
	var height float32
	status = this.m_NavMeshQuery.GetPolyHeightLocal(targetPolyRef, result, &height)
	if !NavMeshStatusSucceed(status) {
		return result, fmt.Errorf("find poly height failed")
	}
	result.y = height
	return result, nil
}

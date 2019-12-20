package unityai

type PathCorridorState uint8

const (
	kPathCorridorValid       PathCorridorState = 1 << 0
	kPathCorridorPartial     PathCorridorState = 1 << 1
	kPathCorridorInterrupted PathCorridorState = 1 << 2
)

type PathCorridor struct {
	m_pos        Vector3f
	m_target     Vector3f
	m_path       []NavMeshPolyRef
	m_stateFlags PathCorridorState
}

func NewPathCorridor() *PathCorridor {
	return &PathCorridor{
		m_pos:        Vector3f{},
		m_target:     Vector3f{},
		m_path:       make([]NavMeshPolyRef, 0, 4),
		m_stateFlags: 0,
	}
}

func (this *PathCorridor) GetCurrentPos() Vector3f {
	return this.m_pos
}

func (this *PathCorridor) ClearPath() {
	this.m_path = this.m_path[:0]
}

func (this *PathCorridor) Reset(ref NavMeshPolyRef, pos Vector3f) {
	if ref == 0 {
		this.Invalidate()
		return
	}

	this.m_path = this.m_path[:0]
	this.m_path = append(this.m_path, ref)
	this.m_pos = pos
	this.m_target = pos
	this.m_stateFlags = kPathCorridorValid
}

func (this *PathCorridor) Invalidate() {
	// Preserve the position and target
	this.m_path = this.m_path[:0]
	this.m_path = append(this.m_path, NavMeshPolyRef(0))
	this.m_stateFlags = 0
}

func (this *PathCorridor) SetToEnd() {
	Assert(this.GetPathCount() != 0)
	this.m_pos = this.m_target
	this.m_path[0] = this.GetLastPoly()
	this.m_path = this.m_path[:1]
}

const kMinTargetDistSq float32 = 0.0001

func (this *PathCorridor) GetPathCount() int32 {
	return int32(len(this.m_path))
}

func (this *PathCorridor) FindCorners(cornerVerts []Vector3f, cornerFlags []uint8,
	cornerPolys []NavMeshPolyRef, cornerCount *int32, maxCorners int32,
	navquery *NavMeshQuery) NavMeshStatus {
	Assert(this.GetPathCount() != 0)
	var ncorners int32 = 0
	status := navquery.FindStraightPath(this.m_pos, this.m_target, this.m_path, this.GetPathCount(),
		cornerVerts, cornerFlags, cornerPolys, &ncorners, maxCorners)
	if ncorners == 0 {
		*cornerCount = 0
		return status
	}

	// Prune points in the beginning of the path which are too close.
	var prune int32
	for prune = 0; prune < ncorners; prune++ {
		if (NavMeshStraightPathFlags(cornerFlags[prune])&kStraightPathOffMeshConnection) != 0 || SqrDistance2D(cornerVerts[prune], this.m_pos) > kMinTargetDistSq {
			break
		}
	}
	ncorners -= prune
	if prune != 0 && ncorners != 0 {
		for i := int32(0); i < ncorners; i++ {
			cornerFlags[i] = cornerFlags[i+prune]
			cornerPolys[i] = cornerPolys[i+prune]
			cornerVerts[i] = cornerVerts[i+prune]
		}
	}

	// Prune points after an off-mesh connection.
	for prune = 0; prune < ncorners; prune++ {
		if NavMeshStraightPathFlags(cornerFlags[prune])&kStraightPathOffMeshConnection != 0 {
			ncorners = prune + 1
			break
		}
	}

	*cornerCount = ncorners
	if NavMeshStatusDetail(status, kNavMeshPartialResult) {
		return kNavMeshSuccess | kNavMeshPartialResult
	}
	return kNavMeshSuccess
}

func (this *PathCorridor) OptimizePathVisibility(next Vector3f, navquery *NavMeshQuery, filter *QueryFilter) {
	var res [kMaxResults]NavMeshPolyRef
	var nres int32 = 0
	var result NavMeshRaycastResult
	navquery.Raycast(this.m_path[0], this.m_pos, next, filter, &result, res[:], &nres, kMaxResults)
	if nres > 1 && result.t > 0.99 {
		ReplacePathStart(&this.m_path, res[:], nres)
	}
}

const kMaxIterations int32 = 8

func (this *PathCorridor) OptimizePathTopology(navquery *NavMeshQuery, filter *QueryFilter) bool {
	if this.GetPathCount() < 3 {
		return false
	}

	var res [kMaxResults]NavMeshPolyRef
	var nres int32 = 0
	status := navquery.InitSlicedFindPath2(this.m_path[0], this.GetLastPoly(), this.m_pos, this.m_target, filter)
	if !NavMeshStatusFailed(status) {
		status = navquery.UpdateSlicedFindPath(kMaxIterations, nil)
	}

	if !NavMeshStatusSucceed(status) {
		// don't accept kNavMeshInProgress
		return false
	}

	status = navquery.FinalizeSlicedFindPathPartial(&nres, this.m_path, this.GetPathCount())
	if !NavMeshStatusSucceed(status) {
		return false
	}

	status = navquery.GetPath(res[:], &nres, kMaxResults)
	if !NavMeshStatusSucceed(status) {
		return false
	}

	return ReplacePathStart(&this.m_path, res[:], nres)
}

func (this *PathCorridor) MoveOverOffmeshConnection(offMeshConRef NavMeshPolyRef, currentPos Vector3f,
	startPos Vector3f, endPos Vector3f, navquery *NavMeshQuery) bool {
	Assert(navquery != nil)
	Assert(this.GetPathCount() != 0)

	// Advance the path up to and over the off-mesh connection.
	var prevRef, nextRef NavMeshPolyRef
	polyRef := this.m_path[0]
	var npos int32 = 0
	npath := this.GetPathCount()
	for npos < npath && polyRef != offMeshConRef {
		prevRef = polyRef
		polyRef = this.m_path[npos]
		if npos+1 < npath {
			nextRef = this.m_path[npos+1]
		}
		npos++
	}
	if npos == npath {
		// Could not find offMeshConRef
		return false
	}

	// Prune path
	this.m_path = this.m_path[npos:]

	nav := navquery.GetAttachedNavMesh()
	Assert(nav != nil)
	conn := nav.GetOffMeshConnection(polyRef)
	if conn == nil {
		return false
	}

	if conn.width > 0.0 {
		// Handle wide link
		status := nav.GetNearestOffMeshConnectionEndPoints(prevRef, polyRef, nextRef, currentPos, &startPos, &endPos)
		if NavMeshStatusSucceed(status) {
			this.m_pos = endPos
			return true
		}
	} else {
		status := nav.GetOffMeshConnectionEndPoints(prevRef, polyRef, &startPos, &endPos)
		if NavMeshStatusSucceed(status) {
			this.m_pos = endPos
			return true
		}
	}

	return false
}

// TODO : notify callers  - return  success/failure
const kMaxVisited int32 = 16

func (this *PathCorridor) MovePosition(newPos Vector3f, navquery *NavMeshQuery, filter *QueryFilter) bool {
	Assert(this.GetPathCount() != 0)
	if SqrDistance2D(newPos, this.m_pos) == 0.0 {
		return false
	}

	// Move along navmesh and update new position.
	var result Vector3f
	var visited [kMaxVisited]NavMeshPolyRef
	var nvisited int32 = 0
	status := navquery.MoveAlongSurface(this.m_path[0], this.m_pos, newPos, filter,
		&result, visited[:], &nvisited, kMaxVisited)

	if !NavMeshStatusSucceed(status) {
		return false
	}

	ReplacePathStartReverse(&this.m_path, visited[:], nvisited)
	// Adjust the position to stay on top of the navmesh.
	navquery.ProjectToPoly(&this.m_pos, this.m_path[0], result)
	return true
}

func (this *PathCorridor) UpdateTargetPosition(ref NavMeshPolyRef, target Vector3f) bool {
	if ref != this.GetLastPoly() {
		return false
	}

	this.m_target = target
	return true
}

const kExtraCapacity uint32 = 16

func (this *PathCorridor) SetCorridor(target Vector3f, navquery *NavMeshQuery, path []NavMeshPolyRef, npath int32, partialPath bool) {
	Assert(npath > 0)

	// Reserving room for extra polygons allows us to subsequently extend the path a bit,
	// e.g. from a thread/job, without allocating memory and possibly locking.
	this.m_path = make([]NavMeshPolyRef, npath)
	this.m_target = target
	copy(this.m_path, path[:npath])

	this.m_stateFlags = kPathCorridorValid
	this.SetPathPartial(partialPath)

	// Adjust the position to stay on top of the navmesh.
	navquery.ProjectToPoly(&this.m_target, this.GetLastPoly(), target)
}

func (this *PathCorridor) SetStateFlag(setFlag bool, stateFlag PathCorridorState) {
	if setFlag {
		this.m_stateFlags |= stateFlag
	} else {
		this.m_stateFlags &= ^stateFlag
	}
}

func (this *PathCorridor) SetPathValid(inbool bool) {
	this.SetStateFlag(inbool, kPathCorridorValid)
}

func (this *PathCorridor) SetPathPartial(inbool bool) {
	this.SetStateFlag(inbool, kPathCorridorPartial)
}

func (this *PathCorridor) SetPathInterrupted(inbool bool) {
	this.SetStateFlag(inbool, kPathCorridorInterrupted)
}

func (this *PathCorridor) GetLastPoly() NavMeshPolyRef {
	return this.m_path[this.GetPathCount()-1]
}

func (this *PathCorridor) GetPath() []NavMeshPolyRef {
	return this.m_path
}

func ReplacePathStart(path *[]NavMeshPolyRef, start []NavMeshPolyRef, nstart int32) bool {
	npath := int32(len(*path))
	var ipath, istart int32
	if !FindFurthestIntersectionIndices(*path, start, npath, nstart, &ipath, &istart) {
		return false
	}

	// the result may only grow before the elements are moved in-place.
	tmpath := make([]NavMeshPolyRef, npath)
	copy(tmpath, *path)

	nres := istart + (npath - ipath)
	if nres > npath {
		*path = append(*path, make([]NavMeshPolyRef, nres-npath)...)
	}

	// move elements in place
	copy((*path)[istart:], tmpath[ipath:npath])
	copy(*path, start[:istart])

	*path = (*path)[:nres]
	// shrink result to fit
	return true
}

func FindFurthestIntersectionIndices(a []NavMeshPolyRef, b []NavMeshPolyRef, na, nb int32, ia, ib *int32) bool {
	for i := na - 1; i >= int32(0); i-- {
		for j := nb - 1; j >= 0; j-- {
			if a[i] == b[j] {
				*ia = i
				*ib = j
				return true
			}
		}
	}
	return false
}

func ReplacePathStartReverse(path *[]NavMeshPolyRef, start []NavMeshPolyRef, nstart int32) bool {
	npath := int32(len(*path))
	var ipath, istart int32
	if !FindFurthestIntersectionIndices(*path, start, npath, nstart, &ipath, &istart) {
		return false
	}

	// the pivot index for the reversed start segment
	istartrev := nstart - 1 - istart

	tmpath := make([]NavMeshPolyRef, npath)
	copy(tmpath, *path)
	nres := istartrev + (npath - ipath)
	// the result may only grow before the elements are moved in-place.
	if nres > npath {
		*path = append(*path, make([]NavMeshPolyRef, nres-npath)...)
	}

	// move elements in place
	copy((*path)[istartrev:], tmpath[ipath:])
	for i := int32(0); i < istartrev; i++ {
		(*path)[i] = start[nstart-1-i]
	}

	// shrink result to fit
	*path = (*path)[:nres]
	return true
}

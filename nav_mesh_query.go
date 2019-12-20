package unityai

type NavMeshRaycastResult struct {
	t         float32        // hit parameter along the segment, 1 if no hit.
	normal    Vector3f       // edge hit normal
	totalCost float32        // weighed length of raycast, using filter weights.
	lastPoly  NavMeshPolyRef // last valid polygon
	hitPoly   NavMeshPolyRef // last polygon or polygon hit - zero if external edge
}

type NavMeshQueryData struct {
	status           NavMeshStatus
	lastBestNode     *NavMeshNode
	startNode        *NavMeshNode
	lastBestNodeCost float32
	startRef, endRef NavMeshPolyRef
	startPos, endPos Vector3f
	filter           *QueryFilter
}

type NavMeshQuery struct {
	m_NavMesh      *NavMesh          // Pointer to navmesh data.
	m_QueryData    NavMeshQueryData  // Sliced query state.
	m_TinyNodePool *NavMeshNodePool  // Pointer to small node pool.
	m_NodePool     *NavMeshNodePool  // Pointer to node pool.
	m_OpenList     *NavMeshNodeQueue // Pointer to open list queue.
}

func (this *NavMeshQuery) GetAttachedNavMesh() *NavMesh {
	return this.m_NavMesh
}

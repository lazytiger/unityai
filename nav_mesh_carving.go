package unityai

type CarveResult struct {
	carvedData     []byte
	carvedDataSize uint32
	status         CarveResultStatus // CarveResultStatus
}

type ObstacleCarveInfo struct {
	shape        NavMeshCarveShape // Current shape
	bounds       MinMaxAABB        // Current bounds, cached from shape to allow matching add/remove.
	obstacle     *NavMeshObstacle
	versionStamp int32
}
type CarveResultContainer []CarveResult

func (c *CarveResultContainer) resize_initialized(size int) {
	if cap(*c) >= size {
		*c = (*c)[:size]
	} else {
		*c = append(*c, make([]CarveResult, size-len(*c))...)
	}
}

type NavMeshCarving struct {
	m_NavMeshManager   *NavMeshManager
	m_TileCarveData    []CarveData
	m_ObstacleInfo     []ObstacleCarveInfo
	m_ObstacleFreelist []int32
	m_DirtyBounds      []MinMaxAABB
	m_CarveResults     CarveResultContainer
}

func NewNavMeshCarving(manager *NavMeshManager) *NavMeshCarving {
	return &NavMeshCarving{
		m_NavMeshManager:   manager,
		m_TileCarveData:    nil,
		m_ObstacleInfo:     nil,
		m_ObstacleFreelist: nil,
		m_DirtyBounds:      nil,
		m_CarveResults:     nil,
	}
}

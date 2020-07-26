package unityai

import (
	"sort"
)

func (this *NavMeshCarving) AddObstacle(obstacle *NavMeshObstacle, handle *int32) {
	size := len(this.m_ObstacleFreelist)
	if size != 0 {
		*handle = this.m_ObstacleFreelist[size-1]
		this.m_ObstacleFreelist = this.m_ObstacleFreelist[:size-1]
	} else {
		*handle = int32(len(this.m_ObstacleInfo))
		this.m_ObstacleInfo = append(this.m_ObstacleInfo, ObstacleCarveInfo{})
	}

	info := &this.m_ObstacleInfo[*handle]
	info.obstacle = obstacle
	info.versionStamp = -1
	info.shape = NavMeshCarveShape{}
}

func (this *NavMeshCarving) RemoveObstacle(handle *int32) {
	Assert(*handle >= 0 && *handle < int32(len(this.m_ObstacleInfo)))
	this.m_ObstacleInfo[*handle].obstacle = nil
	this.m_DirtyBounds = append(this.m_DirtyBounds, this.m_ObstacleInfo[*handle].bounds)
	this.m_ObstacleFreelist = append(this.m_ObstacleFreelist, *handle)
	*handle = -1
}

func (this *NavMeshCarving) Carve() {
	Assert(len(this.m_TileCarveData) == len(this.m_CarveResults))
	if len(this.m_TileCarveData) == 0 {
		return
	}

	for i := range this.m_TileCarveData {
		this.CarveOne(&this.m_TileCarveData[i], &this.m_CarveResults[i])
	}
}

func (this *NavMeshCarving) CarveOne(carveData *CarveData, res *CarveResult) {
	surfaceID := carveData.m_SurfaceID
	tileIndex := carveData.m_TileIndex
	res.status = kRemoveTile
	res.carvedData = nil
	res.carvedDataSize = 0

	// Get source tile data
	sourceTile := this.m_NavMeshManager.GetSourceTileData(surfaceID, tileIndex)
	if sourceTile == nil {
		return
	}

	settings := this.m_NavMeshManager.GetNavMeshBuildSettings(surfaceID)
	height := settings.agentHeight
	radius := settings.agentRadius
	cellSize := settings.cellSize
	quantSize := cellSize / 64.0

	// This tile requires carving, carve based on source data.
	sourceData := sourceTile.GetData()
	sourceDataSize := len(sourceData)

	sort.Sort(Shapes(carveData.m_Shapes))
	res.status = CarveNavMeshTile(&res.carvedData, &res.carvedDataSize, sourceData, int32(sourceDataSize),
		carveData.m_Shapes, len(carveData.m_Shapes),
		height, radius, quantSize, carveData.m_Position, carveData.m_Rotation)
}

type TileLocation struct {
	m_Bounds    MinMaxAABB
	m_SurfaceID int32
	m_TileIndex int32
}

func (this *NavMeshCarving) PrepareCarving() {
	if len(this.m_ObstacleInfo) == 0 && len(this.m_DirtyBounds) == 0 {
		return
	}

	// Update the bounds for carving
	// Dirty previously carved bounds if obstacle moves
	var updateBounds []MinMaxAABB
	obstacleCount := len(this.m_ObstacleInfo)
	for i := 0; i < obstacleCount; i++ {
		if this.m_ObstacleInfo[i].obstacle == nil {
			continue
		}

		// If the obstacle has not changed, no need to update.
		if this.m_ObstacleInfo[i].versionStamp == this.m_ObstacleInfo[i].obstacle.GetVersionStamp() {
			continue
		}

		this.m_ObstacleInfo[i].versionStamp = this.m_ObstacleInfo[i].obstacle.GetVersionStamp()

		// Mark previously carved bounds as dirty.
		//this.m_DirtyBounds = append(this.m_DirtyBounds, this.m_ObstacleInfo[i].bounds)

		// Get new carve data and mark the new location to be updated.
		shape := &this.m_ObstacleInfo[i].shape
		this.m_ObstacleInfo[i].obstacle.GetCarveShape(shape)
		this.m_ObstacleInfo[i].bounds = shape.bounds
		updateBounds = append(updateBounds, this.m_ObstacleInfo[i].bounds)
	}

	// No tiles or obstacles changed
	if len(updateBounds) == 0 && len(this.m_DirtyBounds) == 0 {
		return
	}

	var locations []TileLocation
	manager := this.m_NavMeshManager
	manager.GetSourceTileDataBounds(&locations)

	currentSurfaceID := int32(0)
	cachedIsDirty := false
	var cachedTilePosition Vector3f
	var cachedTileRotation Quaternionf
	var cachedTileTransform Matrix4x4f
	navmesh := manager.GetNavMesh()

	for i := 0; i < len(locations); i++ {
		loc := locations[i]
		tileBounds := loc.m_Bounds
		settings := manager.GetNavMeshBuildSettings(loc.m_SurfaceID)
		walkableHeight := settings.agentHeight
		walkableRadius := settings.agentRadius

		// Extend the tile bounds by the carving dimensions
		// Note that the asymmetry in the vertical direction - which includes tiles below carving object, but not above
		// Carve hull is expanded so that some corners are offset up to (and a little over) sqrt (2) * radius.
		kOffsetRadius := walkableRadius * 1.415
		tileBounds.m_Min.x -= kOffsetRadius
		tileBounds.m_Min.z -= kOffsetRadius

		tileBounds.m_Max.x += kOffsetRadius
		tileBounds.m_Max.y += walkableHeight
		tileBounds.m_Max.z += kOffsetRadius

		// TODO: clean up - the outer loop should be over surfaces
		if currentSurfaceID != loc.m_SurfaceID {
			navmesh.GetSurfaceTransform(loc.m_SurfaceID, &cachedTilePosition, &cachedTileRotation)
			cachedTileTransform.SetTR(cachedTilePosition, cachedTileRotation)
			cachedIsDirty = false
			currentSurfaceID = loc.m_SurfaceID
		}

		var worldTileBounds MinMaxAABB
		TransformAABBSlow(tileBounds, cachedTileTransform, &worldTileBounds)
		carveData := NewCarveData(loc.m_SurfaceID, loc.m_TileIndex)
		carveData.m_Position = cachedTilePosition
		carveData.m_Rotation = cachedTileRotation
		if cachedIsDirty {
			// Surface is dirty - this tile should be carved if any obstacle overlaps
			for j := 0; j < len(this.m_ObstacleInfo); j++ {
				info := this.m_ObstacleInfo[j]
				if info.obstacle != nil {
					if IntersectAABBAABB(info.bounds, worldTileBounds) {
						carveData.AddShape(info.shape)
					}
				}
			}
			if !carveData.Empty() {
				this.m_TileCarveData = append(this.m_TileCarveData, *carveData)
			}

			// Intentionally ignore the dirty-bounds vs. dirty-tile overlap (restore tile)
			// assuming the dirty-tile is just loaded - and hence restoring is needless.
		} else {
			if this.NeedsUpdateCollectCarveData(carveData, worldTileBounds, updateBounds) {
				this.m_TileCarveData = append(this.m_TileCarveData, *carveData)
			}
		}
	}

	this.m_CarveResults.resize_initialized(len(this.m_TileCarveData))
	this.m_DirtyBounds = this.m_DirtyBounds[:0]
}

func CompareCarveShapes(lhs, rhs *NavMeshCarveShape) bool {
	if lhs.center.x < rhs.center.x {
		return true
	}
	if rhs.center.x < lhs.center.x {
		return false
	}

	// rhs.x == lhs.x
	if lhs.center.z < rhs.center.z {
		return true
	}
	if rhs.center.z < lhs.center.z {
		return false
	}

	// rhs.x == lhs.x && rhs.z == lhs.z
	if lhs.center.y < rhs.center.y {
		return true
	}
	if rhs.center.y < lhs.center.y {
		return false
	}

	// all same - favour the biggest
	return SqrMagnitude(lhs.extents) > SqrMagnitude(rhs.extents)
}

func (this *NavMeshCarving) ApplyCarveResults() bool {
	carved := false

	Assert(len(this.m_TileCarveData) == len(this.m_CarveResults))
	manager := this.m_NavMeshManager
	for i := 0; i < len(this.m_TileCarveData); i++ {
		carveData := &this.m_TileCarveData[i]
		surfaceID := carveData.m_SurfaceID
		tileIndex := carveData.m_TileIndex
		res := &this.m_CarveResults[i]
		status := res.status
		if status == kRestoreTile {
			manager.RestoreTile(surfaceID, tileIndex)
		} else if status == kReplaceTile {
			carved = true
			manager.RemoveTile(surfaceID, tileIndex)
			if res.carvedData != nil && res.carvedDataSize > 0 {
				manager.ReplaceTile(surfaceID, tileIndex, res.carvedData, int32(res.carvedDataSize))
			}
		} else {
			Assert(status == kRemoveTile)
			carved = true
			manager.RemoveTile(surfaceID, tileIndex)
		}
	}

	this.m_TileCarveData = this.m_TileCarveData[:0]
	this.m_CarveResults = this.m_CarveResults[:0]
	return carved
}
func AnyOverlaps(arrayOfBounds []MinMaxAABB, bounds MinMaxAABB) bool {
	count := len(arrayOfBounds)
	for i := 0; i < count; i++ {
		if IntersectAABBAABB(arrayOfBounds[i], bounds) {
			return true
		}
	}
	return false
}

func (this *NavMeshCarving) NeedsUpdateCollectCarveData(carveData *CarveData, bounds MinMaxAABB,
	updateBounds []MinMaxAABB) bool {
	dirtyOverlaps := AnyOverlaps(this.m_DirtyBounds, bounds)
	if dirtyOverlaps || AnyOverlaps(updateBounds, bounds) {
		for i := 0; i < len(this.m_ObstacleInfo); i++ {
			if this.m_ObstacleInfo[i].obstacle != nil {
				if IntersectAABBAABB(this.m_ObstacleInfo[i].bounds, bounds) {
					carveData.AddShape(this.m_ObstacleInfo[i].shape)
				}
			}
		}
	}
	return dirtyOverlaps || !carveData.Empty()
}

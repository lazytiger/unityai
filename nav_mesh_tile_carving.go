package unityai

import (
	"math"
	"reflect"
	"sort"
	"unsafe"
)

type CarveResultStatus int32

const (
	kReplaceTile CarveResultStatus = iota
	kRestoreTile
	kRemoveTile
)

type ClippedDetailMesh struct {
	polyIndex int
	vertices  []Vector3f
	triangles []uint16
}

func NewClippedDetailMesh() *ClippedDetailMesh {
	return &ClippedDetailMesh{
		polyIndex: 0,
		vertices:  nil,
		triangles: nil,
	}
}

type DetailMeshBVNode struct {
	min Vector3f
	max Vector3f
	idx int32
}

type DetailMeshPoly struct {
	vertBase  int32
	vertCount int32
	triBase   int32
	triCount  int32
	bvBase    int32
	bvCount   int32
}

type Triangles []uint16

func (t *Triangles) resize_uninitialized(size int) {
	if cap(*t) >= size {
		*t = (*t)[:size]
	} else {
		*t = append(*t, make([]uint16, size-len(*t))...)
	}
}

type PolyContainer []DetailMeshPoly

func (c *PolyContainer) resize_uninitialized(size int) {
	if cap(*c) >= size {
		*c = (*c)[:size]
	} else {
		*c = append(*c, make([]DetailMeshPoly, size-len(*c))...)
	}
}

type DetailMesh struct {
	vertices  []Vector3f
	triangles Triangles
	polys     PolyContainer
	bvNodes   DetailMeshBVNodeContainer
}

type DetailMeshBVNodeContainer []DetailMeshBVNode

func (this *DetailMeshBVNodeContainer) resize_uninitialized(size int32) {
	if cap(*this) >= int(size) {
		*this = (*this)[:size]
	}else {
		*this = append(*this, make([]DetailMeshBVNode, int(size) - len(*this))...)
	}
}

func NewDetailMesh() *DetailMesh {
	return &DetailMesh{}
}

func CarveNavMeshTile(tileData *[]byte, tileDataSize *uint32,
	sourceData []byte, sourceDataSize int32,
	shapes []NavMeshCarveShape, shapeCount int,
	carveDepth float32, carveWidth float32, quantSize float32,
	position Vector3f, rotation Quaternionf) CarveResultStatus {
	Assert(sourceData != nil)
	Assert(sourceDataSize > 0)
	*tileData = nil
	*tileDataSize = 0
	if shapeCount == 0 {
		return kRestoreTile
	}

	tile := NewNavMeshTile()
	if !PatchMeshTilePointers(tile, sourceData, sourceDataSize) {
		// remove tile altogether if we cannot patch source data pointers
		return kRemoveTile
	}

	Assert(tile.header != nil)
	tileOffset := tile.header.bmin.Add(tile.header.bmax).Mulf(0.5)

	detailMesh := NewDetailMesh()
	UnpackDetailMesh(detailMesh, tile, tileOffset)
	var mat Matrix4x4f
	mat.SetTRInverse(position, rotation)
	hull := Hull{}
	var carveHulls DetailHullContainer
	for i := 0; i < shapeCount; i++ {
		var localShape NavMeshCarveShape
		localShape.shape = shapes[i].shape
		localShape.center = mat.MultiplyPoint3(shapes[i].center)
		localShape.extents = shapes[i].extents
		localShape.xAxis = mat.MultiplyVector3(shapes[i].xAxis)
		localShape.yAxis = mat.MultiplyVector3(shapes[i].yAxis)
		localShape.zAxis = mat.MultiplyVector3(shapes[i].zAxis)
		TransformAABBSlow(shapes[i].bounds, mat, &localShape.bounds)
		validHull := false
		var localBounds MinMaxAABB
		if localShape.shape == kObstacleShapeCapsule {
			validHull = CalculateCapsuleHull(&hull, &localBounds, &localShape, tileOffset, carveDepth, carveWidth)
		} else if localShape.shape == kObstacleShapeBox {
			validHull = CalculateBoxHull(&hull, &localBounds, &localShape, tileOffset, carveDepth, carveWidth)
		}

		if !validHull {
			continue
		}

		localBounds.m_Min = localBounds.m_Min.Sub(NewVector3f(quantSize, quantSize, quantSize))
		localBounds.m_Max = localBounds.m_Max.Add(NewVector3f(quantSize, quantSize, quantSize))

		// Find potentially intersecting polygons and create new cutter
		// based on the intersection points in detail mesh.
		var detailHulls DetailHullContainer
		validHull = BuildDetailHulls(&detailHulls, hull, localBounds, detailMesh, tile, tileOffset, quantSize)
		if validHull {
			carveHulls = append(carveHulls, detailHulls...)
		}
	}

	// The vertex quantization factor needs to match the tile size
	// in order to not get any gaps at tile boundaries.
	// As long as the divider is large enough and divisible by 2
	// (because tileOffset is at tile center during carving),
	// things should work fine.
	quantFactor := quantSize
	dynamicMesh := NewDynamicMesh(quantFactor)
	TileToDynamicMesh(tile, dynamicMesh, tileOffset)

	// Restore if nothing was clipped
	if !dynamicMesh.ClipPolys2(carveHulls) {
		return kRestoreTile
	}

	// Remove if nothing is left
	if dynamicMesh.PolyCount() == 0 {
		return kReplaceTile
	}

	// Project new vertices to detail meshes.
	ProjectNewVerticesToDetailMesh(dynamicMesh, detailMesh)
	dynamicMesh.FindNeighbors()

	// Clip the detail triangles of the original polygons to match each new polygon.
	var clipped []*ClippedDetailMesh
	clipped = make([]*ClippedDetailMesh, dynamicMesh.PolyCount())
	ClipDetailMeshes(clipped, dynamicMesh, detailMesh, tile, tileOffset, quantFactor)
	*tileData = DynamicMeshToTile(tileDataSize, dynamicMesh, clipped, tile, tileOffset)
	return kReplaceTile
}

func PatchMeshTilePointers(tile *NavMeshTile, data []byte, dataSize int32) bool {
	header := (*NavMeshDataHeader)(unsafe.Pointer(&(data[0])))
	tile.header = nil
	if header.magic != kNavMeshMagic {
		return false
	}
	if header.version != kNavMeshVersion {
		return false
	}

	tile.header = header

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

	return true
}

func HullPolygonIntersection(inside *Polygon, hull *Hull, temp *Polygon, quantFactor float32) {
	planeCount := len(*hull)

	for ic := 0; ic < planeCount; ic++ {
		plane := (*hull)[ic]
		result := SplitPoly(temp, *inside, plane, quantFactor, nil, 0)
		if result == 0 {
			inside.resize_uninitialized(len(*temp))
			copy(*inside, *temp)
		} else if result == 1 {
			inside.resize_uninitialized(0)
			return
		}
	}
}

type DetailNodeXSorter []DetailMeshBVNode

func (d DetailNodeXSorter) Len() int {
	return len(d)
}

func (d DetailNodeXSorter) Less(i, j int) bool {
	ra := d[i]
	rb := d[j]
	a := (ra.min.x + ra.max.x) * 0.5
	b := (rb.min.x + rb.max.x) * 0.5
	return a < b
}

func (d DetailNodeXSorter) Swap(i, j int) {
	d[i], d[j] = d[j], d[i]
}

type DetailNodeYSorter []DetailMeshBVNode

func (d DetailNodeYSorter) Len() int {
	return len(d)
}

func (d DetailNodeYSorter) Less(i, j int) bool {
	ra := d[i]
	rb := d[j]
	a := (ra.min.y + ra.max.y) * 0.5
	b := (rb.min.y + rb.max.y) * 0.5
	return a < b
}

func (d DetailNodeYSorter) Swap(i, j int) {
	d[i], d[j] = d[j], d[i]
}

type DetailNodeZSorter []DetailMeshBVNode

func (d DetailNodeZSorter) Len() int {
	return len(d)
}

func (d DetailNodeZSorter) Less(i, j int) bool {
	ra := d[i]
	rb := d[j]
	a := (ra.min.z + ra.max.z) * 0.5
	b := (rb.min.z + rb.max.z) * 0.5
	return a < b
}

func (d DetailNodeZSorter) Swap(i, j int) {
	d[i], d[j] = d[j], d[i]
}

func OverlapBoundsXZ(amin Vector3f, amax Vector3f,
	bmin Vector3f, bmax Vector3f) bool {
	if amin.x > bmax.x || amax.x < bmin.x {
		return false
	}
	if amin.z > bmax.z || amax.z < bmin.z {
		return false
	}
	return true
}

func DetailLongestAxis(v Vector3f) int32 {
	axis := int32(0)
	max := v.x
	if v.y > max {
		axis = 1
		max = v.y
	}
	if v.z > max {
		axis = 2
		max = v.z
	}
	return axis
}

func Subdivide(nodes *[]DetailMeshBVNode,
	items []DetailMeshBVNode,
	imin int32, imax int32) {
	inum := imax - imin
	*nodes = append(*nodes, DetailMeshBVNode{})
	icur := len(*nodes) - 1
	node := &(*nodes)[icur]

	// Update bounds
	node.min = items[imin].min
	node.max = items[imin].max
	for i := imin + 1; i < imax; i++ {
		node.min = MinVector3f(node.min, items[i].min)
		node.max = MaxVector3f(node.max, items[i].max)
	}

	if (imax - imin) <= 1 {
		// Leaf, copy triangles.
		node.idx = items[imin].idx
	} else {
		// Split remaining items along longest axis
		axis := DetailLongestAxis(node.max.Sub(node.min))
		if axis == 0 {
			sort.Sort(DetailNodeXSorter(items[imin:imax]))
		} else if axis == 1 {
			sort.Sort(DetailNodeYSorter(items[imin:imax]))
		} else {
			sort.Sort(DetailNodeZSorter(items[imin:imax]))
		}
		isplit := imin + inum/2

		// Left
		Subdivide(nodes, items, imin, isplit)
		// Right
		Subdivide(nodes, items, isplit, imax)
		iescape := (len(*nodes) - 1) - icur
		// Negative index means escape.
		(*nodes)[icur].idx = int32(-iescape) // 'node' ref may be invalid because of realloc.
	}
}

func BuildBVTree(nodes *[]DetailMeshBVNode,
	vertices []Vector3f,
	tris []uint16, triCount int32) bool {
	*nodes = (*nodes)[:0]

	// Build input items
	items := make([]DetailMeshBVNode, triCount)
	for i := int32(0); i < triCount; i++ {
		t := tris[i*4:]
		it := &items[i]
		it.idx = int32(i)
		// Calc triangle bounds.
		it.min = vertices[t[0]]
		it.max = vertices[t[0]]
		it.min = MinVector3f(it.min, vertices[t[1]])
		it.max = MaxVector3f(it.max, vertices[t[1]])
		it.min = MinVector3f(it.min, vertices[t[2]])
		it.max = MaxVector3f(it.max, vertices[t[2]])
	}

	Subdivide(nodes, items, 0, triCount)
	return true
}

type QueryDetailBVTreeCallback interface {
	process(detailMesh *DetailMesh, poly *DetailMeshPoly, tris []int32, triCount int32)
}

const BATCH_SIZE = 32

func QueryDetailBVTree(detailMesh *DetailMesh, poly *DetailMeshPoly,
	queryMin Vector3f, queryMax Vector3f,
	callback QueryDetailBVTreeCallback) {
	var batch [BATCH_SIZE]int32
	batchCount := int32(0)

	// Clip all detail triangles against the polygon.
	if poly.bvCount > 0 {
		nodes := detailMesh.bvNodes[poly.bvBase:]
		n := int32(0)
		for n < poly.bvCount {
			node := &nodes[n]
			overlap := OverlapBoundsXZ(queryMin, queryMax, node.min, node.max)
			isLeafNode := node.idx >= int32(0)
			if isLeafNode && overlap {
				if batchCount+1 > BATCH_SIZE {
					callback.process(detailMesh, poly, batch[:], batchCount)
					batchCount = 0
				}
				batch[batchCount] = poly.triBase + node.idx
				batchCount++
			}

			if overlap || isLeafNode {
				n++
			} else {
				escapeIndex := -node.idx
				n += escapeIndex
			}
		}
	} else {
		for j := int32(0); j < poly.triCount; j++ {
			if batchCount+1 > BATCH_SIZE {
				callback.process(detailMesh, poly, batch[:], batchCount)
				batchCount = 0
			}
			batch[batchCount] = poly.triBase + j
			batchCount++
		}
	}

	if batchCount > 0 {
		callback.process(detailMesh, poly, batch[:], batchCount)
		batchCount = 0
	}
}

func UnpackDetailMesh(detailMesh *DetailMesh, tile *NavMeshTile, tileOffset Vector3f) {
	// Unpack
	polyCount := tile.header.polyCount
	detailTriCount := tile.header.detailTriCount
	detailPolyCount := tile.header.detailMeshCount
	Assert(polyCount == detailPolyCount)
	detailMesh.triangles.resize_uninitialized(int(detailTriCount) * 4)
	detailMesh.polys.resize_uninitialized(int(detailPolyCount))
	bvTriCount := int32(0)
	maxTriCount := int32(0)
	kBVTreeThreshold := int32(6)
	for i := int32(0); i < polyCount; i++ {
		p := &tile.polys[i]
		pd := &tile.detailMeshes[i]
		poly := &detailMesh.polys[i]
		poly.bvBase = 0
		poly.bvCount = 0
		poly.vertBase = int32(len(detailMesh.vertices))
		poly.vertCount = int32(p.vertCount) + int32(pd.vertCount)
		for j := uint8(0); j < p.vertCount; j++ {
			detailMesh.vertices = append(detailMesh.vertices, tile.verts[p.verts[j]].Sub(tileOffset))
		}
		for j := uint32(0); j < uint32(pd.vertCount); j++ {
			detailMesh.vertices = append(detailMesh.vertices, tile.detailVerts[pd.vertBase+j].Sub(tileOffset))
		}

		poly.triBase = int32(pd.triBase)
		poly.triCount = int32(pd.triCount)
		for j := uint32(0); j < uint32(pd.triCount); j++ {
			t := tile.detailTris[(pd.triBase+j)*4:]
			detailMesh.triangles[(pd.triBase+j)*4+0] = uint16(t[0])
			detailMesh.triangles[(pd.triBase+j)*4+1] = uint16(t[1])
			detailMesh.triangles[(pd.triBase+j)*4+2] = uint16(t[2])
			detailMesh.triangles[(pd.triBase+j)*4+3] = uint16(t[3])
		}

		if poly.triCount > kBVTreeThreshold {
			bvTriCount += poly.triCount
			if maxTriCount < poly.triCount {
				maxTriCount = poly.triCount
			}
		}
	}

	if bvTriCount > 0 {
		// Build BV-tree for polys which have many detail triangles.
		var nodes []DetailMeshBVNode
		for i := int32(0); i < polyCount; i++ {
			poly := &detailMesh.polys[i]
			if poly.triCount > kBVTreeThreshold {
				BuildBVTree(&nodes, detailMesh.vertices[poly.vertBase:], detailMesh.triangles[poly.triBase*4:], poly.triCount)
				nodeCount := int32(len(nodes))
				if nodeCount > 0 {
					poly.bvBase = int32(len(detailMesh.bvNodes))
					poly.bvCount = nodeCount
					detailMesh.bvNodes.resize_uninitialized(poly.bvBase + nodeCount)
					for j := int32(0); j < nodeCount; j++ {
						detailMesh.bvNodes[poly.bvBase+j] = nodes[j]
					}
				}
			}
		}
	}
}

func ClosestHeightToTriangleEdge(height *float32, dmin *float32,
	samplePos, va, vb, vc Vector3f) {
	var d, t float32
	*dmin = math.MaxFloat32
	d = SqrDistancePointSegment2D(&t, samplePos, va, vb)
	if d < *dmin {
		*height = va.y + (vb.y-va.y)*t
		*dmin = d
	}
	d = SqrDistancePointSegment2D(&t, samplePos, vb, vc)
	if d < *dmin {
		*height = vb.y + (vc.y-vb.y)*t
		*dmin = d
	}
	d = SqrDistancePointSegment2D(&t, samplePos, vc, va)
	if d < *dmin {
		*height = vc.y + (va.y-vc.y)*t
		*dmin = d
	}
}

func PickDetailTriHeight(height *float32, dmin *float32,
	samplePos, va, vb, vc Vector3f) {
	var h float32
	if ClosestHeightPointTriangle(&h, samplePos, va, vb, vc) {
		*height = h
		*dmin = 0.0
	}
	if *dmin > 0.0 {
		var dist float32
		ClosestHeightToTriangleEdge(&h, &dist, samplePos, va, vb, vc)
		if dist < *dmin {
			*height = h
			*dmin = dist
		}
	}
}

type PickHeightCallback struct {
	samplePos    Vector3f
	height, dmin float32
}

func NewPickHeightCallback(pos Vector3f) *PickHeightCallback {
	return &PickHeightCallback{
		samplePos: pos,
		height:    pos.y,
		dmin:      math.MaxFloat32,
	}
}

func (this *PickHeightCallback) process(detailMesh *DetailMesh, poly *DetailMeshPoly, tris []int32, triCount int32) {
	for i := int32(0); i < triCount; i++ {
		t := detailMesh.triangles[tris[i]*4:]
		va := detailMesh.vertices[poly.vertBase+int32(t[0])]
		vb := detailMesh.vertices[poly.vertBase+int32(t[1])]
		vc := detailMesh.vertices[poly.vertBase+int32(t[2])]
		PickDetailTriHeight(&this.height, &this.dmin, this.samplePos, va, vb, vc)
	}
}

func PickDetailPolyHeight(detailMesh *DetailMesh, polyIdx int32, samplePos Vector3f) float32 {
	poly := &detailMesh.polys[polyIdx]
	sampleExt := NewVector3f(0.1, 0, 0.1)
	queryMin := samplePos.Sub(sampleExt)
	queryMax := samplePos.Add(sampleExt)
	callback := NewPickHeightCallback(samplePos)
	QueryDetailBVTree(detailMesh, poly, queryMin, queryMax, callback)
	return callback.height
}

func ProjectNewVerticesToDetailMesh(mesh *DynamicMesh, detailMesh *DetailMesh) {
	vertCount := mesh.VertCount()
	polyCount := mesh.PolyCount()
	vertexSourcePoly := make([]int32, vertCount)
	for i := range vertexSourcePoly {
		vertexSourcePoly[i] = -1
	}

	// Check which vertices have changed and store their original polygon too.
	// TODO: check if we need to store all source polys, now just projecting to the last one.
	for i := 0; i < polyCount; i++ {
		p := mesh.GetPoly(i)
		if p.m_Status != kOriginalPolygon {
			sourcePolyIndex := *mesh.GetData(i)
			for j := uint8(0); j < p.m_VertexCount; j++ {
				vertexSourcePoly[p.m_VertexIDs[j]] = int32(sourcePolyIndex)
			}
		}
	}

	for i := 0; i < vertCount; i++ {
		ip := vertexSourcePoly[i]
		if ip == -1 {
			continue
		}
		pos := mesh.GetVertex(i)
		pos.y = PickDetailPolyHeight(detailMesh, ip, pos)
		mesh.SetVertex(i, pos)
	}
}

func CalcPolyDetailBounds(bounds *MinMaxAABB, detailMesh *DetailMesh, ip int32) {
	poly := detailMesh.polys[ip]
	bounds.m_Min = detailMesh.vertices[poly.vertBase]
	bounds.m_Max = detailMesh.vertices[poly.vertBase]
	for i := int32(1); i < poly.vertCount; i++ {
		bounds.EncapsulateV(detailMesh.vertices[poly.vertBase+i])
	}
}

func HasBoundaryVertices(verts Vertex2Array, bmin Vector2f, bmax Vector2f) bool {
	if len(verts) == 0 {
		return false
	}

	var vmin, vmax Vector2f
	vmin.x = verts[0].x
	vmax.x = verts[0].x
	vmin.y = verts[0].y
	vmax.y = verts[0].y
	for i := 1; i < len(verts); i++ {
		vmin = MinVector2f(vmin, verts[i])
		vmax = MaxVector2f(vmax, verts[i])
	}

	dmin := vmin.Sub(bmin)
	if Sqr(dmin.x) < Sqr(MAGIC_EDGE_DISTANCE) {
		return true
	}
	if Sqr(dmin.y) < Sqr(MAGIC_EDGE_DISTANCE) {
		return true
	}

	dmax := vmax.Sub(bmax)
	if Sqr(dmax.x) < Sqr(MAGIC_EDGE_DISTANCE) {
		return true
	}
	if Sqr(dmax.y) < Sqr(MAGIC_EDGE_DISTANCE) {
		return true
	}
	return false
}

type ClipCallback struct {
	m_Hull        *Hull
	m_Inside      *Polygon
	m_Temp        *Polygon
	m_Footprint   *Vertex2Array
	m_QuantFactor float32
	m_Hit         bool
}

func NewClipCallback(hull *Hull, inside *Polygon, temp *Polygon, footPrint *Vertex2Array, quantFactor float32) *ClipCallback {
	return &ClipCallback{
		m_Hull:        hull,
		m_Inside:      inside,
		m_Temp:        temp,
		m_Footprint:   footPrint,
		m_QuantFactor: quantFactor,
		m_Hit:         false,
	}
}

func (this *ClipCallback) process(detailMesh *DetailMesh, poly *DetailMeshPoly, tris []int32, triCount int32) {
	for i := int32(0); i < triCount; i++ {
		t := detailMesh.triangles[tris[i]*4:]
		this.m_Inside.resize_uninitialized(3)
		(*this.m_Inside)[0] = detailMesh.vertices[poly.vertBase+int32(t[0])]
		(*this.m_Inside)[1] = detailMesh.vertices[poly.vertBase+int32(t[1])]
		(*this.m_Inside)[2] = detailMesh.vertices[poly.vertBase+int32(t[2])]
		HullPolygonIntersection(this.m_Inside, this.m_Hull, this.m_Temp, this.m_QuantFactor)
		if len(*this.m_Inside) == 0 {
			continue
		}

		for i := 0; i < len(*this.m_Inside); i++ {
			*this.m_Footprint = append(*this.m_Footprint, Vector2f{})
			v := &(*this.m_Footprint)[len(*this.m_Footprint)-1]
			v.x = (*this.m_Inside)[i].x
			v.y = (*this.m_Inside)[i].z
		}
		this.m_Hit = true
	}
}

func BuildDetailHulls(detailHulls *DetailHullContainer,
	hull Hull, bounds MinMaxAABB,
	detailMesh *DetailMesh, tile *NavMeshTile, tileOffset Vector3f, quantSize float32) bool {
	polyCount := tile.header.polyCount
	var inside Polygon
	var temp Polygon
	var footPrint Vertex2Array = make([]Vector2f, 0, 32)

	// Find polygons that potentially intersect with the cave hull.
	// We'll use detail mesh for this to capture all cases.
	// As we go we keep track of the polygons that were touched
	// as well as the vertices of the detail mesh intersection.
	// These intersection points will later be used to create a new infinite
	// carver which is actually used for carving.

	kTouched := byte(1)
	kVisited := byte(2)
	visited := make([]byte, polyCount)

	// TODO: we should be able to use BV-tree for this.
	nTouched := 0
	for ip := int32(0); ip < polyCount; ip++ {
		var polyBounds MinMaxAABB
		CalcPolyDetailBounds(&polyBounds, detailMesh, ip)
		if !IntersectAABBAABB(bounds, polyBounds) {
			continue
		}
		visited[ip] = kTouched
		nTouched++
	}

	var stack []int32

	// Merge connecting regions.
	for ip := int32(0); ip < polyCount; ip++ {
		if visited[ip] != kTouched {
			continue
		}

		var detailHull DetailHull
		stack = stack[:0]
		stack = append(stack, ip)

		for len(stack) != 0 {
			curLen := len(stack)
			cur := stack[curLen-1]
			stack = stack[:curLen-1]
			detailHull.polysIds = append(detailHull.polysIds, int(cur))
			poly := &tile.polys[cur]
			for j := uint8(0); j < poly.vertCount; j++ {
				// Skip if no neighbour or if at tile border.
				if poly.neis[j] == uint16(0) || poly.neis[j]&uint16(0x8000) != 0 {
					continue
				}
				nei := poly.neis[j] - 1
				if visited[nei] == kTouched {
					visited[nei] = kVisited
					stack = append(stack, int32(nei))
				}
			}
		}
		*detailHulls = append(*detailHulls, detailHull)
	}

	if len(*detailHulls) == 0 {
		return false
	}

	var convexHull Vertex2Array

	detailHullCount := len(*detailHulls)
	for hi := 0; hi < detailHullCount; hi++ {
		detailHull := &(*detailHulls)[hi]
		footPrint = footPrint[:0]

		for i := 0; i < len(detailHull.polysIds); i++ {
			ip := detailHull.polysIds[i]
			dpoly := &detailMesh.polys[ip]
			callback := NewClipCallback(&hull, &inside, &temp, &footPrint, quantSize)
			QueryDetailBVTree(detailMesh, dpoly, bounds.m_Min, bounds.m_Max, callback)
			if !callback.m_Hit {
				polyIdLen := len(detailHull.polysIds)
				detailHull.polysIds[i] = detailHull.polysIds[polyIdLen-1]
				detailHull.polysIds = detailHull.polysIds[:polyIdLen-1]
				i--
			}
		}

		// TODO: Optimization, if all the potentially intersecting polygons are flat, we could
		// just use the original hull.

		// Build carve hull from a convex hull of footprint.
		if len(footPrint) == 0 {
			detailHull.polysIds = detailHull.polysIds[:0]
			continue
		}

		CalculateConvexHull(&convexHull, &footPrint)

		// Avoid simplifying the hull if it touches the tile boundary.
		tileOffset2 := NewVector2f(tileOffset.x, tileOffset.z)
		bmin := NewVector2f(tile.header.bmin.x, tile.header.bmin.z).Sub(tileOffset2)
		bmax := NewVector2f(tile.header.bmax.x, tile.header.bmax.z).Sub(tileOffset2)
		if !HasBoundaryVertices(convexHull, bmin, bmax) {
			SimplifyPolyline(&convexHull, quantSize)
		}

		if len(convexHull) < 3 {
			detailHull.polysIds = detailHull.polysIds[:0]
			continue
		}

		// Create hull planes from the polygon
		detailHull.hull = detailHull.hull[:0]
		convexHullCount := len(convexHull)
		for i := 0; i < convexHullCount; i++ {
			position2 := convexHull[i]
			dir2 := convexHull[NextIndex(int32(i), int32(convexHullCount))].Sub(position2)
			len2 := Magnitude2(dir2)
			if len2 <= kEpsilon {
				continue
			}

			dir2 = dir2.Div(len2)
			position := NewVector3f(position2.x, 0, position2.y)
			normal := NewVector3f(-dir2.y, 0, dir2.x)
			detailHull.hull.emplace_back_uninitialized().SetNormalAndPosition(normal, position)
		}
	}

	return true
}

func HullFromPoly(hull *Hull, poly []Vector3f) {
	vertCount := len(poly)
	*hull = make([]Plane, vertCount)
	for i := 0; i < vertCount; i++ {
		position := poly[i]
		dir := poly[NextIndex(int32(i), int32(vertCount))].Sub(position)
		normal := NewVector3f(-dir.z, 0, dir.x)
		normal = NormalizeSafe(normal, NewVector3f(0, 0, 0))
		(*hull)[i].SetNormalAndPosition(normal, position)
	}
}

type ClipDetailMeshCallback struct {
	dmesh       *ClippedDetailMesh
	hull        *Hull
	welder      *VertexWelder //64
	inside      *Polygon
	temp        *Polygon
	quantFactor float32
}

func NewDetailMeshClipCallback(dmeshIn *ClippedDetailMesh, hullIn *Hull, welderIn *VertexWelder,
	insideIn *Polygon, tempIn *Polygon, quantFactorIn float32) *ClipDetailMeshCallback {
	return &ClipDetailMeshCallback{
		dmesh:       dmeshIn,
		hull:        hullIn,
		welder:      welderIn,
		inside:      insideIn,
		temp:        tempIn,
		quantFactor: quantFactorIn,
	}
}

const MAGIC_EDGE_DISTANCE = 1e-2

func (this *ClipDetailMeshCallback) process(detailMesh *DetailMesh, poly *DetailMeshPoly, tris []int32, triCount int32) {
	for i := int32(0); i < triCount; i++ {
		t := detailMesh.triangles[tris[i]*4:]
		this.inside.resize_uninitialized(3)
		(*this.inside)[0] = detailMesh.vertices[poly.vertBase+int32(t[0])]
		(*this.inside)[1] = detailMesh.vertices[poly.vertBase+int32(t[1])]
		(*this.inside)[2] = detailMesh.vertices[poly.vertBase+int32(t[2])]
		HullPolygonIntersection(this.inside, this.hull, this.temp, this.quantFactor)
		vertexCount := len(*this.inside)
		if vertexCount < 3 {
			continue
		}

		v0 := this.welder.AddUnique((*this.inside)[0])
		v1 := this.welder.AddUnique((*this.inside)[1])
		for i := 2; i < vertexCount; i++ {
			v2 := this.welder.AddUnique((*this.inside)[i])
			triArea2 := TriArea2D((*this.inside)[0], (*this.inside)[i-1], (*this.inside)[i])
			if triArea2 < MAGIC_EDGE_DISTANCE*MAGIC_EDGE_DISTANCE {
				v1 = v2
				continue
			}

			if v0 != v1 && v1 != v2 && v2 != v0 {
				this.dmesh.triangles = append(this.dmesh.triangles, uint16(v0))
				this.dmesh.triangles = append(this.dmesh.triangles, uint16(v1))
				this.dmesh.triangles = append(this.dmesh.triangles, uint16(v2))
			}
			v1 = v2
		}
	}
}

func ClipDetailMeshes(clipped []*ClippedDetailMesh,
	mesh *DynamicMesh, detailMesh *DetailMesh,
	tile *NavMeshTile,
	tileOffset Vector3f,
	quantFactor float32) {

	queryPadding := NewVector3f(quantFactor*2.0, 0, quantFactor*2.0)
	polyCount := mesh.PolyCount()
	var hull Hull = make([]Plane, 0, 8)
	verts := make([]Vector3f, 0, 8)
	var inside Polygon = make([]Vector3f, 0, 32)
	var temp Polygon = make([]Vector3f, 0, 32)
	welder := NewVertexWelder(64, nil, quantFactor)
	for i := 0; i < polyCount; i++ {
		p := mesh.GetPoly(i)
		// Process only new polygons
		if p.m_Status == kOriginalPolygon {
			continue
		}
		ip := *mesh.GetData(i)
		dpoly := &detailMesh.polys[ip]

		// If the detail mesh does not have any extra vertices,
		// no need to clip, just retriangulate later.
		if dpoly.vertCount == int32(p.m_VertexCount) {
			continue
		}

		// Build clip hull from the polygons
		verts = make([]Vector3f, p.m_VertexCount)
		for j := uint8(0); j < p.m_VertexCount; j++ {
			verts[j] = mesh.GetVertex(int(p.m_VertexIDs[j]))
		}
		HullFromPoly(&hull, verts)

		// Build query box from the polygon.
		var queryMin, queryMax Vector3f
		queryMin = verts[0]
		queryMax = verts[0]
		vertsCount := len(verts)
		for j := 0; j < vertsCount; j++ {
			queryMin = MinVector3f(queryMin, verts[j])
			queryMax = MaxVector3f(queryMax, verts[j])
		}
		queryMin = queryMin.Sub(queryPadding)
		queryMax = queryMax.Add(queryPadding)
		clipped[i] = NewClippedDetailMesh()
		dmesh := clipped[i]
		dmesh.polyIndex = i
		welder.SetVertexArray(&dmesh.vertices)
		welder.Reset()

		// Clip all detail triangles against the polygon.
		callback := NewDetailMeshClipCallback(dmesh, &hull, welder, &inside, &temp, quantFactor)
		QueryDetailBVTree(detailMesh, dpoly, queryMin, queryMax, callback)

		// Offset dmesh back to tile location.
		vertCount := len(dmesh.vertices)
		for j := 0; j < vertCount; j++ {
			dmesh.vertices[j] = dmesh.vertices[j].Add(tileOffset)
		}

		if len(dmesh.vertices) < 3 || len(dmesh.triangles) < 3 {
			clipped[i] = nil
		}
	}
}

func AreColinear(u, v Vector3f, cosAngleAccept float32) bool {
	return FloatAbs(DotVector3f(v, u)) > cosAngleAccept
}

func DistancePointSegmentSqr(pt, s1, s2 Vector2f) float32 {
	ds := s2.Sub(s1)
	dp := pt.Sub(s1)
	den := DotVector2f(ds, ds)
	if den == 0 {
		return DotVector2f(dp, dp)
	}
	t := DotVector2f(ds, dp) / den
	t = FloatClamp(t, 0, 1)
	diff := ds.Mulf(t).Sub(dp)
	return DotVector2f(diff, diff)
}

func SimplifyPolyline(hull *Vertex2Array, thr float32) {
	i := 0
	count := len(*hull)
	for i < count && count > 2 {
		pa := (*hull)[PrevIndex(int32(i), int32(count))]
		pb := (*hull)[i]
		pc := (*hull)[NextIndex(int32(i), int32(count))]
		if DistancePointSegmentSqr(pb, pa, pc) < thr*thr {
			hull.erase(i)
			count--
		} else {
			i++
		}
	}
}

func OffsetPolygon(dest *Vertex2Array, poly Vertex2Array, offset float32) {
	count := int32(len(poly))
	*dest = make([]Vector2f, 0, count)
	for i := int32(0); i < count; i++ {
		curr := poly[i]
		prev := poly[PrevIndex(i, count)]
		next := poly[NextIndex(i, count)]
		diffa := NormalizeSafe2(curr.Sub(prev), NewVector2f(0, 0))
		diffb := NormalizeSafe2(next.Sub(curr), NewVector2f(0, 0))

		// Calculate offset vectors based on neighbor segment directions.
		// Scale the offsets to maintain constant line width.
		dla := NewVector2f(-diffa.y, diffa.x)
		dlb := NewVector2f(-diffb.y, diffb.x)

		// More than 90.1 degree turn, add 2 points.
		// Use slack so that about 90 degree corners won't get beveled (common case with box obstacle).
		dot := DotVector2f(diffa, diffb)
		kCos90p1 := float32(-0.00174542)
		if dot < kCos90p1 {
			// This is poorman's approximation of a bevel which is offset
			// so that it approximates a circle. Consider 2 cases below.
			//   B._____   A._____.B
			// A.´          |     |
			//  |  x----    |  x  |
			//  |  |        |  |  |
			// A correct version likely needs asin () & co, not used for speed reasons.
			pos := curr.Add(diffa.Mulf(0.25 + FloatAbs(dot)*0.75*offset))
			*dest = append(*dest, pos.Add(dla.Mulf(offset)))
			*dest = append(*dest, pos.Add(dlb.Mulf(offset)))
		} else {
			dm := dla.Add(dlb).Mulf(0.5)
			dmr2 := DotVector2f(dm, dm)
			if dmr2 > 0.0 {
				dm = dm.Mulf(1.0 / dmr2)
			}
			*dest = append(*dest, curr.Add(dm.Mulf(offset)))
		}
	}
}

func CalculateCarveHullFromPoints(carveHull *Hull, localHullBounds *MinMaxAABB, shape *NavMeshCarveShape,
	tileOffset Vector3f, carveDepth float32, carveWidth float32,
	points []Vector3f, pointCount int32) bool {
	// Calculate convex hull of the obstacle on XZ plane
	*carveHull = make([]Plane, 0)
	var projectedPoints Vertex2Array
	var hull Vertex2Array
	var hullOffset Vertex2Array
	projectedPoints = make([]Vector2f, pointCount)
	for i := int32(0); i < pointCount; i++ {
		projectedPoints[i] = NewVector2f(points[i].x, points[i].z)
	}

	CalculateConvexHull(&hull, &projectedPoints)
	SimplifyPolyline(&hull, carveWidth*0.1)
	OffsetPolygon(&hullOffset, hull, carveWidth)

	// Bail out if hull has been degenerated.
	// It is possible that SimplifyPolyline will simplify the obstacle down to a line,
	// OffsetPolygon can take care of that. This is should only happen when the obstacle
	// degenerates to a point.
	if len(hullOffset) < 3 {
		return false
	}

	*localHullBounds = shape.bounds
	localHullBounds.m_Min = localHullBounds.m_Min.Sub(tileOffset)
	localHullBounds.m_Max = localHullBounds.m_Max.Sub(tileOffset)
	hullCount := int32(len(hullOffset))
	for i := int32(0); i < hullCount; i++ {
		pt := hullOffset[i]
		dir := hullOffset[NextIndex(i, hullCount)].Sub(pt)
		position := NewVector3f(pt.x, 0, pt.y)
		normal := NewVector3f(-dir.y, 0, dir.x)
		normal = NormalizeSafe(normal, NewVector3f(0, 0, 0))
		*carveHull = append(*carveHull, Plane{})
		plane := &(*carveHull)[len(*carveHull)-1]
		plane.SetNormalAndPosition(normal, position)
		localHullBounds.m_Min.x = FloatMin(localHullBounds.m_Min.x, pt.x)
		localHullBounds.m_Min.z = FloatMin(localHullBounds.m_Min.z, pt.y)
		localHullBounds.m_Max.x = FloatMax(localHullBounds.m_Max.x, pt.x)
		localHullBounds.m_Max.z = FloatMax(localHullBounds.m_Max.z, pt.y)
	}

	// Calculate approximate up axis.
	// The approx normal is chosen so that it is visually plausible, i.e. prefer larger extents.
	zero := NewVector3f(0.0, 0.0, 0.0)
	yAxis := zero
	yAxis = yAxis.Add(shape.xAxis.Mulf(shape.xAxis.y * FloatMax(shape.extents.y, shape.extents.z)))
	yAxis = yAxis.Add(shape.yAxis.Mulf(shape.yAxis.y * FloatMax(shape.extents.z, shape.extents.x)))
	yAxis = yAxis.Add(shape.zAxis.Mulf(shape.zAxis.y * FloatMax(shape.extents.x, shape.extents.y)))
	yAxis = NormalizeSafe(yAxis, zero)
	worldYAxis := NewVector3f(0.0, 1.0, 0.0)
	if CompareApproximately(yAxis, zero, kEpsilon) {
		yAxis = worldYAxis
	}

	distMin := float32(math.MaxFloat32)
	distMax := float32(-math.MaxFloat32)
	for i := int32(0); i < pointCount; i++ {
		dist := DotVector3f(points[i], yAxis)
		distMin = FloatMin(distMin, dist)
		distMax = FloatMax(distMax, dist)
	}

	// Add top/bottom caps
	*carveHull = append(*carveHull, NewPlane(yAxis.Mulf(-1), yAxis.Mulf(distMin-carveDepth)))
	*carveHull = append(*carveHull, NewPlane(yAxis, yAxis.Mulf(distMax)))

	// The aabb top/bottom planes if needed
	cosAngleConsiderAxisAligned := float32(0.984807753012208) // Consider colinear if within 10 degrees
	isAlmostAxisAlignedY := AreColinear(yAxis, worldYAxis, cosAngleConsiderAxisAligned)
	if !isAlmostAxisAlignedY {
		min := shape.bounds.m_Min.Sub(tileOffset)
		max := shape.bounds.m_Max.Sub(tileOffset)
		*carveHull = append(*carveHull, NewPlane(worldYAxis.Mulf(-1), min.Sub(NewVector3f(0, 1, 0).Mulf(carveDepth))))
		*carveHull = append(*carveHull, NewPlane(worldYAxis, max))
	} else {
		diagonal2D := localHullBounds.m_Max.Sub(localHullBounds.m_Min)
		diagonal2D.y = 0
		halfSpread := 0.5 * Magnitude(diagonal2D)
		tanAngleConsiderAxisAligned := float32(0.176326980708464) // Cap slanted up to 10 degrees
		maxCapDepth := tanAngleConsiderAxisAligned * halfSpread
		localHullBounds.m_Min.y -= maxCapDepth
		localHullBounds.m_Max.y += maxCapDepth
	}

	localHullBounds.m_Min.y -= carveDepth
	return true
}

// Compute the set of planes defining an extruded bounding box.
// Bounding box is represented by transform and size.
// Extrusion is based on 'carveWidth' horizontally and 'carveDepth' vertically down.
// Everything is translated relative to 'tileOffset'.
func CalculateBoxHull(
	carveHull *Hull, localHullBounds *MinMaxAABB,
	shape *NavMeshCarveShape, tileOffset Vector3f,
	carveDepth, carveWidth float32) bool {
	// Calculate obstacle vertices.
	var box [8]Vector3f
	for i := 0; i < 8; i++ {
		box[i] = shape.center.Sub(tileOffset)
		x := shape.extents.x
		if i&1 == 0 {
			x = -shape.extents.x
		}
		box[i] = box[i].Add(shape.xAxis.Mulf(x))
		y := shape.extents.y
		if i&2 == 0 {
			y = -shape.extents.y
		}
		box[i] = box[i].Add(shape.yAxis.Mulf(y))
		z := shape.extents.z
		if i&4 == 0 {
			z = -shape.extents.z
		}
		box[i] = box[i].Add(shape.zAxis.Mulf(z))
	}

	return CalculateCarveHullFromPoints(carveHull, localHullBounds, shape, tileOffset, carveDepth, carveWidth, box[:], 8)
}

const kDivs = 8

func CalculateCapsuleHull(
	carveHull *Hull, localHullBounds *MinMaxAABB,
	shape *NavMeshCarveShape, tileOffset Vector3f,
	carveDepth, carveWidth float32) bool {
	// TODO: it should be possible to optimize the hull shape a bit more by
	// creating the capsule data in 2D, and add min/max points.
	// See how a 2D capsule is drawn in NavMeshVisulization.cpp

	var radius float32 = 0
	var height float32 = 0
	FitCapsuleToExtents(&radius, &height, shape.extents)

	// Calculate obstacle vertices.
	var cylinder [(kDivs + kDivs + 1) * 2]Vector3f
	{
	}
	var n int32 = 0
	// Scale for "outer" polygon, the polygon is created so that the cylinder circle is inscribing the polygon.
	radiusScale := float32(1.0 / math.Cos(math.Pi*2.0/float64(kDivs)*0.5))
	// We have 8 divs effectively in other direction too.
	h := float32(0.7071067812) * radius * radiusScale
	r := radius * radiusScale
	center := shape.center.Sub(tileOffset)
	for i := 0; i < kDivs; i++ {
		angle := float64(i) / float64(kDivs) * math.Pi * 2.0
		dx := math.Cos(angle)
		dz := math.Sin(angle)
		ax := shape.xAxis.Mulf(float32(dx))
		az := shape.zAxis.Mulf(float32(dz))
		cylinder[n] = center.Add(ax.Mulf(r)).Add(az.Mulf(r)).Sub(shape.yAxis.Mulf(height))
		n++
		cylinder[n] = center.Add(ax.Mulf(r)).Add(az.Mulf(r)).Add(shape.yAxis.Mulf(height))
		n++
		cylinder[n] = center.Add(ax.Mulf(h)).Add(az.Mulf(h)).Sub(shape.yAxis.Mulf(height + h))
		n++
		cylinder[n] = center.Add(ax.Mulf(h)).Add(az.Mulf(h)).Add(shape.yAxis.Mulf(height + h))
		n++
	}

	// Capsule tips
	cylinder[n] = center.Sub(shape.yAxis.Mulf(height + r))
	n++
	cylinder[n] = center.Add(shape.yAxis.Mulf(height + r))
	n++

	return CalculateCarveHullFromPoints(carveHull, localHullBounds, shape, tileOffset, carveDepth, carveWidth, cylinder[:], n)
}

// Set flags on polygon edges colinear to tile edges.
// Flagged edges are considered when dynamically stitching neighboring tiles.
func WritePortalFlags(verts []Vector3f, polys []NavMeshPoly, polyCount int32, sourceHeader *NavMeshDataHeader) {
	bmax := sourceHeader.bmax
	bmin := sourceHeader.bmin
	for ip := int32(0); ip < polyCount; ip++ {
		poly := &polys[ip]
		for iv := uint8(0); iv < poly.vertCount; iv++ {
			// Skip already connected edges
			if poly.neis[iv] != 0 {
				continue
			}

			vert := verts[poly.verts[iv]]
			ivn := iv + 1
			if iv+1 == poly.vertCount {
				ivn = 0
			}
			nextVert := verts[poly.verts[ivn]]

			//
			//       z+
			//    o---->o
			//    ^     |
			// x- |     | x+
			//    |     v
			//    o<----o
			//       z-

			dx := nextVert.x - vert.x
			dz := nextVert.z - vert.z
			nei := uint16(0)
			if dz < 0.0 && FloatMax(FloatAbs(vert.x-bmax.x), FloatAbs(nextVert.x-bmax.x)) < MAGIC_EDGE_DISTANCE {
				nei = kNavMeshExtLink | 0 // x+ portal
			} else if dx > 0.0 && FloatMax(FloatAbs(vert.z-bmax.z), FloatAbs(nextVert.z-bmax.z)) < MAGIC_EDGE_DISTANCE {
				nei = kNavMeshExtLink | 2 // z+ portal
			} else if dz > 0.0 && FloatMax(FloatAbs(vert.x-bmin.x), FloatAbs(nextVert.x-bmin.x)) < MAGIC_EDGE_DISTANCE {
				nei = kNavMeshExtLink | 4 // x- portal
			} else if dx < 0.0 && FloatMax(FloatAbs(vert.z-bmin.z), FloatAbs(nextVert.z-bmin.z)) < MAGIC_EDGE_DISTANCE {
				nei = kNavMeshExtLink | 6 // z- portal
			}
			poly.neis[iv] = nei
		}
	}
}

func SimplePolygonTriangulation(dtl *NavMeshPolyDetail, dtris []NavMeshPolyDetailIndex, detailTriBase int32, polygonVertexCount int32) int32 {
	dtl.vertBase = 0
	dtl.vertCount = 0
	dtl.triBase = uint32(detailTriBase)
	dtl.triCount = NavMeshPolyDetailIndex(polygonVertexCount - 2)

	// Triangulate polygon (local indices).
	for j := int32(2); j < polygonVertexCount; j++ {
		t := dtris[4*detailTriBase:]
		t[0] = 0
		t[1] = NavMeshPolyDetailIndex(j - 1)
		t[2] = NavMeshPolyDetailIndex(j)
		// Bit for each edge that belongs to poly boundary.
		t[3] = 1 << 2
		if j == 2 {
			t[3] |= 1 << 0
		}
		if j == polygonVertexCount-1 {
			t[3] |= 1 << 4
		}
		detailTriBase++
	}
	return detailTriBase
}

func GetEdgeFlags(va Vector3f, poly []Vector3f) byte {
	// Return mask indicating which edges the vertex touches.
	thrSqr := Sqr(MAGIC_EDGE_DISTANCE)
	npoly := len(poly)
	var flags byte = 0
	for i, j := 0, npoly-1; i < npoly; i, j = i+1, i {
		var t float32
		if SqrDistancePointSegment2D(&t, va, poly[j], poly[i]) < thrSqr {
			flags |= 1 << uint8(j)
		}
	}
	return flags
}

func GetTriFlags(va, vb, vc byte) byte {
	var flags byte = 0
	if (va & vb) != 0 {
		flags |= 1 << 0
	}
	if (vb & vc) != 0 {
		flags |= 1 << 2
	}
	if (vc & va) != 0 {
		flags |= 1 << 4
	}
	return flags
}

func TileToDynamicMesh(tile *NavMeshTile, mesh *DynamicMesh, tileOffset Vector3f) {

	vertCount := tile.header.vertCount
	polyCount := tile.header.polyCount
	mesh.Reserve(vertCount, polyCount)
	for iv := int32(0); iv < vertCount; iv++ {
		mesh.AddVertex(tile.verts[iv].Sub(tileOffset))
	}

	for ip := int32(0); ip < polyCount; ip++ {
		srcPoly := tile.polys[ip]
		mesh.AddPolygon3(srcPoly.verts[:], DataType(ip), int32(srcPoly.vertCount))
	}
}

func DynamicMeshToTile(dataSize *uint32, mesh *DynamicMesh, clipped []*ClippedDetailMesh,
	sourceTile *NavMeshTile, tileOffset Vector3f) []byte {

	// Determine data size
	vertCount := mesh.VertCount()
	polyCount := mesh.PolyCount()
	sourceHeader := sourceTile.header
	totVertCount := vertCount
	totPolyCount := polyCount
	detailVertCount := int32(0)
	detailTriCount := int32(0)
	RequirementsForDetailMeshMixed(&detailVertCount, &detailTriCount, mesh, sourceTile, clipped)
	headSize := Align4(unsafe.Sizeof(NavMeshDataHeader{}))
	vertSize := Align4(uintptr(totVertCount) * unsafe.Sizeof(Vector3f{}))
	polySize := Align4(uintptr(totPolyCount) * unsafe.Sizeof(NavMeshPoly{}))
	detailMeshesSize := Align4(uintptr(polyCount) * unsafe.Sizeof(NavMeshPolyDetail{}))
	detailVertsSize := Align4(uintptr(detailVertCount) * unsafe.Sizeof(Vector3f{}))
	detailTrisSize := Align4(uintptr(detailTriCount) * 4 * unsafe.Sizeof(NavMeshPolyDetailIndex(0)))
	bvTreeSize := uint32(0)
	newSize := headSize + vertSize + polySize +
		detailTrisSize + detailVertsSize + detailMeshesSize + bvTreeSize
	newTile := make([]byte, newSize+1)
	if newTile == nil {
		*dataSize = 0
		return nil
	}
	*dataSize = newSize

	// Serialize in the detour recognized format
	header := (*NavMeshDataHeader)(unsafe.Pointer(&(newTile[0])))
	d := headSize
	var verts []Vector3f
	sliceHeader := (*reflect.SliceHeader)(unsafe.Pointer(&(verts)))
	sliceHeader.Cap = int(totVertCount)
	sliceHeader.Len = int(totVertCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(newTile[d])))
	d += vertSize

	var polys []NavMeshPoly
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(polys)))
	sliceHeader.Cap = int(totPolyCount)
	sliceHeader.Len = int(totPolyCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(newTile[d])))
	d += polySize

	var detail []NavMeshPolyDetail
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(detail)))
	sliceHeader.Cap = int(polyCount)
	sliceHeader.Len = int(polyCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(newTile[d])))
	d += detailMeshesSize

	var dverts []Vector3f
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(dverts)))
	sliceHeader.Cap = int(detailVertCount)
	sliceHeader.Len = int(detailVertCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(newTile[d])))
	d += detailVertsSize

	var dtris []NavMeshPolyDetailIndex
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(dtris)))
	sliceHeader.Cap = int(detailTriCount * 4)
	sliceHeader.Len = int(detailTriCount * 4)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(newTile[d])))
	d += detailTrisSize

	/*
		var bvTree []NavMeshBVNode
		sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(bvTree)))
		sliceHeader.Cap = int(header.bvNodeCount)
		sliceHeader.Len = int(header.bvNodeCount)
		sliceHeader.Data = uintptr(unsafe.Pointer(&(newTile[d])))
	*/
	d += bvTreeSize
	Assert(d == newSize)
	for iv := 0; iv < vertCount; iv++ {
		// TODO: apply tile offset earlier, after carving. Now needs to be handled all over the place.
		verts[iv] = mesh.GetVertex(iv).Add(tileOffset)
	}

	for ip := 0; ip < polyCount; ip++ {
		p := mesh.GetPoly(ip)
		sourcePolyIndex := *mesh.GetData(ip)
		srcPoly := sourceTile.polys[sourcePolyIndex]
		poly := &polys[ip]
		copy(poly.verts[:], p.m_VertexIDs[:])
		copy(poly.neis[:], p.m_Neighbours[:])
		area := srcPoly.area
		poly.flags = 1 << area
		poly.area = area
		poly.vertCount = p.m_VertexCount
	}

	// Set external portal flags
	WritePortalFlags(verts, polys, int32(polyCount), sourceHeader)
	WriteDetailMeshMixed(detail, dverts, dtris, mesh, sourceTile, tileOffset, clipped,
		int32(detailTriCount), int32(detailVertCount))

	// Copy values from source
	*header = *sourceHeader

	// (re)set new tile values
	header.polyCount = int32(totPolyCount)
	header.vertCount = int32(totVertCount)
	header.detailMeshCount = int32(polyCount)
	header.detailVertCount = int32(detailVertCount)
	header.detailTriCount = int32(detailTriCount)
	header.bvNodeCount = 0 // Fixme: bv-tree

	return newTile
}

func RequirementsForDetailMeshMixed(detailVertCount *int32, detailTriCount *int32,
	mesh *DynamicMesh, sourceTile *NavMeshTile, clipped []*ClippedDetailMesh) {
	vertCount := NavMeshPolyDetailIndex(0)
	triCount := NavMeshPolyDetailIndex(0)

	// Collect sizes needed for detail mesh
	polyCount := mesh.PolyCount()
	for ip := 0; ip < polyCount; ip++ {
		p := mesh.GetPoly(ip)
		sourcePolyIndex := *mesh.GetData(ip)
		if p.m_Status == kOriginalPolygon {
			// When preserving polygon detail mesh just add the source counts
			sourceDetail := sourceTile.detailMeshes[sourcePolyIndex]
			vertCount += sourceDetail.vertCount
			triCount += sourceDetail.triCount
		} else {
			if clipped[ip] != nil {
				vertCount += NavMeshPolyDetailIndex(len(clipped[ip].vertices))
				triCount += NavMeshPolyDetailIndex(len(clipped[ip].triangles)) / 3
			} else {
				// Simple triangulation needs n-2 triangles but no extra detail vertices
				triCount += NavMeshPolyDetailIndex(p.m_VertexCount) - 2
			}
		}
	}
	*detailVertCount = int32(vertCount)
	*detailTriCount = int32(triCount)
}

func WriteDetailMeshMixed(detail []NavMeshPolyDetail, dverts []Vector3f, dtris []NavMeshPolyDetailIndex,
	mesh *DynamicMesh, sourceTile *NavMeshTile, tileOffset Vector3f,
	clipped []*ClippedDetailMesh, detailTriCount int32, detailVertCount int32) {

	detailVertBase := uint32(0)
	detailTriBase := uint32(0)
	var edgeFlags []byte
	var poly []Vector3f

	polyCount := mesh.PolyCount()
	for ip := 0; ip < polyCount; ip++ {
		dtl := &detail[ip]
		p := mesh.GetPoly(ip)
		if p.m_Status == kOriginalPolygon {
			// Fill in the original detail mesh for this polygon
			sourcePolyIndex := *mesh.GetData(ip)
			sourceDetail := sourceTile.detailMeshes[sourcePolyIndex]
			dtl.vertBase = detailVertBase
			dtl.vertCount = sourceDetail.vertCount
			dtl.triBase = detailTriBase
			dtl.triCount = sourceDetail.triCount

			// copy source detail vertices and triangles
			size := uint32(sourceDetail.vertCount)
			copy(dverts[detailVertBase:detailVertBase+size], sourceTile.detailVerts[sourceDetail.vertBase:sourceDetail.vertBase+size])
			size = uint32(4 * uintptr(sourceDetail.triCount))
			copy(dtris[4*detailTriBase:4*detailTriBase+size], sourceTile.detailTris[4*sourceDetail.triBase:4*sourceDetail.triBase+size])

			detailVertBase += uint32(sourceDetail.vertCount)
			detailTriBase += uint32(sourceDetail.triCount)
		} else {
			if clipped[ip] != nil {
				poly = make([]Vector3f, p.m_VertexCount)
				for j := uint8(0); j < p.m_VertexCount; j++ {
					poly[j] = tileOffset.Add(mesh.GetVertex(int(p.m_VertexIDs[j])))
				}

				// TODO: check vertex count so that detail vertex index won't overflow.
				// TODO: locate and remap polygon vertices to reduce space (now stores poly vertices too).
				clip := clipped[ip]
				vertCount := len(clip.vertices)
				triCount := len(clip.triangles) / 3
				dtl.vertBase = detailVertBase
				dtl.vertCount = NavMeshPolyDetailIndex(vertCount)
				dtl.triBase = detailTriBase
				dtl.triCount = NavMeshPolyDetailIndex(triCount)

				// Copy vertices
				for j := uint32(0); j < uint32(vertCount); j++ {
					dverts[detailVertBase+j] = clip.vertices[j]
				}

				// Calculate edge flags.
				edgeFlags = make([]byte, vertCount)
				for j := 0; j < vertCount; j++ {
					edgeFlags[j] = GetEdgeFlags(clip.vertices[j], poly)
				}

				// Copy triangles.
				for j := uint32(0); j < uint32(triCount); j++ {
					t := dtris[4*(detailTriBase+j):]
					t[0] = NavMeshPolyDetailIndex(uint16(p.m_VertexCount) + clip.triangles[j*3+0])
					t[1] = NavMeshPolyDetailIndex(uint16(p.m_VertexCount) + clip.triangles[j*3+1])
					t[2] = NavMeshPolyDetailIndex(uint16(p.m_VertexCount) + clip.triangles[j*3+2])
					t[3] = NavMeshPolyDetailIndex(GetTriFlags(edgeFlags[clip.triangles[j*3+0]],
						edgeFlags[clip.triangles[j*3+1]],
						edgeFlags[clip.triangles[j*3+2]]))
				}

				detailVertBase += uint32(vertCount)
				detailTriBase += uint32(triCount)
			} else {
				detailTriBase = uint32(SimplePolygonTriangulation(dtl, dtris, int32(detailTriBase), int32(p.m_VertexCount)))
			}
		}
	}
	Assert(detailTriBase == uint32(detailTriCount))
	Assert(detailVertBase == uint32(detailVertCount))
}

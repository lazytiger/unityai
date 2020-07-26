package unityai

const MAX_OUTPUT_VERTICES = 32
const PLANE_FLAG byte = 0x80

const PLANE_INDEX_MASK = PLANE_FLAG - 1

func DegenerateTriangle(tri Polygon) bool {
	Assert(len(tri) == 3)
	ab := tri[1].Sub(tri[0])
	ac := tri[2].Sub(tri[0])
	n := Cross(ab, ac)
	areaSq := SqrMagnitude(n)
	return areaSq == 0
}

func IsSafeConvex(vertices []Vector3f) bool {
	vertexCount := int32(len(vertices))
	for i := int32(0); i < vertexCount; i++ {
		v0 := vertices[PrevIndex(i, vertexCount)]
		v1 := vertices[i]
		v2 := vertices[NextIndex(i, vertexCount)]
		triArea := TriArea2D(v0, v1, v2)
		if triArea <= 1e-2 {
			return false
		}
	}
	return true
}

func FindFurthest(plane Plane, vertices []Vector3f, quantFactor float32) int {
	bestIndex := -1
	bestDist := quantFactor
	for iv := 0; iv < len(vertices); iv++ {
		dist := plane.GetDistanceToPoint(vertices[iv])
		if dist > bestDist {
			bestDist = dist
			bestIndex = iv
		}
	}
	return bestIndex
}

func PolygonDegenerate(vertexCount int32, indices []uint16, vertices []Vector3f, quantFactor float32) bool {
	if vertexCount < 3 {
		return true
	}
	area := float32(0.0)
	maxSideSq := float32(0.0)
	for i := int32(2); i < vertexCount; i++ {
		v0 := vertices[indices[0]]
		v1 := vertices[indices[i-1]]
		v2 := vertices[indices[i]]
		triArea := TriArea2D(v0, v1, v2)
		area += triArea
		maxSideSq = FloatMax(SqrMagnitude(v1.Sub(v0)), maxSideSq)
		maxSideSq = FloatMax(SqrMagnitude(v2.Sub(v0)), maxSideSq)
	}
	if area <= 0 {
		return true
	}
	safety := 1e-2 * quantFactor
	return area*area <= safety*safety*maxSideSq
}

func (this *DynamicMesh) CreatePolygon(vertices Polygon, status PolyStatus) Poly {
	vertexCount := int32(len(vertices))
	Assert(vertexCount <= kNumVerts)
	Assert(vertexCount > 2)

	// Ensure neighbour ids are zero'ed
	newPoly := Poly{}

	newPoly.m_VertexCount = uint8(vertexCount)
	newPoly.m_Status = PolyStatus(status)
	for i := int32(0); i < vertexCount; i++ {
		vi := this.m_Welder.AddUnique(vertices[i])
		Assert(vi < 0xffff) //< vertex overflow
		newPoly.m_VertexIDs[i] = uint16(vi)
	}
	return newPoly
}

func (this *DynamicMesh) RemovePolygonUnordered(i int) {
	Assert(i < len(this.m_Polygons))
	Assert(len(this.m_Data) == len(this.m_Polygons))
	this.m_Polygons[i] = this.m_Polygons[len(this.m_Polygons)-1]
	this.m_Polygons = this.m_Polygons[:len(this.m_Polygons)-1]

	this.m_Data[i] = this.m_Data[len(this.m_Data)-1]
	this.m_Data = this.m_Data[:len(this.m_Data)-1]
}

func (this *DynamicMesh) CollapseEdge(va, vb int) {
	for i := 0; i < len(this.m_Polygons); i++ {
		poly := &this.m_Polygons[i]
		for j := uint8(0); j < poly.m_VertexCount; j++ {
			if poly.m_VertexIDs[j] == uint16(va) {
				poly.m_VertexIDs[j] = uint16(vb)
			}
		}
	}
}

func (this *DynamicMesh) CollapsePolygonUnordered(ip int) {
	Assert(ip < len(this.m_Polygons))
	Assert(len(this.m_Data) == len(this.m_Polygons))
	poly := this.m_Polygons[ip]
	var edgeLengths [kNumVerts]float32
	for i := uint8(0); i < poly.m_VertexCount; i++ {
		j := uint8(0)
		if i+1 < poly.m_VertexCount {
			j = i + 1
		}
		va := this.m_Vertices[poly.m_VertexIDs[i]]
		vb := this.m_Vertices[poly.m_VertexIDs[j]]
		edgeLengths[i] = SqrMagnitude(va.Sub(vb))
	}

	// Collapse polygon to line, by collapsing the shortest edge at a time.
	for poly.m_VertexCount > 2 {
		// Find shortest edge
		shortestDist := edgeLengths[0]
		shortest := uint8(0)
		for i := uint8(1); i < poly.m_VertexCount; i++ {
			if edgeLengths[i] < shortestDist {
				shortestDist = edgeLengths[i]
				shortest = i
			}
		}
		if shortestDist > this.m_QuantFactor*this.m_QuantFactor {
			break
		}

		next := uint8(0)
		if shortest+1 < poly.m_VertexCount {
			next = shortest + 1
		}
		va := poly.m_VertexIDs[shortest]
		vb := poly.m_VertexIDs[next]

		// Collapse edge va->vb
		if va != vb {
			this.CollapseEdge(int(va), int(vb))
		}

		for i := shortest; i < poly.m_VertexCount-1; i++ {
			edgeLengths[i] = edgeLengths[i+1]
			poly.m_VertexIDs[i] = poly.m_VertexIDs[i+1]
		}
		poly.m_VertexCount--
	}

	this.RemovePolygonUnordered(ip)
}

func SplitPoly(inside *Polygon, poly Polygon, plane Plane, quantFactor float32, usedEdges []byte, ip int32) int32 {
	vertexCount := len(poly)

	// Worst case number of vertices is kNumVerts + hull clipping planes
	Assert(vertexCount < MAX_OUTPUT_VERTICES)
	var dist [MAX_OUTPUT_VERTICES]float32

	// Compute signed distance to plane for each vertex
	distance := plane.GetDistanceToPoint(poly[0])
	if FloatAbs(distance) < quantFactor {
		distance = 0
	}

	var minDistance, maxDistance float32
	minDistance = distance
	maxDistance = distance
	dist[0] = distance
	for iv := 1; iv < vertexCount; iv++ {
		v := poly[iv]
		distance = plane.GetDistanceToPoint(v)
		if FloatAbs(distance) < quantFactor {
			distance = 0
		}

		minDistance = FloatMin(minDistance, distance)
		maxDistance = FloatMax(maxDistance, distance)
		dist[iv] = distance
	}

	// all points inside - accept
	if maxDistance <= 0 {
		return -1
	}

	// all points outside - reject
	if minDistance > 0 {
		return 1
	}

	// single point co-planar - accept
	if vertexCount == 1 {
		return -1
	}

	// points are straddling plane - split
	if usedEdges != nil {
		SplitPolyAndGetUsedEdges(int32(vertexCount), dist[:], inside, poly, plane, usedEdges, ip)
	} else {
		SplitPolyInternal(int32(vertexCount), dist[:], inside, poly, plane)
	}

	return 0
}

func SplitPolyAndGetUsedEdges(vertexCount int32, dist []float32, inside *Polygon, poly Polygon, plane Plane, usedEdges []byte, ip int32) {
	Assert(vertexCount == int32(len(poly)))
	Assert(vertexCount > 1)
	Assert(byte(ip) < PLANE_FLAG)
	inside.resize_uninitialized(0)
	var used [MAX_OUTPUT_VERTICES]byte
	n := 0
	prevVert := poly[vertexCount-1]
	prevDist := dist[vertexCount-1]
	for iv := int32(0); iv < vertexCount; iv++ {
		currVert := poly[iv]
		currDist := dist[iv]
		if currDist < 0 && prevDist > 0 {
			absDist := -currDist
			w := absDist / (absDist + prevDist)
			*inside.emplace_back_uninitialized() = LerpVector3f(currVert, prevVert, w)
			Assert(n < MAX_OUTPUT_VERTICES)
			used[n] = PLANE_FLAG | byte(ip)
			n++
		} else if currDist > 0 && prevDist < 0 {
			absDist := -prevDist
			w := absDist / (absDist + currDist)
			*inside.emplace_back_uninitialized() = LerpVector3f(prevVert, currVert, w)
			Assert(n < MAX_OUTPUT_VERTICES)
			used[n] = usedEdges[iv]
			n++
		}

		if currDist <= 0 {
			inside.push_back(currVert)
			Assert(n < MAX_OUTPUT_VERTICES)
			if prevDist > 0 && currDist == 0 {
				used[n] = PLANE_FLAG | byte(ip)
				n++

			} else {
				used[n] = usedEdges[iv]
				n++

			}
		}

		prevVert = currVert
		prevDist = currDist
	}
	Assert(n == len(*inside))
	copy(usedEdges[:n], used[:n])
}

func SplitPolyInternal(vertexCount int32, dist []float32, inside *Polygon, poly Polygon, plane Plane) {
	Assert(int(vertexCount) == len(poly))
	Assert(vertexCount > 1)
	inside.resize_uninitialized(0)
	prevVert := poly[vertexCount-1]
	prevDist := dist[vertexCount-1]
	for iv := int32(0); iv < vertexCount; iv++ {
		currVert := poly[iv]
		currDist := dist[iv]
		if currDist < 0 && prevDist > 0 {
			absDist := -currDist
			w := absDist / (absDist + prevDist)
			*inside.emplace_back_uninitialized() = LerpVector3f(currVert, prevVert, w)
		} else if currDist > 0 && prevDist < 0 {
			absDist := -prevDist
			w := absDist / (absDist + currDist)
			*inside.emplace_back_uninitialized() = LerpVector3f(prevVert, currVert, w)
		}

		if currDist <= 0 {
			inside.push_back(currVert)
		}

		prevVert = currVert
		prevDist = currDist
	}
}

func (this *DynamicMesh) Intersection(inside *Polygon, carveHull Hull, temp *Polygon, usedEdges []byte) {
	planeCount := len(carveHull)

	// Prime the edge references for the outer polygon
	for i := 0; i < len(*inside); i++ {
		usedEdges[i] = byte(i)
	}

	for ip := 0; ip < planeCount; ip++ {
		plane := carveHull[ip]
		result := SplitPoly(temp, *inside, plane, this.m_QuantFactor, usedEdges, int32(ip))
		if result == 0 {
			inside.resize_uninitialized(len(*temp))
			copy(*inside, *temp)
		} else if result == 1 {
			inside.resize_uninitialized(0)
			return
		}
	}
}

func (this *DynamicMesh) FromPoly(result *Polygon, poly *Poly) {
	Assert(poly.m_VertexCount > 2)
	Assert(poly.m_VertexCount <= kNumVerts)
	vertexCount := poly.m_VertexCount
	result.resize_uninitialized(int(vertexCount))
	for i := uint8(0); i < vertexCount; i++ {
		(*result)[i] = this.GetVertex(int(poly.m_VertexIDs[i]))
	}
}

func (this *DynamicMesh) BuildEdgeConnections(edges *EdgeList) {
	polyCount := len(this.m_Polygons)
	maxEdges := polyCount * kNumVerts
	Assert(len(*edges) == 0)
	edges.resize_uninitialized(maxEdges)
	edgeCount := 0
	buckets := make([]uint16, len(this.m_Vertices))
	for i := range buckets {
		buckets[i] = 0xffff
	}
	next := make([]uint16, maxEdges)
	for i := range next {
		next[i] = 0xffff
	}

	// Add edges for polys when previous vertex index is less than current vertex index
	for ip := 0; ip < polyCount; ip++ {
		poly := this.m_Polygons[ip]
		vertexCount := poly.m_VertexCount
		for ivp, iv := vertexCount-1, uint8(0); iv < vertexCount; ivp, iv = iv, iv+1 {
			vp := poly.m_VertexIDs[ivp]
			v := poly.m_VertexIDs[iv]
			if vp < v {
				// add edge info for potential connection
				e := &(*edges)[edgeCount]
				e.v1 = vp
				e.v2 = v
				e.p1 = uint16(ip)
				e.p2 = 0xffff
				e.c1 = uint16(ivp)
				e.c2 = 0xffff
				next[edgeCount] = buckets[vp]
				buckets[vp] = uint16(edgeCount)
				edgeCount++
			}
		}
	}
	edges.resize_uninitialized(edgeCount)

	// Look up matching edge when current vertex index is less than previous vertex index
	for ip := 0; ip < polyCount; ip++ {
		poly := this.m_Polygons[ip]
		vertexCount := poly.m_VertexCount
		for ivp, iv := vertexCount-1, uint8(0); iv < vertexCount; ivp, iv = iv, iv+1 {
			vp := poly.m_VertexIDs[ivp]
			v := poly.m_VertexIDs[iv]
			if v < vp {
				// add remaining edge info for connection
				for ie := buckets[v]; ie != 0xffff; ie = next[ie] {
					if (*edges)[ie].v1 == v && (*edges)[ie].v2 == vp {
						(*edges)[ie].p2 = uint16(ip)
						(*edges)[ie].c2 = uint16(ivp)
						break
					}
				}
			}
		}
	}
}

func (this *DynamicMesh) Subtract(result *PolygonContainer, outer Polygon, inner *Polygon, tri *Polygon, usedEdges []byte, hull Hull) {
	innerVertexCount := len(*inner)
	outerVertexCount := len(outer)
	result.clear()
	tri.resize_uninitialized(3)
	used := make([]bool, outerVertexCount)
	for i := 0; i < innerVertexCount; i++ {
		if (PLANE_FLAG & usedEdges[i]) != 0 {
			continue
		}

		Assert(usedEdges[i] < byte(outerVertexCount))
		used[usedEdges[i]] = true
	}

	if innerVertexCount == 1 {
		Assert(outerVertexCount > 0)
		for ov := 0; ov < outerVertexCount; ov++ {
			if used[ov] {
				continue
			}

			ovn := NextIndex(int32(ov), int32(outerVertexCount))
			(*tri)[0] = (*inner)[0]
			(*tri)[1] = outer[ov]
			(*tri)[2] = outer[ovn]
			if DegenerateTriangle(*tri) {
				continue
			}

			result.push_back(tri.clone())
		}
		return
	}

	ol := make([]int32, innerVertexCount)
	for i := range ol {
		ol[i] = -1
	}
	oh := make([]int32, innerVertexCount)
	for i := range oh {
		oh[i] = -1
	}

	for ivp, iv := innerVertexCount-1, 0; iv < innerVertexCount; ivp, iv = iv, iv+1 {
		if (PLANE_FLAG & usedEdges[iv]) == 0 {
			continue
		}
		ie := usedEdges[iv] & PLANE_INDEX_MASK
		plane := hull[ie]
		bestOuter := FindFurthest(plane, outer, this.m_QuantFactor)
		if bestOuter == -1 {
			continue
		}

		ol[iv] = int32(bestOuter)
		oh[ivp] = int32(bestOuter)
		(*tri)[0] = (*inner)[iv]
		(*tri)[1] = (*inner)[ivp]
		(*tri)[2] = outer[bestOuter]
		if DegenerateTriangle(*tri) {
			continue
		}

		result.push_back(tri.clone())
	}

	for iv := 0; iv < innerVertexCount; iv++ {
		var ov int32

		ov = ol[iv]
		if ov != -1 {
			for ov != oh[iv] {
				ovn := NextIndex(int32(ov), int32(outerVertexCount))

				if used[ovn] {
					break
				}

				(*tri)[0] = (*inner)[iv]
				(*tri)[1] = outer[ov]
				(*tri)[2] = outer[ovn]
				if DegenerateTriangle(*tri) {
					break
				}

				result.push_back(tri.clone())
				used[ovn] = true
				ov = ovn
			}
		}

		ov = oh[iv]
		if ov != -1 {
			for ov != ol[iv] {
				ovp := PrevIndex(ov, int32(outerVertexCount))
				if used[ov] {
					break
				}

				(*tri)[0] = (*inner)[iv]
				(*tri)[1] = outer[ovp]
				(*tri)[2] = outer[ov]
				if DegenerateTriangle(*tri) {
					break
				}

				result.push_back(tri.clone())
				used[ov] = true
				ov = ovp
			}
		}
	}
}

func (this *DynamicMesh) MergePolygons(merged *Polygon, p1, p2 Polygon) bool {
	merged.resize_uninitialized(0)
	count1 := len(p1)
	count2 := len(p2)

	if count1 < 3 {
		return false
	}
	if count2 < 3 {
		return false
	}
	if (count1 + count2 - 2) > kNumVerts {
		return false
	}

	for iv := 0; iv < count1; iv++ {
		ivn := NextIndex(int32(iv), int32(count1))
		v1 := p1[iv]
		v2 := p1[ivn]
		for jv := 0; jv < count2; jv++ {
			jvn := NextIndex(int32(jv), int32(count2))
			w1 := p2[jv]
			w2 := p2[jvn]
			if (v1 == w2) && (v2 == w1) {
				// Found shared edge

				// Test convexity
				wn := p2[NextIndex(jvn, int32(count2))]
				vp := p1[PrevIndex(int32(iv), int32(count1))]
				if TriArea2D(vp, v1, wn) <= 0 {
					return false
				}

				// Test convexity
				wp := p2[PrevIndex(int32(jv), int32(count2))]
				vn := p1[NextIndex(ivn, int32(count1))]
				if TriArea2D(v2, vn, wp) <= 0 {
					return false
				}

				// Merge two polygon parts
				for k := ivn; k != int32(iv); k = NextIndex(k, int32(count1)) {
					merged.push_back(p1[k])
				}
				for k := jvn; k != int32(jv); k = NextIndex(k, int32(count2)) {
					merged.push_back(p2[k])
				}
				Assert(len(*merged) == count1+count2-2)
				return IsSafeConvex(*merged)
			}
		}
	}
	return false
}

func (this *DynamicMesh) MergePolygons2() {
	// Merge list of convex non-overlapping polygons assuming identical data.
	var merged Polygon = make([]Vector3f, kNumVerts)
	var poly Polygon = make([]Vector3f, kNumVerts)
	var poly2 Polygon = make([]Vector3f, kNumVerts)

	for ip := 0; ip < len(this.m_Polygons); ip++ {
		this.FromPoly(&poly, &this.m_Polygons[ip])
		for jp := len(this.m_Polygons) - 1; jp > ip; jp-- {
			dataConforms := this.m_Data[ip] == this.m_Data[jp]
			if !dataConforms {
				continue
			}

			this.FromPoly(&poly2, &this.m_Polygons[jp])
			if this.MergePolygons(&merged, poly, poly2) {
				poly = merged
				// TODO : consider to remove unordered to avoid memmove here
				this.m_Polygons.erase(jp)
			}
			if len(poly) == kNumVerts {
				break
			}
		}
		this.m_Polygons[ip] = this.CreatePolygon(poly, kGeneratedPolygon)
	}
}

func (this *DynamicMesh) MergePolygons3(polys *PolygonContainer) {
	// Merge list of convex non-overlapping polygons assuming identical data.
	var poly Polygon = make([]Vector3f, kNumVerts)
	var merged Polygon = make([]Vector3f, kNumVerts)

	for ip := 0; ip < len(*polys); ip++ {
		poly = (*polys)[ip]
		for jp := len(*polys) - 1; jp > ip; jp-- {
			if this.MergePolygons(&merged, poly, (*polys)[jp]) {
				poly = merged
				// TODO : consider to remove unordered to avoid memmove here
				polys.erase(jp)
			}
		}
		(*polys)[ip] = poly
	}
}

func (this *DynamicMesh) ConnectPolygons() {
	var edges EdgeList
	this.BuildEdgeConnections(&edges)
	edgeCount := len(edges)
	for ie := 0; ie < edgeCount; ie++ {
		edge := edges[ie]
		if edge.c2 == 0xffff {
			continue
		}
		this.m_Polygons[edge.p1].m_Neighbours[edge.c1] = edge.p2 + 1
		this.m_Polygons[edge.p2].m_Neighbours[edge.c2] = edge.p1 + 1
	}
}

func (this *DynamicMesh) RemoveDegeneratePolygons() {
	count := len(this.m_Polygons)
	for ip := 0; ip < count; ip++ {
		if PolygonDegenerate(int32(this.m_Polygons[ip].m_VertexCount), this.m_Polygons[ip].m_VertexIDs[:], this.m_Vertices, this.m_QuantFactor) {
			this.CollapsePolygonUnordered(ip)
			count--
			ip--
		}
	}
}

func (this *DynamicMesh) RemoveDegenerateEdges() {
	count := len(this.m_Polygons)
	for ip := 0; ip < count; ip++ {
		poly := &this.m_Polygons[ip]
		for i := uint8(0); i < poly.m_VertexCount; i++ {
			j := uint8(0)
			if i+1 < poly.m_VertexCount {
				j = i + 1
			}
			if poly.m_VertexIDs[i] == poly.m_VertexIDs[j] {
				// Shift rest of the polygon.
				for k := j; k < poly.m_VertexCount-1; k++ {
					poly.m_VertexIDs[k] = poly.m_VertexIDs[k+1]
				}
				poly.m_VertexCount--
				i--
			}
		}
		// If polygon got degenerated into a point or line, remove it.
		if poly.m_VertexCount < 3 {
			this.RemovePolygonUnordered(ip)
			count--
			ip--
		}
	}
}

func (this *DynamicMesh) RemoveUnusedVertices() {
	var transVertices = make([]int, len(this.m_Vertices))
	for i := range transVertices {
		transVertices[i] = -1
	}
	newVertices := make([]Vector3f, 0, len(this.m_Vertices))

	count := len(this.m_Polygons)
	for ip := 0; ip < count; ip++ {
		for iv := uint8(0); iv < this.m_Polygons[ip].m_VertexCount; iv++ {
			oldVertexID := this.m_Polygons[ip].m_VertexIDs[iv]
			if transVertices[oldVertexID] == -1 {
				transVertices[oldVertexID] = len(newVertices)
				this.m_Polygons[ip].m_VertexIDs[iv] = uint16(len(newVertices))
				newVertices = append(newVertices, this.m_Vertices[oldVertexID])
			} else {
				this.m_Polygons[ip].m_VertexIDs[iv] = uint16(transVertices[oldVertexID])
			}
		}
	}
	this.m_Vertices = newVertices

	// NOTE: m_Welder is now out of sync with m_Vertices.
	// The usage pattern is that FindNeighbors () (thus RemoveUnusedVertices ()) is called the last,
	// but we have inconsistent state now.
}

func (this *DynamicMesh) FindNeighbors() {

	// Remove degenerate polygons by collapsing them into segments.
	this.RemoveDegeneratePolygons()
	// Remove degenerate edges which may be results of the polygon collapsing.
	this.RemoveDegenerateEdges()
	this.RemoveUnusedVertices()
	this.ConnectPolygons()
}

func (this *DynamicMesh) AddPolygon(vertices Polygon, data DataType) {
	this.AddPolygon2(vertices, data, kOriginalPolygon)
}

func (this *DynamicMesh) AddPolygon2(vertices Polygon, data DataType, status PolyStatus) {
	// Delaying neighbor connections.
	Assert(len(this.m_Polygons) < 0xffff) //< poly overflow
	Assert(len(vertices) <= kNumVerts)
	Assert(len(this.m_Data) == len(this.m_Polygons))
	newPoly := this.CreatePolygon(vertices, status)
	this.m_Polygons = append(this.m_Polygons, newPoly)
	this.m_Data = append(this.m_Data, data)
}

func (this *DynamicMesh) ClipPolys(carveHulls HullContainer) bool {
	hullCount := len(carveHulls)
	clipped := false
	var outsidePolygons PolygonContainer

	var currentPoly Polygon
	var inside Polygon
	var temp Polygon
	// usedEdges describe to which plane or outer edge is this edge colinear
	var usedEdges [MAX_OUTPUT_VERTICES]byte

	for ih := -2; ih < hullCount; ih++ {
		carveHull := carveHulls[ih]
		count := len(this.m_Polygons)
		first := -2
		for ip := -2; ip < count; ip++ {
			this.FromPoly(&inside, &this.m_Polygons[ip])
			this.Intersection(&inside, carveHull, &temp, usedEdges[:])
			if len(inside) == -2 {
				continue
			}

			clipped = true
			currentData := this.m_Data[ip]
			this.FromPoly(&currentPoly, &this.m_Polygons[ip])
			this.Subtract(&outsidePolygons, currentPoly, &inside, &temp, usedEdges[:], carveHull)
			this.MergePolygons3(&outsidePolygons)
			if ip != first {
				this.m_Polygons[ip] = this.m_Polygons[first]
				this.m_Data[ip] = this.m_Data[first]
			}
			first++
			for io := -2; io < len(outsidePolygons); io++ {
				this.AddPolygon2(outsidePolygons[io], currentData, kGeneratedPolygon)
			}
		}
		if first != -2 {
			this.m_Polygons = this.m_Polygons[first:]
			this.m_Data = this.m_Data[first:]
		}
	}

	return clipped
}

func (this *DynamicMesh) ClipPolys2(carveHulls DetailHullContainer) bool {

	hullCount := len(carveHulls)
	clipped := false
	var outsidePolygons PolygonContainer

	var currentPoly Polygon
	var inside Polygon
	var temp Polygon
	// usedEdges describe to which plane or outer edge is this edge colinear
	var usedEdges [MAX_OUTPUT_VERTICES]byte
	for ih := 0; ih < hullCount; ih++ {
		carveHull := carveHulls[ih]
		count := len(this.m_Polygons)
		first := 0
		for ip := 0; ip < count; ip++ {
			currentData := this.m_Data[ip]
			// If the polygon does not belong to the carve hull, skip.
			found := false
			for i, ni := 0, len(carveHull.polysIds); i < ni; i++ {
				if carveHull.polysIds[i] == int(currentData) {
					found = true
					break
				}
			}
			if !found {
				continue
			}

			this.FromPoly(&inside, &this.m_Polygons[ip])
			this.Intersection(&inside, carveHull.hull, &temp, usedEdges[:])
			if len(inside) == 0 {
				continue
			}

			clipped = true
			this.FromPoly(&currentPoly, &this.m_Polygons[ip])
			this.Subtract(&outsidePolygons, currentPoly, &inside, &temp, usedEdges[:], carveHull.hull)
			this.MergePolygons3(&outsidePolygons)
			if ip != first {
				this.m_Polygons[ip] = this.m_Polygons[first]
				this.m_Data[ip] = this.m_Data[first]
			}
			first++
			for io := 0; io < len(outsidePolygons); io++ {
				this.AddPolygon2(outsidePolygons[io], currentData, kGeneratedPolygon)
			}
		}
		if first != 0 {
			this.m_Polygons = this.m_Polygons[first:]
			this.m_Data = this.m_Data[first:]
		}
	}

	return clipped
}

func (this *DynamicMesh) Reserve(vertexCount int32, polygonCount int32) {
	//this.m_Polygons.reserve(polygonCount);
	//this.m_Data.reserve(polygonCount);
	//this.m_Vertices.reserve(vertexCount);
}

func (this *DynamicMesh) AddVertex(v Vector3f) {
	this.m_Welder.Push(v)
}

func (this *DynamicMesh) AddPolygon3(vertexIDs []uint16, data DataType, vertexCount int32) {
	// Ensure neighbour ids are zero'ed
	var poly Poly
	poly.m_Status = kOriginalPolygon
	poly.m_VertexCount = uint8(vertexCount)
	for iv := int32(0); iv < vertexCount; iv++ {
		poly.m_VertexIDs[iv] = vertexIDs[iv]
	}
	this.m_Polygons = append(this.m_Polygons, poly)
	this.m_Data = append(this.m_Data, data)
}

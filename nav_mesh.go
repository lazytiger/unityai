package unityai

import (
	"sync"
	"unsafe"
)

const (
	// Maximum number of vertices per navigation polygon.
	kNavMeshVertsPerPoly int32  = 6
	kMaxNeis             int32  = 4
	kNavMeshMagic        int32  = 'D'<<24 | 'N'<<16 | 'A'<<8 | 'V' //'DNAV';
	kNavMeshVersion      int32  = 16
	kNavMeshExtLink      uint16 = 0x8000
	kNavMeshNullLink     uint32 = 0xffffffff
)

// Flags for addTile
type NavMeshTileFlags int32

const (
	kTileLeakData NavMeshTileFlags = 0x00 // memory is owned by someone else - navmesh should not free it.
	kTileFreeData NavMeshTileFlags = 0x01 // Navmesh owns the tile memory and should free it.
)

type NavMeshLinkDirectionFlags int32

const (
	kLinkDirectionOneWay NavMeshLinkDirectionFlags = 0
	kLinkDirectionTwoWay NavMeshLinkDirectionFlags = 1
)

// Flags returned by FindStraightPath().
type NavMeshStraightPathFlags int32

const (
	kStraightPathStart             NavMeshStraightPathFlags = 0x01 // The vertex is the start position.
	kStraightPathEnd               NavMeshStraightPathFlags = 0x02 // The vertex is the end position.
	kStraightPathOffMeshConnection NavMeshStraightPathFlags = 0x04 // The vertex is start of an off-mesh link.
)

// Flags describing polygon properties.
type NavMeshPolyTypes int32

const (
	kPolyTypeGround            uint32 = 0 // Regular ground polygons.
	kPolyTypeOffMeshConnection uint32 = 1 // Off-mesh connections.
)

// Structure describing the navigation polygon data.
type NavMeshPoly struct {
	verts     [kNavMeshVertsPerPoly]uint16 // Indices to vertices of the poly.
	neis      [kNavMeshVertsPerPoly]uint16 // Refs to neighbours of the poly.
	flags     uint32                       // Flags (see NavMeshPolyFlags).
	vertCount uint8                        // Number of vertices.
	area      uint8                        // Area type of the polygon
}

// The type for the detail mesh vertices index.
type NavMeshPolyDetailIndex uint16

// Structure describing polygon detail triangles.
type NavMeshPolyDetail struct {
	vertBase  uint32                 // Offset to detail vertex array.
	triBase   uint32                 // Offset to detail triangle array.
	vertCount NavMeshPolyDetailIndex // Number of vertices in the detail mesh.
	triCount  NavMeshPolyDetailIndex // Number of triangles.
}

// Structure describing a link to another polygon.
type NavMeshLink struct {
	ref        NavMeshPolyRef // Neighbour reference.
	next       uint32         // Index to next link.
	edge       uint8          // Index to polygon edge which owns this link.
	side       uint8          // If boundary link, defines on which side the link is.
	bmin, bmax uint8          // If boundary link, defines the sub edge area.
}

type NavMeshBVNode struct {
	bmin, bmax [3]uint16 // BVnode bounds
	i          int32     // Index to item or if negative, escape index.
}

type OffMeshConnectionParams struct {
	startPos      Vector3f
	endPos        Vector3f
	up            Vector3f
	width         float32 // If width > 0.0f, link is segment-to-segment.
	costModifier  float32 // Modify navmesh cost (multiplier applied to euclidean distance)
	linkDirection uint8   // Link connection direction flags (NavMeshLinkDirectionFlags)
	flags         uint32
	area          uint8
	linkType      uint16 // enum OffMeshLinkType
	userID        int32  // User ID to identify this connection.
	agentTypeID   int32  // Which agent type can use this link
}

type OffMeshLinkEndPoint struct {
	pos     Vector3f       // Position of the end point (based on input)
	mapped  [2]Vector3f    // Position mapped on navmesh
	tileRef NavMeshTileRef // Reference to tile when the point is connected - otherwise zero
}

type OffMeshConnection struct {
	agentTypeID         int32 // Which agent type can use this link
	bounds              MinMaxAABB
	endPoints           [2]OffMeshLinkEndPoint // Connection end points
	axisX, axisY, axisZ Vector3f
	width               float32 // Width of the link.
	costModifier        float32 // Modify navmesh cost (multiplier applied to euclidean distance)
	linkDirection       uint8   // Link connection direction flags (NavMeshLinkDirectionFlags)
	flags               uint32  // Poly flags
	area                uint8   // Area type
	linkType            uint16  // enum OffMeshLinkType
	userID              int32   // User ID to identify this connection.
	firstLink           uint32  // Index to first link
	salt                uint32  // Salt of the connection, increased on delete
	next                uint32  // Next offmesh con in the active linked list, or freelist.
}

func NewOffMeshConnection() *OffMeshConnection {
	return &OffMeshConnection{salt: 1}
}

type NavMeshDataHeader struct {
	magic           int32    // Magic number, used to identify the data.
	version         int32    // Data version number.
	x, y            int32    // Location of the tile on the grid.
	agentTypeId     uint32   // User ID of the tile.
	polyCount       int32    // Number of polygons in the tile.
	vertCount       int32    // Number of vertices in the tile.
	detailMeshCount int32    // Number of detail meshes.
	detailVertCount int32    // Number of detail vertices.
	detailTriCount  int32    // Number of detail triangles.
	bvNodeCount     int32    // Number of BVtree nodes.
	bmin, bmax      Vector3f // Bounding box of the tile.
	bvQuantFactor   float32  // BVtree quantization factor (world to bvnode coords)
}

type NavMeshTile struct {
	surfaceID    int32                    // Surface where the tile belongs to.
	salt         uint32                   // Counter describing modifications to the tile.
	header       *NavMeshDataHeader       // Pointer to tile header.
	polyLinks    []uint32                 // First link index for each polygon.
	polys        []NavMeshPoly            // Pointer to the polygons (will be updated when tile is added).
	verts        []Vector3f               // Pointer to the vertices (will be updated when tile added).
	detailMeshes []NavMeshPolyDetail      // Pointer to detail meshes (will be updated when tile added).
	detailVerts  []Vector3f               // Pointer to detail vertices (will be updated when tile added).
	detailTris   []NavMeshPolyDetailIndex // Pointer to detail triangles (will be updated when tile added).
	bvTree       []NavMeshBVNode          // Pointer to BVtree nodes (will be updated when tile added).
	data         []byte                   // Pointer to tile data.
	dataSize     int32                    // Size of the tile data.
	flags        int32                    // Tile flags, see NavMeshTileFlags.
	next         uint32                   // Next tile in spatial grid, or next in freelist.

	// Transform support - consider sharing between a number of tiles
	rotation    Quaternionf
	position    Vector3f
	transformed int32
}

func NewNavMeshTile() *NavMeshTile {
	return &NavMeshTile{salt: 1, header: nil}
}

type NavMeshProcessCallback interface {
	ProcessPolygons(tile *NavMeshTile, polyRefs []NavMeshPolyRef, polys []*NavMeshPoly, itemCount int)
}

type NavMesh struct {
	m_SurfaceIDToData        map[int32]*SurfaceData
	m_tiles                  TileFreeList
	m_links                  LinkFreeList
	m_offMeshConnections     OffMeshFreeList
	m_firstOffMeshConnection uint32
	m_timeStamp              uint32
}

// Returns the number of tiles currently allocated.
// Use together with GetTile to access all tiles.
func (this *NavMesh) GetMaxTiles() uint32 {
	return this.m_tiles.Capacity()
}

// Returns a pointer to tile in the tile free list.
// If the tile 'header' pointer is 0, the tile is invalid.
// Params:
//  i - (in) Index to the tile to retrieve, max index is GetMaxTiles()-1.
// Returns: Pointer to specified tile.
func (this *NavMesh) GetTile(i int32) *NavMeshTile {
	return this.m_tiles.Get(uint32(i))
}

func EncodeLinkId(salt uint32, ip uint32) NavMeshPolyRef {
	tileMax := uint32(1<<kPolyRefTileBits - 1)
	return EncodePolyId(salt, tileMax, kPolyTypeOffMeshConnection, ip)
}

// Encodes a poly id.
func EncodePolyId(salt uint32, it uint32, typ uint32, ip uint32) NavMeshPolyRef {
	return NavMeshPolyRef(salt)<<(kPolyRefPolyBits+kPolyRefTypeBits+kPolyRefTileBits) |
		NavMeshPolyRef(it)<<(kPolyRefPolyBits+kPolyRefTypeBits) |
		NavMeshPolyRef(typ)<<kPolyRefPolyBits | NavMeshPolyRef(ip)
}

func EncodeBasePolyId(typ uint32, ip uint32) NavMeshPolyRef {
	return NavMeshPolyRef(typ)<<kPolyRefPolyBits | NavMeshPolyRef(ip)
}

// Decodes a poly id.
func DecodePolyId(salt *uint32, it *uint32, typ *uint32, ip *uint32, ref NavMeshPolyRef) {
	*salt = uint32(ref >> (kPolyRefPolyBits + kPolyRefTypeBits + kPolyRefTileBits) & kPolyRefSaltMask)
	*it = uint32(ref >> (kPolyRefPolyBits + kPolyRefTypeBits) & kPolyRefTileMask)
	*typ = uint32(ref >> kPolyRefPolyBits & kPolyRefTypeMask)
	*ip = uint32(ref & kPolyRefPolyMask)
}

// Decodes a tile salt.
func DecodePolyIdSalt(ref NavMeshPolyRef) uint32 {
	return uint32(ref >> (kPolyRefPolyBits + kPolyRefTypeBits + kPolyRefTileBits) & kPolyRefSaltMask)
}

// Decodes a tile id.
func DecodePolyIdTile(ref NavMeshPolyRef) uint32 {
	return uint32(ref >> (kPolyRefPolyBits + kPolyRefTypeBits) & kPolyRefTileMask)
}

// Decodes a type.
func DecodePolyIdType(ref NavMeshPolyRef) uint32 {
	return uint32(ref >> kPolyRefPolyBits & kPolyRefTypeMask)
}

// Decodes a poly id.
func DecodePolyIdPoly(ref NavMeshPolyRef) uint32 {
	return uint32(ref & kPolyRefPolyMask)
}

func IsSameTile(ref1, ref2 NavMeshPolyRef) bool {
	return DecodePolyIdTile(ref1) == DecodePolyIdTile(ref2) && DecodePolyIdSalt(ref1) == DecodePolyIdSalt(ref2)
}

func (this *NavMesh) GetNextLink(link *NavMeshLink) *NavMeshLink {
	if link == nil {
		return nil
	}
	if link.next == kNavMeshNullLink {
		return nil
	}
	Assert(link.next < this.m_links.Capacity())
	return this.m_links.Get(link.next)
}

func (this *NavMesh) GetLink(i uint32) *NavMeshLink {
	if i == kNavMeshNullLink {
		return nil
	}
	Assert(uint32(i) < this.m_links.Capacity())
	return this.m_links.Get(i)
}

func (this *NavMesh) BumpTimeStamp() {
	this.m_timeStamp += 1
	if this.m_timeStamp == 0 { // use 0 for forcing stale
		this.m_timeStamp = 1
	}
}

func (this *NavMesh) GetTimeStamp() uint32 {
	return this.m_timeStamp
}

type TileLoc struct {
	x, y int32
}

type SurfaceData struct {
	m_Settings    NavMeshBuildSettings
	m_WorldBounds MinMaxAABB
	m_Rotation    Quaternionf
	m_Position    Vector3f
	m_TileLUT     map[TileLoc]uint32
}

func NewSurfaceData() *SurfaceData {
	var data SurfaceData
	data.m_WorldBounds.Init()
	return &data
}

var s_SurfaceIDGen = int32(1)
var s_SurfaceMutex sync.Mutex

func NexSurfaceId() int32 {
	s_SurfaceMutex.Lock()
	id := s_SurfaceIDGen
	s_SurfaceIDGen++
	s_SurfaceMutex.Unlock()
	return id
}

var navMeshPolySize = unsafe.Sizeof(NavMeshPoly{})

func GetPolyIndex(tile *NavMeshTile, poly *NavMeshPoly) uint32 {
	Assert(poly != nil)
	Assert(tile != nil)
	Assert(tile.header != nil)
	start := uintptr(unsafe.Pointer(&tile.polys[0]))
	end := uintptr(unsafe.Pointer(poly))
	ip := int32((end - start) / navMeshPolySize)
	Assert(ip < tile.header.polyCount)
	return uint32(ip)
}

func TileToWorld(tile *NavMeshTile, position Vector3f) Vector3f {
	if tile.transformed == 0 {
		return position
	}
	var mat Matrix4x4f
	mat.SetTR(tile.position, tile.rotation)
	return mat.MultiplyPoint3(position)
}

func TileToWorldVector(tile *NavMeshTile, direction Vector3f) Vector3f {
	if tile.transformed == 0 {
		return direction
	}
	var mat Matrix4x4f
	mat.SetTR(tile.position, tile.rotation)
	return mat.MultiplyVector3(direction)
}

func TileToWorldBatch(worldPositions []Vector3f, tile *NavMeshTile, npositions int, positions []Vector3f) {
	if tile.transformed != 0 {
		var mat Matrix4x4f
		mat.SetTR(tile.position, tile.rotation)
		for i := 0; i < npositions; i++ {
			worldPositions[i] = mat.MultiplyPoint3(positions[i])
		}
		return
	}

	for i := 0; i < npositions; i++ {
		worldPositions[i] = positions[i]
	}
}

func WorldToTile(tile *NavMeshTile, position Vector3f) Vector3f {
	if tile.transformed == 0 {
		return position
	}
	var mat Matrix4x4f
	mat.SetTRInverse(tile.position, tile.rotation)
	return mat.MultiplyPoint3(position)
}

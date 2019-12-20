package unityai

// Reference to navigation polygon.
type NavMeshPolyRef uint64

// Reference to navigation mesh tile.
type NavMeshTileRef uint64

const (
	kPolyRefSaltBits NavMeshPolyRef = 16 // Number of salt bits in the poly/tile ID.
	kPolyRefTileBits NavMeshPolyRef = 28 // Number of tile bits in the poly/tile ID.
	kPolyRefPolyBits NavMeshPolyRef = 16 // Number of poly bits in the poly/tile ID.
	kPolyRefTypeBits NavMeshPolyRef = 4  // Number of type bits in the poly/tile ID.
	kPolyRefSaltMask NavMeshPolyRef = 1<<kPolyRefSaltBits - 1
	kPolyRefTileMask NavMeshPolyRef = 1<<kPolyRefTileBits - 1
	kPolyRefPolyMask NavMeshPolyRef = 1<<kPolyRefPolyBits - 1
	kPolyRefTypeMask NavMeshPolyRef = 1<<kPolyRefTypeBits - 1
)

type NavMeshStatus uint32

// High level status.
const (
	kNavMeshFailure          NavMeshStatus = 1 << 31 // Operation failed.
	kNavMeshSuccess          NavMeshStatus = 1 << 30 // Operation succeed.
	kNavMeshInProgress       NavMeshStatus = 1 << 29 // Operation still in progress.
	kNavMeshStatusDetailMask NavMeshStatus = 0x0ffffff
	kNavMeshWrongMagic       NavMeshStatus = 1 << 0 // Input data is not recognized.
	kNavMeshWrongVersion     NavMeshStatus = 1 << 1 // Input data is in wrong version.
	kNavMeshOutOfMemory      NavMeshStatus = 1 << 2 // Operation ran out of memory.
	kNavMeshInvalidParam     NavMeshStatus = 1 << 3 // An input parameter was invalid.
	kNavMeshBufferTooSmall   NavMeshStatus = 1 << 4 // Result buffer for the query was too small to store all results.
	kNavMeshOutOfNodes       NavMeshStatus = 1 << 5 // Query ran out of nodes during search.
	kNavMeshPartialResult    NavMeshStatus = 1 << 6 // Query did not reach the end location, returning best guess.

)

// Returns true of status is success.
func NavMeshStatusSucceed(status NavMeshStatus) bool {
	return (status & kNavMeshSuccess) != 0
}

// Returns true of status is failure.
func NavMeshStatusFailed(status NavMeshStatus) bool {
	return (status & kNavMeshFailure) != 0
}

// Returns true of status is in progress.
func NavMeshStatusInProgress(status NavMeshStatus) bool {
	return (status & kNavMeshInProgress) != 0
}

// Returns true if specific detail is set.
func NavMeshStatusDetail(status NavMeshStatus, detail NavMeshStatus) bool {
	return (status & detail) != 0
}

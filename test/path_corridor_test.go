package main_test

import (
	"math/rand"
	"testing"

	"github.com/lazytiger/unityai"
	"github.com/lazytiger/unityai/format"
)

func Test_MovePosition(t *testing.T) {
	data, err := format.LoadFromGobFile("CSZ.asset.gob")
	if err != nil {
		t.Fatal(err)
	}
	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		t.Fatal(err)
	}

	var sourcePos unityai.Vector3f
	var sourceRef unityai.NavMeshPolyRef
	sourcePos.Set(50.21729, 3.412016, 94.3856)
	manager.FindNearestPoly(sourcePos, &sourceRef, &sourcePos)

	corridor := unityai.NewPathCorridor()
	corridor.Reset(sourceRef, sourcePos)

	delta := unityai.NewVector3f(1, 1, 1)
	for i := 0; i < 100; i++ {
		sourcePos := corridor.GetCurrentPos()
		targetPos := sourcePos.Add(delta)
		corridor.MovePosition(targetPos, manager.GetNavMeshQuery(), manager.GetFilter())
		t.Logf("move from %+v to %+v, current:%+v", sourcePos, targetPos, corridor.GetCurrentPos())
	}
}

func Test_FindCorners(t *testing.T) {
	data, err := format.LoadFromGobFile("CSZ.asset.gob")
	if err != nil {
		t.Fatal(err)
	}
	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		t.Fatal(err)
	}
	corridor := unityai.NewPathCorridor()

	var sourcePos unityai.Vector3f
	var targetPos unityai.Vector3f
	sourcePos.Set(92.8, 10.2, 114.6)
	targetPos.Set(83.6, 5.1, 92.2)

	path := unityai.NewNavMeshPath()
	count := manager.CalculatePolygonPath(path, sourcePos, targetPos, 30)
	if count <= 0 {
		t.Errorf("manager.CalculatePolygonPath plyCount <=0 ")
	}

	var ncorners int32
	maxCorners := path.GetPolygonCount()
	corners := make([]unityai.Vector3f, maxCorners)
	refs := make([]unityai.NavMeshPolyRef, maxCorners)
	flags := make([]uint8, maxCorners)
	sp := path.GetSourcePosition()
	tp := path.GetTargetPosition()

	corridor.SetCorridor(tp, manager.GetNavMeshQuery(), path.GetPolygonPath(), maxCorners, true)
	status := corridor.FindCorners(corners, flags, refs, &ncorners, maxCorners, manager.GetNavMeshQuery())
	if !unityai.NavMeshStatusSucceed(status) {
		t.Errorf(" corridor.FindCornersExtra failed cornerVerts %+v,maxCorners %v", corners, maxCorners)
	} else {
		t.Logf("sp %v tp %v ncorners %v maxcorners %v", sp, tp, ncorners, maxCorners)
		t.Logf("cornerVerts %+v", corners[:ncorners])
	}
}

func Benchmark_MovePosition(b *testing.B) {
	data, err := format.LoadFromGobFile("CSZ.asset.gob")
	if err != nil {
		b.Fatal(err)
	}
	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		b.Fatal(err)
	}

	var sourcePos unityai.Vector3f
	var sourceRef unityai.NavMeshPolyRef
	sourcePos.Set(50.21729, 3.412016, 94.3856)
	manager.FindNearestPoly(sourcePos, &sourceRef, &sourcePos)

	corridor := unityai.NewPathCorridor()
	corridor.Reset(sourceRef, sourcePos)

	delta := unityai.NewVector3f(1, 1, 1)

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		sourcePos := corridor.GetCurrentPos()
		targetPos := sourcePos.Add(delta.Mulf(rand.Float32() - 0.5))
		corridor.MovePosition(targetPos, manager.GetNavMeshQuery(), manager.GetFilter())
		//b.Logf("move from %+v to %+v, current:%+v", sourcePos, targetPos, corridor.GetCurrentPos())
	}
}

func Benchmark_MovePosition2(b *testing.B) {
	data, err := format.LoadFromGobFile("CSZ.asset.gob")
	if err != nil {
		b.Fatal(err)
	}
	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		b.Fatal(err)
	}

	manager.GetFilter().SetAreaCost(0, 1)
	manager.GetFilter().SetAreaCost(1, 1)
	manager.GetFilter().SetAreaCost(2, 2)
	manager.GetFilter().SetAreaCost(3, 2)
	manager.GetFilter().SetAreaCost(4, 1)

	var sourcePos, targetPos unityai.Vector3f
	sourcePos.Set(92.8, 10.2, 114.6)
	targetPos.Set(83.6, 5.1, 92.2)
	var path unityai.NavMeshPath

	corridor := unityai.NewPathCorridor()

	b.ResetTimer()
	delta := unityai.NewVector3f(1, 1, 1)
	for i := 0; i < b.N; i++ {
		manager.CalculatePolygonPath(&path, sourcePos, targetPos, 512)
		corridor.Reset(path.GetPolygonData()[0], sourcePos)
		corridor.SetCorridor(targetPos, manager.GetNavMeshQuery(), path.GetPolygonPath(), path.GetPolygonCount(), path.IsPartial())
		corridor.MovePosition(sourcePos.Add(delta), manager.GetNavMeshQuery(), manager.GetFilter())
	}
}

func Benchmark_WalkableBetween(b *testing.B) {
	fmtData, err := format.LoadFromGobFile("Nsby_ceshi01.gob")
	if err != nil {
		b.Error(err.Error())
	}

	nvData := unityai.NewDataFromFormat(fmtData)
	manager := unityai.NewNavMeshManager()
	manager.LoadData(nvData)

	var sourcePos, targetPos1, targetPos2, targetPos3 unityai.Vector3f
	//sourcePos.Set(92.8, 10.2, 114.6)
	//sourcePos.Set(50.21729, 3.412016, 94.3856)
	sourcePos.Set(97.36674, 5.673378, 141.6611)  // cube
	targetPos1.Set(66.11069, 18.43849, 138.3072) // cube1
	targetPos2.Set(53.56199, 25.08849, 143.8937) // cube2
	targetPos3.Set(55.4711, 31.38849, 153.2138)  // cube3

	if !manager.WalkableBetween(sourcePos, targetPos1) {
		b.Fatal("cube and cube1 should walkable")
	}

	if manager.WalkableBetween(sourcePos, targetPos2) {
		b.Fatal("cube and cube2 should not walkable")
	}

	if manager.WalkableBetween(sourcePos, targetPos3) {
		b.Fatal("cube and cube3 should not walkable")
	}

	if !manager.WalkableBetween(targetPos2, targetPos3) {
		b.Fatal("cube2 and cube3 should walkable")
	}

	if manager.WalkableBetween(targetPos1, targetPos2) {
		b.Fatal("cube1 and cube2 shoud not walkable")
	}

	b.ResetTimer()
	for n := 0; n < b.N; n++ {
		manager.WalkableBetween(sourcePos, targetPos3)
	}
}

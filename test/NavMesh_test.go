package main_test

import (
	"io/ioutil"
	"math"
	"testing"

	"github.com/lazytiger/unityai"
	"github.com/lazytiger/unityai/format"
)

func Test_SamplePosition(t *testing.T) {
	data, err := format.LoadFromGobFile("CSZ.asset.gob")
	if err != nil {
		println(err.Error())
	}

	nvData := unityai.NewDataFromFormat(data)
	manager := unityai.NewNavMeshManager()
	manager.LoadData(nvData)
	var hit unityai.NavMeshHit
	//filter.SetIncludeFlags(0)

	var sp1, sp2, sp3, sp4, sp5, sp6 unityai.Vector3f
	sp1.Set(61.9, 5.44082, 87.9)
	sp2.Set(70.2145, 5.09082, 97.4380)
	sp3.Set(63.02, 3.02, 102.38)
	sp4.Set(52.8, 6.69, 107.5)
	sp5.Set(63.5, 5.34082, 89.1000)
	sp6.Set(95.74, 5.95, 146.84)
	spArray := []unityai.Vector3f{sp1, sp2, sp3, sp4, sp5, sp6}

	/*
				client:
				Test_NavMeshSamplePosition sourcePos(61.9, 5.4, 87.9) (61.9, 5.4, 87.9) 9.059906E-06 (0.0, 0.0, 0.0) 8
			    Test_NavMeshSamplePosition sourcePos(70.2, 5.1, 97.4) (70.2, 5.1, 97.4) 9.536743E-06 (0.0, 0.0, 0.0) 8
		        Test_NavMeshSamplePosition sourcePos(63.0, 3.0, 102.4) (63.0, 6.6, 102.4) 3.562907 (0.0, 0.0, 0.0) 8
		        Test_NavMeshSamplePosition sourcePos(52.8, 6.7, 107.5) (59.7, 6.7, 107.5) 6.900002 (0.0, 0.0, 0.0) 8
		        Test_NavMeshSamplePosition sourcePos(63.5, 5.3, 89.1) (63.5, 5.3, 89.1) 1.184439E-05 (0.0, 0.0, 0.0) 8

				server
			     sourcePos {61.9 5.44082 87.9},hit={position:{x:61.9 y:5.44082 z:87.9} normal:{x:0 y:0 z:0} distance:0 mask:0 hit:true}
		    	 sourcePos {70.2145 5.09082 97.438},hit={position:{x:70.2145 y:5.09082 z:97.438} normal:{x:0 y:0 z:0} distance:0 mask:0 hit:true}
		    	 sourcePos {63.02 3.02 102.38},hit={position:{x:63.02 y:3.02 z:102.38} normal:{x:0 y:0 z:0} distance:0 mask:0 hit:true}
		    	 sourcePos {52.8 6.69 107.5} manager no SamplePosition
		    	 sourcePos {63.5 5.34082 89.1},hit={position:{x:63.5 y:5.34082 z:89.1} normal:{x:0 y:0 z:0} distance:0 mask:0 hit:true}
	*/
	for _, sourcePos := range spArray {
		if manager.SamplePosition(&hit, sourcePos, math.MaxFloat32) {
			t.Logf("sourcePos %v,hit=%+v", sourcePos, hit)
		} else {
			t.Errorf("sourcePos %v manager no SamplePosition", sourcePos)
			manager.SamplePosition(&hit, sourcePos, math.MaxFloat32)
		}
	}

}

func Benchmark_CalculatePath(b *testing.B) {
	//data, err := format.LoadFromGobFile("Chishazhen01.gob")
	data, err := format.LoadFromGobFile("CSZ.asset.gob")
	if err != nil {
		println(err.Error())
	}

	nvData := unityai.NewDataFromFormat(data)
	manager := unityai.NewNavMeshManager()
	manager.LoadData(nvData)
	manager.GetFilter().SetAreaCost(0, 1)
	manager.GetFilter().SetAreaCost(1, 1)
	manager.GetFilter().SetAreaCost(2, 2)
	manager.GetFilter().SetAreaCost(3, 2)
	manager.GetFilter().SetAreaCost(4, 1)

	/*
		client
		result True source (83.6, 0.0, 92.2) target (83.6, 5.1, 92.2) corners (92.8, 10.2, 114.6)(94.5, 11.0, 98.4)(92.3, 9.9, 95.9)(92.3, 9.9, 95.9)(89.8, 8.2, 95.3)(89.6, 8.2, 95.3)(87.7, 6.6, 94.5)(87.5, 6.6, 94.4)(86.2, 5.1, 93.1)(83.6, 5.1, 92.2) arenaMask -1
		server
		corners []
	*/

	var sourcePos unityai.Vector3f
	var targetPos unityai.Vector3f
	sourcePos.Set(92.8, 10.2, 114.6)
	targetPos.Set(83.6, 5.1, 92.2)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, partial := manager.CalculatePath(sourcePos, targetPos, 400)
		if partial {
			b.Fatal("found partial result")
		}
		//b.Logf("corners count:%d, data:%+v", len(corners), corners)
	}
}

func Test_CalculatePath(t *testing.T) {
	fmtData, err := format.LoadFromGobFile("CSZ.asset.gob")
	if err != nil {
		t.Error(err.Error())
	}

	nvData := unityai.NewDataFromFormat(fmtData)
	manager := unityai.NewNavMeshManager()
	manager.LoadData(nvData)
	manager.GetFilter().SetAreaCost(0, 1)
	manager.GetFilter().SetAreaCost(1, 1)
	manager.GetFilter().SetAreaCost(2, 2)
	manager.GetFilter().SetAreaCost(3, 2)
	manager.GetFilter().SetAreaCost(4, 1)

	/*
		client
		result True source (83.6, 0.0, 92.2) target (83.6, 5.1, 92.2) corners (92.8, 10.2, 114.6)(94.5, 11.0, 98.4)(92.3, 9.9, 95.9)(92.3, 9.9, 95.9)(89.8, 8.2, 95.3)(89.6, 8.2, 95.3)(87.7, 6.6, 94.5)(87.5, 6.6, 94.4)(86.2, 5.1, 93.1)(83.6, 5.1, 92.2) arenaMask -1
		server
		corners []
	*/

	var sourcePos unityai.Vector3f
	var targetPos unityai.Vector3f
	sourcePos.Set(92.8, 10.2, 114.6)
	targetPos.Set(83.6, 5.1, 92.2)
	//sourcePos.Set(168.1, 5.1, 124.3)
	//targetPos.Set(65.2, 5.2, 82.0)
	for i := int32(1); i < 10; i++ {
		corners, partial := manager.CalculatePath(sourcePos, targetPos, 50*i)
		t.Logf("iteration:%d, corners partial:%v count:%d, data:%+v", 50*i, partial, len(corners), corners)
	}
}

func Test_Raycast(t *testing.T) {
	data, err := format.LoadFromGobFile("CSZ.asset.gob")
	if err != nil {
		println(err.Error())
	}

	nvData := unityai.NewDataFromFormat(data)
	manager := unityai.NewNavMeshManager()
	manager.LoadData(nvData)

	var hit unityai.NavMeshHit
	var sourcePos unityai.Vector3f
	var targetPos unityai.Vector3f
	// 不可达的
	sourcePos.Set(48.3522, 3.292, 97.5603)
	targetPos.Set(48.3863, 4.30509, 105)
	// client output
	//Test_NavMeshRaycast source (48.4, 3.3, 97.6) target (48.4, 4.3, 105.0) blocked True hit (48.4, 3.5, 100.3)

	// 可达的
	//sourcePos.Set(45.7007, 3.63167, 99.9223)
	//targetPos.Set(48.3863, 4.30509, 96.5)
	// client output
	//Test_NavMeshRaycast source (45.7, 3.6, 99.9) target (48.4, 4.3, 96.5) blocked False hit (48.4, 3.3, 96.5)

	if manager.Raycast(&hit, sourcePos, targetPos) {
		t.Logf("Test_Raycast result false sourcePos %v targetPos %v hit %v", sourcePos, targetPos, hit)
	} else {
		t.Errorf("Test_Raycast result false sourcePos %v targetPos %v hit %v", sourcePos, targetPos, hit)
	}
}

func Test_MoveFindNearestPoly(t *testing.T) {
	data, err := format.LoadFromGobFile("LaomoJishi01.gob")
	if err != nil {
		println(err.Error())
	}

	nvData := unityai.NewDataFromFormat(data)
	manager := unityai.NewNavMeshManager()
	manager.LoadData(nvData)

	var tarPos unityai.Vector3f
	var hitPolyRef unityai.NavMeshPolyRef
	var hitPos unityai.Vector3f
	tarPos.Set(95.74, 5.95, 146.84)
	manager.FindNearestPoly(tarPos, &hitPolyRef, &hitPos)

	t.Logf("tarPos=%v,hitPos=%v,hitRef=%d", tarPos, hitPos, hitPolyRef)
}

func Test_MoveAlongSurface(t *testing.T) {
	data, err := format.LoadFromGobFile("CSZ.asset.gob")
	if err != nil {
		println(err.Error())
	}

	nvData := unityai.NewDataFromFormat(data)
	manager := unityai.NewNavMeshManager()
	manager.LoadData(nvData)

	var starPos unityai.Vector3f
	var histPos unityai.Vector3f
	var hitRef unityai.NavMeshPolyRef
	var endPos unityai.Vector3f
	starPos.Set(50.21729, 3.412016, 94.3856)
	endPos.Set(56.6443, 3.447872, 88.18468)
	manager.FindNearestPoly(starPos, &hitRef, &histPos)
	t.Logf("hisPos=%v,hitRef=%v", histPos, hitRef)

	var resultPos unityai.Vector3f
	var visited [32]unityai.NavMeshPolyRef
	var visitedCount, maxVisitedSize int32
	maxVisitedSize = 256

	var status unityai.NavMeshStatus
	status = manager.MoveAlongSurface(hitRef, histPos, endPos, &resultPos, visited[:], &visitedCount, maxVisitedSize)
	if !unityai.NavMeshStatusSucceed(status) {
		t.Errorf("manager.MoveAlongSurface failed startRef %v startPos %v endPos %v", hitRef, histPos, endPos)
	} else {
		t.Logf("resultPos=%v,visited=%v,visitedCount=%v,status=%v", resultPos, visited, visitedCount, status)
	}
}

func Benchmark_MoveAlongSurface(b *testing.B) {
	data, err := format.LoadFromGobFile("CSZ.asset.gob")
	if err != nil {
		println(err.Error())
	}

	nvData := unityai.NewDataFromFormat(data)
	manager := unityai.NewNavMeshManager()
	manager.LoadData(nvData)

	var starPos unityai.Vector3f
	var histPos unityai.Vector3f
	var hitRef unityai.NavMeshPolyRef
	var endPos unityai.Vector3f
	starPos.Set(92.3, 9.9, 95.9)
	endPos.Set(83.6, 5.1, 92.2)
	manager.FindNearestPoly(starPos, &hitRef, &histPos)
	//b.Logf("hisPos=%v,hitRef=%v", histPos, hitRef)

	var resultPos unityai.Vector3f
	var visited [256]unityai.NavMeshPolyRef
	var visitedCount, maxVisitedSize int32
	maxVisitedSize = 256
	var status unityai.NavMeshStatus
	b.ResetTimer()

	for i := 0; i < b.N; i++ {
		status = manager.MoveAlongSurface(hitRef, histPos, endPos, &resultPos, visited[:], &visitedCount, maxVisitedSize)
		if !unityai.NavMeshStatusSucceed(status) {
			b.Errorf("manager.MoveAlongSurface failed startRef %v startPos %v endPos %v", hitRef, histPos, endPos)
		} else {
			//t.Logf("resultPos=%v,visited=%v,visitedCount=%v,status=%v", resultPos, visited, visitedCount, status)
		}
	}
}

func Test_FindRandomPointAroundCircle(t *testing.T) {
	data, err := format.LoadFromGobFile("CSZ.asset.gob")
	if err != nil {
		t.Fatal(err)
	}

	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		t.Fatal(err)
	}

	var center unityai.Vector3f
	center.Set(92.8, 10.2, 114.6)
	for i := 0; i < 100; i++ {
		result, err := manager.FindRandomPointInCircle(center, 1)
		if err != nil {
			t.Fatal(err)
		}
		t.Logf("found point:%v", result)
	}
}

func Test_LoadNavMeshJsonFile(t *testing.T) {
	areaCosts, offMeshLinks, err := format.LoadNavMeshSceneDatFromJsonFile("Chishazhen01.json")
	if err != nil {
		t.Error(err)
	}
	t.Logf("areaCost %+v offMeshLins %+v", areaCosts, offMeshLinks)
}

func Test_LoadMavMeshFromGob(t *testing.T) {
	data, err := ioutil.ReadFile("Chishazhen01.gob")
	if err != nil {
		t.Errorf("ioutil.ReadFile failed %s", err)
	}

	navMeshData, err := format.LoadFromByteStream(data)
	if err != nil {
		t.Errorf("format.LoadFromByteStream failed %s", err)
	}

	t.Logf("navMeshData=%+v", navMeshData)
}

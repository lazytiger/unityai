package main_test

import (
	"github.com/lazytiger/unityai"
	"github.com/lazytiger/unityai/format"
	"encoding/gob"
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

func Test_DynamicObstacle1(t *testing.T) {
	//data, err := format.LoadFromTxtFile("Obstacle.txt")
	//data, err := format.LoadFromGobFile("CSZ_Wenquan_01.gob")
	//data, err := format.LoadFromGobFile("CSZ_chishalukou.gob")
	//data, err := format.LoadFromGobFile("Chishazhen01.gob")
	data, err := format.LoadFromGobFile("NavMesh.asset.gob")
	if err != nil {
		t.Fatal(err.Error())
	}

	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		t.Fatal(err)
	}

	N := 10000
	rand.Seed(time.Now().Unix())

	M := 0
	for i := 0; i < N; i++ {
		var hit unityai.NavMeshHit
		origin := unityai.NewVector3f(rand.Float32(), rand.Float32(), rand.Float32()).Mulf(500)
		if !manager.SamplePosition(&hit, origin, 999) {
			t.Logf("position:%+v map to NavMesh failed", origin)
			continue
		}

		pos := hit.GetPosition()
		rotation := unityai.NewVector3f(rand.Float32(), rand.Float32() ,rand.Float32()).Mulf(180)
		scale := unityai.NewVector3f(rand.Float32(), rand.Float32(), rand.Float32()).Mulf(10)
		size := unityai.NewVector3f(rand.Float32(), rand.Float32(), rand.Float32()).Mulf(10)

		//t.Logf("obstacle position:%+v", pos)

		obs := unityai.NewNavMeshObstacle(unityai.NavMeshObstacleShape(1), pos, scale, unityai.EulerToQuaternionUnity(rotation))
		obs.SetSize(size)
		handle := manager.AddObstacle(obs)
		if !manager.UpdateCarvingImmediately() {
			t.Errorf("Obstacle did not carve NavMesh:%#v, %#v", origin, pos)
			M++
		}

		manager.RemoveObstacle(handle)
		manager.UpdateCarvingImmediately()
	}
	t.Logf("total test:%v, failed:%v", N, M)
}

func Test_Plane(t *testing.T) {

	data, err := format.LoadFromTxtFile("Obstacle.txt")
	if err != nil {
		println(err.Error())
	}

	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		t.Fatal(err)
	}
	pos := unityai.NewVector3f(3.5, 0.083, 3.83)
	rotation := unityai.NewQuaternionf(0, 0, 0, 1)
	scale := unityai.Vector3_One

	obs := unityai.NewNavMeshObstacle(unityai.NavMeshObstacleShape(1), pos, scale, rotation)
	handle := manager.AddObstacle(obs)
	if !manager.UpdateCarvingImmediately() {
		t.Errorf("Obstacle did not carve NavMesh:%+v", pos)
	}
	manager.RemoveObstacle(handle)
	manager.UpdateCarvingImmediately()
}

func Test_Panic(t *testing.T) {
	data, err := format.LoadFromGobFile("CSZ_Wenquan_01.gob")
	if err != nil {
		println(err.Error())
	}

	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		t.Fatal(err)
	}
	pos := unityai.NewVector3f(20.31487, -0.94257045, 37.88024)
	rotation := unityai.NewQuaternionf(0, 0, 0, 1)
	scale := unityai.Vector3_One

	obs := unityai.NewNavMeshObstacle(unityai.NavMeshObstacleShape(1), pos, scale, rotation)
	handle := manager.AddObstacle(obs)
	if !manager.UpdateCarvingImmediately() {
		t.Errorf("Obstacle did not carve NavMesh:%+v", pos)
	}
	manager.RemoveObstacle(handle)
	manager.UpdateCarvingImmediately()
}

func Test_Remove(t *testing.T) {
	data, err := format.LoadFromGobFile("CSZ_Wenquan_01.gob")
	if err != nil {
		println(err.Error())
	}

	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		t.Fatal(err)
	}

	surfaceId := manager.GetSurfaceId()
	for i :=0; i< manager.GetMaxTileIndex();i++ {
		manager.RemoveTile(surfaceId, int32(i))
		manager.RestoreTile(surfaceId, int32(i))
	}

}

func Test_ExportAddData(t *testing.T) {
	data, err := format.LoadFromGobFile("CSZ_Wenquan_01.gob")
	if err != nil {
		t.Fatal(err)
	}

	addData, err := format.LoadNavMeshSceneDataFromJsonFile("CSZ_Wenquan_01.json")
	if err != nil {
		t.Fatal(err)
	}
	data.M_AdditionalData = *addData

	f, err := os.OpenFile("CSZ_Wenquan_01.gob", os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0644)
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	err = gob.NewEncoder(f).Encode(data)
	if err != nil {
		t.Fatal(err)
	}
}

func Test_DynamicObstacle2(t *testing.T) {
	data, err := format.LoadFromGobFile("CSZ_Wenquan_01.gob")
	if err != nil {
		t.Fatal(err)
	}

	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		t.Fatal(err)
	}

	obsInfo := data.M_AdditionalData.GetObstacle("CSZ_Wenquan_obs_04")
	obs := unityai.NewNavMeshObstacleFromFormat(obsInfo)
	handler := manager.AddObstacle(obs)
	if !manager.UpdateCarvingImmediately() {
		t.Errorf("obstacle carved nothing")
	}
	t.Logf("handler %v", handler)

	// 验证结果
	// source (43.5, -1.2, 37.8) target (55.9, 0.8, 36.6) blocked True hit (46.5, -1.2, 37.5) // 【客户端数据】阻挡开启Raycast结果
	// source (43.5, -1.2, 37.8) target (55.9, 0.8, 36.6) blocked False hit (55.9, 0.8, 36.6) // 【客户端数据】阻挡关闭Raycast结果
	var hit unityai.NavMeshHit
	var sourcePos unityai.Vector3f
	var targetPos unityai.Vector3f
	sourcePos.Set(43.5, -1.2, 37.8)
	targetPos.Set(55.9, 0.8, 36.6)

	result := manager.Raycast(&hit, sourcePos, targetPos)
	if !result {
		t.Errorf("Test_Raycast with obstacle should hit, result %v sourcePos %v targetPos %v hit %v", result, sourcePos, targetPos, hit)
	}

	manager.RemoveObstacle(handler)
	manager.UpdateCarvingImmediately()
	result = manager.Raycast(&hit, sourcePos, targetPos)
	if result {
		t.Errorf("Test_Raycast without obstacle should pass,  result %v sourcePos %v targetPos %v hit %v", result, sourcePos, targetPos, hit)
	}
}

func Test_DynamicObstacle3(t *testing.T) {
	data, err := format.LoadFromGobFile("CSZ_Wenquan_01.gob")
	if err != nil {
		println(err.Error())
	}

	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		t.Fatal(err)
	}

	/*		第一组数据
				 Scale (8.877,5.181,1)
				 obsPos(47.7, 4.3, 37.6)
				 height 1
				 radius 0.5
				 velocity (0.0, 0.0, 0.0)
				 carving True
				 carveOnlyStationary True
				 carvingMoveThreshold 0.1
				 carvingTimeToStationary 0.5
				 NavMeshObstacleShape Box  Box enum NavMeshObstacleShape { Capsule = 0,Box = 1}
				 center (47.7, 4.3, 37.6)
				 size(1.0, 2.0, 1.0)
			     xAxjs (1.0, 0.0, 0.0)
			     yAxjs (0.0, 1.0, 0.0)
			     zAxjs(0.0, 0.0, 1.0)
			     rotation (0.0, -0.7, 0.0, 0.7)
	*/

	var handlerList []int32
	obsInfo := data.M_AdditionalData.GetObstacle("CSZ_Wenquan_obs_04")
	obs := unityai.NewNavMeshObstacleFromFormat(obsInfo)
	handle := manager.AddObstacle(obs)
	handlerList = append(handlerList, handle)
	if !manager.UpdateCarvingImmediately() {
		t.Errorf("obstacle carved nothing")
	}
	t.Logf("handlerlist %v", handlerList)

	// 验证结果
	// source (43.5, -1.2, 37.8) target (55.9, 0.8, 36.6) blocked True hit (46.5, -1.2, 37.5) // 【客户端数据】阻挡开启Raycast结果
	// source (43.5, -1.2, 37.8) target (55.9, 0.8, 36.6) blocked False hit (55.9, 0.8, 36.6) // 【客户端数据】阻挡关闭Raycast结果
	var hit unityai.NavMeshHit
	var sourcePos unityai.Vector3f
	var targetPos unityai.Vector3f
	sourcePos.Set(43.5, -1.2, 37.8)
	targetPos.Set(55.9, 0.8, 36.6)
	result := manager.Raycast(&hit, sourcePos, targetPos)
	if !result {
		t.Errorf("Test_Raycast not hit, result %v sourcePos %v targetPos %v hit %v", result, sourcePos, targetPos, hit)
	}

	/*
	 第二组数据
	 abScale (8.878, 5.181, 1.0)
	 obsPos(78.0, 0.8, 51.9)
	 height 0.5
	 radius 0.5
	 velocity (0.0, 0.0, 0.0)
	 carving True
	 carveOnlyStationary True
	 carvingMoveThreshold 0.1
	 carvingTimeToStationary 0.5
	 NavMeshObstacleShape Box
	 center (78.0, 0.8, 51.9)
	 size(1.0, 1.0, 1.0)
	 rotation (0.0, 0.0, 0.0, 1.0)
	*/

	obsInfo = data.M_AdditionalData.GetObstacle("CSZ_Wenquan_obs_03")
	obs = unityai.NewNavMeshObstacleFromFormat(obsInfo)
	handle = manager.AddObstacle(obs)
	handlerList = append(handlerList, handle)
	if !manager.UpdateCarvingImmediately() {
		t.Errorf("obstacle2 carved nothing")
	}
	t.Logf("handlerlist %v", handlerList)

	// 验证结果
	// source (77.82066, -1.24257, 47.56031) target (78.20289, -1.445682, 57.93384) blocked True hit (46.5, -1.2, 37.5) // 【客户端数据】阻挡开启Raycast结果
	// source (77.82066, -1.24257, 47.56031) target (78.20289, -1.445682, 57.93384) blocked False hit (55.9, 0.8, 36.6) // 【客户端数据】阻挡关闭Raycast结果
	var hit1 unityai.NavMeshHit
	var sourcePos1 unityai.Vector3f
	var targetPos1 unityai.Vector3f
	sourcePos1.Set(77.82066, -1.24257, 47.56031)
	targetPos1.Set(78.20289, -1.445682, 57.93384)
	result = manager.Raycast(&hit1, sourcePos1, targetPos1)
	if !result {
		t.Errorf("Test_Raycast not hit result %v sourcePos %v targetPos %v hit %v", result, sourcePos1, targetPos1, hit1)
	}

	manager.RemoveObstacle(handlerList[0])
	manager.UpdateCarvingImmediately()
	result = manager.Raycast(&hit, sourcePos, targetPos)
	if result {
		t.Errorf("Test_Raycast hit, result %v sourcePos %v targetPos %v hit %v", result, sourcePos, targetPos, hit)
	}

	manager.RemoveObstacle(handlerList[1])
	manager.UpdateCarvingImmediately()
	result = manager.Raycast(&hit1, sourcePos1, targetPos1)
	if result {
		t.Errorf("Test_Raycast hit, result %v sourcePos %v targetPos %v hit %v", result, sourcePos1, targetPos1, hit1)
	}

}

func Test_DynamicObstacle4(t *testing.T) {
	data, err := format.LoadFromGobFile("CSZ_Wenquan_01.gob")
	if err != nil {
		println(err.Error())
	}
	t.Logf("Obstacles:%+v", data.M_AdditionalData)

	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		t.Fatal(err)
	}

	var handleList []int32
	for _, obsInfo := range data.M_AdditionalData.ObsLists {
		t.Logf("obsInfo %+v", obsInfo)
		obs := unityai.NewNavMeshObstacleFromFormat(obsInfo)
		handle := manager.AddObstacle(obs)
		handleList = append(handleList, handle)
		if !manager.UpdateCarvingImmediately() {
			t.Errorf("obstacle carved nothing")
		}
		//manager.RemoveObstacle(handle)
	}

	t.Logf("handlerlist %v", handleList)

}

func Test_DynamicObstacle5(t *testing.T) {
	// 动态npc
	data, err := format.LoadFromGobFile("Chishazhen01.gob")
	if err != nil {
		println(err.Error())
	}

	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		t.Fatal(err)
	}

	/*
		   npc 310639 CreateAndEnterZone pos={3333.043 514.16486 11581.816 0},dir={0.43208575 0 0.9018325 0}
		   source (31.2, 5.4, 111.3) target (33.5, 5.1, 116.7) blocked True hit (32.9, 5.2, 115.3)
		   source (31.2, 5.4, 111.3) target (33.5, 5.1, 116.7) blocked False hit (33.5, 5.1, 116.7)


			 abScale (1.0, 1.0, 1.0)
			 obsPos(33.3, 5.1, 115.8)
			 height 0.25
			 radius 1
			 velocity (0.0, 0.0, 0.0)
			 carving True
			 carveOnlyStationary True
			 carvingMoveThreshold 0.1
			 carvingTimeToStationary 0.5
			 NavMeshObstacleShape Box
			 center (33.3, 5.1, 115.8)
			 size(2.0, 0.5, 1.0)
			 rotation (0.0, 0.2, 0.0, 1.0) eulerAngles (0.0, 25.6, 0.0), dir (0.4, 0.0, 0.9)
	*/
	var handleList []int32

	dir := unityai.NewVector3f(31.2, 5.4137642, 111.3)
	radian := math.Atan2(float64(dir.GetData(0)), float64(dir.GetData(2)))
	eulerAngles := unityai.NewVector3f(0, float32(radian), 0)
	rotation := unityai.EulerToQuaternion(eulerAngles, unityai.OrderUnity)
	t.Logf("rotation is %v", rotation)

	size := unityai.NewVector3f(2.0, 0.5, 1)
	obs := unityai.NewNavMeshObstacle(1, dir, unityai.Vector3_One, rotation)
	obs.SetSize(size)
	obs.SetCenter(unityai.NewVector3f(0, 0, 0))
	handleList = append(handleList, manager.AddObstacle(obs))
	if !manager.UpdateCarvingImmediately() {
		t.Errorf("obstacle carved nothing")
	}
	t.Logf("handleList %v rotatioin %v ddir %v", handleList, rotation, eulerAngles)

	var hit1 unityai.NavMeshHit
	var sourcePos1 unityai.Vector3f
	var targetPos1 unityai.Vector3f
	sourcePos1.Set(31.2, 5.4, 111.3)
	targetPos1.Set(33.5, 5.1, 116.7)
	result := manager.Raycast(&hit1, sourcePos1, targetPos1)
	if !result {
		t.Errorf("Test_Raycast not hit, result %v sourcePos %v targetPos %v hit %v", result, sourcePos1, targetPos1, hit1)
	}

	manager.RemoveObstacle(handleList[0])
	manager.UpdateCarvingImmediately()
	result = manager.Raycast(&hit1, sourcePos1, targetPos1)
	if result {
		t.Errorf("Test_Raycast hit, result %v sourcePos %v targetPos %v hit %v", result, sourcePos1, targetPos1, hit1)
	}

}

func Test_Obstacle(t *testing.T) {
	data, err := format.LoadFromGobFile("CSZ_Wenquan_01.gob")
	if err != nil {
		println(err.Error())
	}

	sp := unityai.NewVector3f(31.62, -1.24, 44.60)

	obsInfo := data.M_AdditionalData.GetObstacle("CSZ_Wenquan_obs_01")
	t.Logf("obsInfo %+v", obsInfo)
	obs := unityai.NewNavMeshObstacleFromFormat(obsInfo)
	var shape unityai.NavMeshCarveShape
	obs.GetCarveShape(&shape)
	bounds := shape.GetBounds()
	t.Logf("Bounds:%+v", bounds)
	if bounds.Contains(sp) {
		t.Logf("source point is in obstacle")
	} else {
		t.Logf("source point is not in obstacle")
	}
}

func Test_DynamicObstacle6(t *testing.T) {
	data, err := format.LoadFromGobFile("CSZ_Wenquan_01.gob")
	if err != nil {
		println(err.Error())
	}

	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		t.Fatal(err)
	}

	obsInfo := data.M_AdditionalData.GetObstacle("CSZ_Wenquan_obs_04")
	t.Logf("obsInfo %+v", obsInfo)
	handle := manager.AddObstacle(unityai.NewNavMeshObstacleFromFormat(obsInfo))
	if !manager.UpdateCarvingImmediately() {
		t.Errorf("obstacle carved nothing")
	}

	sp := unityai.NewVector3f(41.37943, -1.24257, 33.38129)
	ep := unityai.NewVector3f(41.71702, -1.227093, 40.01076)

	//sp1 := unityai.NewVector3f(24.99, -1.26, 38.48)
	//ep1 := unityai.NewVector3f(25.198,-1.173, 28.643)
	corners, _ := manager.CalculatePath(sp, ep, 500)
	t.Logf("add Obs corners %+v", corners)

	var hit unityai.NavMeshHit
	result := manager.Raycast(&hit, sp, ep)
	if !result {
		t.Errorf("Test_Raycast add Obs not hit, result %v sourcePos %v targetPos %v hit %+v", result, sp, ep, hit)
	}

	manager.RemoveObstacle(handle)
	manager.UpdateCarvingImmediately()
	corners, _ = manager.CalculatePath(sp, ep, 500)
	t.Logf("remove Obs corners %v", corners)

	result = manager.Raycast(&hit, sp, ep)
	if result {
		t.Errorf("Test_Raycast remove Obs hit,  result %v sourcePos %v targetPos %v hit %+v", result, sp, ep, hit)
	}
}

func Benchmark_DynamicObstacle(b *testing.B) {
	data, err := format.LoadFromGobFile("CSZ_chishalukou.gob")
	if err != nil {
		println(err.Error())
	}

	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		b.Fatal(err)
	}
	position := unityai.NewVector3f(47.7, 4.3, 37.6)
	rotation := unityai.NewQuaternionf(0, 1, 0, 1)
	scale := unityai.NewVector3f(8.877, 5.181, 1)
	obs := unityai.NewNavMeshObstacle(1, position, scale, rotation)

	for i := 0; i < b.N; i++ {
		handler := manager.AddObstacle(obs)
		if !manager.UpdateCarvingImmediately() {
			b.Errorf("carve failed")
		}
		manager.RemoveObstacle(handler)
		manager.UpdateCarvingImmediately()
	}
}

func Benchmark_DynamicObstacle1(b *testing.B) {
	funWalk := func(path string, info os.FileInfo, err error) error {
		if info.IsDir() {
			return nil
		}

		if !strings.HasSuffix(info.Name(), ".gob") {
			return nil
		}

		data, err := format.LoadFromGobFile(path)
		if err != nil {
			b.Fatal(err)
			return nil
		}

		if len(data.M_AdditionalData.ObsLists) <= 0 {
			return nil
		}

		manager, err := unityai.NewManagerFromData(data)
		if err != nil {
			b.Fatal(err)
			return nil
		}

		b.Run(info.Name(), func(b *testing.B) {
			for _, obsInfo := range data.M_AdditionalData.ObsLists {
				obs := unityai.NewNavMeshObstacleFromFormat(obsInfo)
				b.Run(obsInfo.Name, func(b *testing.B) {
					for i := 0; i < b.N; i++ {
						handler := manager.AddObstacle(obs)
						manager.UpdateCarvingImmediately()
						manager.RemoveObstacle(handler)
						manager.UpdateCarvingImmediately()
					}
				})
			}
		})

		return nil
	}

	path := "F:\\project\\mmo\\BOTG_Server\\src\\bin\\gameconfig\\public\\navmesh\\"
	err := filepath.Walk(path, funWalk)
	if err != nil {
		b.Errorf("filepath.Walk %s failed %s", path, err.Error())
	}
}

func Test_DeepClone(t *testing.T) {
	data, err := format.LoadFromTxtFile("Obstacle.txt")
	if err != nil {
		println(err.Error())
	}

	manager, err := unityai.NewManagerFromData(data)
	if err != nil {
		t.Fatal(err)
	}

	manager = manager.DeepClone()

	position := unityai.NewVector3f(0, 5, 0)
	rotation := unityai.NewQuaternionf(0, 0, 0, 1)
	size := unityai.NewVector3f(55, 10, 1)
	obs := unityai.NewNavMeshObstacle(1, position, size, rotation)
	handler := manager.AddObstacle(obs)
	manager.UpdateCarvingImmediately()
	t.Logf("handler %v", handler)

	var hit unityai.NavMeshHit
	var sourcePos unityai.Vector3f
	var targetPos unityai.Vector3f
	sourcePos.Set(0, 0, -22.23)
	targetPos.Set(0, 0, 16.67)
	if !manager.Raycast(&hit, sourcePos, targetPos) {
		t.Errorf("Test_Raycast result false sourcePos %v targetPos %v hit %v", sourcePos, targetPos, hit)
	}

	manager.RemoveObstacle(handler)
	manager.UpdateCarvingImmediately()
	if manager.Raycast(&hit, sourcePos, targetPos) {
		t.Errorf("Test_Raycast result false sourcePos %v targetPos %v hit %v", sourcePos, targetPos, hit)
	}
}

func TestLossyScale(t *testing.T) {
	center := unityai.NewVector3f(47.7, 4.3, 37.6)
	rotation := unityai.NewQuaternionf(0.0, -0.7, 0.0, 0.7)
	scale := unityai.NewVector3f(45.1, 16.2, 49.3)
	var matrix unityai.Matrix4x4f
	matrix.SetTRS(center, rotation, scale)
	abScale := unityai.NewVector3f(11.8, 6.0, 2.1)
	t.Logf("%v, %v, %v, %v", abScale, matrix.Get(0, 0), matrix.Get(1, 1), matrix.Get(2, 2))
	t.Logf("%v", matrix.GetLossyScale())
}

func Test_EulerToQuaternion(t *testing.T) {
	q := unityai.EulerToQuaternionUnity(unityai.NewVector3f(0, 30, 0))
	t.Logf("%v, %v, %v, %v", q.X(), q.Y(), q.Z(), q.W())
}

package main

import (
	"fmt"

	"github.com/lazytiger/unityai"
	"github.com/lazytiger/unityai/format"
)

func main() {
	var a uint8 = 0x01
	a &= ^uint8(0x01)
	a |= 0x02
	fmt.Printf("%x\n", a)
	return
	data, err := format.LoadFromTxtFile("CSZ.asset.txt")
	if err != nil {
		println(err.Error())
	}
	fmt.Printf("%+v\n", len(data.M_NavMeshTiles))
	nvData := unityai.NewDataFromFormat(data)
	manager := unityai.NewNavMeshManager()
	manager.LoadData(nvData)
	var hit unityai.NavMeshHit
	var sourcePos, targetPos, nSource, nTarget unityai.Vector3f
	sourcePos.Set(100, 6, 68)
	if manager.SamplePosition(&hit, sourcePos, 100) {
		fmt.Printf("hit found:%+v\n", hit)
		nSource = hit.GetPosition()
	} else {
		println("nothing hit")
	}
	targetPos.Set(100, 10, 100)
	if manager.Raycast(&hit, sourcePos, targetPos) {
		fmt.Printf("hit found:%+v\n", hit)
		nTarget = hit.GetPosition()
	} else {
		println("nothing hit")
	}

	path := unityai.NewNavMeshPath()
	count := manager.CalculatePolygonPath(path, nSource, nTarget, 30)
	if count > 0 {
		fmt.Printf("count:%d, path:%+v\n", count, path)
	} else {
		fmt.Printf("no path found\n")
	}

	corners := make([]unityai.Vector3f, count)
	count = manager.CalculatePathCorners(corners, count, path)
	if count > 0 {
		fmt.Printf("count:%d, path:%+v\n", count, corners)
	} else {
		println("not found")
	}
}

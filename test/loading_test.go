package main_test

import (
	"bytes"
	"encoding/gob"
	"testing"

	"github.com/lazytiger/unityai"
	"github.com/lazytiger/unityai/format"
)

func BenchmarkLoading(b *testing.B) {
	for i := 0; i < b.N; i++ {
		_, err := format.LoadFromGobFile("NavMesh.asset.gob")
		if err != nil {
			b.Fatal(err)
		}
	}
}

func text2binary(name string) error {
	data, err := format.LoadFromTxtFile(name + ".txt")
	if err != nil {
		return err
	}

	return format.SaveToGobFile(data, name+".gob")
}

func TestSave(t *testing.T) {
	err := text2binary("NavMesh.asset")
	if err != nil {
		t.Error(err)
		t.FailNow()
	}
	err = text2binary("CSZ.asset")
	if err != nil {
		t.Error(err)
		t.FailNow()
	}
	data, err := format.LoadFromGobFile("NavMesh.asset.gob")
	if err != nil {
		t.Fatal(err)
	}

	if len(data.M_NavMeshTiles[0].M_MeshData) != 476 {
		t.FailNow()
	}

	if data.M_NavMeshBuildSettings.AgentSlope != 60 {
		t.FailNow()
	}

	if len(data.M_HeightMeshes) != 1 {
		t.FailNow()
	}

	if len(data.M_HeightMeshes[0].M_Vertices) != 1610 {
		t.FailNow()
	}

	if len(data.M_HeightMeshes[0].M_Indices) != 5067 {
		t.FailNow()
	}

	if len(data.M_HeightMeshes[0].M_Nodes) != 255 {
		t.FailNow()
	}

	nvData := unityai.NewDataFromFormat(data)
	manager := unityai.NewNavMeshManager()
	manager.LoadData(nvData)
	var hit unityai.NavMeshHit
	var sourcePos unityai.Vector3f
	sourcePos.Set(62, 1, 98)
	if !manager.SamplePosition(&hit, sourcePos, 100) {
		t.Fatal("SamplePosition failed")
	}
}

func BenchmarkRaycast(b *testing.B) {
	data, err := format.LoadFromGobFile("NavMesh.asset.gob")
	if err != nil {
		b.Fatal(err)
	}
	nvData := unityai.NewDataFromFormat(data)
	manager := unityai.NewNavMeshManager()
	manager.LoadData(nvData)
	for i := 0; i < b.N; i++ {
		var hit unityai.NavMeshHit
		var sourcePos unityai.Vector3f
		sourcePos.Set(62, 1, 98)
		manager.Clone().SamplePosition(&hit, sourcePos, 100)
	}
}

func TestLoading(t *testing.T) {
	var content []byte
	var odata, ndata *format.NavMeshData
	var err error
	var i int
	N := 10
	for i = 0; i < N; i++ {
		ndata, err = format.LoadFromTxtFile("CSZ.asset.txt")
		if err != nil {
			t.Error(err)
		}
		buffer := bytes.NewBuffer(nil)
		encoder := gob.NewEncoder(buffer)
		err = encoder.Encode(ndata)
		if err != nil {
			t.Error(err)
		}
		if content == nil {
			content = buffer.Bytes()
			odata = ndata
		} else {
			if len(content) != buffer.Len() {
				t.Logf("content length not match -> [%d, %d]", len(content), buffer.Len())
				break
			}

			for pos, b := range buffer.Bytes() {
				if b != content[pos] {
					t.Logf("content not match at pos:%d", pos)
					break
				}
			}
		}
	}
	if i != N {
		t.Errorf("old data:%+v", odata)
		t.Errorf("new data:%+v", ndata)
	}
}

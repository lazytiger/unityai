package format

import (
	"bufio"
	"bytes"
	"encoding/gob"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"reflect"
	"regexp"
	"strconv"
	"strings"
	"unicode"
)

type NavMeshBuildSettings struct {
	AgentTypeID           int32
	AgentRadius           float32
	AgentHeight           float32
	AgentSlope            float32
	AgentClimb            float32
	LedgeDropHeight       float32
	MaxJumpAcrossDistance float32

	// Advanced
	MinRegionArea     float32
	ManualCellSize    int32
	CellSize          float32
	ManualTileSize    int32
	TileSize          int32
	AccuratePlacement int32
}

type AABB struct {
	M_Center Vector3f
	M_Extent Vector3f
}

type Vector3f struct {
	X, Y, Z float32
}

func (this *Vector3f) Set(i int, v float32) {
	switch i {
	case 0:
		this.X = v
	case 1:
		this.Y = v
	case 2:
		this.Z = v
	}
}

type Quaternionf struct {
	X, Y, Z, W float32
}

type NavMeshData struct {
	M_NavMeshBuildSettings NavMeshBuildSettings
	M_NavMeshTiles         []NavMeshTileData
	M_HeightMeshes         []HeightMeshData
	M_OffMeshLinks         []AutoOffMeshLinkData
	M_FilterAreaCosts      []float32

	M_SourceBounds AABB
	M_Rotation     Quaternionf
	M_Position     Vector3f
	M_AgentTypeID  int32
}

func (this *NavMeshData) SetSceneData(areaCosts []float32, offMeshLinks []AutoOffMeshLinkData) {
	this.M_FilterAreaCosts = areaCosts
	this.M_OffMeshLinks = offMeshLinks
}

type NavMeshTileData struct {
	M_MeshData []byte
	M_Hash     [16]byte
}

type HeightMeshData struct {
	M_Vertices []Vector3f
	M_Indices  []int32
	M_Nodes    []HeightMeshBVNode
	M_Bounds   AABB
}

type AutoOffMeshLinkData struct {
	M_Start         Vector3f
	M_End           Vector3f
	M_Radius        float32
	M_LinkType      uint16 // Off-mesh poly flags.
	M_Area          byte   // Off-mesh poly  area ids.
	M_LinkDirection byte   // Off-mesh connection direction flags (NavMeshLinkDirectionFlags)
}

type HeightMeshBVNode struct {
	Min, Max Vector3f
	I, N     int32
}

func SaveToGobFile(data *NavMeshData, file string) error {
	f, err := os.OpenFile(file, os.O_CREATE|os.O_TRUNC|os.O_WRONLY, 0700)
	if err != nil {
		return err
	}
	defer f.Close()
	return gob.NewEncoder(f).Encode(data)
}

func LoadFromGobFile(file string) (*NavMeshData, error) {
	f, err := os.Open(file)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	var data NavMeshData
	err = gob.NewDecoder(f).Decode(&data)
	if err != nil {
		return nil, err
	} else {
		return &data, nil
	}
}

func LoadFromByteStream(byteStream []byte) (*NavMeshData, error) {
	var data NavMeshData

	var buff bytes.Buffer
	_, err := buff.Write(byteStream)
	if err != nil {
		return nil, err
	}

	err = gob.NewDecoder(&buff).Decode(&data)
	if err != nil {
		return nil, err
	} else {
		return &data, nil
	}
}

func LoadFromTxtFile(file string) (*NavMeshData, error) {
	f, err := os.Open(file)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	reader := bufio.NewReader(f)
	data := new(NavMeshData)

	match1, err := regexp.Compile(`\s*([^\s]+) (.*) \((.+)\)`)
	if err != nil {
		return nil, err
	}

	match2, err := regexp.Compile(`\s*([^\s]+) \(([^\s]+)\) #[0-9]+: (.+)`)

	for {
		line, prefix, err := reader.ReadLine()
		if err == io.EOF {
			return data, nil
		} else if err != nil {
			return nil, err
		} else if prefix {
			return nil, fmt.Errorf("buffer is too small")
		}
		fields := strings.Fields(string(line))
		nfields := len(fields)
		if nfields > 0 && fields[nfields-1] == "NavMeshData" {
			err = readType(reader, match1, match2, reflect.ValueOf(data).Elem(), 1)
			return data, err
		}
	}
}

func LoadNavMeshSceneDatFromJsonFile(file string) ([]float32, []AutoOffMeshLinkData, error) {
	f, err := os.Open(file)
	if err != nil {
		return nil, nil, err
	}
	defer f.Close()

	jsonData := make(map[string][]interface{})
	err = json.NewDecoder(f).Decode(&jsonData)
	if err != nil {
		return nil, nil, err
	}

	areaCosts, err := parseAreaCosts(jsonData["areaCosts"])
	if err != nil {
		return nil, nil, err
	}

	offMeshLinks, err := parseOffMeshLinks(jsonData["offMeshLinks"])
	if err != nil {
		return nil, nil, err
	}

	return areaCosts, offMeshLinks, nil
}

func parseAreaCosts(params []interface{}) ([]float32, error) {
	areaCosts := make([]float32, 0, 32)
	for _, param := range params {
		areaCost, ok := param.(float64)
		if !ok {
			return nil, errors.New("parseAreaCosts param.(float64) failed")
		}
		areaCosts = append(areaCosts, float32(areaCost))
	}
	return areaCosts, nil
}

func parseOffMeshLinks(params []interface{}) ([]AutoOffMeshLinkData, error) {
	var offMeshLinks []AutoOffMeshLinkData
	for _, param := range params {
		paramMap, ok := param.(map[string]interface{})
		if !ok {
			return nil, errors.New("offMeshLinkI.(map[string]interface{}) failed")
		}

		area, ok := paramMap["area"].(float64)
		if !ok {
			return nil, errors.New("paramMap[\"area\"].(float64) failed")
		}

		biDirectional, ok := paramMap["biDirectional"].(bool)
		if !ok {
			return nil, errors.New("paramMap[\"biDirectional\"].(bool) failed")
		}

		startPos, ok := paramMap["startPos"].(map[string]interface{})
		if !ok {
			return nil, errors.New("paramMap[\"startPos\"].(map[string]interface{})failed")
		}

		endPos, ok := paramMap["endPos"].(map[string]interface{})
		if !ok {
			return nil, errors.New("paramMap[\"endPos\"].(map[string]interface{}) failed")
		}

		startPosX, ok := startPos["x"].(float64)
		if !ok {
			return nil, errors.New("startPos[\"x\"].(float64) failed")
		}

		startPosY, ok := startPos["y"].(float64)
		if !ok {
			return nil, errors.New("startPos[\"y\"].(float64) failed")
		}

		startPosZ, ok := startPos["z"].(float64)
		if !ok {
			return nil, errors.New("startPos[\"z\"].(float64) failed")
		}

		endPosX, ok := endPos["x"].(float64)
		if !ok {
			return nil, errors.New("endPos[\"x\"].(float64) failed")
		}

		endPosY, ok := endPos["y"].(float64)
		if !ok {
			return nil, errors.New("endPos[\"y\"].(float64) failed")
		}

		endPosZ, ok := endPos["z"].(float64)
		if !ok {
			return nil, errors.New("endPos[\"z\"].(float64) failed")
		}

		var linkDirection byte
		if biDirectional {
			linkDirection = 1
		}
		offMeshLinks = append(offMeshLinks, AutoOffMeshLinkData{
			M_Area:          byte(area),
			M_LinkDirection: byte(linkDirection),
			M_Start:         Vector3f{float32(startPosX), float32(startPosY), float32(startPosZ)},
			M_End:           Vector3f{float32(endPosX), float32(endPosY), float32(endPosZ)},
		})
	}
	return offMeshLinks, nil
}

func readLine(reader *bufio.Reader, match *regexp.Regexp, tab int) (fieldName, fieldValue, fieldType string, err error) {
	var head []byte
	head, err = reader.Peek(tab + 1)
	if err != nil {
		return
	}

	if head[0] == '\r' || head[0] == '\n' {
		//空行直接跳過
		reader.ReadLine()
		return readLine(reader, match, tab)
	}

	// 检查tab的数目，如果小于指定的则表示当前的type已经读完了
	for i := 0; i < tab; i++ {
		if head[i] != '\t' {
			err = io.EOF
			return
		}
	}

	// 如果当前的tab大于指定的，则表示这是个子类型
	if head[tab] == '\t' {
		err = io.EOF
		return
	}

	line, prefix, err := reader.ReadLine()
	if err != nil {
		return
	} else if prefix {
		err = fmt.Errorf("buffer is too small")
		return
	}

	fields := match.FindStringSubmatch(string(line))
	if len(fields) < 4 {
		err = fmt.Errorf("invalid line:%s", string(line))
		return
	} else {
		v := []rune(fields[1])
		v[0] = unicode.ToUpper(v[0])
		fieldName = string(v)
		fieldValue = fields[2]
		fieldType = fields[3]
	}
	return
}

func readType(reader *bufio.Reader, match1, match2 *regexp.Regexp, v reflect.Value, tab int) error {
	for {
		fieldName, fieldValue, fieldType, err := readLine(reader, match1, tab)
		if err == io.EOF {
			return nil
		} else if err != nil {
			return err
		}
		if fieldValue == "" {
			if !v.IsValid() || !v.FieldByName(fieldName).IsValid() {
				err := readType(reader, match1, match2, reflect.Value{}, tab+1)
				if err != nil {
					return err
				}
				continue
			}
			field := v.FieldByName(fieldName)
			switch fieldType {
			case "vector":
				_, value, _, err := readLine(reader, match1, tab+1)
				if err != nil {
					return err
				}
				length, _ := strconv.Atoi(value)
				d := reflect.MakeSlice(field.Type(), length, length)
				typ := field.Type().Elem().Kind()
				signed := true
				switch typ {
				case reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64, reflect.Uint:
					signed = false
					fallthrough
				case reflect.Int8, reflect.Int16, reflect.Int, reflect.Int32, reflect.Int64:
					offset := 0
					for {
						_, _, buffer, err := readLine(reader, match2, tab+1)
						if err != nil {
							return err
						}
						data := strings.Fields(buffer)
						for i := 0; i < len(data); i++ {
							if signed {
								x, err := strconv.ParseInt(data[i], 10, 64)
								if err != nil {
									return err
								}
								d.Index(offset).SetInt(x)
							} else {
								x, err := strconv.ParseUint(data[i], 10, 64)
								if err != nil {
									return err
								}
								d.Index(offset).SetUint(x)
							}
							offset++
						}
						if offset == length {
							break
						}
					}
				default:
					for i := 0; i < length; i++ {
						a, _, _, err := readLine(reader, match1, tab+1)
						if a != "Data" {
							return fmt.Errorf("invalid vector format:%s", fieldName)
						}
						if err != nil {
							return err
						}
						err = readType(reader, match1, match2, d.Index(i), tab+2)
						if err != nil {
							return err
						}
					}
				}
				field.Set(d)
			case "Hash128":
				var hash [16]byte
				for i := 0; i < 16; i++ {
					_, value, _, err := readLine(reader, match1, tab+1)
					if err != nil {
						return err
					}
					b, err := strconv.Atoi(value)
					if err != nil {
						return err
					}
					hash[i] = byte(b)
				}
				field.Set(reflect.ValueOf(hash))
			default:
				err = readType(reader, match1, match2, field, tab+1)
				if err != nil {
					return err
				}
			}
		} else {
			if !v.IsValid() || !v.FieldByName(fieldName).IsValid() {
				continue
			}
			field := v.FieldByName(fieldName)
			if fieldType == "Vector3f" {
				data := strings.Fields(strings.Trim(fieldValue, "()"))
				if len(data) != 3 {
					return fmt.Errorf("invalid Vector3f %s", fieldValue)
				}
				v := new(Vector3f)
				for i := 0; i < 3; i++ {
					num, err := strconv.ParseFloat(data[i], 64)
					if err != nil {
						return err
					}
					v.Set(i, float32(num))
				}
				field.Set(reflect.ValueOf(*v))
				continue
			}
			switch field.Kind() {
			case reflect.Int, reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:
				num, err := strconv.ParseInt(fieldValue, 0, 64)
				if err != nil {
					return err
				}
				field.SetInt(num)
			case reflect.Uint, reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64:
				num, err := strconv.ParseUint(fieldValue, 0, 64)
				if err != nil {
					return err
				}
				field.SetUint(num)
			case reflect.Float32, reflect.Float64:
				num, err := strconv.ParseFloat(fieldValue, 64)
				if err != nil {
					return err
				}
				field.SetFloat(num)
			case reflect.String:
				field.SetString(fieldValue)
			default:
				return fmt.Errorf("invalid line:%s %s = %s", fieldName, fieldType, fieldValue)
			}
		}
	}
}

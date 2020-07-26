package main_test

import "testing"

type Object struct {
	polys PolyList
}

type PolyList []int32

func (this *PolyList) resize(size int) {
	if cap(*this) >= size {
		*this = (*this)[:size]
	} else {
		*this = append(*this, make([]int32, size-len(*this))...)
	}
}

func TestPointer(t *testing.T) {
	var obj Object
	obj.polys.resize(1024)
	if len(obj.polys) != 1024 {
		t.Error("failed")
	}
}

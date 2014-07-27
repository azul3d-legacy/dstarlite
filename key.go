// Copyright 2014 The Azul3D Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package dstarlite

import (
	"fmt"
)

// key is used to assign priority to states inside the DSL planner.
//
// Keys are compared in lexical order. That is, key a is considered less than
// key b in the following case:
//
//  a1 < b1 || a1 == b1 && a2 < b2
//
type key struct {
	A, B float64
}

func (a key) String() string {
	return fmt.Sprintf("key(%v, %v)", a.A, a.B)
}

// Compare tells if key A is less than key B.
//
// A < B returns -1
//
// A > B returns 1
//
// A == B returns 0
//
func (a key) compare(b key) int {
	if a.A < b.A {
		return -1
	} else if a.A > b.A {
		return 1
	}

	if a.B < b.B {
		return -1
	} else if a.B > b.B {
		return 1
	}

	return 0
}

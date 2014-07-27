// Copyright 2014 The Azul3D Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package dstarlite

import (
	"math"
)

// Just a small helper type.
type valueMap map[State]float64

// Get returns the specified key in the map, or if the specified key does not
// exist returns +Inf
func (v valueMap) get(s State) float64 {
	val, ok := v[s]
	if !ok {
		return math.Inf(1)
	}
	return val
}

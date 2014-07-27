// Copyright 2014 The Azul3D Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Package dstarlite implements the D* Lite pathfinding algorithm.
//
// The Planner struct implements the optimized D* Lite pathfinding algorithm
// described on Page 8, Figure 9 in Sven Koenig and Maxim Likhachev's paper:
//
//  Fast Replanning for Navigation in Unknown Terrain
//  http://pub1.willowgarage.com/~konolige/cs225b/dlite_tro05.pdf
//
// D* Lite is an incremental algorithm, as such updates to it are very fast (in
// comparison to other pathfinding algorithms like A* where the entire path
// must be recalculated).
package dstarlite

import (
	"math"
)

// State represents an single DSL state.
type State interface {
	// Equals should simply tell if they're equal (useful for pointer types, etc)
	Equals(other State) bool
}

// Simple float64 epsilon comparence
func float64Equals(a, b float64) bool {
	if a == b {
		return true
	}

	if math.Abs(a-b) < math.SmallestNonzeroFloat64 {
		return true
	}
	return false
}

// Data is the data that the DSL Planner struct will plan through.
//
// See dstarlite/grid for example usage.
type Data interface {
	// Succ should return an slice of successors to the specified state.
	Succ(s State) []State

	// Pred should return an slice of predecessors to the specified state.
	Pred(s State) []State

	// Dist should return the distance between the two states. In actual use
	// the second vertex will always be the goal state.
	//
	// It must follow these rules:
	//
	//  Dist(a, a) == 0
	//  Dist(a, b) <= Cost(a, c) + Dist(c, b) (where a and c are neighbors)
	//
	Dist(a, b State) float64

	// Cost should return the exact cost for the distance between two
	// neighboring states.
	//
	// The result for non-neighboring states is undefined.
	//
	// Note: Neighbors can be determined by the Pred() and Succ() functions.
	Cost(a, b State) float64
}

// Planner plans an path through DSL Data.
type Planner struct {
	d           Data
	start, goal State
	rhs, g      valueMap
	u           *priorityQueue
	km          float64
}

// Start returns the start state, as it is currently.
func (s *Planner) Start() State {
	return s.start
}

// Goal returns the goal state, as it is currently.
func (s *Planner) Goal() State {
	return s.goal
}

func (s *Planner) calcKey(st State) key {
	a := math.Min(s.g.get(st), s.rhs.get(st)) + s.d.Dist(s.start, st) + s.km
	b := math.Min(s.g.get(st), s.rhs.get(st))
	return key{a, b}
}

func (s *Planner) updateVertex(u State) {
	eq := float64Equals(s.g.get(u), s.rhs.get(u))
	cont := s.u.contains(u)

	if !eq && cont {
		s.u.update(u, s.calcKey(u))
	} else if !eq && !cont {
		s.u.insert(u, s.calcKey(u))
	} else if eq && cont {
		s.u.remove(u)
	}
}

func (s *Planner) computeShortestPath() {
	for !s.u.isEmpty() && (s.u.topKey().compare(s.calcKey(s.start)) == -1 || s.rhs.get(s.start) > s.g.get(s.start)) {
		u := s.u.top()
		kOld := s.u.topKey()
		kNew := s.calcKey(u)

		if kOld.compare(kNew) == -1 {
			s.u.update(u, kNew)
		} else if s.g.get(u) > s.rhs.get(u) {
			s.g[u] = s.rhs.get(u)
			s.u.remove(u)
			for _, st := range s.d.Pred(u) {
				if !st.Equals(s.goal) {
					s.rhs[st] = math.Min(s.rhs.get(st), s.d.Cost(st, u)+s.g.get(u))
				}

				s.updateVertex(st)
			}
		} else {
			gOld := s.g.get(u)
			s.g[u] = math.Inf(1)

			preds := s.d.Pred(u)
			preds = append(preds, u)

			for _, st := range preds {
				if float64Equals(s.rhs.get(st), s.d.Cost(st, u)+gOld) {
					if !st.Equals(s.goal) {
						minRhs := math.Inf(0)

						for _, sPrime := range s.d.Succ(st) {
							rhsPrime := s.d.Cost(st, sPrime) + s.g.get(sPrime)
							if rhsPrime < minRhs {
								minRhs = rhsPrime
							}
						}

						s.rhs[st] = minRhs
					}
				}

				s.updateVertex(st)
			}
		}
	}
}

// FlagChanged indicates that the cost of traversal from state u to state v has
// changed from cOld to cNew and needs to be replanned at the next iteration.
func (s *Planner) FlagChanged(u, v State, cOld, cNew float64) {
	if cOld > cNew {
		if !u.Equals(s.goal) {
			s.rhs[u] = math.Min(s.rhs.get(u), cNew+s.g.get(v))
		}
	} else if float64Equals(s.rhs.get(u), cOld+s.g.get(v)) {
		if !u.Equals(s.goal) {
			minRhs := math.Inf(1)

			for _, sPrime := range s.d.Succ(u) {
				rhsPrime := s.d.Cost(u, sPrime) + s.g.get(sPrime)
				if rhsPrime < minRhs {
					minRhs = rhsPrime
				}
			}

			s.rhs[u] = minRhs
		}
	}

	s.updateVertex(u)
}

// UpdateStart changes the start location post-initialization. Use this to
// cheaply move along the path (I.e. this does not need to replan the entire
// path).
func (p *Planner) UpdateStart(s State) {
	oldStart := p.start
	p.start = s
	p.km += p.d.Dist(oldStart, s)
}

// Plan recomputes the lowest cost path through the map, taking into account
// changes in start location and edge costs.
//
// If no path is found, nil is returned.
func (s *Planner) Plan() (path []State) {
	st := s.start
	path = append(path, st)

	s.computeShortestPath()
	for !st.Equals(s.goal) {
		// If rhs(sStart) == Inf then there is no known path.
		if math.IsInf(s.rhs.get(st), 0) {
			return nil
		}

		succs := s.d.Succ(st)
		minRhs := math.Inf(1)
		var minS State

		for _, sPrime := range succs {
			rhsPrime := s.d.Cost(st, sPrime) + s.g.get(sPrime)
			if rhsPrime < minRhs {
				minRhs = rhsPrime
				minS = sPrime
			}
		}

		st = minS
		path = append(path, st)
	}

	return path
}

// Returns an new D* Lite Planner given the specified Data interface, start
// and end goal states.
func New(data Data, start, goal State) *Planner {
	dsl := new(Planner)
	dsl.d = data
	dsl.rhs = make(valueMap)
	dsl.g = make(valueMap)
	dsl.u = newPriorityQueue()

	dsl.start = start
	dsl.goal = goal
	dsl.rhs[goal] = 0.0

	k := key{dsl.d.Dist(start, goal), 0}
	dsl.u.insert(goal, k)
	return dsl
}

// Copyright 2014 The Azul3D Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package dstarlite

import (
	"container/heap"
	"math"
)

type pqItem struct {
	s State
	k key

	// The index is needed by update and is maintained by the heap.Interface methods.
	//index int // The index of the item in the heap.
	//
	// Note: Kept by lookups map below instead
}

type priorityQueue struct {
	// State:index
	lookups map[State]int
	items   []pqItem
}

//
// heap.Interface methods
//

func (q *priorityQueue) Len() int {
	return len(q.items)
}

func (q *priorityQueue) Less(i, j int) bool {
	a := q.items[i]
	b := q.items[j]

	// We want Pop to give us the lowest priority so we use less than here.
	//
	// Remember from compare() docs that:
	//
	// A < B returns -1
	//
	return a.k.compare(b.k) == -1
}

func (q *priorityQueue) Swap(i, j int) {
	q.items[i], q.items[j] = q.items[j], q.items[i]

	// Update item indices in lookups map
	q.lookups[q.items[i].s] = i
	q.lookups[q.items[j].s] = j
}

func (q *priorityQueue) Push(x interface{}) {
	n := len(q.items)
	item := x.(pqItem)
	q.lookups[item.s] = n
	q.items = append(q.items, item)
}

func (q *priorityQueue) Pop() interface{} {
	old := q.items
	n := len(old)
	item := old[n-1]
	delete(q.lookups, item.s)
	q.items = old[0 : n-1]
	return item
}

//
// Methods as described by the paper
//

func (q *priorityQueue) contains(s State) bool {
	_, ok := q.lookups[s]
	return ok
}

func (q *priorityQueue) isEmpty() bool {
	return len(q.lookups) == 0
}

// U.Top(): returns a vertex with the smallest priority of all vertices in
// priority queue U.
func (q *priorityQueue) top() State {
	return q.items[0].s
}

// U.TopKey() returns the smallest priority of all vertices in priority queue
// U.
//
// If U is empty, then U.TopKey() returns key{Inf, Inf}
func (q *priorityQueue) topKey() key {
	if len(q.items) == 0 {
		return key{math.Inf(1), math.Inf(1)}
	}
	return q.items[0].k
}

// U.Pop() deletes the vertex with the smallest priority in priority queue U
// and returns the vertex.
func (q *priorityQueue) pop() State {
	return heap.Pop(q).(pqItem).s
}

// U.Insert(s, k) inserts vertex s into priority queue U with priority k.
func (q *priorityQueue) insert(s State, k key) {
	heap.Push(q, pqItem{s, k})
}

// U.Update(s, k) changes the priority of vertex s in priority queue U to k.
//
// It does nothing if the current priority of vertex s already equals k.
func (q *priorityQueue) update(s State, k key) {
	index := q.lookups[s]

	// Check if current priority is already 'k' (a.compare(b) == 0 means perfectly equal)
	if q.items[index].k.compare(k) != 0 {
		heap.Remove(q, index)
		heap.Push(q, pqItem{s, k})
	}
}

// U.Remove(s) removes vertex s from priority queue U.
func (q *priorityQueue) remove(s State) {
	index := q.lookups[s]
	delete(q.lookups, s)
	heap.Remove(q, index)
}

func newPriorityQueue() *priorityQueue {
	q := new(priorityQueue)
	q.lookups = make(map[State]int)
	q.items = make([]pqItem, 0)
	return q
}

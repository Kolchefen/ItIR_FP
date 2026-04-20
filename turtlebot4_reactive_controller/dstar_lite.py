"""D* Lite on a 2D 8-connected occupancy grid.

Uses https://github.com/Sollimann/Dstar-lite-pathplanner as a reference.
The backward search from `goal` yields g(s) = cost-to-goal for every
expanded cell s; path extraction then greedily descends g from `start`.

The class exposes two run modes:
  - compute_shortest_path(): terminates as soon as g(start) == rhs(start)
    (the focused D* Lite behavior).
  - compute_full(): drains the priority queue to populate g for every
    reachable cell in one pass. Useful for all-pairs costs (e.g. TSP).

Grid convention: grid[row, col] != 0  means "obstacle / inflated obstacle".
"""

from __future__ import annotations

import heapq
import math
from typing import Iterable

import numpy as np

# using infinity as a sentinel for "unreachable" - cleaner than a magic number like 9999
INF = math.inf


class DStarLite:
    # movement offsets and their costs for all 8 directions
    # diagonals cost sqrt(2) (~1.414) because Pythagorean theorem
    _NEIGHBORS = (
        (-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)),
        ( 0, -1, 1.0),                         ( 0, 1, 1.0),
        ( 1, -1, math.sqrt(2)), ( 1, 0, 1.0), ( 1, 1, math.sqrt(2)),
    )

    def __init__(self, grid: np.ndarray, goal: tuple[int, int],
                 start: tuple[int, int] | None = None):
        self.grid = grid
        self.rows, self.cols = grid.shape
        self.goal = goal
        # if no start given just use goal - mostly used for compute_full() calls anyway
        self.start = start if start is not None else goal
        self.k_m = 0.0  # accumulated heuristic offset for when the start moves
        # g[s] = current best known cost from s to goal
        self.g: dict[tuple[int, int], float] = {}
        # rhs[s] = one-step lookahead cost (what g *should* be)
        # goal is 0 by definition since we're searching backwards
        self.rhs: dict[tuple[int, int], float] = {goal: 0.0}
        # lazy-deletion priority queue - we never truly remove items, just invalidate them
        self._heap: list[tuple[tuple[float, float], int, tuple[int, int]]] = []
        # tracks the most recent (key, counter) for each cell so we can detect stale entries
        self._latest: dict[tuple[int, int], tuple[tuple[float, float], int]] = {}
        self._counter = 0  # tie-breaker so heapq never tries to compare cells directly
        # seed the search from the goal (D* Lite searches backwards from goal to start)
        self._push(goal)

    # ---- public API -------------------------------------------------------

    def compute_shortest_path(self) -> bool:
        """Focused search; returns True if a finite path to `start` exists."""
        # keep expanding until the start cell is locally consistent
        while (self._top_key() < self._key(self.start)
               or self._rhs(self.start) != self._g(self.start)):
            if not self._expand_once():
                break
        return self._g(self.start) < INF

    def compute_full(self) -> None:
        """Drain the queue; populates g for every reachable cell.

        This turns D* Lite into a Dijkstra-equivalent pass (heuristic becomes
        irrelevant to expansion order once we refuse to terminate early).
        """
        # just keep going until there's nothing left to expand
        while self._heap_nonempty():
            self._expand_once()

    def cost_to_goal(self, cell: tuple[int, int]) -> float:
        """cost(cell -> goal). Requires compute_full() or compute_shortest_path()
        to have already visited `cell`."""
        return self._g(cell)

    def extract_path(self, start: tuple[int, int] | None = None
                     ) -> list[tuple[int, int]]:
        """Greedy descent of g from start to goal. Empty list if no path."""
        s = start if start is not None else self.start
        if self._g(s) >= INF:
            return []  # unreachable, give up early
        path = [s]
        while s != self.goal:
            # at each step, move to whichever neighbor minimizes (edge cost + g)
            best, best_cell = INF, None
            for n, w in self._succ(s):
                cand = w + self._g(n)
                if cand < best:
                    best, best_cell = cand, n
            if best_cell is None or best >= INF:
                return []  # got stuck, no valid neighbor
            s = best_cell
            path.append(s)
            if len(path) > self.rows * self.cols:  # safety: shouldn't happen but just in case
                return []
        return path

    # ---- algorithm internals ---------------------------------------------

    def _h(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        # octile distance (admissible for 8-connected unit grid)
        # basically: go diagonally as much as possible, then straight the rest
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return (dx + dy) + (math.sqrt(2) - 2.0) * min(dx, dy)

    # g and rhs default to INF if a cell has never been touched
    def _g(self, s): return self.g.get(s, INF)
    def _rhs(self, s): return self.rhs.get(s, INF)

    def _key(self, s):
        # D* Lite priority key is a pair (k1, k2) - lower is higher priority
        # k1 = min(g, rhs) + heuristic, k2 = min(g, rhs) for tie-breaking
        m = min(self._g(s), self._rhs(s))
        return (m + self._h(self.start, s) + self.k_m, m)

    def _in_bounds(self, s):
        r, c = s
        return 0 <= r < self.rows and 0 <= c < self.cols

    def _blocked(self, s):
        # any nonzero value = obstacle (includes inflated cells from _dilate)
        return self.grid[s[0], s[1]] != 0

    def _succ(self, s) -> Iterable[tuple[tuple[int, int], float]]:
        # yield all 8 neighbors that are in bounds and not obstacles
        for dr, dc, w in self._NEIGHBORS:
            n = (s[0] + dr, s[1] + dc)
            if self._in_bounds(n) and not self._blocked(n):
                yield n, w

    # 8-connected grid: successors == predecessors (the graph is undirected)
    _pred = _succ

    def _push(self, s):
        # each push gets a unique counter so heapq never falls back to comparing cells
        self._counter += 1
        key = self._key(s)
        entry = (key, self._counter, s)
        self._latest[s] = (key, self._counter)  # mark this as the valid entry for s
        heapq.heappush(self._heap, entry)

    def _clean(self):
        # pop stale entries off the top until we hit a valid one
        # stale = the cell was re-pushed with a newer key after this entry was added
        while self._heap:
            key, cnt, s = self._heap[0]
            latest = self._latest.get(s)
            if latest is not None and latest == (key, cnt):
                return  # top is valid, stop cleaning
            heapq.heappop(self._heap)

    def _heap_nonempty(self) -> bool:
        self._clean()
        return bool(self._heap)

    def _top_key(self):
        self._clean()
        return self._heap[0][0] if self._heap else (INF, INF)

    def _top(self):
        self._clean()
        return self._heap[0][2] if self._heap else None

    def _pop_top(self):
        self._clean()
        if not self._heap:
            return None
        key, cnt, s = heapq.heappop(self._heap)
        # only clear _latest if this entry is still the current one for s
        if self._latest.get(s) == (key, cnt):
            del self._latest[s]
        return s

    def _remove(self, s):
        # "remove" by just deleting from _latest - the heap entry becomes stale and gets skipped
        self._latest.pop(s, None)

    def _update_vertex(self, u):
        if u != self.goal:
            # recompute rhs as the best one-step cost to any neighbor
            best = INF
            for sp, w in self._succ(u):
                cand = w + self._g(sp)
                if cand < best:
                    best = cand
            self.rhs[u] = best
        self._remove(u)
        # only push back onto the heap if g and rhs disagree (cell is "inconsistent")
        if self._g(u) != self._rhs(u):
            self._push(u)

    def _expand_once(self) -> bool:
        u = self._top()
        if u is None:
            return False
        k_old = self._top_key()
        k_new = self._key(u)
        if k_old < k_new:
            # key got stale (start moved or graph changed) - reinsert with fresh key
            self._pop_top()
            self._push(u)
        elif self._g(u) > self._rhs(u):
            # underconsistent: g is too high, lower it and propagate to predecessors
            self.g[u] = self._rhs(u)
            self._pop_top()
            for p, _ in self._pred(u):
                self._update_vertex(p)
        else:
            # overconsistent: g is too low, reset to INF and re-evaluate
            self.g[u] = INF
            self._pop_top()
            for p, _ in self._pred(u):
                self._update_vertex(p)
            self._update_vertex(u)
        return True

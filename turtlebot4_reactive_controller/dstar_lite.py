"""D* Lite on a 2D 8-connected occupancy grid.

Follows Koenig & Likhachev, "D* Lite" (AAAI 2002). The backward search from
`goal` yields g(s) = cost-to-goal for every expanded cell s; path extraction
then greedily descends g from `start`.

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

INF = math.inf


class DStarLite:
    # (dr, dc, step_cost) for an 8-connected grid
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
        self.start = start if start is not None else goal
        self.k_m = 0.0
        self.g: dict[tuple[int, int], float] = {}
        self.rhs: dict[tuple[int, int], float] = {goal: 0.0}
        self._heap: list[tuple[tuple[float, float], int, tuple[int, int]]] = []
        self._latest: dict[tuple[int, int], tuple[tuple[float, float], int]] = {}
        self._counter = 0
        self._push(goal)

    # ---- public API -------------------------------------------------------

    def compute_shortest_path(self) -> bool:
        """Focused search; returns True if a finite path to `start` exists."""
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
            return []
        path = [s]
        while s != self.goal:
            best, best_cell = INF, None
            for n, w in self._succ(s):
                cand = w + self._g(n)
                if cand < best:
                    best, best_cell = cand, n
            if best_cell is None or best >= INF:
                return []
            s = best_cell
            path.append(s)
            if len(path) > self.rows * self.cols:  # safety
                return []
        return path

    # ---- algorithm internals ---------------------------------------------

    def _h(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        # octile distance (admissible for 8-connected unit grid)
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return (dx + dy) + (math.sqrt(2) - 2.0) * min(dx, dy)

    def _g(self, s): return self.g.get(s, INF)
    def _rhs(self, s): return self.rhs.get(s, INF)

    def _key(self, s):
        m = min(self._g(s), self._rhs(s))
        return (m + self._h(self.start, s) + self.k_m, m)

    def _in_bounds(self, s):
        r, c = s
        return 0 <= r < self.rows and 0 <= c < self.cols

    def _blocked(self, s):
        return self.grid[s[0], s[1]] != 0

    def _succ(self, s) -> Iterable[tuple[tuple[int, int], float]]:
        for dr, dc, w in self._NEIGHBORS:
            n = (s[0] + dr, s[1] + dc)
            if self._in_bounds(n) and not self._blocked(n):
                yield n, w

    # 8-connected grid: successors == predecessors
    _pred = _succ

    def _push(self, s):
        self._counter += 1
        key = self._key(s)
        entry = (key, self._counter, s)
        self._latest[s] = (key, self._counter)
        heapq.heappush(self._heap, entry)

    def _clean(self):
        while self._heap:
            key, cnt, s = self._heap[0]
            latest = self._latest.get(s)
            if latest is not None and latest == (key, cnt):
                return
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
        if self._latest.get(s) == (key, cnt):
            del self._latest[s]
        return s

    def _remove(self, s):
        self._latest.pop(s, None)

    def _update_vertex(self, u):
        if u != self.goal:
            best = INF
            for sp, w in self._succ(u):
                cand = w + self._g(sp)
                if cand < best:
                    best = cand
            self.rhs[u] = best
        self._remove(u)
        if self._g(u) != self._rhs(u):
            self._push(u)

    def _expand_once(self) -> bool:
        u = self._top()
        if u is None:
            return False
        k_old = self._top_key()
        k_new = self._key(u)
        if k_old < k_new:
            self._pop_top()
            self._push(u)
        elif self._g(u) > self._rhs(u):
            self.g[u] = self._rhs(u)
            self._pop_top()
            for p, _ in self._pred(u):
                self._update_vertex(p)
        else:
            self.g[u] = INF
            self._pop_top()
            for p, _ in self._pred(u):
                self._update_vertex(p)
            self._update_vertex(u)
        return True

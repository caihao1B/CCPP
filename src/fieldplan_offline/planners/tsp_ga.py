from __future__ import annotations
import math, random, itertools, shutil
from typing import List, Sequence, Dict, Tuple, Optional, Any
import networkx as nx
from networkx.algorithms.approximation import traveling_salesman
import numpy as np

def _euclidean_matrix(points: Sequence[Tuple[float, float]]) -> np.ndarray:
    pts = np.asarray(points, float)
    diff = pts[None, :, :] - pts[:, None, :]
    return np.hypot(diff[..., 0], diff[..., 1])

def _graph_matrix(targets: Sequence[Any], neighbor_map: Dict[Any, Sequence[Any]], id_attr: str = "cell_number") -> np.ndarray:
    ids = [getattr(t, id_attr) for t in targets]
    idx_of = {vid: i for i, vid in enumerate(ids)}
    n = len(ids)
    D = np.full((n, n), np.inf, float)
    np.fill_diagonal(D, 0.0)
    for vid in ids:
        q, seen = [vid], {vid: 0}
        while q:
            cur = q.pop(0)
            for nb in neighbor_map.get(cur, ()):
                if nb not in seen:
                    seen[nb] = seen[cur] + 1
                    q.append(nb)
        i = idx_of[vid]
        for nb, d in seen.items():
            j = idx_of.get(nb)
            if j is not None:
                D[i, j] = D[j, i] = float(d)
    return D

def _tour_len(order: Sequence[int], dist: np.ndarray) -> float:
    return sum(dist[order[i], order[(i + 1) % len(order)]] for i in range(len(order)))

def _pmx(parent1: List[int], parent2: List[int]) -> Tuple[List[int], List[int]]:
    size = len(parent1)
    a, b = sorted(random.sample(range(size), 2))
    child1, child2 = [None]*size, [None]*size
    child1[a:b], child2[a:b] = parent1[a:b], parent2[a:b]
    def _fill(ch, p_full):
        for gene in p_full:
            if gene not in ch:
                ch[ch.index(None)] = gene
        return ch
    return _fill(child1, parent2), _fill(child2, parent1)

def _mutate(ind: List[int], rate: float):
    for i in range(len(ind)):
        if random.random() < rate:
            j = random.randrange(len(ind))
            ind[i], ind[j] = ind[j], ind[i]

def _two_opt(route: List[int], dist: np.ndarray) -> List[int]:
    best, best_d, improved = route, _tour_len(route, dist), True
    while improved:
        improved = False
        for i in range(1, len(best)-2):
            for j in range(i+1, len(best)):
                if j-i == 1: 
                    continue
                new = best[:]
                new[i:j] = best[j-1:i-1:-1]
                new_d = _tour_len(new, dist)
                if new_d < best_d:
                    best, best_d, improved = new, new_d, True
    return best

def _ga_tsp(dist: np.ndarray, pop: int, gens: int, mut: float, use_2opt: bool, early_stop: Optional[int]=None) -> List[int]:
    n = dist.shape[0]
    population = [random.sample(range(n), n) for _ in range(pop)]
    best = min(population, key=lambda c: _tour_len(c, dist))
    best_len = _tour_len(best, dist)
    stagnate = 0
    for _ in range(gens):
        fit = [1/_tour_len(ind, dist) for ind in population]
        total = sum(fit)
        probs = [f/total for f in fit]
        sel = random.choices(population, probs, k=pop)
        nextgen = []
        for i in range(0, pop, 2):
            ch1, ch2 = _pmx(sel[i], sel[(i+1)%pop])
            nextgen += [ch1, ch2]
        for ind in nextgen:
            _mutate(ind, mut)
        if use_2opt:
            nextgen = [_two_opt(ind, dist) for ind in nextgen]
        nextgen[random.randint(0, pop-1)] = best
        population = nextgen
        cand = min(population, key=lambda c: _tour_len(c, dist))
        cand_len = _tour_len(cand, dist)
        if cand_len < best_len:
            best, best_len, stagnate = cand, cand_len, 0
        else:
            stagnate += 1
        if early_stop and stagnate >= early_stop:
            break
    return best

class TSPPlanner:
    def __init__(self, population: int = 200, generations: int = 500, mutation: float = 0.03,
                 apply_2opt: bool = True, seed: Optional[int] = None, early_stop: Optional[int] = None):
        self.population = population
        self.generations = generations
        self.mutation = mutation
        self.apply_2opt = apply_2opt
        self.early_stop = early_stop
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)

    def plan(self, targets: Sequence[Any], *, id_attr: str = "cell_number",
             dist_mode: str = "euclidean", neighbor_map: Optional[Dict[Any, Sequence[Any]]] = None,
             dist_matrix: Optional[np.ndarray] = None, start_id: Optional[Any] = None,
             goal_id: Optional[Any] = None, solver: str = "ga", return_objects: bool = False) -> List[Any]:
        if not targets:
            return []
        if dist_mode == "euclidean":
            coords = [getattr(t, "center") for t in targets]
            D = _euclidean_matrix(coords)
        elif dist_mode == "graph":
            if neighbor_map is None:
                raise ValueError("neighbor_map required for graph mode")
            D = _graph_matrix(targets, neighbor_map, id_attr)
        elif dist_mode == "precomputed":
            if dist_matrix is None:
                raise ValueError("dist_matrix must be supplied")
            D = dist_matrix
        else:
            raise ValueError(f"unknown dist_mode '{dist_mode}'")

        ids = [getattr(t, id_attr) for t in targets]
        n = len(ids)

        open_path = goal_id is not None and goal_id != start_id
        if open_path:
            if start_id not in ids or goal_id not in ids:
                raise ValueError("start_id / goal_id not in targets")
            idx_s, idx_t = ids.index(start_id), ids.index(goal_id)
            big = np.full((n+1, n+1), D.max()+1000, float)
            big[:n, :n] = D
            big[n, idx_s] = big[idx_s, n] = 0.0
            big[n, idx_t] = big[idx_t, n] = 0.0
            dist_for_solver = big
        else:
            dist_for_solver = D

        if solver == "ga":
            order_idx = _ga_tsp(dist_for_solver, self.population, self.generations, self.mutation, self.apply_2opt, self.early_stop)
        elif solver == "christofides":
            Gc = nx.Graph()
            for i, j in itertools.combinations(range(dist_for_solver.shape[0]), 2):
                Gc.add_edge(i, j, weight=float(dist_for_solver[i, j]))
            order_idx = traveling_salesman.traveling_salesman_problem(
                Gc, weight="weight", cycle=True, method=traveling_salesman.christofides
            )
        elif solver == "lkh":
            if shutil.which("LKH") is None:
                raise RuntimeError("LKH executable not found in PATH")
            raise NotImplementedError("LKH glue not implemented in this template")
        else:
            raise ValueError(f"unknown solver '{solver}'")

        if open_path:
            kD = order_idx.index(n)
            order_idx = order_idx[kD + 1:] + order_idx[1:kD]
            while order_idx[0] != idx_s:
                order_idx = order_idx[1:] + order_idx[:1]
            if order_idx[-1] != idx_t:
                order_idx = list(reversed(order_idx))
            order_idx = [i for i in order_idx if i < n]
        else:
            if start_id is not None and start_id in ids:
                k = order_idx.index(ids.index(start_id))
                order_idx = order_idx[k:] + order_idx[:k]
            order_idx.append(order_idx[0])

        if return_objects:
            tgt_list = list(targets)
            return [tgt_list[i] for i in order_idx if 0 <= i < n]
        return [ids[i] for i in order_idx if 0 <= i < n]

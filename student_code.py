from heapq import heappush, heappop, heapify, nsmallest
from helpers import load_map, show_map

def calc_distance(map_, intersection_1, intersection_2):
    x1, y1 = map_.intersections[intersection_1]
    x2, y2 = map_.intersections[intersection_2]
    distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    return distance

def get_h_lookup(map_, goal):
    h_lookup = dict()
    for key in map_.intersections:
        h_lookup[key] = calc_distance(map_, key, goal)
    return h_lookup

def init_g_lookup(map_):
    g_lookup = dict()
    for key in map_.intersections:
        g_lookup[key] = 0
    return g_lookup

def shortest_path(map_, start, goal):
    if start == goal:
        return [start]
    visited = set()
    h_lookup = get_h_lookup(map_, goal)
    g_lookup = init_g_lookup(map_)
    path = [start]
    heap = []
    alternatives = []
    while True:
        frontiers = set(map_.roads[path[-1]]) - visited
        for frontier in frontiers:
            frontier_path = path.copy()
            frontier_path.append(frontier)
            g_lookup[frontier] = g_lookup[path[-1]] + calc_distance(map_, frontier, path[-1])
            f = g_lookup[frontier] + h_lookup[frontier]
            heappush(heap, (f, frontier_path))
        visited.add(path[-1])
        f_score, path = heappop(heap)
        if path[-1] == goal:
            heappush(alternatives, (f_score, path))
        if goal in visited:
            break
    _, result = heappop(alternatives)
    return result
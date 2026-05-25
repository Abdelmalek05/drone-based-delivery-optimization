"""Shared utilities for drone delivery route optimization."""

import json
import math
import random
from pathlib import Path


def load_dataset(dataset_path="algiers_delivery_dataset.json"):
    path = Path(dataset_path)
    if not path.exists():
        alt = Path(__file__).parent.parent / dataset_path
        if alt.exists():
            path = alt
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def compute_distance_matrix(locations):
    n = len(locations)
    dm = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            d = haversine_distance(locations[i][0], locations[i][1], locations[j][0], locations[j][1])
            dm[i][j] = d
            dm[j][i] = d
    return dm


def point_in_nofly(lat, lon, no_fly_zones):
    for zone in no_fly_zones:
        if haversine_distance(lat, lon, zone['center'][0], zone['center'][1]) <= zone['radius']:
            return True
    return False


def segment_intersects_nofly(lat1, lon1, lat2, lon2, zone_lat, zone_lon, radius_km):
    lat_factor = 111.0
    lon_factor = 111.0 * math.cos(math.radians(zone_lat))
    ax = (lon1 - zone_lon) * lon_factor
    ay = (lat1 - zone_lat) * lat_factor
    bx = (lon2 - zone_lon) * lon_factor
    by = (lat2 - zone_lat) * lat_factor
    abx, aby = bx - ax, by - ay
    len2 = abx * abx + aby * aby
    if len2 == 0:
        return math.hypot(ax, ay) <= radius_km
    t = max(0.0, min(1.0, -(ax * abx + ay * aby) / len2))
    return math.hypot(ax + t * abx, ay + t * aby) <= radius_km


class Problem:
    def __init__(self, dataset_path="algiers_delivery_dataset.json"):
        self.dataset = load_dataset(dataset_path)
        self.customers = self.dataset['customers']
        self.depot = self.dataset['depot']
        self.drones = self.dataset['fleet']
        self.no_fly_zones = self.dataset.get('no_fly_zones', [])
        self.drone_specs = self.dataset.get('drone_specs', {})
        self.n = len(self.customers)
        self.locations = [(self.depot['latitude'], self.depot['longitude'])]
        for c in self.customers:
            self.locations.append((c['latitude'], c['longitude']))
        self.dist_matrix = compute_distance_matrix(self.locations)
        self.demands = [c['demand_kg'] for c in self.customers]
        self.cust_in_nofly = [point_in_nofly(c['latitude'], c['longitude'], self.no_fly_zones) for c in self.customers]
        # Instead of blocking edges, add a detour penalty (approx 2x zone radius)
        self.edge_penalty = {}
        for i in range(self.n + 1):
            for j in range(i + 1, self.n + 1):
                lat1, lon1 = self.locations[i]
                lat2, lon2 = self.locations[j]
                i_in = (i == 0) and point_in_nofly(lat1, lon1, self.no_fly_zones)
                j_in = (j > 0) and self.cust_in_nofly[j - 1]
                if i_in or j_in:
                    continue
                max_penalty = 0.0
                for zone in self.no_fly_zones:
                    if segment_intersects_nofly(lat1, lon1, lat2, lon2, zone['center'][0], zone['center'][1], zone['radius']):
                        max_penalty = max(max_penalty, 2.0 * zone['radius'])
                if max_penalty > 0:
                    self.edge_penalty[(i, j)] = max_penalty
                    self.edge_penalty[(j, i)] = max_penalty

    def route_distance(self, route):
        if not route:
            return 0.0
        total = 0.0
        prev = 0
        for c in route:
            total += self.dist_matrix[prev][c + 1] + self.edge_penalty.get((prev, c + 1), 0.0)
            prev = c + 1
        return total + self.dist_matrix[prev][0] + self.edge_penalty.get((prev, 0), 0.0)

    def route_energy(self, route, drone):
        """
        Exact energy with payload-dependent consumption.
        Assumes avg_energy_consumption_wh_per_km is at max payload.
        Base consumption (empty drone) is half of the avg value.
        """
        if not route:
            return 0.0
        avg_wh = drone.get('avg_energy_consumption_wh_per_km',
                           self.drone_specs.get('avg_energy_consumption_wh_per_km', 80))
        max_payload = drone['max_payload_kg']
        base_wh = avg_wh / 2.0
        per_kg_wh = (avg_wh - base_wh) / max_payload if max_payload > 0 else 0.0

        total_wh = 0.0
        prev = 0
        remaining_payload = sum(self.demands[c] for c in route)

        for c in route:
            dist = self.dist_matrix[prev][c + 1]
            wh_per_km = base_wh + per_kg_wh * remaining_payload
            total_wh += dist * wh_per_km
            remaining_payload -= self.demands[c]
            prev = c + 1

        # return to depot empty
        total_wh += self.dist_matrix[prev][0] * base_wh
        return total_wh / 1000.0

    def route_payload(self, route):
        return sum(self.demands[c] for c in route)

    def nofly_violations(self, route):
        return sum(1 for c in route if self.cust_in_nofly[c])

    def is_feasible(self, route, drone, strict_nofly=False):
        if not route:
            return True
        if self.route_payload(route) > drone['max_payload_kg']:
            return False
        dist = self.route_distance(route)
        if dist > drone['max_distance_km']:
            return False
        if self.route_energy(route, drone) > drone['battery_capacity_kwh']:
            return False
        if strict_nofly and self.nofly_violations(route) > 0:
            return False
        return True

    def evaluate(self, solution, penalty=10000):
        served = set()
        total_energy = 0.0
        violations = 0
        for d_idx, route in solution:
            if not route:
                continue
            drone = self.drones[d_idx]
            if not self.is_feasible(route, drone):
                violations += 1
            violations += self.nofly_violations(route)
            for c in route:
                if c in served:
                    violations += 1
                served.add(c)
            total_energy += self.route_energy(route, drone)
        unserved = set(range(self.n)) - served
        violations += len(unserved)
        return total_energy + penalty * violations, total_energy, violations, unserved

    def format(self, solution):
        lines = []
        for d_idx, route in solution:
            if not route:
                continue
            drone = self.drones[d_idx]
            names = [self.customers[c]['name'] for c in route]
            lines.append(f"Drone {drone['drone_id']} (trip): {names}")
            lines.append(f"  Distance: {self.route_distance(route):.2f} km | Energy: {self.route_energy(route, drone):.3f} kWh | Payload: {self.route_payload(route):.1f} kg | NoFly: {self.nofly_violations(route)}")
        return "\n".join(lines)

    def decode(self, permutation, max_trips=10):
        unserved = set(permutation)
        solution = []
        for d_idx in range(len(self.drones)):
            drone = self.drones[d_idx]
            trip = 0
            while unserved and trip < max_trips:
                route = []
                for c in permutation:
                    if c not in unserved:
                        continue
                    tentative = route + [c]
                    if not self.is_feasible(tentative, drone):
                        continue
                    route.append(c)
                    unserved.remove(c)
                if route:
                    solution.append((d_idx, route))
                    trip += 1
                else:
                    break
        return solution, unserved


def greedy_solve(problem, max_trips=10):
    unserved = set(range(problem.n))
    solution = []
    trips = [0] * len(problem.drones)
    while unserved:
        best_route = None
        best_drone = None
        best_score = None
        best_left = None
        for d_idx, drone in enumerate(problem.drones):
            if trips[d_idx] >= max_trips:
                continue
            local = set(unserved)
            route = []
            while True:
                nxt = None
                best_cost = float('inf')
                for c in local:
                    tentative = route + [c]
                    if not problem.is_feasible(tentative, drone):
                        continue
                    # Greedy criterion: distance from last visited node (or depot)
                    if route:
                        last_node = route[-1] + 1
                    else:
                        last_node = 0
                    cost = problem.dist_matrix[last_node][c + 1] + problem.edge_penalty.get((last_node, c + 1), 0.0)
                    if cost < best_cost:
                        best_cost = cost
                        nxt = c
                if nxt is None:
                    break
                route.append(nxt)
                local.remove(nxt)
            if route:
                score = (-len(route), problem.route_energy(route, drone))
                if best_score is None or score < best_score:
                    best_score = score
                    best_route = route
                    best_drone = d_idx
                    best_left = local
        if best_route is None:
            break
        solution.append((best_drone, best_route))
        trips[best_drone] += 1
        unserved = best_left
    return solution

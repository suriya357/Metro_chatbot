from flask import Flask, request, jsonify, render_template
from flask_cors import CORS
from collections import deque
import math
import heapq

app = Flask(__name__)
CORS(app)

# Chennai Metro Graph (Adjacency List)
graph = {
    "Central": ["High Court", "Egmore", "Government Estate"],
    "High Court": ["Mannadi", "Central"],
    "Mannadi": ["Washermanpet", "High Court"],
    "Washermanpet": ["Mannadi"],
    "Egmore": ["Central", "Nehru Park"],
    "Nehru Park": ["Egmore", "Kilpauk"],
    "Kilpauk": ["Nehru Park", "Pachaiyappa's College"],
    "Pachaiyappa's College": ["Kilpauk", "Shenoy Nagar"],
    "Shenoy Nagar": ["Pachaiyappa's College", "Anna Nagar East"],
    "Anna Nagar East": ["Shenoy Nagar", "Anna Nagar Tower"],
    "Anna Nagar Tower": ["Anna Nagar East", "Thirumangalam"],
    "Thirumangalam": ["Anna Nagar Tower", "Koyambedu"],
    "Koyambedu": ["Thirumangalam", "CMBT"],
    "CMBT": ["Koyambedu", "Arumbakkam"],
    "Arumbakkam": ["CMBT", "Vadapalani"],
    "Vadapalani": ["Arumbakkam", "Ashok Nagar"],
    "Ashok Nagar": ["Vadapalani", "Ekkatuthangal"],
    "Ekkatuthangal": ["Ashok Nagar", "Guindy"],
    "Guindy": ["Ekkatuthangal", "Alandur"],
    "Alandur": ["Guindy", "St. Thomas Mount", "Nanganallur Road"],
    "St. Thomas Mount": ["Alandur", "Meenambakkam"],
    "Meenambakkam": ["St. Thomas Mount", "Airport"],
    "Airport": ["Meenambakkam"],
    "Nanganallur Road": ["Alandur", "Little Mount"],
    "Little Mount": ["Nanganallur Road", "Saidapet"],
    "Saidapet": ["Little Mount", "TEYNAMPET"],
    "TEYNAMPET": ["Saidapet", "AG-DMS"],
    "AG-DMS": ["TEYNAMPET", "Thousand Lights"],
    "Thousand Lights": ["AG-DMS", "LIC"],
    "LIC": ["Thousand Lights", "Government Estate"],
    "Government Estate": ["LIC", "Central"]
}

# Ensure the graph is undirected
for node in list(graph.keys()):
    for neighbor in graph[node]:
        if node not in graph.get(neighbor, []):
            graph.setdefault(neighbor, []).append(node)

# Station Coordinates (Latitude, Longitude)
station_coords = {
    "Central": [13.0827, 80.2707],
    "High Court": [13.0972, 80.2879],
    "Mannadi": [13.0991, 80.2910],
    "Washermanpet": [13.1106, 80.2867],
    "Egmore": [13.0829, 80.2610],
    "Nehru Park": [13.0871, 80.2470],
    "Kilpauk": [13.0887, 80.2400],
    "Pachaiyappa's College": [13.0900, 80.2350],
    "Shenoy Nagar": [13.0887, 80.2300],
    "Anna Nagar East": [13.0878, 80.2170],
    "Anna Nagar Tower": [13.0874, 80.2100],
    "Thirumangalam": [13.0852, 80.2030],
    "Koyambedu": [13.0722, 80.1986],
    "CMBT": [13.0670, 80.2050],
    "Arumbakkam": [13.0633, 80.2120],
    "Vadapalani": [13.0484, 80.2133],
    "Ashok Nagar": [13.0334, 80.2135],
    "Ekkatuthangal": [13.0250, 80.2097],
    "Guindy": [13.0082, 80.2206],
    "Alandur": [12.9944, 80.2037],
    "St. Thomas Mount": [12.9883, 80.2016],
    "Meenambakkam": [12.9822, 80.1762],
    "Airport": [12.9808, 80.1638],
    "Nanganallur Road": [12.9964, 80.1967],
    "Little Mount": [13.0046, 80.2210],
    "Saidapet": [13.0234, 80.2278],
    "TEYNAMPET": [13.0350, 80.2370],
    "AG-DMS": [13.0440, 80.2489],
    "Thousand Lights": [13.0526, 80.2572],
    "LIC": [13.0641, 80.2672],
    "Government Estate": [13.0718, 80.2703]
}

# Approximate average speed of Chennai Metro in km/h
AVERAGE_METRO_SPEED_KMH = 35

# Haversine formula to calculate distance between two lat/lon points
def haversine(lat1, lon1, lat2, lon2):
    R = 6371
    lat1_rad, lon1_rad, lat2_rad, lon2_rad = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2_rad - lon1_rad
    dlat = lat2_rad - lat1_rad
    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

# Helper function to get distance between two stations
def get_distance_between_stations(station1, station2):
    if station1 in station_coords and station2 in station_coords:
        lat1, lon1 = station_coords[station1]
        lat2, lon2 = station_coords[station2]
        return haversine(lat1, lon1, lat2, lon2)
    return float('inf')

def bfs_path(start, end):
    if start not in graph or end not in graph: return None
    queue = deque([[start]])
    visited = {start}
    while queue:
        path = queue.popleft()
        station = path[-1]
        if station == end: return path
        for neighbor in graph[station]:
            if neighbor not in visited:
                visited.add(neighbor)
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)
    return None

def dfs_path(start, end, visited=None, path=None):
    if visited is None: visited = set()
    if path is None: path = []
    
    path.append(start)
    visited.add(start)

    if start == end: return list(path)
    
    for neighbor in graph[start]:
        if neighbor not in visited:
            result = dfs_path(neighbor, end, visited.copy(), path[:])
            if result:
                return result
    return None

def dls_path(start, end, max_depth, path=None):
    if path is None: path = []
    
    path.append(start)
    if start == end:
        return path
    
    if max_depth <= 0:
        return None
    
    for neighbor in graph[start]:
        result = dls_path(neighbor, end, max_depth - 1, path[:])
        if result:
            return result
    return None

def iddfs_path(start, end):
    for depth in range(len(graph)):
        result = dls_path(start, end, depth)
        if result:
            return result
    return None

def uniform_cost_search(start, end):
    if start not in graph or end not in graph: return None
    # Priority queue: (cost, path)
    priority_queue = [(0, [start])]
    visited = {start: 0}
    
    while priority_queue:
        cost, path = heapq.heappop(priority_queue)
        current_station = path[-1]

        if current_station == end:
            return path
        
        for neighbor in graph[current_station]:
            new_cost = cost + get_distance_between_stations(current_station, neighbor)
            if neighbor not in visited or new_cost < visited[neighbor]:
                visited[neighbor] = new_cost
                new_path = path + [neighbor]
                heapq.heappush(priority_queue, (new_cost, new_path))
    return None

def greedy_best_first_search(start, end):
    if start not in graph or end not in graph: return None
    
    def heuristic(node):
        end_coords = station_coords.get(end, [0, 0])
        node_coords = station_coords.get(node, [0, 0])
        return haversine(node_coords[0], node_coords[1], end_coords[0], end_coords[1])
        
    # Priority queue: (heuristic_cost, path)
    priority_queue = [(heuristic(start), [start])]
    visited = {start}
    
    while priority_queue:
        h_cost, path = heapq.heappop(priority_queue)
        current_station = path[-1]

        if current_station == end:
            return path
        
        # Don't explore if already visited. This check is crucial.
        if current_station in visited and current_station != start:
            continue
        
        visited.add(current_station)

        for neighbor in graph[current_station]:
            if neighbor not in visited:
                new_path = path + [neighbor]
                heapq.heappush(priority_queue, (heuristic(neighbor), new_path))
    return None

def a_star_search(start, end):
    if start not in graph or end not in graph: return None
    
    def heuristic(node):
        end_coords = station_coords.get(end, [0, 0])
        node_coords = station_coords.get(node, [0, 0])
        return haversine(node_coords[0], node_coords[1], end_coords[0], end_coords[1])
    
    # Priority queue: (f_cost, path) where f_cost = g_cost + h_cost
    priority_queue = [(0 + heuristic(start), [start])]
    g_costs = {start: 0}
    
    while priority_queue:
        f_cost, path = heapq.heappop(priority_queue)
        current_station = path[-1]
        
        if current_station == end:
            return path
        
        if f_cost > g_costs.get(current_station, float('inf')) + heuristic(current_station):
            continue

        for neighbor in graph[current_station]:
            # g_cost is the actual cost from start to neighbor
            new_g_cost = g_costs[current_station] + get_distance_between_stations(current_station, neighbor)
            
            if new_g_cost < g_costs.get(neighbor, float('inf')):
                g_costs[neighbor] = new_g_cost
                new_f_cost = new_g_cost + heuristic(neighbor)
                new_path = path + [neighbor]
                heapq.heappush(priority_queue, (new_f_cost, new_path))
    return None

@app.route("/")
def home():
    return render_template("index.html")

@app.route("/stations")
def stations():
    return jsonify(sorted(list(graph.keys())))

@app.route("/find-path", methods=["POST"])
def find_path():
    data = request.json
    start = data.get("start")
    end = data.get("end")
    algo = data.get("algo")

    if not start or not end:
        return jsonify({"message": "Please select both start and end stations."}), 400

    path_stations = None
    if algo == "dfs":
        path_stations = dfs_path(start, end)
    elif algo == "bfs":
        path_stations = bfs_path(start, end)
    elif algo == "a_star":
        path_stations = a_star_search(start, end)
    elif algo == "ucs":
        path_stations = uniform_cost_search(start, end)
    elif algo == "greedy":
        path_stations = greedy_best_first_search(start, end)
    elif algo == "dls":
        # A simple, reasonable depth limit
        path_stations = dls_path(start, end, max_depth=len(graph)) 
    elif algo == "iddfs":
        path_stations = iddfs_path(start, end)
    elif algo == "ao_star":
        return jsonify({
            "path": [], 
            "station_count": 0,
            "total_distance": 0,
            "estimated_time_minutes": 0,
            "path_coords": [],
            "error": "AO* is not suitable for simple pathfinding graphs."
        })

    total_distance_km = 0
    path_coords = []
    estimated_time_minutes = 0

    if path_stations:
        for i in range(len(path_stations)):
            station = path_stations[i]
            if station in station_coords:
                path_coords.append(station_coords[station])
            else:
                path_coords.append([0, 0])

            if i > 0:
                total_distance_km += get_distance_between_stations(path_stations[i-1], station)

        if AVERAGE_METRO_SPEED_KMH > 0:
            estimated_time_hours = total_distance_km / AVERAGE_METRO_SPEED_KMH
            estimated_time_minutes = estimated_time_hours * 60
            estimated_time_minutes += (len(path_stations) - 1) * 1.5
        estimated_time_minutes = max(0, estimated_time_minutes)

    return jsonify({
        "path": path_stations,
        "station_count": len(path_stations) if path_stations else 0,
        "total_distance": round(total_distance_km, 2),
        "estimated_time_minutes": round(estimated_time_minutes, 1),
        "path_coords": path_coords
    })

@app.route("/graph-data")
def get_graph_data():
    nodes = []
    for station, coords in station_coords.items():
        nodes.append({"id": station, "lat": coords[0], "lon": coords[1]})
    
    edges = []
    processed_edges = set()
    for station, neighbors in graph.items():
        for neighbor in neighbors:
            edge_tuple = tuple(sorted((station, neighbor)))
            if edge_tuple not in processed_edges:
                edges.append({"source": station, "target": neighbor})
                processed_edges.add(edge_tuple)
    return jsonify({"nodes": nodes, "edges": edges})

if __name__ == "__main__":
    app.run(debug=True)
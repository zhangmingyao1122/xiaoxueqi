import heapq

def build_graph(roads):
    graph = {}
    for start in roads:
        if start not in graph:
            graph[start] = []
        for end, info in roads[start].items():
            length = info['length']
            traffic = info['traffic']
            weight = calculate_weight(length, traffic)
            graph[start].append((end, weight))
            if end not in graph:
                graph[end] = []
            graph[end].append((start, weight))
    return graph
def calculate_weight(length, traffic, a=0.1, b=0.1):
    weight = a * length + b * traffic
    return round(weight, 2)

def dijkstra(graph, start, target):
    distances = {vertex: float('infinity') for vertex in graph}
    distances[start] = 0
    
    priority_queue = [(0, start, [start])]
    
    while priority_queue:
        current_distance, current_vertex, path = heapq.heappop(priority_queue)
        
        if current_vertex == target:
            return round(current_distance,2), path
        
        if current_distance > distances[current_vertex]:
            continue
        
        for neighbor, weight in graph[current_vertex]:
            distance = current_distance + weight
            
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor, path + [neighbor]))
    
    return float('infinity'), []

roads = {
    'A': {'B': {'length': 5, 'traffic': 10}, 'D': {'length': 9, 'traffic': 8}},
    'B': {'A': {'length': 5, 'traffic': 10}, 'C': {'length': 3, 'traffic': 15}, 'D': {'length': 2, 'traffic': 5}},
    'C': {'B': {'length': 3, 'traffic': 15}, 'D': {'length': 7, 'traffic': 20}, 'E': {'length': 4, 'traffic': 12}},
    'D': {'A': {'length': 9, 'traffic': 8}, 'B': {'length': 2, 'traffic': 5}, 'C': {'length': 7, 'traffic': 20}, 'E': {'length': 6, 'traffic': 18}},
    'E': {'C': {'length': 4, 'traffic': 12}, 'D': {'length': 6, 'traffic': 18}}
}
graph = build_graph(roads)

start = 'A'
target = 'D'

shortest_distance, shortest_path = dijkstra(graph, start, target)

if shortest_distance != float('infinity'):
    print(f"最短路径从 {start} 到 {target} 距离是: {shortest_distance}")
    print(f"路径是: {' -> '.join(shortest_path)}")
else:
    print(f"从{start} 到 {target} 没有路径")

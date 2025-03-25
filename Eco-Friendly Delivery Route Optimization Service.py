#!/usr/bin/env python
# coding: utf-8

# In[ ]:


# Eco-Friendly Delivery Route Optimization System
import heapq
import random


# In[ ]:


def get_user_input():
    """Prompts the user to enter valid start and destination cities."""
    cities = [
        'Manchester', 'Holyhead', 'Liverpool', 'York', 'Carlisle',
        'Newcastle', 'Glasgow', 'Edinburgh', 'Oban', 'Aberdeen', 'Inverness'
    ]
    print("Available cities:")
    print(", ".join(cities))

    start_city = input("Enter starting city: ").strip()
    end_city = input("Enter destination city: ").strip()

    if start_city not in cities or end_city not in cities:
        print("Invalid input. Please select from the available cities.")
        return get_user_input()

    return start_city, end_city


# In[ ]:


def calculate_total_distance(graph, path):
    """Calculates total driving distance for a given path."""
    if not path:
        return None
    total_distance = 0
    for i in range(len(path) - 1):
        if path[i] in graph and path[i + 1] in graph[path[i]]:
            total_distance += graph[path[i]][path[i + 1]]
        else:
            return None
    return total_distance


# In[ ]:


def adjust_map_with_traffic(graph):
    """Applies simulated traffic conditions by adjusting route distances."""
    traffic_multiplier = {city: random.uniform(0.8, 1.2) for city in graph}
    adjusted_graph = {}
    for city, connections in graph.items():
        adjusted_graph[city] = {
            neighbor: int(distance * traffic_multiplier[city])
            for neighbor, distance in connections.items()
        }
    return adjusted_graph


# In[ ]:


def depth_first_search(graph, start, goal, path=None, visited=None):
    """Performs Depth-First Search to find a route from start to goal."""
    if path is None:
        path = []
    if visited is None:
        visited = set()
    path.append(start)
    visited.add(start)

    if start == goal:
        return path

    for neighbor in graph[start]:
        if neighbor not in visited:
            result = depth_first_search(graph, neighbor, goal, path.copy(), visited)
            if result:
                return result
    return None


# In[ ]:


def breadth_first_search(graph, start, goal):
    """Performs Breadth-First Search to find the shortest route (in steps)."""
    queue = [(start, [start])]
    visited = set()
    while queue:
        (node, path) = queue.pop(0)
        if node == goal:
            return path
        if node not in visited:
            visited.add(node)
            for neighbor in graph[node]:
                queue.append((neighbor, path + [neighbor]))
    return None


# In[ ]:


def dijkstra_search(graph, start, goal):
    """Uses Dijkstra's algorithm to find the lowest-cost path."""
    priority_queue = [(0, start, [])]
    visited = set()
    while priority_queue:
        (cost, node, path) = heapq.heappop(priority_queue)
        if node in visited:
            continue
        visited.add(node)
        path = path + [node]
        if node == goal:
            return path
        for neighbor, distance in graph[node].items():
            heapq.heappush(priority_queue, (cost + distance, neighbor, path))
    return None


# In[ ]:


def a_star_search(graph, start, goal, heuristic=None):
    """Uses A* search algorithm with an optional heuristic (defaults to 0)."""
    if heuristic is None:
        heuristic = {city: 0 for city in graph}
    priority_queue = [(0, start, [])]
    visited = set()
    while priority_queue:
        (cost, node, path) = heapq.heappop(priority_queue)
        if node in visited:
            continue
        visited.add(node)
        path = path + [node]
        if node == goal:
            return path
        for neighbor, distance in graph[node].items():
            heapq.heappush(priority_queue, (cost + distance + heuristic[neighbor], neighbor, path))
    return None


# In[ ]:


def simulated_annealing(graph, start, goal, initial_temp=1000, cooling_rate=0.003):
    """Optimizes an existing route using Simulated Annealing."""
    current_path = breadth_first_search(graph, start, goal)
    if not current_path or len(current_path) < 3:
        return current_path  # Not enough to optimize

    current_distance = calculate_total_distance(graph, current_path)
    temp = initial_temp

    while temp > 1:
        new_path = current_path[:]
        if len(new_path) > 3:
            # Swap two intermediate cities
            i, j = sorted(random.sample(range(1, len(new_path) - 1), 2))
            new_path[i], new_path[j] = new_path[j], new_path[i]
        new_distance = calculate_total_distance(graph, new_path)
        if new_distance is not None:
            if new_distance < current_distance or random.uniform(0, 1) < pow(2.7, -(new_distance - current_distance) / temp):
                current_path, current_distance = new_path, new_distance
        temp *= 1 - cooling_rate

    return current_path


# In[ ]:


def main():
    """Main driver function to run the delivery route optimization."""
    map_graph = {
        'Manchester': {'Liverpool': 40, 'York': 60, 'Carlisle': 120, 'Edinburgh': 220},
        'Liverpool': {'Manchester': 40, 'Holyhead': 90},
        'Holyhead': {'Liverpool': 90},
        'York': {'Manchester': 60, 'Newcastle': 80},
        'Carlisle': {'Manchester': 120, 'Glasgow': 100},
        'Newcastle': {'York': 80, 'Edinburgh': 110},
        'Glasgow': {'Carlisle': 100, 'Edinburgh': 40, 'Oban': 90, 'Inverness': 170, 'Aberdeen': 140},
        'Edinburgh': {'Newcastle': 110, 'Glasgow': 40, 'Manchester': 220},
        'Oban': {'Glasgow': 90, 'Inverness': 110},
        'Aberdeen': {'Inverness': 110, 'Glasgow': 140},
        'Inverness': {'Aberdeen': 110, 'Oban': 110, 'Glasgow': 170}
    }

    start_city, end_city = get_user_input()

    # Run core search algorithms
    dfs_path = depth_first_search(map_graph, start_city, end_city)
    bfs_path = breadth_first_search(map_graph, start_city, end_city)
    dijkstra_path = dijkstra_search(map_graph, start_city, end_city)
    a_star_path = a_star_search(map_graph, start_city, end_city)

    print("\n=== Results with Original Map ===")
    print("DFS Path:", dfs_path, "| Distance:", calculate_total_distance(map_graph, dfs_path))
    print("BFS Path:", bfs_path, "| Distance:", calculate_total_distance(map_graph, bfs_path))
    print("Dijkstra Path:", dijkstra_path, "| Distance:", calculate_total_distance(map_graph, dijkstra_path))
    print("A* Path:", a_star_path, "| Distance:", calculate_total_distance(map_graph, a_star_path))

    # Apply traffic conditions
    map_graph = adjust_map_with_traffic(map_graph)
    print("\nTraffic conditions applied.")

    # Run simulated annealing on the traffic-adjusted map
    sa_path = simulated_annealing(map_graph, start_city, end_city)
    print("\nSimulated Annealing Path (with traffic):", sa_path, "| Distance:", calculate_total_distance(map_graph, sa_path))


# In[ ]:


if __name__ == "__main__":
    main()


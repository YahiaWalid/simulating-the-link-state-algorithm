import networkx as nx
import matplotlib.pyplot as plt
import heapq


def dijkstra_shortest_path(G, start, end):

    dist = {node: float('inf') for node in G.nodes()}  # initializing distances to all nodes to infinity
    dist[start] = 0  # setting distance of start node to 0
    prev = {node: None for node in G.nodes()}  # initializing the previous node for all nodes to none
    visited = set()  # initializing set of visited nodes
    heap = [(0, start)]  # initialize the heap with the start node and its distance from itself which is 0

    while heap:
        (d, u) = heapq.heappop(heap)  # extract the node with the smallest distance

        # if we reached the end node, return the shortest path
        if u == end:
            path = []
            while prev[u]:
                path.append(u)
                u = prev[u]
            path.append(start)
            return (d, path[::-1])

        # skipping nodes that have already been visited
        if u in visited:
            continue

        visited.add(u)  # mark the current node as visited

        for v in G.neighbors(u):  # For each neighbor of u

            if v in visited:  # Skip nodes that have already been visited
                continue

            alt = dist[u] + G[u][v]['weight']  # Compute the distance to the neighbor

            if alt < dist[v]:  # If the new distance is shorter than the current distance, update it
                dist[v] = alt
                prev[v] = u
                heapq.heappush(heap, (alt, v))  # Add the neighbor to the heap

    return float('inf'), []  # If we haven't found a path ,return infinity and an empty path



with open('input.txt') as f:
    # n--> number of nodes     m --> number of edges
    n, m = map(int, f.readline().strip().split(','))
    edges = [tuple(line.strip().split(',')) for line in f]

G = nx.Graph()         #simple undirected graph

for i in range(m) :
    G.add_edge(edges[i][0], edges[i][1], weight=int(edges[i][2]))

nx.draw_circular(G, with_labels=True)
edge_labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, nx.circular_layout(G), edge_labels=edge_labels)
plt.show()

# forwarding table for each node
for node in G.nodes():
    print("Forwarding table for node", node)
    print("-------------------------")
    print("Destination\t\tLink")

    for dest in G.nodes():
        if dest == node:
            continue
        cost , path = dijkstra_shortest_path(G, node, dest)
        next_hop = path[1]
        #print(path)
        #print("Dest: %s, Next Hop: %s, Cost: %d" % (dest, next_hop, cost))
        print("%s\t\t\t\t(%s,%s)" %(dest,node,next_hop))

    print()
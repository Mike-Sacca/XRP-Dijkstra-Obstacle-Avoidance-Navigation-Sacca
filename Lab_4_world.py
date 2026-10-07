from XRPLib.defaults import *

from Lab_4_graph import * 

# Description : This is the world class which is responsible for actually implementing the Dijkstra algorithm and can be called to alter the grid if neccesarry (the pathdriver uses these calls). 
# This function also has a reset feature to re instantiate itself in the event of route recalculation due to an obstacle. 

# Inputs : Takes in the graph class and the source node (where the robot begins).

# Outputs : Mainly used to output a list of tuples which represent the shortest path from a to b (Dijkstra). This list is fed to the pathdriver to execute the route. 

class World:
    def __init__(self, width, height, graph: Graph, source_node=(0, 0)):
        
        # graph dimension definitions 
        self.W = width
        self.H = height
        self.graph = graph
        self.realistic = True
        
        # path and location definitions
        self.source_node = source_node
        self.path = [(0, 0)]
        self.unvisited = set(graph.nodes.keys()) - {self.source_node}

        # Dijkstra algorithm tables which store the generated path and distance values for each node 
        self.path_table = {self.source_node: [self.source_node]} 
        self.distance_table = {self.source_node: 0}

        # for the algorithm to work all other nodes must be set to an infinite distance at the start 
        for node_name in self.unvisited:
            
            self.distance_table[node_name] = float('inf') 
            
            self.path_table[node_name] = ["unknown"]

    # Description : Remove edge function as lab 4 document instructs. 

    # Inputs : a point to remove, "b" (as a tuple)

    # Outputs : calls graph function to remove b and all its references 
    
    def remove_nodes(self, b):
        
        self.graph.remove_nodes(b)

    # Description : simmilar to the previous function but specifically removes just the edges 

    # Inputs : a node whose edges will be removed,"b" (as a tuple)

    # Outputs : calls graph function to remove b's edges 
        

    def remove_edge(self, b):
        
        self.graph.remove_edges(b)

    # Description : The Dijkstra algorithm itself. Calls other functions to update the Dijkstra table and list of points. 

    # Inputs : None (uses class attributes)

    # Outputs : A full Dijkstra table and list containing all points and relevant distance information 

    def dijkstra_alg(self):
        
        N = len(set(self.graph.nodes.keys())) 
        
        for _ in range(1, N):
            
            self.update_tables()
            
            self.update_list()

    # Description : Shortest path function that actually produces the list of tuples which represents the shortest possible path from a to b

    # Inputs :The goal node (also uses class attributes)

    # Outputs : A list of tuples representing the shortest path from a to b given the node information 
    
    def shortest_path(self, goal_node):
        
    # repeatedly run dijkstra until the goal node is included in the path, then stop updating the list. 
    
     while goal_node in self.unvisited:
         
         prev_unvisited_count = len(self.unvisited)
         
         self.update_tables()
         
         self.update_list()
        
        # if no progress made, exit loop and raise an error (the goal node is not reachable)
         if len(self.unvisited) == prev_unvisited_count:
             
             print("Goal node:", goal_node, "is unreachable.")
             
             self.realistic = False
             
             break

    # if the goal node is not in the path table, the same error should occur (these were useful for troubleshooting)
    
     if goal_node not in self.path_table or self.path_table[goal_node] == ["unknown"]:
         
         print("Goal node:", goal_node, "is unreachable.")
    
    # return shortest path
     return self.path_table[goal_node]
    
    # Description : Updates the Dijkstra table 

    # Inputs : Class attributes 

    # Outputs : Updated Dijkstra table 

    # update path distances
    def update_tables(self):
        # retrieve visited nodes
        visited = set(self.graph.nodes.keys()) - self.unvisited

        for node_name in visited:
            
            node_neighbors = self.graph.nodes[node_name]

            for neighbor_name, distance in node_neighbors.items():
                # new path length is calcualted
                path_length = self.distance_table[node_name] + distance

                # is the found path shorter than the previous one? if so, update the shortest path
                
                if path_length < self.distance_table[neighbor_name]:
                    
                    self.distance_table[neighbor_name] = path_length
                    
                    # list recursion! (sorta, updating the neighbors)
                    
                    self.path_table[neighbor_name] = self.path_table[node_name].copy()
                    
                    self.path_table[neighbor_name].append(neighbor_name)

    # Description : Updates the Dijkstra list of nodes to keep on the path

    # Inputs : Class attributes 

    # Outputs : Completed Dijkstra table 

    # unvisited list updates
    def update_list(self):
        # find closest unvisited node (and determine if it is truly the closest)
        
        min_node = None
        
        shortest_distance = float('inf') # set to infinity 

        for node in self.unvisited:
            
            node_dist = self.distance_table[node]
            
            if node_dist < shortest_distance:
                
                min_node = node
                
                shortest_distance = node_dist

        # node is removed from unvisited set of nodes
        if min_node is not None:
            
            self.unvisited.remove(min_node)

    # Description : Re initialization function for the World class in the event of an obstacle detection

    # Inputs : Class attributes, source node (although typically this actually means the NEW source node as this is a reset function)

    # Outputs : Nothing but fully resets the World class      
        
    def reset(self, source_node):
        
        # method to recalculate the source node
        
        # this is literally a re initilization of the source node and table logic
        
        self.source_node = source_node
        
        self.path = [(0, 0)]
        
        self.unvisited = set(self.graph.nodes.keys()) - {self.source_node}
        
        self.path_table = {self.source_node: [self.source_node]} 
        
        self.distance_table = {self.source_node: 0}

        for node_name in self.unvisited:
            
            self.distance_table[node_name] = float('inf')
            
            self.path_table[node_name] = ["unknown"]
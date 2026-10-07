from XRPLib.defaults import *

# Description : This is the graph function. This function is in charge of intaking the grid dimensions and generating the dictionary of nodes and the sub dictionaries of their neighbors automatically. 
# It is because of this automated system that the robot can implement the Dijkstra algorithm as it requires a list of nodes and their distances. 

# Inputs : Nothing is required specifically to initiate the graph class 

# Outputs : Can be used to automatically generate a grid of connected nodes and can be callsed to alter or remove nodes and their connections from the grid.  

class Graph:
    
    def __init__(self):
        
        self.nodes = {}
    
    # Description : Automatically generates a connected grid given the bounds. 

    # Inputs : Grid bounds (given as a list, [x_min, x_max, y_min, y_max])

    # Outputs : A fully connected grid (a dictionary of nodes containign sub dictonaries of their connections to other nodes)

    def create_grid(self, grid_bounds):
        
        x_min, x_max, y_min, y_max = grid_bounds
        
        for x in range(x_min, x_max + 1):
            
            for y in range(y_min, y_max + 1):
                
                node = (x, y)
                
                self.nodes[node] = {}

        # this segment automatically generates node edges by checking if 
        # neighbors exist and making the connections 
        # if they do. 
        for x in range(x_min, x_max + 1):
            
            for y in range(y_min, y_max + 1):
                
                node = (x, y)
               
                if x - 1 >= x_min:
                    self.add_edge(node, (x - 1, y), 1)
         
                if x + 1 <= x_max:
                    self.add_edge(node, (x + 1, y), 1)
 
                if y - 1 >= y_min:
                    self.add_edge(node, (x, y - 1), 1)

                if y + 1 <= y_max:
                    self.add_edge(node, (x, y + 1), 1)
                    
        # these functions add and remove nodes and adges which is not useful for path finding but 
        # is useful for object detection where obstructed nodes will need to be removed. 

    # Description : Removes edges of node e, with e being given as a tuple of the node desired to be removed. Mainly used when distances between nodes want to be altered. 

    # Inputs : A tuple, e

    # Outputs : Removes the edges of e from the grid 

    def remove_edges(self, *e):
        
        for node_0, node_1 in e:
            
            self.nodes[node_0].pop(node_1)
            
            self.nodes[node_1].pop(node_0)

    # Description : Removes node e from the grid altogether, with e being given as a tuple 

    # Inputs : A tuple, e

    # Outputs : Removes e and its references from the grid

    def remove_nodes(self, *e):
        
        for node_location in e:
            
            print(node_location)
            
            if node_location in self.nodes:
                
                # Iterate through neighbors and remove references to the node
                
                for neighbor_location in list(self.nodes[node_location]):
                    
                    if neighbor_location in self.nodes and node_location in self.nodes[neighbor_location]:
                        
                        self.nodes[neighbor_location].pop(node_location)
                        
                # Remove the node itself
                
                self.nodes.pop(node_location)
            else:
                
                print(f"Node {node_location} not found in graph")

    # Description : Adds edges between existing nodes 

    # Inputs : tuple of node 1, tuple of node 2, and a distance between them 

    # Outputs : Adds an edge with distance input value between tuples 1 and 2

    def add_edge(self, node1, node2, distance):
       
        self.nodes[node1][node2] = distance
        
        self.nodes[node2][node1] = distance

    # Description : Helper function that gets the list of all neighbors connected to a node 

    # Inputs : Node being examined

    # Outputs : A list of all nodes with connections directly to that one.

    def get_neighbors(self, node):
        
        return self.nodes.get(node, {})


from XRPLib.defaults import *

from Lab_4_world import *

from Lab_4_drivetrain import *

import time

# Description : The pathdriver is the main character of this program. The pathdriver is the only class that uses all of the others and is responsible for: 
# - Interpereting the shortest_path output (a list of tuples) and driving between all nodes contained within the list
# - Keeping track of robot orientation and current position within the grid
# - Properly recalculating a new path if an obstacle is detected and adjusting the grid data accordingly 
# - Raising recalculation flags for other parts of the code in the event of an obstacle detected 

# Inputs : The Pathdriver class requires the World class, the starting node (does not have to be 0,0), and a starting direction 
# (start_direction in previous programs was a number of degrees N, 
# however has been switched to a string represeting a cardinal direction to allow the robot to navigate non straight grids for extra credit)

# Outputs : Commands the robot to move as desired by calling the drivetrain 

class PathDriver:
    def __init__(self, imu, drivetrain, world: World, start_node, start_direction):
        
        # definitions for hardware
        self.imu = imu
        self.world = world 
        self.drivetrain = drivetrain
        
        # definitions for location logic
        self.b = 0
        self.Paths_Driven = 0
        self.current_node = []
        self.next_node = []
        self.path_recalculated = False
        
        if self.Paths_Driven <= 0:
            
            #self.Orientation = start_direction
            self.Ori = start_direction
            
            time.sleep(0.01)
            self.imu.reset_yaw()
            time.sleep(0.01)
            
            # # Initialize true north using imu and start direction
            # self.true_north = (-self.imu.get_yaw()) - start_direction
            self.a = start_node  # Start node of the path
        else:
            #self.Orientation = self.Orientation
            self.Ori = self.Ori
            
            self.a = self.a

    # Description : Calls the path finding functions and then uses this path to drive to a specific point on the grid from a (where it is) to b (ending coordinate)

    # Inputs : Class attributes, imports from world / graph, and an ending coordinate provided by the user 

    # Outputs : A drive distance from a to b for the program to use later, calls the pathfinding functions, prints them for troubleshooting. Also returns the current position 
    # of the robot based on how far it moved, in waht direction, and if the movement was sucessful or obstructed. 

    def drive_to(self, ending_coordinate):
        self.b = ending_coordinate  # Set the destination
        
        self.world.path = []  # Clear any old path data
        
        self.world.reset(self.a)  # Reset the world's source node to the current position
        
        path = self.world.shortest_path(self.b)  # Calculate the new path
        
        self.path = path  # Save the recalculated path
        
        print(path)  # Debug output to verify the new path
        
        for index in range(len(path) - 1):
            self.current_node = path[index]
            self.next_node = path[index + 1]
            
            # Check if current_node matches the destination
            if self.current_node == ending_coordinate:
                print("Reached the destination:", self.current_node)
                return  # Exit the function if the destination is reached
            
            # Calculate x and y distances to the next node and drive them
            x_dist = self.next_node[0] - self.current_node[0]
            
            y_dist = self.next_node[1] - self.current_node[1]
            
            
            if self.path_recalculated == False and self.world.realistic == True:
                
                self.__drive_to_node__(x_dist, y_dist)
                
                self.a = self.b # Move to the next node
                
            elif self.path_recalculated == True or self.world.realistic == False: 
                
                break
                
            
            print(self.current_node)
            
            print(path)  # Debug output after each movement
        
        self.Paths_Driven += 1

    # Description : This massive function handles all Orientation tracking and drive commands for the robot. This is the true "pathdriver". 
    # It also properly resets the path when necesarry to recalculate in the event of a blocked node. 

    # Inputs : Class attributes, the diference in X and diifference in Y (for the purposes of this iteration of the program, x_diff or y_diff will always be <= 1)

    # Outputs : Returns the state of the path (sucessful or obstructed) and the flags along with it. Can reset the path if needed and change the state of drive_to properly. 
    # Also returns updated orinetation information aswell as if the path has been recalculated or not. 

    def __drive_to_node__(self, x_diff, y_diff):
     
        if not x_diff == 0:
            
            if x_diff > 0 and self.Ori == "N":
                
                self.drivetrain.turn_to_line("Right")
                
                print("Turned right from N to E")
                
                self.Ori = "E"
            
            if x_diff < 0 and self.Ori == "N":
                
                self.drivetrain.turn_to_line("Left")
                
                print("Turned left from N to W")
                
                self.Ori = "W"
                
            if x_diff > 0 and self.Ori == "S":
                
                self.drivetrain.turn_to_line("Left")
                
                print("Turned left from S to E")
                
                self.Ori = "E"
            
            if x_diff < 0 and self.Ori == "S":
                
                self.drivetrain.turn_to_line("Right")
                
                print("Turned right from S to W")
                
                self.Ori = "W"
            
            if x_diff > 0 and self.Ori == "E":
                
                print("Already facing E")
                
                self.Ori = "E"
            
            if x_diff < 0 and self.Ori == "E":
                
                self.drivetrain.turn_to_line("Right")
                
                time.sleep(0.2)
                
                self.drivetrain.turn_to_line("Right")
                
                print("Turned right twice from E to W")
                
                self.Ori = "W"
                
            if x_diff < 0 and self.Ori == "W":
                
                print("Already facing W")
                
                self.Ori = "W"
            
            if x_diff > 0 and self.Ori == "W":
                
                self.drivetrain.turn_to_line("Left")
                
                time.sleep(0.2)
                
                self.drivetrain.turn_to_line("Left")
                
                print("Turned left twice from W to E")
                
                self.Ori = "E"
            
            else: 
                
                print("Unknown Direction Error")
            
            self.path_recalculated = False
    
            for _ in range(abs(x_diff)):
                
                print("Moving in X direction...")
                
                self.drivetrain.follow_line()
                
                if self.drivetrain.no_object == False:
                    
                    print("Object detected, route recalculating...")
                    
                    self.world.remove_nodes(self.next_node)  # Remove obstructed node
                    
                    self.drivetrain.no_object = True
                    
                    self.world.path = []  
                    
                    self.a = self.current_node
                    
                    self.world.reset(self.a) # Update current node as the new start
                    
                    # Recalculate path
                    
                    self.drive_to(self.b)
                    
                    self.path_recalculated = True
                    
                    break  # Exit the current movement loop immediately
                
                if self.path_recalculated:
                    
                    return  # Ensure no further movement after recalculation
                    

        if not y_diff == 0:
            
            # Assuming `y_diff` is the change in the y-axis position.
            if y_diff > 0 and self.Ori == "E":
                
                self.drivetrain.turn_to_line("Left")
                
                print("Turned left from E to N")
                
                self.Ori = "N"
            
            if y_diff < 0 and self.Ori == "E":
                
                self.drivetrain.turn_to_line("Right")
                
                print("Turned right from E to S")
                
                self.Ori = "S"
            
            if y_diff > 0 and self.Ori == "W":
                
                self.drivetrain.turn_to_line("Right")
                
                print("Turned right from W to N")
                
                self.Ori = "N"
            
            if y_diff < 0 and self.Ori == "W":
                
                self.drivetrain.turn_to_line("Left")
                
                print("Turned left from W to S")
                
                self.Ori = "S"
            
            if y_diff > 0 and self.Ori == "N":
                
                print("Already facing N")
                
                self.Ori = "N"
            
            if y_diff < 0 and self.Ori == "N":
                
                self.drivetrain.turn_to_line("Right")
                
                time.sleep(0.2)
                
                self.drivetrain.turn_to_line("Right")
                
                print("Turned right twice from N to S")
                
                self.Ori = "S"
            
            if y_diff < 0 and self.Ori == "S":
                
                print("Already facing S")
                
                self.Ori = "S"
            
            if y_diff > 0 and self.Ori == "S":
                
                self.drivetrain.turn_to_line("Right")
                
                time.sleep(0.2)
                
                self.drivetrain.turn_to_line("Right")
                
                print("Turned right twice from S to N")
                
                self.Ori = "N"
            
            else:
                
                print("Unknown Direction Error")
            
            path_recalculated = False

            for _ in range(abs(y_diff)):
                
                print("Moving in Y direction...")
                
                self.drivetrain.follow_line()
                
                if self.drivetrain.no_object == False:
                    
                    print("Object detected, route recalculating...")
                    
                    self.world.remove_nodes(self.next_node)  # Remove obstructed node
                    
                    self.drivetrain.no_object = True
                    
                    self.world.path = []  
                    
                    self.a = self.current_node
                    
                    self.world.reset(self.a) # Update current node as the new start
                    
                    # Recalculate path
                    
                    self.drive_to(self.b)
                    
                    self.path_recalculated = True
                    
                    break  # Exit the current movement loop immediately
                
                if self.path_recalculated:
                    
                    return  # Ensure no further movement after recalculation
                    
                
drivetrain = Drivetrain(left_motor, right_motor, reflectance, imu) 


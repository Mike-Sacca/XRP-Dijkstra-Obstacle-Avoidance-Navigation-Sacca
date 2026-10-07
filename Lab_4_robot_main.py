from XRPLib.defaults import *

from XRPLib . defaults import imu , left_motor , right_motor

import time

from Lab_4_graph import * 

from Lab_4_drivetrain import *

from Lab_4_world import *

from Lab_4_pathdriver import *

# Description : This is a terminal that can be used to run the program. It imports all files from the program. 
# The print command is used to signal to the user that the program is ready to be run. The user will then press the start button on the XRP robot to begin the route. 

# Inputs : All data can be instantiated here. 

# Outputs : The final project. 

print("Compiled Succesfully! Now Press Start to Begin.")

#==============================================================================#
#                             GRIDNAV V.0.50                                   #
#                             VERSION: 0.50                                    #
#------------------------------------------------------------------------------#                                                                              
#                             Variables Table:                                 #
#                             N = North                                        #
#                             E = East                                         #
#                             S = South                                        #
#                             W = West                                         #
#------------------------------------------------------------------------------#
#                        INPUT STARTING CONDITIONS: 
#
#                    Grid = (x_min x_max y_min y_max)                      
#               Match to the worlds width and height accordingly

#                 Source node can be any node within grid
#                    Match with start_node accordingly        
#          
#             Start direction given as a string value (see above) 
#------------------------------------------------------------------------------#

                                                                          
grid_limits = [0, 3, 0, 3] 
graph = Graph()
graph.create_grid(grid_limits)

world = World(
width = 3, height = 3, 
graph = graph, 
source_node = (0, 0)
)

path_driver = PathDriver(
imu, drivetrain, 
world, 
start_node = (0, 0), 
start_direction = "N"
)                                            
                                                                                                                   
                                                                               #                                                
#------------------------------------------------------------------------------#
#                            INPUT POINTS:                                     #
#                         [IN FORMAT (X, Y)]                                   #
#------------------------------------------------------------------------------#  
while True:
     board.wait_for_button()
     
     
     path_driver.drive_to((2, 1))  
     
     
#==============================================================================#

USING THE ROBOT MAIN TERMINAL: 

Initialization: 

- Within the terminal, all systems must first be initialized. Sample numbers are given to the user, 
however any number on screen may be changed as neccesary. 

- Descriptions on what each value does are located in the information box above the initialization. 

- DO NOT change non number values (with the exception of the Orientation string in version 0.5) and remember to leave commas and 
parenthesis / brackets in place. They are essential for the program to run. 

Path execution: 

- After The grid dimensions, starting node, and starting orientation have all been defined, input the desired location using the command 
"path_driver.drive_to((x,y))". 

-By default the robot will wait for the user button to be pressed before executing its route. This can be disabled by removing or commenting out the 
"wait for button push" line on line 69. 

-Also by default, the robot will print a "Compiled Succesfully! Now Press Start to Begin." message if the code compiles correctly. Wait until this message 
displays to press the user button. 

-Once the message has displayed, press the button, and the robot will travel the shortest possible, tracking its orientation and position while 
avoiding obstacles and along the way, and recalculating itself as needed. 

-If the desired node is impossible to reach, the robot will notify the user of this when the user attempts to run this route. 

FINAL WORD: 

- A lot of time was spent making this project as clean and functional as possible. I really hope you somewhat enjoy reading through the comments left in the code, many hours 
were spent developing this program. I have zero python or text based code experience prior to this project. I hope the passion I had for it comes through. 




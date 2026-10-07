from XRPLib.defaults import *

from XRPLib . defaults import imu , left_motor , right_motor

import time

# Description : this is the Drivetrain class which is responsible for handling the motor speed and PID calculations for the robot.
# This class is not generally aware of the Grid or World. Instead, this class is a library of functions to be called for movement commands for the robot. 

# Inputs : This class onyl takes in hardware libraries from the main XRP micropython libraries and does not need anything else. 

# Outputs : This function calculates the proper motor speeds and sensor values and preforms actions with them. 
# It can follow the line, Turn relativley (n degrees blindly from where we are facing now), turn absoloutley (turn n degrees from where we last reset the gyro to), back out (reverse follow line),
# turn to the line (turn to next line, regardless of gyro heading), detect objects along the path and raise flags as neccesarry to avoid them. These flags are read by the other functions
# and handled accordingly. 

class Drivetrain:
    def __init__(self, left_motor, right_motor, reflectance, imu):
        
        # Definitions for hardware
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.reflectance = reflectance
        self.imu = imu
        self.center_heading = 0
        self.no_object = True
        self.Intersection = 0
        
        # Definitions for constant values
        self.KP = 125
        self.KD = 100
        self.base_effort = 60
        self.turn_KP = 0.6
        self.turn_KD = 0.75
        self.rollup_KP = 120
        self.rollup_KD = 140
        self.turn_base_effort = 40
        self.task = "undefined"
        
        # Definitions for error thresholds
        self.tol = 0.2
        self.line_value = 0.9
        self.correction_value = 0.75
        self.error = 0 
        self.prev_error = 0
        self.L_error = 0
        self.R_error = 0
        self.Left_prev_error = 0
        self.Right_prev_error = 0
     
    # Description : This is a PID calculator state machine. This calculator can read the current state of movement of the robot which is raised as a flag and run diferetn equations accordingly.

    # Inputs : Uses the pre defined KP and KD values and the current self.error value

    # Outputs : a useable speed for the motors to spin at for PID calculation 

    def PID_Calculator(self):
        
        # PID equation calculator
        
        if self.task == "follow_line" or self.task == "backout":
            
            self.PID_Equation = (self.KP * self.error) + (self.KD * (self.error - self.prev_error))
            
        elif self.task == "turn" or self.task == "absolute_left_turn" or self.task == "absolute_right_turn" or self.task == "absolute_turn" or self.task == "right_turn" or self.task == "left_turn": 
            
            self.PID_Equation = (self.turn_KP * self.error) + (self.turn_KD * (self.error - self.prev_error))
            
        elif self.task == "rollup":
            
            self.Left_PID_Equation = (self.rollup_KP * self.L_error) - (self.rollup_KD * self.L_error)
            self.Right_PID_Equation = (self.rollup_KP * self.R_error) - (self.rollup_KD * self.R_error)
            
        else: 
            print("Error: PID Controller encountered unidentified Task", self.task)
            
            
        self.prev_error = self.error
        self.Left_prev_error = self.L_error
        self.right_prev_error = self.R_error

    # Description : Handles the useage of the PID calculation output

    # Inputs : current self.task, the base effort and the PID output 

    # Outputs : sets the motor to the appropriate speed

    def set_speed(self):
        
        # Combine PID calculation and motor direction to produce desired movement
        
        if self.task == "follow_line": 
            
         left_speed = self.base_effort - self.PID_Equation
         right_speed = self.base_effort + self.PID_Equation
        
        elif self.task == "backout":
            
            left_speed = -(self.base_effort + self.PID_Equation)
            right_speed = -(self.base_effort - self.PID_Equation)
        
        elif self.task == "left_turn" or self.task == "absolute_left_turn":
            
             left_speed = -(self.turn_base_effort + self.PID_Equation)
             right_speed = self.turn_base_effort + self.PID_Equation
             
        elif self.task == "right_turn" or self.task == "absolute_right_turn":
            
             left_speed = self.turn_base_effort + self.PID_Equation
             right_speed = -(self.turn_base_effort + self.PID_Equation)
             
        elif self.task == "rollup":
            
            left_speed = self.base_effort - self.Left_PID_Equation
            right_speed = self.base_effort - self.Right_PID_Equation 
            
        else: 
            raise ValueError("Unidentified Task Requestion from set_speed function")
        
        # Set motor speeds
        self.left_motor.set_speed(left_speed)
        self.right_motor.set_speed(right_speed)

    # Description : Allows the robot to center its wheelbase over the node by rolling up a specific amount of centimeters 

    # Inputs : distance_to_drive (given as an amount of wheel rotations, in this case a decimal)

    # Outputs : moves the robot up by that distance while using PID to adjust speed      

    def rollup(self, distance_to_drive):
        
        self.task = "follow_line"
        
        print("Centering wheelbase...")
        
        left_motor.reset_encoder_position()
        right_motor.reset_encoder_position()
        
        while left_motor.get_position() < distance_to_drive and right_motor.get_position() < distance_to_drive:
            
           # Get sensor values
           left = self.reflectance.get_left()
           right = self.reflectance.get_right()
            
           # Calculate error
           self.error = left - right
           
           # PID calculation
           self.PID_Calculator()
            
            # Set motor speeds using the set_speed function
           self.set_speed()
           
         #self.absolute_turn(self.center_heading)
        
    # Description : The big one! Follows the line to the next intersection. 

    # Inputs : No inputs, uses the class values 

    # Outputs : follows the line to the next intersection

    def follow_line(self):
        
        self.task = "follow_line"
        
        print("Following Line...")
        
        # Intersection Counter 
        self.Intersection = 0
        # Allow IMU to calibrate
        
        self.center_heading = -self.imu.get_yaw()
        
        while self.Intersection == 0:
            
            self.range_check()
            
            if not self.range_check(): 
             return
            
            # Get sensor values
            left = self.reflectance.get_left()
            right = self.reflectance.get_right()
            
            # Calculate error
            self.error = left - right
            
            # PID calculation
            self.PID_Calculator()
            
            # Set motor speeds using the set_speed function
            self.set_speed()
            
            # Check for intersection detection
            if left > self.line_value and right > self.line_value:
                self.left_motor.set_speed(0)
                self.right_motor.set_speed(0)
                print("Intersection Detected")
                self.rollup(0.375)
                self.left_motor.set_speed(0)
                self.right_motor.set_speed(0)
                self.Intersection = 1

    # Description : Turns the robot by n degrees relativley (from wherever it faces now)

    # Inputs : N degrees 

    # Outputs : Turns the robot                
                
    def turn(self, n):
        
        self.task = "turn"
        
        if n == 0:
            
            print("Error: Cannot Turn 0 Degrees")
            
            return
        
        print("Turning", n , "degrees...")

        # IMU reset in event of a relative turn
        
        time.sleep(0.01)
        self.imu.reset_yaw()
        time.sleep(0.01)

        # Making a variable that tracks our current heading 
        c = -self.imu.get_yaw()
        
        Complete = 0
        
        while Complete == 0:
            
            # Calculate the error (abs value of where we want to go vs where we are)
            
            self.error = abs(n - c)
            
            self.PID_Calculator()
            
            
            # The set_speed function will recognise a left turn and right turn case and calculate speed accordingly
            if c > n:
                
                self.task = "left_turn"
                
                self.set_speed()
                
            elif c < n:
                
                self.task = "right_turn"
                
                self.set_speed()
                
            # Update current heading at the end of the while loop
            
            c = -self.imu.get_yaw()
            print(c) 
            
            # Check if the turn is complete and continue the while loop if it is not 
            if self.error <= self.tol:
                print("Turn Complete")
                self.left_motor.set_speed(0)
                self.right_motor.set_speed(0)
                Complete = 1

    # Description : Turns the robot absoloutley (n degrees from the last time the heading was reset)

    # Inputs : N degrees 

    # Outputs : Turns the robot       
             
    def absolute_turn(self, n):
        
        print("Fed value", n, "into absolute turn sucesfully")
        print("Turning to absolute heading", n , "degrees...")

        # IMU does not reset for an absolute turn

        # Making a variable that tracks our current heading 
        c = -self.imu.get_yaw()
        print("Starting C is", c)
        Complete = 0
        prev_error = 0

        while Complete == 0:
            
            # Calculate the error (abs value of where we want to go vs where we are)
            
            
            # The set_speed function will recognise a left turn and right turn case and calculate speed accordingly
            if c > n:
                
                self.task = "absolute_left_turn"
                
                self.error = abs(n - c)
                
                self.PID_Calculator()
                
                self.set_speed()
                
            elif c < n:
                
                self.task = "absolute_right_turn"
                
                self.error = abs(n - c)
                
                self.PID_Calculator()
                
                self.set_speed()
                
            # Update current heading at the end of the while loop
            
            c = -self.imu.get_yaw()
            
            # Check if the turn is complete and continue the while loop if it is not 
            if self.error <= self.tol:
                print("Turn Complete")
                self.left_motor.set_speed(0)
                self.right_motor.set_speed(0)
                Complete = 1

    # Description : Turns the robot to the next line (physically cannot use PID to do this because theres no way to calculate error)

    # Inputs : a Direction as a string 

    # Outputs : Turns the robot to the next line 
               
    def turn_to_line(self, direction):
      
        if direction == "Right":
            
            self.left_motor.set_speed(60)
            self.right_motor.set_speed(-60)
            time.sleep(.4)
            
            while self.reflectance.get_right() < .7:
                
                self.left_motor.set_speed(60)
                
                self.right_motor.set_speed(-60)
                
            self.left_motor.set_speed(60)
            self.right_motor.set_speed(-60)
            time.sleep(.4)
            print("Turned to line (Right)")
            self.left_motor.set_speed(0)
            self.right_motor.set_speed(0)
            
        if direction == "Left":
            
            self.left_motor.set_speed(-60)
            self.right_motor.set_speed(60)
            time.sleep(.4)
            
            while self.reflectance.get_left() < .7:
                
                self.left_motor.set_speed(-60)
                
                self.right_motor.set_speed(60)
                
            self.left_motor.set_speed(-60)
            self.right_motor.set_speed(60)
            time.sleep(.4)
            print("Turned to line (Left)")
            self.left_motor.set_speed(0)
            self.right_motor.set_speed(0)
            
        if not direction == "Left" and direction == "Right":
           print("Error: Unknown direction given for a turn to line command")
        
        else: 
            print("Turn to Line leftover variable instance occured")

    # Description : Checks if there is an object in the way and raises flag if there is 

    # Inputs : Takes no specific input

    # Outputs : Object flag if there is an object 

    def range_check(self):
        
        if int(rangefinder.distance()) < 5: 
            
            self.no_object = False
            
            print("Object detected! Backing out...")
            
            self.backout()
        
        return(self.no_object)

    # Description : Backs the robot out of a node to avoid an object collision (reverse line follow)

    # Inputs : None (uses class attributeS)

    # Outputs : Backs the robot up to previous node 

    def backout(self):
        
        self.task = "backout"
        
        self.Intersection = 0
        
        while self.Intersection == 0:
            
            # Get sensor values
            left = self.reflectance.get_left()
            right = self.reflectance.get_right()
            
            # Calculate error
            self.error = left - right
            
            # PID calculation
            self.PID_Calculator()
            
            # Set motor speeds using the set_speed function
            self.set_speed()
            
            # Check for intersection detection
            if left > self.line_value and right > self.line_value:
                self.left_motor.set_speed(0)
                self.right_motor.set_speed(0)
                print("Intersection Detected")
                self.rollup(0.375)
                self.left_motor.set_speed(0)
                self.right_motor.set_speed(0)
                self.Intersection = 1
        
       

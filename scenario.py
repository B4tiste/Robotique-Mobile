#!/usr/bin/env python  

__author__ = 'Raphael Leber'

from vehicles.TurtleBot import TurtleBot   # Import TurtleBot
from vehicles.Bicycle import Bicycle   # Import TurtleBot
from VehicleSimulator import *

# create simulator instance
vs = VehicleSimlulator()

# create vehicle instance
# bicycle = Bicycle()  
turtlebot = TurtleBot()  # Create TurtleBot instance

# Choose one particular vehicle
# vs.selectVehicle(bicycle)
vs.selectVehicle(turtlebot)

# --- SCENARIO TurtleBot ---
print("Moving to BLUE")
vs.toPoint(*BLUE)
print("Moving to GREEN")
vs.toPoint(*GREEN)
print("Moving to PURPLE")
vs.toPoint(*PURPLE)
print("Turning by pi/4")
vs.turn(pi/4)


# End of Scenario
vs.frame.mainloop()

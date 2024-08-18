#Arc radius (1000 = straight) The Vehicle Controller
#verifies this doesn't violate the minimum radius
float64 radius
		    
#Rover speed (positive = forward, negative=reverse)
#The Vehicle Controller verifies this doesn't violate the maximum speed.
float64 speed

#Duration (in secs), future arcs will supercede the current one.
#Note that the duration should be short, if the VC
#stops responding the rover will not stop until the
#current arc is finished. 
float64 time

#Name of the sending module.
string sender

import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def read_log_file(filename):
    data_list = []  # Will hold the list of lists
    
    # Open the file for reading
    with open(filename, 'r') as file:
        # Iterate over each line in the file
        for line in file:
            # Strip newline characters and split by spaces
            values = line.strip().split()
            
            # Convert each value to a float and create a list
            data_list.append([float(val) for val in values])
    
    return data_list

dt = 1                      # Time step
rad = 0.325                 # Wheel Radius
width = 1.64                # Axle Width
wheelbase = 1.91            # Wheelbase of ZOE2
map_bounds = [-5,-5,5,5]      # min_x, min_y, max_x, max_y
cost_map = np.zeros((map_bounds[2]-map_bounds[0], map_bounds[3]-map_bounds[1])) 
for i in range(cost_map.shape[0]):
    for j in range(cost_map.shape[1]):
        cost_map[i,j] = np.max([10-2*np.abs(i-j),0])

# Example usage
filename = './output.txt'
data = read_log_file(filename)
start_x = data[0][0]
start_y = data[0][0]
start_th = data[0][0]
goal_x = data[0][0]
goal_y = data[0][0]
path = data[1:]
print(data[0])

path_points = []
for i in range(len((path))):
    if i == 0:
        continue
    init_x = path[i-1][0]
    init_y = path[i-1][1]
    init_th = path[i-1][2]
    arc_rad = path[i][3]
    dt_comm = path[i][4]
    v_comm = path[i][5]
    ang_velo = v_comm/arc_rad
    for angv_step in np.linspace(0,ang_velo, int(10*dt_comm)):
        next_th = init_th-angv_step*dt_comm
        if next_th >= 2*np.pi:
            next_th = math.remainder(next_th, 2*np.pi)
        if next_th < 0:
            next_th = 2*np.pi + math.remainder(next_th, 2*np.pi)

        steer_angle = np.pi/2-np.arcsin(np.abs(arc_rad)/np.sqrt(arc_rad**2+(wheelbase/2)**2))
        if arc_rad > 0:
            next_x = init_x + arc_rad*(np.cos(-next_th)-np.cos(-init_th))
            next_y = init_y + arc_rad*(np.sin(-next_th)-np.sin(-init_th))
            steer_angle /= -1
        else:
            next_x = init_x + (-arc_rad)*(np.cos(np.pi-next_th)-np.cos(np.pi-init_th))
            next_y = init_y + (-arc_rad)*(np.sin(np.pi-next_th)-np.sin(np.pi-init_th))
        path_points.append([next_x, next_y, next_th, steer_angle])
path_points = np.array(path_points)

plt.figure(0)
plt.xlim((map_bounds[0],map_bounds[2]))
plt.ylim((map_bounds[1],map_bounds[3]))
plt.title("Scatter Plot of Path")
plt.scatter(path_points[:,0], path_points[:,1], label="Path Points")
plt.legend()
plt.grid(True)
plt.show() 

plt.figure(figsize=(8, 6))
plt.imshow(cost_map, cmap='viridis', origin='lower', extent=(map_bounds[0],map_bounds[2],map_bounds[1],map_bounds[3]))
plt.scatter(path_points[:,0], path_points[:,1], color='r',label="Path Points")
plt.colorbar(label='Normalized Cost Value')  # Add a color bar
plt.title('Cost Map Visualization')
plt.xlabel('X coordinate')
plt.legend()
plt.ylabel('Y coordinate')
plt.grid(True)

# Show the plot
plt.show()

fig, ax = plt.subplots()
ax.set_xlim((map_bounds[0],map_bounds[2]))
ax.set_ylim((map_bounds[1],map_bounds[3]))
ax.grid(True)
ax.set_aspect('equal') 

car, = ax.plot([], [], 'k-', lw=2, label="Zoe2")  # Initial empty plot for the car
front_axle, = ax.plot([], [], 'ko-', lw=2)  
rear_axle, = ax.plot([], [], 'ko-', lw=2) 
center_point, = ax.plot([], [], 'ro', markersize=4, label="Zoe2 Centroid")
ax.plot([start_x], [start_y], 'mo', markersize=4, label="Initial Position")
ax.plot([goal_x], [goal_y], 'co', markersize=10, label="Goal Radius") 
plt.title("Zoe2 Rover - A-Star Plan")
plt.xlabel("X (in m)")
plt.ylabel("Y (in m)")

def init():
    car.set_data([], [])
    front_axle.set_data([], [])
    rear_axle.set_data([], [])
    center_point.set_data([], [])
    ax.legend(loc='upper left')  # Customize location as needed
    return car, front_axle, rear_axle, center_point

# Set up the animation
def update(frame):
    # Get the current position and heading
    car_length = wheelbase  # length of the "car" or rod
    axle_length = width  # Length of the axles
    
    cx = path_points[:,0][frame]
    cy = path_points[:,1][frame]
    angle = path_points[:,2][frame]
    steer_angle = path_points[:,3][frame]
    
    # Calculate the car's orientation as a line (from the center to the front)
    dx = car_length * np.cos(np.pi/2-angle)
    dy = car_length * np.sin(np.pi/2-angle)
    
    # Update the car's position and orientation (as a line with an arrowhead)
    car.set_data([cx - dx/2, cx + dx/2], [cy - dy/2, cy + dy/2])

    rear_axle_x1 = cx - dx/2 - axle_length/2 * np.sin(np.pi/2-angle+steer_angle)
    rear_axle_y1 = cy - dy/2 + axle_length/2 * np.cos(np.pi/2-angle+steer_angle)
    rear_axle_x2 = cx - dx/2 + axle_length/2 * np.sin(np.pi/2-angle+steer_angle)
    rear_axle_y2 = cy - dy/2  - axle_length/2 * np.cos(np.pi/2-angle+steer_angle)
    
    # Front axle: located at the front end of the car (same direction as the heading)
    front_axle_x1 = cx + dx/2 - axle_length/2 * np.sin(np.pi/2-angle-steer_angle)
    front_axle_y1 = cy + dy/2 + axle_length/2 * np.cos(np.pi/2-angle-steer_angle)
    front_axle_x2 = cx + dx/2 + axle_length/2 * np.sin(np.pi/2-angle-steer_angle)
    front_axle_y2 = cy + dy/2 - axle_length/2 * np.cos(np.pi/2-angle-steer_angle)
    
    # Update the axles' positions
    front_axle.set_data([front_axle_x1, front_axle_x2], [front_axle_y1, front_axle_y2])
    rear_axle.set_data([rear_axle_x1, rear_axle_x2], [rear_axle_y1, rear_axle_y2])

    center_point.set_data([cx],[cy])
    
    return car, front_axle, rear_axle, center_point

ani = FuncAnimation(fig, update, frames=len(path_points[:,0]), init_func=init, blit=True, interval=50)
# ani.save('animation_3.gif', writer='pillow', fps=20)
plt.show()
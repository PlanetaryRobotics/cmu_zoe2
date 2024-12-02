import numpy as np
import itertools
import heapq

class Node:
    def __init__(self, x, y, theta, vl, vr, g, h, v_comm, arc_rad, parent=None):
        self.x = x
        self.y = y
        self.theta = theta         
        self.vl = vl               
        self.vr = vr               
        self.arc_radius = arc_rad  
        self.transl_velocity = v_comm  
        self.g = g                 
        self.h = h                 
        self.f = g + h             
        self.parent = parent       
    
    def __lt__(self, other):
        return self.f < other.f

class A_Star_Planner:
    def __init__(self, start_x, start_y, goal_x, goal_y, other_args):
        
        self.start_x = start_x
        self.start_y = start_y
        self.goal_x = goal_x
        self.goal_y = goal_y

        self.dt = other_args[0]
        self.rad = other_args[1]
        self.width = other_args[2]
        self.wheelbase = other_args[3]
        self.wgt_heur = other_args[4]
        self.goal_radius = other_args[5]
        self.th_gain = other_args[6]

    def heuristic(self, x, y):
        return pow(x-self.goal_x, 2)+pow(y-self.goal_y, 2)

    def velocity_control(self, rad_comm, velo_comm, th_prev):
        th_comm = np.arctan(self.wheelbase/2/rad_comm)
        ang_velo = velo_comm/rad_comm
        phys_term = np.array([[1/np.cos(th_comm), -self.width/2],[1/np.cos(th_comm), self.width/2]])@np.array([[velo_comm, ang_velo]]).T
        gain_term = self.th_gain*np.array([[-(th_comm-th_prev), th_comm-th_prev]]).T
        velos = phys_term+gain_term
        return velos

    def a_star(self):
        open_list = []
        closed_list = set()
        open_set = {}

        poss_R = np.arange(-10,10,3)
        poss_vel = np.arange(1,10,.5)

        start_node = Node(self.start_x, self.start_y, 0, 0, 0, 0, self.heuristic(self.start_x, self.start_y),0,0, parent=None)
        open_set[(self.start_x, self.start_y, 0, 0, 0)] = start_node  
        heapq.heappush(open_list, start_node)
        while open_list:
            curr = heapq.heappop(open_list)
            
            print(curr.x, curr.y, curr.h)
            if curr.h <= self.goal_radius:
                path = []
                while curr:
                    path.append((curr.x, curr.y, curr.theta, curr.arc_radius, curr.transl_velocity))
                    curr = curr.parent
                return path[::-1] 

            closed_list.add((curr.x, curr.y, curr.theta))

            for arc_rad in poss_R:
                for v_comm in poss_vel:
                    comm_velos = self.velocity_control(arc_rad, v_comm, curr.theta)
                    vl = comm_velos[0][0]
                    vr = comm_velos[1][0]

                    velo = (self.rad/2)*(vl+vr)
                    ang_velo = (self.rad/self.wheelbase)*(vr-vl)
                    next_th = curr.theta+ang_velo*self.dt

                    next_x = curr.x+(velo*-np.sin(curr.theta))*(ang_velo*self.dt)
                    next_y = curr.y+(velo*np.cos(curr.theta))*(ang_velo*self.dt)

                    if not (0 <= next_x < 5 and 0 <= next_y < 5):
                        continue
                    if next_th >= 2*np.pi or next_th <= 0:
                        continue
                    if (next_x, next_y, next_th) in closed_list:
                        continue

                    g_cost = curr.g + 5*(np.abs(curr.vl-vl)+np.abs(curr.vr-vr)) 
                    h_cost = wgt_heur*self.heuristic(next_x, next_y)
                    neighbor_node = Node(next_x, next_y, next_th, vl, vr, g_cost, h_cost,v_comm, arc_rad, curr)

                    if (next_x, next_y, next_th, vl, vr) not in open_set:
                        heapq.heappush(open_list, neighbor_node)
                        open_set[(next_x, next_y, next_th, vl, vr)] = neighbor_node

        return None

if __name__ == '__main__':
    dt = 2                     
    rad = 0.325                 
    width = 1.64               
    wheelbase = 1.91            
    wgt_heur = 5                
    goal_radius = np.sqrt(.005) 
    gain = 0                    

    start_x, start_y = 0, 0    
    goal_x, goal_y = 4, 4      

    new_planner = A_Star_Planner(start_x, start_y, goal_x, goal_y, [dt, rad, width, wheelbase, wgt_heur, goal_radius, gain]) 
    plan = new_planner.a_star() 
    print(plan)

    




        
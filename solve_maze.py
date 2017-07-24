import maze_1, showmaze_1
import numpy as np
from operator import add, sub
import heapq
import sys
import time

location = start = (0,0)
heading = 'u'
goal_route = []
rotate_cost = 1.1
no_rotate_cost = 1
alpha = 0.6
goal = False
showroute = False
start_time = time.clock()

dir_move =     {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0]}
dir_rotation = {'u': {'l':-1, 'u':0, 'r':1}, 'r': {'u':-1, 'r':0, 'd':1},
                'd': {'r':-1, 'd':0, 'l':1}, 'l': {'d':-1, 'l':0, 'u':1},
                'up': {'l':-1, 'u':0, 'r':1}, 'right': {'u':-1, 'r':0, 'd':1},
                'down': {'r':-1, 'd':0, 'l':1}, 'left': {'d':-1, 'l':0, 'u':1}}
dir_sensors =  {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
                'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
                'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
                'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}

def solver(argv1, argv2 = None):
    '''Main function for returning the shortest path for any maze with the least turns.'''
    
    # Check if 'showroute' argument is present.
    try:
        showroute = argv2
    except:
        showroute = False
    
    #To begin relocate robot to start, (0,0), with heading up and compute the best route to goal_door
    maze = maze_1.Maze(argv1)
    maze_map = maze.make_map()
    goal_door_location = maze.goal_door(maze_map)
    goal_route,cost = a_star_search(goal_door_location, maze_map)
    #goal_route = d_star_lite_search(goal_door_location, maze_map, maze.dim)
    
    #Update route_map of rotations and movements for display
    rmap,rcount,mcount = route_map(goal_route, goal_door_location)
    
    # If 'showroute' was sent as an argument in the execution command, retrieve the route map from robot
    if showroute:
        showmaze_1.run(argv1,rmap)
    else:
        #Print the route_map
        print_grid(rmap,maze.dim)
    
    score = float(rcount + mcount + len(goal_route))/maze.dim
    
    print '\nTotal length of route is {} grid cells.'.format(len(goal_route))
    print 'Total movements to goal is {}.'.format(mcount)
    print 'Total rotations required is {}.'.format(rcount)
    #print 'Solver score is {}.'.format(score)
    print 'Straightness of route is {}.'.format((len(goal_route)-rcount)/float(len(goal_route)))
    #print 'Movements per route is {}.'.format(float(mcount)/len(goal_route))
    print 'Routing cost is {}.'.format(float(mcount + rcount)/len(goal_route))
    print 'Total compute time is {:.4} seconds.'.format(time.clock() - start_time)
    #print '\nCost map:'        
    #print_map(cost,maze.dim)
    
    return
    
def neighbors(cell, maze_map):
    '''Returns all the available neighbors for any cell.'''
    
    open_dirs = []
    neighbors = []
    #Check for available neighbor locations that are open
    open_dirs = [k for k,v in maze_map[cell].iteritems() if v != 0]
    #print self.maze_map[cell]
    neighbors = [tuple(map(add,cell,dir_move[i])) for i in open_dirs]
    #print neighbors
    
    return neighbors

def cost(prev_cell, cell, next_cell):
    '''Returns the cost for moving from one cell to the next.'''
    
    #get current cell heading based on previous cell
    if cell != start:
        heading = [k for k,v in dir_move.iteritems() if v == map(sub,cell,prev_cell)][0]
    #get heading necessary to move into the next cell
    next_cell_heading = [k for k,v in dir_move.iteritems() if v == map(sub,next_cell,cell)][0]
    #if these headings are equal, get cost for going straight
    if cell == start:
        cost = no_rotate_cost
    elif heading == next_cell_heading:
        cost = no_rotate_cost
    #if the headings are not the same, get cost for turning
    else:
        cost = rotate_cost
    return cost
    
def heuristic(a, b):
    '''Computes the Manhattan distance between a and b.'''
    
    (x1, y1) = a
    (x2, y2) = b
    
    return abs(x1 - x2) + abs(y1 - y2)
                              
def a_star_search(goal, maze_map):
    '''NOTE: This implementation of A* search was adapted from Amit Patel's Red Blob Games website: http://www.redblobgames.com/'''
    
    open = []
    heapq.heappush(open, (0, start))
    came_from = {}
    cost_so_far ={}
    came_from[start] = None      #Every key in came_from is a cell in the maze and 
                                 #has for its value the previous cell the robot was in.
    cost_so_far[start] = 0       #Every key in cost_so_far is a cell in the maze and
                                 #has for its value the number of steps to get to that cell
                                 #from self.start (0,0) plus the cost of moving to this same
                                 #cell from the previous cell.
            
    found = False  # flag that is set when goal is found
    resign = False # flag set if robot can't find goal
    
    #This loop tracks the cost to get from any cell to the goal
    while not found and not resign:
        if len(open) == 0:
            resign = True
            print "Failed to find route to target."
        else:
            current = heapq.heappop(open)[1]
            if current == goal:
                found = True
            else:
                for next_cell in neighbors(current,maze_map):
                    # f = g + h: typical format for cost function for A* search
                    #Calculate the cost for moving from the current cell to any of the available neighhbors.
                    #To calculate the current heading to determine whether a turn is needed we need the
                    #previous cell from which the robot moved to the current cell. This can be found in came_from.
                    move_cost = cost_so_far[current] + cost(came_from[current],current, next_cell)
                    #if next not in cost_so_far or new_cost < cost_so_far[next]:
                    if move_cost < cost_so_far.get(next_cell, float("inf")):
                        cost_so_far[next_cell] = move_cost
                        #priority = alpha * move_cost + (1 - alpha) * heuristic(goal, next_cell)
                        priority = move_cost + heuristic(goal, next_cell)
                        heapq.heappush(open,(priority, next_cell))
                        came_from[next_cell] = current
    
    #Once costs have all been calculated, we can now unpack the path to get from S to G
    current1 = goal
    path = [current1]
    while current1 != start:
        current1 = came_from[current1]
        path.append(current1)
        
    path.reverse()
    
    return path,cost_so_far
    
def d_star_lite_search(goal, maze_map, dim):
    value = {}          #Every key in value is one the orientations - u, r, d, l - that have each cell as values.
                        #Each cell is also a key that contains the cost value for that cell in that orientation.
            
    policy = {}         #Every key in policy is one the orientations - u, r, d, l - that have each cell as values.
                        #Each cell is also a key that contains the policy action - L, #, R - for that cell in
                        #that orientation.
            
    path = []
    route = {}
    cost = [rotate_cost, no_rotate_cost, rotate_cost]    #Cost for left turn, no turn, right turn
    action_name = ['L', '#', 'R']    #Symbols for left turn, no turn, right turn
    
    init = 999
    for i in range(dim):      
        for j in range(dim):
            value[(i,j)] = {'u':init,'r':init,'d':init,'l':init}
            policy[(i,j)] = {'u':init,'r':init,'d':init,'l':init}
    
    change = True
    while change:
        change = False
        
        for cell in maze_map:
            for orientation in ['u','r','d','l']:
                if cell == goal:
                #if cell[0] in self.goal_bounds and cell[1] in self.goal_bounds:
                    if value[cell][orientation] > 0:
                        value[cell][orientation] = 0
                        policy[cell][orientation] = 'G'
                        change = True
                else:
                    for i in range(3):
                        o2 = dir_sensors[orientation][i]
                        c2 = tuple(map(add,cell,dir_move[o2]))
                        if o2 in maze_map[cell]:
                            if c2 in maze_map and maze_map[cell][o2] != 0:
                                v2 = value[c2][o2] + cost[i]
                                if v2 < value[cell][orientation]:
                                    change = True
                                    value[cell][orientation] = v2
                                    policy[cell][orientation] = action_name[i]
                                    
    #for orientation in ['u','r','d','l']:
    #    policy[goal][orientation] = 'G'
    
    #Print functions to visualize value and policy for debugging and understanding
    #how the algorithm works.
    #print_map2(value,dim)
    #print_map2(policy,dim)
        
    cell = start
    orientation = 'u'
    route[cell] = policy[cell][orientation]
    path = [cell]
    
    #while cell != goal:
    while policy[cell][orientation] != 'G':
        if policy[cell][orientation] == '#':
            o2 = orientation
        elif policy[cell][orientation] == 'R':
            o2 = dir_sensors[orientation][2]
        elif policy[cell][orientation] == 'L':
            o2 = dir_sensors[orientation][0]
        cell = tuple(map(add,cell,dir_move[o2]))
        orientation = o2
        route[cell] = policy[cell][orientation]
        path.append(cell)
    
    #print_grid(route)
    
    return path
    
def route_map(path, goal):
    '''Function to create a list of moves from start (S) to goal (G) using '#' for no turn, 'L' for left turn, 'R' for right turn.'''
    
    action_list = []
    action_list.append('S')
    rmap = {}
    rcount = 0
    mcount = 0
    index = 0
    
    for i,cell in enumerate(path):
        if cell != start and cell != goal:
            prev_cell = path[i-1]
            next_cell = path[i+1]
            #get current cell heading based on previous cell
            heading = [k for k,v in dir_move.iteritems() if v == map(sub,cell,prev_cell)][0]
            #get heading necessary to move into the next cell
            next_heading = [k for k,v in dir_move.iteritems() if v == map(sub,next_cell,cell)][0]
            rot = dir_rotation[heading][next_heading]
            if rot == 0:
                action_list.append('#')
                if action_list[i-1] == 'L' or action_list[i-1] == 'R':
                    mcount += 1
                    index = 0
                else:
                    index += 1
                if index == 3:
                    mcount += 1
                    index = 0
            elif rot == 1:
                action_list.append('R')
                index = 0
                mcount += 1
                rcount += 1
            else:
                action_list.append('L')
                index = 0
                mcount += 1
                rcount += 1
                
        if cell == goal:
            action_list.append('G')
            if action_list[i-1] == '#':
                mcount += 1
            else:
                mcount += 1
            
            for i in range(len(path)):
                rmap[path[i]] = action_list[i]
                
    return rmap,rcount,mcount

def print_grid(grid, dim):
    '''Function to print out a dictionary as an array with start (0,0) in the lower left corner.'''
    printgrid = []
    for i in reversed(range(dim)):
        gridrow = []
        for j in range(dim):
            gridrow.append(grid.get((j,i),'-'))
        printgrid.append(gridrow)
    for i in range(len(printgrid)):
        print '[%s]'%'  '.join(map(str,printgrid[i]))
        
def print_map(cmap,dim):
    '''Function to print the count map dictionary as an array.'''
    
    grid = []
    for i in reversed(range(dim)):
        gridrow = []
        for j in range(dim):
            if (j,i) in cmap:
                gridrow.append(cmap[(j,i)])
            else:
                gridrow.append('-')
        grid.append(gridrow)
    for i in range(len(grid)):
        print '[%s]'%' '.join(map(str,[format(el,'>2') for el in grid[i]]))
        #print '[%s]'%'  '.join(map(str,grid[i]))
    return
        
def print_map2(vmap,dim):
    '''Function to visualize value and policy dictionaries in D* Lite'''
    
    grids = {'u':{},'r':{},'d':{},'l':{}}
    for orientation in grids:
        for i in range(dim):
            for j in range(dim):
                grids[orientation][(i,j)] = vmap[(i,j)][orientation]

    for orientation in grids:
        print orientation
        gridmap = []
        for j in reversed(range(dim)):
            gridrow = []
            for i in range(dim):
                gridrow.append(grids[orientation].get((i,j)))
            gridmap.append(gridrow)
        for i in range(len(gridmap)):
            print '[%s]'%' '.join(map(str,[format(el,'>3') for el in gridmap[i]]))
            
    return
        

if __name__ == "__main__":
    
    try:
        solver(str(sys.argv[1]), str(sys.argv[2]))
    except:
        solver(str(sys.argv[1]))
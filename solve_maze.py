import maze_1, showmaze_1
import numpy as np
from operator import add, sub
import heapq
import sys

location = start = (0,0)
heading = 'u'
goal_route = []
rotate_cost = 2
no_rotate_cost = 1
goal = False
showroute = False

dir_move =     {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0]}
dir_rotation = {'u': {'l':-1, 'u':0, 'r':1}, 'r': {'u':-1, 'r':0, 'd':1},
                'd': {'r':-1, 'd':0, 'l':1}, 'l': {'d':-1, 'l':0, 'u':1},
                'up': {'l':-1, 'u':0, 'r':1}, 'right': {'u':-1, 'r':0, 'd':1},
                'down': {'r':-1, 'd':0, 'l':1}, 'left': {'d':-1, 'l':0, 'u':1}}

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
    goal_route = a_star_search(start, goal_door_location, maze_map)
    
    #Update route_map of rotations and movements for display
    rmap = route_map(goal_route, goal_door_location)
    
    # If 'showroute' was sent as an argument in the execution command, retrieve the route map from robot
    if showroute:
        #testroute = testrobot.rmap
        showmaze_1.run(argv1,rmap)
    else:
        #Print the route_map
        print_grid(rmap,maze.dim)
        
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

def cost(prev_cell, cell, next_cell, start):
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
                              
def a_star_search(start, target, maze_map):
    '''NOTE: This implementation of A* search was adapted from Amit Patel's Red Blob Games website: http://www.redblobgames.com/'''
    
    open = []
    heapq.heappush(open, (0, start))
    came_from = {}
    cost_so_far ={}
    #cost_so_far = {}
    came_from[start] = None      #Every key in came_from is a cell in the maze and 
                                 #has for its value the previous cell the robot was in.
    cost_so_far[start] = 0       #Every key in cost_so_far is a cell in the maze and
                                 #has for its value the number of steps to get to that cell
                                 #from self.start (0,0) plus the cost of moving to this same
                                 #cell from the previous cell.
            
    found = False  # flag that is set when goal is found
    resign = False # flag set if robot can't find goal
    count = 0
    
    #This loop tracks the cost to get from any cell to the goal
    while not found and not resign:
        if len(open) == 0:
            resign = True
            print "Failed to find route to target."
        else:
            count += 1
            current = heapq.heappop(open)[1]
            
            if current == target:
                found = True
            #else:
            elif current in maze_map:
                #print 'test1'
                for next_cell in neighbors(current,maze_map):
                    # f = g + h: typical format for cost function for A* search
                    #Calculate the cost for moving from the current cell to any of the available neighhbors.
                    #The cost for moving to a neighbor in the same direction will be 1 and that for moving
                    #to a neighbor requiring a rotation will be 2.
                    #To calculate the current heading to determine whether a turn is needed we need the
                    #previous cell from which the robot moved to the current cell. This can be found in came_from.
                    move_cost = cost_so_far[current] + cost(came_from[current],current, next_cell, start)
                    #if next not in cost_so_far or new_cost < cost_so_far[next]:
                    if move_cost < cost_so_far.get(next_cell, float("inf")):
                        cost_so_far[next_cell] = move_cost
                        priority = move_cost + heuristic(target, next_cell)
                        heapq.heappush(open,(priority, next_cell))
                        came_from[next_cell] = current
    
    #Once costs have all been calculated, we can now unpack the path to get from S to G
    current1 = target
    path = [current1]
    while current1 != start:
        current1 = came_from[current1]
        path.append(current1)
        
    path.reverse()
    
    return path
    
def route_map(path, goal):
    '''Function to create a list of moves from start (S) to goal (G) using '#' for no turn, 'L' for left turn, 'R' for right turn.'''
    
    action_list = []
    action_list.append('S')
    rmap = {}
    
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
            elif rot == 1:
                action_list.append('R')
            elif rot == -1:
                action_list.append('L')
                
        if cell == goal:
            action_list.append('G')
            for i in range(len(path)):
                rmap[path[i]] = action_list[i]
        
    return rmap

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
        

if __name__ == "__main__":
    
    try:
        solver(str(sys.argv[1]), str(sys.argv[2]))
    except:
        solver(str(sys.argv[1]))
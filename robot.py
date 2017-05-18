import numpy as np
import Graph
from Queues import Queue,PriorityQueue

dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
                    'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
                    'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
                    'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}
dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
                    'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}
dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0]}
dir_rotation = {'u': {'l':-1, 'u':0, 'r':1}, 'r': {'u':-1, 'r':0, 'd':1},
                    'd': {'r':-1, 'd':0, 'l':1}, 'l': {'d':-1, 'l':0, 'u':1},
                    'up': {'l':-1, 'u':0, 'r':1}, 'right': {'u':-1, 'r':0, 'd':1},
                    'down': {'r':-1, 'd':0, 'l':1}, 'left': {'d':-1, 'l':0, 'u':1}}

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.location = [0, 0]
        self.heading = 'up'
        self.robot_pos = {'location': self.location, 'heading': self.heading}
        self.maze_dim = maze_dim
        self.g = Graph()
        
        

        self.next_cell = ()
        self.goal = ()
        
        self.start = self.location
        self.frontier = Queue()
        self.frontier.put(start)
        self.p_frontier = PriorityQueue()
        self.p_frontier.put(start, 0)
        self.came_from = {}
        self.came_from[start] = None
        self.cost_so_far = {}
        self.cost_so_far[start] = 0
        self.paths = {}
        self.found_goal = False
               
    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        clockwise rotation, and -90 for a counterclockwise rotation. Other 
        values will result in no rotation. The second value indicates robot 
        movement, and the robot will attempt to move the number of indicated
        squares: a positive number indicates forwards movement, while a negative
        number indicates backwards movement. The robot may move a maximum of
        three units per turn. Any excess movement is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''

        rotation = 0
        movement = 0
        self.sensors = sensors
        
        #update self.g with the cell and walls
        self.g.update(self.location['location'], self.location['heading'], self.sensors)
        
        #Check to see if the goal room entrance has been found
        self.goal = goal_door(self.maze_dim, self.next_cell)
        
        #If the goal room door is found, reconstruct the path from start to that location:
        if self.next_cell == self.goal:
            self.paths[0] = reconstruct_path(self.came_from,self.start,self.goal,first)
            self.found_goal = True
        
        #Search for the next cell to move to using breadth_first_search if the goal room has not
        #yet been found.
        #If it has been, look for a new path back to start until some time limit is reached
        #If start has been found or time limit reached, signal reset.
        if not self.found_goal:
            self.next_cell = breadth_first_search(self.g, self.location)
        elif:
            self.next_cell
        
        #After the goal room door has been found, keep exploring until the start is found again,
        #Then reconstruct that path to the goal and determine which one is faster
        if self.found_goal == True and self.next_cell == self.start:
            self.paths[1] = reconstruct_path(self.came_from,self.goal,self.start,second)
            (rotation, movement) == ('Reset', 'Reset')
        
        #Once we know which cell the algorithm wants to search next, compare this to the previously
        #updated self.location to determine rotation and movement
        diff = self.next_cell - self.location['location']
        
        #diff shows me which direction the robot will have to turn to when moving from the current cell
        #to the next cell.
        #Use this value to look up which direction this translates to in the dir_move dictionary.
        #This use this direction to look up the rotation factor given the heading in the 
        
        #dictionary. These operations will return -1, 0, 1 which will be multiplied by 90 to determine
        #the value of the rotation necessary to arrive at the next cell.
        #The problem is that this can't be used for moving in reverse. The only reverse would be used is
        #when sensors = (0,0,0)
        
        if self.sensors != (0,0,0):
            
        #for k,v in dir_move.iteritems():
        #    if tuple(v) == diff:
        #        next_heading = k
        #        
        #rotation_factor = dir_rotation[heading][next_heading]
        #
        #rot = 90 * rotation_factor
        
        #The above code can be collapsed to the following. The dict comprehension inside the braces, {},
        #returns a set that can't be used to iterate the dir_rotation dict, so I used the set.pop() method to
        #pop out the value that can then be used to iterate through the dict.
            rotation = 90 * dir_rotation[heading][set.pop({k for k,v in dir_move.iteritems() if tuple(v) == diff})]
            movement = 1
            
        
        #update robot_pos with the choices for the next_move rotation and movement updates
        #[self.robot_pos = position for k in self.robot_pos.iterkeys()]
        self.location = position(rotation,movement)
            
        
        return rotation, movement
    
    def position(rot, mov):
        
        # perform rotation
        if rot == -90:
            r_pos['heading'] = dir_sensors[r_pos['heading']][0]
        elif rot == 90:
            r_pos['heading'] = dir_sensors[r_pos['heading']][2]
        elif rot == 0:
            pass
        else:
            print "Invalid rotation value, no rotation performed."
            
        # perform movement
        if abs(mov) > 3:
            print "Movement limited to three squares in a turn."
        mov = max(min(int(mov), 3), -3) # fix to range [-3, 3]
        while mov:
            if mov > 0:
                if self.g.is_permissible(r_pos['location'], r_pos['heading']):
                    r_pos['location'][0] += dir_reverse[r_pos['heading']][0]
                    r_pos['location'][1] += dir_reverse[r_pos['heading']][1]
                    mov -= 1
                else:
                    print "Movement stopped by wall."
                    mov = 0
            else:
                rev_heading = dir_reverse[r_pos['heading']]
                if self.g.is_permissible(r_pos['location'], rev_heading):
                    r_pos['location'][0] += dir_reverse[rev_heading][0]
                    r_pos['location'][1] += dir_reverse[rev_heading][1]
                    mov += 1
                else:
                    print "Movement stopped by wall."
                    mov = 0
        
        return r_pos
            



def breadth_first_search(graph,current):     # For open search with goal detection
    # return "came_from"
    #If robot hasn't been to the next cell, explore the options
    if not self.frontier.empty():
        current = self.frontier.get()
        #If there's more than one open cells to choose from, pick one at random.
        next = random.choice(self.graph.neighbors(current))
        if next not in self.came_from:
            self.frontier.put(next)
            self.came_from[next] = current
    #If robot has already visited the next cell (self.frontier is empty), self.came_from[current]
    #will tell it which cell it visited before the current cell it's in the last time it was there.
    #Choose any other cell but that one
    elif:
        previous = self.came_from[current]
    
    return current, next

def dijkstra_search(graph):
    #while not frontier.empty():
    current = self.p_frontier.get()
    #    
    #    if current == goal:
    #        break
    goal = goal_door(graph.dim, current)
    
    for next in graph.neighbors(current):
        new_cost = self.cost_so_far[current] + graph.cost(current, next)
        #if next not in cost_so_far or new_cost < cost_so_far[next]:
        if new_cost < self.cost_so_far.get(next, float("inf")):
            self.cost_so_far[next] = new_cost
            priority = new_cost
            self.p_frontier.put(next, priority)
            self.came_from[next] = current
    
    return self.came_from, self.cost_so_far

def reconstruct_path(came_from, start, goal, num):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    if num == first:
        #path.append(start) # optional
        path.reverse() # optional
    return path

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def a_star_search(graph):
    #frontier = PriorityQueue()
    #frontier.put(start, 0)
    #came_from = {}
    #cost_so_far = {}
    #came_from[start] = None
    #cost_so_far[start] = 0
    
    #while not frontier.empty():
    current = self.p_frontier.get()
        
    goal = goal_door(graph.dim, cell)
    
    for next in graph.neighbors(cell):
        new_cost = self.cost_so_far[cell] + graph.cost(cell, next)
        #if next not in cost_so_far or new_cost < cost_so_far[next]:
        if new_cost < self.cost_so_far.get(next, float("inf")):
            self.cost_so_far[next] = new_cost
            priority = new_cost + heuristic(goal, next)
            frontier.put(next, priority)
            came_from[next] = cell
    
    return self.came_from, self.cost_so_far

# check for goal entered
def goal_door(maze_dim, location):
    goal_bounds = [maze_dim/2 - 1, maze_dim/2]
    if location[0] in goal_bounds and location[1] in goal_bounds:
        goal_door_location = location
    else:
        goal_door_location = None
    return goal_door_location
            
'''

I want to create a graph structure that captures where the walls are in the maze as seen from any cell.

This need results from technically not knowing the layout of the maze beforehand; the idea being
that the robot learns the layout of the maze as it progresses from cell to cell.

In each cell the tester.py prompts the robot for the next_move, but only sends along the sensors
measurements of the distances to each wall from the left, center, and right of the robot.

So, the robot will have to track its position(location(cell) and heading(direction)) based on the
previous call to robot.next_move.

Upon doing that the robot.next_move will update the Graph with the new sensor info as well as the current
position.

But that position and the sensors readings need to be translated into where the walls are and where the openings
are. Once that information has been processed, each cell is added as a key to the Graph.walls dictionary.
The values for that cell's key are another dictionary that has as it's keys the cardinal directions of the
maze: up, down, left, right. The values for these keys are the distances to the walls as input from the
sensors.

As an example, the first entry to the Graph.walls dictionary for test_maze_01.txt, which is a 12 x 12 grid:

Graph.walls[(1,1)] = {'d': 0, 'l': 0, 'r': 0, 'u': 11}

where 0 is the distance to the wall, indicating that a wall adjoins the cell in that direction

Because the sensors only shows what's in front of the robot and to the sides, I have to add into this dictionary
the previous cell as having a distance to wall equal to 1 

'''

class Graph(object):
    def __init__(self):
        self.walls = {}
        self.weights = {}
        
        self.dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
                       'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
                       'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
                       'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}
        self.dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
                       'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}
        self.dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0]}
        self.dir_rotation = {'u': {'l':-1, 'u':0, 'r':1}, 'r': {'u':-1, 'r':0, 'd':1},
                        'd': {'r':-1, 'd':0, 'l':1}, 'l': {'d':-1, 'l':0, 'u':1},
                        'up': {'l':-1, 'u':0, 'r':1}, 'right': {'u':-1, 'r':0, 'd':1},
                        'down': {'r':-1, 'd':0, 'l':1}, 'left': {'d':-1, 'l':0, 'u':1}}
        
        self.start = (0,0)

    def update(self, cell, direction, sensors):
        self.cell = cell
        self.direction = direction
        self.sensors = sensors
        
        '''
        Want to the find the absolute orientation of the sensors (basic on the direction the robot is facing, using
        self.dir_sensors[self.direction]. With these I want to attach the self.sensors values to them by looping through
        self.dir_sensors[self.direction and self.sensors simultaneously using zip(self.dir_sensors[self.direction],
        self.sensors).
        
        Once I have these, create a dictionary entry with the position of the cell and the sensor value and place this 
        dictionary into self.walls with the cell location as the key.
        
        for heading,i in zip(self.dir_sensors[self.direction],self.sensors):
            self.walls.setdefault(self.cell, dict((heading,i))
            
        The above can be collapsed into the code below.
        '''
        self.walls.setdefault(self.cell, dict((heading,i) for heading,i in zip(self.dir_sensors[self.direction],self.sensors)))
        #add to this dictionary the previous cell that the robot just came from as having no wall either to
        #allow neighbors to work as needed; the exception is cell (0,0) at the start - it has the boundary wall behind it.
        if self.cell == self.start:
            self.walls[self.cell].update({self.dir_reverse[self.direction]:0})
        else: 
            self.walls[self.cell].update({self.dir_reverse[self.direction]:1})
            
         
        return
    
    def neighbors(self, cell):
        #only return the cells that the robot can move to - no walls
        (x, y) = cell
        
        #create dictionary of locations associated with 
        results = {'u':(x, y+1), 'r':(x+1, y), 'd':(x, y-1), 'l':(x-1, y)}
        results = {results[k] for k,v in self.walls[cell].items() if v != 0}
        return results
    
    def cost(self, from_node, to_node):
        #cost to move without rotation 1
        #cost to move after rotation 2
        #cost to enter a deadend 3
        dir_no_rot = {'u': (x, y+1), 'r': (x+1, y), 'd': (x, y-1), 'l': (x-1, y)}
        
        #if to_node lies in the same direction as the direction the robot is facing
        if to_node == dir_no_rot[self.direction]:
            cost = self.weights.get(to_node,1)
        #if there are walls on 3 sides robot is in dead end    
        elif self.sensors == (0,0,0):
            cost = self.weights.get(to_node,3)
        elif to_node != dir_no_rot[self.direction]:
            cost = self.weights.get(to_node,2)
        
        return cost

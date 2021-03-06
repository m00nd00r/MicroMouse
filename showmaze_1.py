from maze import Maze
import turtle
import sys
import canvasvg

def run(argv, rmap = None):
    '''
    This function uses Python's turtle library to draw a picture of the maze
    given as an argument when running the script.
    '''

    # Create a maze based on input argument on command line.
    testmaze = Maze(argv)

    # Intialize the window and drawing turtle.
    #window = turtle.Screen()
    wally = turtle.Turtle()
    wally.speed(0)
    wally.hideturtle()
    wally.penup()

    # maze centered on (0,0), squares are 20 units in length.
    sq_size = 30
    origin = testmaze.dim * sq_size / -2

    # iterate through squares one by one to decide where to draw walls
    for x in range(testmaze.dim):
        for y in range(testmaze.dim):
            if not testmaze.is_permissible([x,y], 'up'):
                wally.goto(origin + sq_size * x, origin + sq_size * (y+1))
                wally.setheading(0)
                wally.pendown()
                wally.forward(sq_size)
                wally.penup()

            if not testmaze.is_permissible([x,y], 'right'):
                wally.goto(origin + sq_size * (x+1), origin + sq_size * y)
                wally.setheading(90)
                wally.pendown()
                wally.forward(sq_size)
                wally.penup()

            # only check bottom wall if on lowest row
            if y == 0 and not testmaze.is_permissible([x,y], 'down'):
                wally.goto(origin + sq_size * x, origin)
                wally.setheading(0)
                wally.pendown()
                wally.forward(sq_size)
                wally.penup()

            # only check left wall if on leftmost column
            if x == 0 and not testmaze.is_permissible([x,y], 'left'):
                wally.goto(origin, origin + sq_size * y)
                wally.setheading(90)
                wally.pendown()
                wally.forward(sq_size)
                wally.penup()
            
            if rmap:
                wally.goto(origin + sq_size * x + sq_size / 2, origin + sq_size * y + sq_size / 4)
                wally.write(rmap.get((x,y),''),align = "center",font = ("Arial",10,"normal"))
                
    #ps = wally.getscreen().getcanvas().postscript(file='tmp.ps', colormode='mono')
    cv = wally.getscreen().getcanvas()
    canvasvg.saveall("tmp.svg", cv)
    #args = ["gs", "-sDEVICE=jpeg", "-sOutputFile=tmp.jpg", "-dJPEGQ=100", "-r300", "-dDEVICEWIDTHPOINTS=100", \
    #        "-dDEVICEHEIGHTPOINTS=100", "-dPSFitPage","-dBATCH", "-dNOPAUSE", "tmp.ps"]
    #args = ["gs", "-dBATCH", "-dNOPAUSE", "-sDEVICE=jpeg", "-sOutputFile=tmp.jpg", "-dJPEGQ=100", "-r300", "tmp.ps"]
    #ghostscript.Ghostscript(*args)
    #window.exitonclick()
    
if __name__ == "__main__":
    run(str(sys.argv[1]))
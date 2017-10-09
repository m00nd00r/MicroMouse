

# Micro Mouse - Robot Motion Planning
## Capstone Project: Plot And Navigate A Virtual Maze
## see https://m00nd00r.github.io/MicroMouse/ for project page


## Install

This project requires **Python 2** and uses the Anaconda package manager.
If you haven't already please download and install Anaconda.

Instructions:
1. Clone the repository and navigate to the downloaded folder.
	
	```	
		git clone https://github.com/m00nd00r/MicroMouse.git
		cd MicroMouse
	```
    
2. Obtain the necessary Python packages.  
	
	For __Mac/OSX__:
	```
		conda env create -f requirements/micromouse-mac.yml
		source activate micromouse
	```

	For __Windows__:
	```
		conda env create -f requirements/micromouse-windows.yml
		activate micromouse
	```

	For __Linux__:
	```
		conda env create -f requirements/micromouse-linux.yml
		source activate micromouse

### Code

Template code is provided in this repository. The 3 scripts necessary for initial run are:  
    `tester.py`  
    `maze.py`  
    `robot.py`

### Run

Run the Jupyter notebook RobotMotionPlanning.ipynb to see how to run the tester.py script.

Or in a terminal or command window, navigate to the top-level project directory (that contains this README) and run one of the following commands replacing '##' with '01', '02', '03', '04':  
    `python tester.py test_maze_##.txt`


```python

```

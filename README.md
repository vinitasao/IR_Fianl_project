# IR_Fianl_project
Dear All,
  Consider the work as given in the .wbt. You robot is the turtlebot3 while the irobot create are the moving obstacles. The obstacles move randomly in the environment. You objective is the following

* Move your robot from the source location to the goal location 
Start pos: (-0.7,-1.2)
Goal pos: (0.7, -1.23)

* Your robot should not collide with any robot or the boundary or the static obstacle.
* Whenever the distance between turtlebot3 and create are less than 0.1, we assume the collision has happened. That from robot body to another robot body and not the centre of the body
* The work should be repeatable.

Evaluation:
1. Each code will be executed 10 times. 
2. Marks obtained = 25 - number_of_times_collided*2
3. The gui should show the path that will be taken by the robot.
4. Submit a report stating - the algorithm used, how are you detecting the obstacles, avoidance moving obstacles. 

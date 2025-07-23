# Dempter-Shafer Grid Fusion

The provided project runs a grid fusion algorithm in a virtual environment. The vehicle is moved on a hard coded path while scanning the environment by a 1D-LIDAR sensor. The result is an environment map containing drivable and not-drivable cells. The provided project uses a Bayes-based vehicle centered approach. Your task will to convert this grid based fusion to a Dempster-Shafer-based earth-fixed system.

## Task
* Decide which programming language you want to use and change the `fusion_package` in `robot_launch/robot_launch/ros2_nodes.py`
* Read through the code in `fusion_grid|fusion_grid_cpp` and try to understand its purpose
* Start with one task after the other!
* *  Change grid anchor
* * * The `onIMU` function provides you movement information. Use these to move your grid to a new location if you come close to the edges.
* * * You need a new function to convert your local sensor coordinate to your an earth fixed coordinate. Remember what we discussed on different coordinate systems
* * * Change the input of the `bresenham` function to use your new values (do not change the Bresenham algorithmus itself!)
* * Change the grid cells content
* * * For Dempster-Shafer cells you need to store at lease two float point values. Change the datastructure as needed. You want to have a frame of dicernment consisting of (Unknown, free, and occupied)
* * * Create a new function realising Dempster-Shafers Rule of Combination to your new frame of dicernment.
* * * Change the `degrade` function to work with Dempster-Shafer data.

## Hints
* Do one step after the other!
* Compile and run your code often. So you only need to debug small parts.
* Wait for complete startup. 
* Starting the simulation can take some time. Give it some time...
* You can try to change the parameters also at runtime via the `ros2` command. If you do so, don't forget to store the values in your code before uploding it!
* At about 30s the view port of the simulation suddenly moves far away. Your code should still work. You can use the mouse to move your viewport back to your vehicles or restart the simulation.
* If you have technical problems -> Write an EMail to Prof. Ohl!

## Upload
* Zip your complete project on the console by running `zip -r upload-myname.zip .`
* Do a screen recording of your 3D view with your favorite screen recorder
* Upload all to StudIP

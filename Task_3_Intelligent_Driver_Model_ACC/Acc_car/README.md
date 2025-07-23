# Adaptive Cruise Control

This project provides a setup to run a adaptive cruise control application. The other vehicle is moved by a traffic simulation driving on the race track. Your vehicle will be laterally controlled by a simple lane following camera system. The longitudinal control is your challenge. Develop a control algorithm based on the intelligent driver model.

## Task
* Decide which programming language you want to use and change the `tesla_acc.package` in `robot_launch/robot_launch/ros2_nodes.py`
* Read through the code in `acc_control|acc_control_cpp` and try to understand its purpose
* Implement the use of the `intelligentDrivingModel` function in the timer`funciton to the target speed in `s.data`.
* Tune the parameters of the intelligent driver model by changing the default values of the ROS parameters. The 0.0 values set in the orginal project does not make sense for any parameter.


## Hints
* Do one step after the other!
* Compile and run your code often. So you only need to debug small parts.
* Wait for complete startup. 
* Starting the simulation can take some time. Give it some time...
* You can try to change the parameters also at runtime via the `ros2` command. If you do so, don't forget to store the values in your code before uploding it!

## Upload
* Zip your complete project on the console by running `zip -r upload-myname.zip .`
* Do a screen recording of your 3D view with your favorite screen recorder
* Upload all to StudIP

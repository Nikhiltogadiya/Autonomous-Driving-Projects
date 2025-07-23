### **Task 3: Adaptive Cruise Control with the Intelligent Driver Model (IDM)**

This project implements an Adaptive Cruise Control (ACC) system in a ROS2 and Webots simulation environment. The core of the project is the **Intelligent Driver Model (IDM)**, a car-following model that calculates acceleration based on the distance and speed difference to a leading vehicle. The goal is to enable an autonomous vehicle to safely follow another car, adapting its speed to maintain a safe gap.

-----

### **Project Structure**

This ROS2 workspace is organized into several packages designed to work together:

  * **`acc_control`**: The main package containing the Python implementation of the ACC logic. The `ACCAgent.py` file defines the core ROS2 node that runs the Intelligent Driver Model.
  * **`acc_control_cpp`**: An alternative C++ implementation of the ACC agent for potentially higher performance.
  * **`robot_launch`**: Contains ROS2 launch files (`local_launch.py`, `cloud_launch.py`) used to start the Webots simulation, the vehicle driver, and all necessary control nodes. It also includes the Webots world (`tesla_world.wbt`) and SUMO traffic configurations.
  * **`webots_interfaces`**: Defines custom ROS2 message types. The `RadarTarget.msg` is crucial for this task, as it provides the distance and relative speed of the vehicle ahead.
  * **`robot_test`**: A package for running automated tests.

-----

### **Key Concepts**

  * **Adaptive Cruise Control (ACC)**: An advanced driver-assistance system that automatically adjusts the vehicle's speed to maintain a safe distance from vehicles ahead.
  * **Intelligent Driver Model (IDM)**: A mathematical car-following model that describes the acceleration of a vehicle based on its own speed, the speed of the leading vehicle, and the gap between them. It elegantly combines a "desired speed" behavior for free roads with a "braking" behavior when approaching a slower vehicle.
  * **Sensor Data**: The system relies on sensor input to function:
      * **Radar**: Provides the distance and relative speed of the vehicle directly in front.
      * **GPS**: Used to determine the ego vehicle's current speed.

-----

### **Implementation Details**

The core logic is implemented in the `ACCAgent.py` script within the `acc_control` package.

1.  **Node and Subscriptions**: The `ACCAgent` node subscribes to two key topics:

      * `/radar_targets` (`webots_interfaces/RadarTarget`): To get the `distance` and `speed` of the vehicle ahead.
      * `/gps/speed` (`std_msgs/Float32`): To get the current speed of the ego vehicle.

2.  **Intelligent Driver Model Logic**:

      * A timer periodically calls the `__timer_callback` function, which executes the main control logic.
      * If valid radar and GPS data are available, the `intelligentDrivingModel` function is called.
      * This function implements the IDM formula to calculate a target acceleration. It considers parameters like `max_acceleration`, desired `headway_time`, and a minimum `safety_distance`. The formula balances a natural acceleration towards a maximum speed with a strong deceleration term that grows as the gap to the lead vehicle shrinks.

3.  **Vehicle Control**:

      * The calculated acceleration is used to update the vehicle's target speed.
      * This new target speed is published as a `Float32` message to the `/cmd_speed` topic, which is read by the vehicle driver to adjust its velocity.
      * The speed is clamped to ensure it stays within a valid range (0 to `max_speed`).


### **Task 2: Environment Perception with Dempster-Shafer Grid Fusion**

This project implements an earth-fixed occupancy grid map using data from a LiDAR sensor within a ROS2 and Webots simulation environment. The core of the project is the use of the **Dempster-Shafer theory of evidence** to robustly fuse sensor readings over time, providing a rich representation of the environment that explicitly models uncertainty through `occupied`, `free`, and `unknown` states.

-----

### **Project Structure**

This ROS2 workspace is organized into several packages, each with a specific role:

  * `fusion_grid`: The primary package containing the Python implementation of the Dempster-Shafer fusion logic. The `lidar_grid.py` file houses the core `LIDARGrid` node.
  * `robot_launch`: Contains the ROS2 launch files (`local_launch.py`, `cloud_launch.py`) required to start the Webots simulation, the vehicle driver, and all necessary nodes. It also includes the Webots world files (`village_realistic.wbt`).
  * `webots_interfaces`: Defines custom ROS2 message types, such as `RadarTarget.msg`, used for communication between nodes.
  * `robot_test`: A package for running automated tests.

-----

### **Key Concepts**

  * **Occupancy Grid Map**: A data structure that represents the environment as a grid of cells, where each cell stores the belief of it being occupied by an obstacle.
  * **Dempster-Shafer Theory (DST)**: A mathematical theory of evidence used for sensor fusion. It generalizes Bayesian theory by allowing belief to be assigned not just to single states but to sets of states, which provides a more nuanced way to handle sensor uncertainty and ignorance.
  * **Earth-Fixed vs. Vehicle-Fixed Frame**: This project uses an **earth-fixed** grid, which remains stationary relative to the world. As the vehicle moves, sensor data from its own moving frame is transformed into this global frame before being integrated into the map.
  * **ROS2 & Webots**: The project is built on ROS2, a framework for robot software development. It interfaces with the Webots simulator, which provides a realistic environment, vehicle dynamics, and sensor data.

-----

### **Implementation Details**

The core logic resides in the `fusion_grid` package (`lidar_grid.py`).

1.  **Pose Tracking and Grid Anchoring**: The `LIDARGrid` node subscribes to GPS and IMU topics to get the vehicle's global pose. To maintain an earth-fixed perspective while keeping the vehicle within the grid's bounds, the grid's origin is dynamically shifted. When the vehicle moves too far from the center, the `update_grid_anchor` function moves the grid data and fills new areas with an `unknown` belief.

2.  **LiDAR Ray Tracing**: For each LiDAR point cloud received, the `__on_lidar` callback transforms the points into the global frame. **Bresenham's line algorithm** then traces a ray from the vehicle to each LiDAR hit, identifying all cells the beam passed through.

3.  **Dempster-Shafer Fusion**:

      * Cells along the traced ray are marked with evidence for being `free`. The final cell of the ray is marked with evidence for being `occupied`.
      * The `_dempster_shafer_combination` function fuses this new evidence with the existing belief in each cell using Dempster's rule of combination.

4.  **Belief Degradation and Visualization**: To account for dynamic environments, a timer periodically calls `degrade_ds_grid_belief` to decay the `occupied` and `free` beliefs, transferring their mass back to `unknown`. The resulting grid is published as a ROS2 `Image` message, where the Red, Green, and Blue channels correspond to the belief masses of `occupied`, `free`, and `unknown`.


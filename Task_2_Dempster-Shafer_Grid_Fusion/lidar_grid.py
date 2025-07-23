import cv2
import numpy as np
import math
import rclpy
from sensor_msgs.msg import PointCloud2, Imu, Image
from geometry_msgs.msg import PointStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from rclpy.parameter import Parameter
from cv_bridge import CvBridge

def getAnglesFromQuaternion(o):
    t0 = 2.0 * (o.w * o.x + o.y * o.z)
    t1 = 1.0 - 2.0 * (o.x * o.x + o.y * o.y)
    roll = math.atan2(t0, t1)
    t2 = 2.0 * (o.w * o.y - o.z * o.x)
    t2 = 1.0 if t2 > 1.0 else (-1.0 if t2 < -1.0 else t2)
    pitch = math.asin(t2)
    t3 = 2.0 * (o.w * o.z + o.x * o.y)
    t4 = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    yaw = math.atan2(t3, t4)
    return roll, yaw, pitch

class LIDARGrid(Node):
    def __init__(self):
        super().__init__('lidar_grid_ds_earth_fixed')
        self.declare_parameter("vehicle", "myvehicle")
        self.vehicle_topic_ns = self.get_parameter('vehicle').get_parameter_value().string_value
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # Grid parameters
        self.grid_width_cells = 100
        self.grid_height_cells = 100
        self.resolution_m_per_cell = [1.0, 1.0]
        self.grid_origin_world_x = 0.0
        self.grid_origin_world_y = 0.0
        self.grid_origin_initialized = False
        self.desired_vehicle_grid_row = self.grid_height_cells // 2
        self.desired_vehicle_grid_col = self.grid_width_cells // 2

        # Grid update pending flag and target coordinates
        self.pending_grid_update = False
        self.target_grid_origin_x = 0.0
        self.target_grid_origin_y = 0.0

        # DS Grid: [mass_free, mass_occupied, mass_unknown]
        self.grid_data = np.full((self.grid_height_cells, self.grid_width_cells, 3), [0.0, 0.0, 1.0], dtype=np.float32)

        # Vehicle pose
        self.vehicle_world_x = 0.0
        self.vehicle_world_y = 0.0
        self.vehicle_world_yaw = 0.0

        # DS Parameters
        self.DS_SENSOR_HIT_OCCUPIED = 0.8
        self.DS_SENSOR_HIT_FREE = 0.05
        self.DS_SENSOR_HIT_UNKNOWN = 1.0 - self.DS_SENSOR_HIT_OCCUPIED - self.DS_SENSOR_HIT_FREE
        self.DS_SENSOR_MISS_FREE = 0.7
        self.DS_SENSOR_MISS_OCCUPIED = 0.05
        self.DS_SENSOR_MISS_UNKNOWN = 1.0 - self.DS_SENSOR_MISS_FREE - self.DS_SENSOR_MISS_OCCUPIED
        self.lidar_max_range = 50.0
        self.DS_DECAY_FACTOR = 0.9  # Increased to slow degradation

        # ROS interface
        self.create_subscription(PointCloud2, f'{self.vehicle_topic_ns}/lidar/point_cloud', self.__on_lidar, qos_profile_sensor_data)
        self.create_subscription(PointStamped, f'{self.vehicle_topic_ns}/gps', self.__on_gps, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu', self.__on_imu, qos_profile_sensor_data)
        self._grid_publisher = self.create_publisher(Image, "myvehicle/grid", 1)
        self.timer = self.create_timer(0.1, self.__timer_callback)

    def __on_gps(self, message):
        new_x, new_y = message.point.x, message.point.y
        if self.grid_origin_initialized:
            dx, dy = new_x - self.vehicle_world_x, new_y - self.vehicle_world_y
            if dx*dx + dy*dy > 10000:  # 100m^2 threshold # distance formula
                return
        self.vehicle_world_x, self.vehicle_world_y = new_x, new_y
        if not self.grid_origin_initialized:
            self.grid_origin_world_x = self.vehicle_world_x - self.desired_vehicle_grid_col * self.resolution_m_per_cell[0]
            self.grid_origin_world_y = self.vehicle_world_y - self.desired_vehicle_grid_row * self.resolution_m_per_cell[1]
            self.grid_origin_initialized = True
        else:
            target_x = self.vehicle_world_x - self.desired_vehicle_grid_col * self.resolution_m_per_cell[0]
            target_y = self.vehicle_world_y - self.desired_vehicle_grid_row * self.resolution_m_per_cell[1]
            if abs(self.grid_origin_world_x - target_x) > 0.6 or abs(self.grid_origin_world_y - target_y) > 0.6:
                # Set the target and flag for update, but do not call the update function here
                self.target_grid_origin_x = target_x
                self.target_grid_origin_y = target_y
                self.pending_grid_update = True

    def __on_imu(self, message):
        _, yaw, _ = getAnglesFromQuaternion(message.orientation)
        self.vehicle_world_yaw = yaw

    def update_grid_anchor(self, new_x, new_y):
        shift_cols = round((self.grid_origin_world_x - new_x) / self.resolution_m_per_cell[0])
        shift_rows = round((self.grid_origin_world_y - new_y) / self.resolution_m_per_cell[1])
        new_grid = np.full_like(self.grid_data, [0.0, 0.0, 1.0])
        
        old_start_row, old_end_row = max(0, -shift_rows), min(self.grid_height_cells, self.grid_height_cells - shift_rows)
        old_start_col, old_end_col = max(0, -shift_cols), min(self.grid_width_cells, self.grid_width_cells - shift_cols)
        
        new_start_row, new_end_row = max(0, shift_rows), min(self.grid_height_cells, self.grid_height_cells + shift_rows)
        new_start_col, new_end_col = max(0, shift_cols), min(self.grid_width_cells, self.grid_width_cells + shift_cols)
        
        height = min(old_end_row - old_start_row, new_end_row - new_start_row)
        width = min(old_end_col - old_start_col, new_end_col - new_start_col)
        
        if height > 0 and width > 0:
            new_grid[new_start_row:new_start_row + height, new_start_col:new_start_col + width] = \
                self.grid_data[old_start_row:old_start_row + height, old_start_col:old_start_col + width]
        self.grid_data = new_grid
        self.grid_origin_world_x, self.grid_origin_world_y = new_x, new_y

    def local_to_global(self, local_x, local_y):
        local_y = -local_y  # Invert y-axis to fix left-right mismatch
        rotated_x = local_x * math.cos(self.vehicle_world_yaw) - local_y * math.sin(self.vehicle_world_yaw)
        rotated_y = local_x * math.sin(self.vehicle_world_yaw) + local_y * math.cos(self.vehicle_world_yaw)
        return rotated_x + self.vehicle_world_x, rotated_y + self.vehicle_world_y

    def __timer_callback(self):
        self.degrade_ds_grid_belief()
        self._grid_publisher.publish(self.show_ds_grid())

    def __on_lidar(self, message):
        # Check if an update is pending from the GPS and apply it
        if self.pending_grid_update:
            self.update_grid_anchor(self.target_grid_origin_x, self.target_grid_origin_y)
            self.pending_grid_update = False

        if not self.grid_origin_initialized:
            return
            
        data = np.frombuffer(message.data, dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32), ("layer", np.int32), ("time", np.float32)])
        if not data.size:
            return
        
        vehicle_col_idx = int((self.vehicle_world_x - self.grid_origin_world_x) / self.resolution_m_per_cell[0])
        vehicle_row_idx = int((self.vehicle_world_y - self.grid_origin_world_y) / self.resolution_m_per_cell[1])
        
        if not (0 <= vehicle_col_idx < self.grid_width_cells and 0 <= vehicle_row_idx < self.grid_height_cells):
            return
        
        for p in data:
            local_x, local_y = p["x"], p["y"]
            if math.isinf(local_x) or math.isnan(local_x) or math.isinf(local_y) or math.isnan(local_y):
                continue
            
            global_x, global_y = self.local_to_global(local_x, local_y)
            if math.isinf(global_x) or math.isnan(global_x) or math.isinf(global_y) or math.isnan(global_y):
                continue
            
            distance = math.sqrt((global_x - self.vehicle_world_x)**2 + (global_y - self.vehicle_world_y)**2)
            
            hit_col_idx = int((global_x - self.grid_origin_world_x) / self.resolution_m_per_cell[0])
            hit_row_idx = int((global_y - self.grid_origin_world_y) / self.resolution_m_per_cell[1])
            
            is_max_range = distance >= self.lidar_max_range - 1e-3
            
            if is_max_range:
                ray_angle = math.atan2(global_y - self.vehicle_world_y, global_x - self.vehicle_world_x)
                max_x = self.vehicle_world_x + self.lidar_max_range * math.cos(ray_angle)
                max_y = self.vehicle_world_y + self.lidar_max_range * math.sin(ray_angle)
                end_col = int((max_x - self.grid_origin_world_x) / self.resolution_m_per_cell[0])
                end_row = int((max_y - self.grid_origin_world_y) / self.resolution_m_per_cell[1])
            else:
                end_col, end_row = hit_col_idx, hit_row_idx
            end_col_clipped = np.clip(end_col, 0, self.grid_width_cells - 1)
            end_row_clipped = np.clip(end_row, 0, self.grid_height_cells - 1)
            path_cells = self.bresenham([vehicle_row_idx, vehicle_col_idx], [end_row_clipped, end_col_clipped])
            hit_cell = (-1, -1) if is_max_range else (hit_row_idx, hit_col_idx)
            
            for r, c in path_cells:
                m_f, m_o, m_u = self.grid_data[r, c]
                is_hit = (r == hit_cell[0] and c == hit_cell[1])
                m_sensor_f = self.DS_SENSOR_HIT_FREE if is_hit else self.DS_SENSOR_MISS_FREE
                m_sensor_o = self.DS_SENSOR_HIT_OCCUPIED if is_hit else self.DS_SENSOR_MISS_OCCUPIED
                m_sensor_u = self.DS_SENSOR_HIT_UNKNOWN if is_hit else self.DS_SENSOR_MISS_UNKNOWN
                new_mf, new_mo, new_mu = self._dempster_shafer_combination(m_f, m_o, m_u, m_sensor_f, m_sensor_o, m_sensor_u)
                self.grid_data[r, c] = [new_mf, new_mo, new_mu]

    def _dempster_shafer_combination(self, m1_f, m1_o, m1_u, m2_f, m2_o, m2_u):
        K = m1_f * m2_o + m1_o * m2_f
        if abs(1.0 - K) < 1e-6:
            return m1_f, m1_o, m1_u
        norm_factor = 1.0 / (1.0 - K)
        m_f = norm_factor * (m1_f * m2_f + m1_f * m2_u + m1_u * m2_f)
        m_o = norm_factor * (m1_o * m2_o + m1_o * m2_u + m1_u * m2_o)
        m_u = 1.0 - m_f - m_o
        return np.clip(m_f, 0.0, 1.0), np.clip(m_o, 0.0, 1.0), np.clip(m_u, 0.0, 1.0)

    def bresenham(self, point_from_rc, point_to_rc):
        ret = []
        r0, c0 = point_from_rc
        r1, c1 = point_to_rc
        dr = abs(r1 - r0)
        sr = 1 if r0 < r1 else -1
        dc = -abs(c1 - c0)
        sc = 1 if c0 < c1 else -1
        error = dr + dc
        while True:
            ret.append([r0, c0])
            if r0 == r1 and c0 == c1:
                break
            e2 = 2 * error
            if e2 >= dc:
                if r0 == r1:
                    break
                error += dc
                r0 += sr
            if e2 <= dr:
                if c0 == c1:
                    break
                error += dr
                c0 += sc
        return ret

    def degrade_ds_grid_belief(self):
        decay = self.DS_DECAY_FACTOR
        m_f, m_o, m_u = self.grid_data[:, :, 0], self.grid_data[:, :, 1], self.grid_data[:, :, 2]
        new_m_f, new_m_o = m_f * decay, m_o * decay
        mass_shift = (m_f - new_m_f) + (m_o - new_m_o)
        self.grid_data[:, :, 0] = np.clip(new_m_f, 0.0, 1.0)
        self.grid_data[:, :, 1] = np.clip(new_m_o, 0.0, 1.0)
        self.grid_data[:, :, 2] = np.clip(m_u + mass_shift, 0.0, 1.0)

    def show_ds_grid(self):
        bgra_image = np.zeros((self.grid_height_cells, self.grid_width_cells, 4), dtype=np.uint8)
        bgra_image[:, :, 2] = (self.grid_data[:, :, 1] * 255).astype(np.uint8)  # Red for occupied
        bgra_image[:, :, 1] = (self.grid_data[:, :, 0] * 255).astype(np.uint8)  # Green for free
        bgra_image[:, :, 0] = (self.grid_data[:, :, 2] * 255).astype(np.uint8)  # Blue for unknown
        bgra_image[:, :, 3] = 255
        bridge = CvBridge()
        return bridge.cv2_to_imgmsg(bgra_image, encoding="bgra8")

def main(args=None):
    rclpy.init(args=args)
    agent = LIDARGrid()
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#include "lidar_grid.hpp"

#include "cv_bridge/cv_bridge.h"
#include <chrono>
#include <sstream>

using std::placeholders::_1;
using namespace std::chrono_literals;

// IGNORE THIS FUNCITON AND JUST USE IT!
geometry_msgs::msg::PointStamped
getAnglesFromQuadrion(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    geometry_msgs::msg::PointStamped result;
    result.header = msg->header;

    const geometry_msgs::msg::Quaternion &q(msg->orientation);

    // ENU: extrensic rotation matrix e = Z(yaw) Y(pitch) X(roll)
    const double t0 = 2.0 * (q.w * q.x + q.y * q.z);
    const double t1 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    const double roll = atan2(t0, t1);
    double t2 = 2.0 * (q.w * q.y - q.z * q.x);
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    const double pitch = asin(t2);
    const double t3 = 2.0 * (q.w * q.z + q.x * q.y);
    const double t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw = atan2(t3, t4);
    result.point.x = roll;
    result.point.y = pitch;
    result.point.z = yaw;

    return result;
}

#pragma pack(push, 1)
struct LIDARPoint
{
    float x, y, z;
    uint32_t layer;
    float time;
};
#pragma pack(pop)

LIDARGrid::LIDARGrid()
    : Node("fusion_grid_cpp"),
      m_grid(100, 100, CV_64FC1, cv::Scalar(0.0)), // initialize as undecided
      m_resolution{1, 1},                          // resolution 1m
      m_degradeValue(0.05) // how fast will information be lost
{
    declare_parameter<std::string>("vehicle");
    m_vehicle_topic = get_parameter("vehicle").as_string();

    set_parameter(
        rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

    // ROS interface
    m_lidarSubscription = create_subscription<sensor_msgs::msg::PointCloud2>(
        "vehicle/lidar/point_cloud",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&LIDARGrid::onLidar, this, _1));
    m_imuSubscription = create_subscription<sensor_msgs::msg::Imu>(
        "imu",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&LIDARGrid::onIMU, this, _1));
    m_gpsSubscription = create_subscription<geometry_msgs::msg::PointStamped>(
        m_vehicle_topic + "/gps",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&LIDARGrid::onGPS, this, _1));

    m_gridPublisher =
        create_publisher<sensor_msgs::msg::Image>("vehicle/grid", 1);

    m_timer = rclcpp::create_timer(this, get_clock(), 100ms,
                                   std::bind(&LIDARGrid::onTimer, this));
}

void LIDARGrid::onGPS(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "got gps %f, %f", msg->point.x, msg->point.y);
}

void LIDARGrid::onIMU(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    auto eulerAngles(getAnglesFromQuadrion(msg));
    RCLCPP_INFO(get_logger(), "got imu %f, %f, %f", eulerAngles.point.x,
                eulerAngles.point.y, eulerAngles.point.z);
}

std::shared_ptr<sensor_msgs::msg::Image> LIDARGrid::show() const
{
    cv::Mat scaled = m_grid * (255.0);
    cv::Mat grayscale;
    scaled.convertTo(grayscale, CV_8U);
    cv::Mat bgra;
    cv::cvtColor(grayscale, bgra, cv::COLOR_GRAY2BGRA);

    std_msgs::msg::Header header;
    cv_bridge::CvImage cvimage(header, "bgra", bgra);
    return cvimage.toImageMsg();
}

void LIDARGrid::onTimer()
{
    RCLCPP_INFO(get_logger(), "timer");
    degrade();
    m_gridPublisher->publish(*show());
}

void LIDARGrid::onLidar(const sensor_msgs::msg::PointCloud2::SharedPtr message)
{
    RCLCPP_INFO(get_logger(), "got point cloud");
    for (uint32_t i = 0; i < message->width * message->height; ++i)
    {
        // accessing point data really sucks...
        const LIDARPoint &p(
            *(reinterpret_cast<LIDARPoint *>(message->data.data()) + i));
        if (!isnan(p.x) && !isinf(p.x)) // do we got a reflection of this ray?
        {
            // convert to discrete coordinates
            DiscreteCoordinate cell{int32_t(p.x / m_resolution[0]),
                                    // move own vehicle to middle of grid
                                    int32_t(p.y / m_resolution[1]) +
                                        (m_grid.size().width / 2)};

            if (cell.x < m_grid.size().width && cell.x >= 0 &&
                cell.y < m_grid.size().height &&
                cell.y >= 0) // target is inside of the grid structure
            {
                std::vector<DiscreteCoordinate> cells;
                bresenham(0, m_grid.size().width / 2, cell.x, cell.y,
                          cells); // start in the middle of the grid
                for (const auto &c : cells)
                {
                    // update all up to the reflection
                    bayes(m_grid.at<double>(c.x, c.y), 0.4);
                }

                // update reflection
                bayes(m_grid.at<double>(cell.x, cell.y), 0.8);

                // RCLCPP_INFO(get_logger(), "update cell %d, %d", cell.x,
                // cell.y);
            }
        }
    }
}

void LIDARGrid::degrade()
{
    const auto denominatorValue = 1.0 + m_degradeValue;
    for (auto x = 0; x < m_grid.size().width; ++x)
    {
        for (auto y = 0; y < m_grid.size().height; ++y)
        {
            auto &cell(m_grid.at<double>(x, y));
            if (fabs(cell - 0.5) > 0.01) // if not undecided
            {
                if (cell < 0.5) // if below .5 add some value
                {
                    cell = 0.5 - ((0.5 - cell) / denominatorValue);
                }
                else
                { // if greater than .5 substract a bit
                    cell = 0.5 + ((cell - 0.5) / denominatorValue);
                }
            }
        }
    }
}

void LIDARGrid::bayes(double &value, const double &update)
{
    value = (value * update) /
            (value * update +
             (1.0 - value) * (1.0 - update)); // binary bayes formular
}

// @see https://de.wikipedia.org/wiki/Bresenham-Algorithmus
void LIDARGrid::bresenham(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
                          std::vector<DiscreteCoordinate> &cells)
{
    auto dx = abs(x1 - x0);
    auto sx = x0 < x1 ? 1 : -1;
    auto dy = -abs(y1 - y0);
    auto sy = y0 < y1 ? 1 : -1;
    auto err = dx + dy; /* error value e_xy */

    while (1)
    {
        if (x0 == x1 && y0 == y1)
            break;
        cells.push_back(DiscreteCoordinate{x0, y0});

        auto e2 = 2 * err;
        if (e2 >= dy)
        {
            if (x0 == x1)
            {
                cells.pop_back();
                break;
            }
            err += dy;
            x0 += sx;
        } /* e_xy+e_x > 0 */
        if (e2 <= dx)
        {
            if (y0 == y1)
            {
                cells.pop_back();
                break;
            }
            err += dx;
            y0 += sy;
        } /* e_xy+e_y < 0 */
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LIDARGrid>());
    rclcpp::shutdown();
    return 0;
}
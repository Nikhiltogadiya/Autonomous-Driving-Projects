#pragma once

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <opencv2/opencv.hpp>

class LIDARGrid : public rclcpp::Node
{
  public:
    LIDARGrid();

  private:
    struct DiscreteCoordinate
    {
        int32_t x;
        int32_t y;
    };
    std::string m_vehicle_topic;
    cv::Mat m_grid;
    double m_resolution[2];
    double m_degradeValue;

    std::shared_ptr<sensor_msgs::msg::Image> show() const;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        m_lidarSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
        m_gpsSubscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_gridPublisher;
    rclcpp::TimerBase::SharedPtr m_timer;

    void onTimer();
    void onLidar(const sensor_msgs::msg::PointCloud2::SharedPtr message);
    void onGPS(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void onIMU(const sensor_msgs::msg::Imu::SharedPtr msg);
    void degrade();
    void bayes(double &value, const double &update);
    void bresenham(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
                   std::vector<DiscreteCoordinate> &cells);
};

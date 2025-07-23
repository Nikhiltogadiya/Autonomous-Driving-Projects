#pragma once

#include "rclcpp/rclcpp.hpp"

#include <string>

#include "webots_interfaces/msg/radar_target.hpp"
#include <std_msgs/msg/float32.hpp>

/**
 * ACC class
 */
class ACCAgent : public rclcpp::Node
{
  public:
    // constructor
    ACCAgent();

  private:
    std::string m_vehicle_topic; ///< name of value

    rclcpp::Subscription<webots_interfaces::msg::RadarTarget>::SharedPtr
        m_radarSubscription; ///< radar subscription
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
        m_gpsSpeedSubscription; ///< gps speed subscription
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        m_speedPublisher;                 ///< speed publisher
    rclcpp::TimerBase::SharedPtr m_timer; ///< timer handle

    double m_radarDistance;             ///< distance to radar object
    double m_radarSpeed;                ///< speed of radar object
    rclcpp::Time m_lastRadarDistUpdate; ///< timestamp of last radar update
    float m_gpsSpeed;                   ///< gps speed
    rclcpp::Time m_lastGPSSpeedUpdate;  ///< timestamp of last gps update

    double m_currentValue; ///< last value that has been published
    rclcpp::Time m_start;  ///< start time

    /// IDM parameters
    double m_maxSpeed;
    double m_maxAcceleration;
    double m_accelerationExponent;
    double m_headwayTime;
    double m_comfortableDeceleration;
    double m_safetyDistance;

    /// timer callback
    void onTimer();
    /// gps speed callback
    void onGPSSpeed(const std_msgs::msg::Float32::SharedPtr message);
    /// radar callback
    void onRadar(const webots_interfaces::msg::RadarTarget::SharedPtr msg);
};

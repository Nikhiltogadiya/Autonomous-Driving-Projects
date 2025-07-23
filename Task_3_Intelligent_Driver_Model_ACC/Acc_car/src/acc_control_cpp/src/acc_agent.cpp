#include "acc_agent.hpp"

#include <chrono>
#include <sstream>

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * keep value within a range
 * @param value value to keen in range
 * @param min min value of range
 * @param max max value of range
 * @return value if value is in [min|max] or min or max
 */
template <typename T> T clamp(const T &value, const T &min, const T &max)
{
    return (value < min) ? min : (value > max) ? max : value;
}

/**
 * intelligent driver model
 * @see https://en.wikipedia.org/wiki/Intelligent_driver_model
 */
double intelligentDrivingModel(
    const double &desiredSpeed, const double &currentSpeed,
    const double &deltaSpeed, const double &currentGap,
    const double &maxAcceleration, const double &accelerationExponent,
    const double &headwayTime, const double &comfortableDeceleration,
    const double &safetyDist)
{

    auto accValue = maxAcceleration * (1.0 - pow(currentSpeed / desiredSpeed,
                                                 accelerationExponent));
    auto param_1 = headwayTime * currentSpeed;
    auto param_2 =
        (currentSpeed * deltaSpeed) /
        (2.0 * (pow(maxAcceleration * comfortableDeceleration, 0.5)));
    auto gap = fmax(safetyDist, param_1) + param_2;

    gap = (gap < 0.0) ? 0.0
                      : gap; // gap less than 0 - implies target faster than ego

    auto brakeValue = maxAcceleration * pow(gap / currentGap, 2);
    return accValue - brakeValue;
}

ACCAgent::ACCAgent()
    : Node("acc_agent_cpp"), m_radarDistance{-1}, m_gpsSpeed{-1}
{
    declare_parameter<std::string>("vehicle");
    m_vehicle_topic = get_parameter("vehicle").as_string();

    // ros parameter definition, can be used for tuining or hardcoded in
    // timer function
    declare_parameter<double>("max_speed", 0.0);
    m_maxSpeed = get_parameter("max_speed").as_double();
    declare_parameter<double>("max_acceleration", 0.0);
    m_maxAcceleration = get_parameter("max_acceleration").as_double();
    declare_parameter<double>("acceleration_exponent", 0.0);
    m_accelerationExponent = get_parameter("acceleration_exponent").as_double();
    declare_parameter<double>("headway_time", 0.0);
    m_headwayTime = get_parameter("headway_time").as_double();
    declare_parameter<double>("comfortable_deceleration", 0.0);
    m_comfortableDeceleration =
        get_parameter("comfortable_deceleration").as_double();
    declare_parameter<double>("safety_distance", 0.0);
    m_safetyDistance = get_parameter("safety_distance").as_double();

    set_parameter(
        rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

    m_currentValue = m_maxSpeed * 0.5;

    // ROS interface
    m_radarSubscription =
        create_subscription<webots_interfaces::msg::RadarTarget>(
            m_vehicle_topic + "/radar_targets",
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(
                            rmw_qos_profile_sensor_data),
                        rmw_qos_profile_sensor_data),
            std::bind(&ACCAgent::onRadar, this, _1));
    m_gpsSpeedSubscription = create_subscription<std_msgs::msg::Float32>(
        m_vehicle_topic + "/gps/speed",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&ACCAgent::onGPSSpeed, this, _1));
    m_speedPublisher = create_publisher<std_msgs::msg::Float32>(
        m_vehicle_topic + "/cmd_speed", 1);

    m_timer = rclcpp::create_timer(this, get_clock(), 100ms,
                                   std::bind(&ACCAgent::onTimer, this));
    m_start = get_clock()->now();
}

void ACCAgent::onTimer()
{
    RCLCPP_INFO(get_logger(), "timer");
    if (get_clock().get()->now() -
            rclcpp::Time(0, 0, get_clock()->get_clock_type()) <
        rclcpp::Duration(5, 0))
    {
        // wait 5s for other vehicle to start
        return;
    }
    std_msgs::msg::Float32 s;
    // initalize with old value to make sure there is a value
    // if IDM cannot be called
    s.data = m_currentValue;
    float acc = 0.0f;

    // calculate speed value here
    // take care of old, therefore, invalid data

    RCLCPP_INFO(get_logger(),
                "value %f speed %f dist %f headway time %f idm %f", s.data,
                m_gpsSpeed, m_radarDistance, m_radarDistance / m_gpsSpeed, acc);

    // publish new pedal value
    m_currentValue = s.data;
    m_speedPublisher->publish(s);
}

void ACCAgent::onGPSSpeed(const std_msgs::msg::Float32::SharedPtr msg)
{
    // RCLCPP_INFO(get_logger(), "got gps speed %f", msg->data);
    if (msg->data < 500) // sometimes there is very big value as first package
    {
        m_gpsSpeed = msg->data;
        m_lastGPSSpeedUpdate = get_clock()->now();
    }
}

void ACCAgent::onRadar(const webots_interfaces::msg::RadarTarget::SharedPtr msg)
{
    // RCLCPP_INFO(get_logger(), "got radar dist %f", msg->distance);
    if (msg->distance > 2) // ghost in first curve
    {
        m_radarDistance = msg->distance;
        m_radarSpeed = msg->speed;
        m_lastRadarDistUpdate = get_clock()->now();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ACCAgent>());
    rclcpp::shutdown();
    return 0;
}
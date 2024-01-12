#ifndef BATTERY_DIAGNOSTIC_CORE_HPP_
#define BATTERY_DIAGNOSTIC_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "battery_diagnostic/diagnostic_algorithm/battery_module.hpp"
#include "battery_diagnostic/diagnostic_algorithm/cell_voltage_analysis.hpp"

class BatteryDiagnosticCore : public rclcpp::Node {
public:
    BatteryDiagnosticCore();

private:
    void batteryDataCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_data_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_result_pub_;
};

#endif  // BATTERY_DIAGNOSTIC_CORE_HPP_

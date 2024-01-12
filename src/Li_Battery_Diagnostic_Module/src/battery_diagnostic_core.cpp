#include "battery_diagnostic/battery_diagnostic_core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

BatteryDiagnosticCore::BatteryDiagnosticCore() : Node("battery_diagnostic_core") {
    battery_data_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "battery_data_topic", 10,
        std::bind(&BatteryDiagnosticCore::batteryDataCallback, this, std::placeholders::_1));
    diagnostic_result_pub_ = this->create_publisher<std_msgs::msg::String>("diagnostic_result_topic", 10);
}

void BatteryDiagnosticCore::batteryDataCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    // TODO: Extract actual cell voltages from the BatteryState message
    std::vector<float> cell_voltages;  // Populate this with actual data from msg

    // Process data and execute diagnostic algorithm
    BatteryModule module(cell_voltages);
    CellVoltageAnalysis analysis(module);
    int faulty_cell_index = analysis.identifyFaultyCell();

    auto result_msg = std_msgs::msg::String();
    result_msg.data = "Faulty Cell Index: " + std::to_string(faulty_cell_index);
    diagnostic_result_pub_->publish(result_msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryDiagnosticCore>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

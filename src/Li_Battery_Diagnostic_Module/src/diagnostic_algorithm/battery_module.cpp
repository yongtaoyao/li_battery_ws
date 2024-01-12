#include "battery_diagnostic/diagnostic_algorithm/battery_module.hpp"

BatteryModule::BatteryModule(const std::vector<float>& cell_voltages) : cell_voltages_(cell_voltages) {}

float BatteryModule::calculateVoltageRange() const {
    // Implement logic to calculate and return the voltage range of the cells
    float max_voltage = *max_element(cell_voltages_.begin(), cell_voltages_.end());
    float min_voltage = *min_element(cell_voltages_.begin(), cell_voltages_.end());
    return max_voltage - min_voltage;
}

float BatteryModule::calculateAverageVoltage() const {
    // Implement logic to calculate and return the average voltage of the cells
    float sum = std::accumulate(cell_voltages_.begin(), cell_voltages_.end(), 0.0f);
    return sum / cell_voltages_.size();
}

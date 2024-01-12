#ifndef BATTERY_MODULE_HPP
#define BATTERY_MODULE_HPP
#include <algorithm> // For std::max_element and std::min_element
#include <numeric>   // For std::accumulate
#include <vector>

class BatteryModule {
public:
    BatteryModule(const std::vector<float>& cell_voltages);
    float calculateVoltageRange() const;
    float calculateAverageVoltage() const;
    // Other methods as needed
     // Method to get cell voltages
    const std::vector<float>& getCellVoltages() const { return cell_voltages_; }

private:
    std::vector<float> cell_voltages_;
};

#endif // BATTERY_MODULE_HPP

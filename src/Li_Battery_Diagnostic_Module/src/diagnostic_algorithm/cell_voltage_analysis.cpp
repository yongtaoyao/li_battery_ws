#include "battery_diagnostic/diagnostic_algorithm/cell_voltage_analysis.hpp"
#include <cmath> // For std::abs

CellVoltageAnalysis::CellVoltageAnalysis(const BatteryModule& module) : module_(module) {}
// Constructor and other methods as needed...

int CellVoltageAnalysis::identifyFaultyCell() {
    // Directly accessing the cell_voltages_ if it's public or accessible
    // Make sure this aligns with your actual BatteryModule class's implementation
    std::vector<float> cell_voltages = module_.getCellVoltages();  

    float average_voltage = module_.calculateAverageVoltage();
    float max_deviation = 0.0;
    int faulty_index = -1;

    for (int i = 0; i < cell_voltages.size(); ++i) {
        float deviation = std::abs(cell_voltages[i] - average_voltage);
        if (deviation > max_deviation) {
            max_deviation = deviation;
            faulty_index = i;
        }
    }
    return faulty_index;
}
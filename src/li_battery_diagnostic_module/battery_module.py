class BatteryModule:
    def __init__(self, cell_voltages):
        self.cell_voltages = cell_voltages
    
    def calculate_voltage_range(self):
        return max(self.cell_voltages) - min(self.cell_voltages)
    
    def calculate_average_voltage(self):
        return sum(self.cell_voltages) / len(self.cell_voltages)

class CellVoltageAnalysis:
    def __init__(self, module):
        self.module = module
    
    def identify_faulty_cell(self):
        average_voltage = self.module.calculate_average_voltage()
        deviations = [abs(voltage - average_voltage) for voltage in self.module.cell_voltages]
        return deviations.index(max(deviations))

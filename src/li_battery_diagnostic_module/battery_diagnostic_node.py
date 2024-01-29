import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from .battery_module import BatteryModule
from .cell_voltage_analysis import CellVoltageAnalysis

class BatteryDiagnosticNode(Node):
    def __init__(self):
        super().__init__('battery_diagnostic_node')
        self.subscription = self.create_subscription(
            BatteryState,
            'battery_data_topic',
            self.battery_data_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(String, 'diagnostic_result_topic', 10)

    def battery_data_callback(self, msg):
        cell_voltages = msg.cell_voltage  # Make sure this matches the actual message structure
        module = BatteryModule(cell_voltages)
        analysis = CellVoltageAnalysis(module)
        faulty_cell_index = analysis.identify_faulty_cell()
        
        result_msg = String()
        result_msg.data = f'Faulty Cell Index: {faulty_cell_index}'
        self.publisher_.publish(result_msg)


def main(args=None):
    rclpy.init(args=args)
    battery_diagnostic_node = BatteryDiagnosticNode()
    rclpy.spin(battery_diagnostic_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import csv
from rosbag2_py import (
    SequentialWriter, 
    StorageOptions, 
    ConverterOptions, 
    TopicMetadata
)
from sensor_msgs.msg import BatteryState
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
import rclpy.time

class CSVToRosbagConverter:
    def __init__(self, csv_file, rosbag_file):
        self.csv_file = csv_file
        self.rosbag_file = rosbag_file

    def convert(self):
        storage_options = StorageOptions(uri=self.rosbag_file, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        writer = SequentialWriter()

        writer.open(storage_options, converter_options)
        
        topic_metadata = TopicMetadata(
            name='/battery_data_topic',
            type='sensor_msgs/msg/BatteryState',
            serialization_format='cdr',
            offered_qos_profiles=''
        )
        writer.create_topic(topic_metadata)
        
        msg_type = get_message(topic_metadata.type)
        
        with open(self.csv_file, 'r') as file:
            csv_reader = csv.reader(file, delimiter=',')
            for row in csv_reader:
                message = msg_type()
                message.header.stamp = rclpy.time.Time(seconds=int(row[0])).to_msg()  # Assuming first column is a timestamp
                message.current = float(row[1]) # Assuming second column is current
                message.cell_voltage = [float(voltage) for voltage in row[2:98]]  # Assuming next 96 columns are cell voltages
                serialized_msg = serialize_message(message)
                writer.write('/battery_data_topic', serialized_msg, int(row[0]))
        
        # writer.close()


if __name__ == '__main__':
    csv_file = 'battery_data_1.csv'
    rosbag_file = 'battery_data_1.bag'
    converter = CSVToRosbagConverter(csv_file, rosbag_file)
    converter.convert()


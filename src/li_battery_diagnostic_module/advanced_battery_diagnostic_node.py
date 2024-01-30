import numpy as np
from sklearn.svm import SVC  # Support Vector Classification
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


class AdvancedBatteryDiagnosticNode(Node):
    def __init__(self):
        super().__init__('advanced_battery_diagnostic_node')
        self.data_samples = deque(maxlen=100)  # Keep the last 100 samples
        self.subscription = self.create_subscription(
            BatteryState,
            'battery_data_topic',
            self.battery_data_callback,
            10)
        self.diagnostic_result_publisher = self.create_publisher(String, 'advanced_diagnostic_result_topic', 10)
        # Initialize your machine learning model (SVM or KMeans)
        # self.model = SVC()  # Example, change as needed
        self.scaler = StandardScaler()

        self.model = KMeans(n_clusters=2)  # Example, adjust as needed
        self.is_model_trained = False

    def battery_data_callback(self, msg):
        current = msg.current
        voltages = msg.cell_voltage
        
        # Append the current sample to the data_samples
        self.data_samples.append([current] + list(voltages))

        # self.get_logger().info(f'len(self.data_samples): {len(self.data_samples)}')
        
        # If we have 100 samples, start the analysis
        if len(self.data_samples) == 100:

            self.analyze_data()

    def analyze_data(self):
        # Extract features from the data_samples
        data = np.array(self.data_samples)
        self.get_logger().info(f'Line 46, {type(data)}, {data.shape}')
        features = self.extract_features(data)
        
        # Normalize the features
        features = self.scaler.fit_transform(features)
        
        # # Predict using your machine learning model
        # predictions = self.model.predict(features)
        
        # # use predictions to identify the faulty cell
        
        # # Publish the result
        # result_msg = BatteryState()  # Example, change as needed
        # self.publisher_.publish(result_msg)

        # Train the model if not already trained
        if not self.is_model_trained:
            self.model.fit(features)
            self.is_model_trained = True

        # Predict using your machine learning model
        predictions = self.model.predict(features)

        # Identify the faulty cell
        self.identify_faulty_cell(features, predictions)

    def identify_faulty_cell(self, features, predictions):
        # Find the index of the cell that is farthest from its cluster center
        distances = self.model.transform(features)  # Get distance from each point to each cluster center
        max_distance_index = np.argmax(np.min(distances, axis=1))

        # Index of the faulty cell
        faulty_cell_index = max_distance_index % 96  # Adjust based on your features structure

        # Log the faulty cell index
        self.get_logger().info(f'Faulty cell index: {faulty_cell_index}')

        # Publish the result to /diagnostic_result_topic
        result_msg = String()
        result_msg.data = f'Faulty cell index: {faulty_cell_index}'
        self.diagnostic_result_publisher.publish(result_msg)

    def delta_v_per_delta_i(self, data):
        delta_v = np.diff(data[:, 1:], axis=0)
        delta_i = np.diff(data[:, 0], axis=0)
        # Prevent division by zero
        delta_i[delta_i == 0] = np.nan
        return (delta_v.T / delta_i).T

    def delta_v_per_delta_t(self, data, delta_t=1):
        # Assuming delta_t is the time difference between each sample (1 second)
        return np.diff(data[:, 1:], axis=0) / delta_t

    def integral_i(self, data):
        # Trapezoidal rule to approximate the integral for current
        return np.trapz(data[:, 0], axis=0)

    def integral_v(self, data):
        # Trapezoidal rule to approximate the integral for each voltage
        return np.trapz(data[:, 1:], axis=0)

    def delta_v_per_integral_i(self, data):
        integral_i = self.integral_i(data)
        delta_v = np.diff(data[:, 1:], axis=0)
        # Prevent division by zero
        integral_i[integral_i == 0] = np.nan
        return (delta_v.T / integral_i).T

    def extract_features(self, data):
        # Calculate features
        feature_1 = self.delta_v_per_delta_i(data)
        feature_2 = self.delta_v_per_delta_t(data)
        # feature_3 = np.full((data.shape[0] - 1, 96), self.integral_i(data))  # Same value for all rows
        # feature_4 = np.full((data.shape[0] - 1, 96), self.integral_v(data))  # Same value for all rows
        # feature_5 = self.delta_v_per_integral_i(data)

        # Concatenate all features
        features = np.hstack((feature_1, feature_2))

        return features

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedBatteryDiagnosticNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Li Battery Diagnostic Module

This document provides instructions for building, running, and testing the Li Battery Diagnostic Module in a ROS 2 environment.

## Building the Module

To build the module, follow these steps:

1. **Navigate to your ROS 2 workspace**:

```bash
    cd ~/li_battery_ws
```

2. **Build the workspace**:

```bash
    colcon build --symlink-install
```

3. **Source the setup file**:

```bash
    source install/setup.bash
```

## Running the Module

After building the module, you can run it as follows:

1. **Launch the module**:

```bash
    ros2 launch li_battery_diagnostic_module battery_diagnostic.launch.xml
```

2. **Check the Active Topics**:

    To see if your topics (`/battery_data_topic` and `/diagnostic_result_topic`) are active, you can use:

```bash
    ros2 topic list
```

## Converting CSV Data to ROS Bag

To convert your CSV data to a ROS bag, use the provided Python script:

**Run the CSV to ROS bag conversion script**:

```bash
    python csv_to_rosbag.py
```

This script reads data from a specified CSV file, converts each row to a `BatteryState` ROS message, and writes those messages to a ROS bag file.

## Play the Rosbag to Publish the Data

Now that you have your data in a rosbag file, you can use ros2 bag play to publish the data.

1. **Publish the data**:

```bash
    ros2 bag play battery_data_1.bag
```
You can also manually publish a message to `/battery_data_topic`.

2. **Publish a test message**:

```bash
    ros2 topic pub /battery_data_topic sensor_msgs/msg/BatteryState "{current: 12.5, cell_voltage: [4.1, 4.2, ...]}"
```

## Testing the Module

After publish the data, you can listen for results on `/diagnostic_result_topic`.

**Listen for diagnostic results**:

```bash
    ros2 topic echo /diagnostic_result_topic
```

You should see the diagnostic results being published by your `li_battery_diagnostic_module` node.

## Recording Data Using Rosbag

You can record the data being published to the `/battery_data_topic`:

**Record the data**:

```bash
    ros2 bag record /battery_data_topic
```

Use this command while your node is publishing the battery data to record the messages.

---


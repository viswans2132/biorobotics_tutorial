import rosbag2_py
import pandas as pd
import importlib
import os

def convert_bag_to_csv(bag_path, output_dir):
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Set up the ROS 2 bag reader
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get list of topics and types
    topic_types = reader.get_all_topics_and_types()
    topics = {topic.name: topic.type for topic in topic_types}

    # Read messages and save to CSV
    while reader.has_next():
        (topic_name, data, timestamp) = reader.read_next()

        # Get the message type dynamically
        msg_type = topics[topic_name].replace("/", ".")
        try:
            module_name, class_name = msg_type.rsplit(".", 1)
            module = importlib.import_module(module_name)
            msg_class = getattr(module, class_name)
            msg = msg_class()
            msg.deserialize(data)
        except Exception as e:
            print(f"Failed to process topic {topic_name} of type {msg_type}: {e}")
            continue

        # Convert message to a dictionary
        msg_dict = {}
        for field in msg.__slots__:
            value = getattr(msg, field)
            msg_dict[field] = value

        # Convert dictionary to DataFrame
        df = pd.DataFrame([msg_dict])

        # Save to CSV file (one per topic)
        csv_filename = os.path.join(output_dir, f"{topic_name.replace('/', '_')}.csv")
        df.to_csv(csv_filename, mode='a', index=False, header=not os.path.exists(csv_filename))

    print(f"Conversion complete! CSV files saved in: {output_dir}")

# Example usage
bag_path = "../../../g1/rosbag2_2025_03_11-12_53_18/rosbag2_2025_03_11-12_53_18_0.db3"  # Replace with your bag file path
output_dir = "./csv_output"
convert_bag_to_csv(bag_path, output_dir)

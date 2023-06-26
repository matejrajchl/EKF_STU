from rosbag_reader import RosBagReader
from data_processor import DataProcessor
from plotter import PoseGraph
import numpy as np
import sys

def data_available_callback(state, z, t, src, plot_data):
    # Process the updated state here
    #print("New data available:")
    #print("State:", state)
    pose_graph.add_data_point(t, state)
    global last_z_b
    global last_z_f

    if src == 'front':
        pose_graph.add_measurement_point(t, plot_data, last_z_b, src)
        last_z_f = plot_data
    elif src == 'back':
        pose_graph.add_measurement_point(t, plot_data, last_z_f, src)
        last_z_b = plot_data

if len(sys.argv) != 2:
    exp_name = 'default'
else:
    exp_name = sys.argv[1]

print("Experiment name is:", exp_name)

pose_graph = PoseGraph(exp_name)
last_z_f = [0] * 16
last_z_f[3] = 1

last_z_b = [0] * 16
last_z_b[3] = 1


# Create an instance of the DataProcessor
initial_state = [0] * 16  # Initial state vector
initial_state[3] = 1
initial_covariance = np.eye(16)  # Initial covariance matrix

data_processor = DataProcessor(initial_state, initial_covariance)
data_processor.set_data_available_callback(data_available_callback)

# Create an instance of the RosBagReader
bag_file = "2023-06-23-17-01-09.bag"
bag_reader = RosBagReader(bag_file)

bag_reader.read_all_names()

# Register the callbacks for the topics
bag_reader.register_callback('/ae_lsm6dso32x/imu', data_processor.update_step)
bag_reader.register_callback('/ov9281_back/visual_odometry/transformed/odom', data_processor.measurement_step)
bag_reader.register_callback('/ov9281_front/visual_odometry/transformed/odom', data_processor.measurement_step)

# Start reading the bag file
bag_reader.start_reading()
pose_graph.final_plot()


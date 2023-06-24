from rosbag_reader import RosBagReader
from data_processor import DataProcessor
from plotter import PoseGraph
import numpy as np

pose_graph = PoseGraph()

def data_available_callback(state,z,t):
    # Process the updated state here
    #print("New data available:")
    #print("State:", state)
    pose_graph.add_data_point(state[0],state[1],state[2],state[3],state[4],state[5],state[6])
    pose_graph.add_measurement_point(z[0],z[1],z[2],z[3],z[4],z[5],z[6])

    #pose_graph.plot_live_graph()
    

# Create an instance of the DataProcessor
initial_state = [0] * 16  # Initial state vector
initial_state[3] = 1
initial_covariance = np.eye(16)  # Initial covariance matrix

data_processor = DataProcessor(initial_state, initial_covariance)
data_processor.set_data_available_callback(data_available_callback)

# Create an instance of the RosBagReader
bag_file = "2023-06-19-16-35-47.bag"
bag_reader = RosBagReader(bag_file)

bag_reader.read_all_names()

# Register the callbacks for the topics
bag_reader.register_callback('/ae_lsm6dso32x/imu', data_processor.update_step)
bag_reader.register_callback('/ov9281_back/visual_odometry/transformed/odom', data_processor.measurement_step)

# Start reading the bag file
bag_reader.start_reading()
pose_graph.final_plot()


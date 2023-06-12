import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

class PoseGraph:
    def __init__(self):
        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(8, 10))
        self.line1, = self.ax1.plot([], [], 'b-')
        self.line2, = self.ax2.plot([], [], 'g-')
        self.line3, = self.ax3.plot([], [], 'r-')

    def add_data_point(self, x, y, z):
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)
    
    def update_graph(self, frame):
        self.line1.set_data(range(len(self.x_data)), self.x_data)
        self.ax1.relim()
        self.ax1.autoscale_view()
        
        self.line2.set_data(range(len(self.y_data)), self.y_data)
        self.ax2.relim()
        self.ax2.autoscale_view()
        
        self.line3.set_data(range(len(self.z_data)), self.z_data)
        self.ax3.relim()
        self.ax3.autoscale_view()
    
    def plot_live_graph(self):
        self.animation = FuncAnimation(self.fig, self.update_graph, frames=range(len(self.x_data)), interval=200)

        self.ax1.set_title('X Coordinate')
        self.ax1.set_xlabel('Time')
        self.ax1.set_ylabel('X')
        
        self.ax2.set_title('Y Coordinate')
        self.ax2.set_xlabel('Time')
        self.ax2.set_ylabel('Y')
        
        self.ax3.set_title('Z Coordinate')
        self.ax3.set_xlabel('Time')
        self.ax3.set_ylabel('Z')
        
        plt.tight_layout()
        plt.show(block = False)
        plt.pause(0.01)


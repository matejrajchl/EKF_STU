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

        self.x_data_z = []
        self.y_data_z = []
        self.z_data_z = []

        self.q1_data = []
        self.q2_data = []
        self.q3_data = []
        self.q4_data = []

        self.q1_data_z = []
        self.q2_data_z = []
        self.q3_data_z = []
        self.q4_data_z = []

        self.fig, (self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6, self.ax7) = plt.subplots(7, 1, figsize=(8, 10))
        self.line1, = self.ax1.plot([], [], 'b-')
        self.line1z, = self.ax1.plot([], [], 'r--')
        self.line2, = self.ax2.plot([], [], 'b-')
        self.line2z, = self.ax2.plot([], [], 'r--')
        self.line3, = self.ax3.plot([], [], 'b-')
        self.line3z, = self.ax3.plot([], [], 'r--')

        self.line4, = self.ax4.plot([], [], 'b-')
        self.line4z, = self.ax4.plot([], [], 'r--')
        self.line5, = self.ax5.plot([], [], 'b-')
        self.line5z, = self.ax5.plot([], [], 'r--')
        self.line6, = self.ax6.plot([], [], 'b-')
        self.line6z, = self.ax6.plot([], [], 'r--')
        self.line7, = self.ax7.plot([], [], 'b-')
        self.line7z, = self.ax7.plot([], [], 'r--')

    def add_data_point(self, x, y, z, q1,q2,q3,q4):
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

        self.q1_data.append(q1)
        self.q2_data.append(q2)
        self.q3_data.append(q3)
        self.q4_data.append(q4)

    def add_measurement_point(self, x, y, z,q1,q2,q3,q4):
        self.x_data_z.append(x)
        self.y_data_z.append(y)
        self.z_data_z.append(z)

        self.q1_data_z.append(q1)
        self.q2_data_z.append(q2)
        self.q3_data_z.append(q3)
        self.q4_data_z.append(q4)

    def final_plot(self):
        self.line1.set_data(range(len(self.x_data)), self.x_data)
        self.line1z.set_data(range(len(self.x_data_z)), self.x_data_z)
        self.ax1.relim()
        self.ax1.autoscale_view()
        
        self.line2.set_data(range(len(self.y_data)), self.y_data)
        self.line2z.set_data(range(len(self.y_data_z)), self.y_data_z)
        self.ax2.relim()
        self.ax2.autoscale_view()
        
        self.line3.set_data(range(len(self.z_data)), self.z_data)
        self.line3z.set_data(range(len(self.z_data_z)), self.z_data_z)
        self.ax3.relim()
        self.ax3.autoscale_view()

        self.line4.set_data(range(len(self.q1_data)), self.q1_data)
        self.line4z.set_data(range(len(self.q1_data_z)), self.q1_data_z)
        self.ax4.relim()
        self.ax4.autoscale_view()

        self.line5.set_data(range(len(self.q2_data)), self.q2_data)
        self.line5z.set_data(range(len(self.q2_data_z)), self.q2_data_z)
        self.ax5.relim()
        self.ax5.autoscale_view()

        self.line6.set_data(range(len(self.q3_data)), self.q3_data)
        self.line6z.set_data(range(len(self.q3_data_z)), self.q3_data_z)
        self.ax6.relim()
        self.ax6.autoscale_view()

        self.line7.set_data(range(len(self.q4_data)), self.q4_data)
        self.line7z.set_data(range(len(self.q4_data_z)), self.q4_data_z)
        self.ax7.relim()
        self.ax7.autoscale_view()

        plt.tight_layout()
        plt.show()
    
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


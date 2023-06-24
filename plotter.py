import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class PoseGraph:
    def __init__(self, experiment_name='default'):
        self.experiment_name = experiment_name
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


        self.x_data_b = []
        self.y_data_b = []
        self.z_data_b = []
        self.q1_data_b = []
        self.q2_data_b = []
        self.q3_data_b = []
        self.q4_data_b = []

        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, constrained_layout=True, figsize=(8, 10))
        self.fig.suptitle(self.experiment_name)
        

        self.line1, = self.ax1.plot([], [], 'b-')
        self.line1z, = self.ax1.plot([], [], 'r--')
        self.line2, = self.ax2.plot([], [], 'b-')
        self.line2z, = self.ax2.plot([], [], 'r--')
        self.line3, = self.ax3.plot([], [], 'b-')
        self.line3z, = self.ax3.plot([], [], 'r--')

        self.line1b, = self.ax1.plot([], [], 'g--')
        self.line2b, = self.ax2.plot([], [], 'g--')
        self.line3b, = self.ax3.plot([], [], 'g--')

        self.fig2, (self.ax4, self.ax5, self.ax6, self.ax7) = plt.subplots(4, 1, constrained_layout=True, figsize=(8, 10))
        self.fig2.suptitle(self.experiment_name)


        self.line4b, = self.ax4.plot([], [], 'g--')
        self.line5b, = self.ax5.plot([], [], 'g--')
        self.line6b, = self.ax6.plot([], [], 'g--')
        self.line7b, = self.ax7.plot([], [], 'g--')
        self.line4, = self.ax4.plot([], [], 'b-')
        self.line4z, = self.ax4.plot([], [], 'r--')
        self.line5, = self.ax5.plot([], [], 'b-')
        self.line5z, = self.ax5.plot([], [], 'r--')
        self.line6, = self.ax6.plot([], [], 'b-')
        self.line6z, = self.ax6.plot([], [], 'r--')
        self.line7, = self.ax7.plot([], [], 'b-')
        self.line7z, = self.ax7.plot([], [], 'r--')

    def add_data_point(self, t, data):
        self.x_data.append(data[0])
        self.y_data.append(data[1])
        self.z_data.append(data[2])
        self.q1_data.append(data[3])
        self.q2_data.append(data[4])
        self.q3_data.append(data[5])
        self.q4_data.append(data[6])

    def add_measurement_point(self, t, data, last_data_dummy,  src):
        if src == 'back':
            self.x_data_b.append(data[0])
            self.y_data_b.append(data[1])
            self.z_data_b.append(data[2])
            self.q1_data_b.append(data[3])
            self.q2_data_b.append(data[4])
            self.q3_data_b.append(data[5])
            self.q4_data_b.append(data[6])

            self.x_data_z.append(last_data_dummy[0])
            self.y_data_z.append(last_data_dummy[1])
            self.z_data_z.append(last_data_dummy[2])
            self.q1_data_z.append(last_data_dummy[3])
            self.q2_data_z.append(last_data_dummy[4])
            self.q3_data_z.append(last_data_dummy[5])
            self.q4_data_z.append(last_data_dummy[6])

        elif src == 'front':
            self.x_data_z.append(data[0])
            self.y_data_z.append(data[1])
            self.z_data_z.append(data[2])
            self.q1_data_z.append(data[3])
            self.q2_data_z.append(data[4])
            self.q3_data_z.append(data[5])
            self.q4_data_z.append(data[6])

            self.x_data_b.append(last_data_dummy[0])
            self.y_data_b.append(last_data_dummy[1])
            self.z_data_b.append(last_data_dummy[2])
            self.q1_data_b.append(last_data_dummy[3])
            self.q2_data_b.append(last_data_dummy[4])
            self.q3_data_b.append(last_data_dummy[5])
            self.q4_data_b.append(last_data_dummy[6])

    def final_plot(self):
        plt.title(self.experiment_name)

        self.line1.set_data(range(len(self.x_data)), self.x_data)
        self.line1z.set_data(range(len(self.x_data_z)), self.x_data_z)
        self.line1b.set_data(range(len(self.x_data_b)), self.x_data_b)
        self.ax1.relim()
        self.ax1.autoscale_view()
        
        self.line2.set_data(range(len(self.y_data)), self.y_data)
        self.line2z.set_data(range(len(self.y_data_z)), self.y_data_z)
        self.line2b.set_data(range(len(self.y_data_b)), self.y_data_b)
        self.ax2.relim()
        self.ax2.autoscale_view()
        
        self.line3.set_data(range(len(self.z_data)), self.z_data)
        self.line3z.set_data(range(len(self.z_data_z)), self.z_data_z)
        self.line3b.set_data(range(len(self.z_data_b)), self.z_data_b)
        self.ax3.relim()
        self.ax3.autoscale_view()

        self.ax1.set_title('X Coordinate')
        self.ax1.set_xlabel('Sample')
        self.ax1.set_ylabel('X')
        
        self.ax2.set_title('Y Coordinate')
        self.ax2.set_xlabel('Sample')
        self.ax2.set_ylabel('Y')
        
        self.ax3.set_title('Z Coordinate')
        self.ax3.set_xlabel('Sample')
        self.ax3.set_ylabel('Z')
        self.ax1.legend([ 'ekf','front','back'])


        self.line4.set_data(range(len(self.q1_data)), self.q1_data)
        self.line4z.set_data(range(len(self.q1_data_z)), self.q1_data_z)
        self.line4b.set_data(range(len(self.q1_data_b)), self.q1_data_b)

        self.ax4.relim()
        self.ax4.autoscale_view()
        self.ax4.legend([ 'ekf','front','back'])

        self.line5.set_data(range(len(self.q2_data)), self.q2_data)
        self.line5z.set_data(range(len(self.q2_data_z)), self.q2_data_z)
        self.line5b.set_data(range(len(self.q2_data_b)), self.q2_data_b)
        self.ax5.relim()
        self.ax5.autoscale_view()

        self.line6.set_data(range(len(self.q3_data)), self.q3_data)
        self.line6z.set_data(range(len(self.q3_data_z)), self.q3_data_z)
        self.line6b.set_data(range(len(self.q3_data_b)), self.q3_data_b)
        self.ax6.relim()
        self.ax6.autoscale_view()

        self.line7.set_data(range(len(self.q4_data)), self.q4_data)
        self.line7z.set_data(range(len(self.q4_data_z)), self.q4_data_z)
        self.line7b.set_data(range(len(self.q4_data_b)), self.q4_data_b)
        self.ax7.relim()
        self.ax7.autoscale_view()

        self.ax4.set_title('w')
        self.ax4.set_xlabel('Sample')
        self.ax4.set_ylabel('w')
        
        self.ax5.set_title('x')
        self.ax5.set_xlabel('Sample')
        self.ax5.set_ylabel('x')
  
        self.ax6.set_title('y')
        self.ax6.set_xlabel('Sample')
        self.ax6.set_ylabel('y')      
        
        self.ax7.set_title('z')
        self.ax7.set_xlabel('Sample')
        self.ax7.set_ylabel('z')
        


        plt.savefig('./graphs/quat_'+ self.experiment_name +'.png')
        plt.close()
        plt.savefig('./graphs/transl_' + self.experiment_name + '.png')
        plt.show()


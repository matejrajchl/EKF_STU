import numpy as np
import rospy
from sympy import symbols, cos, sin, Matrix, eye, lambdify
import math
from time import time

class DataProcessor:
    def __init__(self, initial_state, initial_covariance):
        self.state = Matrix(initial_state)  # Initial state vector
        self.covariance = Matrix(initial_covariance)  # Initial covariance matrix
        self.updated_state_ready = False  # Flag to indicate if the updated state is ready
        self.data_available_callback = None  # Callback for notifying when new data is available
        self.last_update_time = None  # Timestamp of the last update step

        # Specify Noise Sigmas
        self.N_g = np.array([1,1,1])*100 # for gyroscope
        self.N_a = np.array([1,1,1])*100 # for accelerometer
        self.N_bias_g = np.array([1,1,1])*100  # for gyro bias
        self.N_bias_a = np.array([1,1,1])*100  # for accelerometer bias
        
        self.Q_t = np.diag(np.concatenate((self.N_g, self.N_a, self.N_bias_g, self.N_bias_a))) # Process noise
        self.R_t = np.diag(np.concatenate((np.ones(3) * 1, np.ones(4) * 1))) #  Measurement Noise: [position;roll_pitch_yaw;velocities];

        self.sigma_t_1 = np.diag(np.concatenate((np.zeros(4), self.N_g, self.N_a, self.N_bias_g, self.N_bias_a)))        # Define symbolic variables

        self.X = symbols('p1 p2 p3 q1 q2 q3 q4 p_dot1 p_dot2 p_dot3 b_g1 b_g2 b_g3 b_a1 b_a2 b_a3')
        self.U = symbols('u1 u2 u3 u4 u5 u6')
        self.N = symbols('n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12')
        self.V = symbols('v1 v2 v3 v4 v5 v6 v7')

        # Define symbolic functions
        #G = Matrix([[cos(self.X[4]), 0, -cos(self.X[3])*sin(self.X[4])],
        #            [0, 1, sin(self.X[3])],
        #            [sin(self.X[4]), 0, cos(self.X[3])*cos(self.X[4])]])

        #self.R = Matrix([[cos(self.X[5])*cos(self.X[4])-sin(self.X[3])*sin(self.X[5])*sin(self.X[4]), -cos(self.X[3])*sin(self.X[5]), cos(self.X[5])*sin(self.X[4])+cos(self.X[4])*sin(self.X[3])*sin(self.X[5])],
        #            [cos(self.X[4])*sin(self.X[5])+cos(self.X[5])*sin(self.X[3])*sin(self.X[4]), cos(self.X[3])*cos(self.X[5]), sin(self.X[5])*sin(self.X[4])-cos(self.X[5])*cos(self.X[4])*sin(self.X[3])],
        #            [-cos(self.X[3])*sin(self.X[4]), sin(self.X[3]), cos(self.X[3])*cos(self.X[4])]])


        self.R = Matrix([[1-2*(self.X[5]**2 + self.X[6]**2), 2*(self.X[4]*self.X[5]-self.X[6]*self.X[3]), 2*(self.X[4]*self.X[6] + self.X[5]*self.X[3])],
                    [2*(self.X[4]*self.X[5] + self.X[6]*self.X[3]), 1-2*(self.X[4]**2 + self.X[6]**2), 2*(self.X[5]*self.X[6] - self.X[4]*self.X[3])],
                    [2*(self.X[4]*self.X[6] - self.X[5]*self.X[3]), 2*(self.X[5]*self.X[6] + self.X[4]*self.X[3]), 1-2*(self.X[4]**2 + self.X[5]**2)]])

        #Ginv = G.inv()
        omega = Matrix([[self.U[0]-self.X[10]-self.N[0]],
                       [self.U[1]-self.X[11]-self.N[1]],
                       [self.U[2]-self.X[12]-self.N[2]]])
        Omega = Matrix([[0, -omega[0], -omega[1], -omega[2]],
                        [omega[0], 0, omega[2], -omega[1]],
                        [omega[1],-omega[2], 0, omega[0]],
                        [omega[2], omega[1], -omega[0], 0]])
        # Define process model
        self.process = Matrix([Matrix(self.X[7:10]),
                               0.5*Omega*Matrix(self.X[3:7]),
                               Matrix([0, 0, -9.81]) + self.R * (Matrix(self.U[3:6]) - Matrix(self.X[13:16]) - Matrix(self.N[3:6])),
                               Matrix(self.N[6:9]),
                               Matrix(self.N[9:12])])

        # Define measurement model
        self.measurement = Matrix(self.X[0:7]) + Matrix(self.V[0:7])

        # Convert tuples to matrices for Jacobian calculation
        X_mat = Matrix(self.X)
        U_mat = Matrix(self.U)
        N_mat = Matrix(self.N)
        V_mat = Matrix(self.V)

        # Define Jacobians
        self.A = self.process.jacobian(X_mat)
        self.B = self.process.jacobian(U_mat)
        self.U_t= self.process.jacobian(N_mat)
        self.C = self.measurement.jacobian(X_mat)
        self.W = self.measurement.jacobian(V_mat)

        # Convert symbolic expressions to functions
        self.process_fn = None
        self.measurement_fn = None
        self.A_fn = None
        self.B_fn = None
        self.U_t_fn = None
        self.W_fn = None
        self.C_fn = None
    
        self.update_process_fn()
        self.update_measurement_fn()

    def update_process_fn(self): 
        self.process_fn = lambdify(self.X + self.N + self.U, self.process, modules='numpy')
        self.A_fn = lambdify(self.X + self.N + self.U, self.A, modules='numpy')
        self.B_fn = lambdify(self.X + self.N + self.U, self.B, modules='numpy')
        self.U_t_fn = lambdify(self.X + self.N + self.U, self.U_t, modules='numpy')

    def update_measurement_fn(self):
        self.measurement_fn = lambdify(self.X + self.V, self.measurement, modules='numpy')
        self.C_fn = lambdify(self.X + self.V, self.C, modules='numpy')
        self.W_fn = lambdify(self.X + self.V, self.W, modules='numpy')

    def update_step(self, msg, t):
        # Extract input data for the update step
        if self.last_update_time == None:
            self.last_update_time = t
            return
        
        self.input_data = self.extract_input_data(msg)
        # Update the model based on the input data
        dt = (t - self.last_update_time).to_sec()
        F_t = np.eye(len(self.state)) + self.A_fn(*self.input_data)*dt
        V_t = self.U_t_fn(*self.input_data)*dt
        self.state_pred = self.state  + self.process_fn(*self.input_data)*dt   # Add process noise?
        self.sigma_bar_t = np.dot(np.dot(F_t,self.sigma_t_1),F_t.T) + np.dot(np.dot(V_t,self.Q_t), V_t.T) # Must use np.dot to multiply non square matrices
        self.updated_state_ready = True  # Set the flag to indicate that the updated state is ready
        self.last_update_time = t  # Update the timestamp of the last update step
        
        self.state = self.state_pred
        self.sigma_t_1 = self.sigma_bar_t

    def measurement_step(self, msg, t):
        if not self.updated_state_ready:
            return

        current_time = t
        time_elapsed = current_time - self.last_update_time

        if time_elapsed.to_sec() <= 0.05:
            
            C_t = self.C_fn(*np.concatenate((tuple(self.state_pred.T.tolist()[0]), np.zeros(7))))
            W_t = self.W_fn(*np.concatenate((tuple(self.state_pred.T.tolist()[0]), np.zeros(7))))
            # Kalman gain calculation

            D_inv = np.linalg.inv(np.dot(np.dot(C_t,self.sigma_bar_t), C_t.T) + W_t * self.R_t * W_t.T)  # Must use np.dot to multiply not square matrices

            K_t = np.dot(self.sigma_bar_t, np.dot(C_t.T,D_inv))

            # Update using measurement
            z = self.extract_measurement(msg)  # Extract measurement vector from the message
            
            self.state = self.state_pred + K_t*(Matrix(z) - self.measurement_fn(*np.concatenate((tuple(self.state_pred.T.tolist()[0]), np.zeros(7)))))  # Residual
        
            self.sigma_t = self.sigma_bar_t - np.dot(np.dot(K_t,C_t),self.sigma_bar_t)
            #print(K_t)
            self.state_pred = self.state
            self.sigma_t_1 = self.sigma_t
            self.updated_state_ready = False  # Reset the flag

            if self.data_available_callback is not None:
                self.data_available_callback(self.state, z, t)  # Notify the callback that new data is available with the timestamp
        else:
            print("Too late update:")
            print(time_elapsed)

    def compute_expected_measurement(self):
        return self.measurement_fn(self.state)

    def extract_input_data(self, msg):
        # Extract input data for the update step
        # Replace with your own implementation
        #print(msg)
        inputs = (msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z,)
        input_data = np.concatenate((list(self.state.T.tolist()[0]), np.zeros(12), inputs))  
        input_data = input_data.astype(float)
        return input_data
    
    def extract_measurement(self, msg):
        # Extract measurement vector from the message
        # Replace with your own implementation
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        #roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x**2 + y**2))
        #pitch = math.asin(2 * (w*y - z*x))
        #yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y**2 + z**2))
        meas_data = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, w, x, y, z)
        return meas_data

    def set_data_available_callback(self, callback):
        self.data_available_callback = callback

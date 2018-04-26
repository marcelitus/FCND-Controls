"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
from frame_utils import euler2RM

DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MAX_TORQUE = 1.0

class NonlinearController(object):

    def __init__(self):
        """Initialize the controller object and control gains"""
        delta = 0.75
        omega =  1.33
        T = 1/omega

        kp = (1/(T*T)) * (1 + 2 * delta)
       # kp = 2 * zeta * omega
   #     kd = omega*omega
        ki = 1/(T * T * T)
        kd = (1/(T)) * (1 + 2 * delta)

        print("Kp", kp)
        print("Ki", ki)
        print("Kd", kd)
        self.z_k_p = kp
        self.z_k_d = kd
        self.x_k_p = kp
        self.x_k_d = kd
        self.y_k_p = kp
        self.y_k_d = kd
        self.k_p_roll = 0.5
        self.k_p_pitch = 0.5
        self.k_p_yaw = 0.5
        self.k_p_p = 5.0
        self.k_p_q = 5.0
        self.k_p_r = 5.0

        self.g = -GRAVITY
        self.m = DRONE_MASS_KG

        return    

    def trajectory_control(self, position_trajectory, yaw_trajectory, time_trajectory, current_time):
        """Generate a commanded position, velocity and yaw based on the trajectory
        
        Args:
            position_trajectory: list of 3-element numpy arrays, NED positions
            yaw_trajectory: list yaw commands in radians
            time_trajectory: list of times (in seconds) that correspond to the position and yaw commands
            current_time: float corresponding to the current time in seconds
            
        Returns: tuple (commanded position, commanded velocity, commanded yaw)
                
        """

        ind_min = np.argmin(np.abs(np.array(time_trajectory) - current_time))
        time_ref = time_trajectory[ind_min]
        
        
        if current_time < time_ref:
            position0 = position_trajectory[ind_min - 1]
            position1 = position_trajectory[ind_min]
            
            time0 = time_trajectory[ind_min - 1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min - 1]
            
        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory) - 1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]
                
                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min + 1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min + 1]
            
        position_cmd = (position1 - position0) * \
                        (current_time - time0) / (time1 - time0) + position0
        velocity_cmd = (position1 - position0) / (time1 - time0)
        
        
        return (position_cmd, velocity_cmd, yaw_cmd)
    
    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                               acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command
            
        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """
             
        x_target = local_position_cmd[0]
        y_target = local_position_cmd[1]
        x_dot_target = local_velocity_cmd[0]
        y_dot_target = local_velocity_cmd[1]
        x_actual = local_position[0]
        y_actual = local_position[1]
        x_dot_actual = local_velocity[0]
        y_dot_actual = local_velocity[1]
        x_dot_dot_target = acceleration_ff[0]
        y_dot_dot_target = acceleration_ff[1]
        
        x_commanded = self.x_k_p * (x_target - x_actual) + self.x_k_d * (x_dot_target - x_dot_actual) + x_dot_dot_target
   
        y_commanded = self.y_k_p * (y_target - y_actual) + self.y_k_d * (y_dot_target - y_dot_actual) + y_dot_dot_target
    
        print("X_Commanded", x_commanded, "YCommanded", y_commanded)
        a = np.array([x_commanded, y_commanded])
        b = np.clip(a,-1,1)
        print ("First", b[0], "Second", b[1])
        return b
        
      #  return np.array([0.0, 0.0])
    
    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            attitude: the vehicle's current attitude, 3 element numpy array (roll, pitch, yaw) in radians
            acceleration_ff: feedforward acceleration command (+up)
            
        Returns: thrust command for the vehicle (+up)
        """
    
    
        z_target = altitude_cmd
        z_dot_target = vertical_velocity_cmd
        z_actual = altitude
        z_dot_actual = vertical_velocity
        z_dot_dot_target = acceleration_ff
        rot_mat = euler2RM(attitude[0], attitude[1], attitude[2])
        
        u_bar_1 = self.z_k_p * (z_target - z_actual) + self.z_k_d * (z_dot_target - z_dot_actual) + z_dot_dot_target
        b = rot_mat[2,2]
        c = (u_bar_1 - self.g) / b**2
        print("C", c)
        a = np.clip(c, 0, 10)
        print("NEWC", a)
        return a
         
     #   return 0.0
        
    
    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame
        
        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thruts command in Newton
            
        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """

        b_x_c_target = acceleration_cmd[0]
        b_y_c_target = acceleration_cmd[1]
        self.k_p_roll = attitude[0]
        self.k_p_pitch = attitude[1]
        self.k_p_yaw = attitude[2]
        rot_mat = euler2RM(self.k_p_roll,self.k_p_pitch,self.k_p_yaw)
        b_x = rot_mat[0,2]
        b_x_err = b_x_c_target - b_x
        b_x_p_term = self.k_p_roll * b_x_err
            
        b_y = rot_mat[1,2]
        b_y_err = b_y_c_target - b_y  
        b_y_p_term = self.k_p_pitch * b_y_err
            
        b_x_commanded_dot = b_x_p_term
        b_y_commanded_dot = b_y_p_term
            
        rot_mat1=np.array([[rot_mat[1,0],-rot_mat[0,0]],[rot_mat[1,1],-rot_mat[0,1]]])/rot_mat[2,2]
            
        rot_rate = np.matmul(rot_mat1,np.array([b_x_commanded_dot,b_y_commanded_dot]).T)
        p_c = rot_rate[0]
        q_c = rot_rate[1]
            
        return np.array([p_c, q_c])        
    #    return np.array([0.0, 0.0])
    
    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame
        
        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2
            
        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        p_c = body_rate_cmd[0]
        q_c = body_rate_cmd[1]
        r_c = body_rate_cmd[2]
        p_actual = body_rate[0]
        q_actual = body_rate[1]
        r_actual = body_rate[2]
        
        p_error = p_c - p_actual
        u_bar_p = self.k_p_p * p_error
    
        q_error = q_c - q_actual
        u_bar_q = self.k_p_q * q_error
    
        r_error = r_c - r_actual
        u_bar_r = self.k_p_r * r_error
    #    print(u_bar_p * MOI[0], u_bar_q * MOI[1], u_bar_r * MOI[2])
        print("u_bar_p", u_bar_p * MOI[0], "u_bar_q", u_bar_q * MOI[1], "u_bar_r", u_bar_r * MOI[2])
        return np.array([u_bar_p * MOI[0], u_bar_q * MOI[1], u_bar_r * MOI[2]])

  #      return np.array([0.0, 0.0, 0.0])
    
    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate
        
        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians
        
        Returns: target yawrate in radians/sec
        """
 
        r_c = self.k_p_yaw * (yaw_cmd - yaw)
        print("R_C", r_c)
        return r_c
     #   return 0.0
    

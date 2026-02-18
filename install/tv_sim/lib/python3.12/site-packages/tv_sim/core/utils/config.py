#TIRE PARAMS
import numpy as np


class VehicleConfig:
    def __init__(self):
        # --- Chassis
        self.m = 1500.0          # Total mass (kg) - light car
        self.g = 9.81           # Gravity (m/s^2)
        self.Iz = 2500.0         # Yaw moment of inertia (kg*m^2)
        self.h = 0.5          # Center of Gravity height (m)
        self.a = 1.2           # Distance from CoG to front axle (m)
        self.b = 1.4           # Distance from CoG to rear axle (m)
        self.L = self.a + self.b # Wheelbase (m)
        self.w = 1.6            # Track width (m)
        self.max_steer=0.6          #max steer angle
        self.front_area=2.2           #front area for aerodynamic loss
        self.drag_coefficient=0.3   #drag coefficient
        self.air_density=1.225      #air density kg/m^3
        
        # --- Wheel & Tire Constants 
        self.Iw = 1.8         # Rotational inertia of one wheel (kg*m^2)
        self.Re = 0.3         # Effective tire radius (m) (approx 16-inch tire)
        self.C_rr = 0.015       #rolling resistance
        self.lateral_damping=250    #lateral dampening of tire


        #Lugre Params
   
        self.sigma0 = np.array([45.0, 35.0])    # [Long, Lat] stiffness (m^-1)
        self.sigma1 = np.array([1.2, 1.0])      # [Long, Lat] damping (s/m)
        self.sigma2 = np.array([0.001, 0.001])  # [Long, Lat] viscous (s/m)
        self.mu_s = 0.7                         # Static friction 
        self.mu_k = 0.6                         # Kinetic friction 
        self.vs = 2.5                           # Stribeck velocity (m/s)

     
        # for controller (assume less complex friction)

        self.tire_long = self.get_bcd_from_lugre(0)# Fx parameters
        self.tire_lat  = self.get_bcd_from_lugre(1)# Fy parameters
        
        
                
        
        # --- Motor Specs ---
        self.max_torque = 300.0  # Max torque per motor (Nm)
        self.gear_ratio = 9.0  # Gear reduction from motor to wheel
        self.max_wheel_torque = self.max_torque * self.gear_ratio # ~250 Nm
        self.axial_viscous_dampening=1
        
        # --- Controller Gains ---

       
        self.kp_yaw =1000 # Proportional gain for TV logic
        self.ki_yaw = 10    # integral gain for TV logic
        self.kd_yaw = 100     # Derivative gain for TV logic

    def get_bcd_from_lugre(self,i):
        # D is the peak friction
        D = self.mu_k
        
        # Cornering stiffness 

        C_alpha = 0.5 * self.sigma0[i] * (0.12**2) 
        
        # C is usually ~1.3 
        C = 1.3
        Fz=self.m*self.g/4
        # Solve for B
        B = C_alpha / (C * D * Fz)

        E=0.8
        
        return [B, C, D,E]

    
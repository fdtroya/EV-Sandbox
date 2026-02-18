#Inputs (T1​,T2​,T3​,T4​) and steering angle (δ)
#Outputs State vector (x,y,ψ,ψ˙​,vx​,vy​,ωwheels​)


from tv_sim.core.models.tire_model import CombinedTireModel
from tv_sim.core.utils.config import VehicleConfig
import numpy as np

class Vehicle7DOF:
    def __init__(self, config:VehicleConfig):
        self.cfg = config
        # State: [vx, vy, yaw_rate, x, y, psi, w1, w2, w3, w4]
        self.state = np.zeros(10)
        self.state_dot=np.zeros(10)
        self.t=0
        self.state[0] = 0.0  # avoid division by zero in slip

        self.wheel_fx=np.zeros(4)
        self.wheel_fy=np.zeros(4)
        
    def get_f_z(self, ax, ay):
        """Calculates dynamic vertical load on each tire (Weight Transfer)"""
        m, g = self.cfg.m, self.cfg.g
        a, b, L, w, h = self.cfg.a, self.cfg.b, self.cfg.L, self.cfg.w, self.cfg.h
        
        # Static loads
        fz_f_static = (m*g*b) / (2*L)
        fz_r_static = (m*g*a) / (2*L)
        
        # Longitudinal transfer
        delta_fz_long = (m*ax*h) / (2*L)
        
        # Lateral transfer
        delta_fz_lat_f = (m*ay*h /w)*(b/L)
        delta_fz_lat_r = (m*ay* h /w)*(a/L)
        
        # Combined Fz for [FL, FR, RL, RR]
        fz = np.array([
            fz_f_static - delta_fz_long - delta_fz_lat_f, # FL
            fz_f_static - delta_fz_long + delta_fz_lat_f, # FR
            fz_r_static + delta_fz_long - delta_fz_lat_r, # RL
            fz_r_static + delta_fz_long + delta_fz_lat_r  # RR
        ])
        return np.maximum(fz, 100.0) # Prevent negative Fz

    def derivatives(self, state, t, torques, delta, tire_model:CombinedTireModel):
        vx, vy, r, x, y, psi = state[0:6]
        omega = state[6:10]
        
        alphas = [
            delta - np.arctan2(vy + self.cfg.a * r, vx - self.cfg.w/2 * r), # FL
            delta - np.arctan2(vy + self.cfg.a * r, vx + self.cfg.w/2 * r), # FR
            -np.arctan2(vy - self.cfg.b * r, vx - self.cfg.w/2 * r),        # RL
            -np.arctan2(vy - self.cfg.b * r, vx + self.cfg.w/2 * r)         # RR
        ]
            
        kappas = [(w * self.cfg.Re - vx) / (max(abs(vx), 0.1)) for w in omega]
        
        #Estimate accelerations from previous step for Weight Transfer
        ax_prev = self.state_dot[0]
        ay_prev = vx * r#vx * yaw rate
        fz = self.get_f_z(ax_prev, ay_prev)
        
        #Tire Forces
        fx = self.wheel_fx
        fy = self.wheel_fy
        
        for i in range(4):
            fx[i], fy[i] = tire_model.get_forces(kappas[i], alphas[i], fz[i])
            
         
        
        #disipation/losses
        f_drag = 0.5 * self.cfg.air_density * self.cfg.drag_coefficient * self.cfg.front_area * (vx**2) * np.sign(vx)
        # Rolling Resistance (Torque on Wheel)
        rolling_torques=np.zeros(4)
        for i, w in enumerate(omega):
            rolling_torques[i] = -np.sign(w) * fz[i] * self.cfg.C_rr * self.cfg.Re 
        f_lateral_damping = self.cfg.lateral_damping * vy if vy > 0.01 else 0




            
        # Sum Forces & Moments
        # Fx total
        sum_fx = (fx[0]+fx[1])*np.cos(delta) - (fy[0]+fy[1])*np.sin(delta) + fx[2] + fx[3] -f_drag
        # Fy total
        sum_fy = (fx[0]+fx[1])*np.sin(delta) + (fy[0]+fy[1])*np.cos(delta) + fy[2] + fy[3]-f_lateral_damping
        # Yaw Moment Mz
        mz = (self.cfg.a * (fy[0] + fy[1]) * np.cos(delta) - 
              self.cfg.b * (fy[2] + fy[3]) +
              self.cfg.w/2 * (fx[1] - fx[0] + fx[3] - fx[2]))

        #ODEs 
        dvx = sum_fx / self.cfg.m + vy * r
        dvy = sum_fy / self.cfg.m - vx * r
        dr  = mz / self.cfg.Iz
        
        dx = vx * np.cos(psi) - vy * np.sin(psi)
        dy = vx * np.sin(psi) + vy * np.cos(psi)
        dpsi = r
        
        domega = [(torques[i] - fx[i] * self.cfg.Re+ rolling_torques[i]) / self.cfg.Iw for i in range(4)]
        
        return np.concatenate(([dvx, dvy, dr, dx, dy, dpsi], domega))

    def step(self, torques, delta, tire_model, dt):
        """Runge-Kutta 4th Order Integration Step"""
        k1 = self.derivatives(self.state, self.t, torques, delta, tire_model)
        k2 = self.derivatives(self.state + 0.5 * dt * k1, self.t, torques, delta, tire_model)
        k3 = self.derivatives(self.state + 0.5 * dt * k2, self.t, torques, delta, tire_model)
        k4 = self.derivatives(self.state + dt * k3, self.t, torques, delta, tire_model)
        
        
        self.state_dot=(1/6.0) * (k1 + 2*k2 + 2*k3 + k4)#estimate derivative 
        self.state += (dt*self.state_dot)#could also re compute the derivative for this state
        self.t+=dt
        return self.state
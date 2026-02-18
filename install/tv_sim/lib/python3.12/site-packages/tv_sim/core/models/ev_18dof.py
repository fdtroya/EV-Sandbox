

from .tire_model import IndependentTire
from ..utils.config import VehicleConfig
import numpy as np

class Vehicle18DOF:
    def __init__(self, config: VehicleConfig):
        self.cfg = config
        # State: [vx, vy, r, x, y, psi, w1..4, zx1..4, zy1..4]
        self.state = np.zeros(18)
        self.state_dot = np.zeros(10)
        self.z_dots_buffer = np.zeros(8)
        self.t = 0
        self.tire_keys = ['FL', 'FR', 'RL', 'RR']
        self.tires = {k: IndependentTire(k, self.cfg) for k in self.tire_keys}
        self.z_states=np.zeros(8)
        self.torques=np.zeros(4)
        self.delta=0
        
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
        return np.maximum(fz, 0.0) # Prevent negative Fz

    def derivatives(self,state, torques, delta, dt):
            vx, vy, r, x, y, psi = state[0:6]
            omega = state[6:10]
            zx_vec = state[10:14]
            zy_vec = state[14:18]
            
            ax_prev = self.state_dot[0]
            ay_prev = vx * r 
            fz = self.get_f_z(ax_prev, ay_prev)
            
            fx, fy, domega,fx_chassis,fy_chassis = np.zeros(4), np.zeros(4), np.zeros(4), np.zeros(4), np.zeros(4)
            dzx,dzy=np.zeros(4),np.zeros(4)
            
            for i, key in enumerate(self.tire_keys):
                zx=zx_vec[i]
                zy=zy_vec[i]
                         
                # Determine distances based on tire position
                lf = self.cfg.a if 'F' in key else -self.cfg.b
                wf = -self.cfg.w/2 if 'L' in key else self.cfg.w/2
                
                # Velocity of the wheel hub in vehicle frame
                v_hub_x = vx - r * wf
                v_hub_y = vy + r * lf
                
                # Local tire frame velocities
                d = delta if 'F' in key else 0.0
                cos_d, sin_d = np.cos(d), np.sin(d)
                
                vx_local = v_hub_x * cos_d + v_hub_y * sin_d
                vy_local = -v_hub_x * sin_d + v_hub_y * cos_d
                
                # Slip velocities
                vrx = omega[i] * self.cfg.Re - vx_local
                vry = -vy_local 
                
                # Tire dyamics
            

                fx[i], fy[i],dzx[i],dzy[i] = self.tires[key].get_dynamics_rk2(vrx, vry, fz[i],zx,zy,dt)


                omega_ref = 1 

                # Smooth signum function
                soft_sgn = np.tanh(omega[i] / omega_ref)
                rolling_resistance=self.cfg.C_rr*fz[i]*self.cfg.Re*soft_sgn
                # 3. Wheel Acceleration (Uses TIRE FRAME fxi)
                domega[i] = (torques[i] - (fx[i] * self.cfg.Re)-(rolling_resistance)) / self.cfg.Iw
                
                # 4. Chassis Projection (Uses CHASSIS FRAME fx/fy)
                fx_chassis[i] = fx[i] * cos_d - fy[i] * sin_d
                fy_chassis[i] = fx[i] * sin_d + fy[i] * cos_d

            drag=0.5*self.cfg.drag_coefficient*self.cfg.air_density*self.cfg.front_area*vx * np.abs(vx)
            # chassis mechanics
            sum_fx = np.sum(fx_chassis)-drag
            sum_fy = np.sum(fy_chassis)
            
            #Mz
            
            mz = (self.cfg.a * (fy_chassis[0] + fy_chassis[1]) - 
            self.cfg.b * (fy_chassis[2] + fy_chassis[3]) + 
            self.cfg.w/2 * (fx_chassis[1] - fx_chassis[0] + fx_chassis[3] - fx_chassis[2]))

            

            dvx = sum_fx / self.cfg.m + vy * r
            dvy = sum_fy / self.cfg.m - vx * r
            dr  = mz / self.cfg.Iz
            
            return np.concatenate(([dvx, dvy, dr, vx*np.cos(psi)-vy*np.sin(psi), 
                                    vx*np.sin(psi)+vy*np.cos(psi), r], 
                                    domega,dzx,dzy))

    

    def step(self, torques, delta, dt, substeps=1, references=False):
        """
        Stabilized RK4 Integration step for 18-DOF LuGre Dynamics.
        
        Args:
            torques: Input torques
            delta: Steering angle
            dt: Total time step
            substeps (int): Number of integration substeps (default 1)
            references (bool): Whether to calculate and return debug values
        """
        # Calculate the time step for each substep
        sub_dt = dt / float(substeps)
        
        # --- Integration Loop ---
        for _ in range(int(substeps)):

            k1 = self.derivatives(self.state, torques, delta, 0.0)
            k2 = self.derivatives(self.state + 0.5 * sub_dt * k1, torques, delta, 0.5 * sub_dt)
            k3 = self.derivatives(self.state + 0.5 * sub_dt * k2, torques, delta, 0.5 * sub_dt)
            k4 = self.derivatives(self.state + sub_dt * k3, torques, delta, sub_dt)
            self.state += (sub_dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
            self.t += sub_dt
            

            self.state_dot = k1


        zx_new = self.state[10:14]
        zy_new = self.state[14:18]
        omega = self.state[6:10]
        vx = self.state[0]
        vy = self.state[1]
        r = self.state[2]
        
        # Use the final acceleration from the last substep
        ax = self.state_dot[0]
        ay = self.state_dot[1]
        fzs = self.get_f_z(ax, ay)

        F_frs = np.zeros(8)
        v_slip = np.zeros(8)
                
        # Update Tire Objects
        for i, key in enumerate(self.tire_keys):
            # Sync tire internal state
            self.tires[key].zx = zx_new[i]
            self.tires[key].zy = zy_new[i]
            self.tires[key].fz = fzs[i]
            
 
            if references:

                lf = self.cfg.a if 'F' in key else -self.cfg.b
                wf = -self.cfg.w/2 if 'L' in key else self.cfg.w/2
                

                v_hub_x = vx - r * wf
                v_hub_y = vy + r * lf
                

                d = delta if 'F' in key else 0.0
                cos_d, sin_d = np.cos(d), np.sin(d)
                
                vx_local = v_hub_x * cos_d + v_hub_y * sin_d
                vy_local = -v_hub_x * sin_d + v_hub_y * cos_d
                

                vrx = omega[i] * self.cfg.Re - vx_local
                vry = -vy_local
                
 
                F_frs[2*i], F_frs[2*i+1] = self.tires[key].get_dynamics(vrx, vry, fzs[i])
                v_slip[2*i], v_slip[2*i+1] = vrx, vry

        if references:
            return np.concatenate((self.state, F_frs, v_slip))

        return self.state


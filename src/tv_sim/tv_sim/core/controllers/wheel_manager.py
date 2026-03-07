
from .wheel_observer import WheelObserver
import numpy as np

class WheelManager:
    def __init__(self, cfg):
        self.cfg = cfg
        self.wheel_observer_fl = WheelObserver(cfg)
        self.wheel_observer_fr = WheelObserver(cfg)
        self.wheel_observer_rl = WheelObserver(cfg)
        self.wheel_observer_rr = WheelObserver(cfg)
    def update(self, t, vx):
        self.wheel_observer_fl.update(t, vx)
        self.wheel_observer_fr.update(t, vx)
        self.wheel_observer_rl.update(t, vx)
        self.wheel_observer_rr.update(t, vx)
    def get_max_yaw_rate(self, vx,ax,ay):
        self.update_f_z(ax, ay)
        fy_fl = self.wheel_observer_fl.get_max_fy()
        fy_fr = self.wheel_observer_fr.get_max_fy() 
        fy_rl = self.wheel_observer_rl.get_max_fy()
        fy_rr = self.wheel_observer_rr.get_max_fy()

        fy_front= fy_fl+ fy_fr
        fy_rear = fy_rl+ fy_rr
        fy_effective = 2*min(fy_front, fy_rear)
        ay_limit= fy_effective / (self.cfg.m )
        max_yaw_rate = ay_limit / vx
        return max_yaw_rate
    def get_max_mz(self):
 
        fx_max_fl = self.wheel_observer_fl.get_max_fx() # Max longitudinal force at current Fz and zero lateral demand
        fx_max_fr = self.wheel_observer_fr.get_max_fx()
        fx_max_rl = self.wheel_observer_rl.get_max_fx()     
        fx_max_rr = self.wheel_observer_rr.get_max_fx()
        self.max_Mz = (fx_max_fl + fx_max_rl + fx_max_fr + fx_max_rr) * (self.cfg.w / 2.0)
        return self.max_Mz
    
    def update_f_z(self, ax, ay):
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
        fz= np.maximum(fz, 1.0) # Prevent negative Fz

        self.wheel_observer_fl.fz=fz[0]
        self.wheel_observer_fr.fz=fz[1]
        self.wheel_observer_rl.fz=fz[2]
        self.wheel_observer_rr.fz=fz[3]
        return fz

import numpy as np
from ..utils.config import VehicleConfig

class ReferenceModel:
    def __init__(self, config: VehicleConfig, limit_factor=0.7):
        self.cfg = config
        
        # Safety Factor: 
        # 0.85 of what we think is peak frcition
        self.limit_factor = limit_factor 


        B, C, D, _ = self.cfg.tire_lat

        fz_f_static = (self.cfg.m * self.cfg.g * self.cfg.b) / (2 * self.cfg.L)
        fz_r_static = (self.cfg.m * self.cfg.g * self.cfg.a) / (2 * self.cfg.L)
        

        self.Cf = 2 * (B * C * D * fz_f_static)
        self.Cr = 2 * (B * C * D * fz_r_static)

        self.K_us = (self.cfg.m) * (
            (self.cfg.b / self.cfg.L) / self.Cf - 
            (self.cfg.a / self.cfg.L) / self.Cr
        )
        
        self.last_vx = 0
        self.filter_pass = 1.0 

    def get_target_yaw_rate(self, vx: float, delta: float, mu: float = None):
        """
        Calculates the desired yaw rate based on speed, steering
        """
        # Default to config mu if none provided, but allow dynamic updates
        if mu is None:
            mu = self.cfg.mu_s
        self.last_vx = vx

        if abs(vx) < 0.5:
            return 0.0
    
        yaw_target = (vx / (self.cfg.L + self.K_us * vx**2)) * delta
        max_yaw_rate = self.limit_factor * (mu * self.cfg.g / abs(vx))
        
        return np.clip(yaw_target, -max_yaw_rate, max_yaw_rate)
    
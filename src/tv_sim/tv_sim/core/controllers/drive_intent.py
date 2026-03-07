import numpy as np
from ..utils.config import VehicleConfig
from .wheel_manager import WheelManager

class ReferenceModel:
    def __init__(self, config: VehicleConfig, wheel_manager: WheelManager):
        self.cfg = config
        self.wheel_manager = wheel_manager 



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
        self.tau = 0.15 
        self.yaw_rate_prev = 0.0

    def update_K_us(self):
        # Summing axle loads
        fz_f = self.wheel_manager.wheel_observer_fl.fz + self.wheel_manager.wheel_observer_fr.fz
        fz_r = self.wheel_manager.wheel_observer_rl.fz + self.wheel_manager.wheel_observer_rr.fz

        
        self.c_alpha_f = self.cfg.lugre_normalized_stiffness * fz_f
        self.c_alpha_r = self.cfg.lugre_normalized_stiffness * fz_r


        # Use these for a stable, ideal Kus
        self.K_us = (self.cfg.m) * (
            (self.cfg.b / (self.cfg.L * self.c_alpha_f)) - 
            (self.cfg.a / (self.cfg.L * self.c_alpha_r))
        )

    def get_target_yaw_rate(self, vx: float, ax: float, ay: float, delta: float, dt: float ):
        """
        Calculates the desired yaw rate based on speed, steering
        """
      
        self.update_K_us()  # Update K_us based on current tire conditions


        if abs(vx) < 0.1:
            self.yaw_rate_prev = 0

            return 0
        else:
            yaw_target = (vx / (self.cfg.L + self.K_us * vx**2)) * delta
        max_yaw_rate = self.wheel_manager.get_max_yaw_rate(vx,ax,ay)


        yaw_capped = np.clip(yaw_target, -max_yaw_rate, max_yaw_rate)


        yaw_capped = np.clip(yaw_target, -max_yaw_rate, max_yaw_rate)

        yaw_target = self.yaw_rate_prev + (dt / self.tau) * (yaw_capped - self.yaw_rate_prev)
        self.yaw_rate_prev = yaw_target

        return yaw_target

    
import numpy as np

class TorqueAllocator:
    def __init__(self, cfg):
        self.cfg = cfg
        self.prev_throttle = 0.0
        self.prev_mz = 0.0 # Added to track previous Mz command
        
        # Time constants for powertrain lag
        self.tau_p = 0.05  # Throttle response lag (50ms)
        self.tau_mz = 0.04 # Mz motor response lag (40ms)
        
        self.drag_torque = 5.0 # Small Nm value for engine braking

    def distribute(self, throttle_request, mz_request, dt, vx):
        """
        throttle_request: 0.0 to 1.0
        mz_request: Corrective moment (Nm)
        dt: Integration time step for the filter
        """

        alpha_p = dt / (self.tau_p + dt)
        smooth_throttle = (1 - alpha_p) * self.prev_throttle + alpha_p * throttle_request
        self.prev_throttle = smooth_throttle
        

        alpha_mz = dt / (self.tau_mz + dt)
        smooth_mz = (1 - alpha_mz) * self.prev_mz + alpha_mz * mz_request
        self.prev_mz = smooth_mz


        if abs(smooth_throttle) < 1e-3:
            if abs(vx) > 1e-3:
                base_torque = -self.drag_torque 
            else:
                base_torque = 0.0
        else:
            base_torque = (smooth_throttle * self.cfg.max_wheel_torque) / 4.0

        delta_t = (smooth_mz * self.cfg.Re) / self.cfg.w
        dt_per_wheel = delta_t / 2.0
        
        torques = np.array([
            base_torque - dt_per_wheel, # FL (Brake left to turn left)
            base_torque + dt_per_wheel, # FR (Drive right to turn left)
            base_torque - dt_per_wheel, # RL
            base_torque + dt_per_wheel  # RR
        ])
        
        return np.clip(torques, -self.cfg.max_wheel_torque, self.cfg.max_wheel_torque)
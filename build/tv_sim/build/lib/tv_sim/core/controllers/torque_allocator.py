import numpy as np



class TorqueAllocator:
    def __init__(self, cfg):
        self.cfg = cfg
        self.prev_throttle = 0.0
        # Time constant for powertrain lag
        self.tau_p = 0.05 
        self.drag_torque = 5.0 # Small Nm value for engine braking stability

    def distribute(self, throttle_request, mz_request, dt,vx):
        """
        throttle_request: 0.0 to 1.0
        mz_request: Corrective moment
        dt: Integration time step for the filter
        """
        #Powertrain Lag 
        alpha = dt / (self.tau_p + dt)
        smooth_throttle = (1 - alpha) * self.prev_throttle + alpha * throttle_request
        self.prev_throttle = smooth_throttle

        
        # If throttle is 0, apply a tiny drag torque to damp the LuGre bristles
        if abs(smooth_throttle)  < 1e-3:
            if(abs(vx)>1e-3):
                base_torque = -self.drag_torque 
            else:
                base_torque=0
        else:
            base_torque = (smooth_throttle * self.cfg.max_wheel_torque) / 4
        
        #(Mz request)

        delta_t = (mz_request * self.cfg.Re) / self.cfg.w
        dt_per_wheel = delta_t / 2.0
        
        torques = np.array([
            base_torque - dt_per_wheel, # FL
            base_torque + dt_per_wheel, # FR
            base_torque - dt_per_wheel, # RL
            base_torque + dt_per_wheel  # RR
        ])
        
        return np.clip(torques, -self.cfg.max_wheel_torque, self.cfg.max_wheel_torque)
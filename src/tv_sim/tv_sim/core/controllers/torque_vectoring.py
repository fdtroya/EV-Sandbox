import numpy as np
from scipy.linalg import solve_continuous_are

class VehicleLQR:
    def __init__(self, config, wheel_manager, dt=0.05):
        self.cfg = config
        self.dt = dt
        self.wheel_manager = wheel_manager

        # LQR-I TUNING 
        self.q_vy = 400.0      
        self.q_r  = 1e6        
        self.q_ki = 5e4       # Penalty for the accumulated integral error
        self.r_input = 0.0001   
        
        # Integral state initialization
        self.integral_error = 0.0
        self.max_integral = 0.5 # Anti-windup limit
        
        self.max_Mz = self.cfg.m * 9.81 * 0.85 * (self.cfg.w / 2.0)

    def _update_dynamic_limits(self):
        """
        Calculates the physical limit of yaw moment based on 
        current vertical loads and friction estimates.
        """
        # At the limit, one side is at +mu*Fz and the other is at -mu*Fz
        self.max_Mz = self.wheel_manager.get_max_mz()

    def _get_augmented_matrices(self, vx):
        vx = max(vx, 1.0)
        
        fz_f = max(1.0, self.wheel_manager.wheel_observer_fl.fz + self.wheel_manager.wheel_observer_fr.fz)
        fz_r = max(1.0, self.wheel_manager.wheel_observer_rl.fz + self.wheel_manager.wheel_observer_rr.fz)
        
        Cf = self.cfg.lugre_normalized_stiffness * fz_f
        Cr = self.cfg.lugre_normalized_stiffness * fz_r

        m, Iz, a, b = self.cfg.m, self.cfg.Iz, self.cfg.a, self.cfg.b

        # Standard 2x2 Bicycle Model A
        a11 = -(Cf + Cr) / (m * vx)
        a12 = ((b * Cr - a * Cf) / (m * vx)) - vx
        a21 = (b * Cr - a * Cf) / (Iz * vx)
        a22 = -(a**2 * Cf + b**2 * Cr) / (Iz * vx)
        
        # In state-space form: [0, 1, 0] * [vy, r, error_int].T
        A_aug = np.array([
            [a11, a12, 0],
            [a21, a22, 0],
            [0,   1,   0] 
        ])

        # Augmented 3x1 B matrix
        B_aug = np.array([
            [0],
            [1 / Iz],
            [0]
        ])
        
        return A_aug, B_aug

    def solve(self, vx, ax, ay, vy, r, delta, target_r_func):
        target_r_raw = target_r_func(vx, ax, ay, delta, self.dt)
        target_r = float(np.asarray(target_r_raw).item() if np.ndim(target_r_raw) > 0 else target_r_raw)

        error_instant = r - target_r
        
        # Only add to the integral if the error is small 
        if abs(np.degrees(error_instant)) < 5.0:
            self.integral_error += error_instant * self.dt
        else:
            self.integral_error *= 0.95 
            
        self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)

        x_now = np.array([
            [float(vy)],
            [float(r)],
            [self.integral_error]
        ])
        
        x_ref = np.array([[0.0], [target_r], [0.0]])

        A, B = self._get_augmented_matrices(vx)
        Q = np.diag([self.q_vy, self.q_r, self.q_ki])
        R = np.array([[self.r_input]])

        try:
            P = solve_continuous_are(A, B, Q, R)
            K = np.linalg.inv(R) @ B.T @ P
        except:
            return 0.0, target_r

        # 4. Control Law
        u = -K @ (x_now - x_ref)
        self._update_dynamic_limits()
        Mz = float(np.clip(u[0, 0], -self.max_Mz, self.max_Mz))

        return Mz, target_r
import numpy as np
from scipy.linalg import solve_continuous_are

class VehicleLQR:
    def __init__(self, config, dt=0.05):
        self.cfg = config
        self.dt = dt


        fz_f_static = (self.cfg.m * self.cfg.g * self.cfg.b) / (2 * self.cfg.L)
        fz_r_static = (self.cfg.m * self.cfg.g * self.cfg.a) / (2 * self.cfg.L)
        
        # Tire Parameters (B, C, D)
        B, C, D, _ = self.cfg.tire_lat
        
        # Cornering Stiffness
        self.Cf = 2 * (B * C * D * fz_f_static)
        self.Cr = 2 * (B * C * D * fz_r_static)
        
        # tunning
        self.q_vy = 20.0    # Keep sideslip close to 0
        self.q_r  = 4000.0  # Hit the target yaw rate hard
        
        # R matrix: Penalty on Control Effort 
        self.r_input = 0.001

        # Max Torque Limit
        self.max_Mz = self.cfg.m * 9.81 * 0.85 * (self.cfg.w / 2.0)

    def _get_system_matrices(self, vx):
        """
        Constructs the A and B matrices for the Linear Bicycle Model
        State x = [vy, r]
        Input u = [Mz]
        """
        # Safety clamp for vx to avoid division by zero
        vx = max(vx, 1.0)
        
        m = self.cfg.m
        Iz = self.cfg.Iz
        a = self.cfg.a
        b = self.cfg.b
        Cf = self.Cf
        Cr = self.Cr

        #A
        a11 = -(Cf + Cr) / (m * vx)
        a12 = ((b * Cr - a * Cf) / (m * vx)) - vx
        
        #A
        a21 = (b * Cr - a * Cf) / (Iz * vx)
        a22 = -(a**2 * Cf + b**2 * Cr) / (Iz * vx)
        
        A = np.array([
            [a11, a12],
            [a21, a22]
        ])

        #B
        B = np.array([
            [0],      # No direct force on vy
            [1 / Iz]  # Direct moment on r
        ])
        
        return A, B

    def solve(self, vx, vy, r, delta, target_r_func):
        """
        Calculates the optimal Yaw Moment Mz
        """
        #ref
        target_r = target_r_func(vx, delta)
        
        #[Target vy, Target r]

        x_ref = np.array([
            [0],        
            [target_r]
        ])
        
        # Current State
        x_now = np.array([
            [vy],
            [r]
        ])

        A, B = self._get_system_matrices(vx)

        #Cost Matrices
        Q = np.array([
            [self.q_vy, 0],
            [0,         self.q_r]
        ])
        R = np.array([[self.r_input]])

        # Solve Algebraic Riccati Equation
        try:
            P = solve_continuous_are(A, B, Q, R)
        except Exception:
            # Fallback if solver fails (rare numerical issue)
            print("LQR Solver Warn: Using fallback gain")
            return 0.0

 
        K = np.linalg.inv(R) @ B.T @ P
        
        #Compute Control Input

        error = x_now - x_ref
        u_optimal = -K @ error
        
        Mz = u_optimal[0, 0]



        # Limits
        Mz = np.clip(Mz, -self.max_Mz, self.max_Mz)

        return Mz
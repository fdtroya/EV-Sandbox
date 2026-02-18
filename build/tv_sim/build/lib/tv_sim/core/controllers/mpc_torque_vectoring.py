import casadi as ca
import numpy as np

class VehicleMPC:
    def __init__(self, config, N=10, dt=0.05):
        self.opti = ca.Opti()
        self.N = N
        self.dt = dt
        self.cfg = config
        
        #---Variables ---
        self.U = self.opti.variable(1, self.N)
        self.X = self.opti.variable(2, self.N+1)
        
        #---Parameters ---
        self.p_x0 = self.opti.parameter(2)
        self.p_vx = self.opti.parameter()
        self.p_delta = self.opti.parameter()
        self.p_target = self.opti.parameter(self.N) 
        self.p_max_Mz = self.opti.parameter()
        
     
        fz_f = (self.cfg.m * self.cfg.g * self.cfg.b) / (2 * self.cfg.L) * 2 # *2 for axle
        fz_r = (self.cfg.m * self.cfg.g * self.cfg.a) / (2 * self.cfg.L) * 2
        
        B, C, D, _ = self.cfg.tire_lat
        self.mu_Fzf = D * fz_f # Max possible front force
        self.mu_Fzr = D * fz_r # Max possible rear force
        

        self.Cf = 2 * (B * C * D * fz_f / 2) 
        self.Cr = 2 * (B * C * D * fz_r / 2)
        
     
        cost = 0
        
    
        W_error = 100   
        W_effort = 1e-4

        for k in range(self.N):
            vy = self.X[0, k]
            r  = self.X[1, k]
            
            # Slip Angles
            alpha_f = self.p_delta - (vy + self.cfg.a * r) / (self.p_vx + 1.0) # Higher buffer for vx
            alpha_r = - (vy - self.cfg.b * r) / (self.p_vx + 1.0)
            
     
            Fyf = self.mu_Fzf * ca.tanh( self.Cf * alpha_f / self.mu_Fzf )
            Fyr = self.mu_Fzr * ca.tanh( self.Cr * alpha_r / self.mu_Fzr )
            
       
            next_vy = vy + ((Fyf + Fyr) / self.cfg.m - self.p_vx * r) * self.dt
            next_r  = r  + ((self.cfg.a * Fyf - self.cfg.b * Fyr + self.U[0, k]) / self.cfg.Iz) * self.dt
            
            self.opti.subject_to(self.X[0, k+1] == next_vy)
            self.opti.subject_to(self.X[1, k+1] == next_r)
            
            cost += W_error * (self.X[1, k+1] - self.p_target[k])**2
            cost += W_effort * self.U[0, k]**2

        self.opti.subject_to(self.X[:, 0] == self.p_x0)
        self.opti.subject_to(self.opti.bounded(-self.p_max_Mz, self.U, self.p_max_Mz))
        self.opti.minimize(cost)
        

        p_opts = {'expand': True}
        s_opts = {
            'ipopt.print_level': 0, 
            'ipopt.max_iter': 500,        # Give it more time
            'ipopt.warm_start_init_point': 'yes' # Enable Warm Start
        }
        self.opti.solver('ipopt', p_opts, s_opts)
        
        # Storage for Warm Start
        self.last_u_traj = np.zeros((1, self.N))
        self.last_x_traj = np.zeros((2, self.N+1))

    def solve(self, vx, vy, r, delta, target_r_func):
        # 1. Safety Check
        if np.any(np.isnan([vx, vy, r, delta])):
            return 0.0

    
        self.opti.set_value(self.p_x0, [vy, r])
        self.opti.set_value(self.p_vx, max(vx, 1.0))
        self.opti.set_value(self.p_delta, delta)
        
        targets = [target_r_func(max(vx, 1.0), delta) for _ in range(self.N)]
        self.opti.set_value(self.p_target, targets)
        
        limit = self.cfg.m * 9.81 * 0.85 * (self.cfg.w / 2.0)
        self.opti.set_value(self.p_max_Mz, limit)
   
        # Initialize the solver with the solution from the previous step
        self.opti.set_initial(self.U, self.last_u_traj)
        self.opti.set_initial(self.X, self.last_x_traj)

        try:
            sol = self.opti.solve()
            
            # Save solution
            self.last_u_traj = sol.value(self.U)
            self.last_x_traj = sol.value(self.X)
            
            return self.last_u_traj[0] # Return first control action
            
        except Exception as e:
            # On fail, use a simpler fallback or just hold previous
            print(f"MPC Fail: {e}")
            return 0.0
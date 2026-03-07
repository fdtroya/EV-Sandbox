class WheelObserver:
    def __init__(self, cfg):
        self.cfg = cfg
        self.last_execution_controller = None
        self.mu=cfg.mu_k
        self.last_vx = 0.0
        self.last_wheel_speed = 0.0
        self.fy=0
        self.fx=0
        self.fz=self.cfg.m * self.cfg.g / 4  # Assuming equal load distribution initially
    

    def update(self, t, vx):
        
        
        if t - self.last_execution_controller > 0.5:  # If more than 0.5s has passed since last update
            self.mu = self.cfg.mu_k  # Reset to safe coeficcient (could be updated to something more sophisticated)
        self.last_vx = vx
        self.last_execution_controller = t
        #do stuff (future) properly update fx and fy based on slip and current mu, for now just return max values based on current fz and mu
    def get_mu(self):
        return self.mu
    def get_max_fy(self):
        max_fy = (max(((self.mu*self.fz)**2 - self.fx**2),0))**0.5
        return max_fy
    def get_max_fx(self):
        max_fx = (max(((self.mu*self.fz)**2 - self.fy**2),0))**0.5
        return max_fx
    
        
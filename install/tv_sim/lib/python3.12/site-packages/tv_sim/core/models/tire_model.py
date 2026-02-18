import numpy as np
from ..utils.config import VehicleConfig
class IndependentTire:
    def __init__(self, key, cfg:VehicleConfig):
        self.key = key
        self.cfg = cfg
        
        # 2D Internal States
        self.zx = 0.0 
        self.zy = 0.0
        self.vrx=0
        self.vry=0

    
        # LuGre Parameters (Coupled)
        self.sigma0 = cfg.sigma0 # [Long, Lat] stiffness
        self.sigma1 = cfg.sigma1   # [Long, Lat] damping
        self.sigma2 = cfg.sigma2   # [Long, Lat] viscous
        self.mu_s = cfg.mu_s
        self.mu_k = cfg.mu_k
        self.vs = cfg.vs
        self.fz=cfg.m*cfg.g/4

    def get_dynamics(self, vrx, vry, fz,zx=None,zy=None):
        """
        Calculates 2D derivatives and forces.
        vrx, vry: Local slip velocities in tire frame.
        """

        if(zx==None):
            zx=self.zx #passed by rk4 estimate
        if(zy==None):
            zy=self.zy
        
        # Total slip velocity magnitude
        vr_mag = np.sqrt(vrx**2 + vry**2) + 1e-5
        g_vr = self.mu_k + (self.mu_s - self.mu_k) * np.exp(-(vr_mag/self.vs)**2)
        
        #2D state
     
        dzx = vrx - (self.sigma0[0] * vr_mag / g_vr) * zx
        dzy = vry - (self.sigma0[1] * vr_mag / g_vr) * zy
        
        # Force 
        fx = fz * (self.sigma0[0] * zx + self.sigma1[0] * dzx + self.sigma2[0] * vrx)
        fy = fz * (self.sigma0[1] * zy + self.sigma1[1] * dzy + self.sigma2[1] * vry)
        
        return fx, fy
    


    def stable_step(self, vrx, vry, dt):
        """
        Implicit update for LuGre bristle states.
        Ensures numerical stability at low speeds and high stiffness.
        """
       
        vr_mag = np.sqrt(vrx**2 + vry**2) + 1e-5
        g_vr = self.mu_k + (self.mu_s - self.mu_k) * np.exp(-(vr_mag/self.vs)**2)
        
        #(Backward Euler form)
        # denominator = 1 + (stiffness * velocity * dt) / friction_limit
        denom_x = 1.0 + (self.sigma0[0] * vr_mag / g_vr) * dt
        denom_y = 1.0 + (self.sigma0[1] * vr_mag / g_vr) * dt
        
        new_zx= (self.zx + vrx * dt) / denom_x
        new_zy = (self.zy + vry * dt) / denom_y

        dzx=(new_zx-self.zx)/dt # use dzx and dzy for forward euler estimate inside rk4
        dzy=(new_zy-self.zy)/dt

        self.zx=new_zx
        self.zy=new_zy
        
        return self.zx,self.zy,dzx,dzy
        
    def get_dynamics_rk2(self, vrx, vry, fz, zx, zy, dt):


        
        v_reg = 0.01   # tune this
        vr_mag = np.sqrt(vrx**2 + vry**2 + v_reg**2)

     
        g_vr = self.mu_k + (self.mu_s - self.mu_k) * np.exp(-(vr_mag/self.vs)**2)
        
        #(Backward Euler form)
        # denominator = 1 + (stiffness * velocity * dt) / friction_limit
        denom_x = 1.0 + (self.sigma0[0] * vr_mag / g_vr) * dt
        denom_y = 1.0 + (self.sigma0[1] * vr_mag / g_vr) * dt
        
        new_zx= (zx + vrx * dt) / denom_x
        new_zy = (zy + vry * dt) / denom_y



        if(dt>1e-9):

            dzx=(new_zx-zx)/dt # use dzx and dzy for forward euler estimate inside rk4
            dzy=(new_zy-zy)/dt

        else:
            dzx=0# use dzx and dzy for forward euler estimate inside rk4
            dzy=0

        v_critical = 0.005

        #'creep' at near-zero speeds.
        k_scale = 1.0 - np.exp(-vr_mag / v_critical)

        fx = fz * ( (self.sigma0[0] * new_zx * k_scale) + (self.sigma1[0] * dzx) + (self.sigma2[0] * vrx) )
        fy = fz * ( (self.sigma0[1] * new_zy * k_scale) + (self.sigma1[1] * dzy) + (self.sigma2[1] * vry) )
  
        return fx,fy,dzx,dzy
    
    def get_dynamics_rk(self, vrx, vry, fz, zx, zy, dt):


        
        v_reg = 0.01   # tune this
        vr_mag = np.sqrt(vrx**2 + vry**2 + v_reg**2)

     
        g_vr = self.mu_k + (self.mu_s - self.mu_k) * np.exp(-(vr_mag/self.vs)**2)
        
        #(Backward Euler form)
        # denominator = 1 + (stiffness * velocity * dt) / friction_limit
        denom_x = 1.0 + (self.sigma0[0] * vr_mag / g_vr) * dt
        denom_y = 1.0 + (self.sigma0[1] * vr_mag / g_vr) * dt
        
        new_zx= (zx + vrx * dt) / denom_x
        new_zy = (zy + vry * dt) / denom_y

        if(dt>1e-9):

            dzx=(new_zx-zx)/dt # use dzx and dzy for forward euler estimate inside rk4
            dzy=(new_zy-zy)/dt

        else:
            dzx=0# use dzx and dzy for forward euler estimate inside rk4
            dzy=0

        if vr_mag < 1e-3:
            dzx = 0
            dzy = 0

        
        fx = fz * (self.sigma0[0] * new_zx + self.sigma1[0] * dzx + self.sigma2[0] * vrx)
        fy = fz * (self.sigma0[1] * new_zy + self.sigma1[1] * dzy + self.sigma2[1] * vry)
        return fx,fy,dzx,dzy
    
import numpy as np
import matplotlib.pyplot as plt
from utils.config import VehicleConfig

from models.tire_model import CombinedTireModel
from models.ev_7dof import Vehicle7DOF
from controllers.drive_intent import ReferenceModel
from controllers.torque_vectoring import TVControllerPID
from controllers.torque_allocator import TorqueAllocator

def run_simulation(enable_tv=True):
    cfg = VehicleConfig()
    car = Vehicle7DOF(cfg)
    tire = CombinedTireModel(cfg.tire_long, cfg.tire_lat)
    ref = ReferenceModel(cfg)
    ctrl = TVControllerPID(cfg)
    alloc = TorqueAllocator(cfg)

    dt = 0.01
    time_steps = np.arange(0, 5.0, dt)
    history = []
    
    for t in time_steps:
        # 1. Driver Inputs
        steering = 0.1 if t > 1.0 else 0.0  # Turn 0.1 rad (~5.7 deg) at 1s
        throttle = 0.4                     # 40% constant throttle
        
        # 2. Get State
        vx = car.state[0]
        actual_yaw = car.state[2]
        
        # 3. Control Logic
        target_yaw = ref.get_target_yaw_rate(vx, steering)
        
        if enable_tv:
            mz_req = ctrl.calculate_mz(actual_yaw, target_yaw, dt)
        else:
            mz_req = 0.0
            
        # 4. Allocate and Step
        torques = alloc.distribute(throttle, mz_req)
        state = car.step(torques, steering, tire, dt)

      
        
        # Log data: [t, x, y, yaw_rate, target_yaw_rate, vx]
        history.append([t, state[3], state[4], state[2], target_yaw, state[0]])

        print(f"Target: {target_yaw:.2f} | Actual: {actual_yaw:.2f} | Mz Req: {mz_req:.2f} ")
     

    return np.array(history)

# --- Execute Both Runs ---
results_off = run_simulation(enable_tv=False)
results_on = run_simulation(enable_tv=True)

# --- Plotting ---
plt.figure(figsize=(12, 8))

# Subplot 1: Trajectory (X vs Y)
plt.subplot(2, 1, 1)
plt.plot(results_off[:, 1], results_off[:, 2], 'r--', label="TV OFF (Understeer)")
plt.plot(results_on[:, 1], results_on[:, 2], 'b-', label="TV ON (Corrected)")
plt.title("Vehicle Trajectory Comparison")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.legend()
plt.axis('equal')

# Subplot 2: Yaw Rate Tracking
plt.subplot(2, 1, 2)
plt.plot(results_on[:, 0], results_on[:, 4], 'k:', label="Target Yaw Rate")
plt.plot(results_off[:, 0], results_off[:, 3], 'r--', label="Actual Yaw (TV OFF)")
plt.plot(results_on[:, 0], results_on[:, 3], 'b-', label="Actual Yaw (TV ON)")
plt.title("Yaw Rate Tracking")
plt.xlabel("Time (s)")
plt.ylabel("Yaw Rate (rad/s)")
plt.legend()

plt.tight_layout()
plt.show()
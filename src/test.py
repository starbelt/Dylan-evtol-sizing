import math
import numpy as np
import matplotlib.pyplot as plt
import logging

# Configure logging (optional)
logging.basicConfig(level=logging.WARNING)

# --- Paste the refined calculate_climb_power_kw function here ---
def calculate_climb_power_kw(mtow_kg, target_speed_mps, rate_of_climb_mps, rho, wing_ref_area_m2, wing_AR, base_cd0, oswald_efficiency, prop_effic, epu_effic, trim_drag_factor, g):

    # --- Calculate Climb Angle ---
    # Ensure speed is high enough to calculate angle; avoid division by zero if speed is exactly min_valid_speed
    effective_speed =target_speed_mps

    sin_gamma = rate_of_climb_mps / effective_speed
    # Clamp argument to avoid domain errors due to potential floating point inaccuracies
    cos_gamma_arg = max(0.0, 1.0 - sin_gamma**2)
    cos_gamma = math.sqrt(cos_gamma_arg)


    # --- Calculate Aerodynamic Forces ---
    weight_n = mtow_kg * g
    lift_needed_n = weight_n * cos_gamma

    q = 0.5 * rho * target_speed_mps**2

    cl_req = lift_needed_n / (q * wing_ref_area_m2)

    cdi_den = math.pi * oswald_efficiency * wing_AR

    cdi = cl_req**2 / cdi_den

    cd_total = (base_cd0 + cdi) * trim_drag_factor
    drag_n = q * wing_ref_area_m2 * cd_total

    # --- Calculate Power Components (in Watts) ---
    power_drag_w = drag_n * target_speed_mps
    power_climb_w = weight_n * rate_of_climb_mps
    total_thrust_power_req_w = power_drag_w + power_climb_w


    total_shaft_power_req_w = total_thrust_power_req_w / prop_effic
    electric_power_req_w = total_shaft_power_req_w / epu_effic

    # --- Convert to kW and Return ---
    electric_power_req_kw = electric_power_req_w / 1000.0
    return max(0.0, electric_power_req_kw)



# --- Simulation Parameters ---
mtow = 1500         # kg
roc = 5             # m/s (Constant rate of climb)
rho_air = 1.1       # kg/m^3 (Example density)
wing_area = 15      # m^2
aspect_ratio = 10
cd0 = 0.025
eff_oswald = 0.8
eff_prop = 0.85
eff_epu = 0.92
trim_drag = 1.05
g = 9.81

# --- Acceleration Phase Parameters ---
start_speed_mps = 40.0   # Start simulation when wing-borne model is somewhat valid (m/s)
# Ensure start_speed is >= min_valid_speed used inside calculate_climb_power_kw
min_model_speed = 1.0 # Should match the value inside the function
start_speed_mps = start_speed_mps

cruise_speed_mps = 60 # Target cruise speed (m/s)
acceleration_mps2 = 1.0 # Constant acceleration along flight path (m/s^2) - adjust as needed

# --- Simulation Setup ---
if cruise_speed_mps <= start_speed_mps:
    print("Error: Cruise speed must be greater than start speed.")
    exit()

if acceleration_mps2 <= 0:
    print("Error: Acceleration must be positive.")
    exit()

total_time_s = (cruise_speed_mps - start_speed_mps) / acceleration_mps2
num_steps = 100 # Number of points to calculate for the graph
time_points = np.linspace(0, total_time_s, num_steps)

# Lists to store data for plotting
time_data = []
speed_data = []
power_data = []

print(f"Simulating acceleration from {start_speed_mps:.1f} m/s to {cruise_speed_mps:.1f} m/s")
print(f"Constant Rate of Climb: {roc} m/s")
print(f"Constant Acceleration: {acceleration_mps2} m/s^2")
print(f"Total simulation time: {total_time_s:.2f} s")
print("-" * 30)
print("Time (s) | Speed (m/s) | Power (kW)")
print("-" * 30)

# --- Run Simulation ---
for t in time_points:
    # Calculate instantaneous speed along flight path
    current_speed_mps = start_speed_mps + acceleration_mps2 * t
    # Ensure we don't exceed cruise speed due to floating point steps
    current_speed_mps =cruise_speed_mps

    # Calculate power required at this instantaneous speed and ROC
    power_kw = calculate_climb_power_kw(
        mtow_kg=mtow,
        target_speed_mps=current_speed_mps,
        rate_of_climb_mps=roc,
        rho=rho_air,
        wing_ref_area_m2=wing_area,
        wing_AR=aspect_ratio,
        base_cd0=cd0,
        oswald_efficiency=eff_oswald,
        prop_effic=eff_prop,
        epu_effic=eff_epu,
        trim_drag_factor=trim_drag,
        g = g
    )

    # Store data if calculation was valid
    if power_kw != float('inf'):
        time_data.append(t)
        speed_data.append(current_speed_mps)
        power_data.append(power_kw)
        if t == 0 or t == total_time_s or (len(time_data) % (num_steps // 10) == 0) : # Print some intermediate values
             print(f"{t:<8.2f} | {current_speed_mps:<11.2f} | {power_kw:<10.2f}")

print("-" * 30)

# --- Plotting ---
if not time_data:
    print("No valid data points generated for plotting. Check parameters and minimum speed.")
else:
    plt.figure(figsize=(10, 6))
    plt.plot(time_data, power_data, label='Required Electric Power')

    plt.xlabel("Time (s)")
    plt.ylabel("Required Electric Power (kW)")
    plt.title(f"eVTOL Power During Accelerating Climb\n(ROC={roc} m/s, Accel={acceleration_mps2} m/s², MTOW={mtow} kg)")
    plt.legend()
    plt.grid(True)
    plt.ylim(bottom=0) # Ensure y-axis starts at 0
    plt.xlim(left=0)

    # Optional: Add a secondary axis for speed
    ax2 = plt.twinx()
    ax2.plot(time_data, speed_data, 'r--', label='Speed', alpha=0.6)
    ax2.set_ylabel('Speed (m/s)', color='r')
    ax2.tick_params(axis='y', labelcolor='r')
    ax2.legend(loc='upper center')

    plt.tight_layout() # Adjust layout to prevent labels overlapping
    plt.show()
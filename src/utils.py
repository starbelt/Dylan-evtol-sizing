import math

# Constants
G = 9.81  
H_PER_S = 1.0 / 3600.0  # Hours per second conversion
W_PER_KW = 1000.0  # Watts per kilowatt

# Used in aircraft.py and missionsegment.py
def calc_hover_shaft_power_k_W(mtow_kg, g, rho, fom, disk_area_m2):
    thrust_n = mtow_kg * g
    ideal_power_w = (thrust_n**1.5) / (math.sqrt(2.0 * rho * disk_area_m2))
    shaft_power_w = ideal_power_w / fom
    return shaft_power_w / W_PER_KW

# Used in battery.py and missionsegment.py
# Calculates the C-rate based on the EOL energy 
def calc_batt_C_rate(batt_rated_energy_k_W_h, electric_power_k_W):
    return electric_power_k_W / batt_rated_energy_k_W_h

# Used in aircraft.py and missionsegment.py
# Calculates the disk area for a single rotor
def calc_disk_area_m2(rotor_diameter_m):
    return math.pi * (rotor_diameter_m / 2)**2


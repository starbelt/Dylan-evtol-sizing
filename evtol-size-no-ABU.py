#!/usr/bin/env python3
# evtol_integrated_sizing_and_analysis.py
#
# Combines eVTOL sizing loop with power profile analysis and battery cell modeling.
# Includes an Average Power Climb segment (multi-point avg), NO Descent segment.
# Power plot shows start/end power during climb.
# ADDED: Optional battery jettison analysis.
#
# Usage: python3 evtol_integrated_sizing_and_analysis.py path/to/params.json

# import Python modules
import json
import math
import sys
from tabulate import tabulate
import matplotlib.pyplot as plt
import numpy as np

# --- Constants ---
H_PER_S = 1.0/3600.0
W_PER_KW = 1000.0
FT_PER_M = 3.28084
M_PER_FT = 1.0 / FT_PER_M
FT_TO_MILE = 0.000189394
M_PER_MILE = 1609.34

# --- Parameter Loading ---
if len(sys.argv) == 2:
    json_i = sys.argv[1]
else:
    print(f'Usage: python3 {sys.argv[0]} /path/to/input.json')
    sys.exit(1)

try:
    with open(json_i, "r") as f:
        params = json.load(f)
    locals().update(params)
    print(f"Loaded parameters from: {json_i}")
except FileNotFoundError:
    print(f"Error: Input JSON file not found at {json_i}")
    sys.exit(1)
except json.JSONDecodeError:
    print(f"Error: Could not decode JSON from file {json_i}")
    sys.exit(1)

# --- Parameter Defaults & Checks ---
# ... (Existing defaults and checks remain the same) ...
if 'target_cruise_range_EOL_km' not in locals():
    print("WARNING: 'target_cruise_range_EOL_km' not found in JSON. Using default: 100.0 km for sizing.")
    target_cruise_range_EOL_km = 100.0
if 'fixed_battery_pack_mass_kg' not in locals():
    fixed_battery_pack_mass_kg = 0

# --- NEW: Jettison Parameters ---
enable_jettison_analysis = params.get('enable_jettison_analysis', False)
jettison_time_s = params.get('jettison_time_s', 0.0)
jettison_mass_kg = params.get('jettison_mass_kg', 0.0)
# --- End NEW ---

# --- NEW: Shorter Range Analysis Parameters ---
enable_shorter_range_analysis = params.get('enable_shorter_range_analysis', False)
default_shorter_range = params.get('target_cruise_range_EOL_km', 100.0) * 0.5
shorter_range_target_km = params.get('shorter_range_target_km', default_shorter_range)
if enable_shorter_range_analysis and shorter_range_target_km <= 0:
    print(f"WARNING: 'shorter_range_target_km' must be positive for shorter range analysis. Disabling.")
    enable_shorter_range_analysis = False
# --- End NEW ---

# --- NEW: Jettison Optimization Parameters ---
jettison_opt_enable = params.get('jettison_opt_enable', False)
jettison_opt_mass_max_kg = params.get('jettison_opt_mass_max_kg', 95.0)
jettison_opt_mass_step_kg = params.get('jettison_opt_mass_step_kg', 5.0)
jettison_opt_time_step_s = params.get('jettison_opt_time_step_s', 10.0)
jettison_opt_max_dod_dropped = params.get('jettison_opt_max_dod_dropped', 100.0)
jettison_opt_max_dod_remaining = params.get('jettison_opt_max_dod_remaining', 100.0)
if jettison_opt_enable and (jettison_opt_mass_step_kg <= 0 or jettison_opt_time_step_s <= 0):
    print("WARNING: Jettison optimization mass/time steps must be positive. Disabling optimization.")
    jettison_opt_enable = False

# Altitudes (convert to meters)
plot_hover_alt_ft = params.get('plot_hover_alt_ft', 50.0)
plot_cruise_alt_ft = params.get('plot_cruise_alt_ft', 4000.0)
plot_hover_alt_m = plot_hover_alt_ft * M_PER_FT
plot_cruise_alt_m = plot_cruise_alt_ft * M_PER_FT
altitude_diff_m = max(0, plot_cruise_alt_m - plot_hover_alt_m)

# Rate of Climb (REQUIRED - use 900 fpm default if missing)
default_roc_mps = 900.0 / 60.0 * M_PER_FT # 4.572 m/s
rate_of_climb_mps = params.get('rate_of_climb_mps', default_roc_mps)
if rate_of_climb_mps <= 0:
    print(f"WARNING: Rate of climb must be positive. Using default ({default_roc_mps:.2f} m/s).")
    rate_of_climb_mps = default_roc_mps

# Define speeds for climb averaging
V_climb_start_mps = params.get('V_climb_start_mps', 20.0)
V_climb_end_mps = cruise_speed_m_p_s
V_climb_mid_mps = (V_climb_start_mps + V_climb_end_mps) / 2.0


# --- Helper Functions ---
# ... (All existing helper functions remain the same) ...
def calc_payload_mass_frac(payload_kg,max_takeoff_wt_kg):
  return payload_kg/max_takeoff_wt_kg if max_takeoff_wt_kg > 0 else 0

def calc_pack_usable_EOL_spec_energy(cell_voltage, cell_charge_ah, cell_mass_kg, cell_to_pack_mass_ratio, batt_soh, batt_inacc_energy_frac):
    if cell_mass_kg <= 0 or cell_to_pack_mass_ratio <= 0: return 0
    cell_energy_Wh = cell_voltage * cell_charge_ah
    cell_spec_energy_Wh_kg = cell_energy_Wh / cell_mass_kg
    pack_spec_energy_gross_BOL_Wh_kg = cell_spec_energy_Wh_kg * cell_to_pack_mass_ratio
    pack_spec_energy_usable_EOL_Wh_kg = pack_spec_energy_gross_BOL_Wh_kg * batt_soh * (1.0 - batt_inacc_energy_frac)
    return pack_spec_energy_usable_EOL_Wh_kg

def calc_pack_gross_BOL_spec_energy(cell_voltage, cell_charge_ah, cell_mass_kg, cell_to_pack_mass_ratio):
    if cell_mass_kg <= 0 or cell_to_pack_mass_ratio <= 0: return 0
    cell_energy_Wh = cell_voltage * cell_charge_ah
    cell_spec_energy_Wh_kg = cell_energy_Wh / cell_mass_kg
    pack_spec_energy_gross_BOL_Wh_kg = cell_spec_energy_Wh_kg * cell_to_pack_mass_ratio
    return pack_spec_energy_gross_BOL_Wh_kg

def calc_structural_weight_kg(wing_weight_kg,horizontal_tail_weight_kg,vertical_wing_weight_kg,fuselage_weight_kg,boom_weight_kg,landing_gear_weight_kg):
    structural_weight_kg = wing_weight_kg + horizontal_tail_weight_kg+vertical_wing_weight_kg+fuselage_weight_kg+boom_weight_kg+landing_gear_weight_kg
    return structural_weight_kg

def calc_batt_mass_frac(payload_mass_frac, empty_weight_kg, max_takeoff_wt_kg):
  if max_takeoff_wt_kg <= 0: return 0
  empty_frac = empty_weight_kg / max_takeoff_wt_kg
  batt_mass_frac = 1.0 - payload_mass_frac - empty_frac
  return max(0, batt_mass_frac)

def calc_batt_mass_kg(batt_mass_frac, max_takeoff_wt_kg):
    return max_takeoff_wt_kg * batt_mass_frac

def calc_batt_rated_energy_k_W_h(pack_spec_energy_gross_BOL_Wh_kg, batt_mass_kg):
    return pack_spec_energy_gross_BOL_Wh_kg * batt_mass_kg / 1000.0

def calc_batt_useable_energy_EOL_W_h(pack_spec_energy_usable_EOL_Wh_kg, batt_mass_kg):
    # Ensure result is float for division later if needed
    return float(pack_spec_energy_usable_EOL_Wh_kg * batt_mass_kg)

def calc_disk_area_m2(rotor_diameter_m,rotor_count):
  return rotor_count*(((rotor_diameter_m/2)**2)*math.pi)

def calc_disk_loading_kg_p_m2(max_takeoff_wt_kg,disk_area_m2):
  return max_takeoff_wt_kg/disk_area_m2 if disk_area_m2 > 0 else 0

def calc_disk_loading_english(disk_loading_kg_p_m2):
  return disk_loading_kg_p_m2*0.204816

def calc_hover_shaft_power_k_W(max_takeoff_wt_kg,g_m_p_s2,air_density_sea_level_kg_p_m3,hover_fom,disk_area_m2):
    if air_density_sea_level_kg_p_m3 <= 0 or disk_area_m2 <= 0 or hover_fom <= 0 or max_takeoff_wt_kg < 0: return 0
    thrust_n = max_takeoff_wt_kg * g_m_p_s2
    # Handle potential negative thrust due to negative weight input during intermediate steps
    if thrust_n < 0: return 0
    # Avoid domain error for sqrt if thrust is tiny and negative due to floating point
    if 2*air_density_sea_level_kg_p_m3*disk_area_m2 <= 0: return 0
    # Check thrust_n again before pow(1.5)
    if thrust_n <= 0: return 0
    power_num = thrust_n**1.5
    power_den = math.sqrt(2*air_density_sea_level_kg_p_m3*disk_area_m2) * hover_fom
    if power_den == 0: return float('inf') # Avoid division by zero

    return power_num / power_den / 1000.0


def calc_hover_electric_power_k_W(epu_effic,hover_shaft_power_k_W):
  return hover_shaft_power_k_W/epu_effic if epu_effic > 0 else 0

def calc_cruise_shaft_power_k_W(max_takeoff_wt_kg,g_m_p_s2,prop_effic,cruise_speed_m_p_s,cruise_L_p_D):
    if cruise_L_p_D <= 0 or prop_effic <= 0: return 0
    # Ensure weight is non-negative before calculation
    if max_takeoff_wt_kg < 0: return 0
    return max_takeoff_wt_kg*g_m_p_s2/cruise_L_p_D/prop_effic*cruise_speed_m_p_s/1000.0

def calc_cruise_electric_power_k_W(cruise_shaft_power_k_w,epu_effic):
  return cruise_shaft_power_k_w/epu_effic if epu_effic > 0 else 0

def calc_power_at_speed_climbing_k_W(
    mtow_kg, target_speed_mps, base_cd0, wing_AR, wing_ref_area_m2, rho,
    rate_of_climb_mps, prop_effic, epu_effic, g, spac_effic_factor, trim_drag_factor
    ):
    """Calculates electric power needed to climb at a specific speed."""
    if target_speed_mps <= 0: target_speed_mps = 1.0 # Avoid zero speed issues
    if wing_ref_area_m2 <= 0 or rho <= 0 or wing_AR <= 0 or prop_effic <= 0 or epu_effic <= 0 or mtow_kg < 0:
        return 0 # Cannot calculate

    lift_needed = mtow_kg * g
    q = 0.5 * rho * target_speed_mps**2
    if q * wing_ref_area_m2 <= 0 : return float('inf') # Avoid division by zero for CL
    cl_req = lift_needed / (q * wing_ref_area_m2)

    # Check denominator for cdi
    cdi_den = math.pi * spac_effic_factor * wing_AR
    cdi = cl_req**2 / cdi_den if cdi_den > 0 else float('inf') # Handle zero denominator case

    cd_total_untimed = base_cd0 + cdi
    if cd_total_untimed <= 0: # Avoid negative or zero drag before trim factor
        l_over_d = float('inf')
    else:
        cd_total = cd_total_untimed * trim_drag_factor
        l_over_d = cl_req / cd_total if cd_total > 0 else float('inf') # High L/D if drag is near zero

    if l_over_d <= 0:
        power_drag_kw = 0
        # If L/D is zero or negative, it implies extremely high drag or error, assume high power
        # However, physically, drag force should be positive if speed > 0
        # If lift_needed is positive, drag should be positive unless L/D is inf.
        # Let's calculate drag directly to avoid L/D issues when near zero.
        drag_n = q * wing_ref_area_m2 * cd_total
        power_drag_kw = drag_n * target_speed_mps / 1000.0

    else:
        drag_n = lift_needed / l_over_d
        power_drag_kw = drag_n * target_speed_mps / 1000.0

    power_vertical_kw = (mtow_kg * g * rate_of_climb_mps) / 1000.0
    # Ensure individual power components are non-negative
    power_drag_kw = max(0, power_drag_kw)
    power_vertical_kw = max(0, power_vertical_kw)

    # Check propeller efficiency
    if prop_effic <= 0: return float('inf')
    total_shaft_power_req_kw = (power_drag_kw + power_vertical_kw) / prop_effic

    # Check EPU efficiency
    if epu_effic <= 0: return float('inf')
    electric_power_req_kw = total_shaft_power_req_kw / epu_effic

    return electric_power_req_kw


def calc_climb_time_s(altitude_diff_m, rate_of_climb_mps):
    if rate_of_climb_mps <= 0: return 0
    return altitude_diff_m / rate_of_climb_mps

def calc_climb_energy_k_W_h(climb_electric_power_avg_k_W, climb_time_s):
    return climb_electric_power_avg_k_W * climb_time_s * H_PER_S

def calc_hover_energy_k_W_h(hover_electric_power_k_W,equiv_hover_dur_s):
    return hover_electric_power_k_W*equiv_hover_dur_s/3600.0

def calc_reserve_hover_energy_k_W_h(hover_electric_power_k_W,reserve_hover_dur_s):
  # Use the power corresponding to the *initial* MTOW for reserve calculations, as per standard practice.
  return reserve_hover_dur_s*hover_electric_power_k_W/3600.0

def calc_reserve_cruise_energy_k_W_h(cruise_speed_m_p_s,reserve_range_km,cruise_electric_power_k_W):
    # Use the power corresponding to the *initial* MTOW for reserve calculations.
    if cruise_speed_m_p_s <= 0: return 0
    return reserve_range_km*1000.0/cruise_speed_m_p_s*cruise_electric_power_k_W/3600.0

def calc_available_cruise_energy_EOL_k_W_h(
    batt_useable_energy_EOL_W_h, hover_energy_k_W_h, climb_energy_k_W_h,
    reserve_hover_energy_k_W_h, reserve_cruise_energy_k_W_h
    ):
  # Note: batt_useable_energy_EOL_W_h comes from the *total* battery mass.
  available_for_cruise = float(batt_useable_energy_EOL_W_h) - hover_energy_k_W_h - climb_energy_k_W_h \
                         - reserve_hover_energy_k_W_h - reserve_cruise_energy_k_W_h
  return max(0.0, available_for_cruise)


def calc_cruise_time_EOL_s(cruise_electric_power_k_W,available_cruise_energy_EOL_k_W_h):
    if cruise_electric_power_k_W <= 0: return 0
    return available_cruise_energy_EOL_k_W_h / cruise_electric_power_k_W * 3600.0

def calc_cruise_range_EOL_km(cruise_time_EOL_s,cruise_speed_m_p_s):
  return cruise_time_EOL_s*cruise_speed_m_p_s/1000.0

def calc_cruise_range_EOL_english(cruise_range_EOL_km):
  return cruise_range_EOL_km / (M_PER_MILE / 1000.0)

def calc_rotor_RPM_hover(sound_speed_m_p_s,rotor_diameter_m,tip_mach):
    if rotor_diameter_m <= 0: return 0
    return sound_speed_m_p_s*tip_mach/(rotor_diameter_m/2.0)*30.0/math.pi

def calc_motor_torque_hover_Nm(rotor_count,rotor_RPM_hover,hover_shaft_power_k_W):
    if rotor_count <= 0 or rotor_RPM_hover <= 0: return 0
    # Ensure power is non-negative
    if hover_shaft_power_k_W < 0: return 0
    return hover_shaft_power_k_W / rotor_count / (rotor_RPM_hover*math.pi/30.0) * 1000.0

def calc_motor_mechanical_power_hover_k_W(rotor_count,hover_shaft_power_k_W):
  return hover_shaft_power_k_W/rotor_count if rotor_count > 0 else 0

def calc_over_torque_factor(rotor_count):
    if rotor_count <= 2: return 1.3
    return rotor_count/(rotor_count-2.0)+0.3

def calc_motor_RPM_max_thrust_min_rho(rotor_RPM_hover,over_torque_factor,air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3):
    if air_density_min_kg_p_m3 <= 0: return rotor_RPM_hover
    density_ratio = air_density_sea_level_kg_p_m3 / air_density_min_kg_p_m3
    # Ensure arguments to sqrt are non-negative
    over_torque_factor = max(0, over_torque_factor)
    density_ratio = max(0, density_ratio)
    return rotor_RPM_hover * math.sqrt(over_torque_factor) * math.sqrt(density_ratio)

def calc_motor_torque_max_thrust_Nm(over_torque_factor,motor_torque_hover_Nm):
  return over_torque_factor*motor_torque_hover_Nm

def calc_motor_mechanical_power_sizing_k_W(motor_RPM_max_thrust_min_rho, motor_torque_max_thrust_Nm):
  return motor_torque_max_thrust_Nm*motor_RPM_max_thrust_min_rho*math.pi/30.0/1000.0

def calc_cruise_max_shaft_power_k_W(cruise_shaft_power_k_w, air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3,
                        max_takeoff_wt_kg, g_m_p_s2, cruise_speed_m_p_s, rotor_count):
    if air_density_min_kg_p_m3 <= 0 or rotor_count < 4 or max_takeoff_wt_kg < 0: return cruise_shaft_power_k_w
    density_ratio_sqrt = math.sqrt(max(0, air_density_sea_level_kg_p_m3 / air_density_min_kg_p_m3))
    drag_term = max_takeoff_wt_kg * g_m_p_s2 * cruise_speed_m_p_s * density_ratio_sqrt * 0.015 / 1000.0
    climb_power_term1 = max_takeoff_wt_kg * g_m_p_s2 * 2.54 / 1000.0
    climb_power_term2 = max_takeoff_wt_kg * g_m_p_s2 * 5.08 / 1000.0
    rotors_per_prop_system = rotor_count / 2.0
    if rotors_per_prop_system <= 1: # Avoid division by zero or negative for term1 denominator
         return max(cruise_shaft_power_k_w * density_ratio_sqrt + drag_term,
                   (cruise_shaft_power_k_w * density_ratio_sqrt + climb_power_term1) / rotors_per_prop_system if rotors_per_prop_system > 0 else float('inf'),
                   (cruise_shaft_power_k_w + climb_power_term2) / rotors_per_prop_system if rotors_per_prop_system > 0 else float('inf'))

    term1 = (cruise_shaft_power_k_w * density_ratio_sqrt + drag_term) / (rotors_per_prop_system - 1.0) * 1.1
    term2 = (cruise_shaft_power_k_w * density_ratio_sqrt + climb_power_term1) / rotors_per_prop_system
    term3 = (cruise_shaft_power_k_w + climb_power_term2) / rotors_per_prop_system
    return max(term1, term2, term3)


def calc_single_EPU_weight_kg(motor_torque_max_thrust_Nm, motor_mechanical_power_sizing_k_W):
    # Ensure power and torque aren't negative for weight calculation
    power_term = max(0, motor_mechanical_power_sizing_k_W)
    torque_term = max(0, motor_torque_max_thrust_Nm)
    return 1.15 * (power_term / 12.67 + torque_term / 52.2 + 2.55)

def calc_total_EPU_weight_kg(single_EPU_weight_kg,rotor_count):
  return single_EPU_weight_kg*rotor_count

def calc_motor_mass_fraction(total_EPU_weight_KG,max_takeoff_wt_kg):
  return total_EPU_weight_KG/max_takeoff_wt_kg if max_takeoff_wt_kg > 0 else 0

def calc_batt_C_rate_hover(batt_rated_energy_k_W_h,hover_electric_power_k_W):
    if batt_rated_energy_k_W_h <= 0: return 0
    return hover_electric_power_k_W/batt_rated_energy_k_W_h

def calc_batt_C_rate_cruise(batt_rated_energy_k_W_h,cruise_electric_power_k_W):
    if batt_rated_energy_k_W_h <= 0: return 0
    return cruise_electric_power_k_W/batt_rated_energy_k_W_h

def calc_batt_C_rate_climb(batt_rated_energy_k_W_h, climb_electric_power_avg_k_W):
    if batt_rated_energy_k_W_h <= 0: return 0
    return climb_electric_power_avg_k_W / batt_rated_energy_k_W_h

def calc_Ct_hover(max_takeoff_wt_kg, g_m_p_s2, rotor_count, air_density_sea_level_kg_p_m3, rotor_diameter_m, rotor_RPM_hover):
    if rotor_count <= 0 or air_density_sea_level_kg_p_m3 <= 0 or rotor_diameter_m <= 0 or rotor_RPM_hover <= 0 or max_takeoff_wt_kg < 0: return 0
    thrust_per_rotor = max_takeoff_wt_kg * g_m_p_s2 / rotor_count
    tip_speed_rad_s = rotor_RPM_hover * math.pi / 30.0
    if tip_speed_rad_s == 0: return 0 # Avoid division by zero
    rotor_radius_m = rotor_diameter_m / 2.0
    if rotor_radius_m == 0: return 0 # Avoid zero radius if diameter is zero
    rotor_area_m2 = math.pi * rotor_radius_m**2
    denominator = air_density_sea_level_kg_p_m3 * rotor_area_m2 * (tip_speed_rad_s * rotor_radius_m)**2 # Incorrect formula in original? Should be rho * A * (Omega*R)^2? Let's use rho * pi * R^4 * Omega^2
    denominator_corrected = air_density_sea_level_kg_p_m3 * math.pi * (rotor_diameter_m / 2.0)**4 * (tip_speed_rad_s/(rotor_diameter_m / 2.0))**2 # This seems wrong too. Let's stick to the provided formula structure for now.
    # Sticking to provided structure: denominator = air_density_sea_level_kg_p_m3 * math.pi * (rotor_diameter_m / 2.0)**4 * tip_speed_rad_s**2 -- No, that's definitely wrong units.
    # Thrust Coefficient T = Ct * rho * A * (Omega*R)^2 --> Ct = T / (rho * A * (Omega*R)^2)
    tip_speed_mps = tip_speed_rad_s * rotor_radius_m
    denominator_standard = air_density_sea_level_kg_p_m3 * rotor_area_m2 * tip_speed_mps**2
    if denominator_standard == 0: return 0
    return thrust_per_rotor / denominator_standard


def calc_rotor_solidity(Ct_hover,rotor_avg_cl):
    if rotor_avg_cl <= 0: return 0
    # Solidity sigma = 6 * Ct / Cl_avg -- This is a common approximation for hover
    return Ct_hover * 6.0 / rotor_avg_cl

def calc_Cq_hover(hover_fom,Ct_hover):
    if hover_fom <= 0: return 0
    # Check Ct_hover non-negative before pow(1.5)
    if Ct_hover < 0: return 0
    numerator = Ct_hover**1.5
    denominator = math.sqrt(2.0) * hover_fom
    if denominator == 0: return float('inf') # Avoid division by zero
    return numerator / denominator

def calc_wing_ref_area_m2(max_takeoff_wt_kg, g_m_p_s2, air_density_sea_level_kg_p_m3, stall_speed_m_p_s,vehicle_cl_max):
    if max_takeoff_wt_kg < 0: return 0 # Handle negative weight
    lift = max_takeoff_wt_kg * g_m_p_s2
    if stall_speed_m_p_s <= 0 or vehicle_cl_max <= 0 or air_density_sea_level_kg_p_m3 <=0 : return 0 # Avoid division issues
    denominator = 0.5 * air_density_sea_level_kg_p_m3 * stall_speed_m_p_s**2 * vehicle_cl_max
    if denominator <= 0: return 0
    return lift / denominator

def calc_wingspan_m(d_value_m): return d_value_m

def calc_rotor_span(rotor_count, rotor_diameter_m, fuselage_w_m):
    rotors_per_side = rotor_count / 2.0
    if rotors_per_side < 1: return fuselage_w_m + rotor_diameter_m
    gap = rotor_diameter_m * 0.1 # Assuming 10% gap
    # Ensure non-negative dimensions
    rotor_diameter_m = max(0, rotor_diameter_m)
    fuselage_w_m = max(0, fuselage_w_m)
    gap = max(0, gap)
    return rotors_per_side * rotor_diameter_m + max(0, rotors_per_side - 1.0) * gap + fuselage_w_m

def calc_wing_AR(wingspan_m,wing_ref_area_m2):
  return wingspan_m**2 / wing_ref_area_m2 if wing_ref_area_m2 > 0 else 0

def calc_wing_loading_kg_p_m2(max_takeoff_wt_kg,wing_ref_area_m2):
  return max_takeoff_wt_kg/wing_ref_area_m2 if wing_ref_area_m2 > 0 else 0

def calc_wing_loading_english(wing_loading_kg_p_m2): return wing_loading_kg_p_m2*0.204816

def calc_emergency_landing_load_factor(stall_speed_m_p_s):
  stall_speed_kts = stall_speed_m_p_s / 0.514444 # Conversion factor from m/s to knots
  if stall_speed_kts <= 0: return 3.0 # Default load factor if stall speed is zero
  # Simplified relation, ensure base speed (61 kts) is not zero
  base_speed_kts = 61.0
  if base_speed_kts <= 0: return 3.0 # Avoid division by zero
  load_factor = (stall_speed_kts / base_speed_kts)**2 * 3.0
  return max(3.0, load_factor) # Ensure minimum load factor (e.g., 3.0)

def calc_fuselage_fineness_ratio(fuselage_w_m,fuselage_l_m,fuselage_h_m):
    avg_diam = (fuselage_w_m+fuselage_h_m)/2.0
    if avg_diam <= 0: return 0
    return fuselage_l_m / avg_diam

def calc_fuselage_reynolds(kinematic_visc_sea_level_m2_p_s,cruise_speed_m_p_s,fuselage_l_m):
    if kinematic_visc_sea_level_m2_p_s <= 0: return 0
    return fuselage_l_m*cruise_speed_m_p_s/kinematic_visc_sea_level_m2_p_s

def calc_fuselage_Cd0_p_Cf(fuselage_fineness_ratio):
    if fuselage_fineness_ratio <= 0: return 1.0 # Default or minimum value
    f = fuselage_fineness_ratio
    # Ensure terms inside power are non-negative if needed, though f>0 here
    term15 = 1.5 / (f**1.5) if f > 0 else 0
    term7 = 7.0 / (f**3) if f > 0 else 0
    return 1.0 + term15 + term7

def calc_fuselage_Cf(fuselage_reynolds,fuselage_fineness_ratio):
    if fuselage_reynolds <= 0: return 0.005 # Typical turbulent flat plate friction coeff estimate
    log_re = math.log10(fuselage_reynolds)
    if log_re <= 0 or log_re**2.58 == 0: return 0.005 # Avoid math errors and division by zero
    # ITTC 1957 friction line approximation
    return 0.455 / (log_re**2.58)

def calc_fuselage_wetted_area_m2(fuselage_fineness_ratio,fuselage_reference_area_m2):
    # Approximation based on fineness ratio and reference area (e.g., frontal area)
    # The factor '3.0' might be specific; standard approx involve pi*D*L etc. Using provided formula.
    return 3.0 * fuselage_fineness_ratio * fuselage_reference_area_m2

def calc_fuselage_reference_area_m2(fuselage_w_m,fuselage_h_m):
    # Frontal area for an elliptical cross-section
    return math.pi / 4.0 * fuselage_w_m * fuselage_h_m

def calc_fuselage_CdA(fuselage_Cd0_p_Cf, fuselage_Cf, fuselage_wetted_area_m2):
    # CdA = Cd based on wetted area * wetted area
    # Cd based on wetted area = Cf * (1 + 1.5/f^1.5 + 7/f^3) = Cf * Cd0_p_Cf
    return fuselage_Cd0_p_Cf * fuselage_Cf * fuselage_wetted_area_m2

def calc_fuselage_Cd0(fuselage_CdA, wing_ref_area_m2):
  # Convert CdA (drag area) to Cd0 based on wing reference area
  return fuselage_CdA / wing_ref_area_m2 if wing_ref_area_m2 > 0 else 0

def calc_wing_Cd0(wing_airfoil_cd_at_cruise_cl): return wing_airfoil_cd_at_cruise_cl

def calc_Cdi(spac_effic_factor,wing_AR,aircraft_CL):
    # Induced drag coefficient: CL^2 / (pi * e * AR)
    denominator = math.pi * spac_effic_factor * wing_AR
    if denominator <= 0: return float('inf') # Avoid division by zero, indicates problem
    return aircraft_CL**2 / denominator

def calc_horizontal_tail_area_m2(fuselage_l_m,wing_MAC,wing_ref_area_m2,horiz_tail_vol_coeff):
    # Approximate tail arm as half fuselage length
    tail_arm_approx = fuselage_l_m * 0.5
    if tail_arm_approx <= 0 or wing_ref_area_m2 <= 0 or wing_MAC <= 0: return 0
    numerator = horiz_tail_vol_coeff * wing_ref_area_m2 * wing_MAC
    return numerator / tail_arm_approx

def calc_vertical_tail_area_m2(fuselage_l_m,wing_ref_area_m2,vert_tail_vol_coeff,wingspan_m):
    # Approximate tail arm as half fuselage length
    tail_arm_approx = fuselage_l_m * 0.5
    if tail_arm_approx <= 0 or wing_ref_area_m2 <= 0 or wingspan_m <= 0: return 0
    numerator = vert_tail_vol_coeff * wingspan_m * wing_ref_area_m2
    return numerator / tail_arm_approx

def calc_wing_MAC(wing_root_chord_m,wing_taper_ratio):
    # Mean Aerodynamic Chord for a trapezoidal wing
    lambda_tr = wing_taper_ratio
    # Ensure taper ratio is non-negative
    lambda_tr = max(0, lambda_tr)
    denominator = 1.0 + lambda_tr
    if denominator == 0: return wing_root_chord_m # Handles case of lambda = -1 (theoretically)
    return (2.0/3.0) * wing_root_chord_m * (1.0 + lambda_tr + lambda_tr**2) / denominator

def calc_wing_root_chord_m(wingspan_m,wing_AR,wing_taper_ratio):
    # Root chord from Aspect Ratio, Span, and Taper Ratio
    lambda_tr = wing_taper_ratio
    # Ensure non-negative taper ratio
    lambda_tr = max(0, lambda_tr)
    denominator = wing_AR * (1.0 + lambda_tr)
    if denominator <= 0: return 0 # Avoid division by zero or negative AR
    return 2.0 * wingspan_m / denominator

def calc_wing_tip_chord_m(wing_root_chord_m, wing_taper_ratio): return wing_root_chord_m*wing_taper_ratio

def calc_wing_cruise_reynolds(wing_MAC,cruise_speed_m_p_s,kinematic_visc_sea_level_m2_p_s):
    if kinematic_visc_sea_level_m2_p_s <= 0: return 0
    return wing_MAC * cruise_speed_m_p_s / kinematic_visc_sea_level_m2_p_s

def calc_horizontal_tail_Cd0(horizontal_tail_area_m2,wing_ref_area_m2,empennage_airfoil_cd0):
    if wing_ref_area_m2 <= 0: return 0
    # Tail Cd0 contribution, referenced to wing area
    return (horizontal_tail_area_m2 / wing_ref_area_m2) * empennage_airfoil_cd0

def calc_vertical_tail_Cd0(vertical_tail_area_m2,wing_ref_area_m2,empennage_airfoil_cd0):
    if wing_ref_area_m2 <= 0: return 0
     # Tail Cd0 contribution, referenced to wing area
    return (vertical_tail_area_m2 / wing_ref_area_m2) * empennage_airfoil_cd0

def calc_landing_gear_Cd0(landing_gear_drag_area_m2, wing_ref_area_m2):
  if wing_ref_area_m2 <= 0: return 0
  # Landing gear Cd0 referenced to wing area
  return landing_gear_drag_area_m2 / wing_ref_area_m2

def calc_booms_Cd0(disk_area_m2, boom_drag_area, wing_ref_area_m2):
  # This formula seems empirically derived or specific. Units don't immediately match standard Cd0 def.
  # Cd0 = D / (q * S_ref). Here it uses disk_area/boom_drag_area? Let's use it as provided.
  # Ensure denominators are non-zero
  if boom_drag_area <= 0 or wing_ref_area_m2 <= 0: return 0
  booms_Cd0 = disk_area_m2 / boom_drag_area / wing_ref_area_m2 * 2.0 / 3.0 # Added '*'
  return booms_Cd0

def calc_booms_CdA_m2(booms_Cd0, wing_ref_area_m2): # Changed second arg to wing_ref_area
    # Calculate Drag Area (CdA) from Cd0 based on wing ref area
    # CdA = Cd0 * S_ref (where S_ref is the reference area used for Cd0)
    booms_CdA_m2 = booms_Cd0 * wing_ref_area_m2
    return booms_CdA_m2

def calc_aircraft_Cd(fuselage_Cd0,wing_Cd0,Cdi,horizontal_tail_Cd0,vertical_tail_Cd0,landing_gear_Cd0,booms_Cd0,trim_drag_factor,excres_protub_factor):
  # Sum of parasite drag components
  cd0_components = fuselage_Cd0+wing_Cd0+horizontal_tail_Cd0+vertical_tail_Cd0+landing_gear_Cd0+booms_Cd0
  # Apply excrescence/protuberance factor to parasite drag
  cd_parasite_total = cd0_components * excres_protub_factor
  # Add induced drag and apply trim drag factor to the total
  return (cd_parasite_total + Cdi) * trim_drag_factor

def calc_aircraft_CL(stall_speed_m_p_s,vehicle_cl_max,cruise_speed_m_p_s):
    # Calculate required CL at cruise speed based on CL_max at stall speed
    # Assumes Lift = Weight, L = 0.5 * rho * V^2 * S * CL
    # At stall: W = 0.5 * rho * Vs^2 * S * CLmax
    # At cruise: W = 0.5 * rho * Vc^2 * S * CLcruise
    # Equating W: CLcruise = CLmax * (Vs / Vc)^2
    if cruise_speed_m_p_s <= 0: return 0
    # Ensure speeds are non-negative
    stall_speed_m_p_s = max(0, stall_speed_m_p_s)
    cruise_speed_m_p_s = max(1e-6, cruise_speed_m_p_s) # Avoid division by zero
    return vehicle_cl_max * (stall_speed_m_p_s / cruise_speed_m_p_s)**2

def calc_cruise_L_p_D(aircraft_CL,aircraft_Cd):
  return aircraft_CL/aircraft_Cd if aircraft_Cd > 0 else float('inf') # Return Inf if drag is zero

def calc_wing_stall_reynolds(stall_speed_m_p_s,wing_MAC,kinematic_visc_sea_level_m2_p_s):
  if kinematic_visc_sea_level_m2_p_s <= 0: return 0
  return stall_speed_m_p_s*wing_MAC/kinematic_visc_sea_level_m2_p_s

def calc_dive_speed_m_p_s(cruise_speed_m_p_s): return cruise_speed_m_p_s*1.4

def calc_produc(payload_kg, cruise_range_EOL_km, hover_energy_k_W_h, climb_energy_k_W_h, cruise_energy_EOL_k_W_h):
    # Productivity = (Payload * Range) / Mission Energy
    total_mission_energy = hover_energy_k_W_h + climb_energy_k_W_h + cruise_energy_EOL_k_W_h
    if total_mission_energy <= 0: return 0
    return payload_kg * cruise_range_EOL_km / total_mission_energy

def calc_energy_effic(cruise_range_EOL_km, hover_energy_k_W_h, climb_energy_k_W_h, cruise_energy_EOL_k_W_h):
     # Energy Efficiency = Range / Mission Energy
    total_mission_energy = hover_energy_k_W_h + climb_energy_k_W_h + cruise_energy_EOL_k_W_h
    if total_mission_energy <= 0: return 0
    return cruise_range_EOL_km / total_mission_energy

def calc_miles_of_range_p_min_hov_time(hover_electric_power_k_W,cruise_electric_power_k_W,cruise_speed_m_p_s):
    # Range gained per minute reduction in hover time (energy equivalent)
    if cruise_electric_power_k_W <= 0: return 0
    energy_per_min_hover_kwh = hover_electric_power_k_W * (60.0 / 3600.0)
    time_equiv_cruise_hr = energy_per_min_hover_kwh / cruise_electric_power_k_W
    range_equiv_km = time_equiv_cruise_hr * cruise_speed_m_p_s * 3.6 # Convert m/s to km/hr
    range_equiv_miles = range_equiv_km / (M_PER_MILE / 1000.0)
    # Original formula simplified:
    # return (hover_electric_power_k_W / cruise_electric_power_k_W) * 60.0 * cruise_speed_m_p_s / M_PER_MILE
    # Let's use the clearer derivation:
    return range_equiv_miles


# --- Weight Estimation Functions ---
# ... (Keep all weight functions: wing, tails, fuselage, boom, lg, rotors, margin) ...
# Add checks for negative inputs if necessary
def calc_wing_weight_kg(max_takeoff_wt_kg, wing_ref_area_m2, wing_AR, wing_taper_ratio, wing_t_p_c):
    mtow_lb = max_takeoff_wt_kg * 2.20462; n_ult = 3.8 * 1.5; s_w_ft2 = wing_ref_area_m2 * FT_PER_M**2
    lambda_tr = wing_taper_ratio; tc_ratio = wing_t_p_c
    if mtow_lb <= 0 or n_ult <= 0 or s_w_ft2 <= 0 or wing_AR <= 0 or tc_ratio <= 0 or lambda_tr < 0: return 0
    # Ensure bases for power calculations are positive
    mtow_term_base = mtow_lb / 1000.0
    if mtow_term_base <= 0: return 0
    term1 = mtow_term_base**0.847

    n_ult_term_base = n_ult
    if n_ult_term_base <= 0: return 0
    term2 = n_ult_term_base**0.39579

    s_w_term_base = s_w_ft2
    if s_w_term_base <= 0: return 0
    term3 = s_w_term_base**0.21754

    ar_term_base = wing_AR
    if ar_term_base <= 0: return 0
    term4 = ar_term_base**0.50016

    tc_term_base = (1 + lambda_tr) / tc_ratio if tc_ratio > 0 else 0
    if tc_term_base <= 0: return 0
    term5 = tc_term_base**0.09359

    wing_weight_lb = 5.66411 * term1 * term2 * term3 * term4 * term5 * 0.9 # Raymer GA composite wing equation adjusted
    return wing_weight_lb * 0.453592

def calculate_horizontal_tail_weight_kg(horizontal_tail_area_m2, dive_speed_m_p_s):
    sh_ft2 = horizontal_tail_area_m2 * FT_PER_M**2; vd_kts = dive_speed_m_p_s * 1.94384 # m/s to knots
    if sh_ft2 <= 0 or vd_kts <= 0: return 0
    term1_base = sh_ft2
    if term1_base <= 0: return 0
    term1 = term1_base**0.2
    term2 = max(0, 0.00395 * term1 * vd_kts - 0.4885) # Empirical relation
    ht_weight_lb = sh_ft2 * term2 * 0.9 # Apply factor
    return ht_weight_lb * 0.453592

def cal_vertical_wing_weight_kg(vertical_tail_area_m2, dive_speed_m_p_s):
    sv_ft2 = vertical_tail_area_m2 * FT_PER_M**2; vd_kts = dive_speed_m_p_s * 1.94384
    if sv_ft2 <= 0 or vd_kts <= 0: return 0
    term1_base = sv_ft2
    if term1_base <= 0: return 0
    term1 = term1_base**0.2
    term2 = max(0, 0.00395 * term1 * vd_kts - 0.4885) # Empirical relation (same form as H-tail?)
    vt_weight_lb = sv_ft2 * term2 * 0.9 # Apply factor
    return vt_weight_lb * 0.453592

def calc_fuselage_weight_kg(fuselage_wetted_area_m2,max_takeoff_wt_kg, fuselage_l_m,
                            fuselage_fineness_ratio, air_density_sea_level_kg_p_m3,
                            cruise_speed_m_p_s):
    swet_f_ft2 = fuselage_wetted_area_m2 * FT_PER_M**2; mtow_lb = max_takeoff_wt_kg * 2.20462
    n_ult = 3.8 * 1.5; l_f_ft = fuselage_l_m * FT_PER_M; f = fuselage_fineness_ratio
    q_cruise_psf = 0.5 * air_density_sea_level_kg_p_m3 * cruise_speed_m_p_s**2 * 0.0208854 # Convert Pa to psf
    if swet_f_ft2 <= 0 or mtow_lb <= 0 or l_f_ft <= 0 or f <= 0 or q_cruise_psf <= 0: return 0
    # Ensure bases are positive
    term1_base = swet_f_ft2
    if term1_base <= 0: return 0
    term1 = term1_base**1.086

    term2_base = n_ult * mtow_lb
    if term2_base <= 0: return 0
    term2 = term2_base**0.177

    term3_base = l_f_ft
    if term3_base <= 0: return 0
    term3 = term3_base**(-0.051) # Check exponent if l_f < 1, but should be okay

    term4_base = f
    if term4_base <= 0: return 0
    term4 = term4_base**(-0.072) # Check exponent

    term5_base = q_cruise_psf
    if term5_base <= 0: return 0
    term5 = term5_base**0.241

    fuse_weight_lb = 0.052 * term1 * term2 * term3 * term4 * term5 * 0.9 # Raymer GA fuselage adjusted
    return fuse_weight_lb * 0.453592

def calc_boom_weight_kg(single_EPU_weight_kg, rotor_count, rotor_diameter_m, wing_MAC):
    epu_wt_lb = single_EPU_weight_kg * 2.20462
    # Approximate boom length needed to place rotors relative to wing
    boom_len_approx_m = (1.2 * rotor_diameter_m + wing_MAC) # Seems like half-span approx
    boom_len_approx_ft = boom_len_approx_m * FT_PER_M
    if epu_wt_lb <= 0 or rotor_count <= 0 or boom_len_approx_ft <= 0: return 0

    # Term 1: Related to EPU mass and number of rotors (mounting?)
    term1a_base = epu_wt_lb
    if term1a_base <= 0: return 0
    term1a = term1a_base**1.1433
    term1b_base = rotor_count
    if term1b_base <= 0: return 0
    term1b = term1b_base**1.3762
    term1_lb = 0.0412 * term1a * term1b

    # Term 2: Related to boom length (structural weight?) - Factor 6 seems arbitrary here.
    term2a_base = boom_len_approx_ft
    if term2a_base <= 0: return 0
    term2a = term2a_base**1.3476
    term2_lb = 6.0 * 0.2315 * term2a

    # Factor 2.0 at end implies two booms (one per side)
    return (term1_lb + term2_lb) * 0.453592 * 2.0

def calc_landing_gear_weight_kg(max_takeoff_wt_kg):
    # Simple factor-based estimation
    if max_takeoff_wt_kg < 0: return 0
    # Factors might represent complexity, retraction, shock absorption etc.
    return 0.0325 * max_takeoff_wt_kg * 1.14 * 1.08

def calc_lift_rotor_plus_hub_weight_kg(rotor_count, rotor_diameter_m, rotor_solidity,
                                  sound_speed_m_p_s, tip_mach,
                                  air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3,
                                  over_torque_factor):
    if rotor_count <= 0 or rotor_diameter_m <= 0 or rotor_solidity <= 0 or sound_speed_m_p_s <= 0 or tip_mach <= 0 or air_density_min_kg_p_m3 <= 0 or over_torque_factor <= 0 or air_density_sea_level_kg_p_m3 <= 0: return 0
    B44 = 3.2808; B47 = 0.4536 # Conversion factors ft/m and kg/lb
    radius_ft = rotor_diameter_m / 2.0 * B44; num_blades = 2 # Assuming 2 blades for lift rotors
    # Calculate chord based on solidity: sigma = (N * c) / (pi * R) -> c = sigma * pi * R / N
    if num_blades <= 0: return 0
    chord_ft = (rotor_solidity * math.pi * (rotor_diameter_m / 2.0) / num_blades) * B44
    # Calculate design tip speed considering density and over-torque effects
    density_ratio = air_density_sea_level_kg_p_m3 / air_density_min_kg_p_m3
    if density_ratio < 0 or over_torque_factor < 0: return 0 # Ensure sqrt args are non-negative
    tip_speed_design_ftps = sound_speed_m_p_s * tip_mach * math.sqrt(density_ratio) * math.sqrt(over_torque_factor) * B44

    # Empirical weight terms (units lb) - Ensure bases are positive
    term1a_base = radius_ft
    if term1a_base <= 0: return 0
    term1a = term1a_base**1.74231

    term1b_base = chord_ft
    if term1b_base <= 0: return 0
    term1b = term1b_base**0.77291

    term1c_base = tip_speed_design_ftps
    if term1c_base <= 0: return 0
    term1c = tip_speed_design_ftps**0.87562

    term1d_base = num_blades
    if term1d_base <= 0: return 0
    term1d = term1d_base**0.53479

    term1e_base = (rotor_count/2.0) # Number of rotor systems (assuming pairs)
    if term1e_base <= 0: return 0

    term1_lb = 0.0024419 * 1.0 * term1e_base * term1d * term1a * term1b * term1c

    term2a_base = radius_ft
    if term2a_base <= 0: return 0
    term2a = term2a_base**1.99321

    term2b_base = chord_ft
    if term2b_base <= 0: return 0
    term2b = term2b_base**0.79577

    term2c_base = tip_speed_design_ftps
    if term2c_base <= 0: return 0
    term2c = tip_speed_design_ftps**0.96323

    term2d_base = num_blades
    if term2d_base <= 0: return 0
    term2d = term2d_base**0.71443

    term2e_base = (rotor_count/2.0)
    if term2e_base <= 0: return 0
    term2f_base = 1.0 # Factor?
    if term2f_base <= 0: return 0
    term2f = term2f_base**1.02958

    term2_lb = 0.00037547 * term2f * term2e_base * term2d * term2a * term2b * term2c

    return (term1_lb + term2_lb) * B47

def calc_tilt_rotor_weight_kg(rotor_count, rotor_diameter_m, rotor_solidity,
                                     sound_speed_m_p_s, tip_mach,
                                     air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3,
                                     over_torque_factor):
     # Similar structure to lift rotor but with different constants/factors/blades
    if rotor_count <= 0 or rotor_diameter_m <= 0 or rotor_solidity <= 0 or sound_speed_m_p_s <= 0 or tip_mach <= 0 or air_density_min_kg_p_m3 <= 0 or over_torque_factor <= 0 or air_density_sea_level_kg_p_m3 <= 0: return 0
    B44 = 3.2808; B47 = 0.4536; radius_ft = rotor_diameter_m / 2.0 * B44; num_blades = 3 # Assuming 3 blades for tilt rotors
    if num_blades <= 0: return 0
    chord_ft = (rotor_solidity * math.pi * (rotor_diameter_m / 2.0) / num_blades) * B44
    density_ratio = air_density_sea_level_kg_p_m3 / air_density_min_kg_p_m3
    if density_ratio < 0 or over_torque_factor < 0: return 0 # Ensure sqrt args are non-negative
    tip_speed_design_ftps = sound_speed_m_p_s * tip_mach * math.sqrt(density_ratio) * math.sqrt(over_torque_factor) * B44
    tilt_factor = 1.1794 # Weight penalty for tilt mechanism?

    # Term 1 (similar to lift rotor)
    term1a_base = radius_ft
    if term1a_base <= 0: return 0
    term1a = term1a_base**1.74231
    term1b_base = chord_ft
    if term1b_base <= 0: return 0
    term1b = term1b_base**0.77291
    term1c_base = tip_speed_design_ftps
    if term1c_base <= 0: return 0
    term1c = tip_speed_design_ftps**0.87562
    term1d_base = num_blades
    if term1d_base <= 0: return 0
    term1d = term1d_base**0.53479
    term1e_base = (rotor_count/2.0)
    if term1e_base <= 0: return 0
    term1_lb = 0.0024419 * tilt_factor * term1e_base * term1d * term1a * term1b * term1c

    # Term 2 (similar to lift rotor)
    term2a_base = radius_ft
    if term2a_base <= 0: return 0
    term2a = term2a_base**1.99321
    term2b_base = chord_ft
    if term2b_base <= 0: return 0
    term2b = term2b_base**0.79577
    term2c_base = tip_speed_design_ftps
    if term2c_base <= 0: return 0
    term2c = tip_speed_design_ftps**0.96323
    term2d_base = num_blades
    if term2d_base <= 0: return 0
    term2d = term2d_base**0.71443
    term2e_base = (rotor_count/2.0)
    if term2e_base <= 0: return 0
    term2f_base = tilt_factor
    if term2f_base <= 0: return 0
    term2f = term2f_base**1.02958
    term2_lb = 0.00037547 * term2f * term2e_base * term2d * term2a * term2b * term2c

    return (term1_lb + term2_lb) * B47

def calc_margin(empty_weight_kg): return 0.05 * empty_weight_kg

# --- Helper Functions from power_profile.py ---
# ... (Keep calc_energy_kwh, calc_charge_ah, etc. as before) ...
def calc_energy_kwh(avg_power_kw, time_sec): return avg_power_kw * (time_sec * H_PER_S)
def calc_charge_ah(avg_power_kw, time_sec, system_voltage): return (calc_energy_kwh(avg_power_kw, time_sec) * W_PER_KW) / system_voltage if system_voltage > 0 else 0
def calc_avg_current_a(avg_power_kw, system_voltage): return (avg_power_kw * W_PER_KW) / system_voltage if system_voltage > 0 else 0
def calc_c_rate(avg_current_a, battery_total_charge_ah): return avg_current_a / battery_total_charge_ah if battery_total_charge_ah > 0 else 0
def calc_trip_energy_kwh(profile_energies_kwh): return sum(profile_energies_kwh)
def calc_battery_total_charge_ah(battery_total_energy_kwh, system_voltage): return (battery_total_energy_kwh * W_PER_KW) / system_voltage if system_voltage > 0 else 0
def calc_cells_series(system_voltage, cell_voltage): return math.ceil(system_voltage / cell_voltage) if cell_voltage > 0 else 0
def calc_cells_parallel(system_total_charge_ah, cell_charge_ah): return math.ceil(system_total_charge_ah / cell_charge_ah) if cell_charge_ah > 0 else 0
def calc_cell_count(cells_series, cells_parallel): return cells_series * cells_parallel
def calc_total_battery_energy_kwh(cells_series, cells_parallel, cell_voltage, cell_charge_ah): return (cells_series * cell_voltage * cells_parallel * cell_charge_ah) / 1000.0
def calc_recharge_time_min(battery_total_energy_kwh, final_dod_percent, charger_power_kw):
    if charger_power_kw <= 0: return float('inf')
    # Ensure DOD is reasonable
    if final_dod_percent is None or math.isinf(final_dod_percent) or final_dod_percent < 0: return 0
    used_fraction = min(1.0, final_dod_percent / 100.0)
    recharge_energy_kwh = battery_total_energy_kwh * used_fraction
    recharge_time_hr = recharge_energy_kwh / charger_power_kw
    return recharge_time_hr * 60.0

fixed_systems_weight_kg = params.get('actuator_weight_kg', 0) + params.get('furnishings_weight_kg', 0) + \
                          params.get('environmental_control_system_weight_kg', 0) + params.get('avionics_weight_kg', 0) + \
                          params.get('hv_power_distribution_weight_kg', 0) + params.get('lv_power_comms_weight_kg', 0)

# --- Main Logic ---
# ... (Calculate derived battery pack specific energies) ...
pack_spec_energy_usable_EOL_Wh_kg = calc_pack_usable_EOL_spec_energy(cell_voltage, cell_charge_ah, cell_mass_kg, cell_to_pack_mass_ratio, batt_soh, batt_inacc_energy_frac)
pack_spec_energy_gross_BOL_Wh_kg = calc_pack_gross_BOL_spec_energy(cell_voltage, cell_charge_ah, cell_mass_kg, cell_to_pack_mass_ratio)
if pack_spec_energy_usable_EOL_Wh_kg <= 0: sys.exit("ERROR: Cannot calculate valid EOL usable pack specific energy.")
print(f"\nDerived Pack Usable EOL Specific Energy: {pack_spec_energy_usable_EOL_Wh_kg:.2f} Wh/kg")
print(f"Derived Pack Gross BOL Specific Energy: {pack_spec_energy_gross_BOL_Wh_kg:.2f} Wh/kg")

# ... (Initialize final variables, including climb powers for plot) ...
final_max_takeoff_wt_kg = 0; final_batt_mass_kg = 0; final_cruise_range_EOL_km = 0
final_cruise_time_EOL_s = 0; final_climb_time_s = 0; final_climb_power_avg_kw = 0
final_climb_energy_kwh = 0
final_climb_power_start_kw = 0
final_climb_power_mid_kw = 0 # Still calculated but not used in 2-step plot
final_climb_power_end_kw = 0

# Initialize variables to store final converged state needed for jettison analysis
final_state = {}

# --- Scenario Handling ---
if fixed_battery_pack_mass_kg > 0:
    # --- Fixed Battery Scenario ---
    # ... (Calculations as before, including calculating final_climb_power_start/mid/end_kw) ...
    print("\n=== Fixed Battery Scenario ===")
    final_batt_mass_kg = fixed_battery_pack_mass_kg
    final_max_takeoff_wt_kg = params.get('max_takeoff_wt_kg', 0)
    print(f"Using Fixed Battery Mass: {final_batt_mass_kg:.2f} kg")
    print(f"Using Fixed MTOW: {final_max_takeoff_wt_kg:.2f} kg")
    if final_max_takeoff_wt_kg <= 0: sys.exit("ERROR: max_takeoff_wt_kg must be positive.")

    print("Calculating performance for fixed configuration...")
    wingspan_m = calc_wingspan_m(d_value_m); disk_area_m2 = calc_disk_area_m2(rotor_diameter_m, rotor_count)
    fuselage_fineness_ratio = calc_fuselage_fineness_ratio(fuselage_w_m, fuselage_l_m, fuselage_h_m)
    fuselage_reference_area_m2 = calc_fuselage_reference_area_m2(fuselage_w_m, fuselage_h_m)
    fuselage_wetted_area_m2 = calc_fuselage_wetted_area_m2(fuselage_fineness_ratio, fuselage_reference_area_m2)
    fuselage_reynolds = calc_fuselage_reynolds(kinematic_visc_sea_level_m2_p_s, cruise_speed_m_p_s, fuselage_l_m)
    fuselage_Cd0_p_Cf = calc_fuselage_Cd0_p_Cf(fuselage_fineness_ratio)
    fuselage_Cf = calc_fuselage_Cf(fuselage_reynolds, fuselage_fineness_ratio)
    fuselage_CdA = calc_fuselage_CdA(fuselage_Cd0_p_Cf, fuselage_Cf, fuselage_wetted_area_m2)
    wing_Cd0 = calc_wing_Cd0(wing_airfoil_cd_at_cruise_cl)
    wing_ref_area_m2 = calc_wing_ref_area_m2(final_max_takeoff_wt_kg, g_m_p_s2, air_density_sea_level_kg_p_m3, stall_speed_m_p_s, vehicle_cl_max)
    wing_AR = calc_wing_AR(wingspan_m, wing_ref_area_m2)
    wing_root_chord_m = calc_wing_root_chord_m(wingspan_m, wing_AR, wing_taper_ratio)
    wing_MAC = calc_wing_MAC(wing_root_chord_m, wing_taper_ratio)
    horizontal_tail_area_m2 = calc_horizontal_tail_area_m2(fuselage_l_m, wing_MAC, wing_ref_area_m2, horiz_tail_vol_coeff)
    vertical_tail_area_m2 = calc_vertical_tail_area_m2(fuselage_l_m, wing_ref_area_m2, vert_tail_vol_coeff, wingspan_m)
    fuselage_Cd0 = calc_fuselage_Cd0(fuselage_CdA, wing_ref_area_m2)
    aircraft_CL_cruise = calc_aircraft_CL(stall_speed_m_p_s, vehicle_cl_max, cruise_speed_m_p_s)
    Cdi_cruise = calc_Cdi(spac_effic_factor, wing_AR, aircraft_CL_cruise)
    horizontal_tail_Cd0 = calc_horizontal_tail_Cd0(horizontal_tail_area_m2, wing_ref_area_m2, empennage_airfoil_cd0)
    vertical_tail_Cd0 = calc_vertical_tail_Cd0(vertical_tail_area_m2, wing_ref_area_m2, empennage_airfoil_cd0)
    landing_gear_Cd0 = calc_landing_gear_Cd0(landing_gear_drag_area_m2, wing_ref_area_m2)
    booms_Cd0 = calc_booms_Cd0(disk_area_m2, boom_drag_area, wing_ref_area_m2)
    cd0_components = fuselage_Cd0 + wing_Cd0 + horizontal_tail_Cd0 + vertical_tail_Cd0 + landing_gear_Cd0 + booms_Cd0
    base_cd0_final = cd0_components * excres_protub_factor # STORE this final base Cd0
    aircraft_Cd_cruise = (base_cd0_final + Cdi_cruise) * trim_drag_factor
    cruise_L_p_D = calc_cruise_L_p_D(aircraft_CL_cruise, aircraft_Cd_cruise)
    # Powers
    hover_shaft_power_k_W = calc_hover_shaft_power_k_W(final_max_takeoff_wt_kg, g_m_p_s2, air_density_sea_level_kg_p_m3, hover_fom, disk_area_m2)
    hover_electric_power_k_W = calc_hover_electric_power_k_W(epu_effic, hover_shaft_power_k_W)
    cruise_shaft_power_k_w = calc_cruise_shaft_power_k_W(final_max_takeoff_wt_kg, g_m_p_s2, prop_effic, cruise_speed_m_p_s, cruise_L_p_D)
    cruise_electric_power_k_W = calc_cruise_electric_power_k_W(cruise_shaft_power_k_w, epu_effic)
    # Climb performance        
        
    final_climb_power_start_kw = calc_power_at_speed_climbing_k_W(final_max_takeoff_wt_kg, V_climb_start_mps, base_cd0_final, wing_AR, wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
    final_climb_power_mid_kw = calc_power_at_speed_climbing_k_W(final_max_takeoff_wt_kg, V_climb_mid_mps, base_cd0_final, wing_AR, wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
    final_climb_power_end_kw = calc_power_at_speed_climbing_k_W(final_max_takeoff_wt_kg, V_climb_end_mps, base_cd0_final, wing_AR, wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
    final_climb_power_avg_kw = (final_climb_power_start_kw + final_climb_power_mid_kw + final_climb_power_end_kw) / 3.0
    final_climb_time_s = calc_climb_time_s(altitude_diff_m, rate_of_climb_mps)
    final_climb_energy_kwh = calc_climb_energy_k_W_h(final_climb_power_avg_kw, final_climb_time_s)
    # Available energy and range
    batt_useable_energy_EOL_W_h = calc_batt_useable_energy_EOL_W_h(pack_spec_energy_usable_EOL_Wh_kg, final_batt_mass_kg)
    hover_energy_k_W_h = calc_hover_energy_k_W_h(hover_electric_power_k_W, equiv_hover_dur_s)
    # Reserves are calculated based on initial MTOW powers
    reserve_hover_energy_k_W_h = calc_reserve_hover_energy_k_W_h(hover_electric_power_k_W, reserve_hover_dur_s)
    reserve_cruise_energy_k_W_h = calc_reserve_cruise_energy_k_W_h(cruise_speed_m_p_s, reserve_range_km, cruise_electric_power_k_W)
    available_cruise_energy_EOL_k_W_h = calc_available_cruise_energy_EOL_k_W_h(batt_useable_energy_EOL_W_h, hover_energy_k_W_h, final_climb_energy_kwh, reserve_hover_energy_k_W_h, reserve_cruise_energy_k_W_h)
    final_cruise_time_EOL_s = calc_cruise_time_EOL_s(cruise_electric_power_k_W, available_cruise_energy_EOL_k_W_h)
    final_cruise_range_EOL_km = calc_cruise_range_EOL_km(final_cruise_time_EOL_s, cruise_speed_m_p_s)

    print(f"Calculated Achievable EOL Cruise Range: {final_cruise_range_EOL_km:.2f} km")
    print(f"Calculated EOL Cruise Time: {final_cruise_time_EOL_s:.2f} s")

    # Store final state
    final_state = {
        'mtow_kg': final_max_takeoff_wt_kg, 'batt_mass_kg': final_batt_mass_kg,
        'hover_power_kw': hover_electric_power_k_W, 'climb_power_avg_kw': final_climb_power_avg_kw,
        'cruise_power_kw': cruise_electric_power_k_W, 'hover_time_s': equiv_hover_dur_s,
        'climb_time_s': final_climb_time_s, 'cruise_time_s': final_cruise_time_EOL_s,
        'climb_power_start_kw': final_climb_power_start_kw, 'climb_power_mid_kw': final_climb_power_mid_kw,
        'climb_power_end_kw': final_climb_power_end_kw, 'base_cd0': base_cd0_final,
        'wing_AR': wing_AR, 'wing_ref_area_m2': wing_ref_area_m2, 'disk_area_m2': disk_area_m2,
        'cruise_L_p_D': cruise_L_p_D
    }


# ... inside the main 'else' block for Iterative Sizing Scenario ...

else:
    # --- Iterative Sizing Scenario ---
    print("\n=== Iterative MTOW Sizing Scenario ===")
    initial_mtow_guess_kg = params.get('max_takeoff_wt_kg', 3000)
    print(f"Initial MTOW guess from JSON: {initial_mtow_guess_kg:.2f} kg")

    # --- Calculate initial performance based on guess (ALWAYS RUN THIS) ---
    # ... (initial calculations remain largely the same) ...
    print("Calculating initial performance estimates...")
    wingspan_m_init = calc_wingspan_m(d_value_m)
    disk_area_m2_init = calc_disk_area_m2(rotor_diameter_m, rotor_count)
    fuselage_fineness_ratio_init = calc_fuselage_fineness_ratio(fuselage_w_m, fuselage_l_m, fuselage_h_m)
    fuselage_reference_area_m2_init = calc_fuselage_reference_area_m2(fuselage_w_m, fuselage_h_m)
    fuselage_wetted_area_m2_init = calc_fuselage_wetted_area_m2(fuselage_fineness_ratio_init, fuselage_reference_area_m2_init)
    fuselage_reynolds_init = calc_fuselage_reynolds(kinematic_visc_sea_level_m2_p_s, cruise_speed_m_p_s, fuselage_l_m)
    fuselage_Cd0_p_Cf_init = calc_fuselage_Cd0_p_Cf(fuselage_fineness_ratio_init)
    fuselage_Cf_init = calc_fuselage_Cf(fuselage_reynolds_init, fuselage_fineness_ratio_init)
    fuselage_CdA_init = calc_fuselage_CdA(fuselage_Cd0_p_Cf_init, fuselage_Cf_init, fuselage_wetted_area_m2_init)
    wing_Cd0_init = calc_wing_Cd0(wing_airfoil_cd_at_cruise_cl)
    dive_speed_m_p_s_init = calc_dive_speed_m_p_s(cruise_speed_m_p_s)
    over_torque_factor_init = calc_over_torque_factor(rotor_count)
    wing_ref_area_m2_init = calc_wing_ref_area_m2(initial_mtow_guess_kg, g_m_p_s2, air_density_sea_level_kg_p_m3, stall_speed_m_p_s, vehicle_cl_max)
    wing_AR_init = calc_wing_AR(wingspan_m_init, wing_ref_area_m2_init)
    wing_root_chord_m_init = calc_wing_root_chord_m(wingspan_m_init, wing_AR_init, wing_taper_ratio)
    wing_MAC_init = calc_wing_MAC(wing_root_chord_m_init, wing_taper_ratio)
    horizontal_tail_area_m2_init = calc_horizontal_tail_area_m2(fuselage_l_m, wing_MAC_init, wing_ref_area_m2_init, horiz_tail_vol_coeff)
    vertical_tail_area_m2_init = calc_vertical_tail_area_m2(fuselage_l_m, wing_ref_area_m2_init, vert_tail_vol_coeff, wingspan_m_init)
    fuselage_Cd0_init = calc_fuselage_Cd0(fuselage_CdA_init, wing_ref_area_m2_init)
    aircraft_CL_cruise_init = calc_aircraft_CL(stall_speed_m_p_s, vehicle_cl_max, cruise_speed_m_p_s)
    Cdi_cruise_init = calc_Cdi(spac_effic_factor, wing_AR_init, aircraft_CL_cruise_init)
    horizontal_tail_Cd0_init = calc_horizontal_tail_Cd0(horizontal_tail_area_m2_init, wing_ref_area_m2_init, empennage_airfoil_cd0)
    vertical_tail_Cd0_init = calc_vertical_tail_Cd0(vertical_tail_area_m2_init, wing_ref_area_m2_init, empennage_airfoil_cd0)
    landing_gear_Cd0_init = calc_landing_gear_Cd0(landing_gear_drag_area_m2, wing_ref_area_m2_init)
    booms_Cd0_init = calc_booms_Cd0(disk_area_m2_init, boom_drag_area, wing_ref_area_m2_init)
    cd0_components_init = fuselage_Cd0_init + wing_Cd0_init + horizontal_tail_Cd0_init + vertical_tail_Cd0_init + landing_gear_Cd0_init + booms_Cd0_init
    base_cd0_init = cd0_components_init * excres_protub_factor
    aircraft_Cd_cruise_init = (base_cd0_init + Cdi_cruise_init) * trim_drag_factor
    cruise_L_p_D_init = calc_cruise_L_p_D(aircraft_CL_cruise_init, aircraft_Cd_cruise_init)
    hover_shaft_power_k_W_init = calc_hover_shaft_power_k_W(initial_mtow_guess_kg, g_m_p_s2, air_density_sea_level_kg_p_m3, hover_fom, disk_area_m2_init)
    hover_electric_power_k_W_init = calc_hover_electric_power_k_W(epu_effic, hover_shaft_power_k_W_init) # Now defined before if/else
    cruise_shaft_power_k_w_init = calc_cruise_shaft_power_k_W(initial_mtow_guess_kg, g_m_p_s2, prop_effic, cruise_speed_m_p_s, cruise_L_p_D_init)
    cruise_electric_power_k_W_init = calc_cruise_electric_power_k_W(cruise_shaft_power_k_w_init, epu_effic) # Now defined before if/else
    # Initial climb energy (multi-point avg) using _init values
    p_climb_start_init = calc_power_at_speed_climbing_k_W(initial_mtow_guess_kg, V_climb_start_mps, base_cd0_init, wing_AR_init, wing_ref_area_m2_init, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
    p_climb_mid_init = calc_power_at_speed_climbing_k_W(initial_mtow_guess_kg, V_climb_mid_mps, base_cd0_init, wing_AR_init, wing_ref_area_m2_init, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
    p_climb_end_init = calc_power_at_speed_climbing_k_W(initial_mtow_guess_kg, V_climb_end_mps, base_cd0_init, wing_AR_init, wing_ref_area_m2_init, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
    climb_power_avg_kw_init = (p_climb_start_init + p_climb_mid_init + p_climb_end_init) / 3.0
    climb_time_s_init = calc_climb_time_s(altitude_diff_m, rate_of_climb_mps)
    climb_energy_kwh_init = calc_climb_energy_k_W_h(climb_power_avg_kw_init, climb_time_s_init) # Now defined before if/else
    # Initial weights
    rotor_RPM_hover_init = calc_rotor_RPM_hover(sound_speed_m_p_s, rotor_diameter_m, tip_mach)
    motor_torque_hover_Nm_init = calc_motor_torque_hover_Nm(rotor_count, rotor_RPM_hover_init, hover_shaft_power_k_W_init)
    motor_RPM_max_thrust_min_rho_init = calc_motor_RPM_max_thrust_min_rho(rotor_RPM_hover_init, over_torque_factor_init, air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3)
    motor_torque_max_thrust_Nm_init = calc_motor_torque_max_thrust_Nm(over_torque_factor_init, motor_torque_hover_Nm_init)
    motor_mechanical_power_k_W_sizing_init = calc_motor_mechanical_power_sizing_k_W(motor_RPM_max_thrust_min_rho_init, motor_torque_max_thrust_Nm_init)
    single_EPU_weight_kg_init = calc_single_EPU_weight_kg(motor_torque_max_thrust_Nm_init, motor_mechanical_power_k_W_sizing_init)
    total_EPU_weight_kg_init = calc_total_EPU_weight_kg(single_EPU_weight_kg_init, rotor_count)
    Ct_hover_init = calc_Ct_hover(initial_mtow_guess_kg, g_m_p_s2, rotor_count, air_density_sea_level_kg_p_m3, rotor_diameter_m, rotor_RPM_hover_init)
    rotor_solidity_init = calc_rotor_solidity(Ct_hover_init, rotor_avg_cl)
    wing_weight_kg_init = calc_wing_weight_kg(initial_mtow_guess_kg, wing_ref_area_m2_init, wing_AR_init, wing_taper_ratio, wing_t_p_c)
    horizontal_tail_weight_kg_init = calculate_horizontal_tail_weight_kg(horizontal_tail_area_m2_init, dive_speed_m_p_s_init)
    vertical_wing_weight_kg_init = cal_vertical_wing_weight_kg(vertical_tail_area_m2_init, dive_speed_m_p_s_init)
    fuselage_weight_kg_init = calc_fuselage_weight_kg(fuselage_wetted_area_m2_init, initial_mtow_guess_kg, fuselage_l_m, fuselage_fineness_ratio_init, air_density_sea_level_kg_p_m3, cruise_speed_m_p_s)
    boom_weight_kg_init = calc_boom_weight_kg(single_EPU_weight_kg_init, rotor_count, rotor_diameter_m, wing_MAC_init)
    landing_gear_weight_kg_init = calc_landing_gear_weight_kg(initial_mtow_guess_kg)
    lift_rotor_plus_hub_weight_kg_init = calc_lift_rotor_plus_hub_weight_kg(rotor_count, rotor_diameter_m, rotor_solidity_init, sound_speed_m_p_s, tip_mach, air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3, over_torque_factor_init)
    tilt_rotor_weight_kg_init = calc_tilt_rotor_weight_kg(rotor_count, rotor_diameter_m, rotor_solidity_init, sound_speed_m_p_s, tip_mach, air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3, over_torque_factor_init)
    empty_weight_less_battery_kg_init = (wing_weight_kg_init + horizontal_tail_weight_kg_init + vertical_wing_weight_kg_init + fuselage_weight_kg_init + boom_weight_kg_init + landing_gear_weight_kg_init + total_EPU_weight_kg_init + lift_rotor_plus_hub_weight_kg_init + tilt_rotor_weight_kg_init + fixed_systems_weight_kg)
    margin_kg_init = calc_margin(empty_weight_less_battery_kg_init)


    # --- Determine Iteration Target Range and Initial Battery Mass ---
    if 'target_cruise_range_EOL_km' in params:
        iteration_target_range_km = target_cruise_range_EOL_km
        print(f"Using target cruise range from JSON for iteration: {iteration_target_range_km:.2f} km")

        # Estimate initial battery mass needed for this target range using initial performance
        target_cruise_time_EOL_s_init = iteration_target_range_km * 1000.0 / cruise_speed_m_p_s if cruise_speed_m_p_s > 0 else 0
        cruise_energy_req_k_W_h_init = cruise_electric_power_k_W_init * target_cruise_time_EOL_s_init / 3600.0
        hover_energy_k_W_h_init = calc_hover_energy_k_W_h(hover_electric_power_k_W_init, equiv_hover_dur_s) # Now defined
        # Reserves based on initial guess powers
        reserve_hover_energy_k_W_h_init = calc_reserve_hover_energy_k_W_h(hover_electric_power_k_W_init, reserve_hover_dur_s) # Now defined
        reserve_cruise_energy_k_W_h_init = calc_reserve_cruise_energy_k_W_h(cruise_speed_m_p_s, reserve_range_km, cruise_electric_power_k_W_init) # Now defined

        total_usable_energy_EOL_req_k_W_h_init = hover_energy_k_W_h_init + climb_energy_kwh_init + cruise_energy_req_k_W_h_init \
                                                + reserve_hover_energy_k_W_h_init + reserve_cruise_energy_k_W_h_init
        batt_mass_kg_estimate = total_usable_energy_EOL_req_k_W_h_init * 1000.0 / pack_spec_energy_usable_EOL_Wh_kg if pack_spec_energy_usable_EOL_Wh_kg > 0 else 0
        print(f"Initial battery mass estimate for target range: {batt_mass_kg_estimate:.2f} kg")

    else: # Verification mode: Calculate implied range and battery mass
        implied_batt_mass_kg_init = initial_mtow_guess_kg - empty_weight_less_battery_kg_init - payload_kg - margin_kg_init
        if implied_batt_mass_kg_init <= 0: sys.exit("ERROR: Initial MTOW implies non-positive battery mass.")

        batt_useable_energy_EOL_W_h_init = calc_batt_useable_energy_EOL_W_h(pack_spec_energy_usable_EOL_Wh_kg, implied_batt_mass_kg_init)
        hover_energy_k_W_h_init = calc_hover_energy_k_W_h(hover_electric_power_k_W_init, equiv_hover_dur_s) # Now defined
        # Reserves based on initial guess powers
        reserve_hover_energy_k_W_h_init = calc_reserve_hover_energy_k_W_h(hover_electric_power_k_W_init, reserve_hover_dur_s) # Now defined
        reserve_cruise_energy_k_W_h_init = calc_reserve_cruise_energy_k_W_h(cruise_speed_m_p_s, reserve_range_km, cruise_electric_power_k_W_init) # Now defined
        available_cruise_energy_EOL_k_W_h_init = calc_available_cruise_energy_EOL_k_W_h(batt_useable_energy_EOL_W_h_init, hover_energy_k_W_h_init, climb_energy_kwh_init, reserve_hover_energy_k_W_h_init, reserve_cruise_energy_k_W_h_init)
        achieved_cruise_time_EOL_s_init = calc_cruise_time_EOL_s(cruise_electric_power_k_W_init, available_cruise_energy_EOL_k_W_h_init)
        achieved_cruise_range_EOL_km_init = calc_cruise_range_EOL_km(achieved_cruise_time_EOL_s_init, cruise_speed_m_p_s)

        iteration_target_range_km = achieved_cruise_range_EOL_km_init
        batt_mass_kg_estimate = implied_batt_mass_kg_init # Use the implied mass as the estimate
        print(f"Using calculated achievable cruise range for iteration (verification mode): {iteration_target_range_km:.2f} km")
        print(f"Initial battery mass estimate (implied): {batt_mass_kg_estimate:.2f} kg")


    # --- Iteration Loop ---
    tolerance = 0.5
    max_iterations = 100
    mtow_current_kg = initial_mtow_guess_kg
    current_batt_mass_kg = batt_mass_kg_estimate # Initialize with the estimate
    converged = False
    base_cd0 = base_cd0_init # Use initial base Cd0 for first iteration

    # Variables to store converged values from the loop
    iter_wing_ref_area_m2 = wing_ref_area_m2_init
    iter_wing_AR = wing_AR_init
    iter_hover_electric_power_k_W = hover_electric_power_k_W_init
    iter_cruise_electric_power_k_W = cruise_electric_power_k_W_init
    iter_cruise_L_p_D = cruise_L_p_D_init
    iter_p_climb_start = p_climb_start_init
    iter_p_climb_mid = p_climb_mid_init
    iter_p_climb_end = p_climb_end_init
    iter_climb_electric_power_avg_k_W = climb_power_avg_kw_init
    iter_climb_time_s = climb_time_s_init
    iter_climb_energy_k_W_h = climb_energy_kwh_init

    # --- Iteration Loop ---
    # ... (The rest of the iteration loop remains the same) ...
    for i in range(max_iterations):
        print(f"\nIteration {i+1}: Current MTOW = {mtow_current_kg:.2f} kg")

        # --- Recalculate dependencies ---
        # Geometry
        iter_wing_ref_area_m2 = calc_wing_ref_area_m2(mtow_current_kg, g_m_p_s2, air_density_sea_level_kg_p_m3, stall_speed_m_p_s, vehicle_cl_max)
        wingspan_m = calc_wingspan_m(d_value_m)
        iter_wing_AR = calc_wing_AR(wingspan_m, iter_wing_ref_area_m2)
        wing_root_chord_m = calc_wing_root_chord_m(wingspan_m, iter_wing_AR, wing_taper_ratio)
        wing_MAC = calc_wing_MAC(wing_root_chord_m, wing_taper_ratio)
        horizontal_tail_area_m2 = calc_horizontal_tail_area_m2(fuselage_l_m, wing_MAC, iter_wing_ref_area_m2, horiz_tail_vol_coeff)
        vertical_tail_area_m2 = calc_vertical_tail_area_m2(fuselage_l_m, iter_wing_ref_area_m2, vert_tail_vol_coeff, wingspan_m)

        # Aero Coefficients
        fuselage_Cd0 = calc_fuselage_Cd0(fuselage_CdA_init, iter_wing_ref_area_m2)
        horizontal_tail_Cd0 = calc_horizontal_tail_Cd0(horizontal_tail_area_m2, iter_wing_ref_area_m2, empennage_airfoil_cd0)
        vertical_tail_Cd0 = calc_vertical_tail_Cd0(vertical_tail_area_m2, iter_wing_ref_area_m2, empennage_airfoil_cd0)
        landing_gear_Cd0 = calc_landing_gear_Cd0(landing_gear_drag_area_m2, iter_wing_ref_area_m2)
        booms_Cd0 = calc_booms_Cd0(disk_area_m2_init, boom_drag_area, iter_wing_ref_area_m2)
        # Base Cd0
        cd0_components = fuselage_Cd0 + wing_Cd0_init + horizontal_tail_Cd0 + vertical_tail_Cd0 + landing_gear_Cd0 + booms_Cd0
        base_cd0 = cd0_components * excres_protub_factor # Recalculate base Cd0 each iteration
        # Cruise Aero
        aircraft_CL_cruise = calc_aircraft_CL(stall_speed_m_p_s, vehicle_cl_max, cruise_speed_m_p_s)
        Cdi_cruise = calc_Cdi(spac_effic_factor, iter_wing_AR, aircraft_CL_cruise)
        aircraft_Cd_cruise = (base_cd0 + Cdi_cruise) * trim_drag_factor
        iter_cruise_L_p_D = calc_cruise_L_p_D(aircraft_CL_cruise, aircraft_Cd_cruise)
        print(f"  Wing Area: {iter_wing_ref_area_m2:.2f} m^2, Cruise L/D: {iter_cruise_L_p_D:.2f}")

        # Power Requirements
        hover_shaft_power_k_W = calc_hover_shaft_power_k_W(mtow_current_kg, g_m_p_s2, air_density_sea_level_kg_p_m3, hover_fom, disk_area_m2_init)
        iter_hover_electric_power_k_W = calc_hover_electric_power_k_W(epu_effic, hover_shaft_power_k_W)
        cruise_shaft_power_k_w = calc_cruise_shaft_power_k_W(mtow_current_kg, g_m_p_s2, prop_effic, cruise_speed_m_p_s, iter_cruise_L_p_D)
        iter_cruise_electric_power_k_W = calc_cruise_electric_power_k_W(cruise_shaft_power_k_w, epu_effic)
        # Average Climb Power (Multi-point)
        iter_p_climb_start = calc_power_at_speed_climbing_k_W(mtow_current_kg, V_climb_start_mps, base_cd0, iter_wing_AR, iter_wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
        iter_p_climb_mid = calc_power_at_speed_climbing_k_W(mtow_current_kg, V_climb_mid_mps, base_cd0, iter_wing_AR, iter_wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
        iter_p_climb_end = calc_power_at_speed_climbing_k_W(mtow_current_kg, V_climb_end_mps, base_cd0, iter_wing_AR, iter_wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
        iter_climb_electric_power_avg_k_W = (iter_p_climb_start + iter_p_climb_mid + iter_p_climb_end) / 3.0
        print(f"  Hover Pwr: {iter_hover_electric_power_k_W:.2f} kW, Climb Pwr (Avg): {iter_climb_electric_power_avg_k_W:.2f} kW, Cruise Pwr: {iter_cruise_electric_power_k_W:.2f} kW")

        # Energy Required for Target Mission
        target_cruise_time_EOL_s = iteration_target_range_km * 1000.0 / cruise_speed_m_p_s if cruise_speed_m_p_s > 0 else 0
        cruise_energy_req_k_W_h = iter_cruise_electric_power_k_W * target_cruise_time_EOL_s / 3600.0
        hover_energy_k_W_h = calc_hover_energy_k_W_h(iter_hover_electric_power_k_W, equiv_hover_dur_s)
        iter_climb_time_s = calc_climb_time_s(altitude_diff_m, rate_of_climb_mps)
        iter_climb_energy_k_W_h = calc_climb_energy_k_W_h(iter_climb_electric_power_avg_k_W, iter_climb_time_s)
        # Reserves based on current iteration's powers
        reserve_hover_energy_k_W_h = calc_reserve_hover_energy_k_W_h(iter_hover_electric_power_k_W, reserve_hover_dur_s)
        reserve_cruise_energy_k_W_h = calc_reserve_cruise_energy_k_W_h(cruise_speed_m_p_s, reserve_range_km, iter_cruise_electric_power_k_W)
        total_usable_energy_EOL_req_k_W_h = hover_energy_k_W_h + iter_climb_energy_k_W_h + cruise_energy_req_k_W_h \
                                            + reserve_hover_energy_k_W_h + reserve_cruise_energy_k_W_h
        print(f"  Required EOL Usable Energy for Target Mission: {total_usable_energy_EOL_req_k_W_h:.2f} kWh")

        # Battery Mass Calculation
        batt_mass_kg_new = total_usable_energy_EOL_req_k_W_h * 1000.0 / pack_spec_energy_usable_EOL_Wh_kg if pack_spec_energy_usable_EOL_Wh_kg > 0 else 0
        print(f"  Calculated Battery Mass: {batt_mass_kg_new:.2f} kg")

        # Motor & Rotor Sizing
        rotor_RPM_hover = calc_rotor_RPM_hover(sound_speed_m_p_s, rotor_diameter_m, tip_mach)
        over_torque_factor = calc_over_torque_factor(rotor_count)
        motor_torque_hover_Nm = calc_motor_torque_hover_Nm(rotor_count, rotor_RPM_hover, hover_shaft_power_k_W)
        motor_RPM_max_thrust_min_rho = calc_motor_RPM_max_thrust_min_rho(rotor_RPM_hover, over_torque_factor, air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3)
        motor_torque_max_thrust_Nm = calc_motor_torque_max_thrust_Nm(over_torque_factor, motor_torque_hover_Nm)
        motor_mechanical_power_k_W_sizing = calc_motor_mechanical_power_sizing_k_W(motor_RPM_max_thrust_min_rho, motor_torque_max_thrust_Nm)
        single_EPU_weight_kg = calc_single_EPU_weight_kg(motor_torque_max_thrust_Nm, motor_mechanical_power_k_W_sizing)
        total_EPU_weight_kg = calc_total_EPU_weight_kg(single_EPU_weight_kg, rotor_count)
        Ct_hover = calc_Ct_hover(mtow_current_kg, g_m_p_s2, rotor_count, air_density_sea_level_kg_p_m3, rotor_diameter_m, rotor_RPM_hover)
        rotor_solidity = calc_rotor_solidity(Ct_hover, rotor_avg_cl)

        # Component Weights
        dive_speed_m_p_s = calc_dive_speed_m_p_s(cruise_speed_m_p_s)
        wing_weight_kg = calc_wing_weight_kg(mtow_current_kg, iter_wing_ref_area_m2, iter_wing_AR, wing_taper_ratio, wing_t_p_c)
        horizontal_tail_weight_kg = calculate_horizontal_tail_weight_kg(horizontal_tail_area_m2, dive_speed_m_p_s)
        vertical_wing_weight_kg = cal_vertical_wing_weight_kg(vertical_tail_area_m2, dive_speed_m_p_s)
        fuselage_weight_kg = calc_fuselage_weight_kg(fuselage_wetted_area_m2_init, mtow_current_kg, fuselage_l_m, fuselage_fineness_ratio_init, air_density_sea_level_kg_p_m3, cruise_speed_m_p_s)
        boom_weight_kg = calc_boom_weight_kg(single_EPU_weight_kg, rotor_count, rotor_diameter_m, wing_MAC)
        landing_gear_weight_kg = calc_landing_gear_weight_kg(mtow_current_kg)
        lift_rotor_plus_hub_weight_kg = calc_lift_rotor_plus_hub_weight_kg(rotor_count, rotor_diameter_m, rotor_solidity, sound_speed_m_p_s, tip_mach, air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3, over_torque_factor)
        tilt_rotor_weight_kg = calc_tilt_rotor_weight_kg(rotor_count, rotor_diameter_m, rotor_solidity, sound_speed_m_p_s, tip_mach, air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3, over_torque_factor)

        # Empty Weight & Margin
        empty_weight_less_battery_kg = (wing_weight_kg + horizontal_tail_weight_kg + vertical_wing_weight_kg + fuselage_weight_kg + boom_weight_kg + landing_gear_weight_kg + total_EPU_weight_kg + lift_rotor_plus_hub_weight_kg + tilt_rotor_weight_kg + fixed_systems_weight_kg)
        margin_kg = calc_margin(empty_weight_less_battery_kg)
        print(f"  Structure+Systems Weight: {empty_weight_less_battery_kg:.2f} kg, Margin: {margin_kg:.2f} kg")

        # New MTOW Estimate
        mtow_new_kg = empty_weight_less_battery_kg + batt_mass_kg_new + payload_kg + margin_kg
        print(f"  New Estimated MTOW: {mtow_new_kg:.2f} kg")

        # Check Convergence
        delta_mtow = abs(mtow_new_kg - mtow_current_kg)
        print(f"  MTOW Delta: {delta_mtow:.3f} kg")
        if delta_mtow < tolerance:
            print(f"\nConvergence achieved after {i+1} iterations.")
            final_max_takeoff_wt_kg = mtow_new_kg; final_batt_mass_kg = batt_mass_kg_new
            # Store final average power and time from converged iteration
            final_climb_power_avg_kw = iter_climb_electric_power_avg_k_W
            final_climb_time_s = iter_climb_time_s
            final_climb_energy_kwh = iter_climb_energy_k_W_h
            # Store final point powers for plotting
            final_climb_power_start_kw = iter_p_climb_start
            final_climb_power_mid_kw = iter_p_climb_mid
            final_climb_power_end_kw = iter_p_climb_end
            # Store other final values
            base_cd0_final = base_cd0 # Store the converged base CD0
            final_wing_ref_area = iter_wing_ref_area_m2
            final_wing_AR = iter_wing_AR
            final_hover_power = iter_hover_electric_power_k_W
            final_cruise_power = iter_cruise_electric_power_k_W
            final_cruise_L_p_D = iter_cruise_L_p_D
            converged = True
            break

        # Update for next iteration
        mtow_current_kg = mtow_new_kg; current_batt_mass_kg = batt_mass_kg_new

    if not converged:
        print(f"\nWarning: MTOW iteration did not converge within {max_iterations} iterations.")
        final_max_takeoff_wt_kg = mtow_current_kg; final_batt_mass_kg = current_batt_mass_kg
        # Store last iteration's values
        final_climb_power_avg_kw = iter_climb_electric_power_avg_k_W
        final_climb_time_s = iter_climb_time_s
        final_climb_energy_kwh = iter_climb_energy_k_W_h
        final_climb_power_start_kw = iter_p_climb_start
        final_climb_power_mid_kw = iter_p_climb_mid
        final_climb_power_end_kw = iter_p_climb_end
        base_cd0_final = base_cd0
        final_wing_ref_area = iter_wing_ref_area_m2
        final_wing_AR = iter_wing_AR
        final_hover_power = iter_hover_electric_power_k_W
        final_cruise_power = iter_cruise_electric_power_k_W
        final_cruise_L_p_D = iter_cruise_L_p_D

    # Store the cruise range/time corresponding to the target
    final_cruise_time_EOL_s = target_cruise_time_EOL_s
    final_cruise_range_EOL_km = iteration_target_range_km

    # Store final state for potential jettison analysis
    final_state = {
        'mtow_kg': final_max_takeoff_wt_kg, 'batt_mass_kg': final_batt_mass_kg,
        'hover_power_kw': final_hover_power, 'climb_power_avg_kw': final_climb_power_avg_kw,
        'cruise_power_kw': final_cruise_power, 'hover_time_s': equiv_hover_dur_s,
        'climb_time_s': final_climb_time_s, 'cruise_time_s': final_cruise_time_EOL_s,
        'climb_power_start_kw': final_climb_power_start_kw, 'climb_power_mid_kw': final_climb_power_mid_kw,
        'climb_power_end_kw': final_climb_power_end_kw, 'base_cd0': base_cd0_final,
        'wing_AR': final_wing_AR, 'wing_ref_area_m2': final_wing_ref_area,
        'disk_area_m2': disk_area_m2_init, # Use the initial disk area as it's fixed
        'cruise_L_p_D': final_cruise_L_p_D
    }


# --- Final Calculations & Analysis ---
# ... (Final print statements, recalculate final state variables, including climb powers) ...
print(f"\nFinal MTOW: {final_max_takeoff_wt_kg:.2f} kg")
print(f"Final Battery Mass: {final_batt_mass_kg:.2f} kg")
print("\n=== Performing Final Calculations & Power Profile Analysis ===")

# Recalculate final state variables based on converged/fixed values
# These are mostly needed if the 'fixed battery' path was taken, or for consistency
# If iterative, these should match the 'final_state' dict values
wing_ref_area_m2 = calc_wing_ref_area_m2(final_max_takeoff_wt_kg, g_m_p_s2, air_density_sea_level_kg_p_m3, stall_speed_m_p_s, vehicle_cl_max)
wingspan_m = calc_wingspan_m(d_value_m)
wing_AR = calc_wing_AR(wingspan_m, wing_ref_area_m2)
wing_root_chord_m = calc_wing_root_chord_m(wingspan_m, wing_AR, wing_taper_ratio)
wing_MAC = calc_wing_MAC(wing_root_chord_m, wing_taper_ratio)
wing_tip_chord_m = calc_wing_tip_chord_m(wing_root_chord_m, wing_taper_ratio)
horizontal_tail_area_m2 = calc_horizontal_tail_area_m2(fuselage_l_m, wing_MAC, wing_ref_area_m2, horiz_tail_vol_coeff)
vertical_tail_area_m2 = calc_vertical_tail_area_m2(fuselage_l_m, wing_ref_area_m2, vert_tail_vol_coeff, wingspan_m)
fuselage_fineness_ratio = calc_fuselage_fineness_ratio(fuselage_w_m, fuselage_l_m, fuselage_h_m)
fuselage_reference_area_m2 = calc_fuselage_reference_area_m2(fuselage_w_m, fuselage_h_m)
fuselage_wetted_area_m2 = calc_fuselage_wetted_area_m2(fuselage_fineness_ratio, fuselage_reference_area_m2)
fuselage_reynolds = calc_fuselage_reynolds(kinematic_visc_sea_level_m2_p_s, cruise_speed_m_p_s, fuselage_l_m)
fuselage_Cd0_p_Cf = calc_fuselage_Cd0_p_Cf(fuselage_fineness_ratio)
fuselage_Cf = calc_fuselage_Cf(fuselage_reynolds, fuselage_fineness_ratio)
fuselage_CdA = calc_fuselage_CdA(fuselage_Cd0_p_Cf, fuselage_Cf, fuselage_wetted_area_m2)
fuselage_Cd0 = calc_fuselage_Cd0(fuselage_CdA, wing_ref_area_m2)
wing_Cd0 = calc_wing_Cd0(wing_airfoil_cd_at_cruise_cl)
aircraft_CL_cruise = calc_aircraft_CL(stall_speed_m_p_s, vehicle_cl_max, cruise_speed_m_p_s)
Cdi_cruise = calc_Cdi(spac_effic_factor, wing_AR, aircraft_CL_cruise)
horizontal_tail_Cd0 = calc_horizontal_tail_Cd0(horizontal_tail_area_m2, wing_ref_area_m2, empennage_airfoil_cd0)
vertical_tail_Cd0 = calc_vertical_tail_Cd0(vertical_tail_area_m2, wing_ref_area_m2, empennage_airfoil_cd0)
landing_gear_Cd0 = calc_landing_gear_Cd0(landing_gear_drag_area_m2, wing_ref_area_m2)
disk_area_m2 = calc_disk_area_m2(rotor_diameter_m, rotor_count) # Recalculate consistent disk area
booms_Cd0 = calc_booms_Cd0(disk_area_m2, boom_drag_area, wing_ref_area_m2)
cd0_components = fuselage_Cd0 + wing_Cd0 + horizontal_tail_Cd0 + vertical_tail_Cd0 + landing_gear_Cd0 + booms_Cd0
# Use the base_cd0_final determined during sizing/fixed analysis
if 'base_cd0' in final_state:
    base_cd0_final = final_state['base_cd0']
else: # Fallback if fixed battery case didn't store it (it should now)
    base_cd0_final = cd0_components * excres_protub_factor

aircraft_Cd_cruise = (base_cd0_final + Cdi_cruise) * trim_drag_factor
cruise_L_p_D = calc_cruise_L_p_D(aircraft_CL_cruise, aircraft_Cd_cruise)

# Final Power (using final state values for consistency)
hover_electric_power_k_W = final_state.get('hover_power_kw', 0)
cruise_electric_power_k_W = final_state.get('cruise_power_kw', 0)
climb_electric_power_avg_k_W = final_state.get('climb_power_avg_kw', 0)
climb_power_start_kw = final_state.get('climb_power_start_kw', 0)
climb_power_mid_kw = final_state.get('climb_power_mid_kw', 0)
climb_power_end_kw = final_state.get('climb_power_end_kw', 0)
climb_time_s = final_state.get('climb_time_s', 0) # Use the climb time from the final state

# Final Energy & Range Confirmation
# Reserves should be based on the initial MTOW powers calculated finally
reserve_hover_energy_k_W_h = calc_reserve_hover_energy_k_W_h(hover_electric_power_k_W, reserve_hover_dur_s)
reserve_cruise_energy_k_W_h = calc_reserve_cruise_energy_k_W_h(cruise_speed_m_p_s, reserve_range_km, cruise_electric_power_k_W)

batt_useable_energy_EOL_W_h = calc_batt_useable_energy_EOL_W_h(pack_spec_energy_usable_EOL_Wh_kg, final_batt_mass_kg)
hover_energy_k_W_h = calc_hover_energy_k_W_h(hover_electric_power_k_W, equiv_hover_dur_s)
climb_energy_k_W_h = calc_climb_energy_k_W_h(climb_electric_power_avg_k_W, climb_time_s) # Use final avg power and time
cruise_energy_for_final_range_kwh = cruise_electric_power_k_W * final_cruise_time_EOL_s / 3600.0
total_mission_energy_req_kwh = hover_energy_k_W_h + climb_energy_k_W_h + cruise_energy_for_final_range_kwh
total_reserve_energy_req_kwh = reserve_hover_energy_k_W_h + reserve_cruise_energy_k_W_h
print(f"  Check: Total Required Mission+Reserve EOL Energy: {total_mission_energy_req_kwh + total_reserve_energy_req_kwh:.2f} kWh vs Battery Usable EOL: {batt_useable_energy_EOL_W_h:.2f} kWh")
cruise_range_EOL_english = calc_cruise_range_EOL_english(final_cruise_range_EOL_km)

# Final Weights & Fractions
# ... (Final weights and fractions calculations - using final_max_takeoff_wt_kg) ...
dive_speed_m_p_s = calc_dive_speed_m_p_s(cruise_speed_m_p_s)
hover_shaft_power_k_W = calc_hover_shaft_power_k_W(final_max_takeoff_wt_kg, g_m_p_s2, air_density_sea_level_kg_p_m3, hover_fom, disk_area_m2) # Recalc shaft power
rotor_RPM_hover = calc_rotor_RPM_hover(sound_speed_m_p_s, rotor_diameter_m, tip_mach)
over_torque_factor = calc_over_torque_factor(rotor_count)
motor_torque_hover_Nm = calc_motor_torque_hover_Nm(rotor_count, rotor_RPM_hover, hover_shaft_power_k_W)
motor_RPM_max_thrust_min_rho = calc_motor_RPM_max_thrust_min_rho(rotor_RPM_hover, over_torque_factor, air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3)
motor_torque_max_thrust_Nm = calc_motor_torque_max_thrust_Nm(over_torque_factor, motor_torque_hover_Nm)
motor_mechanical_power_k_W_sizing = calc_motor_mechanical_power_sizing_k_W(motor_RPM_max_thrust_min_rho, motor_torque_max_thrust_Nm)
single_EPU_weight_kg = calc_single_EPU_weight_kg(motor_torque_max_thrust_Nm, motor_mechanical_power_k_W_sizing)
total_EPU_weight_kg = calc_total_EPU_weight_kg(single_EPU_weight_kg, rotor_count)
Ct_hover = calc_Ct_hover(final_max_takeoff_wt_kg, g_m_p_s2, rotor_count, air_density_sea_level_kg_p_m3, rotor_diameter_m, rotor_RPM_hover)
rotor_solidity = calc_rotor_solidity(Ct_hover, rotor_avg_cl)
wing_weight_kg = calc_wing_weight_kg(final_max_takeoff_wt_kg, wing_ref_area_m2, wing_AR, wing_taper_ratio, wing_t_p_c)
horizontal_tail_weight_kg = calculate_horizontal_tail_weight_kg(horizontal_tail_area_m2, dive_speed_m_p_s)
vertical_wing_weight_kg = cal_vertical_wing_weight_kg(vertical_tail_area_m2, dive_speed_m_p_s)
fuselage_weight_kg = calc_fuselage_weight_kg(fuselage_wetted_area_m2, final_max_takeoff_wt_kg, fuselage_l_m, fuselage_fineness_ratio, air_density_sea_level_kg_p_m3, cruise_speed_m_p_s)
boom_weight_kg = calc_boom_weight_kg(single_EPU_weight_kg, rotor_count, rotor_diameter_m, wing_MAC)
landing_gear_weight_kg = calc_landing_gear_weight_kg(final_max_takeoff_wt_kg)
lift_rotor_plus_hub_weight_kg = calc_lift_rotor_plus_hub_weight_kg(rotor_count, rotor_diameter_m, rotor_solidity, sound_speed_m_p_s, tip_mach, air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3, over_torque_factor)
tilt_rotor_weight_kg = calc_tilt_rotor_weight_kg(rotor_count, rotor_diameter_m, rotor_solidity, sound_speed_m_p_s, tip_mach, air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3, over_torque_factor)
structural_weight_kg = calc_structural_weight_kg(wing_weight_kg, horizontal_tail_weight_kg, vertical_wing_weight_kg, fuselage_weight_kg, boom_weight_kg, landing_gear_weight_kg)
empty_weight_less_battery_kg = structural_weight_kg + total_EPU_weight_kg + lift_rotor_plus_hub_weight_kg + tilt_rotor_weight_kg + fixed_systems_weight_kg
margin_kg = calc_margin(empty_weight_less_battery_kg)
empty_weight_kg = empty_weight_less_battery_kg + margin_kg # This IS empty weight (excluding battery)
payload_mass_frac = calc_payload_mass_frac(payload_kg, final_max_takeoff_wt_kg)
batt_mass_frac_final = final_batt_mass_kg / final_max_takeoff_wt_kg if final_max_takeoff_wt_kg > 0 else 0
motor_mass_fraction = calc_motor_mass_fraction(total_EPU_weight_kg, final_max_takeoff_wt_kg)

# Other Final Calcs
# ... (Other final calculations as before) ...
disk_loading_kg_p_m2 = calc_disk_loading_kg_p_m2(final_max_takeoff_wt_kg, disk_area_m2)
disk_loading_english = calc_disk_loading_english(disk_loading_kg_p_m2)
motor_mechanical_power_hover_k_W = calc_motor_mechanical_power_hover_k_W(rotor_count, hover_shaft_power_k_W)
cruise_shaft_power_k_w = calc_cruise_shaft_power_k_W(final_max_takeoff_wt_kg, g_m_p_s2, prop_effic, cruise_speed_m_p_s, cruise_L_p_D) # Recalc shaft power
cruise_max_shaft_power_k_W = calc_cruise_max_shaft_power_k_W(cruise_shaft_power_k_w, air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3, final_max_takeoff_wt_kg, g_m_p_s2, cruise_speed_m_p_s, rotor_count)
Cq_hover = calc_Cq_hover(hover_fom, Ct_hover)
rotor_span = calc_rotor_span(rotor_count, rotor_diameter_m, fuselage_w_m)
wing_loading_kg_p_m2 = calc_wing_loading_kg_p_m2(final_max_takeoff_wt_kg, wing_ref_area_m2)
wing_loading_english = calc_wing_loading_english(wing_loading_kg_p_m2)
emergency_landing_load_factor = calc_emergency_landing_load_factor(stall_speed_m_p_s)
wing_cruise_reynolds = calc_wing_cruise_reynolds(wing_MAC, cruise_speed_m_p_s, kinematic_visc_sea_level_m2_p_s)
booms_CdA_m2 = calc_booms_CdA_m2(booms_Cd0, wing_ref_area_m2) # Use final wing area
wing_stall_reynolds = calc_wing_stall_reynolds(stall_speed_m_p_s, wing_MAC, kinematic_visc_sea_level_m2_p_s)
produc = calc_produc(payload_kg, final_cruise_range_EOL_km, hover_energy_k_W_h, climb_energy_k_W_h, cruise_energy_for_final_range_kwh)
energy_effic = calc_energy_effic(final_cruise_range_EOL_km, hover_energy_k_W_h, climb_energy_k_W_h, cruise_energy_for_final_range_kwh)
miles_of_range_p_min_hov_time = calc_miles_of_range_p_min_hov_time(hover_electric_power_k_W, cruise_electric_power_k_W, cruise_speed_m_p_s)

# --- Generate Power Profile (for Energy Analysis - uses AVG climb power) ---
# This represents the *original* mission profile based on converged results
hover_segment_time_s = equiv_hover_dur_s / 2.0 # Split total hover time
# Use the climb time determined during sizing
climb_time_s_final = final_state.get('climb_time_s', 0)
cruise_time_s_final = final_state.get('cruise_time_s', 0)

# Define segments based on final converged/calculated values
original_power_profile_segments = [
    # [Name, Power (kW), Duration (s), Start Time (s), End Time (s)]
    ['Hover_Takeoff', hover_electric_power_k_W, hover_segment_time_s, 0.0, hover_segment_time_s],
    ['Climb', climb_electric_power_avg_k_W, climb_time_s_final, hover_segment_time_s, hover_segment_time_s + climb_time_s_final],
    ['Cruise', cruise_electric_power_k_W, cruise_time_s_final, hover_segment_time_s + climb_time_s_final, hover_segment_time_s + climb_time_s_final + cruise_time_s_final],
    ['Hover_Land', hover_electric_power_k_W, hover_segment_time_s, hover_segment_time_s + climb_time_s_final + cruise_time_s_final, hover_segment_time_s + climb_time_s_final + cruise_time_s_final + hover_segment_time_s]
]
# Calculate total time for validation later
total_mission_time_s_orig = original_power_profile_segments[-1][4]

print("\nOriginal Calculated Power Profile (for Energy Analysis - uses AVG climb power):")
print(tabulate([[s[0], s[1], s[2]] for s in original_power_profile_segments], headers=['Segment', 'Power (kW)', 'Time (s)'], tablefmt='grid', floatfmt=".2f"))


# --- Power Profile Analysis ---
print("\n--- Battery Pack & Mission Analysis (Original Mission) ---")
profile_energies_kwh = [calc_energy_kwh(p, t) for name, p, t, ts, te in original_power_profile_segments]
trip_energy_kwh = calc_trip_energy_kwh(profile_energies_kwh)
print(f"Trip Energy (Original Profile, excl. reserves): {trip_energy_kwh:.3f} kWh")
# Battery Cell Configuration
if cell_mass_kg <= 0 or cell_to_pack_mass_ratio <=0 or final_batt_mass_kg <=0:
    print("ERROR: Invalid cell mass, pack ratio, or final battery mass for cell calculation.")
    cells_series, cells_parallel, cell_count = 0, 0, 0
    battery_total_energy_kwh, battery_total_charge_ah = 0, 0
else:
    total_cell_mass_kg = final_batt_mass_kg * cell_to_pack_mass_ratio
    cell_count_f = total_cell_mass_kg / cell_mass_kg if cell_mass_kg > 0 else 0
    cell_count = int(cell_count_f) if cell_count_f > 0 else 0
    cells_series = calc_cells_series(system_voltage, cell_voltage)
    cells_parallel = cell_count // cells_series if cells_series > 0 else 0
    # Recalculate cell count based on integer S and P config for consistency
    cell_count = calc_cell_count(cells_series, cells_parallel)
    # Calculate BOL energy/charge based on the *actual* number of cells that fit
    battery_total_energy_kwh = calc_total_battery_energy_kwh(cells_series, cells_parallel, cell_voltage, cell_charge_ah)
    battery_total_charge_ah = calc_battery_total_charge_ah(battery_total_energy_kwh, system_voltage)

print(f"Cells in Series: {cells_series}")
print(f"Cells in Parallel: {cells_parallel}")
print(f"Total Cell Count: {cell_count}")
print(f"Calculated Total Pack Gross Energy (BOL): {battery_total_energy_kwh:.3f} kWh")
print(f"Calculated Total Pack Charge Capacity (BOL): {battery_total_charge_ah:.3f} Ah")
# DOD and Remaining Energy
if battery_total_energy_kwh <= 0:
    final_DOD_percent = float('inf')
    remaining_energy_kwh = -trip_energy_kwh
else:
    final_DOD_percent = trip_energy_kwh / battery_total_energy_kwh * 100.0
    remaining_energy_kwh = battery_total_energy_kwh - trip_energy_kwh
print(f"Final Depth of Discharge (DOD) for mission: {final_DOD_percent:.2f} %")
print(f"Remaining Gross Energy after mission (BOL): {remaining_energy_kwh:.3f} kWh")
# Recharge Time
recharge_time_minutes = calc_recharge_time_min(battery_total_energy_kwh, final_DOD_percent, charger_power_kw)
print(f"Estimated Recharge Time (@{charger_power_kw} kW): {recharge_time_minutes:.2f} min")
# Per-segment analysis
print("\nPower Profile Segment Analysis (Using AVG Climb Power for Energy):")
segment_analysis_data = []
header = ['Segment', 'Avg Pwr (kW)', 'Time (s)', 'Energy (kWh)', 'Charge (Ah)', 'Avg Curr (A)', 'C-Rate']
for segment_name, segment_power, segment_time, ts, te in original_power_profile_segments:
    energy_kwh = calc_energy_kwh(segment_power, segment_time)
    charge_ah = calc_charge_ah(segment_power, segment_time, system_voltage)
    avg_current_a = calc_avg_current_a(segment_power, system_voltage)
    c_rate = calc_c_rate(avg_current_a, battery_total_charge_ah)
    segment_analysis_data.append([segment_name, segment_power, segment_time, energy_kwh, charge_ah, avg_current_a, c_rate])
print(tabulate(segment_analysis_data, headers=header, tablefmt='grid', floatfmt=".3f"))

# Recalculate C-Rates
batt_rated_energy_k_W_h_BOL = battery_total_energy_kwh # Use BOL energy for C-rate calc base
batt_C_rate_hover = calc_batt_C_rate_hover(batt_rated_energy_k_W_h_BOL, hover_electric_power_k_W)
batt_C_rate_climb = calc_batt_C_rate_climb(batt_rated_energy_k_W_h_BOL, climb_electric_power_avg_k_W)
batt_C_rate_cruise = calc_batt_C_rate_cruise(batt_rated_energy_k_W_h_BOL, cruise_electric_power_k_W)


# --- Plotting ---
print("\nGenerating Plots...")

try:
    plt.figure(figsize=(10, 5))
    plot_x_vals = [0]; plot_y_vals = [0] # Start at time 0, power 0
    current_time = 0

    # Hover Takeoff
    power_hover = hover_electric_power_k_W
    time_hover_seg = hover_segment_time_s
    plot_x_vals.extend([current_time, current_time + time_hover_seg])
    plot_y_vals.extend([power_hover, power_hover])
    current_time += time_hover_seg
    t_hover_end = current_time

    #CHANGE power climb end start mid
    # Climb (Three Steps Viz: Start -> Mid -> End Power)
    time_climb_total = climb_time_s_final
    time_climb_third = time_climb_total / 3.0 if time_climb_total > 0 else 0
    power_climb_mid = climb_power_start_kw # Use specific power from final calc
    power_climb_end = climb_power_mid_kw     # Use specific power from final calc
    power_climb_start = climb_power_end_kw     # Use specific power from final calc

    if time_climb_total > 0:
        # Step 1: Low Speed Climb Power (Transition complete)
        plot_x_vals.extend([current_time, current_time + time_climb_third])
        plot_y_vals.extend([power_climb_start, power_climb_start])
        current_time += time_climb_third

        # Step 2: Mid Speed Climb Power
        plot_x_vals.extend([current_time, current_time + time_climb_third])
        plot_y_vals.extend([power_climb_mid, power_climb_mid])
        current_time += time_climb_third

        # Step 3: High Speed Climb Power
        plot_x_vals.extend([current_time, current_time + time_climb_third])
        plot_y_vals.extend([power_climb_end, power_climb_end])
        # Ensure current_time aligns exactly with calculated end time
        current_time = t_hover_end + time_climb_total
    else:
        # If climb time is zero, just update current_time
         current_time = t_hover_end

    t_climb_end_plot = current_time

    # Cruise
    power_cruise = cruise_electric_power_k_W
    time_cruise = cruise_time_s_final
    plot_x_vals.extend([current_time, current_time + time_cruise])
    plot_y_vals.extend([power_cruise, power_cruise])
    current_time += time_cruise
    t_cruise_end_plot = current_time

    # Hover Land
    power_hover_land = hover_electric_power_k_W # Same as takeoff hover power
    time_hover_land = hover_segment_time_s
    plot_x_vals.extend([current_time, current_time + time_hover_land])
    plot_y_vals.extend([power_hover_land, power_hover_land])
    current_time += time_hover_land

    # End at zero
    plot_x_vals.append(current_time); plot_y_vals.append(0)

    plt.step(plot_x_vals, plot_y_vals, where='post', linewidth=2)
    plt.xlabel('Time (sec)'); plt.ylabel('Calculated Power (kW)')
    plt.title('Original Power Profile Visualization (3-Step Climb Approx.)')
    plt.grid(True); plt.ylim(bottom=0); plt.tight_layout(); plt.show(block=False) # Use block=False

except Exception as e:
    print(f"Could not generate Original Power Profile plot: {e}")


# 2. Mission Altitude/Velocity Plot (No Descent Segment)
# ... (Altitude/Velocity plot logic remains the same, uses final times) ...
try:
    t_hover_to = hover_segment_time_s
    t_climb_end = t_hover_to + climb_time_s_final # Use final climb time
    t_cruise_end = t_climb_end + cruise_time_s_final # Use final cruise time
    t_hover_land_start = t_cruise_end
    t_hover_land_end = t_hover_land_start + hover_segment_time_s
    total_time_plot = t_hover_land_end

    time_alt = np.linspace(0, total_time_plot, 500)
    altitude_profile = np.zeros_like(time_alt)

    # Define masks based on segment end times
    mask_hover_to = (time_alt >= 0) & (time_alt < t_hover_to)
    altitude_profile[mask_hover_to] = plot_hover_alt_ft

    mask_climb = (time_alt >= t_hover_to) & (time_alt < t_climb_end)
    # Handle zero climb time case
    if np.sum(mask_climb) > 0 and t_climb_end > t_hover_to:
         altitude_profile[mask_climb] = np.linspace(plot_hover_alt_ft, plot_cruise_alt_ft, np.sum(mask_climb))
    elif t_climb_end == t_hover_to: # If no climb, altitude jumps at t_hover_to if cruise alt > hover alt
         if plot_cruise_alt_ft > plot_hover_alt_ft:
             # Find index near t_hover_to and set altitude
             idx_climb_start = np.searchsorted(time_alt, t_hover_to)
             if idx_climb_start < len(altitude_profile):
                 altitude_profile[idx_climb_start:] = plot_cruise_alt_ft # Assume instant jump for plot if no climb duration


    mask_cruise = (time_alt >= t_climb_end) & (time_alt < t_cruise_end)
    altitude_profile[mask_cruise] = plot_cruise_alt_ft

    mask_hover_land = (time_alt >= t_hover_land_start) & (time_alt <= t_hover_land_end)
    # Need to handle the step down carefully if previous segment ended at cruise alt
    idx_land_start = np.searchsorted(time_alt, t_hover_land_start)
    if idx_land_start > 0: # Ensure index is valid
         altitude_profile[idx_land_start:] = plot_hover_alt_ft # Set rest to hover alt
         # Correct the value just before landing starts if needed
         altitude_profile[idx_land_start-1] = plot_cruise_alt_ft


    # Prepare for plotting steps
    time_alt_plot = np.array([0, t_hover_to, t_hover_to, t_climb_end, t_climb_end, t_cruise_end, t_cruise_end, t_hover_land_end, t_hover_land_end])
    altitude_profile_plot = np.array([0, plot_hover_alt_ft, plot_hover_alt_ft, plot_cruise_alt_ft, plot_cruise_alt_ft, plot_cruise_alt_ft, plot_hover_alt_ft, plot_hover_alt_ft, 0])

    # Remove duplicates for plotting if times are identical (e.g., zero duration climb)
    unique_indices = np.where(np.diff(time_alt_plot) > 1e-9)[0] # Use tolerance for float comparison
    time_alt_plot = np.concatenate(([time_alt_plot[0]], time_alt_plot[unique_indices + 1]))
    altitude_profile_plot = np.concatenate(([altitude_profile_plot[0]], altitude_profile_plot[unique_indices + 1]))


    # Velocity Profile (Simpler Step Plot)
    # Use Climb End Speed for visualization simplicity during cruise
    v_cruise_plot = V_climb_end_mps # Or cruise_speed_m_p_s? Let's use cruise_speed_m_p_s
    time_vel_steps =    [0,          t_hover_to, t_hover_to, t_climb_end,  t_cruise_end, t_cruise_end,       t_hover_land_end, t_hover_land_end]
    vel_profile_steps = [0,          0,          V_climb_start_mps, v_cruise_plot, v_cruise_plot, 0,               0,                0] # Assumes start climb speed, step to cruise, step down to zero at land

    plt.figure(figsize=(10, 8))
    ax1 = plt.subplot(2, 1, 1)
    ax1.plot(time_alt_plot, altitude_profile_plot, marker='.', linestyle='-', linewidth=1) # Use step plot for altitude too
    #ax1.step(time_alt_plot, altitude_profile_plot, where='post', linewidth=2)
    ax1.set_xlabel('Time (sec)'); ax1.set_ylabel('Altitude (ft)'); ax1.set_title('Mission Altitude Profile (No Descent)')
    ax1.set_ylim(bottom=0); ax1.set_xlim(left=0, right=total_time_plot); ax1.grid(True)

    ax2 = plt.subplot(2, 1, 2)
    ax2.step(time_vel_steps, vel_profile_steps, where='post', linewidth=2)
    ax2.set_xlabel('Time (sec)'); ax2.set_ylabel('Horizontal Velocity (m/s)'); ax2.set_title('Mission Horizontal Velocity Profile (No Descent)')
    ax2.set_ylim(bottom=0); ax2.set_xlim(left=0, right=total_time_plot); ax2.grid(True)

    plt.tight_layout(); plt.show(block=False) # Use block=False

except Exception as e:
    print(f"Could not generate Mission Profile plot: {e}")
    # Optionally print traceback
    # import traceback
    # traceback.print_exc()


# --- Output Results ---
# (Output table generation - unchanged from previous version, uses final calculated values)
output_vars = [
    ["Scenario", "Fixed Battery" if fixed_battery_pack_mass_kg > 0 else "Iterative Sizing"],
    ["Final MTOW (kg)", final_max_takeoff_wt_kg], ["Payload (kg)", payload_kg],
    ["Final Battery Mass (kg)", final_batt_mass_kg],
    ["Achieved/Target EOL Cruise Range (km)", final_cruise_range_EOL_km],
    ["Achieved/Target EOL Cruise Range (miles)", cruise_range_EOL_english],
    ["Achieved EOL Cruise Time (s)", final_cruise_time_EOL_s],
    ["Payload Mass Fraction", payload_mass_frac], ["Battery Mass Fraction", batt_mass_frac_final],
    ["Motor/EPU Mass Fraction", motor_mass_fraction], ["Empty Weight (kg, excl. batt)", empty_weight_kg],
    ["Cruise L/D", cruise_L_p_D], ["Hover Electric Power (kW)", hover_electric_power_k_W],
    ["Climb Electric Power (Avg, kW)", climb_electric_power_avg_k_W], # Report Avg
    ["Cruise Electric Power (kW)", cruise_electric_power_k_W],
    ["Productivity (payload*range/energy)", produc], ["Energy Efficiency (range/energy)", energy_effic],
    ["Range tradeoff (miles range / min hover)", miles_of_range_p_min_hov_time],
    ["Climb Time (s)", climb_time_s_final], ["Hover Energy (kWh)", hover_energy_k_W_h],
    ["Climb Energy (kWh)", climb_energy_k_W_h], ["Cruise Energy (kWh)", cruise_energy_for_final_range_kwh],
    ["Pack Usable EOL Spec Energy (Wh/kg)", pack_spec_energy_usable_EOL_Wh_kg],
    ["Pack Gross BOL Spec Energy (Wh/kg)", pack_spec_energy_gross_BOL_Wh_kg],
    ["Pack Total Gross Energy (BOL, kWh)", battery_total_energy_kwh],
    ["Pack Total Charge Capacity (BOL, Ah)", battery_total_charge_ah],
    ["System Voltage (V)", system_voltage], ["Cells in Series", cells_series],
    ["Cells in Parallel", cells_parallel], ["Total Cell Count", cell_count],
    ["Trip Energy (kWh, excl. reserves)", trip_energy_kwh], ["Final DOD (%)", final_DOD_percent],
    ["Remaining Gross Energy (BOL, kWh)", remaining_energy_kwh], ["Recharge Time (min)", recharge_time_minutes],
    ["Hover C-Rate (BOL)", batt_C_rate_hover], ["Climb C-Rate (Avg, BOL)", batt_C_rate_climb], # Report Avg
    ["Cruise C-Rate (BOL)", batt_C_rate_cruise],
    ["Wing Area (m^2)", wing_ref_area_m2], ["Wing Aspect Ratio", wing_AR],
    ["Wing Loading (kg/m^2)", wing_loading_kg_p_m2], ["Wing Loading (psf)", wing_loading_english],
    ["Disk Area (m^2)", disk_area_m2], ["Disk Loading (kg/m^2)", disk_loading_kg_p_m2],
    ["Disk Loading (psf)", disk_loading_english],
    ["Wing Weight (kg)", wing_weight_kg], ["H-Tail Weight (kg)", horizontal_tail_weight_kg],
    ["V-Tail Weight (kg)", vertical_wing_weight_kg], ["Fuselage Weight (kg)", fuselage_weight_kg],
    ["Boom Weight (kg)", boom_weight_kg], ["Landing Gear Weight (kg)", landing_gear_weight_kg],
    ["Total EPU Weight (kg)", total_EPU_weight_kg], ["Lift Rotor+Hub Weight (kg)", lift_rotor_plus_hub_weight_kg],
    ["Tilt Rotor+Hub Weight (kg)", tilt_rotor_weight_kg], ["Fixed Systems Weight (kg)", fixed_systems_weight_kg],
    ["Margin Weight (kg)", margin_kg],
    ["Rotor RPM (hover)", rotor_RPM_hover], ["Motor Torque (hover, Nm)", motor_torque_hover_Nm],
    ["Motor Torque (max thrust, Nm)", motor_torque_max_thrust_Nm],
    ["Motor Power (sizing, kW)", motor_mechanical_power_k_W_sizing],
    ["Rotor Solidity", rotor_solidity], ["Rotor Ct (hover)", Ct_hover], ["Rotor Cq (hover)", Cq_hover]
]
print("\n=== Integrated eVTOL Sizing & Analysis Results (Original Mission) ===\n")
print(tabulate(output_vars, headers=["Parameter", "Value"], tablefmt="grid", floatfmt=".3f", disable_numparse=[0]))


# --- NEW: Jettison Analysis ---

# --- Global variable to store post-jettison powers ---
post_jettison_powers = {} # Used by single jettison analysis AND optimization

# --- Jettison Analysis Function ---
# (run_jettison_analysis remains the same as the previous version - it calculates
# powers for a SINGLE scenario and stores them in post_jettison_powers)
def run_jettison_analysis(params, final_state, original_profile_segments, total_mission_time_s_orig):
    # ... (Function definition as in previous response - calculates powers for
    #      params['jettison_mass_kg'] and params['jettison_time_s'] and
    #      stores in global post_jettison_powers dict. Returns True/False) ...
    global post_jettison_powers # Declare intent to modify global variable
    post_jettison_powers = {} # Clear previous results if any

    print("\n\n=== Battery Jettison Analysis ===")
    # Get jettison parameters from main scope (loaded from JSON)
    jettison_time_s = params.get('jettison_time_s', 0.0)
    jettison_mass_kg = params.get('jettison_mass_kg', 0.0)

    # --- Validation ---
    if jettison_time_s < 0:
        print("WARNING: jettison_time_s is negative. Skipping jettison analysis.")
        return False # Indicate failure/skip
    if jettison_time_s >= total_mission_time_s_orig:
         print(f"WARNING: jettison_time_s ({jettison_time_s:.1f}s) is >= total mission time ({total_mission_time_s_orig:.1f}s). Skipping jettison analysis.")
         return False # Indicate failure/skip
    if jettison_mass_kg <= 0:
        print("WARNING: jettison_mass_kg must be positive. Skipping jettison analysis.")
        return False # Indicate failure/skip

    final_batt_mass_kg = final_state.get('batt_mass_kg', 0)
    if final_batt_mass_kg <= 0:
        print("WARNING: Final battery mass is zero or negative. Skipping jettison analysis.")
        return False # Indicate failure/skip

    jettison_mass_kg_actual = jettison_mass_kg
    if jettison_mass_kg > final_batt_mass_kg:
        print(f"WARNING: jettison_mass_kg ({jettison_mass_kg:.2f} kg) exceeds final battery mass ({final_batt_mass_kg:.2f} kg). Clamping to final battery mass.")
        jettison_mass_kg_actual = final_batt_mass_kg

    print(f"Simulating jettison of {jettison_mass_kg_actual:.2f} kg of battery at {jettison_time_s:.1f} seconds.")

    # --- Calculate Post-Jettison State ---
    mtow_orig_kg = final_state.get('mtow_kg', 0)
    mtow_post_jettison_kg = mtow_orig_kg - jettison_mass_kg_actual
    print(f"Original MTOW: {mtow_orig_kg:.2f} kg -> Post-Jettison MTOW: {mtow_post_jettison_kg:.2f} kg")

    if mtow_post_jettison_kg <= 0:
         print("ERROR: Post-jettison MTOW is non-positive. Cannot proceed.")
         return False # Indicate failure/skip

    # Extract necessary parameters from final_state and params
    g_m_p_s2 = params['g_m_p_s2']
    air_density_sea_level_kg_p_m3 = params['air_density_sea_level_kg_p_m3']
    hover_fom = params['hover_fom']
    disk_area_m2 = final_state.get('disk_area_m2', 0)
    epu_effic = params['epu_effic']
    prop_effic = params['prop_effic']
    cruise_speed_m_p_s = params['cruise_speed_m_p_s']
    base_cd0 = final_state.get('base_cd0', 0)
    wing_AR = final_state.get('wing_AR', 0)
    wing_ref_area_m2 = final_state.get('wing_ref_area_m2', 0)
    rate_of_climb_mps = params.get('rate_of_climb_mps', default_roc_mps) # Use param directly
    spac_effic_factor = params['spac_effic_factor']
    trim_drag_factor = params['trim_drag_factor']
    V_climb_start_mps = params.get('V_climb_start_mps', 20.0)
    V_climb_end_mps = params['cruise_speed_m_p_s']
    V_climb_mid_mps = (V_climb_start_mps + V_climb_end_mps) / 2.0

    # --- Recalculate Powers ---
    hover_shaft_power_post_j = calc_hover_shaft_power_k_W(mtow_post_jettison_kg, g_m_p_s2, air_density_sea_level_kg_p_m3, hover_fom, disk_area_m2)
    hover_power_post_j = calc_hover_electric_power_k_W(epu_effic, hover_shaft_power_post_j)
    cruise_shaft_power_post_j = calc_cruise_shaft_power_k_W(mtow_post_jettison_kg, g_m_p_s2, prop_effic, cruise_speed_m_p_s, cruise_L_p_D)
    cruise_power_post_j = calc_cruise_electric_power_k_W(cruise_shaft_power_post_j, epu_effic)
    climb_power_start_post_j = calc_power_at_speed_climbing_k_W(mtow_post_jettison_kg, V_climb_start_mps, base_cd0, wing_AR, wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
    climb_power_mid_post_j = calc_power_at_speed_climbing_k_W(mtow_post_jettison_kg, V_climb_mid_mps, base_cd0, wing_AR, wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
    climb_power_end_post_j = calc_power_at_speed_climbing_k_W(mtow_post_jettison_kg, V_climb_end_mps, base_cd0, wing_AR, wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
    climb_power_avg_post_j = (climb_power_start_post_j + climb_power_mid_post_j + climb_power_end_post_j) / 3.0

    print(f"Recalculated Post-Jettison Powers (Hover: {hover_power_post_j:.2f} kW, Climb Avg: {climb_power_avg_post_j:.2f} kW, Cruise: {cruise_power_post_j:.2f} kW)")

    # --- Store Post-Jettison Powers Globally ---
    post_jettison_powers = {
        'hover': hover_power_post_j,
        'climb_avg': climb_power_avg_post_j,
        'cruise': cruise_power_post_j,
        'climb_start': climb_power_start_post_j,
        'climb_mid': climb_power_mid_post_j,
        'climb_end': climb_power_end_post_j,
        'valid': True, # Flag indicating powers were calculated
        'jettison_mass_kg_actual': jettison_mass_kg_actual # Store the actual mass dropped
    }

    # --- Construct Jettison Profile ---
    jettison_profile = []
    current_time = 0.0
    epsilon = 1e-9 # Tolerance for time comparisons
    climb_pts_orig = [final_state['climb_power_start_kw'], final_state['climb_power_mid_kw'], final_state['climb_power_end_kw']]

    for segment_name, P_orig, T_orig, t_start_orig, t_end_orig in original_profile_segments:
        P_post_j = 0
        P_climb_pts_post = [0,0,0]
        if "Hover" in segment_name: P_post_j = post_jettison_powers['hover']
        elif "Climb" in segment_name:
             P_post_j = post_jettison_powers['climb_avg']
             P_climb_pts_post = [post_jettison_powers['climb_start'], post_jettison_powers['climb_mid'], post_jettison_powers['climb_end']]
        elif "Cruise" in segment_name: P_post_j = post_jettison_powers['cruise']
        else: P_post_j = P_orig

        if jettison_time_s <= t_start_orig + epsilon:
             jettison_profile.append([segment_name, P_post_j, T_orig, t_start_orig, t_end_orig, P_climb_pts_post])
        elif jettison_time_s >= t_end_orig - epsilon:
             jettison_profile.append([segment_name, P_orig, T_orig, t_start_orig, t_end_orig, climb_pts_orig])
        else:
             t_pre = jettison_time_s - t_start_orig
             jettison_profile.append([f"{segment_name} (Pre-Jettison)", P_orig, t_pre, t_start_orig, jettison_time_s, climb_pts_orig])
             t_post = t_end_orig - jettison_time_s
             jettison_profile.append([f"{segment_name} (Post-Jettison)", P_post_j, t_post, jettison_time_s, t_end_orig, P_climb_pts_post])

    # --- Calculate Jettison Energy ---
    jettison_energies_kwh = [calc_energy_kwh(p, t) for name, p, t, ts, te, pts in jettison_profile]
    jettison_trip_energy_kwh = sum(jettison_energies_kwh)

    # --- Generate Comparison Table ---
    print("\n--- Power Profile Comparison (Original vs. Jettison) ---")
    # ... (Comparison table code remains the same) ...
    # (Code as in previous response)

    # --- Generate Jettison Plot ---
    # ... (Jettison plot code remains the same) ...
    # (Code as in previous response)

    return True # Indicate success


# --- Shorter Range Analysis Function ---
# (run_shorter_range_analysis remains the same as the previous version - it analyzes
# two scenarios for a fixed shorter range, one with and one without the *single*
# jettison defined in the params. It also calculates component DODs.)
def run_shorter_range_analysis(params, final_state, post_jettison_powers, battery_total_energy_kwh, pack_spec_energy_gross_BOL_Wh_kg):
    # ... (Function definition as in previous response) ...
    # (Code as in previous response)
    pass # Placeholder - keep the implementation from the previous step

# --- NEW: Jettison Optimization Function ---

def run_jettison_optimization(params, final_state, battery_total_energy_kwh, pack_spec_energy_gross_BOL_Wh_kg):
    """
    Optimizes jettison mass and time for the shorter mission profile
    to minimize energy consumption subject to DOD constraints.
    """
    print("\n\n=== Jettison Optimization for Shorter Range Mission ===")

    # --- Prerequisites Check ---
    if not jettison_opt_enable:
        print("Jettison optimization not enabled in JSON. Skipping.")
        return
    if not final_state:
        print("ERROR: Final vehicle state not available. Cannot run optimization.")
        return
    if not enable_shorter_range_analysis or shorter_range_target_km <= 0:
        print("ERROR: Shorter range analysis must be enabled and target range positive for optimization.")
        return
    if battery_total_energy_kwh <= 0 or pack_spec_energy_gross_BOL_Wh_kg <= 0:
        print("ERROR: Battery total energy or spec energy is invalid. Cannot run optimization.")
        return

    # --- Extract Parameters ---
    # Optimization Params
    mass_max_config_kg = params.get('jettison_opt_mass_max_kg', 95.0)
    mass_step_kg = params.get('jettison_opt_mass_step_kg', 5.0)
    time_step_s = params.get('jettison_opt_time_step_s', 10.0)
    max_dod_dropped_config = params.get('jettison_opt_max_dod_dropped', 100.0)
    max_dod_remaining_config = params.get('jettison_opt_max_dod_remaining', 100.0)
    # Vehicle/Mission Params
    final_batt_mass_kg = final_state.get('batt_mass_kg', 0)
    hover_time_segment_s = final_state['hover_time_s'] / 2.0
    climb_time_s = final_state['climb_time_s']
    cruise_speed_m_p_s = params['cruise_speed_m_p_s']
    hover_power_orig_kw = final_state['hover_power_kw']
    climb_power_avg_orig_kw = final_state['climb_power_avg_kw']
    climb_power_pts_orig = [final_state['climb_power_start_kw'], final_state['climb_power_mid_kw'], final_state['climb_power_end_kw']]
    cruise_power_orig_kw = final_state['cruise_power_kw']
    mtow_orig_kg = final_state.get('mtow_kg', 0)
    # Post-jettison calculation params
    g_m_p_s2 = params['g_m_p_s2']
    air_density_sea_level_kg_p_m3 = params['air_density_sea_level_kg_p_m3']
    hover_fom = params['hover_fom']
    disk_area_m2 = final_state.get('disk_area_m2', 0)
    epu_effic = params['epu_effic']
    prop_effic = params['prop_effic']
    base_cd0 = final_state.get('base_cd0', 0)
    wing_AR = final_state.get('wing_AR', 0)
    wing_ref_area_m2 = final_state.get('wing_ref_area_m2', 0)
    rate_of_climb_mps = params.get('rate_of_climb_mps', default_roc_mps)
    spac_effic_factor = params['spac_effic_factor']
    trim_drag_factor = params['trim_drag_factor']
    V_climb_start_mps = params.get('V_climb_start_mps', 20.0)
    V_climb_end_mps = params['cruise_speed_m_p_s']
    V_climb_mid_mps = (V_climb_start_mps + V_climb_end_mps) / 2.0

    # --- Calculate Shorter Mission Timing & Baseline ---
    if cruise_speed_m_p_s <= 0:
        print("ERROR: Cruise speed is zero or negative. Cannot optimize.")
        return
    shorter_cruise_time_s = (shorter_range_target_km * 1000.0) / cruise_speed_m_p_s
    t_hover_end = hover_time_segment_s
    t_climb_end = t_hover_end + climb_time_s
    t_cruise_end_short = t_climb_end + shorter_cruise_time_s
    t_land_start_short = t_cruise_end_short
    t_land_end_short = t_land_start_short + hover_time_segment_s

    # Baseline Profile (Shorter, No Jettison)
    short_no_jettison_profile_opt = [
        ['Hover_Takeoff', hover_power_orig_kw, hover_time_segment_s, 0.0, t_hover_end, [0,0,0]],
        ['Climb', climb_power_avg_orig_kw, climb_time_s, t_hover_end, t_climb_end, climb_power_pts_orig],
        ['Cruise', cruise_power_orig_kw, shorter_cruise_time_s, t_climb_end, t_cruise_end_short, [0,0,0]],
        ['Hover_Land', hover_power_orig_kw, hover_time_segment_s, t_land_start_short, t_land_end_short, [0,0,0]]
    ]
    min_energy_found = sum(calc_energy_kwh(p, t) for name, p, t, ts, te, pts in short_no_jettison_profile_opt)
    print(f"Baseline energy for shorter range ({shorter_range_target_km:.1f} km) with NO jettison: {min_energy_found:.3f} kWh")

    # --- Initialize Optimization Variables ---
    optimal_mass = 0.0
    optimal_time = 0.0
    optimal_energy = min_energy_found
    optimal_dod_dropped = 0.0 # DOD at drop time for baseline is 0 if time is 0
    optimal_dod_remaining = (min_energy_found / battery_total_energy_kwh * 100.0) if battery_total_energy_kwh > 0 else float('inf') # DOD remaining for baseline is overall DOD

    # --- Setup Grid Search ---
    mass_max_check = min(mass_max_config_kg, final_batt_mass_kg)
    # Ensure mass_max_check is positive before creating range
    if mass_max_check <= 0:
        print("WARNING: Maximum jettison mass to check is zero or negative. Only baseline (no jettison) will be considered.")
        mass_values_kg = [0.0] # Only check the baseline
    else:
        # Include 0 mass (baseline) and go up to mass_max_check
        mass_values_kg = np.arange(0, mass_max_check + mass_step_kg / 2.0, mass_step_kg) # Add half step to include endpoint if close
        mass_values_kg = np.clip(mass_values_kg, 0, mass_max_check) # Ensure we don't exceed max
        mass_values_kg = np.unique(mass_values_kg) # Remove duplicates

    # Time range: 0 to end of shorter cruise segment
    time_max_check = t_cruise_end_short
    if time_max_check < 0: time_max_check = 0 # Handle edge case
    time_values_s = np.arange(45, time_max_check + time_step_s / 2.0, time_step_s) # MIN DROP TIME
    time_values_s = np.clip(time_values_s, 0, time_max_check)
    time_values_s = np.unique(time_values_s)

    num_simulations = len(mass_values_kg) * len(time_values_s)
    print(f"Starting jettison optimization grid search: ~{num_simulations} simulations...")
    print(f"  Mass range: {mass_values_kg.min():.1f} kg to {mass_values_kg.max():.1f} kg ({len(mass_values_kg)} steps)")
    print(f"  Time range: {time_values_s.min():.1f} s to {time_values_s.max():.1f} s ({len(time_values_s)} steps)")
    print(f"  Constraints: DOD Dropped <= {max_dod_dropped_config:.1f}%, DOD Remaining <= {max_dod_remaining_config:.1f}%")

    # --- Grid Search Loop ---
    sim_count = 0
    feasible_points_found = 0
    progress_interval = max(1, num_simulations // 20) # Update progress roughly 20 times

    # Pre-define the structure of the shorter mission before jettison logic
    base_short_profile_info_opt = [
        # Name, P_orig, T_max, t_start, t_end, ClimbPts_orig
        ['Hover_Takeoff', hover_power_orig_kw, hover_time_segment_s, 0.0, t_hover_end, [0,0,0]],
        ['Climb', climb_power_avg_orig_kw, climb_time_s, t_hover_end, t_climb_end, climb_power_pts_orig],
        ['Cruise', cruise_power_orig_kw, shorter_cruise_time_s, t_climb_end, t_cruise_end_short, [0,0,0]],
        ['Hover_Land', hover_power_orig_kw, hover_time_segment_s, t_land_start_short, t_land_end_short, [0,0,0]]
    ]

    # Dictionary to cache post-jettison powers for each mass
    cached_post_jettison_powers = {}

    for m_j in mass_values_kg:
        # --- Pre-calculate Post-Jettison Powers for this mass ---
        if m_j > 1e-6: # Don't calculate for zero mass (use original powers)
            mtow_post_jettison_kg = mtow_orig_kg - m_j
            if mtow_post_jettison_kg <= 0:
                 print(f"Skipping mass {m_j:.1f} kg and above (non-positive MTOW).")
                 break # No point checking higher masses

            hover_shaft_post_j = calc_hover_shaft_power_k_W(mtow_post_jettison_kg, g_m_p_s2, air_density_sea_level_kg_p_m3, hover_fom, disk_area_m2)
            h_pow_post_j = calc_hover_electric_power_k_W(epu_effic, hover_shaft_post_j)
            cruise_shaft_post_j = calc_cruise_shaft_power_k_W(mtow_post_jettison_kg, g_m_p_s2, prop_effic, cruise_speed_m_p_s, cruise_L_p_D)
            cr_pow_post_j = calc_cruise_electric_power_k_W(cruise_shaft_post_j, epu_effic)
            cl_pwr_s_post_j = calc_power_at_speed_climbing_k_W(mtow_post_jettison_kg, V_climb_start_mps, base_cd0, wing_AR, wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
            cl_pwr_m_post_j = calc_power_at_speed_climbing_k_W(mtow_post_jettison_kg, V_climb_mid_mps, base_cd0, wing_AR, wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
            cl_pwr_e_post_j = calc_power_at_speed_climbing_k_W(mtow_post_jettison_kg, V_climb_end_mps, base_cd0, wing_AR, wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
            cl_pwr_avg_post_j = (cl_pwr_s_post_j + cl_pwr_m_post_j + cl_pwr_e_post_j) / 3.0
            current_mass_post_powers = {
                'hover': h_pow_post_j, 'climb_avg': cl_pwr_avg_post_j, 'cruise': cr_pow_post_j,
                'climb_start': cl_pwr_s_post_j, 'climb_mid': cl_pwr_m_post_j, 'climb_end': cl_pwr_e_post_j
            }
        else: # m_j is essentially zero
             current_mass_post_powers = {} # Indicate no change needed


        for t_j in time_values_s:
            sim_count += 1
            if sim_count % progress_interval == 0:
                print(f"  Progress: {sim_count / num_simulations * 100:.0f}% ({sim_count}/{num_simulations})")

            # Handle the baseline case separately (or ensure loop logic does)
            if m_j < 1e-6 and t_j < 1e-6:
                 # This is the baseline no-jettison case, already evaluated
                 continue

            # --- Simulate this (m_j, t_j) point ---
            profile_sim = []
            epsilon = 1e-9

            for segment_name, P_orig, T_seg_max, t_start_seg, t_end_seg, P_climb_pts_orig_seg in base_short_profile_info_opt:

                # Get post-jettison powers if m_j > 0
                P_post_j = P_orig # Default to original power
                P_climb_pts_post = P_climb_pts_orig_seg # Default
                if m_j > 1e-6:
                    if "Hover" in segment_name: P_post_j = current_mass_post_powers['hover']
                    elif "Climb" in segment_name:
                        P_post_j = current_mass_post_powers['climb_avg']
                        P_climb_pts_post = [current_mass_post_powers['climb_start'], current_mass_post_powers['climb_mid'], current_mass_post_powers['climb_end']]
                    elif "Cruise" in segment_name: P_post_j = current_mass_post_powers['cruise']
                    # else: stays P_orig

                # Apply jettison logic
                if t_j <= t_start_seg + epsilon: # Jettison before segment
                    profile_sim.append([segment_name, P_post_j, T_seg_max, t_start_seg, t_end_seg, P_climb_pts_post])
                elif t_j >= t_end_seg - epsilon: # Jettison after segment
                    profile_sim.append([segment_name, P_orig, T_seg_max, t_start_seg, t_end_seg, P_climb_pts_orig_seg])
                else: # Jettison during segment
                    t_pre = t_j - t_start_seg
                    profile_sim.append([f"{segment_name} (Pre)", P_orig, t_pre, t_start_seg, t_j, P_climb_pts_orig_seg])
                    t_post = t_end_seg - t_j
                    profile_sim.append([f"{segment_name} (Post)", P_post_j, t_post, t_j, t_end_seg, P_climb_pts_post])

            # --- Calculate Energy ---
            E_total_sim = sum(calc_energy_kwh(p, t) for name, p, t, ts, te, pts in profile_sim)

            # --- Calculate Energy at Jettison ---
            E_at_jettison_sim = 0.0
            for name, p, t, ts, te, pts in profile_sim:
                 if te <= t_j + epsilon:
                      E_at_jettison_sim += calc_energy_kwh(p, t)
                 # Segment splitting ensures we don't need elif ts < t_j

            # --- Check Constraint 1: DOD Dropped ---
            dod_dropped_sim = (E_at_jettison_sim / battery_total_energy_kwh * 100.0) if battery_total_energy_kwh > 0 else float('inf')
            if dod_dropped_sim > max_dod_dropped_config + epsilon: # Add tolerance
                continue # Infeasible

            # --- Check Constraint 2: DOD Remaining ---
            mass_remaining_kg = final_batt_mass_kg - m_j
            dod_remaining_sim = float('inf') # Default to infeasible if calculation fails

            if mass_remaining_kg < -epsilon: # Should not happen with loop bounds, but check
                print(f"Warning: Negative remaining mass encountered ({mass_remaining_kg:.2f} kg). Skipping.")
                continue

            if abs(mass_remaining_kg) < epsilon: # Jettisoned entire battery
                if abs(E_total_sim - E_at_jettison_sim) < epsilon * E_total_sim + epsilon : # No energy consumed after jettison
                    dod_remaining_sim = 0.0
                else: # Energy consumed after jettisoning everything - impossible
                    continue # Infeasible
            else: # Some battery remaining
                 capacity_remaining_bol = mass_remaining_kg * pack_spec_energy_gross_BOL_Wh_kg / 1000.0
                 if capacity_remaining_bol <= 0:
                     if E_total_sim - E_at_jettison_sim > epsilon: # Consumed energy post-jettison with zero capacity
                        continue # Infeasible
                     else:
                        dod_remaining_sim = 0.0 # No capacity, no post-jettison energy use
                 else:
                     fraction_remaining = mass_remaining_kg / final_batt_mass_kg
                     E_pre_jett_from_rem = E_at_jettison_sim * fraction_remaining
                     E_post_jett_from_rem = E_total_sim - E_at_jettison_sim
                     E_total_from_rem = E_pre_jett_from_rem + E_post_jett_from_rem
                     dod_remaining_sim = (E_total_from_rem / capacity_remaining_bol) * 100.0

            if dod_remaining_sim > max_dod_remaining_config + epsilon: # Add tolerance
                continue # Infeasible

            # --- Feasible Point Found ---
            feasible_points_found += 1
            if E_total_sim < optimal_energy - epsilon: # Found a better solution (use tolerance)
                optimal_energy = E_total_sim
                optimal_mass = m_j
                optimal_time = t_j
                optimal_dod_dropped = dod_dropped_sim
                optimal_dod_remaining = dod_remaining_sim
                
                

    # --- Report Optimization Results ---
    # Inside run_jettison_optimization function...

    # --- Report Optimization Results ---
    print(f"\nOptimization Complete. Checked {sim_count} scenarios.")
    print(f"Found {feasible_points_found} feasible jettison scenarios meeting DOD constraints.")

    optimal_results = {
        'found_optimum': optimal_mass > 1e-6, # True if non-baseline optimum found
        'mass_kg': optimal_mass,
        'time_s': optimal_time,
        'energy_kwh': optimal_energy,
        'dod_dropped_pct': optimal_dod_dropped,
        'dod_remaining_pct': optimal_dod_remaining,
        'baseline_energy_kwh': min_energy_found # Also return the baseline energy for comparison
    }

    if optimal_results['found_optimum']:
        print("\n--- Optimal Jettison Scenario (Shorter Range) ---")
        print(f"  Optimal Jettison Mass: {optimal_results['mass_kg']:.2f} kg")
        print(f"  Optimal Jettison Time: {optimal_results['time_s']:.1f} s")
        print(f"  Minimum Energy Achieved: {optimal_results['energy_kwh']:.3f} kWh")
        saved_energy = optimal_results['baseline_energy_kwh'] - optimal_results['energy_kwh']
        saved_pct = (saved_energy / optimal_results['baseline_energy_kwh'] * 100.0) if optimal_results['baseline_energy_kwh'] > 0 else 0
        print(f"  Energy Saved vs. No Jettison (Short Range): {saved_energy:.3f} kWh ({saved_pct:.1f}%)")
        print(f"  DOD of Jettisoned Part (@{optimal_results['time_s']:.1f}s): {optimal_results['dod_dropped_pct']:.1f}% (Constraint: <= {max_dod_dropped_config:.1f}%)")
        print(f"  DOD of Remaining Part (End of Mission): {optimal_results['dod_remaining_pct']:.1f}% (Constraint: <= {max_dod_remaining_config:.1f}%)")
    else:
        print("\n--- Optimization Result ---")
        print("No jettison scenario provided lower energy consumption while meeting constraints.")
        print(f"Best energy remains the baseline (no jettison): {optimal_results['baseline_energy_kwh']:.3f} kWh")

    return optimal_results # <-- ADD THIS RETURN STATEMENT
# --- Main Script Execution Flow ---
# ... (Existing code up to calling run_shorter_range_analysis) ...

# Call jettison analysis if enabled
jettison_analysis_performed = False
if enable_jettison_analysis:
    if final_state:
        jettison_analysis_performed = run_jettison_analysis(params, final_state, original_power_profile_segments, total_mission_time_s_orig)
        if not post_jettison_powers or not post_jettison_powers.get('valid', False):
            print("WARNING: Jettison analysis ran but failed to calculate post-jettison powers.")
            jettison_analysis_performed = False
    else:
        print("WARNING: Final state not available. Cannot run jettison analysis.")

# Call shorter range analysis if enabled
if enable_shorter_range_analysis:
    run_shorter_range_analysis(params, final_state, post_jettison_powers, battery_total_energy_kwh, pack_spec_energy_gross_BOL_Wh_kg)

# Call jettison optimization if enabled
optimal_jettison_results = None # Initialize
if jettison_opt_enable:
    optimal_jettison_results = run_jettison_optimization(params, final_state, battery_total_energy_kwh, pack_spec_energy_gross_BOL_Wh_kg)
    
# --- NEW: Power Profile Comparison Plot (Shorter Mission: Jettison vs. No Jettison) ---
# This plot specifically compares the profiles for the SHORTER range analysis.
if enable_shorter_range_analysis and enable_jettison_analysis and post_jettison_powers.get('valid', False):
    print("\nGenerating Power Profile Comparison Plot (Shorter Mission: Jettison vs. No Jettison)...")
    try:
        # --- Parameters for Shorter Mission ---
        # Vehicle/Mission Params from final state
        hover_power_orig_kw = final_state['hover_power_kw']
        climb_powers_orig = [final_state['climb_power_start_kw'], final_state['climb_power_mid_kw'], final_state['climb_power_end_kw']]
        cruise_power_orig_kw = final_state['cruise_power_kw']
        hover_time_segment_s = final_state['hover_time_s'] / 2.0
        climb_time_s = final_state['climb_time_s']
        cruise_speed_m_p_s = params['cruise_speed_m_p_s']

        # Shorter Mission Timing
        if cruise_speed_m_p_s <= 0:
             raise ValueError("Cruise speed must be positive to calculate shorter mission time.")
        shorter_cruise_time_s = (shorter_range_target_km * 1000.0) / cruise_speed_m_p_s
        t_hover_end_short = hover_time_segment_s
        t_climb_end_short = t_hover_end_short + climb_time_s
        t_cruise_end_short = t_climb_end_short + shorter_cruise_time_s
        t_land_start_short = t_cruise_end_short
        t_land_end_short = t_land_start_short + hover_time_segment_s
        total_time_short_mission = t_land_end_short

        # --- 1. Generate Data for Shorter Mission - NO Jettison ---
        no_jettison_short_x = [0]
        no_jettison_short_y = [0]
        current_time_no_jett = 0.0
        epsilon = 1e-9

        # Hover Takeoff (Original Power)
        no_jettison_short_x.extend([current_time_no_jett, t_hover_end_short])
        no_jettison_short_y.extend([hover_power_orig_kw, hover_power_orig_kw])
        current_time_no_jett = t_hover_end_short

        # Climb (Original Power - 3 Steps)
        if climb_time_s > epsilon:
            t_third = climb_time_s / 3.0
            p_climb1, p_climb2, p_climb3 = climb_powers_orig
            # Step 1
            no_jettison_short_x.extend([current_time_no_jett, current_time_no_jett + t_third])
            no_jettison_short_y.extend([p_climb1, p_climb1])
            current_time_no_jett += t_third
            # Step 2
            no_jettison_short_x.extend([current_time_no_jett, current_time_no_jett + t_third])
            no_jettison_short_y.extend([p_climb2, p_climb2])
            current_time_no_jett += t_third
            # Step 3
            step3_dur = max(0, t_climb_end_short - current_time_no_jett) # Ensure exact end time
            no_jettison_short_x.extend([current_time_no_jett, current_time_no_jett + step3_dur])
            no_jettison_short_y.extend([p_climb3, p_climb3])
            current_time_no_jett += step3_dur
        current_time_no_jett = t_climb_end_short # Ensure alignment

        # Cruise (Original Power, Shorter Duration)
        if shorter_cruise_time_s > epsilon:
             no_jettison_short_x.extend([current_time_no_jett, t_cruise_end_short])
             no_jettison_short_y.extend([cruise_power_orig_kw, cruise_power_orig_kw])
             current_time_no_jett = t_cruise_end_short

        # Hover Land (Original Power)
        no_jettison_short_x.extend([current_time_no_jett, t_land_end_short])
        no_jettison_short_y.extend([hover_power_orig_kw, hover_power_orig_kw])
        current_time_no_jett = t_land_end_short

        # End at zero
        no_jettison_short_x.append(current_time_no_jett)
        no_jettison_short_y.append(0)

        # --- 2. Generate Data for Shorter Mission - WITH Jettison ---
        jettison_short_x = [0]
        jettison_short_y = [0]
        current_time_jett_short = 0.0

        # Get post-jettison powers
        hover_power_post = post_jettison_powers['hover']
        climb_powers_post = [post_jettison_powers['climb_end'], post_jettison_powers['climb_start'], post_jettison_powers['climb_mid']]
        cruise_power_post = post_jettison_powers['cruise']

        # Get the specific jettison time and mass used in the analysis
        jettison_t = params.get('jettison_time_s', 0.0)
        jettison_m = post_jettison_powers.get('jettison_mass_kg_actual', 0.0)

        # --- Build the segments list applying jettison logic to SHORTER mission times ---
        jettison_profile_short_for_plot = []

        # Segment 1: Hover Takeoff
        seg_start = 0.0
        seg_end = t_hover_end_short
        seg_dur = hover_time_segment_s
        if jettison_t <= seg_start + epsilon:
             jettison_profile_short_for_plot.append({'name': 'Hover_T_Post', 'power': hover_power_post, 'duration': seg_dur, 'climb_pts': None})
        elif jettison_t >= seg_end - epsilon:
             jettison_profile_short_for_plot.append({'name': 'Hover_T_Pre', 'power': hover_power_orig_kw, 'duration': seg_dur, 'climb_pts': None})
        else:
             t_pre = jettison_t - seg_start; t_post = seg_end - jettison_t
             jettison_profile_short_for_plot.append({'name': 'Hover_T_Pre', 'power': hover_power_orig_kw, 'duration': t_pre, 'climb_pts': None})
             jettison_profile_short_for_plot.append({'name': 'Hover_T_Post', 'power': hover_power_post, 'duration': t_post, 'climb_pts': None})

        # Segment 2: Climb
        seg_start = t_hover_end_short
        seg_end = t_climb_end_short
        seg_dur = climb_time_s
        if seg_dur > epsilon:
            if jettison_t <= seg_start + epsilon:
                jettison_profile_short_for_plot.append({'name': 'Climb_Post', 'power': None, 'duration': seg_dur, 'climb_pts': climb_powers_post})
            elif jettison_t >= seg_end - epsilon:
                jettison_profile_short_for_plot.append({'name': 'Climb_Pre', 'power': None, 'duration': seg_dur, 'climb_pts': climb_powers_orig})
            else:
                t_pre = jettison_t - seg_start; t_post = seg_end - jettison_t
                jettison_profile_short_for_plot.append({'name': 'Climb_Pre', 'power': None, 'duration': t_pre, 'climb_pts': climb_powers_orig})
                jettison_profile_short_for_plot.append({'name': 'Climb_Post', 'power': None, 'duration': t_post, 'climb_pts': climb_powers_post})

        # Segment 3: Cruise (Shorter Duration)
        seg_start = t_climb_end_short
        seg_end = t_cruise_end_short
        seg_dur = shorter_cruise_time_s
        if seg_dur > epsilon:
            if jettison_t <= seg_start + epsilon:
                 jettison_profile_short_for_plot.append({'name': 'Cruise_Post', 'power': cruise_power_post, 'duration': seg_dur, 'climb_pts': None})
            elif jettison_t >= seg_end - epsilon:
                 jettison_profile_short_for_plot.append({'name': 'Cruise_Pre', 'power': cruise_power_orig_kw, 'duration': seg_dur, 'climb_pts': None})
            else:
                 t_pre = jettison_t - seg_start; t_post = seg_end - jettison_t
                 jettison_profile_short_for_plot.append({'name': 'Cruise_Pre', 'power': cruise_power_orig_kw, 'duration': t_pre, 'climb_pts': None})
                 jettison_profile_short_for_plot.append({'name': 'Cruise_Post', 'power': cruise_power_post, 'duration': t_post, 'climb_pts': None})

        # Segment 4: Hover Land
        seg_start = t_land_start_short
        seg_end = t_land_end_short
        seg_dur = hover_time_segment_s
        landing_power = hover_power_post if jettison_t < seg_start + epsilon else hover_power_orig_kw
        jettison_profile_short_for_plot.append({'name': 'Hover_L', 'power': landing_power, 'duration': seg_dur, 'climb_pts': None})

        # --- Now build the x, y arrays for the jettison line ---
        segment_start_time = 0.0
        for seg in jettison_profile_short_for_plot:
            duration = seg['duration']
            if duration < epsilon: continue

            power = seg['power']
            climb_pts = seg['climb_pts']
            segment_end_time = segment_start_time + duration

            if climb_pts is not None: # Handle 3-step climb
                if len(climb_pts) == 3 and duration > epsilon:
                    t_third = duration / 3.0
                    p_climb1, p_climb2, p_climb3 = climb_pts
                    # Step 1
                    jettison_short_x.extend([current_time_jett_short, current_time_jett_short + t_third])
                    jettison_short_y.extend([p_climb1, p_climb1])
                    current_time_jett_short += t_third
                    # Step 2
                    jettison_short_x.extend([current_time_jett_short, current_time_jett_short + t_third])
                    jettison_short_y.extend([p_climb2, p_climb2])
                    current_time_jett_short += t_third
                    # Step 3
                    step3_dur = max(0, segment_end_time - current_time_jett_short) # Use calculated end time
                    jettison_short_x.extend([current_time_jett_short, current_time_jett_short + step3_dur])
                    jettison_short_y.extend([p_climb3, p_climb3])
                    current_time_jett_short += step3_dur
                else:
                     print(f"Warning: Invalid climb segment found during jettison plot generation: {seg}")
            elif power is not None: # Hover or Cruise segment
                jettison_short_x.extend([current_time_jett_short, segment_end_time])
                jettison_short_y.extend([power, power])
                current_time_jett_short = segment_end_time
            else:
                print(f"Warning: Segment with no power or climb points: {seg}")

            segment_start_time = segment_end_time # Update for next segment start

        # End at zero power
        jettison_short_x.append(current_time_jett_short)
        jettison_short_y.append(0)


        # --- 3. Create the Comparison Plot ---
        plt.figure(figsize=(12, 6))

        # Plot Shorter Mission - No Jettison
        plt.step(no_jettison_short_x, no_jettison_short_y, where='post', linewidth=2,
                 label=f'Shorter Mission ({shorter_range_target_km:.0f} km) - No Jettison',
                 color='blue', linestyle='--')

        # Plot Shorter Mission - With Jettison
        plt.step(jettison_short_x, jettison_short_y, where='post', linewidth=2,
                 label=f'Shorter Mission ({shorter_range_target_km:.0f} km) - With Jettison ({jettison_m:.1f} kg @ {jettison_t:.1f}s)',
                 color='red')

        # Add vertical line for jettison time
        max_power_plot = max(max(no_jettison_short_y), max(jettison_short_y)) * 1.05
        plt.vlines(jettison_t, 0, max_power_plot, color='green', linestyle=':', linewidth=2, label=f'Jettison Time ({jettison_t:.1f}s)')

        plt.xlabel('Time (sec)')
        plt.ylabel('Calculated Power (kW)')
        plt.title(f'Power Profile Comparison for Shorter Mission ({shorter_range_target_km:.0f} km)')
        plt.legend()
        plt.grid(True)
        plt.ylim(bottom=0, top=max_power_plot)
        plt.xlim(left=0, right=total_time_short_mission * 1.02)
        plt.tight_layout()
        plt.show(block=False)

    except Exception as e:
        print(f"Could not generate Shorter Mission Power Profile Comparison plot: {e}")
        # import traceback
        # traceback.print_exc()

# --- End NEW block ---
# --- NEW: Power Profile Comparison Plot (Shorter Mission: Optimal Jettison vs. No Jettison) ---
if enable_shorter_range_analysis and jettison_opt_enable and optimal_jettison_results and optimal_jettison_results['found_optimum']:
    print("\nGenerating Power Profile Comparison Plot (Shorter Mission: Optimal Jettison vs. No Jettison)...")
    try:
        # --- Parameters ---
        # Vehicle/Mission Params from final state & params
        hover_power_orig_kw = final_state['hover_power_kw']
        climb_powers_orig = [final_state['climb_power_start_kw'], final_state['climb_power_mid_kw'], final_state['climb_power_end_kw']]
        cruise_power_orig_kw = final_state['cruise_power_kw']
        hover_time_segment_s = final_state['hover_time_s'] / 2.0
        climb_time_s = final_state['climb_time_s']
        cruise_speed_m_p_s = params['cruise_speed_m_p_s']
        mtow_orig_kg = final_state.get('mtow_kg', 0)
        # Params needed for post-jettison power calculation
        g_m_p_s2 = params['g_m_p_s2']; air_density_sea_level_kg_p_m3 = params['air_density_sea_level_kg_p_m3']
        hover_fom = params['hover_fom']; disk_area_m2 = final_state.get('disk_area_m2', 0)
        epu_effic = params['epu_effic']; prop_effic = params['prop_effic']
        base_cd0 = final_state.get('base_cd0', 0); wing_AR = final_state.get('wing_AR', 0)
        wing_ref_area_m2 = final_state.get('wing_ref_area_m2', 0)
        rate_of_climb_mps = params.get('rate_of_climb_mps', default_roc_mps)
        spac_effic_factor = params['spac_effic_factor']; trim_drag_factor = params['trim_drag_factor']
        V_climb_start_mps = params.get('V_climb_start_mps', 20.0); V_climb_end_mps = params['cruise_speed_m_p_s']
        V_climb_mid_mps = (V_climb_start_mps + V_climb_end_mps) / 2.0

        # Optimal jettison parameters
        opt_mass = optimal_jettison_results['mass_kg']
        opt_time = optimal_jettison_results['time_s']

        # Shorter Mission Timing
        if cruise_speed_m_p_s <= 0: raise ValueError("Cruise speed must be positive.")
        shorter_cruise_time_s = (shorter_range_target_km * 1000.0) / cruise_speed_m_p_s
        t_hover_end_short = hover_time_segment_s
        t_climb_end_short = t_hover_end_short + climb_time_s
        t_cruise_end_short = t_climb_end_short + shorter_cruise_time_s
        t_land_start_short = t_cruise_end_short
        t_land_end_short = t_land_start_short + hover_time_segment_s
        total_time_short_mission = t_land_end_short
        epsilon = 1e-9

        # --- 1. Generate Data for Shorter Mission - NO Jettison (Baseline) ---
        #    (This logic is the same as in the previous plot)
        no_jettison_short_x = [0]; no_jettison_short_y = [0]
        current_time_no_jett = 0.0
        # Hover Takeoff
        no_jettison_short_x.extend([current_time_no_jett, t_hover_end_short])
        no_jettison_short_y.extend([hover_power_orig_kw, hover_power_orig_kw])
        current_time_no_jett = t_hover_end_short
        # Climb
        if climb_time_s > epsilon:
            t_third = climb_time_s / 3.0; p_climb1, p_climb2, p_climb3 = climb_powers_orig
            no_jettison_short_x.extend([current_time_no_jett, current_time_no_jett + t_third]); no_jettison_short_y.extend([p_climb1, p_climb1]); current_time_no_jett += t_third
            no_jettison_short_x.extend([current_time_no_jett, current_time_no_jett + t_third]); no_jettison_short_y.extend([p_climb2, p_climb2]); current_time_no_jett += t_third
            step3_dur = max(0, t_climb_end_short - current_time_no_jett)
            no_jettison_short_x.extend([current_time_no_jett, current_time_no_jett + step3_dur]); no_jettison_short_y.extend([p_climb3, p_climb3]); current_time_no_jett += step3_dur
        current_time_no_jett = t_climb_end_short # Align
        # Cruise
        if shorter_cruise_time_s > epsilon:
             no_jettison_short_x.extend([current_time_no_jett, t_cruise_end_short]); no_jettison_short_y.extend([cruise_power_orig_kw, cruise_power_orig_kw])
             current_time_no_jett = t_cruise_end_short
        # Hover Land
        no_jettison_short_x.extend([current_time_no_jett, t_land_end_short]); no_jettison_short_y.extend([hover_power_orig_kw, hover_power_orig_kw])
        current_time_no_jett = t_land_end_short
        # End at zero
        no_jettison_short_x.append(current_time_no_jett); no_jettison_short_y.append(0)

        # --- 2. Calculate Post-OPTIMAL-Jettison Powers ---
        mtow_post_opt_jett_kg = mtow_orig_kg - opt_mass
        if mtow_post_opt_jett_kg <= 0:
            raise ValueError(f"Optimal jettison mass ({opt_mass} kg) results in non-positive MTOW.")

        hover_shaft_post_opt = calc_hover_shaft_power_k_W(mtow_post_opt_jett_kg, g_m_p_s2, air_density_sea_level_kg_p_m3, hover_fom, disk_area_m2)
        hover_power_post_opt = calc_hover_electric_power_k_W(epu_effic, hover_shaft_post_opt)
        cruise_shaft_post_opt = calc_cruise_shaft_power_k_W(mtow_post_opt_jett_kg, g_m_p_s2, prop_effic, cruise_speed_m_p_s, cruise_L_p_D)
        cruise_power_post_opt = calc_cruise_electric_power_k_W(cruise_shaft_post_opt, epu_effic)
        cl_pwr_s_post_opt = calc_power_at_speed_climbing_k_W(mtow_post_opt_jett_kg, V_climb_start_mps, base_cd0, wing_AR, wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
        cl_pwr_m_post_opt = calc_power_at_speed_climbing_k_W(mtow_post_opt_jett_kg, V_climb_mid_mps, base_cd0, wing_AR, wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
        cl_pwr_e_post_opt = calc_power_at_speed_climbing_k_W(mtow_post_opt_jett_kg, V_climb_end_mps, base_cd0, wing_AR, wing_ref_area_m2, air_density_sea_level_kg_p_m3, rate_of_climb_mps, prop_effic, epu_effic, g_m_p_s2, spac_effic_factor, trim_drag_factor)
        climb_powers_post_opt = [cl_pwr_s_post_opt, cl_pwr_m_post_opt, cl_pwr_e_post_opt]


        # --- 3. Generate Data for Shorter Mission - WITH OPTIMAL Jettison ---
        optimal_jettison_short_x = [0]; optimal_jettison_short_y = [0]
        current_time_opt_jett = 0.0

        # --- Build the segments list applying OPTIMAL jettison logic ---
        optimal_profile_short_for_plot = []
        # Segment 1: Hover Takeoff
        seg_start = 0.0; seg_end = t_hover_end_short; seg_dur = hover_time_segment_s
        if opt_time <= seg_start + epsilon: optimal_profile_short_for_plot.append({'name': 'Hover_T_Post', 'power': hover_power_post_opt, 'duration': seg_dur, 'climb_pts': None})
        elif opt_time >= seg_end - epsilon: optimal_profile_short_for_plot.append({'name': 'Hover_T_Pre', 'power': hover_power_orig_kw, 'duration': seg_dur, 'climb_pts': None})
        else: t_pre = opt_time - seg_start; t_post = seg_end - opt_time; optimal_profile_short_for_plot.append({'name': 'Hover_T_Pre', 'power': hover_power_orig_kw, 'duration': t_pre, 'climb_pts': None}); optimal_profile_short_for_plot.append({'name': 'Hover_T_Post', 'power': hover_power_post_opt, 'duration': t_post, 'climb_pts': None})
        # Segment 2: Climb
        seg_start = t_hover_end_short; seg_end = t_climb_end_short; seg_dur = climb_time_s
        if seg_dur > epsilon:
            if opt_time <= seg_start + epsilon: optimal_profile_short_for_plot.append({'name': 'Climb_Post', 'power': None, 'duration': seg_dur, 'climb_pts': climb_powers_post_opt})
            elif opt_time >= seg_end - epsilon: optimal_profile_short_for_plot.append({'name': 'Climb_Pre', 'power': None, 'duration': seg_dur, 'climb_pts': climb_powers_orig})
            else: t_pre = opt_time - seg_start; t_post = seg_end - opt_time; optimal_profile_short_for_plot.append({'name': 'Climb_Pre', 'power': None, 'duration': t_pre, 'climb_pts': climb_powers_orig}); optimal_profile_short_for_plot.append({'name': 'Climb_Post', 'power': None, 'duration': t_post, 'climb_pts': climb_powers_post_opt})
        # Segment 3: Cruise
        seg_start = t_climb_end_short; seg_end = t_cruise_end_short; seg_dur = shorter_cruise_time_s
        if seg_dur > epsilon:
            if opt_time <= seg_start + epsilon: optimal_profile_short_for_plot.append({'name': 'Cruise_Post', 'power': cruise_power_post_opt, 'duration': seg_dur, 'climb_pts': None})
            elif opt_time >= seg_end - epsilon: optimal_profile_short_for_plot.append({'name': 'Cruise_Pre', 'power': cruise_power_orig_kw, 'duration': seg_dur, 'climb_pts': None})
            else: t_pre = opt_time - seg_start; t_post = seg_end - opt_time; optimal_profile_short_for_plot.append({'name': 'Cruise_Pre', 'power': cruise_power_orig_kw, 'duration': t_pre, 'climb_pts': None}); optimal_profile_short_for_plot.append({'name': 'Cruise_Post', 'power': cruise_power_post_opt, 'duration': t_post, 'climb_pts': None})
        # Segment 4: Hover Land
        seg_start = t_land_start_short; seg_end = t_land_end_short; seg_dur = hover_time_segment_s
        landing_power_opt = hover_power_post_opt if opt_time < seg_start + epsilon else hover_power_orig_kw
        optimal_profile_short_for_plot.append({'name': 'Hover_L', 'power': landing_power_opt, 'duration': seg_dur, 'climb_pts': None})

        # --- Now build the x, y arrays for the optimal jettison line ---
        segment_start_time = 0.0
        for seg in optimal_profile_short_for_plot:
            duration = seg['duration']
            if duration < epsilon: continue
            power = seg['power']; climb_pts = seg['climb_pts']
            segment_end_time = segment_start_time + duration
            if climb_pts is not None:
                if len(climb_pts) == 3 and duration > epsilon:
                    t_third = duration / 3.0; p_climb1, p_climb2, p_climb3 = climb_pts
                    optimal_jettison_short_x.extend([current_time_opt_jett, current_time_opt_jett + t_third]); optimal_jettison_short_y.extend([p_climb1, p_climb1]); current_time_opt_jett += t_third
                    optimal_jettison_short_x.extend([current_time_opt_jett, current_time_opt_jett + t_third]); optimal_jettison_short_y.extend([p_climb2, p_climb2]); current_time_opt_jett += t_third
                    step3_dur = max(0, segment_end_time - current_time_opt_jett)
                    optimal_jettison_short_x.extend([current_time_opt_jett, current_time_opt_jett + step3_dur]); optimal_jettison_short_y.extend([p_climb3, p_climb3]); current_time_opt_jett += step3_dur
                else: print(f"Warning: Invalid climb segment for optimal jettison plot: {seg}")
            elif power is not None:
                optimal_jettison_short_x.extend([current_time_opt_jett, segment_end_time]); optimal_jettison_short_y.extend([power, power])
                current_time_opt_jett = segment_end_time
            else: print(f"Warning: Segment with no power or climb points for optimal jettison plot: {seg}")
            segment_start_time = segment_end_time
        optimal_jettison_short_x.append(current_time_opt_jett); optimal_jettison_short_y.append(0)


        # --- 4. Create the Comparison Plot ---
        plt.figure(figsize=(12, 6))
        # Plot Baseline (No Jettison, Shorter Mission)
        plt.step(no_jettison_short_x, no_jettison_short_y, where='post', linewidth=2,
                 label=f'Shorter Mission ({shorter_range_target_km:.0f} km) - No Jettison',
                 color='blue', linestyle='--')
        # Plot Optimal Jettison (Shorter Mission)
        plt.step(optimal_jettison_short_x, optimal_jettison_short_y, where='post', linewidth=2,
                 label=f'Shorter Mission ({shorter_range_target_km:.0f} km) - Optimal Jettison ({opt_mass:.1f} kg @ {opt_time:.1f}s)',
                 color='purple') # Changed color for distinction
        # Add vertical line for optimal jettison time
        max_power_plot_opt = max(max(no_jettison_short_y), max(optimal_jettison_short_y)) * 1.05
        plt.vlines(opt_time, 0, max_power_plot_opt, color='orange', linestyle=':', linewidth=2, label=f'Optimal Jettison Time ({opt_time:.1f}s)')

        plt.xlabel('Time (sec)')
        plt.ylabel('Calculated Power (kW)')
        plt.title(f'Power Profile Comparison for Shorter Mission ({shorter_range_target_km:.0f} km): Optimal Jettison vs. No Jettison')
        plt.legend()
        plt.grid(True)
        plt.ylim(bottom=0, top=max_power_plot_opt)
        plt.xlim(left=0, right=total_time_short_mission * 1.02)
        plt.tight_layout()
        plt.show(block=False)

    except Exception as e:
        print(f"Could not generate Optimal Jettison Power Profile Comparison plot: {e}")
        # import traceback
        # traceback.print_exc()

# --- End NEW block ---


print("\nDone.")
# Keep plots open until user closes them
plt.show()





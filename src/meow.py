import math

def calc_power_at_speed_climbing_k_W_advanced_prop_model(
    mtow_kg, target_speed_mps, base_cd0, wing_AR, wing_ref_area_m2, rho,
    rate_of_climb_mps, 
    # prop_effic is no longer directly used for shaft power from thrust power,
    # as the advanced model calculates shaft power. It could represent other
    # drivetrain efficiencies if epu_effic is purely EPU.
    # For simplicity, we'll assume this model gives total shaft power.
    prop_effic, # This argument might be vestigial or re-purposed
    epu_effic, g, spac_effic_factor, trim_drag_factor,
    # New parameters for the advanced propeller model
    prop_disk_area_m2, 
    prop_kappa=1.2 # Default kappa from the paper
    ):

    weight_n = mtow_kg * g

    if target_speed_mps <= 0:
        if rate_of_climb_mps > 0 and target_speed_mps == 0: # Pure vertical flight (hover climb)
            thrust_req_hover_climb_n = weight_n # Simplified: no download, assuming D=0 for pure vertical
            
            # Use advanced prop model for hover climb
            # V_perp is effectively the rate_of_climb_mps if props are horizontal (for vertical lift)
            # However, the model is typically for V_inf (freestream). For pure hover climb,
            # induced velocity dominates. A common simplification for hover power is from momentum theory:
            # P_hover_ideal = T * sqrt(T / (2 * rho * A_disk))
            # Using the paper's model with V_perp = rate_of_climb_mps (if props face airflow of ROC)
            # This part gets tricky for pure vertical VTOL mode vs. cruise climb.
            # For simplicity, let's assume T_req for vertical power calc is just W * ROC
            # and then use the general formula.
            # If it's true vertical lift (props horizontal), V_inf_perp relative to prop IS rate_of_climb_mps for axial flow component
            # However, the dominant effect is induced velocity.
            # For this specific case, let's just use the formula with V_perp = rate_of_climb_mps
            # It might be more accurate to use a dedicated hover power model and add it.
            # Let's stick to the paper's equation for thrust T = W, V_perp = rate_of_climb_mps
            
            # If ROC is the primary airflow through horizontal propellers:
            v_perp_for_vertical_climb = abs(rate_of_climb_mps) # speed normal to disk
            thrust_for_vertical_climb = weight_n # In pure vertical, thrust supports weight

            if prop_disk_area_m2 <=0: return float('nan')

            # Power from advanced model for vertical climb (T=W, V=ROC)
            # This assumes propellers are oriented to provide vertical thrust and ROC is axial flow component
            term_sqrt_inner_vert = v_perp_for_vertical_climb**2 + thrust_for_vertical_climb / (rho * prop_disk_area_m2)
            if term_sqrt_inner_vert < 0: return float('nan') # Should not happen for T > 0

            power_shaft_watts_vert = (thrust_for_vertical_climb * v_perp_for_vertical_climb +
                                   prop_kappa * thrust_for_vertical_climb *
                                   (-v_perp_for_vertical_climb + math.sqrt(term_sqrt_inner_vert)))
            
            total_shaft_power_req_kw_vert = power_shaft_watts_vert / 1000.0
            if epu_effic <= 0: return float('inf') if total_shaft_power_req_kw_vert > 0 else 0.0
            return total_shaft_power_req_kw_vert / epu_effic
        return float('nan') # Undefined for V=0, ROC=0 with this model

    # --- Standard climb calculation using W*cos(gamma) for Lift ---
    sin_gamma = rate_of_climb_mps / target_speed_mps
    if not (-1.0 <= sin_gamma <= 1.0):
        # print(f"Warning: Impossible climb angle sin_gamma={sin_gamma:.2f}")
        sin_gamma = max(-1.0, min(1.0, sin_gamma)) # Clamp it

    cos_gamma = math.sqrt(max(0, 1 - sin_gamma**2))
    lift_needed = weight_n * cos_gamma

    q = 0.5 * rho * target_speed_mps**2
    if q == 0: return float('nan') # Should have been caught by target_speed_mps <= 0

    if wing_ref_area_m2 <= 0: return float('nan')
    cl_req = lift_needed / (q * wing_ref_area_m2) if q * wing_ref_area_m2 != 0 else 0

    if wing_AR <= 0 or spac_effic_factor <= 0: return float('nan')
    cdi = cl_req**2 / (math.pi * spac_effic_factor * wing_AR)
    cd_total_untrimmed = base_cd0 + cdi
    cd_total = cd_total_untrimmed * trim_drag_factor
    drag_n = q * wing_ref_area_m2 * cd_total

    # Thrust required for climb
    thrust_req_n = drag_n + weight_n * sin_gamma
    if thrust_req_n < 0: # e.g. steep descent where drag > -W*sin_gamma
        thrust_req_n = 0 # Assuming no reverse thrust capability or not modeled here

    # --- Apply Advanced Propeller Power Model (Corrected Chauhan & Martins form) ---
    # V_inf_perp is target_speed_mps, assuming propellers are aligned with flight path
    v_perp = target_speed_mps 
    
    if prop_disk_area_m2 <=0: return float('nan')

    # Term T/(rho*A_disk) must be non-negative if T is non-negative.
    term_for_inner_sqrt = thrust_req_n / (rho * prop_disk_area_m2)
    if term_for_inner_sqrt < 0 and thrust_req_n >=0 : # Should only happen if T_req is negative and large
        # This implies a braking scenario not well handled by this forward thrust model.
        # For simplicity, if T_req became negative (e.g. steep dive), we set it to 0 above.
        # If T_req is 0, then power should be 0 from this model.
        pass

    term_sqrt_inner = v_perp**2 + term_for_inner_sqrt
    if term_sqrt_inner < 0:
        # This can happen if thrust_req_n is negative (braking) and large enough.
        # The model is for power *generation*. For braking, power might be absorbed or dissipated.
        # If thrust_req_n was set to 0, this part won't be an issue.
        # print(f"Warning: sqrt term negative ({term_sqrt_inner:.2f}) for T_req={thrust_req_n:.1f} N, V={v_perp:.1f} m/s. Setting shaft power to 0 for this case.")
        power_shaft_watts = 0.0 # Or handle as error
    else:
        power_shaft_watts = (thrust_req_n * v_perp +
                           prop_kappa * thrust_req_n *
                           (-v_perp + math.sqrt(term_sqrt_inner)))
    
    total_shaft_power_req_kw = power_shaft_watts / 1000.0
    
    # If total_shaft_power_req_kw is negative (e.g. windmilling propeller in steep descent),
    # it implies power generation. We'll cap at 0 for required power.
    if total_shaft_power_req_kw < 0:
        total_shaft_power_req_kw = 0.0

    if epu_effic <= 0:
        return float('inf') if total_shaft_power_req_kw > 0 else 0.0
        
    electric_power_req_kw = total_shaft_power_req_kw / epu_effic

    return electric_power_req_kw

# --- Test with original parameters that previously showed U-shaped power curve ---
# Parameters from your original example that resulted in U-shaped power curve
mtow_kg_val = 1000
base_cd0_val = 0.025
wing_AR_val = 10
wing_ref_area_m2_val = 15
rho_val = 1.225
g_val = 9.81
spac_effic_factor_val = 0.85 
trim_drag_factor_val = 1.05
prop_effic_val = 0.80 # Will be largely ignored by new model, but pass for signature
epu_effic_val = 0.90
roc_mps_val = 2.0

# New parameters for the advanced propeller model:
# Estimate prop_disk_area_m2: For 1000kg MTOW, hover T ~ 10000 N.
# If disk loading T/A = 500 N/m^2 (typical), then A_total = 10000/500 = 20 m^2.
# This is total area for all props.
prop_disk_area_m2_val = 20.0
prop_kappa_val = 1.2 # From the paper

print(f"Calculating with ADVANCED PROPELLER MODEL for ROC = {roc_mps_val} m/s:\n")
print(f"Original Aero Params: CD0={base_cd0_val}, AR={wing_AR_val}, e={spac_effic_factor_val}")
print(f"Prop Model Params: A_disk={prop_disk_area_m2_val} m^2, kappa={prop_kappa_val}\n")

speeds_mps = [20, 30, 40, 50, 60, 70]

print("--- Power with Advanced Propeller Model (Corrected Interpretation L=Wcos(gamma)) ---")
for speed in speeds_mps:
    if speed <= 0 or (roc_mps_val > 0 and roc_mps_val >= speed and speed > 0):
        # print(f"Speed: {speed:5.1f} m/s - Potentially problematic climb angle (ROC {roc_mps_val} m/s)")
        # Let function handle clamping or errors.
        pass
    
    power_adv = calc_power_at_speed_climbing_k_W_advanced_prop_model(
        mtow_kg_val, speed, base_cd0_val, wing_AR_val, wing_ref_area_m2_val, rho_val,
        roc_mps_val, prop_effic_val, epu_effic_val, g_val, 
        spac_effic_factor_val,
        trim_drag_factor_val,
        prop_disk_area_m2=prop_disk_area_m2_val,
        prop_kappa=prop_kappa_val
    )
    
    # For display:
    weight_n_test = mtow_kg_val * g_val
    sin_gamma_test = roc_mps_val / speed if speed > 0 else 0
    if abs(sin_gamma_test) > 1: sin_gamma_test = math.copysign(1, sin_gamma_test)
    cos_gamma_test = math.sqrt(max(0, 1 - sin_gamma_test**2))
    lift_needed_corrected = weight_n_test * cos_gamma_test
    q_test = 0.5 * rho_val * speed**2 if speed > 0 else 0
    cl_req_test = lift_needed_corrected / (q_test * wing_ref_area_m2_val) if q_test * wing_ref_area_m2_val !=0 else 0

    print(f"Speed: {speed:5.1f} m/s, Lift: {lift_needed_corrected:7.0f} N, CL: {cl_req_test:6.3f}, Power (Adv Prop): {power_adv:6.1f} kW")

# For comparison, let's run the original "corrected" code (without advanced prop model)
# to see the difference the advanced prop model makes.
print("\n--- For Reference: Original Corrected Model (Simple prop_effic) ---")
# Re-define the previous "corrected" function for clarity
def calc_power_at_speed_climbing_k_W_original_corrected(
    mtow_kg, target_speed_mps, base_cd0, wing_AR, wing_ref_area_m2, rho,
    rate_of_climb_mps, prop_effic, epu_effic, g, spac_effic_factor, trim_drag_factor
):
    weight_n = mtow_kg * g
    if target_speed_mps <= 0: return float('nan')
    sin_gamma = rate_of_climb_mps / target_speed_mps
    if not (-1.0 <= sin_gamma <= 1.0): sin_gamma = max(-1.0, min(1.0, sin_gamma))
    cos_gamma = math.sqrt(max(0, 1 - sin_gamma**2))
    lift_needed = weight_n * cos_gamma
    q = 0.5 * rho * target_speed_mps**2
    if q == 0: return float('nan')
    if wing_ref_area_m2 <= 0: return float('nan')
    cl_req = lift_needed / (q * wing_ref_area_m2) if q * wing_ref_area_m2 != 0 else 0
    if wing_AR <= 0 or spac_effic_factor <= 0: return float('nan')
    cdi = cl_req**2 / (math.pi * spac_effic_factor * wing_AR)
    cd_total_untrimmed = base_cd0 + cdi
    cd_total = cd_total_untrimmed * trim_drag_factor
    drag_n = q * wing_ref_area_m2 * cd_total
    power_drag_kw = (drag_n * target_speed_mps) / 1000.0
    power_vertical_kw = (weight_n * rate_of_climb_mps) / 1000.0
    if prop_effic <= 0: return float('inf') if (power_drag_kw + power_vertical_kw) > 0 else 0.0
    total_shaft_power_req_kw = (power_drag_kw + power_vertical_kw) / prop_effic
    if epu_effic <= 0: return float('inf') if total_shaft_power_req_kw > 0 else 0.0
    electric_power_req_kw = total_shaft_power_req_kw / epu_effic
    return electric_power_req_kw

for speed in speeds_mps:
    power_orig_corr = calc_power_at_speed_climbing_k_W_original_corrected(
        mtow_kg_val, speed, base_cd0_val, wing_AR_val, wing_ref_area_m2_val, rho_val,
        roc_mps_val, prop_effic_val, epu_effic_val, g_val, spac_effic_factor_val, trim_drag_factor_val
    )
    print(f"Speed: {speed:5.1f} m/s, Power (Orig Corrected): {power_orig_corr:6.1f} kW")
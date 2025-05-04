import json
import math
import sys
import traceback
import argparse 
from tabulate import tabulate 
from missionsegment import Mission, HoverSegment, ClimbSegment, CruiseSegment, ReserveSegment, ShorterRangeMission
from base_component import AircraftComponent 
from aircraft import Aircraft, Wing, Fuselage, HorizontalTail, VerticalTail, LandingGear, Boom, LiftRotor, TiltRotor
from battery import Battery
from iterative_size import IterativeSizer # Import the new sizer class
import copy
import numpy as np # 
from tabulate import tabulate


W_PER_KW = 1000.0
DEFAULT_JSON_PATH = "evtol-param.json" 

# --- Main Execution ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="eVTOL Aircraft Sizing Simulation")
    parser.add_argument(
        "-c", "--config",
        default=DEFAULT_JSON_PATH,
        help=f"Path to the JSON configuration file (default: {DEFAULT_JSON_PATH})"
    )
    parser.add_argument(
        "-i", "--max_iter",
        type=int, default=1000
    )
    parser.add_argument(
        "-t", "--tolerance",
        type=float, default=0.1

    )
    parser.add_argument(
        "-d", "--damping",
        type=float, default=0.3
    )
    args = parser.parse_args()

    path_to_json = args.config
    MAX_ITERATIONS = args.max_iter
    TOLERANCE = args.tolerance
    DAMPING_FACTOR = args.damping

    print("--- eVTOL Aircraft Sizing Simulation ---")
    print(f"Using configuration file: {path_to_json}")
    print(f"Convergence Params: Max Iter={MAX_ITERATIONS}, Tolerance={TOLERANCE} kg, Damping={DAMPING_FACTOR}")
    print("-" * 40)

    # --- Initialization Phase ---
    # 1. Component Instantiation
    print("Initializing components...")
    wing = Wing(path_to_json)
    fuselage = Fuselage(path_to_json)
    horizontaltail = HorizontalTail(path_to_json)
    verticaltail = VerticalTail(path_to_json)
    landinggear = LandingGear(path_to_json)
    boom = Boom(path_to_json)
    tilt_rotor = TiltRotor(path_to_json) 
    lift_rotor = LiftRotor(path_to_json) 
    battery_instance = Battery(path_to_json) 

    # All ccmponents that contribute to aircraft weight/performance
    aircraft_components = [
        wing, fuselage, horizontaltail, verticaltail, landinggear,
        boom, tilt_rotor, lift_rotor, battery_instance
    ]

    for i, component in enumerate(aircraft_components):
        if not isinstance(component, AircraftComponent): 
            print(f"ERROR: Item at index {i} ({type(component).__name__}) is not a valid AircraftComponent.")
            sys.exit(1)
        component.load_variables_from_json()
    print("Components initialized.")

    # 2. Aircraft Instance
    aircraft = Aircraft(components=aircraft_components, path_to_json=path_to_json)

    # 3. Mission Segments & Mission Instance
    mission_segments = [
        HoverSegment(path_to_json, segment_type="Takeoff Hover"),
        ClimbSegment(path_to_json),
        CruiseSegment(path_to_json),
        HoverSegment(path_to_json, segment_type="Landing Hover"),
        ReserveSegment(path_to_json)
    ]
    # Pass the aircraft instance to the Mission
    mission = Mission(aircraft=aircraft, mission_segments=mission_segments)


    # --- MTOW Convergence Loop (using IterativeSizer) ---
    try:
        # Instantiate the sizer
        sizer = IterativeSizer(
            config_path=path_to_json,
            max_iter=MAX_ITERATIONS,
            tolerance=TOLERANCE,
            damping=DAMPING_FACTOR
        )

        # Run the sizing loop
        converged, final_mtow = sizer.run_sizing_loop(aircraft, mission)

        if final_mtow is None:
             print("ERROR: Sizing loop failed to produce a final MTOW estimate.")
             sys.exit(1)

        print(f"\nFinal MTOW (Converged={converged}): {final_mtow:.2f} kg")
        print("-" * 60)

        # --- Final Update & Results Display ---
        print("Performing final state update with converged/final MTOW...")
        # Final update of all components based on the final MTOW
        aircraft.update_state_for_iteration(final_mtow)

        # Re-run mission and resize battery one last time based on final MTOW
        # UpdateComponent recalculates cell counts based on the *last calculated* mass
        aircraft.battery.UpdateComponent({'mtow_kg': final_mtow})
        print(f"  Final battery state updated. BOL Cap: {aircraft.battery.get_gross_BOL_capacity_kwh():.2f} kWh")

        # Re-run mission with the final aircraft state
        mission.run_mission()
        if not mission.is_calculated:
            print("Warning: Final mission run failed.")
        else:
            # Recalculate final battery mass based on this final energy demand
            final_energy = mission.total_energy_kwh
            final_spec_e = aircraft.battery.get_usable_EOL_spec_energy()
            if (hasattr(aircraft, 'max_dod') and aircraft.max_dod > 0 and
                final_energy is not None and final_spec_e > 0):

                final_req_total_usable_kwh = final_energy / aircraft.max_dod
                final_batt_mass = (final_req_total_usable_kwh * W_PER_KW) / final_spec_e

                # Update the battery mass and internal state one last time
                aircraft.battery.final_batt_mass_kg = final_batt_mass
                aircraft.battery.UpdateComponent({'mtow_kg': final_mtow})
                print(f"  Final battery mass recalculated based on final energy: {final_batt_mass:.2f} kg")
            else:
                 print("  Warning: Could not perform final battery mass recalculation due to missing data.")


            # Final check of calculated MTOW vs the converged/final value
            final_mtow_check = aircraft.CalculateMTOW()
            print("Final state update complete.")
            print(f"Final Check MTOW: {final_mtow_check:.2f} kg (Should be close to {final_mtow:.2f} kg)")
            if abs(final_mtow_check - final_mtow) > TOLERANCE * 2: # Allow slightly larger diff due to final recalc
                 print(f"Warning: Final MTOW check ({final_mtow_check:.2f} kg) differs significantly from loop result ({final_mtow:.2f} kg).")


            # --- Display Final Results ---
            print("\n" + "="*60)
            print("          FINAL AIRCRAFT CONFIGURATION & PERFORMANCE")
            print("="*60)

            # Mission Summary
            print("\n--- Final Mission Simulation Results ---")
            mission.display_summary()

            # Drag Coefficients
            print("\n--- Final Cruise Aerodynamic Performance ---")
            final_wing = next((c for c in aircraft.components if isinstance(c, Wing)), None)
            if final_wing and final_wing.wing_ref_area_m2 > 0: # Check area > 0
                final_cruise_speed = aircraft._varlist['cruise_speed_m_p_s']
                final_rho_cruise = aircraft._varlist.get('air_density_cruise_kg_p_m3')
                g_m_p_s2 = aircraft._varlist.get('g_m_p_s2', 9.81)

                # Use the final checked MTOW for consistency
                final_cl = aircraft.calculate_cruise_CL(final_mtow_check * g_m_p_s2, final_rho_cruise, final_cruise_speed, final_wing.wing_ref_area_m2)
                final_cdi = aircraft.calc_Cdi(aircraft.spac_effic_factor, final_wing.wing_AR, final_cl)
                final_cd0_p_total = aircraft.Cd0_total_parasite
                final_cd = (final_cd0_p_total + final_cdi) * aircraft.trim_drag_factor
                final_ld = final_cl / final_cd if final_cd > 1e-9 else float('inf') # Avoid division by zero

                print(f"  Cruise CL:          {final_cl:.4f}")
                print(f"  Parasite Drag (Cd0):{final_cd0_p_total:.5f}")
                print(f"  Induced Drag (Cdi): {final_cdi:.5f}")
                print(f"  Total Drag (Cd):    {final_cd:.5f}")
                print(f"  Cruise L/D:         {final_ld:.2f}")
                print(f"  Wing ref area:      {final_wing.wing_ref_area_m2:.2f} m^2")
            else:
                print("  Could not calculate final aero performance (Wing data missing or invalid).")


            # Component Weights
            print("\n--- Final Component Weights ---")
            weight_data = []
            calculated_sum = 0.0
            total_payload_kg = getattr(aircraft, 'total_payload_kg', 0.0)
            fixed_systems_kg = getattr(aircraft, 'fixed_systems_kg', 0.0)

            for comp in aircraft.components:
                 try:
                    # Use the 'weight' attribute directly after UpdateComponent calls
                    w = comp.weight if hasattr(comp, 'weight') else 0.0
                    weight_data.append([type(comp).__name__, f"{w:.2f}"])
                    calculated_sum += w
                 except Exception as e:
                    print(f"Warning: Error getting weight for {type(comp).__name__}: {e}")
                    weight_data.append([type(comp).__name__, "Error"])

            weight_data.append(["Total Payload", f"{total_payload_kg:.2f}"])
            weight_data.append(["Fixed Systems", f"{fixed_systems_kg:.2f}"])
            calculated_sum += total_payload_kg + fixed_systems_kg

            print(tabulate(weight_data, headers=["Component/Item", "Weight (kg)"], tablefmt="grid"))
            print(f"\nSum of Weights: {calculated_sum:.2f} kg")
            print(f"Final MTOW:     {final_mtow_check:.2f} kg")
            if abs(calculated_sum - final_mtow_check) > TOLERANCE * 2: # Allow slightly larger diff
                 print("Warning: Sum of weights differs significantly from final calculated MTOW.")

            # Battery Details
            if hasattr(aircraft, 'battery') and aircraft.battery and isinstance(aircraft.battery, Battery):
                 print("\n--- Final Battery Properties ---")
                 batt = aircraft.battery
                 # Ensure battery state is fully updated before displaying
                 # batt.UpdateComponent({'mtow_kg': final_mtow_check}) # Already done above

                 details = [
                     ["Final Mass", f"{batt.weight:.2f} kg"],
                     ["Gross BOL Spec Energy", f"{batt.get_gross_BOL_spec_energy():.1f} Wh/kg"],
                     ["Usable EOL Spec Energy", f"{batt.get_usable_EOL_spec_energy():.1f} Wh/kg"],
                     ["Gross BOL Capacity", f"{batt.get_gross_BOL_capacity_kwh():.2f} kWh"],
                     ["Usable EOL Capacity", f"{batt.get_usable_EOL_capacity_kwh():.2f} kWh"],
                     ["Total Charge BOL", f"{batt.get_total_charge_ah_BOL():.1f} Ah"],
                     ["Cell Config (SxP)", f"{getattr(batt, 'cells_series', 'N/A')} x {getattr(batt, 'cells_parallel', 'N/A')}"],
                     ["Total Cell Count", f"{getattr(batt, 'cell_count', 'N/A')}"],
                 ]
                 print(tabulate(details, tablefmt="plain"))
                 print("-" * 30)
            else: print("\n--- Final Battery Properties --- \n  Battery details not available.")

            # Plotting
            print("\nGenerating mission profile plot...")
            try:
                import matplotlib.pyplot as plt
                if mission.is_calculated:
                    time_s, power_kw = mission.get_power_profile()
                    time_s_c, c_rate = mission.get_c_rate_profile()
                    if time_s and power_kw and time_s_c and c_rate: # Check if lists are not empty
                        fig, ax1 = plt.subplots(figsize=(10, 6))
                        color = 'tab:blue'
                        ax1.set_xlabel('Time (s)')
                        ax1.set_ylabel('Power Draw (kW)', color=color)
                        ax1.step(time_s, power_kw, where='post', color=color, linewidth=2, label='Power Profile')
                        ax1.tick_params(axis='y', labelcolor=color)
                        ax1.grid(True, axis='y', linestyle=':', alpha=0.6)
                        ax1.set_ylim(bottom=0)

                        ax2 = ax1.twinx()
                        color = 'tab:red'
                        ax2.set_ylabel('C-Rate', color=color)
                        ax2.step(time_s_c, c_rate, where='post', color=color, linestyle='--', linewidth=2, label='C-Rate Profile')
                        ax2.tick_params(axis='y', labelcolor=color)
                        ax2.set_ylim(bottom=0)

                        plt.title('Mission Power and C-Rate Profile (Final Sized Aircraft)')
                        lines, labels = ax1.get_legend_handles_labels()
                        lines2, labels2 = ax2.get_legend_handles_labels()
                        ax2.legend(lines + lines2, labels + labels2, loc='best')
                        fig.tight_layout()
                        plt.show()
                        print("Plot generated.")
                    else: print("Mission profile data is empty. Cannot plot.")
                else: print("Mission was not calculated successfully. Cannot plot profile.")
            except ImportError:
                print("Matplotlib not found. Skipping plot generation. Install with: pip install matplotlib")
            except Exception as plot_err:
                print(f"Error during plot generation: {plot_err}")


    # END OF SIZING
    except Exception as main_err:
        print(f"\nFATAL ERROR during main execution: {main_err}")
        traceback.print_exc()
    finally:
        print("\n--- End of Aircraft Sizing Simulation ---")


# --- ABU Optimization Section (Remains mostly unchanged) ---

def run_mission_subset(aircraft, mission_segments, start_idx, end_idx, current_mtow, current_batt_capacity_kwh):

    total_energy = 0.0
    segment_results_list = []
    temp_aircraft_state = copy.deepcopy(aircraft) # Work on a copy if methods modify state

    # --- Crucial: Update the state for the subset simulation ---
    temp_aircraft_state.mtow_kg = current_mtow
    # Temporarily adjust battery capacity for C-rate calcs if needed
    # This assumes calculate_performance uses aircraft.get_battery_capacity_kwh()
    original_capacity = temp_aircraft_state.battery.get_gross_BOL_capacity_kwh()
    # Modify the battery state *within the temporary aircraft object*
    # This requires the Battery object to allow temporary modification or accept capacity override
    # Option A: Modify battery object directly (if safe on a copy)
    temp_aircraft_state.battery.gross_BOL_kWh = current_batt_capacity_kwh # Adjust capacity used by get_...
    # Option B: Modify segment calculations? Harder.

    print(f"      Simulating segments {start_idx}-{end_idx} with MTOW={current_mtow:.2f} kg, Batt Cap={current_batt_capacity_kwh:.2f} kWh")

    try:
        for i in range(start_idx, end_idx + 1):
            segment = mission_segments[i]
            # Ensure the segment uses the UPDATED state from temp_aircraft_state
            segment.calculate_performance(temp_aircraft_state) # Pass the modified aircraft state

            if segment.energy_kwh is None or segment.duration_s is None:
                 print(f"      ERROR: Segment {i} calculation failed.")
                 # Restore original capacity before returning failure
                 temp_aircraft_state.battery.gross_BOL_kWh = original_capacity
                 return None, None

            total_energy += segment.energy_kwh
            segment_results_list.append(segment.get_results())
            # print(f"        Segment {i}: E={segment.energy_kwh:.3f} kWh") # Debug print

        # Restore original capacity on the temporary object before finishing
        temp_aircraft_state.battery.gross_BOL_kWh = original_capacity
        return total_energy, segment_results_list

    except Exception as e:
        print(f"      ERROR during subset simulation: {e}")
        import traceback
        traceback.print_exc()
        # Restore original capacity on error
        temp_aircraft_state.battery.gross_BOL_kWh = original_capacity
        return None, None


def optimize_abu_jettison(sized_aircraft, base_mission_segments):
    """
    Optimizes ABU mass and drop timing based on JSON parameters.
    Uses the "Snap to Segment End" approach for drop timing.
    """
    print("\n" + "="*40)
    print("--- Running ABU Jettison Optimization ---")
    print("="*40)

    # 0. Check if enabled and load params
    if not sized_aircraft._varlist.get("jettison_opt_enable", False):
        print("ABU Jettison Optimization disabled in JSON config.")
        return None

    try:
        shorter_range_km = sized_aircraft._varlist["shorter_range_km"]
        max_abu_mass = sized_aircraft._varlist["jettison_opt_batt_mass_max_kg"]
        step_abu_mass = sized_aircraft._varlist["jettison_opt_batt_mass_step_kg"]
        time_start = sized_aircraft._varlist["jettison_opt_time_start_s"]
        time_end = sized_aircraft._varlist["jettison_opt_time_end_s"]
        time_step = sized_aircraft._varlist["jettison_opt_time_step_s"]
        max_dod = sized_aircraft.max_dod # Assumes loaded correctly
        print(f"Params: Shorter Range={shorter_range_km} km, Max ABU={max_abu_mass} kg (Step={step_abu_mass} kg)")
        print(f"        Time Window=[{time_start:.0f}s - {time_end:.0f}s] (Step={time_step:.0f}s)")
    except KeyError as e:
        print(f"ERROR: Missing ABU optimization parameter in JSON: {e}")
        return None

    # 1. Define and run shorter mission without drop for baseline
    print("\nRunning shorter mission without ABU drop (Baseline)...")
    baseline_mission = ShorterRangeMission(
        aircraft=sized_aircraft,
        base_mission_segments=base_mission_segments,
        shorter_range_km=shorter_range_km
    )
    baseline_mission.run_mission()
    if not baseline_mission.is_calculated:
        print("ERROR: Baseline shorter mission failed to run.")
        return None
    baseline_shorter_energy = baseline_mission.total_energy_kwh
    print(f"Baseline Shorter Mission Energy: {baseline_shorter_energy:.3f} kWh")

    # 2. Get initial state & pre-calculate segment end times (using the baseline mission's results)
    original_mtow = sized_aircraft.mtow_kg
    original_batt_mass = sized_aircraft.battery.weight
    original_usable_EOL_kwh = sized_aircraft.battery.get_usable_EOL_capacity_kwh()
    original_BOL_kwh = sized_aircraft.battery.get_gross_BOL_capacity_kwh() # For C-rate checks
    usable_EOL_spec_energy_Wh_kg = sized_aircraft.battery.get_usable_EOL_spec_energy()

    if usable_EOL_spec_energy_Wh_kg <= 0:
        print("ERROR: Invalid battery usable EOL specific energy.")
        return None
    
    # Check for zero or negative original capacities to prevent division by zero later
    if original_usable_EOL_kwh <= 1e-9:
        print("ERROR: Original usable EOL capacity is zero or negative.")
        return None
    if original_BOL_kwh <= 1e-9:
        print("ERROR: Original BOL capacity is zero or negative.")
        return None


    segment_end_times = [0.0] * len(baseline_mission.mission_segments)
    cumulative_time = 0.0
    for i, result in enumerate(baseline_mission.segment_results):
         # Handle potential None duration if a segment failed in baseline
         duration = result.get('Duration (s)', 0.0)
         if duration is None: duration = 0.0
         cumulative_time += duration
         segment_end_times[i] = cumulative_time

    print(f"Segment End Times (Baseline Shorter Mission): {segment_end_times}")

    # 3. Initialize Optimization Tracking
    best_saving = -float('inf')
    optimal_result = {
        "effective_drop_time_s": None,
        "drop_after_segment_idx": None,
        "optimal_abu_mass_kg": 0.0,
        "min_energy_kwh": baseline_shorter_energy,
        "energy_saved_kwh": 0.0,
        "DoD_at_optimum": None,
    }
    last_processed_segment_idx = -1 # Track effective drop point changes

    # 4. Outer Loop: Iterate through nominal drop times to find effective segment boundaries
    nominal_drop_times = np.arange(time_start, time_end + time_step, time_step)
    print(f"\nTesting nominal drop times: {nominal_drop_times}")

    for t_nominal_drop in nominal_drop_times:
        # Find the index of the last segment completed *before* or *at* this nominal time
        effective_drop_segment_idx = -1
        for i, end_time in enumerate(segment_end_times):
            # Ensure we don't drop after the last segment before reserve
            # Allow drop after any segment except the very last one (reserve)
            if i >= len(baseline_mission.mission_segments) - 1:
                break
            if end_time <= t_nominal_drop:
                effective_drop_segment_idx = i
            else:
                break # Found the boundary

        # If this nominal time maps to the same effective drop segment as before, skip redundant calculations
        # Also skip if no valid segment index was found (e.g., time_start is before the end of the first segment)
        if effective_drop_segment_idx <= last_processed_segment_idx or effective_drop_segment_idx < 0:
            continue

        last_processed_segment_idx = effective_drop_segment_idx
        effective_drop_time_s = segment_end_times[effective_drop_segment_idx]
        segment_name = baseline_mission.mission_segments[effective_drop_segment_idx].segment_type
        print(f"\n--- Evaluating Effective Drop Point: After Segment {effective_drop_segment_idx} ({segment_name}) at t={effective_drop_time_s:.1f}s ---")

        # --- Simulate Pre-Drop ---
        print(f"  Simulating Pre-Drop (Segments 0 to {effective_drop_segment_idx})")
        energy_before_drop, _ = run_mission_subset(
            sized_aircraft, baseline_mission.mission_segments, 0, effective_drop_segment_idx,
            original_mtow, original_BOL_kwh
        )
        if energy_before_drop is None:
             print("  ERROR: Pre-drop simulation failed.")
             continue
        print(f"    Energy consumed before drop: {energy_before_drop:.3f} kWh")

        # --- Inner Loop: Iterate through ABU Mass Steps ---
        max_possible_abu = min(max_abu_mass, original_batt_mass)
        if step_abu_mass <= 0:
            print("  ERROR: ABU mass step must be positive.")
            continue
        if max_possible_abu < step_abu_mass:
             print(f"  No ABU mass steps to evaluate (Max possible {max_possible_abu:.2f} kg < Step {step_abu_mass:.2f} kg).")
             continue

        abu_mass_steps = np.arange(step_abu_mass, max_possible_abu + step_abu_mass, step_abu_mass)
        if abu_mass_steps[-1] > max_possible_abu + 1e-6:
            abu_mass_steps = abu_mass_steps[:-1]
            if max_possible_abu not in abu_mass_steps:
                 if not abu_mass_steps or abs(abu_mass_steps[-1] - max_possible_abu) > 1e-6:
                     abu_mass_steps = np.append(abu_mass_steps, max_possible_abu)

        if len(abu_mass_steps) == 0 : print("  No ABU mass steps to evaluate."); continue
        print(f"    Testing ABU Masses (kg): {abu_mass_steps}")

        for m_abu in abu_mass_steps:
            # --- Calculate Post-Drop State ---
            mtow_after_drop = original_mtow - m_abu
            capacity_dropped_usable_EOL_kwh = (m_abu * usable_EOL_spec_energy_Wh_kg) / 1000.0
            capacity_remaining_usable_EOL_kwh = original_usable_EOL_kwh - capacity_dropped_usable_EOL_kwh

            if original_usable_EOL_kwh <= 1e-9:
                 print(f"      Skipping m_abu={m_abu:.1f}kg due to zero original usable EOL capacity.")
                 continue
            bol_capacity_dropped_kwh = capacity_dropped_usable_EOL_kwh * (original_BOL_kwh / original_usable_EOL_kwh)
            capacity_remaining_BOL_kwh = original_BOL_kwh - bol_capacity_dropped_kwh


            if capacity_remaining_usable_EOL_kwh <= 1e-6:
                 continue
            if capacity_remaining_BOL_kwh <= 1e-6:
                 continue

            # --- Simulate Post-Drop ---
            print(f"      Testing m_abu={m_abu:.1f}kg:")
            start_segment_after_drop = effective_drop_segment_idx + 1
            end_segment_idx = min(len(baseline_mission.mission_segments) - 1, len(baseline_mission.mission_segments) -1)
            if start_segment_after_drop > end_segment_idx:
                print("      No segments remaining after drop point.")
                energy_after_drop_actual = 0.0
            else:
                energy_after_drop_actual, _ = run_mission_subset(
                    sized_aircraft, baseline_mission.mission_segments, start_segment_after_drop, end_segment_idx,
                    mtow_after_drop, capacity_remaining_BOL_kwh
                )

            if energy_after_drop_actual is None:
                print(f"      ERROR: Post-drop simulation failed for m_abu={m_abu:.1f}kg.")
                continue

            print(f"        Energy consumed after drop: {energy_after_drop_actual:.3f} kWh")

            # --- DoD Check ---
            if capacity_remaining_usable_EOL_kwh <= 1e-9:
                 print(f"        INVALID: Remaining usable EOL capacity is zero or negative for m_abu={m_abu:.1f}kg.")
                 continue

            actual_DoD_after_drop = energy_after_drop_actual / capacity_remaining_usable_EOL_kwh
            print(f"        DoD on remaining battery: {(actual_DoD_after_drop * 100):.1f}% (Max allowed: {(max_dod * 100):.1f}%)")

            if actual_DoD_after_drop > max_dod:
                 continue

            # --- Calculate Total Energy & Savings ---
            total_energy_with_drop = energy_before_drop + energy_after_drop_actual
            energy_saved = baseline_shorter_energy - total_energy_with_drop
            print(f"        VALID: Total energy={total_energy_with_drop:.3f} kWh, Saved={energy_saved:.3f} kWh")

            # --- Update Optimum ---
            if energy_saved > best_saving:
                best_saving = energy_saved
                optimal_result["effective_drop_time_s"] = effective_drop_time_s
                optimal_result["drop_after_segment_idx"] = effective_drop_segment_idx
                optimal_result["optimal_abu_mass_kg"] = m_abu
                optimal_result["min_energy_kwh"] = total_energy_with_drop
                optimal_result["energy_saved_kwh"] = energy_saved
                optimal_result["DoD_at_optimum"] = actual_DoD_after_drop
                print(f"        *** New optimal found! ***")

    # 5. Report Final Results
    print("\n" + "="*40)
    print("--- ABU Jettison Optimization Results ---")
    print("="*40)
    print(f"Baseline Shorter Mission Energy: {baseline_shorter_energy:.3f} kWh")
    if optimal_result["drop_after_segment_idx"] is None:
        print("\nNo beneficial ABU drop point/mass combination found.")
    else:
        idx = optimal_result["drop_after_segment_idx"]
        segment_name = baseline_mission.mission_segments[idx].segment_type
        print("\nOptimal Configuration Found:")
        print(f"  Drop After Segment: {idx} ({segment_name})")
        print(f"  Effective Drop Time:{optimal_result['effective_drop_time_s']:.1f} s")
        print(f"  Optimal ABU Mass:   {optimal_result['optimal_abu_mass_kg']:.2f} kg")
        print(f"  Min Energy w/ Drop: {optimal_result['min_energy_kwh']:.3f} kWh")
        print(f"  Max Energy Saved:   {optimal_result['energy_saved_kwh']:.3f} kWh")
        print(f"  Resulting DoD:      {(optimal_result['DoD_at_optimum'] * 100):.1f}% (Max: {(max_dod * 100):.1f}%)")

        # --- Add Post-Drop C-Rate Table ---
        print("\n--- Post-Drop C-Rate Analysis (Optimal Scenario) ---")

        opt_abu_mass = optimal_result['optimal_abu_mass_kg']
        opt_drop_idx = optimal_result['drop_after_segment_idx']

        mtow_after_opt_drop = original_mtow - opt_abu_mass
        capacity_dropped_usable_EOL_kwh_opt = (opt_abu_mass * usable_EOL_spec_energy_Wh_kg) / 1000.0

        if original_usable_EOL_kwh <= 1e-9:
            print("  Error: Original usable EOL capacity is zero. Cannot calculate C-rates.")
        else:
            bol_capacity_dropped_kwh_opt = capacity_dropped_usable_EOL_kwh_opt * (original_BOL_kwh / original_usable_EOL_kwh)
            capacity_remaining_BOL_kwh_opt = original_BOL_kwh - bol_capacity_dropped_kwh_opt

            if capacity_remaining_BOL_kwh_opt <= 1e-6:
                print("  Error: Optimal remaining BOL capacity is near zero. Cannot calculate C-rates.")
            else:
                start_segment_after_drop_opt = opt_drop_idx + 1
                end_segment_idx_opt = len(baseline_mission.mission_segments) - 1

                if start_segment_after_drop_opt > end_segment_idx_opt:
                     print("  No segments remaining after optimal drop point for C-rate analysis.")
                     post_drop_segment_results = []
                else:
                    _, post_drop_segment_results = run_mission_subset(
                        sized_aircraft, baseline_mission.mission_segments, start_segment_after_drop_opt, end_segment_idx_opt,
                        mtow_after_opt_drop, capacity_remaining_BOL_kwh_opt
                    )

                if post_drop_segment_results is not None:
                    c_rate_table_data = []
                    headers = ["Post-Drop Segment", "Avg Power (kW)", "Avg C-Rate (1/hr)"]

                    for i, result in enumerate(post_drop_segment_results):
                        segment_index = start_segment_after_drop_opt + i
                        if segment_index < len(baseline_mission.mission_segments):
                            segment_name = baseline_mission.mission_segments[segment_index].segment_type
                            avg_power_kw = result.get('Power (kW)', None)

                            if avg_power_kw is not None:
                                c_rate = avg_power_kw / capacity_remaining_BOL_kwh_opt
                                c_rate_table_data.append([f"{segment_index}: {segment_name}", f"{avg_power_kw:.2f}", f"{c_rate:.2f}"])
                            else:
                                c_rate_table_data.append([f"{segment_index}: {segment_name}", "N/A", "N/A"])
                        else:
                             print(f"Warning: Segment index {segment_index} out of bounds during C-rate table generation.")


                    if c_rate_table_data:
                        print(tabulate(c_rate_table_data, headers=headers, tablefmt="grid"))
                    elif start_segment_after_drop_opt <= end_segment_idx_opt:
                        print("  No valid power data found in post-drop segments to calculate C-rates.")
                else:
                    print("  Failed to simulate post-drop segments for C-rate analysis.")


        print("\nGenerating baseline mission profile plot...")
        import matplotlib.pyplot as plt
        if baseline_mission.is_calculated:
            time_s, power_kw = baseline_mission.get_power_profile()
            time_s_c, c_rate = baseline_mission.get_c_rate_profile()
            if time_s and power_kw and time_s_c and c_rate:
                fig, ax1 = plt.subplots(figsize=(10, 6))
                color = 'tab:blue'
                ax1.set_xlabel('Time (s)')
                ax1.set_ylabel('Power Draw (kW)', color=color)
                ax1.step(time_s, power_kw, where='post', color=color, linewidth=2, label='Power Profile (Baseline Short Mission)')
                ax1.tick_params(axis='y', labelcolor=color)
                ax1.grid(True, axis='y', linestyle=':', alpha=0.6)
                ax1.set_ylim(bottom=0)

                ax2 = ax1.twinx()
                color = 'tab:red'
                ax2.set_ylabel('C-Rate (Baseline)', color=color)
                ax2.step(time_s_c, c_rate, where='post', color=color, linestyle='--', linewidth=2, label='C-Rate Profile (Baseline)')
                ax2.tick_params(axis='y', labelcolor=color)
                ax2.set_ylim(bottom=0)

                if optimal_result["effective_drop_time_s"] is not None:
                    drop_time = optimal_result["effective_drop_time_s"]
                    ax1.axvline(x=drop_time, color='k', linestyle=':', linewidth=1.5, label=f'Optimal Drop Time ({drop_time:.1f}s)')


                plt.title('Baseline Short Mission Power and C-Rate Profile')
                lines, labels = ax1.get_legend_handles_labels()
                lines2, labels2 = ax2.get_legend_handles_labels()
                combined_lines = lines + lines2
                combined_labels = labels + labels2
                vline_label = f'Optimal Drop Time ({optimal_result["effective_drop_time_s"]:.1f}s)' if optimal_result["effective_drop_time_s"] is not None else None
                if vline_label:
                    vline_handle = next((line for line in ax1.get_lines() if line.get_linestyle() == ':'), None)
                    if vline_handle:
                         combined_lines.append(vline_handle)
                         combined_labels.append(vline_label)

                ax1.legend(combined_lines, combined_labels, loc='best')
                fig.tight_layout()
                plt.show()
                print("Plot generated.")
            else: print("Baseline mission profile data is empty. Cannot plot.")
        else: print("Baseline mission was not calculated successfully. Cannot plot profile.")



    print("="*40)
    return optimal_result

try:
     abu_results = optimize_abu_jettison(aircraft, mission_segments)
     if abu_results:
          pass
except Exception as e:
      print(f"\nERROR during ABU optimization execution: {e}")
      import traceback
      traceback.print_exc()
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

    # Intitialize components
    print("Initializing components...")
    
    # Load configuration to get rotor counts
    with open(path_to_json, "r") as f:
        config = json.load(f)
    
    lift_rotor_count = config.get("lift_rotor_count", 0)
    tilt_rotor_count = config.get("tilt_rotor_count", 0)
    total_rotors = lift_rotor_count + tilt_rotor_count
    config_rotor_count = config.get("rotor_count", 0)
    
    wing = Wing(path_to_json)
    fuselage = Fuselage(path_to_json)
    horizontaltail = HorizontalTail(path_to_json)
    verticaltail = VerticalTail(path_to_json)
    landinggear = LandingGear(path_to_json)
    battery_instance = Battery(path_to_json)
    
    # Initialize list with core components
    aircraft_components = [
        wing, fuselage, horizontaltail, verticaltail, landinggear, battery_instance
    ]
    
    # Add Tilt Rotors
    print(f"Adding {tilt_rotor_count} TiltRotors...")
    for _ in range(tilt_rotor_count):
        aircraft_components.append(TiltRotor(path_to_json))
    
    # Add Lift Rotors 
    print(f"Adding {lift_rotor_count} LiftRotors and {lift_rotor_count + tilt_rotor_count} Booms...")
    for _ in range(lift_rotor_count):
        aircraft_components.append(LiftRotor(path_to_json))
    
    # Add one boom per rotor (both lift and tilt)
    for _ in range(total_rotors):
        aircraft_components.append(Boom(path_to_json))

    for i, component in enumerate(aircraft_components):
        if not isinstance(component, AircraftComponent): 
            print(f"ERROR: Item at index {i} ({type(component).__name__}) is not a valid AircraftComponent.")
            sys.exit(1)
        component.load_variables_from_json()
    print(f"Components initialized: {len(aircraft_components)} total components")

    # 2. Aircraft 
    aircraft = Aircraft(components=aircraft_components, path_to_json=path_to_json)

    # 3. Mission Segments & Mission 
    mission_segments = [
        HoverSegment(path_to_json, segment_type="Takeoff Hover"),
        ClimbSegment(path_to_json),
        CruiseSegment(path_to_json),
        HoverSegment(path_to_json, segment_type="Landing Hover"),
        ReserveSegment(path_to_json)
    ]
    # Pass the aircraft instance to the Mission
    mission = Mission(aircraft=aircraft, mission_segments=mission_segments)


    # MTOW convergence using iterative sizer class
    try:
        sizer = IterativeSizer(
            config_path=path_to_json,
            max_iter=MAX_ITERATIONS,
            tolerance=TOLERANCE,
            damping=DAMPING_FACTOR
        )

        converged, final_mtow = sizer.run_sizing_loop(aircraft, mission)

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

                # Separate non-reserve energy and reserve energy
                non_reserve_energy_kwh = 0.0
                reserve_energy_kwh = 0.0
                
                for segment, result in zip(mission.mission_segments, mission.segment_results):
                    from missionsegment import ReserveSegment
                    if isinstance(segment, ReserveSegment):
                        reserve_energy_kwh += result.get('Energy (kWh)', 0.0)
                    else:
                        non_reserve_energy_kwh += result.get('Energy (kWh)', 0.0)
                
                # Apply max_dod only to non-reserve energy
                required_non_reserve_capacity_kwh = non_reserve_energy_kwh / aircraft.max_dod
                final_req_total_usable_kwh = required_non_reserve_capacity_kwh + reserve_energy_kwh
                
                print(f"  Non-Reserve Energy: {non_reserve_energy_kwh:.3f} kWh, Reserve Energy: {reserve_energy_kwh:.3f} kWh")
                print(f"  Required Capacity (with DoD={aircraft.max_dod:.2f}): {final_req_total_usable_kwh:.3f} kWh")
                
                final_batt_mass = (final_req_total_usable_kwh * W_PER_KW) / final_spec_e

                # Update the battery mass and internal state one last time
                aircraft.battery.final_batt_mass_kg = final_batt_mass
                aircraft.battery.UpdateComponent({'mtow_kg': final_mtow})
                print(f"  Final battery mass recalculated based on final energy: {final_batt_mass:.2f} kg")
                
                # Additional consistency loop to reconcile battery mass with MTOW
                print("\n--- Running consistency iterations after battery recalculation ---")
                consistency_iterations = 0
                max_consistency_iterations = 100
                consistency_tolerance = TOLERANCE
                previous_mtow = final_mtow
                
                while consistency_iterations < max_consistency_iterations:
                    # Recalculate MTOW with the updated battery mass
                    new_mtow = aircraft.CalculateMTOW()
                    mtow_diff = abs(new_mtow - previous_mtow)
                    print(f"  Consistency iter {consistency_iterations+1}: MTOW = {new_mtow:.2f} kg (Δ = {mtow_diff:.2f} kg)")
                    
                    if mtow_diff < consistency_tolerance:
                        print(f"  Consistency achieved! Final MTOW: {new_mtow:.2f} kg")
                        final_mtow = new_mtow
                        break
                    
                    # Update aircraft state with the new MTOW
                    aircraft.update_state_for_iteration(new_mtow)
                    
                    # Run mission again to get updated energy requirements
                    mission.run_mission()
                    if not mission.is_calculated:
                        print("  Warning: Mission calculation failed during consistency iteration")
                        break
                        
                    # Calculate new battery mass based on updated mission energy
                    non_reserve_energy_kwh = 0.0
                    reserve_energy_kwh = 0.0
                    for segment, result in zip(mission.mission_segments, mission.segment_results):
                        if isinstance(segment, ReserveSegment):
                            reserve_energy_kwh += result.get('Energy (kWh)', 0.0)
                        else:
                            non_reserve_energy_kwh += result.get('Energy (kWh)', 0.0)
                    
                    req_total_usable_kwh = (non_reserve_energy_kwh / aircraft.max_dod) + reserve_energy_kwh
                    new_batt_mass = (req_total_usable_kwh * W_PER_KW) / final_spec_e
                    
                    print(f"    Updated energy: {non_reserve_energy_kwh + reserve_energy_kwh:.2f} kWh")
                    print(f"    Updated battery mass: {new_batt_mass:.2f} kg (previous: {aircraft.battery.weight:.2f} kg)")
                    
                    # Update battery mass
                    aircraft.battery.final_batt_mass_kg = new_batt_mass
                    aircraft.battery.UpdateComponent({'mtow_kg': new_mtow})
                    
                    previous_mtow = new_mtow
                    consistency_iterations += 1
                
                if consistency_iterations == max_consistency_iterations:
                    print(f"  Reached max consistency iterations. Final MTOW: {new_mtow:.2f} kg")
                    final_mtow = new_mtow
                
            else:
                 print("  Warning: Could not perform final battery mass recalculation due to missing data.")


            # Final check of calculated MTOW vs the converged/final value using the sizer
            final_mtow_check, mtow_consistent = sizer.verify_final_mtow(aircraft, final_mtow, tolerance_factor=2.0)

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
            
            # Initialize total weights for rotors and booms
            lift_rotors_total_kg = 0.0
            tilt_rotors_total_kg = 0.0
            booms_total_kg = 0.0
            
            # Group components by type
            for comp in aircraft.components:
                try:
                    # Use the 'weight' attribute directly after UpdateComponent calls
                    w = comp.weight if hasattr(comp, 'weight') else 0.0
                    
                    # Group rotors and booms
                    if isinstance(comp, LiftRotor):
                        lift_rotors_total_kg += w
                    elif isinstance(comp, TiltRotor):
                        tilt_rotors_total_kg += w
                    elif isinstance(comp, Boom):
                        booms_total_kg += w
                    else:
                        # Add other components individually
                        weight_data.append([type(comp).__name__, f"{w:.2f}"])
                        calculated_sum += w
                except Exception as e:
                    print(f"Warning: Error getting weight for {type(comp).__name__}: {e}")
                    weight_data.append([type(comp).__name__, "Error"])
            
            # Add grouped components to weight data
            if lift_rotors_total_kg > 0:
                weight_data.append(["LiftRotors (Total)", f"{lift_rotors_total_kg:.2f}"])
                calculated_sum += lift_rotors_total_kg
            if tilt_rotors_total_kg > 0:
                weight_data.append(["TiltRotors (Total)", f"{tilt_rotors_total_kg:.2f}"])
                calculated_sum += tilt_rotors_total_kg
            if booms_total_kg > 0:
                weight_data.append(["Booms (Total)", f"{booms_total_kg:.2f}"])
                calculated_sum += booms_total_kg

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

# Performs the complete aircraft sizing process and returns the sized aircraft and mission


import json

import traceback
import argparse 
from tabulate import tabulate 
import missionsegment
from missionsegment import Mission, HoverSegment, ClimbSegment, CruiseSegment, ReserveSegment, ShorterRangeMission
from base_component import AircraftComponent 
from aircraft import Aircraft, Wing, Fuselage, HorizontalTail, VerticalTail, LandingGear, Boom, LiftRotor, TiltRotor
from battery import Battery
from iterative_size import IterativeSizer
import copy
import numpy as np
from tabulate import tabulate


W_PER_KW = 1000.0
DEFAULT_JSON_PATH = "evtol-param.json"
def size_aircraft(path_to_json, max_iterations=1000, tolerance=0.1, damping=0.3, display_results=True):
    if display_results:
        print("--- eVTOL Aircraft Sizing Simulation ---")
        print(f"Using configuration file: {path_to_json}")
        print(f"Convergence Params: Max Iter={max_iterations}, Tolerance={tolerance} kg, Damping={damping}")
        print("-" * 40)

    # Initialize components
    if display_results:
        print("Initializing components...")
    
    # Load configuration to get rotor counts
    with open(path_to_json, "r") as f:
        config = json.load(f)
    
    lift_rotor_count = config.get("lift_rotor_count")
    tilt_rotor_count = config.get("tilt_rotor_count")
    total_rotors = lift_rotor_count + tilt_rotor_count
    
    wing = Wing(path_to_json)
    fuselage = Fuselage(path_to_json)
    horizontaltail = HorizontalTail(path_to_json)
    verticaltail = VerticalTail(path_to_json)
    landinggear = LandingGear(path_to_json)
    battery_instance = Battery(path_to_json)
    
    aircraft_components = [
        wing, fuselage, horizontaltail, verticaltail, landinggear, battery_instance
    ]
    
    # Add Tilt Rotors
    for _ in range(tilt_rotor_count):
        aircraft_components.append(TiltRotor(path_to_json))
    
    # Add Lift Rotors 
    for _ in range(lift_rotor_count):
        aircraft_components.append(LiftRotor(path_to_json))
    
    # Add one boom per rotor (to both lift and tilt)
    for _ in range(total_rotors):
        aircraft_components.append(Boom(path_to_json))

    for i, component in enumerate(aircraft_components):
        component.load_variables_from_json()
    
    if display_results:
        print(f"Components initialized: {len(aircraft_components)} total components")

    # Aircraft 
    aircraft = Aircraft(components=aircraft_components, path_to_json=path_to_json)

    # Mission Segments & Mission 
    mission_segments = [
        HoverSegment(path_to_json, segment_type="Takeoff Hover"),
        ClimbSegment(path_to_json),
        CruiseSegment(path_to_json),
        HoverSegment(path_to_json, segment_type="Landing Hover"),
        ReserveSegment(path_to_json)
    ]
    # Pass the aircraft into Mission
    mission = Mission(aircraft=aircraft, mission_segments=mission_segments)

    # MTOW convergence using iterative sizer class
    try:
        sizer = IterativeSizer(
            config_path=path_to_json,
            max_iter=max_iterations,
            tolerance=tolerance,
            damping=damping
        )

        converged, final_mtow = sizer.run_sizing_loop(aircraft, mission)
        
        if display_results:
            print(f"\nFinal MTOW (Converged={converged}): {final_mtow:.2f} kg")
            print("-" * 60)

        # --- Final Update & Results Display ---
        print("Performing final state update with converged/final MTOW...")
        # Final update of all components based on the final MTOW
        aircraft.update_state_for_iteration(final_mtow)

        # Re-run mission and resize battery one last time based on final MTOW
        aircraft.battery.UpdateComponent({'mtow_kg': final_mtow})
        print(f"  Final battery state updated. BOL Cap: {aircraft.battery.get_gross_BOL_capacity_kwh():.2f} kWh")
        
        # Recalculate MTOW based on updated component weights including adjusted battery mass
        final_mtow = aircraft.CalculateMTOW()
        print(f"  Recalculated final MTOW after battery update: {final_mtow:.2f} kg")
        
        # CRITICAL FIX: Update the aircraft state again with the recalculated MTOW
        aircraft.mtow_kg = final_mtow  # Ensure aircraft object has the updated MTOW
        aircraft.update_state_for_iteration(final_mtow)  # Update all components with new MTOW
        print(f"  Aircraft state updated with recalculated MTOW: {final_mtow:.2f} kg")
        
        # Re-run mission with the final aircraft state using updated MTOW
        print(f"  Running mission simulation with final MTOW: {final_mtow:.2f} kg")
        mission.run_mission()
        
        # Make a FINAL MTOW calculation after running the mission to ensure 
        # it captures any components that may have been updated during the mission
        final_mtow = aircraft.CalculateMTOW()
        print(f"  Final MTOW after mission simulation: {final_mtow:.2f} kg")
        
        # Add a final convergence loop to ensure MTOW is completely stable
        print("\n=== Starting Final MTOW Convergence Loop ===")
        final_convergence_iters = 0
        final_max_iters = 5  # Usually 2-3 iterations should be enough
        final_mtow_prev = final_mtow
        final_mtow_converged = False
        
        while final_convergence_iters < final_max_iters and not final_mtow_converged:
            # Update aircraft state with latest MTOW
            print(f"  Final convergence iteration {final_convergence_iters + 1}...")
            aircraft.mtow_kg = final_mtow
            aircraft.update_state_for_iteration(final_mtow)
            
            # Re-run the mission
            print(f"  Re-running mission with MTOW: {final_mtow:.2f} kg")
            mission.run_mission()
            
            # Recalculate MTOW
            new_final_mtow = aircraft.CalculateMTOW()
            mtow_diff = abs(new_final_mtow - final_mtow_prev)
            print(f"  Updated MTOW: {new_final_mtow:.2f} kg (change: {mtow_diff:.2f} kg)")
            
            # Check for convergence
            if mtow_diff < tolerance:
                print(f"  Final MTOW convergence achieved after {final_convergence_iters + 1} iterations")
                final_mtow_converged = True
            
            final_mtow = new_final_mtow
            final_mtow_prev = final_mtow
            final_convergence_iters += 1
        
        if not final_mtow_converged:
            print(f"  Warning: Final MTOW did not fully converge after {final_max_iters} iterations")
            print(f"  Using final value: {final_mtow:.2f} kg")
        
        print("=== Final MTOW Convergence Loop Complete ===\n")
        
        return aircraft, mission, final_mtow, converged

    except Exception as err:
        if display_results:
            print(f"\nERROR during sizing: {err}")
            traceback.print_exc()
        return None, None, None, False

def display_final_results(aircraft, mission, final_mtow):
    """Display detailed results for the sized aircraft"""
    print("\n" + "="*60)
    print("          FINAL AIRCRAFT CONFIGURATION & PERFORMANCE")
    print("="*60)

    # Mission Summary
    print("\n--- Final Mission Simulation Results ---")
    mission.display_summary()

    # Drag Coefficients
    print("\n--- Final Cruise Aerodynamic Performance ---")
    final_wing = next((c for c in aircraft.components if isinstance(c, Wing)), None)
    if final_wing and final_wing.wing_ref_area_m2 > 0:
        final_cruise_speed = aircraft._varlist['cruise_speed_m_p_s']
        final_rho_cruise = aircraft._varlist.get('air_density_cruise_kg_p_m3')
        g_m_p_s2 = aircraft._varlist.get('g_m_p_s2', 9.81)

        final_cl = aircraft.calculate_cruise_CL(final_mtow * g_m_p_s2, final_rho_cruise, final_cruise_speed, final_wing.wing_ref_area_m2)
        final_cdi = aircraft.calc_Cdi(aircraft.spac_effic_factor, final_wing.wing_AR, final_cl)
        final_cd0_p_total = aircraft.Cd0_total_parasite
        final_cd = (final_cd0_p_total + final_cdi) * aircraft.trim_drag_factor
        final_ld = final_cl / final_cd if final_cd > 1e-9 else float('inf')

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
    print(f"Final MTOW:     {final_mtow:.2f} kg")
    
    # Battery Details
    if hasattr(aircraft, 'battery') and aircraft.battery and isinstance(aircraft.battery, Battery):
        print("\n--- Final Battery Properties ---")
        batt = aircraft.battery

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
    else:
        print("\n--- Final Battery Properties --- \n  Battery details not available.")

    # Plotting
    print("\nGenerating mission profile plot...")
    try:
        import matplotlib.pyplot as plt
        if mission.is_calculated:
            time_s, power_kw = mission.get_power_profile()
            time_s_c, c_rate = mission.get_c_rate_profile()
            if time_s and power_kw and time_s_c and c_rate:
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
            else:
                print("Mission profile data is empty. Cannot plot.")
        else:
            print("Mission was not calculated successfully. Cannot plot profile.")
    except ImportError:
        print("Matplotlib not found. Skipping plot generation. Install with: pip install matplotlib")
    except Exception as plot_err:
        print(f"Error during plot generation: {plot_err}")

# Main execution when run as script
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="eVTOL Aircraft Sizing Simulation")
    parser.add_argument(
        "-c", "--config",
        default=DEFAULT_JSON_PATH,
        help=f"Path to the JSON configuration file (default: {DEFAULT_JSON_PATH})"
    )
    parser.add_argument(
        "-i", "--max_iter",
        type=int, default=10000
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

    try:
        aircraft, mission, final_mtow, converged = size_aircraft(
            args.config, 
            args.max_iter, 
            args.tolerance, 
            args.damping,
            display_results=True
        )
        
        if not converged:
            print("Warning: Sizing process did not fully converge.")
    except Exception as main_err:
        print(f"\nFATAL ERROR during main execution: {main_err}")
        traceback.print_exc()
    finally:
        print("\n--- End of Aircraft Sizing Simulation ---")
        display_final_results(aircraft, mission, final_mtow)



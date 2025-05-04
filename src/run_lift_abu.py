# --- START OF FILE run_lifting_abu.py ---

import json
import math
import sys
import traceback
import argparse
import copy
from tabulate import tabulate

# --- Component and System Imports ---
from base_component import AircraftComponent
from battery import Battery
from aircraft import (Aircraft, Wing, Fuselage, HorizontalTail, VerticalTail,
                      LandingGear, Boom, LiftRotor, TiltRotor)
from lift_ABU import LiftABU
from lifting_abu_manager import LiftingABUManager
from missionsegment import (Mission, MissionSegment, HoverSegment, ClimbSegment,
                              CruiseSegment, ReserveSegment)
# --- Import Sizer ---
from iterative_size import IterativeSizer

# --- Matplotlib (Optional) ---
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

DEFAULT_JSON_PATH = "evtol-param.json"
W_PER_KW = 1000.0 # Add constant if not imported elsewhere

# --- Helper Function: Build Initial Components (Keep as is) ---
def build_initial_component_list(config_path: str, config: dict) -> list[AircraftComponent]:
    # (Code from previous answer - builds the initial list based on config counts)
    components = []
    try:
        # Core components
        components.append(Wing(config_path))
        components.append(Fuselage(config_path))
        components.append(HorizontalTail(config_path))
        components.append(VerticalTail(config_path))
        components.append(LandingGear(config_path))
        components.append(Battery(config_path)) # Main battery

        # Propulsion components based on counts
        lift_rotor_count = config.get("lift_rotor_count", 0)
        tilt_rotor_count = config.get("tilt_rotor_count", 0)
        total_defined_rotors = lift_rotor_count + tilt_rotor_count
        config_rotor_count = config.get("rotor_count", 0)
        if total_defined_rotors != config_rotor_count:
             print(f"Warning: lift_rotor_count ({lift_rotor_count}) + tilt_rotor_count ({tilt_rotor_count}) != rotor_count ({config_rotor_count}) in JSON.")
             print(f"         Using lift={lift_rotor_count}, tilt={tilt_rotor_count} for component creation.")


        # Add Tilt Rotors
        for _ in range(tilt_rotor_count):
            components.append(TiltRotor(config_path))
            # Assuming one boom per tilt rotor? Add logic if needed

        # Add Lift Rotors and corresponding Booms
        for i in range(lift_rotor_count):
            components.append(LiftRotor(config_path))
            components.append(Boom(config_path)) # Assume one boom per lift rotor

        # Load variables for all components
        for comp in components:
            if not isinstance(comp, AircraftComponent):
                raise TypeError(f"Item {type(comp).__name__} is not an AircraftComponent.")
            comp.load_variables_from_json()

        print(f"Built initial list with {len(components)} components.")
        return components

    except KeyError as e:
        print(f"ERROR: Missing key {e} in JSON config while building components.")
        traceback.print_exc()
        return None
    except Exception as e:
        print(f"ERROR building component list: {e}")
        traceback.print_exc()
        return None

# --- Helper Function: Display Results (Keep mostly as is, maybe add title arg) ---
def display_detailed_results(aircraft: Aircraft, mission: Mission, title: str):
    # (Code from previous answer - displays mission summary, state, weights, plot)
    print("\n" + "="*60)
    print(f"          {title}")
    print("="*60)

    if mission and mission.is_calculated:
        print("\n--- Mission Simulation Results ---")
        mission.display_summary()
        valid_mission = True
    else:
        print("\n--- Mission Simulation Results ---")
        print(" Mission failed or was not calculated.")
        valid_mission = False

    # --- Display State ---
    print("\n--- Aircraft State ---")
    state_data = [
        ["MTOW", f"{aircraft.mtow_kg:.2f} kg"],
        ["Cruise L/D", f"{aircraft.cruise_L_p_D:.2f}"],
        ["Parasite Drag (Cd0)", f"{aircraft.Cd0_total_parasite:.5f}"],
        ["Total Disk Area", f"{aircraft.get_rotor_disk_area_m2():.2f} m^2"],
        ["Main Batt Mass", f"{aircraft.battery.weight:.2f} kg"],
        ["Total Usable EOL Cap", f"{aircraft.get_current_usable_EOL_kwh():.2f} kWh"],
        ["Total Gross BOL Cap", f"{aircraft.get_current_gross_BOL_kwh():.2f} kWh"],
    ]
    # Add LiftABU specific info if manager exists
    if hasattr(aircraft, 'lifting_abu_manager') and aircraft.lifting_abu_manager and aircraft.lifting_abu_manager.is_configured:
         num_abus = len(aircraft.lifting_abu_manager.lift_abu_units)
         abu_mass_each = aircraft.lifting_abu_manager.abu_mass_per_unit_kg
         state_data.append(["Num LiftABUs", f"{num_abus}"])
         state_data.append(["ABU Batt Mass/Unit", f"{abu_mass_each:.2f} kg"])
         # Check if jettisoned
         jettisoned = any(unit.is_jettisoned for unit in aircraft.lifting_abu_manager.lift_abu_units)
         state_data.append(["LiftABUs Jettisoned", f"{jettisoned}"])


    print(tabulate(state_data, tablefmt="plain"))

    # --- Component Weights ---
    print("\n--- Component Weights ---")
    weight_data = []
    calculated_sum = 0.0
    try:
        total_payload_kg = getattr(aircraft, 'total_payload_kg', 0.0)
        fixed_systems_kg = getattr(aircraft, 'fixed_systems_kg', 0.0)

        for comp in aircraft.components:
             w = comp.get_weight_kg()
             # Add identifier if it's a LiftABU
             comp_name = type(comp).__name__
             if isinstance(comp, LiftABU):
                 comp_name += f" ({comp.unit_id})" + (" [Jettisoned]" if comp.is_jettisoned else "")
             weight_data.append([comp_name, f"{w:.2f}"])
             calculated_sum += w

        weight_data.append(["Total Payload", f"{total_payload_kg:.2f}"])
        weight_data.append(["Fixed Systems", f"{fixed_systems_kg:.2f}"])
        calculated_sum += total_payload_kg + fixed_systems_kg

        print(tabulate(weight_data, headers=["Component/Item", "Weight (kg)"], tablefmt="grid"))
        print(f"\nSum of Weights: {calculated_sum:.2f} kg (Should match MTOW above)")
        if abs(calculated_sum - aircraft.mtow_kg) > 1.0: # Check consistency
            print("Warning: Sum of weights differs significantly from aircraft MTOW.")

    except Exception as e:
        print(f"Error displaying weights: {e}")
        traceback.print_exc()

    # --- Plotting ---
    if valid_mission and MATPLOTLIB_AVAILABLE:
        print("\nGenerating mission profile plot...")
        try:
            time_s, power_kw = mission.get_power_profile()
            time_s_c, c_rate = mission.get_c_rate_profile()
            if time_s and power_kw and time_s_c and c_rate:
                fig, ax1 = plt.subplots(figsize=(10, 6))
                power_color = 'tab:blue'
                crate_color = 'tab:red'
                # Change color/style based on whether jettison happened in *this* mission run
                label_suffix = ""
                if hasattr(aircraft, 'lifting_abu_manager') and aircraft.lifting_abu_manager and any(u.is_jettisoned for u in aircraft.lifting_abu_manager.lift_abu_units):
                    power_color = 'tab:green'
                    crate_color = 'tab:orange'
                    label_suffix = " (Jettison)"


                ax1.set_xlabel('Time (s)')
                ax1.set_ylabel('Power Draw (kW)', color=power_color)
                ax1.step(time_s, power_kw, where='post', color=power_color, linewidth=2, label=f'Power{label_suffix}')
                ax1.tick_params(axis='y', labelcolor=power_color)
                ax1.grid(True, axis='y', linestyle=':', alpha=0.6)
                ax1.set_ylim(bottom=0)

                ax2 = ax1.twinx()
                ax2.set_ylabel('C-Rate', color=crate_color)
                ax2.step(time_s_c, c_rate, where='post', color=crate_color, linestyle='--', linewidth=2, label=f'C-Rate{label_suffix}')
                ax2.tick_params(axis='y', labelcolor=crate_color)
                ax2.set_ylim(bottom=0)

                # Add jettison line if applicable
                if hasattr(aircraft, 'lifting_abu_manager') and aircraft.lifting_abu_manager and aircraft.lifting_abu_manager.is_configured:
                     jettison_time = aircraft.lifting_abu_manager.jettison_time_s
                     if jettison_time < float('inf'):
                          ax1.axvline(x=jettison_time, color='k', linestyle=':', linewidth=1.5, label=f'Jettison Time ({jettison_time:.1f}s)')

                plt.title(f'{title} - Mission Profile')
                lines, labels = ax1.get_legend_handles_labels()
                lines2, labels2 = ax2.get_legend_handles_labels()
                # Avoid duplicate legend labels if colors are the same
                unique_labels = {}
                for line, label in zip(lines + lines2, labels + labels2):
                    if label not in unique_labels:
                         unique_labels[label] = line
                ax1.legend(unique_labels.values(), unique_labels.keys(), loc='best')

                fig.tight_layout()
                plt.show()
                print("Plot generated.")
            else: print("Mission profile data is empty. Cannot plot.")
        except Exception as plot_err:
            print(f"Error during plot generation: {plot_err}")
            traceback.print_exc()
    elif valid_mission and not MATPLOTLIB_AVAILABLE:
        print("\nMatplotlib not found. Skipping plot generation.")


# --- Main Execution ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="eVTOL Sizing and Lifting ABU Scenario Simulation")
    parser.add_argument(
        "-c", "--config",
        default=DEFAULT_JSON_PATH,
        help=f"Path to the JSON configuration file (default: {DEFAULT_JSON_PATH})"
    )
    # Add sizing arguments
    parser.add_argument(
        "-i", "--max_iter", type=int, default=100, help="Max iterations for sizing loop"
    )
    parser.add_argument(
        "-t", "--tolerance", type=float, default=0.1, help="Convergence tolerance for MTOW (kg)"
    )
    parser.add_argument(
        "-d", "--damping", type=float, default=0.3, help="Damping factor for MTOW updates"
    )
    args = parser.parse_args()
    path_to_json = args.config

    print("--- eVTOL Sizing and Lifting ABU Scenario Simulation ---")
    print(f"Using configuration file: {path_to_json}")
    print(f"Sizing Params: Max Iter={args.max_iter}, Tolerance={args.tolerance} kg, Damping={args.damping}")
    print("-" * 50)

    aircraft = None
    abu_manager = None
    final_mtow = None
    converged = False

    try:
        # === Initialization Phase ===
        print("\n=== Phase 1: Initialization ===")
        with open(path_to_json, "r") as f:
            config_data = json.load(f)

        print("Building initial aircraft components...")
        initial_components = build_initial_component_list(path_to_json, config_data)
        if initial_components is None:
            sys.exit(1)

        aircraft = Aircraft(components=initial_components, path_to_json=path_to_json)

        mission_segments = [
            HoverSegment(path_to_json, segment_type="Takeoff Hover"),
            ClimbSegment(path_to_json),
            CruiseSegment(path_to_json),
            HoverSegment(path_to_json, segment_type="Landing Hover"),
            ReserveSegment(path_to_json)
        ]
        mission = Mission(aircraft=aircraft, mission_segments=mission_segments)
        print("Initialization complete.")


        # === Sizing Phase ===
        print("\n=== Phase 2: Iterative Sizing ===")
        sizer = IterativeSizer(
            config_path=path_to_json,
            max_iter=args.max_iter,
            tolerance=args.tolerance,
            damping=args.damping
        )
        converged, final_mtow = sizer.run_sizing_loop(aircraft, mission) # Modifies 'aircraft' in place

        if final_mtow is None:
             print("ERROR: Sizing loop failed to produce a final MTOW estimate.")
             sys.exit(1)
        print(f"Sizing Complete. Converged: {converged}, Final MTOW: {final_mtow:.2f} kg")

        # --- Final Update & Display for Sized Aircraft ---
        print("\nPerforming final update on sized aircraft...")
        aircraft.update_state_for_iteration(final_mtow) # Update state based on final MTOW
        # Final battery adjustment based on last mission run in sizer
        # The sizer should leave the battery mass correctly set. UpdateComponent ensures cell count is right.
        aircraft.battery.UpdateComponent({'mtow_kg': final_mtow})

        # Rerun mission on final sized state for display
        final_sized_mission = Mission(aircraft, copy.deepcopy(mission_segments))
        final_sized_mission.run_mission()

        # Optional final battery mass adjustment (can sometimes refine slightly)
        if final_sized_mission.is_calculated and converged: # Only adjust if converged
            print("Performing optional final battery mass adjustment...")
            final_energy = final_sized_mission.total_energy_kwh
            final_spec_e = aircraft.battery.get_usable_EOL_spec_energy()
            if (hasattr(aircraft, 'max_dod') and aircraft.max_dod > 0 and
                final_energy is not None and final_spec_e > 0):
                 final_req_total_usable_kwh = final_energy / aircraft.max_dod
                 final_batt_mass = (final_req_total_usable_kwh * W_PER_KW) / final_spec_e
                 mass_diff = abs(final_batt_mass - aircraft.battery.final_batt_mass_kg)
                 if mass_diff > args.tolerance * 0.5: # Only update if diff is significant
                     print(f"  Adjusting main battery mass from {aircraft.battery.final_batt_mass_kg:.2f} kg to {final_batt_mass:.2f} kg.")
                     aircraft.battery.final_batt_mass_kg = final_batt_mass
                     aircraft.battery.UpdateComponent({'mtow_kg': final_mtow})
                     aircraft.mtow_kg = aircraft.CalculateMTOW() # Recalculate MTOW
                     final_mtow = aircraft.mtow_kg
                     print(f"  Adjusted Final MTOW: {final_mtow:.2f} kg")
                     # Rerun final update and mission with this adjusted mass
                     print("  Re-running final update and mission...")
                     aircraft.update_state_for_iteration(final_mtow)
                     final_sized_mission = Mission(aircraft, copy.deepcopy(mission_segments))
                     final_sized_mission.run_mission()
                 else:
                      print("  Final battery mass adjustment deemed unnecessary.")
            else:
                 print("  Warning: Could not perform final battery mass recalc.")


        # Display results for the purely sized aircraft
        display_detailed_results(aircraft, final_sized_mission, "SIZED AIRCRAFT CONFIGURATION (PRE-ABU SCENARIO)")


        # === Lifting ABU Scenario Phase ===
        print("\n=== Phase 3: Lifting ABU Scenario Analysis ===")
        if not config_data.get("enable_lifting_abu_scenario", False):
            print("Lifting ABU scenario disabled in JSON config. Skipping Phase 3.")
        else:
            # IMPORTANT: The 'aircraft' object now holds the fully sized state.
            # The LiftingABUManager will modify this object directly.
            print("Instantiating Lifting ABU Manager...")
            abu_manager = LiftingABUManager(aircraft, path_to_json) # Modifies 'aircraft'
            aircraft.lifting_abu_manager = abu_manager # Link manager back

            if abu_manager.is_configured:
                # --- Run Baseline (No Jettison, ABU-configured Aircraft) ---
                print("\n--- Running Baseline Mission (LiftABUs Attached, No Jettison) ---")
                # Need a fresh mission object using the *modified* aircraft
                baseline_abu_mission = Mission(aircraft, copy.deepcopy(mission_segments))

                # Reset jettison state & ensure MTOW/state is current post-config
                for unit in abu_manager.lift_abu_units: unit.is_jettisoned = False
                aircraft.mtow_kg = aircraft.CalculateMTOW() # MTOW with LiftABUs
                aircraft.update_state_for_iteration(aircraft.mtow_kg)

                baseline_abu_mission.run_mission() # Use standard mission run
                baseline_energy = baseline_abu_mission.total_energy_kwh if baseline_abu_mission.is_calculated else None

                # Display Baseline Results (aircraft state now includes ABUs)
                display_detailed_results(aircraft, baseline_abu_mission, "LIFTING ABU - BASELINE (NO JETTISON)")


                # --- Run Mission WITH Jettison ---
                print("\n--- Running Mission WITH LiftABU Jettison ---")
                # Use the manager's dedicated run method
                jettison_mission = abu_manager.run_mission_with_jettison(mission_segments)
                jettison_energy = jettison_mission.total_energy_kwh if jettison_mission and jettison_mission.is_calculated else None

                # Display Jettison Results (aircraft state reflects post-jettison)
                display_detailed_results(aircraft, jettison_mission, "LIFTING ABU - WITH JETTISON")


                # --- Compare Results ---
                print("\n" + "="*60)
                print("          SCENARIO COMPARISON")
                print("="*60)
                if baseline_energy is not None and jettison_energy is not None:
                     saved_energy = baseline_energy - jettison_energy
                     print(f"Baseline Energy (LiftABUs Attached): {baseline_energy:.3f} kWh")
                     print(f"Jettison Mission Energy:             {jettison_energy:.3f} kWh")
                     print(f"Energy Saved by Jettison:            {saved_energy:.3f} kWh")
                else:
                     print("Could not calculate energy savings (one or both missions failed).")

            else:
                print("\nLifting ABU scenario failed configuration. No simulation run for this phase.")

    except FileNotFoundError:
        print(f"ERROR: Configuration file not found at '{path_to_json}'")
    except KeyError as e:
        print(f"\nFATAL ERROR: Missing required key in JSON configuration: {e}")
        traceback.print_exc()
    except Exception as e:
        print(f"\nFATAL ERROR during execution: {e}")
        traceback.print_exc()
    finally:
        print("\n--- End of Simulation ---")
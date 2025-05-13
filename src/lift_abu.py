import argparse
from tabulate import tabulate
import copy
import traceback

# Import everything needed explicitly
from missionsegment import (
    Mission, HoverSegment, ClimbSegment, CruiseSegment, ReserveSegment, 
    ShorterRangeMission, JettisonCruiseSegment, JettisonClimbSegment
)
from sized_aircraft import size_aircraft, DEFAULT_JSON_PATH, W_PER_KW

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="eVTOL Aircraft Baseline Mission Simulation (via lift_abu.py)")
    parser.add_argument(
        "-c", "--config",
        default=DEFAULT_JSON_PATH,
        help=f"Path to the JSON configuration file (default: {DEFAULT_JSON_PATH})"
    )
    parser.add_argument(
        "-i", "--max_iter",
        type=int, default=1000,
        help="Maximum iterations for sizing loop"
    )
    parser.add_argument(
        "-t", "--tolerance",
        type=float, default=0.1,
        help="Convergence tolerance for MTOW (kg)"
    )
    parser.add_argument(
        "-d", "--damping",
        type=float, default=0.3,
        help="Damping factor for sizing loop"
    )
    args = parser.parse_args()

    path_to_json = args.config
    MAX_ITERATIONS = args.max_iter
    TOLERANCE = args.tolerance
    DAMPING_FACTOR = args.damping

    print("--- eVTOL Aircraft Baseline Mission Simulation (from lift_abu.py) ---")
    print(f"Using configuration file: {path_to_json}")

    try:
        # Get the sized aircraft from sized_aircraft module
        print(f"Getting sized aircraft from sized_aircraft module...")
        aircraft, mission, final_mtow, converged = size_aircraft(
            path_to_json, 
            MAX_ITERATIONS, 
            TOLERANCE, 
            DAMPING_FACTOR,
            display_results=False  # No need to display detailed results here
        )
        
        if aircraft is None or mission is None:
            print("ERROR: Failed to get sized aircraft from sized_aircraft module.")
            import sys
            sys.exit(1)
            
        print(f"Successfully obtained sized aircraft with MTOW={final_mtow:.2f} kg (Converged={converged})")
        print("-" * 60)
        
        # Run the baseline mission with the sized aircraft
        print("\nRunning baseline mission with sized aircraft...")
        mission.run_mission()
        
        if not mission.is_calculated:
            print("Warning: Baseline mission run failed after sizing.")
        else:
            print("\n--- Baseline Mission Simulation Results ---")
            mission.display_summary()
            
            # Run ABU jettison mission at cruise start (original approach)
            print("\n--- Creating and Running ABU Jettison at Cruise Mission ---")
            
            # Configuration for jettisoning components
            jettison_rotor_type_to_drop = "lift"  # Options: "lift" or "tilt"
            jettison_rotor_number_to_drop = 2     # Number of selected rotors to jettison
            battery_mass_to_jettison_kg = 500.0   # Mass of battery to jettison

            jettison_config = {
                "boom_count": jettison_rotor_number_to_drop,  # Assuming one boom per jettisoned rotor
                "jettison_rotor_type": jettison_rotor_type_to_drop,
                "jettison_rotor_count": jettison_rotor_number_to_drop,
                "battery_mass_kg": battery_mass_to_jettison_kg
            }

            print(f"This mission will jettison {jettison_config['boom_count']} booms, "
                  f"{jettison_config['jettison_rotor_count']} {jettison_config['jettison_rotor_type']} rotors, "
                  f"and {jettison_config['battery_mass_kg']:.1f} kg of battery mass before cruise")
            
            # Create jettison mission segments
            jettison_cruise_mission_segments = [
                HoverSegment(path_to_json, segment_type="Takeoff Hover"),
                ClimbSegment(path_to_json),
                JettisonCruiseSegment(path_to_json, jettison_config),
                HoverSegment(path_to_json, segment_type="Landing Hover"),
                ReserveSegment(path_to_json)
            ]
            
            # Create a copy of the aircraft to avoid modifying the original
            aircraft_copy_cruise = copy.deepcopy(aircraft)
            jettison_cruise_mission = Mission(aircraft=aircraft_copy_cruise, mission_segments=jettison_cruise_mission_segments)
            jettison_cruise_mission.run_mission()
            
            # Run ABU jettison mission at climb start
            print("\n--- Creating and Running ABU Jettison at Climb Mission ---")
            print(f"This mission will jettison {jettison_config['boom_count']} booms, "
                  f"{jettison_config['jettison_rotor_count']} {jettison_config['jettison_rotor_type']} rotors, "
                  f"and {jettison_config['battery_mass_kg']:.1f} kg of battery mass before climb")
            
            # Create jettison mission segments for climb
            jettison_climb_mission_segments = [
                HoverSegment(path_to_json, segment_type="Takeoff Hover"),
                JettisonClimbSegment(path_to_json, jettison_config),
                JettisonCruiseSegment(path_to_json, jettison_config), 
                HoverSegment(path_to_json, segment_type="Landing Hover"),
                ReserveSegment(path_to_json)
            ]
            
            # Create another copy of the aircraft
            aircraft_copy_climb = copy.deepcopy(aircraft)
            jettison_climb_mission = Mission(aircraft=aircraft_copy_climb, mission_segments=jettison_climb_mission_segments)
            jettison_climb_mission.run_mission()
            
            # Display all mission results
            if jettison_cruise_mission.is_calculated and jettison_climb_mission.is_calculated:
                print("\n--- ABU Jettison at Cruise Mission Results ---")
                jettison_cruise_mission.display_summary()
                
                print("\n--- ABU Jettison at Climb Mission Results ---")
                jettison_climb_mission.display_summary()
                
                # Show comparison of all three missions
                print("\n--- Mission Comparison ---")
                comparison_data = [
                    ["Metric", "Baseline", "Jettison at Cruise", "Jettison at Climb"],
                    ["Total Energy (kWh)", 
                     f"{mission.total_energy_kwh:.3f}",
                     f"{jettison_cruise_mission.total_energy_kwh:.3f}",
                     f"{jettison_climb_mission.total_energy_kwh:.3f}"],
                    ["Energy Saved (kWh)", 
                     "0.000",
                     f"{mission.total_energy_kwh - jettison_cruise_mission.total_energy_kwh:.3f}",
                     f"{mission.total_energy_kwh - jettison_climb_mission.total_energy_kwh:.3f}"],
                    ["Energy Saving (%)", 
                     "0.00%",
                     f"{(mission.total_energy_kwh - jettison_cruise_mission.total_energy_kwh) / mission.total_energy_kwh * 100:.2f}%",
                     f"{(mission.total_energy_kwh - jettison_climb_mission.total_energy_kwh) / mission.total_energy_kwh * 100:.2f}%"],
                    ["Max Power (kW)", 
                     f"{mission.max_power_kw:.2f}",
                     f"{jettison_cruise_mission.max_power_kw:.2f}",
                     f"{jettison_climb_mission.max_power_kw:.2f}"],
                    ["Max C-Rate", 
                     f"{mission.max_c_rate:.2f}",
                     f"{jettison_cruise_mission.max_c_rate:.2f}",
                     f"{jettison_climb_mission.max_c_rate:.2f}"]
                ]
                print(tabulate(comparison_data, headers="firstrow", tablefmt="grid"))

                # Generate mission profile plots if matplotlib is available
                try:
                    import matplotlib.pyplot as plt
                    # Plot baseline mission power profile
                    time_s, power_kw = mission.get_power_profile()
                    time_s_c, c_rate = mission.get_c_rate_profile()
                    
                    if time_s and power_kw:
                        plt.figure(figsize=(12, 8))
                        
                        plt.subplot(3, 1, 1)
                        plt.step(time_s, power_kw, where='post', label='Baseline')
                        plt.title('Baseline Mission Power Profile')
                        plt.ylabel('Power (kW)')
                        plt.grid(True, linestyle='--', alpha=0.7)
                        
                        # Plot Cruise Jettison mission power profile
                        time_s_cruise, power_kw_cruise = jettison_cruise_mission.get_power_profile()
                        plt.subplot(3, 1, 2)
                        plt.step(time_s_cruise, power_kw_cruise, where='post', color='green', label='Jettison at Cruise')
                        plt.title('Jettison at Cruise Power Profile')
                        plt.ylabel('Power (kW)')
                        plt.grid(True, linestyle='--', alpha=0.7)
                        
                        # Plot Climb Jettison mission power profile
                        time_s_climb, power_kw_climb = jettison_climb_mission.get_power_profile()
                        plt.subplot(3, 1, 3)
                        plt.step(time_s_climb, power_kw_climb, where='post', color='red', label='Jettison at Climb')
                        plt.title('Jettison at Climb Power Profile')
                        plt.xlabel('Time (s)')
                        plt.ylabel('Power (kW)')
                        plt.grid(True, linestyle='--', alpha=0.7)
                        
                        plt.tight_layout()
                        plt.show()
                        print("Mission comparison plot generated.")
                    else:
                        print("Mission profile data is empty. Cannot plot.")
                except ImportError:
                    print("Matplotlib not found. Skipping plot generation.")
                except Exception as plot_err:
                    print(f"Error during plot generation: {plot_err}")

    except Exception as main_err:
        print(f"\nFATAL ERROR during baseline mission execution: {main_err}")
        traceback.print_exc()
    finally:
        print("\n--- End of Baseline Mission Simulation (from lift_abu.py) ---")


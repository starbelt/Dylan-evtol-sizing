import math
import sys
from aircraft import Aircraft
from missionsegment import Mission
from battery import Battery 

W_PER_KW = 1000.0

class IterativeSizer:
    def __init__(self, config_path: str, max_iter: int, tolerance: float, damping: float):

        self.config_path = config_path
        self.max_iterations = max_iter
        self.tolerance = tolerance
        self.damping_factor = damping
        self.converged = False
        self.final_mtow_kg: float 
        self.iterations_run: int = 0

    def run_sizing_loop(self, aircraft: Aircraft, mission: Mission):

        try:
            initial_mtow_guess = aircraft._varlist.get("initial_mtow_guess_kg")
            mtow_guess_kg = initial_mtow_guess
            new_mtow_kg = mtow_guess_kg

            # Initial Battery Mass Guess
            initial_batt_mass_fraction = aircraft._varlist.get("initial_battery_mass_fraction_guess")
            initial_batt_mass_guess = mtow_guess_kg * initial_batt_mass_fraction
            aircraft.battery.final_batt_mass_kg = initial_batt_mass_guess

            # Get the maximum C-rate constraint from configuration
            battery_max_c_rate = aircraft._varlist.get("battery_max_c_rate")

            print("\nStarting MTOW & Battery Sizing Convergence Loop...")
            print("-" * 60)
            print(f"Convergence Params: Max Iter={self.max_iterations}, Tolerance={self.tolerance} kg, Damping={self.damping_factor}")
            print(f"Battery Max C-Rate Constraint: {battery_max_c_rate}")
            print("-" * 60)

            final_convergence_phase = False
            self.converged = False
            self.iterations_run = 0
            total_iterations = 0  # Track both main and final convergence iterations
            
            while total_iterations < self.max_iterations:
                total_iterations += 1
                self.iterations_run += 1
                
                # Print appropriate header for current phase
                if not final_convergence_phase:
                    print(f"\nIteration {self.iterations_run}: Current MTOW Guess = {mtow_guess_kg:.2f} kg")
                    print(f"  Current Est. Batt Mass = {aircraft.battery.final_batt_mass_kg:.2f} kg")
                else:
                    print(f"\nFinal Convergence Iteration {self.iterations_run - main_iterations}: MTOW = {mtow_guess_kg:.2f} kg")
                    print(f"  Current Battery Mass = {aircraft.battery.final_batt_mass_kg:.2f} kg")

                # 1: Update airframe and performance based on MTOW guess
                aircraft.update_state_for_iteration(mtow_guess_kg)
                if not final_convergence_phase:
                    print(f"  Aircraft state updated. Cruise L/D: {aircraft.cruise_L_p_D:.2f}")

                # 2: Update battery properties based on MTOW guess
                aircraft.battery.UpdateComponent({'mtow_kg': mtow_guess_kg})
                if not final_convergence_phase:
                    print(f"  Battery capacity updated for mission (BOL kWh: {aircraft.battery.get_gross_BOL_capacity_kwh():.2f})")
                
                # 3: Run mission simulation
                mission.run_mission()
                total_mission_energy_kwh = mission.total_energy_kwh
                max_mission_power_kw = mission.max_power_kw
                
                if not final_convergence_phase:
                    lift_rotor_count = aircraft._varlist.get("lift_rotor_count")
                    tilt_rotor_count = aircraft._varlist.get("tilt_rotor_count")
                    
                    print(f"  Mission run complete with {lift_rotor_count} lift rotors and {tilt_rotor_count} tilt rotors")
                    print(f"  Total disk area: {aircraft.get_rotor_disk_area_m2():.2f} m²")
                    print(f"  Total Energy: {total_mission_energy_kwh:.3f} kWh")
                    print(f"  Max Power: {max_mission_power_kw:.3f} kW")
                else:
                    print(f"  Mission re-run complete. Energy: {total_mission_energy_kwh:.3f} kWh, Max Power: {max_mission_power_kw:.3f} kW")

                # 4: Calculate required battery mass for both energy and C-rate constraints
                usable_EOL_spec_energy_Wh_kg = aircraft.battery.get_usable_EOL_spec_energy()
                
                # Calculate required total usable capacity needed from battery based on energy
                required_total_usable_EOL_kwh = total_mission_energy_kwh / aircraft.max_dod
                
                if not final_convergence_phase:
                    print(f"  Mission Energy: {total_mission_energy_kwh:.3f} kWh, Max DoD: {aircraft.max_dod*100:.1f}%")
                    print(f"  -> Required Total Usable EOL Capacity (Energy): {required_total_usable_EOL_kwh:.3f} kWh")
                
                # Calculate the mass required to meet energy requirements
                energy_based_batt_mass_kg = (required_total_usable_EOL_kwh * W_PER_KW) / usable_EOL_spec_energy_Wh_kg
                
                if not final_convergence_phase:
                    print(f"  -> Calculated Energy-Based Battery Mass: {energy_based_batt_mass_kg:.2f} kg (using EOL spec energy: {usable_EOL_spec_energy_Wh_kg:.1f} Wh/kg)")
                
                # Calculate required capacity based on C-rate constraint
                bol_to_eol_ratio = aircraft.battery.get_gross_BOL_capacity_kwh() / aircraft.battery.get_usable_EOL_capacity_kwh() if aircraft.battery.get_usable_EOL_capacity_kwh() > 0 else 1.0
                required_crate_capacity_kwh = max_mission_power_kw / battery_max_c_rate
                required_crate_usable_EOL_kwh = required_crate_capacity_kwh
                
                if not final_convergence_phase:
                    print(f"  -> Required Total Usable EOL Capacity (C-rate): {required_crate_usable_EOL_kwh:.3f} kWh (max power / max C-rate)")
                
                # Calculate the mass required to meet C-rate constraint
                crate_based_batt_mass_kg = (required_crate_usable_EOL_kwh * W_PER_KW) / usable_EOL_spec_energy_Wh_kg
                
                if not final_convergence_phase:
                    print(f"  -> Calculated C-Rate-Based Battery Mass: {crate_based_batt_mass_kg:.2f} kg")
                
                # Take the larger of the two battery masses to satisfy both constraints
                if crate_based_batt_mass_kg > energy_based_batt_mass_kg:
                    required_batt_mass_kg = crate_based_batt_mass_kg
                    if not final_convergence_phase:
                        print(f"  -> C-Rate Constraint is DRIVING Battery Sizing (C-rate: {battery_max_c_rate}, Max Power: {max_mission_power_kw:.2f} kW)")
                else:
                    required_batt_mass_kg = energy_based_batt_mass_kg
                    if not final_convergence_phase:
                        print(f"  -> Energy Requirement is DRIVING Battery Sizing")
                
                if not final_convergence_phase:
                    print(f"  -> Final Required Battery Mass: {required_batt_mass_kg:.2f} kg")
                
                # 5: Update battery component properties based on the new mass
                prev_batt_mass = aircraft.battery.final_batt_mass_kg
                aircraft.battery.final_batt_mass_kg = required_batt_mass_kg
                aircraft.battery.UpdateComponent({'mtow_kg': mtow_guess_kg})
                
                if not final_convergence_phase:
                    print(f"  Battery component updated with new mass. Final Weight: {aircraft.battery.weight:.2f} kg")
                else:
                    print(f"  Battery mass updated: {prev_batt_mass:.2f} kg → {aircraft.battery.weight:.2f} kg (change: {aircraft.battery.weight - prev_batt_mass:.2f} kg)")
                
                # 6: Calculate new MTOW
                new_mtow_kg = aircraft.CalculateMTOW()
                
                if not final_convergence_phase:
                    print(f"  -> New Calculated MTOW Estimate: {new_mtow_kg:.2f} kg")
                else:
                    print(f"  -> Updated MTOW: {new_mtow_kg:.2f} kg (previous: {mtow_guess_kg:.2f} kg)")
                
                # 7: Check convergence
                mtow_diff = abs(new_mtow_kg - mtow_guess_kg)
                
                if not final_convergence_phase:
                    print(f"  MTOW Difference: {mtow_diff:.3f} kg")
                else:
                    print(f"  MTOW Change: {mtow_diff:.3f} kg")
                
                if mtow_diff < self.tolerance:
                    if not final_convergence_phase:
                        # Main phase converged, switch to final convergence phase
                        print("-" * 60)
                        print(f"Initial convergence reached after {self.iterations_run} iterations.")
                        print("Starting final convergence phase to ensure stability...")
                        print("-" * 60)
                        final_convergence_phase = True
                        main_iterations = self.iterations_run
                        self.iterations_run = 0  # Reset for final phase count
                        mtow_guess_kg = new_mtow_kg  # Use the converged value
                    else:
                        # Final convergence phase also converged
                        print("-" * 60)
                        print(f"Final convergence achieved after {self.iterations_run} iterations.")
                        print(f"Total iterations: {total_iterations} (Main: {main_iterations}, Final: {self.iterations_run})")
                        print("-" * 60)
                        mtow_guess_kg = new_mtow_kg  # Lock in the final value
                        self.converged = True
                        self.final_mtow_kg = mtow_guess_kg
                        break  # Exit loop
                else:
                    # Apply damping only in main convergence phase
                    if not final_convergence_phase:
                        mtow_guess_kg = (1.0 - self.damping_factor) * mtow_guess_kg + self.damping_factor * new_mtow_kg
                    else:
                        # No damping in final convergence phase for precise result
                        mtow_guess_kg = new_mtow_kg
            
            # Post loop processing
            if not self.converged:
                print("-" * 60)
                print(f"WARNING: Max iterations ({self.max_iterations}) reached without convergence.")
                print(f"         Last MTOW estimate: {new_mtow_kg:.2f} kg (Diff: {mtow_diff:.3f} kg)")
                print("-" * 60)
                self.final_mtow_kg = new_mtow_kg
            else:
                self.final_mtow_kg = mtow_guess_kg
            
            return self.converged, self.final_mtow_kg
        
    
        except Exception as e:
            print(f"\nERROR during sizing loop: {e}")
            import traceback
            traceback.print_exc()
            return False, None

    def verify_final_mtow(self, aircraft: Aircraft, final_mtow: float, tolerance_factor: float = 1.0):

        final_mtow_check = aircraft.CalculateMTOW()
        mtow_diff = abs(final_mtow_check - final_mtow)
        is_consistent = mtow_diff <= self.tolerance * tolerance_factor
        
        print("Final state update complete.")
        print(f"Final Check MTOW: {final_mtow_check:.2f} kg (Should be close to {final_mtow:.2f} kg)")
        
        if not is_consistent:
            print(f"Warning: Final MTOW check ({final_mtow_check:.2f} kg) differs significantly from loop result ({final_mtow:.2f} kg).")
            
        return final_mtow_check, is_consistent

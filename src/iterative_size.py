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
            battery_max_c_rate = aircraft._varlist.get("battery_max_c_rate")  # Default to 6.0 if not specified

            print("\nStarting MTOW & Battery Sizing Convergence Loop...")
            print("-" * 60)
            print(f"Convergence Params: Max Iter={self.max_iterations}, Tolerance={self.tolerance} kg, Damping={self.damping_factor}")
            print(f"Battery Max C-Rate Constraint: {battery_max_c_rate}")
            print("-" * 60)

            for i in range(self.max_iterations):
                self.iterations_run = i + 1
                print(f"\nIteration {self.iterations_run}: Current MTOW Guess = {mtow_guess_kg:.2f} kg")
                print(f"  Current Est. Batt Mass = {aircraft.battery.final_batt_mass_kg:.2f} kg")

                # 1: Update airframe and performance based off MTOW guess
                aircraft.update_state_for_iteration(mtow_guess_kg)
                print(f"  Aircraft state updated. Cruise L/D: {aircraft.cruise_L_p_D:.2f}")

                # 2: Updates battery properties based off MTOW guess
                # Recalculates cell counts and final BOL capacity based on the current battery mass
                aircraft.battery.UpdateComponent({'mtow_kg': mtow_guess_kg})
                print(f"  Battery capacity updated for mission (BOL kWh: {aircraft.battery.get_gross_BOL_capacity_kwh():.2f})")

                # 3: Runs mission simulation
                mission.run_mission()
                total_mission_energy_kwh = mission.total_energy_kwh
                max_mission_power_kw = mission.max_power_kw
                
                # for testing purposes CHANGE
                lift_rotor_count = aircraft._varlist.get("lift_rotor_count")
                tilt_rotor_count = aircraft._varlist.get("tilt_rotor_count")
                
                print(f"  Mission run complete with {lift_rotor_count} lift rotors and {tilt_rotor_count} tilt rotors")
                print(f"  Total disk area: {aircraft.get_rotor_disk_area_m2():.2f} m²")
                print(f"  Total Energy: {total_mission_energy_kwh:.3f} kWh")
                print(f"  Max Power: {max_mission_power_kw:.3f} kW")

                # Recalculates battery mass
                usable_EOL_spec_energy_Wh_kg = aircraft.battery.get_usable_EOL_spec_energy()

                # Calculate required total usable capacity needed from battery based on energy
                required_total_usable_EOL_kwh = total_mission_energy_kwh / aircraft.max_dod
                print(f"  Mission Energy: {total_mission_energy_kwh:.3f} kWh, Max DoD: {aircraft.max_dod*100:.1f}%")
                print(f"  -> Required Total Usable EOL Capacity (Energy): {required_total_usable_EOL_kwh:.3f} kWh")

                # Calculate the mass required to meet energy requirements
                energy_based_batt_mass_kg = (required_total_usable_EOL_kwh * W_PER_KW) / usable_EOL_spec_energy_Wh_kg
                print(f"  -> Calculated Energy-Based Battery Mass: {energy_based_batt_mass_kg:.2f} kg (using EOL spec energy: {usable_EOL_spec_energy_Wh_kg:.1f} Wh/kg)")

                # Calculate required capacity based on C-rate constraint
                # We need BOL capacity for C-rate, so adjust using the ratio between BOL and EOL usable
                bol_to_eol_ratio = aircraft.battery.get_gross_BOL_capacity_kwh() / aircraft.battery.get_usable_EOL_capacity_kwh() if aircraft.battery.get_usable_EOL_capacity_kwh() > 0 else 1.0
                required_crate_capacity_kwh = max_mission_power_kw / battery_max_c_rate
                required_crate_usable_EOL_kwh = required_crate_capacity_kwh
                print(f"  -> Required Total Usable EOL Capacity (C-rate): {required_crate_usable_EOL_kwh:.3f} kWh (max power / max C-rate)")

                # Calculate the mass required to meet C-rate constraint
                crate_based_batt_mass_kg = (required_crate_usable_EOL_kwh * W_PER_KW) / usable_EOL_spec_energy_Wh_kg
                print(f"  -> Calculated C-Rate-Based Battery Mass: {crate_based_batt_mass_kg:.2f} kg")

                # Take the larger of the two battery masses to satisfy both constraints
                if crate_based_batt_mass_kg > energy_based_batt_mass_kg:
                    required_batt_mass_kg = crate_based_batt_mass_kg
                    print(f"  -> C-Rate Constraint is DRIVING Battery Sizing (C-rate: {battery_max_c_rate}, Max Power: {max_mission_power_kw:.2f} kW)")
                else:
                    required_batt_mass_kg = energy_based_batt_mass_kg
                    print(f"  -> Energy Requirement is DRIVING Battery Sizing")

                print(f"  -> Final Required Battery Mass: {required_batt_mass_kg:.2f} kg")

                # 5: update battery component properties based on the new mass
                aircraft.battery.final_batt_mass_kg = required_batt_mass_kg
                aircraft.battery.UpdateComponent({'mtow_kg': mtow_guess_kg})
                print(f"  Battery component updated with new mass. Final Weight: {aircraft.battery.weight:.2f} kg")

                # 6: Calculate new MTOW
                new_mtow_kg = aircraft.CalculateMTOW()
                print(f"  -> New Calculated MTOW Estimate: {new_mtow_kg:.2f} kg")

                # 7: Check convergence
                mtow_diff = abs(new_mtow_kg - mtow_guess_kg)
                print(f"  MTOW Difference: {mtow_diff:.3f} kg")
                if mtow_diff < self.tolerance:
                    print("-" * 60)
                    print(f"Convergence reached after {self.iterations_run} iterations.")
                    print("-" * 60)
                    mtow_guess_kg = new_mtow_kg # Locks it in if it converged
                    self.converged = True
                    self.final_mtow_kg = mtow_guess_kg
                    break # Exit loop

                # 8: Update MTOW guess for next iteration with damping
                mtow_guess_kg = (1.0 - self.damping_factor) * mtow_guess_kg + self.damping_factor * new_mtow_kg

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

    def verify_final_mtow(self, aircraft: Aircraft, final_mtow: float, tolerance_factor: float = 2.0):

        final_mtow_check = aircraft.CalculateMTOW()
        mtow_diff = abs(final_mtow_check - final_mtow)
        is_consistent = mtow_diff <= self.tolerance * tolerance_factor
        
        print("Final state update complete.")
        print(f"Final Check MTOW: {final_mtow_check:.2f} kg (Should be close to {final_mtow:.2f} kg)")
        
        if not is_consistent:
            print(f"Warning: Final MTOW check ({final_mtow_check:.2f} kg) differs significantly from loop result ({final_mtow:.2f} kg).")
            
        return final_mtow_check, is_consistent


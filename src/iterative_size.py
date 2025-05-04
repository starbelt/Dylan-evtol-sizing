import math
import sys
from aircraft import Aircraft
from missionsegment import Mission
from battery import Battery # Needed for type hinting and checks

W_PER_KW = 1000.0

class IterativeSizer:
    """
    Encapsulates the iterative sizing logic for an eVTOL aircraft.
    """
    def __init__(self, config_path: str, max_iter: int, tolerance: float, damping: float):
        """
        Initializes the IterativeSizer.

        Args:
            config_path (str): Path to the JSON configuration file.
            max_iter (int): Maximum number of iterations for convergence.
            tolerance (float): Convergence tolerance for MTOW (in kg).
            damping (float): Damping factor for MTOW updates.
        """
        self.config_path = config_path
        self.max_iterations = max_iter
        self.tolerance = tolerance
        self.damping_factor = damping
        self.converged = False
        self.final_mtow_kg: float 
        self.iterations_run: int = 0

    def run_sizing_loop(self, aircraft: Aircraft, mission: Mission) -> tuple[bool, float]:
        """
        Executes the iterative MTOW and battery sizing loop.

        Args:
            aircraft (Aircraft): The aircraft instance to be sized.
            mission (Mission): The mission profile instance.

        Returns:
            tuple[bool, float | None]: A tuple containing:
                - bool: True if convergence was reached, False otherwise.
                - float | None: The final converged MTOW in kg, or the last estimate if not converged.
        """
        try:
            initial_mtow_guess = aircraft._varlist.get("initial_mtow_guess_kg")
            mtow_guess_kg = initial_mtow_guess
            new_mtow_kg = mtow_guess_kg

            # Initial Battery Mass Guess
            initial_batt_mass_fraction = aircraft._varlist.get("initial_battery_mass_fraction_guess")
            initial_batt_mass_guess = mtow_guess_kg * initial_batt_mass_fraction
            if not isinstance(aircraft.battery, Battery):
                 print("ERROR: Aircraft object does not have a valid Battery component.")
                 return False, None
            aircraft.battery.final_batt_mass_kg = initial_batt_mass_guess

            print("\nStarting MTOW & Battery Sizing Convergence Loop...")
            print("-" * 60)
            print(f"Convergence Params: Max Iter={self.max_iterations}, Tolerance={self.tolerance} kg, Damping={self.damping_factor}")
            print("-" * 60)

            for i in range(self.max_iterations):
                self.iterations_run = i + 1
                print(f"\nIteration {self.iterations_run}: Current MTOW Guess = {mtow_guess_kg:.2f} kg")
                print(f"  Current Est. Batt Mass = {aircraft.battery.final_batt_mass_kg:.2f} kg")

                # --- Step 1: Update Airframe & Performance based on MTOW Guess ---
                aircraft.update_state_for_iteration(mtow_guess_kg)
                print(f"  Aircraft state updated. Cruise L/D: {aircraft.cruise_L_p_D:.2f}")

                # --- Step 2: Update Battery Capacity ---
                # This recalculates cell counts and final BOL capacity based on current mass estimate
                aircraft.battery.UpdateComponent({'mtow_kg': mtow_guess_kg})
                print(f"  Battery capacity updated for mission (BOL kWh: {aircraft.battery.get_gross_BOL_capacity_kwh():.2f})")

                # --- Step 3: Run Mission Simulation ---
                mission.run_mission()
                total_mission_energy_kwh = mission.total_energy_kwh
                if total_mission_energy_kwh is None:
                    print("  ERROR: Mission simulation failed to produce energy result.")
                    return False, mtow_guess_kg # Return last guess
                print(f"  Mission run complete. Total Energy: {total_mission_energy_kwh:.3f} kWh")

                # --- Step 4: Calculate Required Battery Mass ---
                usable_EOL_spec_energy_Wh_kg = aircraft.battery.get_usable_EOL_spec_energy()
                if usable_EOL_spec_energy_Wh_kg <= 0:
                    print("  ERROR: Battery usable EOL specific energy is zero or negative.")
                    return False, mtow_guess_kg

                # Calculate required total usable capacity needed from battery
                required_total_usable_EOL_kwh = total_mission_energy_kwh / aircraft.max_dod
                print(f"  Mission Energy: {total_mission_energy_kwh:.3f} kWh, Max DoD: {aircraft.max_dod*100:.1f}%")
                print(f"  -> Required Total Usable EOL Capacity: {required_total_usable_EOL_kwh:.3f} kWh")

                # Calculate the mass required to provide this capacity
                required_batt_mass_kg = (required_total_usable_EOL_kwh * W_PER_KW) / usable_EOL_spec_energy_Wh_kg
                print(f"  -> Calculated Required Battery Mass: {required_batt_mass_kg:.2f} kg (using EOL spec energy: {usable_EOL_spec_energy_Wh_kg:.1f} Wh/kg)")

                # --- Step 5: Update Battery Component Weight & Properties ---
                # Set the *newly calculated* mass in the battery object for the *next* iteration's MTOW calc
                aircraft.battery.final_batt_mass_kg = required_batt_mass_kg
                # Update internal battery properties (cell counts, final capacity, weight) based on the *new* final_batt_mass_kg
                # This ensures the battery's weight attribute is correct for the MTOW sum
                aircraft.battery.UpdateComponent({'mtow_kg': mtow_guess_kg}) # Arg might be optional if not used
                print(f"  Battery component updated with new mass. Final Weight: {aircraft.battery.weight:.2f} kg")

                # --- Step 6: Calculate New MTOW Estimate ---
                # Sum weights of all components including updated battery, payload, fixed systems
                new_mtow_kg = aircraft.CalculateMTOW()
                print(f"  -> New Calculated MTOW Estimate: {new_mtow_kg:.2f} kg")

                # --- Step 7: Check Convergence ---
                mtow_diff = abs(new_mtow_kg - mtow_guess_kg)
                print(f"  MTOW Difference: {mtow_diff:.3f} kg")
                if mtow_diff < self.tolerance:
                    print("-" * 60)
                    print(f"Convergence reached after {self.iterations_run} iterations.")
                    print("-" * 60)
                    mtow_guess_kg = new_mtow_kg # Lock in the converged value
                    self.converged = True
                    self.final_mtow_kg = mtow_guess_kg
                    break # Exit loop

                # --- Step 8: Update MTOW Guess for Next Iteration (Apply Damping) ---
                mtow_guess_kg = (1.0 - self.damping_factor) * mtow_guess_kg + self.damping_factor * new_mtow_kg
                # Prevent guess from going non-physical if something went wrong
                if mtow_guess_kg <= 0:
                    print("ERROR: MTOW guess became non-positive. Aborting.")
                    return False, None


            # --- Post-Loop Processing ---
            if not self.converged:
                print("-" * 60)
                print(f"WARNING: Max iterations ({self.max_iterations}) reached without convergence.")
                print(f"         Last MTOW estimate: {new_mtow_kg:.2f} kg (Diff: {mtow_diff:.3f} kg)")
                print("-" * 60)
                self.final_mtow_kg = new_mtow_kg # Store the last estimate
            else:
                 self.final_mtow_kg = mtow_guess_kg # Ensure final_mtow is set if converged on last iteration

            return self.converged, self.final_mtow_kg

        except Exception as e:
            print(f"\nERROR during sizing loop: {e}")
            import traceback
            traceback.print_exc()
            return False, None # Indicate failure

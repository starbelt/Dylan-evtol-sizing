# --- START OF FILE lifting_abu_manager.py ---

import copy
import json
from typing import List, Dict, Any, Optional
from aircraft import Aircraft, Boom, LiftRotor, Battery, AircraftComponent
from lift_ABU import LiftABU
from missionsegment import Mission, MissionSegment # Import MissionSegment base

W_PER_KW = 1000.0

class LiftingABUManager:
    """
    Manages the configuration and simulation of an aircraft
    with jettisonable Lifting ABU units replacing standard lift systems.
    """
    def __init__(self, sized_aircraft: Aircraft, config_path: str):
        """
        Initializes the manager and modifies the aircraft configuration.

        Args:
            sized_aircraft (Aircraft): The aircraft object *after* initial sizing.
                                      This object will be modified in place.
            config_path (str): Path to the JSON configuration file.
        """
        self.config_path = config_path
        self.aircraft = sized_aircraft # Keep a reference to the modified aircraft
        self._load_config()

        if not self.enable_scenario:
            print("Lifting ABU Scenario disabled in config.")
            self.is_configured = False
            return

        self.original_components: List[AircraftComponent] = copy.deepcopy(self.aircraft.components)
        self.original_main_battery_mass: float = self.aircraft.battery.final_batt_mass_kg
        self.original_mtow: float = self.aircraft.mtow_kg

        self.lift_abu_units: List[LiftABU] = []
        self.abu_mass_per_unit_kg: float = 0.0

        print("\n--- Configuring Lifting ABU Scenario ---")
        self.is_configured = self._configure_aircraft_with_lifting_abus()

        if self.is_configured:
            print(f"Configuration Complete. New MTOW: {self.aircraft.mtow_kg:.2f} kg")
            print(f"Main Battery Mass Reduced To: {self.aircraft.battery.final_batt_mass_kg:.2f} kg")
            print(f"Added {len(self.lift_abu_units)} LiftABU units.")
        else:
            print("ERROR: Failed to configure aircraft for Lifting ABU scenario.")


    def _load_config(self):
        """Loads lifting ABU specific parameters from the JSON config."""
        with open(self.config_path, "r") as f:
            config = json.load(f)
        self.enable_scenario = config.get("enable_lifting_abu_scenario", False)
        self.num_lift_systems_to_replace = config.get("num_lift_systems_to_replace", 0)
        self.total_abu_mass_kg = config.get("total_abu_mass_kg", 0.0) # Total mass for *all* ABUs
        self.jettison_time_s = config.get("lifting_abu_jettison_time_s", float('inf')) # Time to drop

    def _configure_aircraft_with_lifting_abus(self) -> bool:
        """
        Internal method to modify the aircraft's component list and battery.
        Returns True if successful, False otherwise.
        """
        if self.num_lift_systems_to_replace <= 0:
            print("Warning: num_lift_systems_to_replace is zero or less. No changes made.")
            return True # Technically successful, no change needed

        if self.total_abu_mass_kg <= 0:
            print("Warning: total_abu_mass_kg is zero or less. LiftABUs will have no battery mass.")
            # Allow proceeding, ABUs will just be structural/rotor mass

        # --- Identify and Remove Original Components ---
        lift_rotor_indices = [i for i, comp in enumerate(self.aircraft.components) if isinstance(comp, LiftRotor)]
        boom_indices = [i for i, comp in enumerate(self.aircraft.components) if isinstance(comp, Boom)]

        if len(lift_rotor_indices) < self.num_lift_systems_to_replace:
            print(f"ERROR: Not enough LiftRotors ({len(lift_rotor_indices)}) to replace {self.num_lift_systems_to_replace}.")
            return False
        # Simplistic: Assume one boom per lift rotor system exists and needs removal.
        # A more robust system might link booms and rotors.
        if len(boom_indices) < self.num_lift_systems_to_replace:
            print(f"ERROR: Not enough Booms ({len(boom_indices)}) to replace {self.num_lift_systems_to_replace}.")
            return False

        # Remove the *last* N components found (often safer than first if order matters elsewhere)
        indices_to_remove = sorted(lift_rotor_indices[-self.num_lift_systems_to_replace:] +
                                   boom_indices[-self.num_lift_systems_to_replace:], reverse=True)

        removed_components = []
        for index in indices_to_remove:
            removed_components.append(self.aircraft.components.pop(index))
        print(f"Removed {len(removed_components)} original components (LiftRotors/Booms).")

        # --- Adjust Main Battery ---
        main_battery = self.aircraft.battery
        if not isinstance(main_battery, Battery):
            print("ERROR: Could not find main Battery component.")
            return False

        mass_removed_from_main = min(self.total_abu_mass_kg, main_battery.final_batt_mass_kg)
        if mass_removed_from_main < self.total_abu_mass_kg:
            print(f"Warning: Requested total ABU mass ({self.total_abu_mass_kg:.2f} kg) exceeds main battery mass "
                  f"({main_battery.final_batt_mass_kg:.2f} kg). Reducing ABU mass to {mass_removed_from_main:.2f} kg.")
            self.total_abu_mass_kg = mass_removed_from_main

        main_battery.final_batt_mass_kg -= self.total_abu_mass_kg
        # CRITICAL: Update the battery's internal state after changing its mass
        main_battery.UpdateComponent({}) # Pass empty state, MTOW not needed here directly
        print(f"Reduced main battery mass by {self.total_abu_mass_kg:.2f} kg.")

        # --- Create and Add LiftABU Units ---
        self.abu_mass_per_unit_kg = self.total_abu_mass_kg / self.num_lift_systems_to_replace if self.num_lift_systems_to_replace > 0 else 0.0

        # Create initial state dict for LiftABU updates
        # Needs the MTOW *before* adding the new units but *after* removing old ones and adjusting battery
        temp_mtow_after_removal = self.aircraft.CalculateMTOW() # Calculate intermediate MTOW
        aircraft_state_for_abu = {'mtow_kg': temp_mtow_after_removal} # Initial guess
        # Add other state needed by Boom/Rotor if not already in aircraft._varlist
        # e.g., 'single_epu_weight_kg' might need recalculation based on *original* design spec
        # For simplicity, assume LiftABU's internal Boom/Rotor use sizing from JSON or original aircraft state
        # Copy relevant original state items if needed:
        aircraft_state_for_abu['single_epu_weight_kg'] = self.aircraft._varlist.get('single_epu_weight_kg_sized', 0.0) # Assuming it was stored
        aircraft_state_for_abu['battery_spec_energy'] = self.aircraft.battery.get_gross_BOL_spec_energy() # Use main batt spec energy? Or ABU spec energy? ABU uses its own calc based on cell params.
        # Wing props might be needed by Boom calc - get from *current* aircraft state
        wing = next((c for c in self.aircraft.components if isinstance(c, Wing)), None)
        if wing:
             aircraft_state_for_abu['wing_ref_area_m2'] = wing.wing_ref_area_m2
             aircraft_state_for_abu['wing_MAC'] = wing.wing_MAC
             # etc.

        for i in range(self.num_lift_systems_to_replace):
            unit_id = f"lift_abu_{i+1}"
            lift_abu = LiftABU(self.config_path, unit_id)
            # Pass the mass specific to this unit for its ABU calculation
            state_for_this_unit = aircraft_state_for_abu.copy()
            state_for_this_unit['abu_mass_kg_per_unit'] = self.abu_mass_per_unit_kg
            state_for_this_unit[f'{unit_id}_abu_mass_kg'] = self.abu_mass_per_unit_kg # Alt way to pass

            lift_abu.UpdateComponent(state_for_this_unit) # Update to calculate its weight
            self.lift_abu_units.append(lift_abu)
            self.aircraft.components.append(lift_abu) # Add to the main list

        # --- Final MTOW Recalculation ---
        # Now update the aircraft's overall state *with* the LiftABUs added
        # This recalculates Cd0 sums, etc.
        self.aircraft.update_state_for_iteration(self.aircraft.CalculateMTOW())
        # Store the final configured MTOW
        self.aircraft.mtow_kg = self.aircraft.CalculateMTOW()

        return True

    def get_current_total_usable_EOL_kwh(self) -> float:
        """ Calculates the total usable EOL capacity of the system
            (main battery + non-jettisoned ABUs). """
        if not self.is_configured:
            return self.aircraft.battery.get_usable_EOL_capacity_kwh() # Original capacity

        total_capacity = self.aircraft.battery.get_usable_EOL_capacity_kwh()
        for unit in self.lift_abu_units:
            if not unit.is_jettisoned:
                total_capacity += unit.get_abu_usable_EOL_capacity_kwh()
        return total_capacity

    def get_current_total_gross_BOL_kwh(self) -> float:
        """ Calculates the total gross BOL capacity for C-rate estimation. """
        if not self.is_configured:
            return self.aircraft.battery.get_gross_BOL_capacity_kwh()

        # Estimate BOL capacity based on EOL capacity and ratios from the main battery config
        # This assumes ABU cells are similar enough to main battery cells for this ratio.
        main_batt = self.aircraft.battery
        try:
            # Calculate ratio: BOL / EOL_usable for the main battery type
            bol_per_eol_usable_ratio = main_batt.get_gross_BOL_capacity_kwh() / main_batt.get_usable_EOL_capacity_kwh()
        except ZeroDivisionError:
            bol_per_eol_usable_ratio = 1.0 # Fallback if EOL usable is zero

        current_eol_usable = self.get_current_total_usable_EOL_kwh()
        return current_eol_usable * bol_per_eol_usable_ratio


    def run_mission_with_jettison(self, mission_segments: List[MissionSegment]) -> Optional[Mission]:
        """
        Simulates the mission, performing jettison at the specified time.

        Args:
            mission_segments (List[MissionSegment]): The list of mission segments to run.

        Returns:
            Optional[Mission]: A Mission object containing the results of the
                               simulation with jettison, or None if failed.
        """
        if not self.is_configured:
            print("ERROR: Lifting ABU scenario not configured. Cannot run mission.")
            return None

        print(f"\n--- Running Mission with Potential Jettison at {self.jettison_time_s}s ---")

        # Create a new Mission instance for this simulation
        # We need to carefully manage the aircraft state and battery capacity used by segments
        mission = Mission(self.aircraft, copy.deepcopy(mission_segments))

        # --- Modify the Mission class or segment calculation temporarily ---
        # Option A: Add state to Aircraft/Manager that segments query (preferred)
        # Option B: Subclass Mission/Segments (more complex)

        # We will use Option A. Segments need to use the manager's capacity methods.
        # Modify segment calculation logic slightly (or ensure it calls the right aircraft methods)

        # --- Simulation Loop ---
        current_time_s = 0.0
        jettison_performed = False
        pre_jettison_mtow = self.aircraft.mtow_kg

        # Reset LiftABU jettison state for this run
        for unit in self.lift_abu_units:
            unit.is_jettisoned = False
            # Ensure weight is reset if running multiple times
            if unit.weight == 0 and unit._pre_jettison_weight > 0:
                unit.weight = unit._pre_jettison_weight


        print(f"Initial State: MTOW={self.aircraft.mtow_kg:.2f} kg, "
              f"Usable EOL Cap={self.get_current_total_usable_EOL_kwh():.2f} kWh")

        for i, segment in enumerate(mission.mission_segments):
            segment_start_time = current_time_s
            print(f"\nCalculating Segment {i+1}: {segment.segment_type} (Time: {segment_start_time:.1f}s)")

            # --- Check for Jettison Event ---
            if not jettison_performed and segment_start_time >= self.jettison_time_s:
                print(f"*** Jettisoning LiftABUs at {current_time_s:.1f}s (Trigger time: {self.jettison_time_s}s) ***")
                total_jettisoned_mass = 0
                for unit in self.lift_abu_units:
                    if not unit.is_jettisoned:
                        total_jettisoned_mass += unit.get_pre_jettison_weight() # Use weight *before* zeroing
                        unit.jettison() # Mark as jettisoned, sets weight to 0

                # Update aircraft MTOW *after* jettison
                self.aircraft.mtow_kg -= total_jettisoned_mass
                # Re-evaluate aircraft state (e.g., drag) with reduced weight and components removed implicitly
                # Note: Components are marked jettisoned, not removed from list here,
                # but their contribution to weight and potentially drag (if Update checks flag) is zero.
                # A cleaner way might be to rebuild the component list, but that's more complex.
                # For now, assume UpdateComponent in Aircraft handles the zero-weight components correctly for Cd0 sum.
                # Let's force an update based on the new MTOW.
                print(f"Updating aircraft state post-jettison. New MTOW: {self.aircraft.mtow_kg:.2f} kg")
                self.aircraft.update_state_for_iteration(self.aircraft.mtow_kg) # Update L/D, Cd0 etc.

                jettison_performed = True
                print(f"Post-Jettison State: MTOW={self.aircraft.mtow_kg:.2f} kg, "
                      f"Usable EOL Cap={self.get_current_total_usable_EOL_kwh():.2f} kWh")


            # --- Calculate Segment Performance ---
            # The segment calculation MUST use the *current* aircraft state
            # and query the *current* total battery capacity for C-rate/DoD.
            try:
                # Pass the manager so segments can query current capacity if needed
                # Or modify aircraft to have methods that reflect current capacity
                # Let's assume Aircraft methods are updated (see modification below)
                segment.calculate_performance(self.aircraft)
            except Exception as e:
                print(f"  ERROR calculating performance for segment {i+1} ({segment.segment_type}): {e}")
                # Handle error - maybe stop simulation?
                return None # Indicate failure

            # Accumulate results (Mission class internals handle this)
            if segment.duration_s is None: segment.duration_s = 0 # Avoid errors later
            current_time_s += segment.duration_s

            print(f"  Segment Result: Dur={segment.duration_s:.1f}s, Pwr={segment.power_draw_kw:.2f}kW, "
                  f"E={segment.energy_kwh:.3f}kWh, C={segment.c_rate:.2f}")


        # --- Finalize Mission Object ---
        # The Mission object's internal run logic isn't used directly here,
        # but we need to populate its summary fields based on the looped calculations.
        mission.total_duration_s = sum(s.duration_s for s in mission.mission_segments if s.duration_s is not None)
        mission.total_energy_kwh = sum(s.energy_kwh for s in mission.mission_segments if s.energy_kwh is not None)
        mission.max_power_kw = max([s.power_draw_kw for s in mission.mission_segments if s.power_draw_kw is not None] + [0])
        mission.max_c_rate = max([s.c_rate for s in mission.mission_segments if s.c_rate is not None] + [0])
        mission.segment_results = [s.get_results() for s in mission.mission_segments]
        mission.is_calculated = True # Mark as calculated

        print("\n--- Mission Simulation with Jettison Complete ---")
        return mission
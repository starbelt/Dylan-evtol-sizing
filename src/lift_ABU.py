# --- START OF FILE lifting_abu_manager.py ---

import json
from typing import Dict, Any
from base_component import AircraftComponent
from abu import ABU
# Correct imports if Boom and LiftRotor are in aircraft.py or separate files
try:
    from aircraft import Boom, LiftRotor
except ImportError:
    # Handle potential circular imports or different file structures if needed
    # This might require refactoring Boom/LiftRotor out of aircraft.py
    # For now, assume they are accessible.
    print("Warning: Could not import Boom/LiftRotor directly from aircraft. Assuming available.")
    # Define dummy classes if necessary for linting/type checking, but ensure real ones load
    class Boom(AircraftComponent): pass
    class LiftRotor(AircraftComponent): pass


class LiftABU(AircraftComponent):
    """
    Represents a combined unit consisting of an Auxiliary Battery Unit (ABU),
    a Boom structure, and a Lift Rotor, potentially designed to be jettisoned together.
    """
    def __init__(self, path_to_json: str, unit_id: str = "lift_abu_1"):
        """
        Initializes the LiftABU component.

        Args:
            path_to_json (str): Path to the main JSON configuration file.
            unit_id (str): A unique identifier for this specific LiftABU unit.
        """
        super().__init__(path_to_json)
        self.unit_id = unit_id
        # Instantiate sub-components: ABU, Boom, and LiftRotor
        # Ensure these sub-components load shared parameters correctly
        self.abu = ABU(path_to_json)
        self.boom = Boom(path_to_json)
        self.lift_rotor = LiftRotor(path_to_json)

        # Flag to indicate if this unit has been jettisoned
        self.is_jettisoned = False
        # Store the calculated weight before jettison if needed
        self._pre_jettison_weight = 0.0

    def load_variables_from_json(self):
        """
        Loads variables for the LiftABU and its sub-components.
        """
        super().load_variables_from_json()
        # Load variables for each sub-component
        # They should share the main config file
        self.abu.load_variables_from_json()
        self.boom.load_variables_from_json()
        self.lift_rotor.load_variables_from_json()


    def UpdateComponent(self, aircraft_state: Dict[Any, Any]):
        """
        Updates the state of the LiftABU and its sub-components based on the overall aircraft state.

        Args:
            aircraft_state (dict): Dictionary containing the current aircraft state.
                                   Crucially, this should contain 'abu_mass_kg_per_unit'
                                   specific to this instance if managed externally.
                                   It also passes MTOW, single_epu_weight_kg etc. needed
                                   by Boom and LiftRotor.
        """
        if self.is_jettisoned:
            self.weight = 0.0
            # Optionally zero out sub-component weights too
            self.abu.weight = 0.0
            self.boom.weight = 0.0
            self.lift_rotor.weight = 0.0
            return # Don't update if jettisoned

        # Ensure variables are loaded if not already
        if not self._varlist:
            self.load_variables_from_json()

        # --- ABU Update ---
        # The manager should provide the specific mass for THIS ABU unit
        abu_mass_for_this_unit = aircraft_state.get(f'{self.unit_id}_abu_mass_kg',
                                                     aircraft_state.get('abu_mass_kg_per_unit', 0.0))
        abu_state = aircraft_state.copy()
        abu_state['abu_mass_kg'] = abu_mass_for_this_unit # Tell ABU its specific mass
        self.abu.UpdateComponent(abu_state) # ABU calculates its capacity and weight

        # --- Boom and Rotor Update ---
        # These depend on overall aircraft state (MTOW, EPU weight, etc.)
        # The aircraft_state passed in should reflect the MTOW *with* LiftABUs attached
        self.boom.UpdateComponent(aircraft_state)
        self.lift_rotor.UpdateComponent(aircraft_state)

        # --- Calculate Total Weight ---
        self.weight = self.abu.get_weight_kg() + \
                      self.boom.get_weight_kg() + \
                      self.lift_rotor.get_weight_kg()
        self._pre_jettison_weight = self.weight

        # # Optional: Print statement for debugging/logging
        # print(f"    - {type(self).__name__} ({self.unit_id}): Updated. Total Weight = {self.weight:.2f} kg "
        #       f"(ABU: {self.abu.weight:.2f}, Boom: {self.boom.weight:.2f}, RotorSys: {self.lift_rotor.weight:.2f})")

    def get_weight_kg(self) -> float:
        """ Returns the current weight (0 if jettisoned). """
        return self.weight

    def get_abu_usable_EOL_capacity_kwh(self) -> float:
        """ Returns the usable EOL capacity of the ABU within this unit. """
        if self.is_jettisoned:
            return 0.0
        return self.abu.get_usable_EOL_capacity_kwh()

    def get_pre_jettison_weight(self) -> float:
        """ Returns the weight the unit had before being jettisoned. """
        return self._pre_jettison_weight

    def jettison(self):
        """ Marks the unit as jettisoned and sets its weight to zero. """
        if not self.is_jettisoned:
            print(f"Jettisoning {self.unit_id}. Original weight: {self.weight:.2f} kg")
            self._pre_jettison_weight = self.weight # Store weight just before zeroing
            self.weight = 0.0
            self.abu.weight = 0.0
            self.boom.weight = 0.0
            self.lift_rotor.weight = 0.0
            self.is_jettisoned = True
import json
import math
from base_component import AircraftComponent
from typing import Dict, Any

W_PER_KW = 1000.0

class ABU(AircraftComponent):
    """
    Represents an Auxiliary Battery Unit (ABU) that can be jettisoned.
    Its mass is typically determined during an optimization process.
    """
    def __init__(self, path_to_json: str):
        """
        Initializes the ABU component.

        Args:
            path_to_json (str): Path to the main JSON configuration file.
        """
        super().__init__(path_to_json)
        self.abu_mass_kg: float = 0.0 # This will be set externally, e.g., by optimization
        self.usable_EOL_capacity_kwh: float = 0.0 # Calculated based on mass and spec energy
        self._pack_spec_energy_usable_EOL_Wh_kg: float = 0.0 # Copied from main battery for consistency

    def load_variables_from_json(self):
        """
        Loads necessary variables from the JSON configuration file.
        Specifically, it needs battery characteristics to calculate capacity from mass.
        """
        super().load_variables_from_json()
        # Load parameters needed to calculate specific energy, mirroring the main Battery class logic
        try:
            cell_voltage = float(self._varlist['cell_voltage'])
            cell_charge_ah = float(self._varlist['cell_charge_ah'])
            cell_mass_kg = float(self._varlist['cell_mass_kg'])
            cell_to_pack_mass_ratio = float(self._varlist['cell_to_pack_mass_ratio'])
            batt_soh = float(self._varlist['batt_soh'])
            batt_inacc_energy_frac = float(self._varlist['batt_inacc_energy_frac'])

            # Calculate specific energy (same calculation as in Battery class)
            cell_energy_Wh = cell_voltage * cell_charge_ah
            if cell_mass_kg > 0:
                cell_specific_energy_Wh_kg = cell_energy_Wh / cell_mass_kg
                pack_spec_energy_gross_BOL_Wh_kg = cell_specific_energy_Wh_kg * cell_to_pack_mass_ratio
                self._pack_spec_energy_usable_EOL_Wh_kg = (
                    pack_spec_energy_gross_BOL_Wh_kg
                    * batt_soh
                    * (1.0 - batt_inacc_energy_frac)
                )
            else:
                self._pack_spec_energy_usable_EOL_Wh_kg = 0.0
                print("Warning: Cell mass is zero in config, cannot calculate ABU specific energy.")

        except KeyError as e:
            print(f"ERROR in ABU: Missing key in JSON config needed for specific energy calculation: {e}")
            self._pack_spec_energy_usable_EOL_Wh_kg = 0.0
        except Exception as e:
            print(f"ERROR in ABU loading variables: {e}")
            self._pack_spec_energy_usable_EOL_Wh_kg = 0.0


    def UpdateComponent(self, aircraft_state: Dict[Any, Any]):
        """
        Updates the ABU's state. Primarily sets its weight based on the externally determined mass.

        Args:
            aircraft_state (dict): Dictionary containing the current aircraft state.
                                   Expected to potentially contain 'abu_mass_kg'.
        """
        # Load base variables if not already loaded (e.g., specific energy)
        if not self._varlist:
            self.load_variables_from_json()
        elif self._pack_spec_energy_usable_EOL_Wh_kg == 0.0: # Reload if spec energy calc failed before
             self.load_variables_from_json()


        # Set the mass from the aircraft state if provided, otherwise use the internal value
        self.abu_mass_kg = aircraft_state.get('abu_mass_kg', self.abu_mass_kg)
        self.weight = self.abu_mass_kg

        # Calculate the usable EOL capacity based on the current mass and specific energy
        if self._pack_spec_energy_usable_EOL_Wh_kg > 0:
            self.usable_EOL_capacity_kwh = (self.abu_mass_kg * self._pack_spec_energy_usable_EOL_Wh_kg) / W_PER_KW
        else:
            self.usable_EOL_capacity_kwh = 0.0
            if self.abu_mass_kg > 0:
                 print("Warning: ABU mass is > 0 but specific energy is 0. Capacity cannot be calculated.")

    def get_usable_EOL_capacity_kwh(self) -> float:
        """
        Returns the calculated usable End-of-Life (EOL) capacity of the ABU in kWh.
        """
        return self.usable_EOL_capacity_kwh

    def get_usable_EOL_spec_energy(self) -> float:
        """
        Returns the pack usable EOL specific energy in Wh/kg used for calculations.
        """
        return self._pack_spec_energy_usable_EOL_Wh_kg

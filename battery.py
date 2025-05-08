# battery.py
import json
import math
from typing import Dict, Any
from base_component import AircraftComponent #
W_PER_KW = 1000.0

# --- Battery Class ---
class Battery(AircraftComponent):
    def __init__(self, path_to_json: str):
        super().__init__(path_to_json)
        self.final_batt_mass_kg: float = 0.0 # set by the sizing loop in the excecution code
        self.parallel_modules: int = 1  # Number of parallel battery modules for C-rate management

        # Cell properties loaded from JSON
        self.cell_voltage: float = 0.0
        self.cell_charge_ah: float = 0.0
        self.cell_mass_kg: float = 0.0
        self.cell_to_pack_mass_ratio: float = 0.0
        self.batt_soh: float = 0.0 
        self.batt_inacc_energy_frac: float = 0.0 
        self.system_voltage: float = 0.0

        # Calculated Specific Energies (Pack Level)
        self._pack_spec_energy_gross_BOL_Wh_kg: float = 0.0
        self._pack_spec_energy_usable_EOL_Wh_kg: float = 0.0

        # Final calculated properties based on final_batt_mass_kg and cell count
        self.gross_BOL_kWh: float = 0.0   
        self.gross_EOL_kWh: float = 0.0
        self.usable_EOL_Wh: float = 0.0           
        self.usable_EOL_kWh: float = 0.0          
        self.total_charge_ah_BOL: float = 0.0     
        self.cells_series: int = 0
        self.cells_parallel: int = 0
        self.cell_count: int = 0                  
        self.total_cell_mass_kg: float = 0.0      

        self._pack_spec_energy_gross_BOL_Wh_kg = 0.0
        self._pack_spec_energy_usable_EOL_Wh_kg = 0.0


    def load_variables_from_json(self):
        super().load_variables_from_json()

        self._load_battery_params()
        self._calculate_specific_energies() 

    def _load_battery_params(self):
        self.cell_voltage = float(self._varlist['cell_voltage'])
        self.cell_charge_ah = float(self._varlist['cell_charge_ah'])
        self.cell_mass_kg = float(self._varlist['cell_mass_kg'])
        self.cell_to_pack_mass_ratio = float(self._varlist['cell_to_pack_mass_ratio'])
        self.batt_soh = float(self._varlist['batt_soh'])
        self.batt_inacc_energy_frac = float(self._varlist['batt_inacc_energy_frac'])
        self.system_voltage = float(self._varlist['system_voltage'])

    # Calculates specific energies based on cell properties
    def _calculate_specific_energies(self):

        # Cell level energy and specific energy
        cell_energy_Wh = self.cell_voltage * self.cell_charge_ah
        cell_specific_energy_Wh_kg = cell_energy_Wh / self.cell_mass_kg

        # Pack level gross BOL specific energy
        self._pack_spec_energy_gross_BOL_Wh_kg = cell_specific_energy_Wh_kg * self.cell_to_pack_mass_ratio

        # Pack level usable EOL specific energy
        self._pack_spec_energy_usable_EOL_Wh_kg = (
            self._pack_spec_energy_gross_BOL_Wh_kg
            * self.batt_soh
            * (1.0 - self.batt_inacc_energy_frac)
        )

    # Updates the components based on the final mass
    # It then recalculatees the BOL capacity based on the final integer cell count
    def UpdateComponent(self, aircraft_state: dict):

        self.weight = self.final_batt_mass_kg

        # Calculate Cell Configuration based on Mass 
        self.cells_series = math.ceil(self.system_voltage / self.cell_voltage)
        self.total_cell_mass_kg = self.final_batt_mass_kg * self.cell_to_pack_mass_ratio
        initial_cell_count_estimate = math.ceil(self.total_cell_mass_kg / self.cell_mass_kg) 

        self.cells_parallel = (initial_cell_count_estimate // self.cells_series) + 1
        self.cell_count = self.cells_series * self.cells_parallel # Final cell count

        # Recalculates final BOL capacity using the final cell count 
        self.gross_BOL_kWh = (self.cells_series * self.cell_voltage *
                              self.cells_parallel * self.cell_charge_ah) / W_PER_KW

        # Recalculating EOL and charge using the final cell count BOL capacity
        self.usable_EOL_kWh = (self.gross_BOL_kWh * self.batt_soh *
                               (1.0 - self.batt_inacc_energy_frac))
        self.usable_EOL_Wh = self.usable_EOL_kWh * W_PER_KW
        self.total_charge_ah_BOL = (self.gross_BOL_kWh * W_PER_KW) / self.system_voltage
        self.gross_EOL_kWh = self.gross_BOL_kWh * (1.0 - self.batt_inacc_energy_frac) * self.batt_soh

    def get_usable_EOL_spec_energy(self) -> float:
        # Returns the pack usable EOL specific energy in Wh/kg 
        return self._pack_spec_energy_usable_EOL_Wh_kg

    def get_gross_BOL_spec_energy(self) -> float:
        # Returns the pack gross BOL specific energy in Wh/kg
        return self._pack_spec_energy_gross_BOL_Wh_kg

    def get_gross_BOL_capacity_kwh(self) -> float:
        # Returns the final calculated gross BOL capacity in kWh based on the final cell count
        return self.gross_BOL_kWh

    def get_gross_EOL_capacity_kwh(self) -> float:
        # Returns the final calculated gross BOL capacity in kWh based on the final cell count
        return self.gross_EOL_kWh

    def get_usable_EOL_capacity_kwh(self) -> float:
        # Returns the final calculated usable EOL capacity kWh based on the final cell count
        return self.usable_EOL_kWh

    def get_total_charge_ah_BOL(self) -> float:
        # Returns the final calculated total charge capacity Ah at BOL based on final cell count
        return self.total_charge_ah_BOL

    # C-rate is based on the EOL energy
    # BOL energy might be more common to calculate C-rate though
    def calculate_c_rate(self, power_kw: float) -> float:
        """Calculate C-rate, accounting for parallel modules if present."""
        eol_capacity_kwh = self.get_gross_EOL_capacity_kwh()
        
        # If we have parallel modules, divide the power among them
        if hasattr(self, 'parallel_modules') and self.parallel_modules > 1:
            # Power is distributed across modules
            power_per_module = power_kw / self.parallel_modules
            # Capacity per module is total capacity / modules
            return power_per_module / (eol_capacity_kwh / self.parallel_modules)
        else:
            # Standard calculation for a single module
            return power_kw / eol_capacity_kwh
    
    def calculate_min_capacity_for_c_rate(self, power_kw: float, max_c_rate: float) -> float:
        """Calculate minimum capacity needed to stay within maximum C-rate for a given power."""
        if max_c_rate <= 0:
            return 0.0  # Invalid C-rate
        return power_kw / max_c_rate
    
    def get_module_count(self) -> int:
        """Return the number of parallel battery modules."""
        return getattr(self, 'parallel_modules', 1)
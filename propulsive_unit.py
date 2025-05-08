import math
from base_component import AircraftComponent
from aircraft import Boom, LiftRotor, TiltRotor

class PropulsiveUnit(AircraftComponent):
    """
    A composite component representing a complete propulsion unit consisting of:
    - 1 boom
    - 1 rotor (either lift or tilt)
    - 1 EPU (Electric Propulsion Unit)
    
    This consolidates related components into a single entity for simplified
    weight tracking, drag calculation, and state management.
    """
    def __init__(self, path_to_json: str, rotor_type: str = "lift"):
        """
        Initialize a propulsive unit.
        
        Args:
            path_to_json: Path to configuration file
            rotor_type: Either "lift" for lift rotor or "tilt" for tilt rotor
        """
        super().__init__(path_to_json)
        self.rotor_type = rotor_type.lower()
        
        # Create component instances for reference/compatibility
        self.boom = None
        self.rotor = None
        
        # Boom properties
        self.boom_disk_area_m2 = 0.0
        self.booms_Cd0 = 0.0
        self.booms_CdA_m2 = 0.0
        self.total_booms_Cd0 = 0.0
        
        # Rotor properties
        self.disk_area_m2 = 0.0
        self.rotor_RPM_hover = 0.0
        self.Ct_hover = 0.0
        self.rotor_weight_kg = 0.0
        self.disk_loading_kg_p_m2 = 0.0
        
        # EPU properties
        self.epu_weight_kg = 0.0
        
        # Combined properties
        self.total_weight_kg = 0.0
        self.unit_cd0 = 0.0
        
    def load_variables_from_json(self):
        """Load variables from JSON configuration."""
        super().load_variables_from_json()
        
    def UpdateComponent(self, aircraft_state: dict):
        """Update all component properties based on aircraft state."""
        self.load_variables_from_json()
        
        # Get required parameters
        mtow_kg = aircraft_state.get('mtow_kg')
        wing_ref_area_m2 = aircraft_state.get('wing_ref_area_m2')
        wing_MAC = aircraft_state.get('wing_MAC')
        battery_spec_energy = aircraft_state.get('battery_spec_energy')
        single_EPU_weight_kg = aircraft_state.get('single_epu_weight_kg', 0.0)
        
        # Get configuration values
        boom_drag_area = self._varlist["boom_drag_area"]
        total_rotor_count = self._varlist["rotor_count"]
        rotor_diameter_m = self._varlist["rotor_diameter_m"]
        sound_speed_m_p_s = self._varlist["sound_speed_m_p_s"]
        tip_mach = self._varlist["tip_mach"]
        air_density_sea_level_kg_p_m3 = self._varlist["air_density_sea_level_kg_p_m3"]
        air_density_min_kg_p_m3 = self._varlist["air_density_min_kg_p_m3"]
        rotor_avg_cl = self._varlist["rotor_avg_cl"]
        g_m_p_s2 = self._varlist["g_m_p_s2"]
        
        # Store EPU weight
        self.epu_weight_kg = single_EPU_weight_kg
        
        # Calculate boom properties
        self.disk_area_m2 = self.calc_disk_area_m2(rotor_diameter_m, 1)  # One rotor per unit
        self.booms_Cd0 = self.calc_booms_Cd0(self.disk_area_m2, boom_drag_area, wing_ref_area_m2)
        self.total_booms_Cd0 = self.booms_Cd0
        self.booms_CdA_m2 = self.calc_booms_CdA_m2(self.booms_Cd0, battery_spec_energy)
        boom_weight = self.calc_boom_weight_kg(single_EPU_weight_kg, rotor_diameter_m, wing_MAC)
        
        # Calculate rotor properties
        self.rotor_RPM_hover = self.calc_rotor_RPM_hover(sound_speed_m_p_s, rotor_diameter_m, tip_mach)
        self.Ct_hover = self.calc_Ct_hover(mtow_kg, g_m_p_s2, total_rotor_count, 
                                           air_density_sea_level_kg_p_m3, rotor_diameter_m, self.rotor_RPM_hover)
        rotor_solidity = self.calc_rotor_solidity(self.Ct_hover, rotor_avg_cl)
        over_torque_factor = self.calc_over_torque_factor(total_rotor_count)
        
        # Calculate rotor weight based on type
        if self.rotor_type == "lift":
            self.rotor_weight_kg = self.calc_lift_rotor_weight_kg(
                total_rotor_count, rotor_diameter_m, rotor_solidity, 
                sound_speed_m_p_s, tip_mach, air_density_sea_level_kg_p_m3, 
                air_density_min_kg_p_m3, over_torque_factor
            )
        else:  # tilt rotor
            self.rotor_weight_kg = self.calc_tilt_rotor_weight_kg(
                total_rotor_count, rotor_diameter_m, rotor_solidity, 
                sound_speed_m_p_s, tip_mach, air_density_sea_level_kg_p_m3, 
                air_density_min_kg_p_m3, over_torque_factor
            )
        
        self.disk_loading_kg_p_m2 = self.calc_disk_loading_kg_p_m2(mtow_kg, self.disk_area_m2)
        
        # Calculate total weight - rotor weight already includes EPU weight contribution
        self.total_weight_kg = self.epu_weight_kg + self.rotor_weight_kg / total_rotor_count
        self.weight = self.total_weight_kg
        
        # Calculate drag contribution
        self.unit_cd0 = self.booms_Cd0
        
        # Log component info
        print(f"  PropulsiveUnit ({self.rotor_type}): Weight={self.weight:.2f} kg (EPU: {self.epu_weight_kg:.2f} kg, "
              f"Rotor: {self.rotor_weight_kg / total_rotor_count:.2f} kg), Disk Area={self.disk_area_m2:.2f} m²")
    
    # --- Boom calculation methods ---
    def calc_booms_Cd0(self, d_area, b_area, w_area):
        return d_area / b_area / w_area * 2 / 3
    
    def calc_booms_CdA_m2(self, b_cd0, b_spec_e):
        return b_cd0 * b_spec_e
    
    def calc_boom_weight_kg(self, epu_w, r_d, w_mac):
        w = (0.0412 * (epu_w * 2.2046) ** 1.1433 * 1 ** 1.3762 * 0.4536 +
             6 * 0.2315 * ((1.2 * r_d + w_mac) * 1) ** 1.3476) * 2
        return w / 2

    def calc_disk_area_m2(self, r_d, r_c):
        return math.pi * (r_d / 2) ** 2
    
    # --- Rotor calculation methods ---
    def calc_rotor_RPM_hover(self, sos, r_d, m_tip):
        return sos * m_tip / (r_d / 2) * 30 / math.pi
    
    def calc_Ct_hover(self, mtow, g, r_c, rho, r_d, rpm):
        denom = rho * math.pi * (r_d / 2) ** 4 * (rpm * math.pi / 30) ** 2
        return (mtow * g / r_c) / denom
    
    def calc_disk_loading_kg_p_m2(self, mtow, d_area):
        return mtow / d_area
    
    def calc_disk_loading_english(self, dl_si):
        return dl_si * 0.204816
    
    def calc_rotor_solidity(self, ct, cl):
        return ct * 6 / cl
    
    def calc_over_torque_factor(self, rotor_count):
        return rotor_count / (rotor_count - 2.0) + 0.3
    
    # Lift rotor specific calculations
    def calc_lift_rotor_weight_kg(self, r_count, r_d, r_s, sos, m_tip, rho_sl, rho_min, ot_f):
        t1b = r_d / 2 * 3.2808
        t2b = math.pi * r_d ** 2 / 4 * r_s / r_d * 3.2808
        t3b = sos * m_tip * math.sqrt(rho_sl / rho_min) * math.sqrt(ot_f) * 3.2808
        w = (0.0024419 * 1 * r_count / 2 * 2 ** 0.53479 * (t1b) ** 1.74231 * (t2b) ** 0.77291 * (t3b) ** 0.87562 * 1.1 ** 2.51048 + 
             0.00037547 * 1 ** 1.02958 * r_count / 2 * 2 ** 0.71443 * (t1b) ** 1.99321 * (t2b) ** 0.79577 * (t3b) ** 0.96323 * 1.1 ** 0.46203 * 1.1 ** 2.58473) * 0.4536
        return w
    
    # Tilt rotor specific calculations
    def calc_tilt_rotor_weight_kg(self, r_count, r_d, r_s, sos, m_tip, rho_sl, rho_min, ot_f):
        t1b = r_d / 2 * 3.2808
        t2b = math.pi * r_d * r_s / 2 / 3 * 3.2808
        t3b = sos * m_tip * math.sqrt(rho_sl / rho_min) * math.sqrt(ot_f) * 3.2808
        w = (0.0024419 * 1.1794 * r_count / 2 * 3 ** 0.53479 * (t1b) ** 1.74231 * (t2b) ** 0.77291 * (t3b) ** 0.87562 * 1.1 ** 2.51048 + 
             0.00037547 * 1.1794 ** 1.02958 * r_count / 2 * 3 ** 0.71443 * (t1b) ** 1.99321 * (t2b) ** 0.79577 * (t3b) ** 0.96323 * 1.1 ** 0.46203 * 1.1 ** 2.58473) * 0.4536
        return w
        
    # --- Public interface methods ---
    def get_weight_kg(self) -> float:
        """Return the total weight of the propulsive unit."""
        return self.weight
    
    def get_disk_area_m2(self) -> float:
        """Return the disk area of the propulsive unit."""
        return self.disk_area_m2
    
    def get_cd0(self) -> float:
        """Return the parasite drag coefficient contribution."""
        return self.unit_cd0

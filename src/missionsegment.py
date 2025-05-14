import json
import math
from tabulate import tabulate
import abc
import copy
from aircraft import Aircraft, Wing, Fuselage, HorizontalTail, VerticalTail, LandingGear, Boom, LiftRotor, TiltRotor
from battery import Battery
from utils import (
    calc_hover_shaft_power_k_W,
    calc_batt_C_rate,
    G as g,
    H_PER_S
)

# Calculates the shaft power required for cruise
def calc_cruise_shaft_power_k_W(mtow_kg, g, prop_effic, cruise_speed_m_p_s, cruise_L_p_D):
    thrust_req_n = (mtow_kg * g) / cruise_L_p_D
    shaft_power_w = (thrust_req_n * cruise_speed_m_p_s) / prop_effic
    return shaft_power_w / 1000.0  # convert W to kW

# Calculates the electric power required for cruise
def calc_cruise_electric_power_k_W(cruise_shaft_power_k_w, epu_effic):
    return cruise_shaft_power_k_w / epu_effic

# Calculates the electric power required to hover
def calc_hover_electric_power_k_W(epu_effic, hover_shaft_power_k_W):
    return hover_shaft_power_k_W / epu_effic

class MissionSegment:
    def __init__(self, path_to_json: str, segment_type: str):
        self.path_to_json = path_to_json
        self.segment_type = segment_type
        self._load_static_params_from_json(path_to_json)

        self.power_draw_kw: float 
        self.c_rate: float 
        self.duration_s: float 
        self.energy_kwh: float 
        self.mass_kg: float = 0.0  # Track current mass for this segment
        self.cd0: float = 0.0      # Track current CD0 for this segment

    @abc.abstractmethod
    def _load_static_params_from_json(self, path_to_json: str):
        raise NotImplementedError

    @abc.abstractmethod
    def calculate_performance(self, aircraft: Aircraft):
        raise NotImplementedError

    def get_results(self) -> dict:
        return {
            "Segment Type": self.segment_type,
            "Duration (s)": self.duration_s,
            "Power (kW)": self.power_draw_kw,
            "C-Rate": self.c_rate,
            "Energy (kWh)": self.energy_kwh,
            "Mass (kg)": self.mass_kg,
            "CD0": self.cd0,
        }

class HoverSegment(MissionSegment):
    """Represents a hover segment (takeoff or landing)."""
    def __init__(self, path_to_json: str, segment_type="Hover"):
        super().__init__(path_to_json, segment_type)

    def _load_static_params_from_json(self, path_to_json):
        with open(path_to_json, 'r') as f:
            data = json.load(f)
        self.rho_sl = data["rho_sl"]
        self.fom = data["fom"]
        self.epu_effic = data["epu_effic"]
        # total hover time for takeoff and landing
        self.total_hover_duration_s = data["hover_duration_s"]


    def calculate_performance(self, aircraft: Aircraft):
        """Calculates hover performance."""

        # Use aircraft's current MTOW
        current_mtow_kg = aircraft.mtow_kg
        batt_capacity_kwh = aircraft.get_current_usable_EOL_kwh()
        disk_area = aircraft.get_rotor_disk_area_m2() # Get from aircraft
        print(f"  Rotor disk area: {disk_area:.2f} m²")

        # Pre-calcs using loaded static params
        self.duration_s = self.total_hover_duration_s / 2.0

        # Power calcs
        hover_shaft_power_kw = calc_hover_shaft_power_k_W(current_mtow_kg, g, self.rho_sl, self.fom, disk_area)
        self.power_draw_kw = calc_hover_electric_power_k_W(self.epu_effic, hover_shaft_power_kw)

        # Energy
        self.energy_kwh = calc_hover_energy_k_W_h(self.power_draw_kw, self.duration_s)

        # C-rate
        self.c_rate = calc_batt_C_rate(batt_capacity_kwh, self.power_draw_kw) 


class ClimbSegment(MissionSegment):
    """Represents a climb segment."""
    def __init__(self, path_to_json: str):
        super().__init__(path_to_json, "Climb")
        self.horizontal_distance_km = 0.0  # Track horizontal distance covered during climb

    def _load_static_params_from_json(self, path_to_json):
        with open(path_to_json, 'r') as f:
            data = json.load(f)

        self.rho_sl = data["rho_sl"] 
        self.roc_mps = data["roc_mps"]
        self.prop_effic = data["prop_effic"]
        self.epu_effic = data["epu_effic"]
        self.spac_effic_factor = data.get("spac_effic_factor") 
        self.trim_drag_factor = data.get("trim_drag_factor") 
        self.hover_alt_m = data["hover_alt_m"]
        self.cruise_alt_m = data["cruise_alt_m"]
        self.stall_speed_m_p_s = data["stall_speed_m_p_s"]
        self.cruise_speed_m_p_s = data["cruise_speed_m_p_s"]
        self.num_steps = data.get("num_integration_steps", 40) # Defaults to 40 because it's not important

    def calculate_performance(self, aircraft: Aircraft):
        """Calculates accelerating climb performance using time integration and JSON stall speed."""
        #  Reset Profiles 
        self.detailed_time_points_relative = []
        self.detailed_power_profile_kw = []
        self.peak_power_kw = 0 

        #  Get Aircraft State 
        current_mtow_kg = aircraft.mtow_kg
        batt_capacity_kwh = aircraft.get_current_usable_EOL_kwh()
        wing_area = aircraft.get_wing_area_m2()
        wing_ar = aircraft.get_wing_ar()
        base_cd0 = aircraft.base_cd0

        # Pre-calcs
        alt_diff_m = self.cruise_alt_m - self.hover_alt_m

        self.duration_s = calc_climb_time_s(alt_diff_m, self.roc_mps)

        # Get Start and End Speeds 
        #v_start_mps = self.stall_speed_m_p_s #start speed is assumed to be the stall speed, CHANGE?
        v_start_mps = 20 # arbitrary number because im lost

        # End speed is the cruise speed
        v_end_mps = 40 # MIN POWER AIRSPEED CHANGE
        

        dt = self.duration_s / self.num_steps
        total_energy_kj = 0.0
        total_horizontal_distance_m = 0.0  # Track horizontal distance

        # Calculate Initial Point (t=0)
        # Determine speed at t=0
        speed_at_t0 = v_start_mps
        speed_at_t0 = speed_at_t0

        # Calculate power at t=0
        initial_power_kw = calc_power_at_speed_climbing_k_W(
            mtow_kg=current_mtow_kg, target_speed_mps=speed_at_t0, base_cd0=base_cd0,
            wing_AR=wing_ar, wing_ref_area_m2=wing_area, rho=self.rho_sl,
            rate_of_climb_mps=self.roc_mps, prop_effic=self.prop_effic, epu_effic=self.epu_effic,
            g=g, spac_effic_factor=self.spac_effic_factor, trim_drag_factor=self.trim_drag_factor
        )

        # Store initial point
        self.detailed_time_points_relative.append(0.0)
        self.detailed_power_profile_kw.append(initial_power_kw)
        self.peak_power_kw = initial_power_kw
        last_power_kw = initial_power_kw 

        # Integration Loop using Trapezoidal rule
        for i in range(1, self.num_steps + 1): # Loop from step 1 to N
            current_time = i * dt
            # Calculate speed at current time step
            if v_end_mps > v_start_mps: 
                 current_speed_mps = v_start_mps + (v_end_mps - v_start_mps) * (current_time / self.duration_s)
            else:
                 current_speed_mps = v_end_mps # Maintain target speed

            current_speed_mps = current_speed_mps

            # Calculate horizontal component of velocity (considering climb angle)
            climb_angle_rad = math.atan2(self.roc_mps, current_speed_mps)
            horizontal_speed_mps = current_speed_mps * math.cos(climb_angle_rad)
            
            # Accumulate horizontal distance for this time step
            horizontal_distance_step = horizontal_speed_mps * dt
            total_horizontal_distance_m += horizontal_distance_step

            # Calculate instantaneous power required
            instant_power_kw = calc_power_at_speed_climbing_k_W(
                mtow_kg=current_mtow_kg, target_speed_mps=current_speed_mps, base_cd0=base_cd0,
                wing_AR=wing_ar, wing_ref_area_m2=wing_area, rho=self.rho_sl,
                rate_of_climb_mps=self.roc_mps, prop_effic=self.prop_effic, epu_effic=self.epu_effic,
                g=g, spac_effic_factor=self.spac_effic_factor, trim_drag_factor=self.trim_drag_factor
            )

            # Store detailed point
            self.detailed_time_points_relative.append(current_time)
            self.detailed_power_profile_kw.append(instant_power_kw)

            # Update peak power
            self.peak_power_kw = max(self.peak_power_kw, instant_power_kw)

            # Accumulate energy using Trapezoidal rule
            avg_power_step = (last_power_kw + instant_power_kw) / 2.0
            total_energy_kj += avg_power_step * dt

            # Update last power for next step
            last_power_kw = instant_power_kw

         # Final Calculations
        self.energy_kwh = total_energy_kj / 3600.0
        self.horizontal_distance_km = total_horizontal_distance_m / 1000.0  # Convert m to km

        print(f"  Horizontal distance covered during climb: {self.horizontal_distance_km:.2f} km")
        
        # Calculate average power
        self.power_draw_kw = total_energy_kj / self.duration_s

        # C-rate based on peak power during the segment
        self.c_rate = calc_batt_C_rate(batt_capacity_kwh, self.peak_power_kw) # Based on EOL battery capacity

        print(f"  Climb Segment Calculated: Avg Power={self.power_draw_kw:.2f} kW, Peak Power={self.peak_power_kw:.2f} kW, Energy={self.energy_kwh:.3f} kWh, C-Rate={self.c_rate:.2f}")


class CruiseSegment(MissionSegment):
    """Represents a cruise segment."""
    def __init__(self, path_to_json: str):
        super().__init__(path_to_json, "Cruise")
        self.actual_cruise_distance_km = 0.0  # Store the actual cruise distance after adjustment

    def _load_static_params_from_json(self, path_to_json):
        with open(path_to_json, 'r') as f:
            data = json.load(f)
        self.prop_effic = data["prop_effic"]
        self.epu_effic = data["epu_effic"]
        self.cruise_speed_m_p_s = data["cruise_speed_m_p_s"]
        self.target_range_km = data["target_range_km"]

    def calculate_performance(self, aircraft: Aircraft, previous_horizontal_distance_km=0.0):
        """Calculates cruise performance."""

        # Adjust cruise distance by subtracting the horizontal distance already traveled
        self.actual_cruise_distance_km = self.target_range_km - previous_horizontal_distance_km
        if self.actual_cruise_distance_km < 0:
            print(f"  Warning: Target range {self.target_range_km} km is less than horizontal distance already covered {previous_horizontal_distance_km:.2f} km")
            self.actual_cruise_distance_km = 0.1  # Minimum cruise distance to avoid errors

        print(f"  Adjusting cruise distance: Target={self.target_range_km} km, Previous horizontal={previous_horizontal_distance_km:.2f} km, Actual cruise={self.actual_cruise_distance_km:.2f} km")

        current_mtow_kg = aircraft.mtow_kg
        batt_capacity_kwh = aircraft.get_current_usable_EOL_kwh()
        cruise_L_p_D = aircraft.cruise_L_p_D

        # Use adjusted cruise distance for duration calculation
        self.duration_s = self.actual_cruise_distance_km * 1000.0 / self.cruise_speed_m_p_s

        # Power calc
        cruise_shaft_power_k_w = calc_cruise_shaft_power_k_W(current_mtow_kg, g, self.prop_effic, self.cruise_speed_m_p_s, cruise_L_p_D)
        self.power_draw_kw = calc_cruise_electric_power_k_W(cruise_shaft_power_k_w, self.epu_effic)

        # Energy
        self.energy_kwh = self.power_draw_kw * self.duration_s * H_PER_S

        # C-rate
        self.c_rate = calc_batt_C_rate(batt_capacity_kwh, self.power_draw_kw) # Based on EOL battery capacity


class JettisonCruiseSegment(CruiseSegment):
    """Represents a cruise segment with jettisoned components."""
    def __init__(self, path_to_json: str, jettison_config: dict):
        super().__init__(path_to_json)
        self.segment_type = "Jettison Cruise"
        self.jettison_config = jettison_config
        self.actual_cruise_distance_km = 0.0  # Store the actual cruise distance after adjustment
        
    def calculate_performance(self, aircraft: Aircraft, previous_horizontal_distance_km=0.0):
        """Calculates cruise performance with jettisoned components."""
        # Store original values
        original_mtow = aircraft.mtow_kg
        original_cd0 = aircraft.base_cd0
        original_batt_capacity = aircraft.get_current_usable_EOL_kwh()
        
        # Adjust cruise distance by subtracting the horizontal distance already traveled
        self.actual_cruise_distance_km = self.target_range_km - previous_horizontal_distance_km
        if self.actual_cruise_distance_km < 0:
            print(f"  Warning: Target range {self.target_range_km} km is less than horizontal distance already covered {previous_horizontal_distance_km:.2f} km")
            self.actual_cruise_distance_km = 0.1  # Minimum cruise distance to avoid errors

        print(f"  Adjusting cruise distance: Target={self.target_range_km} km, Previous horizontal={previous_horizontal_distance_km:.2f} km, Actual cruise={self.actual_cruise_distance_km:.2f} km")
        
        try:
            # Find boom components to calculate their mass and drag
            boom_mass = 0
            boom_cd0 = 0
            boom_count = 0
            
            # Find lift rotor components to calculate their mass
            lift_rotor_mass = 0
            lift_rotor_count = 0
            
            # Get component masses
            for comp in aircraft.components:
                if isinstance(comp, Boom) and boom_count < self.jettison_config.get('boom_count'):
                    boom_mass += getattr(comp, 'weight') 
                    boom_cd0 += getattr(comp, 'booms_Cd0') 
                    boom_count += 1
                elif isinstance(comp, LiftRotor) and lift_rotor_count < self.jettison_config.get('lift_rotor_count', 0):
                    lift_rotor_mass += getattr(comp, 'weight') 
                    lift_rotor_count += 1
                    
            # Calculate total mass reduction and battery capacity reduction
            battery_mass_reduction = self.jettison_config.get('battery_mass_kg')
            total_mass_reduction = boom_mass + lift_rotor_mass + battery_mass_reduction
            
            # Calculate adjusted battery capacity based on mass reduction
            # Get total battery mass from the battery component
            total_battery_mass = 0
            for comp in aircraft.components:
                if isinstance(comp, Battery):
                    total_battery_mass = getattr(comp, 'weight', 0)
                    break
            
            # If battery mass is found, calculate capacity reduction proportional to mass reduction
            battery_capacity_reduction = 0
            adjusted_battery_capacity = original_batt_capacity
            if total_battery_mass > 0:
                battery_mass_fraction = battery_mass_reduction / total_battery_mass
                battery_capacity_reduction = original_batt_capacity * battery_mass_fraction
                adjusted_battery_capacity = original_batt_capacity - battery_capacity_reduction
            
            print(f"  Jettisoning {boom_count} booms ({boom_mass:.2f} kg), {lift_rotor_count} lift rotors ({lift_rotor_mass:.2f} kg)")
            print(f"  Jettisoning {battery_mass_reduction:.2f} kg of battery mass ({battery_capacity_reduction:.2f} kWh)")
            print(f"  Remaining battery capacity: {adjusted_battery_capacity:.2f} kWh")
            print(f"  Total mass reduction: {total_mass_reduction:.2f} kg")
            print(f"  Drag reduction: CD0 -{boom_cd0:.6f}")
            
            # Temporarily modify aircraft properties
            aircraft.mtow_kg -= total_mass_reduction
            aircraft.base_cd0 -= boom_cd0
            
            # Store the modified values for the segment results
            self.mass_kg = aircraft.mtow_kg
            self.cd0 = aircraft.base_cd0
            
            # Calculate cruise performance with modified aircraft
            current_mtow_kg = aircraft.mtow_kg
            cruise_L_p_D = aircraft.cruise_L_p_D
            
            # Use adjusted cruise distance for duration calculation
            self.duration_s = self.actual_cruise_distance_km * 1000.0 / self.cruise_speed_m_p_s
            
            # Power calc
            cruise_shaft_power_k_w = calc_cruise_shaft_power_k_W(current_mtow_kg, g, self.prop_effic, self.cruise_speed_m_p_s, cruise_L_p_D)
            self.power_draw_kw = calc_cruise_electric_power_k_W(cruise_shaft_power_k_w, self.epu_effic)

            # Energy
            self.energy_kwh = self.power_draw_kw * self.duration_s * H_PER_S
            
            # C-rate based on adjusted battery capacity
            self.c_rate = calc_batt_C_rate(adjusted_battery_capacity, self.power_draw_kw)
            
            # Restore original values after calculation
            aircraft.mtow_kg = original_mtow
            aircraft.base_cd0 = original_cd0
            
        except Exception as e:
            print(f"  Error in jettison cruise calculation: {e}")
            print("  Using fallback method...")
            
            # Fallback approach with hard-coded estimates
            battery_mass_reduction = self.jettison_config.get('battery_mass_kg', 0)
            boom_mass = self.jettison_config.get('boom_count', 0) * 15.0  # CHANGE SHOULD GET THESE FROM ROTOR/ BOOM CLASSES
            lift_rotor_mass = self.jettison_config.get('lift_rotor_count', 0) * 10.0  
            boom_cd0 = self.jettison_config.get('boom_count', 0) * 0.0018  
            
            total_mass_reduction = boom_mass + lift_rotor_mass + battery_mass_reduction
            
            # Estimate capacity reduction based on battery mass
            # Assume 200 Wh/kg energy density for the battery
            battery_capacity_reduction = battery_mass_reduction * .129  # CHANGE THIS TO GET FROM BATTERY CLASS
            adjusted_battery_capacity = original_batt_capacity - battery_capacity_reduction
            
            print(f"  Using estimated values:")
            print(f"  - Battery capacity reduction: {battery_capacity_reduction:.2f} kWh")
            print(f"  - Remaining battery capacity: {adjusted_battery_capacity:.2f} kWh")
            print(f"  - Total mass reduction: {total_mass_reduction:.2f} kg")
            print(f"  - Drag reduction: CD0 -{boom_cd0:.6f}")
            
            # Temporarily modify aircraft properties
            aircraft.mtow_kg -= total_mass_reduction
            aircraft.base_cd0 -= boom_cd0
            
            try:
                # Calculate performance with modified aircraft but use adjusted cruise distance
                current_mtow_kg = aircraft.mtow_kg
                cruise_L_p_D = aircraft.cruise_L_p_D
                
                # Use adjusted cruise distance for duration calculation in fallback method
                self.duration_s = self.actual_cruise_distance_km * 1000.0 / self.cruise_speed_m_p_s
                
                # Power calc
                cruise_shaft_power_k_w = calc_cruise_shaft_power_k_W(current_mtow_kg, g, self.prop_effic, self.cruise_speed_m_p_s, cruise_L_p_D)
                self.power_draw_kw = calc_cruise_electric_power_k_W(cruise_shaft_power_k_w, self.epu_effic)
                
                # Energy
                self.energy_kwh = self.power_draw_kw * self.duration_s * H_PER_S
                
                # C-rate based on adjusted battery capacity
                self.c_rate = calc_batt_C_rate(adjusted_battery_capacity, self.power_draw_kw)
            finally:
                # Restore original values
                aircraft.mtow_kg = original_mtow
                aircraft.base_cd0 = original_cd0


class JettisonClimbSegment(ClimbSegment):
    """Represents a climb segment with jettisoned components."""
    def __init__(self, path_to_json: str, jettison_config: dict):
        super().__init__(path_to_json)
        self.segment_type = "Jettison Climb"
        self.jettison_config = jettison_config
        
    def calculate_performance(self, aircraft: Aircraft):
        """Calculates climb performance with jettisoned components."""
        # Store original values
        original_mtow = aircraft.mtow_kg
        original_cd0 = aircraft.base_cd0
        original_batt_capacity = aircraft.get_current_usable_EOL_kwh()
        
        try:
            # Find boom components to calculate their mass and drag
            boom_mass = 0
            boom_cd0 = 0
            boom_count = 0
            
            # Find lift rotor components to calculate their mass
            lift_rotor_mass = 0
            lift_rotor_count = 0
            
            # Get component masses
            for comp in aircraft.components:
                if isinstance(comp, Boom) and boom_count < self.jettison_config.get('boom_count'):
                    boom_mass += getattr(comp, 'weight') 
                    boom_cd0 += getattr(comp, 'booms_Cd0') 
                    boom_count += 1
                elif isinstance(comp, LiftRotor) and lift_rotor_count < self.jettison_config.get('lift_rotor_count', 0):
                    lift_rotor_mass += getattr(comp, 'weight') 
                    lift_rotor_count += 1
                    
            # Calculate total mass reduction and battery capacity reduction
            battery_mass_reduction = self.jettison_config.get('battery_mass_kg')
            total_mass_reduction = boom_mass + lift_rotor_mass + battery_mass_reduction
            
            # Calculate adjusted battery capacity based on mass reduction
            # Get total battery mass from the battery component
            total_battery_mass = 0
            for comp in aircraft.components:
                if isinstance(comp, Battery):
                    total_battery_mass = getattr(comp, 'weight', 0)
                    break
            
            # If battery mass is found, calculate capacity reduction proportional to mass reduction
            battery_capacity_reduction = 0
            adjusted_battery_capacity = original_batt_capacity
            if total_battery_mass > 0:
                battery_mass_fraction = battery_mass_reduction / total_battery_mass
                battery_capacity_reduction = original_batt_capacity * battery_mass_fraction
                adjusted_battery_capacity = original_batt_capacity - battery_capacity_reduction
            
            print(f"  Jettisoning {boom_count} booms ({boom_mass:.2f} kg), {lift_rotor_count} lift rotors ({lift_rotor_mass:.2f} kg)")
            print(f"  Jettisoning {battery_mass_reduction:.2f} kg of battery mass ({battery_capacity_reduction:.2f} kWh)")
            print(f"  Remaining battery capacity: {adjusted_battery_capacity:.2f} kWh")
            print(f"  Total mass reduction: {total_mass_reduction:.2f} kg")
            print(f"  Drag reduction: CD0 -{boom_cd0:.6f}")
            
            # Temporarily modify aircraft properties
            aircraft.mtow_kg -= total_mass_reduction
            aircraft.base_cd0 -= boom_cd0
            
            # Store the modified values for the segment results
            self.mass_kg = aircraft.mtow_kg
            self.cd0 = aircraft.base_cd0
            
            # Continue with standard climb performance calculation using reduced mass and drag
            # Reset profiles for this segment
            self.detailed_time_points_relative = []
            self.detailed_power_profile_kw = []
            self.peak_power_kw = 0 
            
            # Initialize horizontal distance tracking
            total_horizontal_distance_m = 0.0  # Track horizontal distance

            # Get aircraft state with reduced mass
            current_mtow_kg = aircraft.mtow_kg
            batt_capacity_kwh = adjusted_battery_capacity  # Use reduced battery capacity
            wing_area = aircraft.get_wing_area_m2()
            wing_ar = aircraft.get_wing_ar()
            base_cd0 = aircraft.base_cd0  # Use reduced CD0

            # Standard climb calculations from ClimbSegment
            alt_diff_m = self.cruise_alt_m - self.hover_alt_m
            self.duration_s = calc_climb_time_s(alt_diff_m, self.roc_mps)

            v_start_mps = self.stall_speed_m_p_s  # Start speed is assumed to be the stall speed
            v_end_mps = self.cruise_speed_m_p_s
            
            dt = self.duration_s / self.num_steps
            total_energy_kj = 0.0

            # Calculate Initial Point (t=0)
            speed_at_t0 = v_start_mps
            
            initial_power_kw = calc_power_at_speed_climbing_k_W(
                mtow_kg=current_mtow_kg, target_speed_mps=speed_at_t0, base_cd0=base_cd0,
                wing_AR=wing_ar, wing_ref_area_m2=wing_area, rho=self.rho_sl,
                rate_of_climb_mps=self.roc_mps, prop_effic=self.prop_effic, epu_effic=self.epu_effic,
                g=g, spac_effic_factor=self.spac_effic_factor, trim_drag_factor=self.trim_drag_factor
            )

            self.detailed_time_points_relative.append(0.0)
            self.detailed_power_profile_kw.append(initial_power_kw)
            self.peak_power_kw = initial_power_kw
            last_power_kw = initial_power_kw

            # Integration Loop using Trapezoidal rule
            for i in range(1, self.num_steps + 1):
                current_time = i * dt
                # Calculate speed at current time step
                if v_end_mps > v_start_mps: 
                    current_speed_mps = v_start_mps + (v_end_mps - v_start_mps) * (current_time / self.duration_s)
                else:
                    current_speed_mps = v_end_mps # Maintain target speed
                
                # Calculate horizontal component of velocity (considering climb angle)
                climb_angle_rad = math.atan2(self.roc_mps, current_speed_mps)
                horizontal_speed_mps = current_speed_mps * math.cos(climb_angle_rad)
                
                # Accumulate horizontal distance for this time step
                horizontal_distance_step = horizontal_speed_mps * dt
                total_horizontal_distance_m += horizontal_distance_step

                # Calculate instantaneous power required
                instant_power_kw = calc_power_at_speed_climbing_k_W(
                    mtow_kg=current_mtow_kg, target_speed_mps=current_speed_mps, base_cd0=base_cd0,
                    wing_AR=wing_ar, wing_ref_area_m2=wing_area, rho=self.rho_sl,
                    rate_of_climb_mps=self.roc_mps, prop_effic=self.prop_effic, epu_effic=self.epu_effic,
                    g=g, spac_effic_factor=self.spac_effic_factor, trim_drag_factor=self.trim_drag_factor
                )

                # Store detailed point
                self.detailed_time_points_relative.append(current_time)
                self.detailed_power_profile_kw.append(instant_power_kw)

                # Update peak power
                self.peak_power_kw = max(self.peak_power_kw, instant_power_kw)

                # Accumulate energy using Trapezoidal rule
                avg_power_step = (last_power_kw + instant_power_kw) / 2.0
                total_energy_kj += avg_power_step * dt

                # Update last power for next step
                last_power_kw = instant_power_kw
            
            # Final Calculations
            self.energy_kwh = total_energy_kj / 3600.0
            self.horizontal_distance_km = total_horizontal_distance_m / 1000.0  # Convert m to km
            
            print(f"  Horizontal distance covered during jettison climb: {self.horizontal_distance_km:.2f} km")
            
            # Calculate average power
            self.power_draw_kw = total_energy_kj / self.duration_s

            # C-rate based on peak power during the segment and adjusted battery capacity
            self.c_rate = calc_batt_C_rate(adjusted_battery_capacity, self.peak_power_kw)
            
            print(f"  Jettison Climb Segment Calculated: Avg Power={self.power_draw_kw:.2f} kW, Peak Power={self.peak_power_kw:.2f} kW, Energy={self.energy_kwh:.3f} kWh, C-Rate={self.c_rate:.2f}")
            
            # Restore original values after calculation
            aircraft.mtow_kg = original_mtow
            aircraft.base_cd0 = original_cd0
            
        except Exception as e:
            print(f"  Error in jettison climb calculation: {e}")
            print("  Using fallback method...")
            
            # Fallback approach with hard-coded estimates
            battery_mass_reduction = self.jettison_config.get('battery_mass_kg', 0)
            boom_mass = self.jettison_config.get('boom_count', 0) * 15.0
            lift_rotor_mass = self.jettison_config.get('lift_rotor_count', 0) * 10.0
            boom_cd0 = self.jettison_config.get('boom_count', 0) * 0.0018
            
            total_mass_reduction = boom_mass + lift_rotor_mass + battery_mass_reduction
            
            # Estimate capacity reduction based on battery mass
            battery_capacity_reduction = battery_mass_reduction * 0.129
            adjusted_battery_capacity = original_batt_capacity - battery_capacity_reduction
            
            print(f"  Using estimated values:")
            print(f"  - Battery capacity reduction: {battery_capacity_reduction:.2f} kWh")
            print(f"  - Remaining battery capacity: {adjusted_battery_capacity:.2f} kWh")
            print(f"  - Total mass reduction: {total_mass_reduction:.2f} kg")
            print(f"  - Drag reduction: CD0 -{boom_cd0:.6f}")
            
            # Temporarily modify aircraft properties
            aircraft.mtow_kg -= total_mass_reduction
            aircraft.base_cd0 -= boom_cd0
            
            # Store the reduced mass and CD0 for this segment
            self.mass_kg = aircraft.mtow_kg
            self.cd0 = aircraft.base_cd0
            
            # Run a simplified version of the climb calculation
            try:
                # Override with base class implementation to get detailed calculations
                # but use the modified aircraft state
                super().calculate_performance(aircraft)
                
                # Adjust C-rate calculation to use the reduced battery capacity
                self.c_rate = calc_batt_C_rate(adjusted_battery_capacity, self.peak_power_kw)
            finally:
                # Restore original values
                aircraft.mtow_kg = original_mtow
                aircraft.base_cd0 = original_cd0

# ...existing code...

# Update the JettisonCruiseSegment class to store mass and CD0 explicitly
class JettisonCruiseSegment(CruiseSegment):
    """Represents a cruise segment with jettisoned components."""
    def __init__(self, path_to_json: str, jettison_config: dict):
        super().__init__(path_to_json)
        self.segment_type = "Jettison Cruise"
        self.jettison_config = jettison_config
        self.actual_cruise_distance_km = 0.0  # Store the actual cruise distance after adjustment
        
    def calculate_performance(self, aircraft: Aircraft, previous_horizontal_distance_km=0.0):
        """Calculates cruise performance with jettisoned components."""
        # Store original values
        original_mtow = aircraft.mtow_kg
        original_cd0 = aircraft.base_cd0
        original_batt_capacity = aircraft.get_current_usable_EOL_kwh()
        
        # Adjust cruise distance by subtracting the horizontal distance already traveled
        self.actual_cruise_distance_km = self.target_range_km - previous_horizontal_distance_km
        if self.actual_cruise_distance_km < 0:
            print(f"  Warning: Target range {self.target_range_km} km is less than horizontal distance already covered {previous_horizontal_distance_km:.2f} km")
            self.actual_cruise_distance_km = 0.1  # Minimum cruise distance to avoid errors

        print(f"  Adjusting cruise distance: Target={self.target_range_km} km, Previous horizontal={previous_horizontal_distance_km:.2f} km, Actual cruise={self.actual_cruise_distance_km:.2f} km")
        
        try:
            # Find boom components to calculate their mass and drag
            boom_mass = 0
            boom_cd0 = 0
            boom_count = 0
            
            # Find lift rotor components to calculate their mass
            lift_rotor_mass = 0
            lift_rotor_count = 0
            
            # Get component masses
            for comp in aircraft.components:
                if isinstance(comp, Boom) and boom_count < self.jettison_config.get('boom_count'):
                    boom_mass += getattr(comp, 'weight') 
                    boom_cd0 += getattr(comp, 'booms_Cd0') 
                    boom_count += 1
                elif isinstance(comp, LiftRotor) and lift_rotor_count < self.jettison_config.get('lift_rotor_count', 0):
                    lift_rotor_mass += getattr(comp, 'weight') 
                    lift_rotor_count += 1
                    
            # Calculate total mass reduction and battery capacity reduction
            battery_mass_reduction = self.jettison_config.get('battery_mass_kg')
            total_mass_reduction = boom_mass + lift_rotor_mass + battery_mass_reduction
            
            # Calculate adjusted battery capacity based on mass reduction
            # Get total battery mass from the battery component
            total_battery_mass = 0
            for comp in aircraft.components:
                if isinstance(comp, Battery):
                    total_battery_mass = getattr(comp, 'weight', 0)
                    break
            
            # If battery mass is found, calculate capacity reduction proportional to mass reduction
            battery_capacity_reduction = 0
            adjusted_battery_capacity = original_batt_capacity
            if total_battery_mass > 0:
                battery_mass_fraction = battery_mass_reduction / total_battery_mass
                battery_capacity_reduction = original_batt_capacity * battery_mass_fraction
                adjusted_battery_capacity = original_batt_capacity - battery_capacity_reduction
            
            print(f"  Jettisoning {boom_count} booms ({boom_mass:.2f} kg), {lift_rotor_count} lift rotors ({lift_rotor_mass:.2f} kg)")
            print(f"  Jettisoning {battery_mass_reduction:.2f} kg of battery mass ({battery_capacity_reduction:.2f} kWh)")
            print(f"  Remaining battery capacity: {adjusted_battery_capacity:.2f} kWh")
            print(f"  Total mass reduction: {total_mass_reduction:.2f} kg")
            print(f"  Drag reduction: CD0 -{boom_cd0:.6f}")
            
            # Temporarily modify aircraft properties
            aircraft.mtow_kg -= total_mass_reduction
            aircraft.base_cd0 -= boom_cd0
            
            # Store the modified values for the segment results
            self.mass_kg = aircraft.mtow_kg
            self.cd0 = aircraft.base_cd0
            
            # Calculate cruise performance with modified aircraft
            current_mtow_kg = aircraft.mtow_kg
            cruise_L_p_D = aircraft.cruise_L_p_D
            
            # Use adjusted cruise distance for duration calculation
            self.duration_s = self.actual_cruise_distance_km * 1000.0 / self.cruise_speed_m_p_s
            
            # Power calc
            cruise_shaft_power_k_w = calc_cruise_shaft_power_k_W(current_mtow_kg, g, self.prop_effic, self.cruise_speed_m_p_s, cruise_L_p_D)
            self.power_draw_kw = calc_cruise_electric_power_k_W(cruise_shaft_power_k_w, self.epu_effic)

            # Energy
            self.energy_kwh = self.power_draw_kw * self.duration_s * H_PER_S
            
            # C-rate based on adjusted battery capacity
            self.c_rate = calc_batt_C_rate(adjusted_battery_capacity, self.power_draw_kw)
            
            # Restore original values after calculation
            aircraft.mtow_kg = original_mtow
            aircraft.base_cd0 = original_cd0
            
        except Exception as e:
            print(f"  Error in jettison cruise calculation: {e}")
            print("  Using fallback method...")
            
            # Fallback approach with hard-coded estimates
            battery_mass_reduction = self.jettison_config.get('battery_mass_kg', 0)
            boom_mass = self.jettison_config.get('boom_count', 0) * 15.0  # CHANGE SHOULD GET THESE FROM ROTOR/ BOOM CLASSES
            lift_rotor_mass = self.jettison_config.get('lift_rotor_count', 0) * 10.0  
            boom_cd0 = self.jettison_config.get('boom_count', 0) * 0.0018  
            
            total_mass_reduction = boom_mass + lift_rotor_mass + battery_mass_reduction
            
            # Estimate capacity reduction based on battery mass
            # Assume 200 Wh/kg energy density for the battery
            battery_capacity_reduction = battery_mass_reduction * .129  # CHANGE THIS TO GET FROM BATTERY CLASS
            adjusted_battery_capacity = original_batt_capacity - battery_capacity_reduction
            
            print(f"  Using estimated values:")
            print(f"  - Battery capacity reduction: {battery_capacity_reduction:.2f} kWh")
            print(f"  - Remaining battery capacity: {adjusted_battery_capacity:.2f} kWh")
            print(f"  - Total mass reduction: {total_mass_reduction:.2f} kg")
            print(f"  - Drag reduction: CD0 -{boom_cd0:.6f}")
            
            # Temporarily modify aircraft properties
            aircraft.mtow_kg -= total_mass_reduction
            aircraft.base_cd0 -= boom_cd0
            
            try:
                # Calculate performance with modified aircraft but use adjusted cruise distance
                current_mtow_kg = aircraft.mtow_kg
                cruise_L_p_D = aircraft.cruise_L_p_D
                
                # Use adjusted cruise distance for duration calculation in fallback method
                self.duration_s = self.actual_cruise_distance_km * 1000.0 / self.cruise_speed_m_p_s
                
                # Power calc
                cruise_shaft_power_k_w = calc_cruise_shaft_power_k_W(current_mtow_kg, g, self.prop_effic, self.cruise_speed_m_p_s, cruise_L_p_D)
                self.power_draw_kw = calc_cruise_electric_power_k_W(cruise_shaft_power_k_w, self.epu_effic)
                
                # Energy
                self.energy_kwh = self.power_draw_kw * self.duration_s * H_PER_S
                
                # C-rate based on adjusted battery capacity
                self.c_rate = calc_batt_C_rate(adjusted_battery_capacity, self.power_draw_kw)
            finally:
                # Restore original values
                aircraft.mtow_kg = original_mtow
                aircraft.base_cd0 = original_cd0


class ReserveSegment(MissionSegment):
    """Represents a reserve segment (typically cruise + hover)."""
    def __init__(self, path_to_json: str):
         # Combine Hover and Cruise for reserve
        super().__init__(path_to_json, "Reserve")

    def _load_static_params_from_json(self, path_to_json):
        with open(path_to_json, 'r') as f:
            data = json.load(f)
        self.rho_sl = data["rho_sl"]
        self.fom = data["fom"]
        self.reserve_hover_dur_s = data["reserve_hover_dur_s"]
        self.prop_effic = data["prop_effic"]
        self.epu_effic = data["epu_effic"] 
        self.cruise_speed_m_p_s = data["cruise_speed_m_p_s"] 
        self.reserve_range_km = data["reserve_range_km"]


    def calculate_performance(self, aircraft: Aircraft):
        """Calculates reserve performance."""

        current_mtow_kg = aircraft.mtow_kg # Using current MTOW
        batt_capacity_kwh = aircraft.get_current_usable_EOL_kwh()
        cruise_L_p_D = aircraft.cruise_L_p_D
        disk_area = aircraft.get_rotor_disk_area_m2()

        # Calculate Reserve Hover 
        hover_shaft_power_kw = calc_hover_shaft_power_k_W(current_mtow_kg, g, self.rho_sl, self.fom, disk_area)
        hover_power_kw = calc_hover_electric_power_k_W(self.epu_effic, hover_shaft_power_kw)
        hover_energy_kwh = calc_hover_energy_k_W_h(hover_power_kw, self.reserve_hover_dur_s)
        hover_c_rate = calc_batt_C_rate(batt_capacity_kwh, hover_power_kw)

        # Calculate Reserve Cruise 
        cruise_duration_s = 0
        cruise_duration_s = self.reserve_range_km * 1000.0 / self.cruise_speed_m_p_s

        cruise_shaft_power_kw = calc_cruise_shaft_power_k_W(current_mtow_kg, g, self.prop_effic, self.cruise_speed_m_p_s, cruise_L_p_D)
        cruise_power_kw = calc_cruise_electric_power_k_W(cruise_shaft_power_kw, self.epu_effic)
        cruise_energy_kwh = calc_reserve_cruise_energy_k_W_h(self.cruise_speed_m_p_s, self.reserve_range_km, cruise_power_kw)
        cruise_c_rate = calc_batt_C_rate(batt_capacity_kwh, cruise_power_kw)

        #  Combine Reserve Segments 
        self.duration_s = self.reserve_hover_dur_s + cruise_duration_s
        self.energy_kwh = hover_energy_kwh + cruise_energy_kwh

        self.power_draw_kw = max(hover_power_kw, cruise_power_kw)
        self.c_rate = max(hover_c_rate, cruise_c_rate)


#  Mission Class 
class Mission:
    """Defines and simulates a complete mission profile."""
    def __init__(self, aircraft: Aircraft, mission_segments: list[MissionSegment]):
        self.aircraft = aircraft
        self.mission_segments = copy.deepcopy(mission_segments)
        self.time_points_s: list[float] = []
        self.power_profile_kw: list[float] = []
        self.c_rate_profile: list[float] = []
        self.segment_results: list[dict] = []
        self.total_duration_s: float = 0.0
        self.total_energy_kwh: float = 0.0
        self.max_power_kw: float = 0.0
        self.max_c_rate: float = 0.0
        self.total_horizontal_distance_km: float = 0.0  # Track total horizontal distance
        self.is_calculated = False

    def run_mission(self):
        self.time_points_s = []
        self.power_profile_kw = []
        self.c_rate_profile = [] 
        self.segment_results = []
        self.total_duration_s = 0.0
        self.total_energy_kwh = 0.0
        self.max_power_kw = 0.0
        self.max_c_rate = 0.0
        current_time_s = 0.0
        last_segment_c_rate = 0.0 

        print(f"Running mission simulation for Aircraft with MTOW: {self.aircraft.mtow_kg:.2f} kg")
        print(f"EOL Battery Capacity: {self.aircraft.get_current_usable_EOL_kwh():.2f} kWh")
        print("-" * 30)

        previous_horizontal_distance_km = 0.0
        total_horizontal_distance_km = 0.0  # Initialize total horizontal distance tracker
        
        for i, segment in enumerate(self.mission_segments):
            print(f"Calculating Segment {i+1}: {segment.segment_type}...")
            segment_start_time = current_time_s

            # Check if previous segment was a climb to account for horizontal distance
            if i > 0 and isinstance(self.mission_segments[i-1], ClimbSegment) and isinstance(segment, CruiseSegment):
                previous_horizontal_distance_km = self.mission_segments[i-1].horizontal_distance_km
                print(f"  Using horizontal distance from previous climb: {previous_horizontal_distance_km:.2f} km")
            
            # Calculate Performance 
            try:
                if isinstance(segment, CruiseSegment) and previous_horizontal_distance_km > 0:
                    segment.calculate_performance(self.aircraft, previous_horizontal_distance_km)
                    # Track horizontal distance from cruise segment
                    total_horizontal_distance_km += segment.actual_cruise_distance_km
                    print(f"  Total horizontal distance so far: {total_horizontal_distance_km:.2f} km")
                else:
                    segment.calculate_performance(self.aircraft)
                
                # Store horizontal distance if this is a climb segment
                if isinstance(segment, ClimbSegment):
                    previous_horizontal_distance_km = segment.horizontal_distance_km
                    total_horizontal_distance_km += segment.horizontal_distance_km
                    print(f"  Total horizontal distance so far: {total_horizontal_distance_km:.2f} km")
                elif isinstance(segment, CruiseSegment) and not hasattr(segment, 'actual_cruise_distance_km'):
                    # If it's a cruise segment without adjusted distance (no prior climb)
                    total_horizontal_distance_km += segment.target_range_km
                    print(f"  Total horizontal distance so far: {total_horizontal_distance_km:.2f} km")
                
            except Exception as e:
                 print(f"  ERROR calculating performance for segment {i+1} ({segment.segment_type}): {e}")
                 segment.duration_s = 0.0
                 segment.power_draw_kw = 0.0
                 segment.energy_kwh = 0.0
                 segment.c_rate = 0.0
                 segment.peak_power_kw = 0.0 

            seg_duration = getattr(segment, 'duration_s')
            seg_avg_power = getattr(segment, 'power_draw_kw')
            seg_energy = getattr(segment, 'energy_kwh')
            seg_c_rate = getattr(segment, 'c_rate')
            seg_peak_power = getattr(segment, 'peak_power_kw', seg_avg_power)

            #  Store segment summary results 
            self.segment_results.append(segment.get_results())

            #  Update Mission Totals & Maxima 
            self.total_duration_s += seg_duration
            self.total_energy_kwh += seg_energy
            self.max_power_kw = max(self.max_power_kw, seg_peak_power) # Use peak power for max
            self.max_c_rate = max(self.max_c_rate, seg_c_rate)

            # Uses the detailed power if the segment has it (only climb at the moment)
            has_detailed_power = (hasattr(segment, 'detailed_time_points_relative') and
                                  hasattr(segment, 'detailed_power_profile_kw') and
                                  segment.detailed_time_points_relative and 
                                  segment.detailed_power_profile_kw) 

            # Build Profiles 
            last_point_time = self.time_points_s[-1] if self.time_points_s else -1.0
            last_point_power = self.power_profile_kw[-1] if self.power_profile_kw else 0.0

            if has_detailed_power:
                print(f"  Segment {i+1} using detailed power profile. Appending {len(segment.detailed_power_profile_kw)} points.")

                # Add point at segment start to hold previous segment's C-rate until power changes
                if not self.time_points_s or segment_start_time > last_point_time:
                    self.time_points_s.append(segment_start_time)
                    self.power_profile_kw.append(last_point_power) # Hold previous power
                    self.c_rate_profile.append(last_segment_c_rate) # Hold previous c-rate

                # Iterate through the detailed points
                current_last_power = last_point_power # Track power within this segment's points
                for idx in range(len(segment.detailed_time_points_relative)):
                    rel_time = segment.detailed_time_points_relative[idx]
                    power = segment.detailed_power_profile_kw[idx]
                    abs_time = segment_start_time + rel_time

                    current_last_profile_time = self.time_points_s[-1] if self.time_points_s else -1.0
                    if abs_time <= current_last_profile_time:
                        if abs_time == current_last_profile_time:
                            self.power_profile_kw[-1] = power
                            self.c_rate_profile[-1] = seg_c_rate # Use this segment's C-rate now
                        continue 

                    # Add point just current time with the previous power level
                    self.time_points_s.append(abs_time)
                    self.power_profile_kw.append(current_last_power)
                    # Use the C-rate associated with the start of the segment until the end
                    self.c_rate_profile.append(seg_c_rate)

                    # Add point at this time with the new power level
                    self.time_points_s.append(abs_time)
                    self.power_profile_kw.append(power)
                    self.c_rate_profile.append(seg_c_rate) # Apply segment's C-rate

                    current_last_power = power

                segment_end_time = segment_start_time + seg_duration
                current_last_profile_time = self.time_points_s[-1] if self.time_points_s else -1.0
                if segment_end_time > current_last_profile_time:
                    self.time_points_s.append(segment_end_time)
                    self.power_profile_kw.append(current_last_power) # Hold last power
                    self.c_rate_profile.append(seg_c_rate) # Hold segment C-rate

            else:
                #  Branch for average power segments (all but climb) 
                print(f"  Segment {i+1} using average power: {seg_avg_power:.2f} kW")

                if not self.time_points_s or segment_start_time > last_point_time:
                     self.time_points_s.append(segment_start_time)
                     self.power_profile_kw.append(last_point_power)
                     self.c_rate_profile.append(last_segment_c_rate) 

                self.time_points_s.append(segment_start_time)
                self.power_profile_kw.append(seg_avg_power)
                self.c_rate_profile.append(seg_c_rate)

                segment_end_time = segment_start_time + seg_duration
                current_last_profile_time = self.time_points_s[-1] if self.time_points_s else -1.0
                if segment_end_time > current_last_profile_time: 
                    self.time_points_s.append(segment_end_time)
                    self.power_profile_kw.append(seg_avg_power)
                    self.c_rate_profile.append(seg_c_rate)


            current_time_s += seg_duration
            last_segment_c_rate = seg_c_rate 

            print(f"  Duration: {seg_duration:.1f} s, Avg Power: {seg_avg_power:.2f} kW, Peak Power: {seg_peak_power:.2f} kW, C-Rate: {seg_c_rate:.2f}, Energy: {seg_energy:.3f} kWh")
            print(f"  Profile points added. Current time: {current_time_s:.1f} s. Total points: {len(self.time_points_s)}")

        # Remove duplicate consecutive time points
        clean_time = []
        clean_power = []
        clean_crate = []
        if self.time_points_s:
            last_t = -1.0
            for t, p, c in zip(self.time_points_s, self.power_profile_kw, self.c_rate_profile):
                 if t > last_t:
                     clean_time.append(t)
                     clean_power.append(p)
                     clean_crate.append(c)
                     last_t = t
                 else:
                      if clean_power: clean_power[-1] = p
                      if clean_crate: clean_crate[-1] = c

            self.time_points_s = clean_time
            self.power_profile_kw = clean_power
            self.c_rate_profile = clean_crate
            print(f"  Profile cleanup complete. Final points: {len(self.time_points_s)}")


        # Store the total horizontal distance
        self.total_horizontal_distance_km = total_horizontal_distance_km
        print(f"Total horizontal distance traveled: {self.total_horizontal_distance_km:.2f} km")
        
        self.is_calculated = True
        print("-" * 30)
        print("Mission calculation complete.")

    def display_summary(self):
        """Prints a summary table of the mission results."""
        if not self.is_calculated:
            print("Mission not calculated yet. Run `run_mission()` first.")
            return

        print("\n Mission Summary ")
        if self.segment_results:
            # Tabulate makes it look pretty
            headers = self.segment_results[0].keys()
            rows = [list(result.values()) for result in self.segment_results]
            formatted_rows = []
            for row in rows:
                 formatted_row = [row[0]] 
                 formatted_row.append(f"{row[1]:.1f}" if isinstance(row[1], (int, float)) else row[1]) # Duration
                 formatted_row.append(f"{row[2]:.2f}" if isinstance(row[2], (int, float)) else row[2]) # Power
                 formatted_row.append(f"{row[3]:.2f}" if isinstance(row[3], (int, float)) else row[3]) # C-Rate
                 formatted_row.append(f"{row[4]:.3f}" if isinstance(row[4], (int, float)) else row[4]) # Energy
                 formatted_row.append(f"{row[5]:.1f}" if isinstance(row[5], (int, float)) else row[5]) # Mass
                 formatted_row.append(f"{row[6]:.4f}" if isinstance(row[6], (int, float)) else row[6]) # CD0
                 formatted_rows.append(formatted_row)

            print(tabulate(formatted_rows, headers=headers, tablefmt="grid"))
        else:
            print("No valid segment results to display.")

        print("\n Mission Totals & Maxima ")
        summary_data = [
            ["Total Duration", f"{self.total_duration_s:.1f} s"],
            ["Total Energy", f"{self.total_energy_kwh:.3f} kWh"],
            ["Total Horizontal Distance", f"{self.total_horizontal_distance_km:.2f} km"],
            ["Max Power Draw", f"{self.max_power_kw:.2f} kW"],
            ["Max C-Rate", f"{self.max_c_rate:.2f}"],
        ]
        print(tabulate(summary_data, tablefmt="plain"))

    def get_power_profile(self) -> tuple[list[float], list[float]]:
        """Returns the time points and power values for the profile."""
        if not self.is_calculated:
            print("Warning: Mission not calculated. Returning empty profiles.")
            return [], []
        return self.time_points_s, self.power_profile_kw

    def get_c_rate_profile(self) -> tuple[list[float], list[float]]:
        """Returns the time points and C-rate values for the profile."""
        if not self.is_calculated:
            print("Warning: Mission not calculated. Returning empty profiles.")
            return [], []
        return self.time_points_s, self.c_rate_profile


# New Class for Shorter Range Mission
class ShorterRangeMission(Mission):
    """
    Used to represent the "common case" example. An eVTOL needs to be sized for a maximum distance but the average 
    mission is shorter than that.
    """
    def __init__(self, aircraft: Aircraft, base_mission_segments: list[MissionSegment], shorter_range_km: float):
        """
        Initializes the ShorterRangeMission.
        """
        # Copy the base segments to modify them locally
        modified_segments = copy.deepcopy(base_mission_segments)
        cruise_modified = False
        for seg in modified_segments:
            if isinstance(seg, CruiseSegment):
                print(f"  Modifying CruiseSegment range from {seg.target_range_km} km to {shorter_range_km} km for shorter mission.")
                seg.target_range_km = shorter_range_km
                cruise_modified = True
                break # Assume only the main cruise segment needs modification

        if not cruise_modified:
            print("Warning: Cruise segment not found in base segments to apply shorter range for ShorterRangeMission.")

        # Call the parent Mission class's __init__ with the modified segments
        super().__init__(aircraft=aircraft, mission_segments=modified_segments)
        self.shorter_range_km = shorter_range_km # Store for reference


def calc_hover_energy_k_W_h(hover_electric_power_k_W,equiv_hover_dur_s):
    return hover_electric_power_k_W * equiv_hover_dur_s * H_PER_S

def calc_climb_time_s(altitude_diff_m, rate_of_climb_mps): 
    return altitude_diff_m / rate_of_climb_mps

def calc_power_at_speed_climbing_k_W(
    mtow_kg, target_speed_mps, base_cd0, wing_AR, wing_ref_area_m2, rho,
    rate_of_climb_mps, prop_effic, epu_effic, g, spac_effic_factor, trim_drag_factor
    ):

    lift_needed = mtow_kg * g
    q = 0.5 * rho * target_speed_mps**2
    cl_req = lift_needed / (q * wing_ref_area_m2)

    cdi = cl_req**2 / (math.pi * spac_effic_factor * wing_AR)

    cd_total_untrimed = base_cd0 + cdi
    cd_total = cd_total_untrimed * trim_drag_factor

    drag_n = q * wing_ref_area_m2 * cd_total

    power_drag_kw = drag_n * target_speed_mps / 1000.0
    power_vertical_kw = ((((mtow_kg* g) * rate_of_climb_mps)) / 1000.0)  # MTOW-lift

    total_shaft_power_req_kw = (power_drag_kw + power_vertical_kw) / prop_effic

    electric_power_req_kw = total_shaft_power_req_kw / epu_effic

    return electric_power_req_kw

def calc_reserve_hover_energy_k_W_h(hover_electric_power_k_W,reserve_hover_dur_s):
    return reserve_hover_dur_s * hover_electric_power_k_W * H_PER_S

def calc_reserve_cruise_energy_k_W_h(cruise_speed_m_p_s,reserve_range_km,cruise_electric_power_k_W):
    reserve_cruise_time_s = reserve_range_km * 1000.0 / cruise_speed_m_p_s
    return cruise_electric_power_k_W * reserve_cruise_time_s * H_PER_S

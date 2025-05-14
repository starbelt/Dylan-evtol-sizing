import json
import math
from tabulate import tabulate
import abc
from battery import Battery
from typing import Dict, Any 
from base_component import AircraftComponent
from battery import Battery
from typing import Optional


# Constants 
G = 9.81
H_PER_S = 1.0 / 3600.0
W_PER_KW = 1000.0

# wing class
class Wing(AircraftComponent):
    def  __init__(self, path_to_json: str):
        super().__init__(path_to_json)
        self.wingspan_m = 0.0
        self.wing_ref_area_m2 = 0.0
        self.wing_AR = 0.0
        self.wing_loading_kg_p_m2 = 0.0
        self.wing_loading_english = 0.0
        self.wing_Cd0 = 0.0
        self.wing_root_chord_m = 0.0
        self.wing_MAC = 0.0
        self.wing_tip_chord_m = 0.0
        self.wing_cruise_reynolds = 0.0
        self.wing_stall_reynolds = 0.0

    def UpdateComponent(self, aircraft_state: dict):
        self.load_variables_from_json()
        mtow_kg = aircraft_state.get('mtow_kg')
        d_value_m = self._varlist["d_value_m"]
        g_m_p_s2 = self._varlist["g_m_p_s2"]
        air_density_sea_level_kg_p_m3 = self._varlist["air_density_sea_level_kg_p_m3"]
        stall_speed_m_p_s = self._varlist["stall_speed_m_p_s"]
        vehicle_cl_max = self._varlist["vehicle_cl_max"]
        wing_airfoil_cd_at_cruise_cl = self._varlist["wing_airfoil_cd_at_cruise_cl"]
        wing_taper_ratio = self._varlist["wing_taper_ratio"]
        kinematic_visc_sea_level_m2_p_s = self._varlist["kinematic_visc_sea_level_m2_p_s"]
        wing_t_p_c = self._varlist["wing_t_p_c"]
        cruise_speed_m_p_s = self._varlist["cruise_speed_m_p_s"]
        self.wingspan_m = self.calc_wingspan_m(d_value_m)
        self.wing_ref_area_m2 = self.calc_wing_ref_area_m2(mtow_kg, g_m_p_s2, air_density_sea_level_kg_p_m3, stall_speed_m_p_s, vehicle_cl_max)
        self.wing_AR = self.calc_wing_AR(self.wingspan_m, self.wing_ref_area_m2)
        self.wing_loading_kg_p_m2 = self.calc_wing_loading_kg_p_m2(mtow_kg, self.wing_ref_area_m2)
        self.wing_loading_english = self.calc_wing_loading_english(self.wing_loading_kg_p_m2)
        self.wing_Cd0 = self.calc_wing_Cd0(wing_airfoil_cd_at_cruise_cl)
        self.wing_root_chord_m = self.calc_wing_root_chord_m(self.wingspan_m, self.wing_AR, wing_taper_ratio)
        self.wing_MAC = self.calc_wing_MAC(self.wing_root_chord_m, wing_taper_ratio)
        self.wing_tip_chord_m = self.calc_wing_tip_chord_m(self.wing_root_chord_m, wing_taper_ratio)
        self.wing_cruise_reynolds = self.calc_wing_cruise_reynolds(self.wing_MAC, cruise_speed_m_p_s, kinematic_visc_sea_level_m2_p_s)
        self.wing_stall_reynolds = self.calc_wing_stall_reynolds(stall_speed_m_p_s, self.wing_MAC, kinematic_visc_sea_level_m2_p_s)
        self.weight = self.calc_wing_weight_kg(mtow_kg, self.wing_ref_area_m2, self.wing_AR, wing_taper_ratio, wing_t_p_c)

    def calc_wingspan_m(self, d_value_m):
        return d_value_m
    
    def calc_wing_ref_area_m2(self, mtow_kg, g_m_p_s2, rho_sl, vs, cl_max):
         denom = 0.5 * rho_sl * vs**2 * cl_max
         return (mtow_kg * g_m_p_s2) / denom 
    
    def calc_wing_AR(self, span, area):
        return span**2 / area
    
    def calc_wing_loading_kg_p_m2(self, mtow, area):
        return mtow / area 
    
    def calc_wing_loading_english(self, wl_si):
        return wl_si * 0.204816
    
    def calc_wing_Cd0(self, cd_airfoil):
        return cd_airfoil
    
    def calc_wing_MAC(self, c_root, taper):
        return (2/3)*c_root*(1+taper**2/(1+taper))
    
    def calc_wing_root_chord_m(self, span, ar, taper):
        return (2*span)/(ar*(1+taper))
    
    def calc_wing_tip_chord_m(self, c_root, taper):
        return c_root * taper
    
    def calc_wing_cruise_reynolds(self, mac, v, nu):
        return (mac*v)/nu 
    
    def calc_wing_stall_reynolds(self, vs, mac, nu):
        return (vs*mac)/nu 
    
    def calc_wing_weight_kg(self, mtow, area, ar, taper, tc):  
        term1 = (mtow * 2.2046 / 1000) ** 0.847
        term3 = (area * 3.2808 ** 2) ** 0.21754
        term4 = ar ** 0.50016
        term5_base = (1 + taper) / tc
        if term5_base <= 0: return 0.0
        term5 = term5_base ** 0.09359
        weight = 5.66411 * term1 * ((3.8 * 1.5) ** 0.39579) * term3 * term4 * term5 * 0.9 * 0.4536
        return weight

# fuselage class
class Fuselage(AircraftComponent):
    def __init__(self, path_to_json: str):
        super().__init__(path_to_json)
        self.fuselage_fineness_ratio = 0.0
        self.fuselage_reynolds = 0.0
        self.fuselage_Cd0_p_Cf = 0.0
        self.fuselage_Cf = 0.0
        self.fuselage_reference_area_m2 = 0.0
        self.fuselage_wetted_area_m2 = 0.0
        self.fuselage_CdA = 0.0
        self.fuselage_Cd0 = 0.0

    def UpdateComponent(self, aircraft_state: dict):
        self.load_variables_from_json()
        mtow_kg = aircraft_state.get('mtow_kg')
        wing_ref_area_m2 = aircraft_state.get('wing_ref_area_m2')
        fuselage_w_m=self._varlist["fuselage_w_m"]
        fuselage_l_m=self._varlist["fuselage_l_m"]
        fuselage_h_m=self._varlist["fuselage_h_m"]
        kinematic_visc_sea_level_m2_p_s=self._varlist["kinematic_visc_sea_level_m2_p_s"]
        cruise_speed_m_p_s=self._varlist["cruise_speed_m_p_s"]
        air_density_sea_level_kg_p_m3=self._varlist["air_density_sea_level_kg_p_m3"]
        self.fuselage_fineness_ratio = self.calc_fuselage_fineness_ratio(fuselage_w_m, fuselage_l_m, fuselage_h_m)
        self.fuselage_reynolds = self.calc_fuselage_reynolds(kinematic_visc_sea_level_m2_p_s, cruise_speed_m_p_s, fuselage_l_m)
        self.fuselage_Cd0_p_Cf = self.calc_fuselage_Cd0_p_Cf(self.fuselage_fineness_ratio)
        self.fuselage_Cf = self.calc_fuselage_Cf(self.fuselage_reynolds, self.fuselage_fineness_ratio)
        self.fuselage_reference_area_m2 = self.calc_fuselage_reference_area_m2(fuselage_w_m, fuselage_h_m)
        self.fuselage_wetted_area_m2 = self.calc_fuselage_wetted_area_m2(self.fuselage_fineness_ratio, self.fuselage_reference_area_m2)
        self.fuselage_CdA = self.calc_fuselage_CdA(self.fuselage_Cd0_p_Cf, self.fuselage_Cf, self.fuselage_reference_area_m2)
        self.fuselage_Cd0 = self.calc_fuselage_Cd0(self.fuselage_CdA, wing_ref_area_m2)
        self.weight = self.calc_fuselage_weight_kg(self.fuselage_wetted_area_m2, mtow_kg, fuselage_l_m, self.fuselage_fineness_ratio, air_density_sea_level_kg_p_m3, cruise_speed_m_p_s)
  
    def calc_fuselage_fineness_ratio(self, w, l, h):
        return (l * 2) / (w + h) 
    
    def calc_fuselage_reynolds(self, nu, v, l):
        return (l * v) / nu
    
    def calc_fuselage_Cd0_p_Cf(self, fr):
        t1=3*fr
        t2=4.5/math.sqrt(fr)
        t3=21/(fr**2)
        return t1+t2+t3
    
    def calc_fuselage_Cf(self, re, fr):
        log_re = math.log10(re)
        term1=0.455/(log_re**2.58)
        term2=0.0016*fr/(re**0.4)
        return term1+term2
    
    def calc_fuselage_wetted_area_m2(self, fr, ref_a):
        return 3 * fr * ref_a
    
    def calc_fuselage_reference_area_m2(self, w, h):
        return math.pi * ((w + h) / 4)**2 
    
    def calc_fuselage_CdA(self, cd0cf, cf, ref_a):
        return cd0cf * cf * ref_a
    
    def calc_fuselage_Cd0(self, cda, wing_a):
        return cda / wing_a
    
    def calc_fuselage_weight_kg(self, swet, mtow, l, fr, rho, v):
        term_L_base = l * 0.5 * 3.2808
        term_q_base = 0.5 * rho * v ** 2 * 0.0209
        weight = (0.052 * (swet*3.2808**2)**1.086 * (3.8*1.5*mtow*2.2046)**0.177 * (term_L_base)**-0.051 * (fr)**-0.072 * (term_q_base)**0.241 * 0.4536*0.9)
        return weight

# horziontal tail class
class HorizontalTail(AircraftComponent):
    def __init__(self, path_to_json: str):
        super().__init__(path_to_json)
        self.horizontal_tail_area_m2 = 0.0
        self.horizontal_tail_Cd0 = 0.0

    def UpdateComponent(self, aircraft_state: dict):
        self.load_variables_from_json()
        wing_MAC = aircraft_state.get('wing_MAC')
        wing_ref_area_m2 = aircraft_state.get('wing_ref_area_m2')
        fuselage_l_m=self._varlist["fuselage_l_m"]
        horiz_tail_vol_coeff=self._varlist["horiz_tail_vol_coeff"]
        empennage_airfoil_cd0=self._varlist["empennage_airfoil_cd0"]
        cruise_speed_m_p_s=self._varlist["cruise_speed_m_p_s"]
        dive_speed_m_p_s = self.calc_dive_speed_m_p_s(cruise_speed_m_p_s)
        self.horizontal_tail_area_m2 = self.calc_horizontal_tail_area_m2(fuselage_l_m, wing_MAC, wing_ref_area_m2, horiz_tail_vol_coeff)
        self.horizontal_tail_Cd0 = self.calc_horizontal_tail_Cd0(self.horizontal_tail_area_m2, wing_ref_area_m2, empennage_airfoil_cd0)
        self.weight = self.calculate_horizontal_tail_weight_kg(self.horizontal_tail_area_m2, dive_speed_m_p_s)

    def calc_horizontal_tail_Cd0(self,h_area,w_area,cd0):
        return h_area/w_area*cd0 
    
    def calc_horizontal_tail_area_m2(self,fuselage_l_m,wing_MAC,wing_ref_area_m2,vol_c):
         Lh=fuselage_l_m*0.5
         return vol_c*wing_ref_area_m2*wing_MAC/Lh 
    
    def calc_dive_speed_m_p_s(self,v_c):
        return v_c*1.4

    def calculate_horizontal_tail_weight_kg(self,h_area,v_d):
        a_ft2 = h_area * 3.2808**2
        v_kts = v_d * 1.9438
        term_base = 0.00395*(a_ft2**0.2)*v_kts
        term = term_base - 0.4885
        w = a_ft2 * term * 0.4536 * 0.9
        return w

# vertical tail class
class VerticalTail(AircraftComponent):
    def __init__(self, path_to_json: str):
        super().__init__(path_to_json)
        self.vertical_tail_area_m2 = 0.0
        self.vertical_tail_Cd0 = 0.0

    def UpdateComponent(self, aircraft_state: dict):
        self.load_variables_from_json()
        wingspan_m = aircraft_state.get('wingspan_m')
        wing_ref_area_m2 = aircraft_state.get('wing_ref_area_m2')
        wing_ref_area_m2 = aircraft_state.get('wing_ref_area_m2')
        fuselage_l_m=self._varlist["fuselage_l_m"]
        vert_tail_vol_coeff=self._varlist["vert_tail_vol_coeff"]
        empennage_airfoil_cd0=self._varlist["empennage_airfoil_cd0"]
        cruise_speed_m_p_s=self._varlist["cruise_speed_m_p_s"]
        dive_speed_m_p_s = self.calc_dive_speed_m_p_s(cruise_speed_m_p_s)
        self.vertical_tail_area_m2 = self.calc_vertical_tail_area_m2(fuselage_l_m, wing_ref_area_m2, vert_tail_vol_coeff, wingspan_m)
        self.weight = self.calc_vertical_wing_weight_kg(self.vertical_tail_area_m2, dive_speed_m_p_s)
        self.vertical_tail_Cd0 = self.calc_vertical_tail_Cd0(self.vertical_tail_area_m2, wing_ref_area_m2, empennage_airfoil_cd0)


    def calc_dive_speed_m_p_s(self,v_c):
        return v_c*1.4

    def calc_vertical_tail_area_m2(self,fuselage_l_m,w_area,vol_c,w_span):
         Lv=fuselage_l_m*0.5
         return vol_c*w_span*w_area/Lv
    
    def calc_vertical_wing_weight_kg(self,v_area,v_d):
        a_ft2 = v_area * 3.2808**2
        v_kts = v_d * 1.9438
        term_base = 0.00395*(a_ft2**0.2)*v_kts
        term = term_base - 0.4885
        w = a_ft2 * term * 0.4536 * 0.9
        return w
    
    def calc_vertical_tail_Cd0(self,v_area,w_area,cd0):
        return v_area/w_area*cd0 

# Landing gear class
class LandingGear(AircraftComponent):
    def __init__(self, path_to_json: str):
        super().__init__(path_to_json)
        self.landing_gear_Cd0 = 0.0

    def UpdateComponent(self, aircraft_state: dict):
        self.load_variables_from_json()
        mtow_kg = aircraft_state.get('mtow_kg')
        wing_ref_area_m2 = aircraft_state.get('wing_ref_area_m2')
        wing_ref_area_m2 = aircraft_state.get('wing_ref_area_m2')
        landing_gear_drag_area_m2=self._varlist["landing_gear_drag_area_m2"]
        self.landing_gear_Cd0 = self.calc_landing_gear_Cd0(landing_gear_drag_area_m2, wing_ref_area_m2)
        self.weight = self.calc_landing_gear_weight_kg(mtow_kg)

    def calc_landing_gear_Cd0(self,lg_da,w_area):
        return lg_da/w_area
    
    def calc_landing_gear_weight_kg(self,mtow):
        return 0.0325*mtow*1.14*1.08

# Boom class
class Boom(AircraftComponent):
    def __init__(self, path_to_json: str):
        super().__init__(path_to_json)
        self.boom_disk_area_m2 = 0.0
        self.booms_Cd0 = 0.0
        self.booms_CdA_m2 = 0.0

    def UpdateComponent(self, aircraft_state: dict):
        self.load_variables_from_json()
        wing_ref_area_m2 = aircraft_state.get('wing_ref_area_m2')
        wing_MAC = aircraft_state.get('wing_MAC')
        battery_spec_energy = aircraft_state.get('battery_spec_energy')
        single_EPU_weight_kg = aircraft_state.get('single_epu_weight_kg')
        boom_drag_area=self._varlist["boom_drag_area"]
        rotor_count=self._varlist["rotor_count"]
        rotor_diameter_m=self._varlist["rotor_diameter_m"]
        self.boom_disk_area_m2 = self.calc_disk_area_m2(rotor_diameter_m)
        self.booms_Cd0 = self.calc_booms_Cd0(self.boom_disk_area_m2, boom_drag_area, wing_ref_area_m2)
        self.total_booms_Cd0 = self.booms_Cd0 * rotor_count
        self.booms_CdA_m2 = self.calc_booms_CdA_m2(self.booms_Cd0, battery_spec_energy)
        self.weight = self.calc_boom_weight_kg(single_EPU_weight_kg, rotor_diameter_m, wing_MAC)
        print(f"    - {type(self).__name__}: Calculated Weight = {self.weight:.2f} kg")

    def calc_booms_Cd0(self,d_area,b_area,w_area):
        return d_area/b_area/w_area*2/3 
    
    def calc_booms_CdA_m2(self,b_cd0,b_spec_e):
        return b_cd0*b_spec_e
    
    def calc_boom_weight_kg(self,epu_w,r_d,w_mac):
        w = (0.0412 * (epu_w * 2.2046) ** 1.1433 * 1 ** 1.3762 * 0.4536 +
          6 * 0.2315 * ((1.2 * r_d+ w_mac) * 1) ** 1.3476) * 2
        return w/2

    def calc_disk_area_m2(self,r_d):
        return math.pi*(r_d/2)**2

# Lift Rotor class
class LiftRotor(AircraftComponent):
    def __init__(self, path_to_json: str):
        super().__init__(path_to_json)
        self.disk_area_m2 = 0.0
        self.rotor_RPM_hover = 0.0
        self.Ct_hover = 0.0
        self.lift_rotor_plus_hub_weight_kg = 0.0
        self.disk_loading_kg_p_m2 = 0.0

    def UpdateComponent(self, aircraft_state: dict):
        self.load_variables_from_json()
        single_EPU_weight_kg = aircraft_state.get('single_epu_weight_kg')
        mtow_kg = aircraft_state.get('mtow_kg')
        total_rotor_count = aircraft_state.get('rotor_count')
        rotor_count=self._varlist.get("lift_rotor_count")
        rotor_diameter_m=self._varlist["rotor_diameter_m"]
        sound_speed_m_p_s=self._varlist["sound_speed_m_p_s"]
        tip_mach=self._varlist["tip_mach"]
        air_density_sea_level_kg_p_m3=self._varlist["air_density_sea_level_kg_p_m3"]
        air_density_min_kg_p_m3=self._varlist["air_density_min_kg_p_m3"]
        rotor_avg_cl=self._varlist["rotor_avg_cl"]
        g_m_p_s2=self._varlist["g_m_p_s2"]
        total_rotor_count=self._varlist["rotor_count"]
        self.disk_area_m2 = self.calc_disk_area_m2(rotor_diameter_m)
        self.rotor_RPM_hover = self.calc_rotor_RPM_hover(sound_speed_m_p_s, rotor_diameter_m, tip_mach)
        self.Ct_hover = self.calc_Ct_hover(mtow_kg, g_m_p_s2, total_rotor_count, air_density_sea_level_kg_p_m3, rotor_diameter_m, self.rotor_RPM_hover)
        rotor_solidity = self.calc_rotor_solidity(self.Ct_hover, rotor_avg_cl)
        over_torque_factor = self.calc_over_torque_factor(total_rotor_count)
        self.lift_rotor_plus_hub_weight_kg = self.calc_lift_rotor_plus_hub_weight_kg(total_rotor_count, rotor_diameter_m, rotor_solidity, sound_speed_m_p_s, tip_mach, air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3, over_torque_factor)
        self.disk_loading_kg_p_m2 = self.calc_disk_loading_kg_p_m2(mtow_kg, math.pi*(rotor_diameter_m/2)**2)
        self.weight = (single_EPU_weight_kg  + self.lift_rotor_plus_hub_weight_kg/rotor_count)
    
        print(f"    - {type(self).__name__}: Calculated Weight = {self.weight:.2f} kg (EPU Comp: {single_EPU_weight_kg:.2f}, Hub Comp: {self.lift_rotor_plus_hub_weight_kg/rotor_count:.2f})")

    def calc_lift_rotor_plus_hub_weight_kg(self,r_count,r_d,r_s,sos,m_tip,rho_sl,rho_min,ot_f):
        t1b=r_d/2*3.2808
        t2b=math.pi*r_d**2/4*r_s/r_d*3.2808
        t3b=sos*m_tip*math.sqrt(rho_sl/rho_min)*math.sqrt(ot_f)*3.2808
        w = (0.0024419*1*r_count/2*2**0.53479*(t1b)**1.74231*(t2b)**0.77291*(t3b)**0.87562*1.1**2.51048 + 0.00037547*1**1.02958*r_count/2*2**0.71443*(t1b)**1.99321*(t2b)**0.79577*(t3b)**0.96323*1.1**0.46203*1.1**2.58473)*0.4536
        return w

    def calc_over_torque_factor(self,rotor_count):
        return rotor_count/(rotor_count-2.0)+0.3 
    
    def calc_rotor_RPM_hover(self,sos,r_d,m_tip):
        return sos*m_tip/(r_d/2)*30/math.pi 
    
    def calc_Ct_hover(self,mtow,g,r_c,rho,r_d,rpm):
         denom = rho*math.pi*(r_d/2)**4*(rpm*math.pi/30)**2
         return (mtow*g/r_c)/denom 
    
    def calc_disk_area_m2(self,r_d):
        return math.pi*(r_d/2)**2
    
    def calc_disk_loading_kg_p_m2(self,mtow,d_area):
        return mtow/d_area
    
    def calc_disk_loading_english(self,dl_si):
        return dl_si*0.204816
    
    def calc_rotor_solidity(self,ct,cl):
        return ct*6/cl

# Tilt rotor class
class TiltRotor(AircraftComponent):
    def __init__(self, path_to_json: str):
        super().__init__(path_to_json)
        self.disk_area_m2 = 0.0
        self.rotor_RPM_hover = 0.0
        self.Ct_hover = 0.0
        self.tilt_rotor_weight_kg = 0.0
        self.disk_loading_kg_p_m2 = 0.0

    def UpdateComponent(self, aircraft_state: dict):
        self.load_variables_from_json()
        mtow_kg = aircraft_state.get('mtow_kg')
        single_EPU_weight_kg = aircraft_state.get('single_epu_weight_kg')
        total_rotor_count = aircraft_state.get('rotor_count')
        rotor_count=self._varlist.get("tilt_rotor_count")
        rotor_diameter_m=self._varlist["rotor_diameter_m"]
        sound_speed_m_p_s=self._varlist["sound_speed_m_p_s"]
        tip_mach=self._varlist["tip_mach"]
        air_density_sea_level_kg_p_m3=self._varlist["air_density_sea_level_kg_p_m3"]
        air_density_min_kg_p_m3=self._varlist["air_density_min_kg_p_m3"]
        rotor_avg_cl=self._varlist["rotor_avg_cl"]
        g_m_p_s2=self._varlist["g_m_p_s2"]
        total_rotor_count=self._varlist["rotor_count"]
        self.disk_area_m2 = self.calc_disk_area_m2(rotor_diameter_m)
        self.rotor_RPM_hover = self.calc_rotor_RPM_hover(sound_speed_m_p_s, rotor_diameter_m, tip_mach)
        self.Ct_hover = self.calc_Ct_hover(mtow_kg, g_m_p_s2, total_rotor_count, air_density_sea_level_kg_p_m3, rotor_diameter_m, self.rotor_RPM_hover)
        rotor_solidity = self.calc_rotor_solidity(self.Ct_hover, rotor_avg_cl)
        over_torque_factor = self.calc_over_torque_factor(total_rotor_count)
        self.tilt_rotor_weight_kg = self.calc_tilt_rotor_weight_kg(total_rotor_count, rotor_diameter_m, rotor_solidity, sound_speed_m_p_s, tip_mach, air_density_sea_level_kg_p_m3, air_density_min_kg_p_m3, over_torque_factor)
        self.disk_loading_kg_p_m2 = self.calc_disk_loading_kg_p_m2(mtow_kg, math.pi*(rotor_diameter_m/2)**2)
        self.weight = (single_EPU_weight_kg + self.tilt_rotor_weight_kg/rotor_count)
        print(f"    - {type(self).__name__}: Calculated Weight = {self.weight:.2f} kg (EPU Comp: {single_EPU_weight_kg:.2f}, Hub Comp: {self.tilt_rotor_weight_kg:.2f})")

    def calc_tilt_rotor_weight_kg(self,r_count,r_d,r_s,sos,m_tip,rho_sl,rho_min,ot_f):
        t1b=r_d/2*3.2808
        t2b=math.pi*r_d*r_s/2/3*3.2808
        t3b=sos*m_tip*math.sqrt(rho_sl/rho_min)*math.sqrt(ot_f)*3.2808
        w = (0.0024419*1.1794*r_count/2*3**0.53479*(t1b)**1.74231*(t2b)**0.77291*(t3b)**0.87562*1.1**2.51048 + 0.00037547*1.1794**1.02958*r_count/2*3**0.71443*(t1b)**1.99321*(t2b)**0.79577*(t3b)**0.96323*1.1**0.46203*1.1**2.58473)*0.4536
        return w

    def calc_over_torque_factor(self,rotor_count):
        return rotor_count/(rotor_count-2.0)+0.3
    
    def calc_rotor_RPM_hover(self,sos,r_d,m_tip):
        return sos*m_tip/(r_d/2)*30/math.pi 
    
    def calc_Ct_hover(self,mtow,g,r_c,rho,r_d,rpm):
         denom = rho*math.pi*(r_d/2)**4*(rpm*math.pi/30)**2
         return (mtow*g/r_c)/denom 
    
    def calc_disk_area_m2(self,r_d):
        return math.pi*(r_d/2)**2 

    def calc_disk_loading_kg_p_m2(self,mtow,d_area):
        return mtow/d_area 

    def calc_disk_loading_english(self,dl_si):
        return dl_si*0.204816

    def calc_rotor_solidity(self,ct,cl):
        return ct*6/cl

class Aircraft:
    def __init__(self, components: list, path_to_json: str):
        for component in components:
            if isinstance(component, Battery):
                self.battery = component
                break

        self.components = components
        self.path_to_json = path_to_json
        self._varlist: Dict[str, Any] = {}
        self.load_aircraft_variables_from_json()
        self.mtow_kg = 0.0
        self.base_cd0 = 0.0 
        self.Cd0_total_parasite = 0.0 
        self.Cd_cruise = 0.0
        self.CL_cruise = 0.0
        self.Cdi_cruise = 0.0
        self.cruise_L_p_D = 0.0
        self.rotor_disk_area_total_m2 = 0.0
        self.rotor_RPM_hover = 0.0
        self._varlist['single_epu_weight_kg_sized'] = 0.0


    def load_aircraft_variables_from_json(self):
        with open(self.path_to_json, "r") as f: self._varlist = json.load(f)
        self.spac_effic_factor = self._varlist["spac_effic_factor"]
        self.trim_drag_factor = self._varlist["trim_drag_factor"]
        self.excres_protub_factor = self._varlist["excres_protub_factor"]
        self.total_payload_kg = self._varlist["total_payload_kg"]
        self.fixed_systems_kg = self._varlist["fixed_systems_kg"]
        self.g_m_p_s2 = self._varlist.get("g_m_p_s2", G)
        self.max_dod = self._varlist["max_dod"]


    def update_state_for_iteration(self, current_mtow_estimate):
        self.mtow_kg = current_mtow_estimate
        aircraft_state: Dict[str, Any] = {'mtow_kg': current_mtow_estimate}
        component_Cd0_sum = 0.0
        wing_props = {}
        single_epu_weight_kg = 0.0

        is_initial_sizing = self._varlist['single_epu_weight_kg_sized'] == 0.0

        if is_initial_sizing:
            rotor_diameter_m = self._varlist["rotor_diameter_m"]
            sound_speed_m_p_s = self._varlist["sound_speed_m_p_s"]
            tip_mach = self._varlist["tip_mach"]
            total_rotor_count = self._varlist["rotor_count"] # Uses the total for intial sizing
            rho_sl = self._varlist["rho_sl"]
            rho_min = self._varlist["air_density_min_kg_p_m3"]
            fom = self._varlist["fom"]

            self.rotor_RPM_hover = self.calc_rotor_RPM_hover(sound_speed_m_p_s, rotor_diameter_m, tip_mach)
            hover_omega_rad_s = self.rotor_RPM_hover * math.pi / 30.0
            initial_rotor_disk_area = total_rotor_count * math.pi * (rotor_diameter_m / 2)**2
            total_hover_shaft_power_kw = calc_hover_shaft_power_k_W(self.mtow_kg, self.g_m_p_s2, rho_sl, fom, initial_rotor_disk_area)
            hover_power_per_rotor_kw = total_hover_shaft_power_kw / total_rotor_count
            hover_torque_per_rotor_nm = (hover_power_per_rotor_kw * W_PER_KW) / hover_omega_rad_s

            over_torque_factor = total_rotor_count / (total_rotor_count - 2.0) + 0.3

            max_torque_per_rotor_nm = over_torque_factor * hover_torque_per_rotor_nm
            max_rpm = self.rotor_RPM_hover * math.sqrt(over_torque_factor) * math.sqrt(rho_sl / rho_min)
            max_power_per_rotor_kw = (max_torque_per_rotor_nm * max_rpm * math.pi / 30.0 / W_PER_KW)

            single_epu_weight_kg = calc_single_EPU_weight_kg(max_torque_per_rotor_nm, max_power_per_rotor_kw)
            self._varlist['single_epu_weight_kg_sized'] = single_epu_weight_kg 
            print(f"  Calculated EPU Weight (Sizing): {single_epu_weight_kg:.2f} kg (P={max_power_per_rotor_kw:.2f} kW, T={max_torque_per_rotor_nm:.1f} Nm)")
        else:
             single_epu_weight_kg = self._varlist['single_epu_weight_kg_sized']
             print(f"  Using Stored EPU Weight: {single_epu_weight_kg:.2f} kg")


        aircraft_state['single_epu_weight_kg'] = single_epu_weight_kg

        # Updates wing first
        wing_component = next((c for c in self.components if isinstance(c, Wing)), None)
        if wing_component:
            wing_component.UpdateComponent(aircraft_state)
            wing_props = {'wing_ref_area_m2': wing_component.wing_ref_area_m2,
                          'wing_AR': wing_component.wing_AR,
                          'wing_MAC': wing_component.wing_MAC,
                          'wingspan_m': wing_component.wingspan_m}
            aircraft_state.update(wing_props)
            cd0_val = getattr(wing_component, 'wing_Cd0', 0.0)
            # Includes only if weight > 0, implicitly handles the jettisoned rotors and booms cd0
            if wing_component.get_weight_kg() > 0:
                 component_Cd0_sum += cd0_val


        # Update components and sum drag
        spec_e = self.battery.get_gross_BOL_spec_energy()
        aircraft_state['battery_spec_energy'] = spec_e # Needed for boom weight calc

        # Calculate current total disk area from non jettisoned rotors
        current_total_disk_area = 0.0
        for component in self.components:
             if isinstance(component, (LiftRotor, TiltRotor)):
                # Update the component first before getting its area
                if not isinstance(component, (Battery, Wing)):
                    component.UpdateComponent(aircraft_state)
                # Add its disk area
                current_total_disk_area += getattr(component, 'disk_area_m2')
             elif not isinstance(component, (Battery, Wing)):
                 # Update other non rotor/wing/battery components
                 component.UpdateComponent(aircraft_state)


        for component in self.components:
            # Implicityly handles the jettisoned rotors and booms cd0 because weight has to be > 0
            if component.get_weight_kg() > 0 and not isinstance(component, (Battery, Wing)):
                 # Gets the Cd0 value for every component and adds them all together for the total cd0
                 cd0_attr = next((attr for attr in ['fuselage_Cd0', 'horizontal_tail_Cd0', 'vertical_tail_Cd0', 'landing_gear_Cd0', 'booms_Cd0'] if hasattr(component, attr)), None)
                 if cd0_attr:
                    cd0_val = getattr(component, cd0_attr, 0.0)
                    component_Cd0_sum += cd0_val


        # After updating all components, print each component's contribution to Cd0
        # Used for debugging 
        print("\n=== Component Drag Coefficient Breakdown ===")
        wing_component = next((c for c in self.components if isinstance(c, Wing)), None)
        if wing_component and wing_component.get_weight_kg() > 0:
            print(f"Wing: wing_Cd0 = {wing_component.wing_Cd0:.6f}")
        for component in self.components:
            if component.get_weight_kg() > 0 and not isinstance(component, (Battery, Wing)):
                for cd0_attr in ['fuselage_Cd0', 'horizontal_tail_Cd0', 'vertical_tail_Cd0', 'landing_gear_Cd0', 'booms_Cd0']:
                    if hasattr(component, cd0_attr):
                        print(f"{type(component).__name__}: {cd0_attr} = {getattr(component, cd0_attr):.6f}")

        print(f"Total component_Cd0_sum = {component_Cd0_sum:.6f}")
        print(f"After excres_protub_factor ({self.excres_protub_factor}): {component_Cd0_sum * self.excres_protub_factor:.6f}")
        print("============================================\n")

        self.base_cd0 = component_Cd0_sum
        self.Cd0_total_parasite = self.base_cd0 * self.excres_protub_factor

        # Calculate cruise performance
        cruise_speed = self._varlist.get('cruise_speed_m_p_s')
        rho_cruise = self._varlist.get('air_density_cruise_kg_p_m3')
        rho_sl = self._varlist.get('air_density_sea_level_kg_p_m3')
        area = wing_props.get('wing_ref_area_m2')
        ar = wing_props.get('wing_AR')

        self.CL_cruise = self.calculate_cruise_CL(self.mtow_kg * self.g_m_p_s2, rho_cruise, cruise_speed, area)
        print(f"Cruise Lift Coefficient (CL_cruise): {self.CL_cruise:.4f}")

        self.Cdi_cruise = self.calc_Cdi(self.spac_effic_factor, ar, self.CL_cruise)
        self.Cd_cruise = (self.Cd0_total_parasite + self.Cdi_cruise) * self.trim_drag_factor
        self.cruise_L_p_D = self.CL_cruise / self.Cd_cruise

        self.rotor_disk_area_total_m2 = current_total_disk_area
        print(f"  Aircraft state updated. Current MTOW={self.mtow_kg:.2f} kg, Cruise L/D={self.cruise_L_p_D:.2f}, "
              f"Cd0={self.Cd0_total_parasite:.5f}, Current Disk Area={self.rotor_disk_area_total_m2:.2f} m^2")
    
    
    def CalculateMTOW(self):
        total_mass = sum(c.get_weight_kg() for c in self.components)
        total_mass += self.total_payload_kg
        total_mass += self.fixed_systems_kg
        return total_mass
    
    # Gets the current usable EOL capacity
    def get_current_usable_EOL_kwh(self):
        return self.battery.get_usable_EOL_capacity_kwh()

    # Gets the current gross BOL capacity
    def get_current_gross_BOL_kwh(self):
        return self.battery.get_gross_BOL_capacity_kwh()

    def calc_rotor_RPM_hover(self,sos,r_d,m_tip):
        return sos*m_tip/(r_d/2)*30/math.pi 

    def calc_Cdi(self, span_effic, ar, cl):
        denom = math.pi * span_effic * ar
        cl_sq = cl**2
        return cl_sq / denom

    def calculate_cruise_CL(self, weight_n, rho, v, area):
        denom = 0.5 * rho * v**2 * area
        return weight_n / denom 
    
    def get_rotor_disk_area_m2(self):
        return self.rotor_disk_area_total_m2
    
    def get_wing_area_m2(self):
        wing = next((c for c in self.components if isinstance(c, Wing)), None)
        return wing.wing_ref_area_m2 
    
    def get_wing_ar(self):
         wing = next((c for c in self.components if isinstance(c, Wing)), None)
         return wing.wing_AR 

def calc_hover_shaft_power_k_W(mtow_kg, g, rho, fom, disk_area_m2):
    thrust_n = mtow_kg * g
    ideal_power_w = (thrust_n**1.5) / (math.sqrt(2.0 * rho * disk_area_m2))
    shaft_power_w = ideal_power_w / fom
    return shaft_power_w / W_PER_KW

def calc_single_EPU_weight_kg(motor_torque_max_thrust_Nm, motor_mechanical_power_sizing_k_W):
    power_term = motor_mechanical_power_sizing_k_W
    torque_term = motor_torque_max_thrust_Nm
    weight_kg = 1.15 * (power_term / 12.67 + torque_term / 52.2 + 2.55)
    return weight_kg
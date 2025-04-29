import numpy as np
import LTA_Classes as LTA

from spatialnde2 import quatpy

# This craft has a lift fan under the center and reversible fans on the left and right
# The LTA craft has been modeled in SolidWorks and coordinates specific to the craft have been extracted and used below. 
g = 9.81 # [m/s^2]
air_density = 1.225 # [kg/m^3]
helium_density = 0.1785 # [kg/m^3]

# -----------------------------------------------------------------------------------------------------
# Configuration Parameters: 
# -----------------------------------------------------------------------------------------------------
mass_craft = 1.3 # [kg]
mass_balloon = 0.1 # [kg], mass of balloon rubber, does not include ballast
ballast_mass = 0.1 # [kg]
balloon_height_over_craft = 0.7 # [m]
balloon_radius = 0.66 # [m]
k_string = 100 # [N/m]
balloon_CD = 0.3
balloon_ref_area = np.pi*balloon_radius**2 # [m^2]
craft_CD = 1.1
craft_ref_area = 0.8*0.8 # [m^2]

# Coordinates from physical model of the aircraft:
r_cm_bodycoords_craft = np.array([0.1399, -0.00862, 0.03788, 1]) # [m]
r_cm_bodycoords_balloon = np.array([0, 0, 0, 1]) # [m]
Icm_cmcoords_craft = np.diag(np.array([0.01, 0.01, 0.01])) # [kg-m^2], Taken from design or estimate from radius of gyration, 3x3 tensor, its the I about the center of mass but in terms of the body axes [kg-m^2]
Icm_cmcoords_balloon = np.diag(np.array([0.0069, 0.0068, 0.0069])) # [kg-m^2]

# initial_craft_position = np.array([30, 12, -27]) # sets aircraft on platform in atrium model ([19.2, -14.5, 9.8])
initial_craft_position = np.array([19.2, -14.5, 9.8])  # [m]
initial_craft_rotation = np.array([1,0,0,0]) # quaternion with real part first
initial_balloon_rotation = np.array([1,0,0,0])

# Position of the string attachment points in body coordinates on the craft:
r_attachment_craftbodycoords1 = np.array([0.06636, 0.04349, 0.08596, 1])  # [m]
r_attachment_craftbodycoords2 = np.array([0.06636, -0.05842, 0.08596, 1])  # [m]
r_attachment_craftbodycoords3 = np.array([0.21042, -0.05842, 0.08596, 1])  # [m]
r_attachment_craftbodycoords4 = np.array([0.21042, 0.04349, 0.08596, 1])  # [m]

# Position of the knot in body coordinates on the balloon:
r_knot_bodycoords = np.array([0,0,-0.3,1])  # [m]

string1 = LTA.LTAString(k_string, nominal_length=None, r_attachment_craftbodycoords=r_attachment_craftbodycoords1)  # [m]
string2 = LTA.LTAString(k_string, nominal_length=None, r_attachment_craftbodycoords=r_attachment_craftbodycoords2)  # [m]
string3 = LTA.LTAString(k_string, nominal_length=None, r_attachment_craftbodycoords=r_attachment_craftbodycoords3)  # [m]
string4 = LTA.LTAString(k_string, nominal_length=None, r_attachment_craftbodycoords=r_attachment_craftbodycoords4)  # [m]
strings = [string1,string2,string3,string4]

landing_gear1_craftbodycoords = np.array([0.23,0.01,-0.05,1])  # [m]
landing_gear2_craftbodycoords = np.array([0.05,0.01,-0.05,1])  # [m]
landing_gears = [landing_gear1_craftbodycoords,landing_gear2_craftbodycoords]  # [m]

# -----------------------------------------------------------------------------------------------------
# Thruster Configurations: 
# -----------------------------------------------------------------------------------------------------
# For control_values, (liftfan, leftfan, rightfan)
# Thruster coordinates are X Right, Y forward, Z UP
# control_values = np.array([1.0, 1.5, 1.5],dtype='d') # [milliseconds], 1.0 milliseconds represents full to the left or throttle = 0, 2.0 milliseconds represents full to the right or max throttle.

# For the radiomaster controller, the values are output as [leftfan,rightfan,liftfan]. Therefore, the following has been adjusted for that:
# Lift Fan, has non-reversing ESCs
liftfan_newtons_per_millisecond = 3 # Maximum throttle (2 milliseconds - 1 millisecond) will give us 5 Newtons
liftfan_newtonmeters_per_millisecond = 0.001 # [N-m/ms]
# All need to be in the y forward in thruster coordinates
F_thrustercoords_function_liftfan = lambda control_values: (0, (control_values[2] - 1.0)*liftfan_newtons_per_millisecond, 0, 0)
M_thrustercoords_function_liftfan = lambda control_values: (0, (control_values[2] - 1.0)*liftfan_newtonmeters_per_millisecond, 0, 0)

# Left Fan, have reversing ESCs
leftfan_newtons_per_millisecond = 3 # Maximum throttle (2 milliseconds - 1.5 millisecond) will give us 5 Newtons
leftfan_newtonmeters_per_millisecond = 0.001 # [N-m/ms]
F_thrustercoords_function_leftfan = lambda control_values: (0, (control_values[0] - 1.5)*leftfan_newtons_per_millisecond, 0, 0)
M_thrustercoords_function_leftfan = lambda control_values: (0, (control_values[0] - 1.5)*leftfan_newtonmeters_per_millisecond, 0, 0)

# Right Fan, have reversing ESCs
rightfan_newtons_per_millisecond = 3 # Maximum throttle (2 milliseconds - 1.5 millisecond) will give us 5 Newtons
rightfan_newtonmeters_per_millisecond = 0.001 # [N-m/ms]
F_thrustercoords_function_rightfan = lambda control_values: (0, (control_values[1] - 1.5)*rightfan_newtons_per_millisecond, 0, 0)
M_thrustercoords_function_rightfan = lambda control_values: (0, (control_values[1] - 1.5)*rightfan_newtonmeters_per_millisecond, 0, 0)

thruster_liftfan_initial = np.array([1/np.sqrt(2), 1/np.sqrt(2),0, 0])
thruster_liftfan = LTA.LTAThruster(lambda control_values: LTA.RelativePose(thruster_liftfan_initial, (0.1399, -0.00862, 0.04, 0)), # pose_bodycoords_over_thrustercoords
                                F_thrustercoords_function_liftfan,M_thrustercoords_function_liftfan) # number of thrusters is dependent on the design
thruster_leftfan_initial = np.array([1, 0, 0, 0])
thruster_leftfan = LTA.LTAThruster(lambda control_values: LTA.RelativePose(thruster_leftfan_initial, (-0.01,0,0.04, 0)), # pose_bodycoords_over_thrustercoords, located 30 cm from the origin
                                F_thrustercoords_function_leftfan,M_thrustercoords_function_leftfan) # number of thrusters is dependent on the design
thruster_rightfan_initial = np.array([1, 0, 0, 0])
thruster_rightfan = LTA.LTAThruster(lambda control_values: LTA.RelativePose(thruster_rightfan_initial, (0.2898,0,0.04, 0)), # pose_bodycoords_over_thrustercoords,  located 30 cm from the origin
                                F_thrustercoords_function_rightfan,M_thrustercoords_function_rightfan) # number of thrusters is dependent on the design
thrusters = [thruster_liftfan,thruster_leftfan,thruster_rightfan]

# -----------------------------------------------------------------------------------------------------
# Parameter Initializations: 
# -----------------------------------------------------------------------------------------------------
pose_worldcoords_over_bodycoords_craft = LTA.RelativePose(initial_craft_rotation, initial_craft_position)
r_cm_worldcoords_craft = quatpy.quaternion_apply_vector(pose_worldcoords_over_bodycoords_craft.quat, r_cm_bodycoords_craft[:3]) + pose_worldcoords_over_bodycoords_craft.offset[:3]  # [m]
initial_balloon_position = np.array([r_cm_worldcoords_craft[0],r_cm_worldcoords_craft[1], r_cm_worldcoords_craft[2] + balloon_height_over_craft]) # [m], put the knot at the origin of the balloon frame

pose_worldcoords_over_bodycoords_balloon = LTA.RelativePose(initial_balloon_rotation, initial_balloon_position)
r_cb_bodycoords_balloon = balloon_radius # [m]
balloon_volume = 4/3*np.pi*balloon_radius**3 # [m^3]
air_weight = air_density*balloon_volume*g # [N]
helium_weight = helium_density*balloon_volume*g # [N]
F_buoyancy_worldcoords = np.array([0,0,air_weight - helium_weight,0]) # [N]

initial_velocity_cm_worldcoords_balloon = np.zeros(4,dtype=np.float64) # [m/s]
initial_angular_momentum_worldcoords_balloon = np.array([0,0,0,0],dtype=np.float64) # [kg-m^2/s]
initial_velocity_cm_worldcoords_craft = np.zeros(4,dtype=np.float64) # [m/s]
initial_angular_momentum_worldcoords_craft = np.array([0,0,0,0],dtype=np.float64) # [kg-m^2/s]
initial_angular_velocity_cmbodcoords_craft = np.array([0,0,0],dtype=np.float64) # [rad/s]
initial_angular_velocity_cmbodcoords_balloon = np.array([0,0,0],dtype=np.float64) # [rad/s]

# -----------------------------------------------------------------------------------------------------
# Rigid Body Initializations: 
# -----------------------------------------------------------------------------------------------------
balloon = LTA.LTABalloon(mass_balloon,balloon_CD,balloon_ref_area,r_cm_bodycoords_balloon,Icm_cmcoords_balloon,pose_worldcoords_over_bodycoords_balloon,    
                         r_knot_bodycoords,r_cb_bodycoords_balloon,F_buoyancy_worldcoords,initial_velocity_cm_worldcoords_balloon,initial_angular_momentum_worldcoords_balloon,initial_angular_velocity_cmbodcoords_balloon,ballast_mass,balloon_radius)

craft = LTA.LTACraft(mass_craft,craft_CD,craft_ref_area,r_cm_bodycoords_craft,Icm_cmcoords_craft,pose_worldcoords_over_bodycoords_craft,
                     initial_velocity_cm_worldcoords_craft,initial_angular_momentum_worldcoords_craft,initial_angular_velocity_cmbodcoords_craft,thrusters,strings,landing_gears)

# Determine the nominal length (design length) of each string  
for s in strings:
    s.evaluate_nominal_string_length(balloon, craft)
    pass
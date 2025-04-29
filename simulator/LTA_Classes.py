import numpy as np

from spatialnde2 import quatpy 
import pdb

class RelativePose():
    quat = None
    offset = None  
    
    def __str__(self):
        offset_str = f"Offset: {str(self.offset):s}"
        quat_str = f"Quaternion: {str(self.quat):s}"
        return f"RelativePose({quat_str:s}, {offset_str:s})"
    
    def __repr__(self):
        return self.__str__()
    
    def __init__(self, quat, offset):
        self.quat = np.array(quat, dtype=np.float64)
        self.offset = np.array(offset, dtype=np.float64)  # Store offset
        self.quat = quatpy.quaternion_normalize(quat)  # Normalize 
        pass
    pass

    def build_transmtx(self): # builds 4x4 homogenous coordinates transformation (rotation + offset matrix)
        rotmtx = self.quaternion_build_rotmtx(self.quat)
        rotmtx[3, :3] = self.offset[:3]
        return rotmtx
    
    def apply(self, position_or_vector):
        position_or_vector = np.array(position_or_vector)
        vec = position_or_vector[:3]
        w = position_or_vector[3]
        
        # Apply quaternion rotation to the vector part
        rotated_vec = quatpy.quaternion_apply_vector(self.quat, vec)
        
        # If w = 1, it's a position
        if np.isclose(w, 1):
            rotated_pos = rotated_vec + self.offset[:3]
            return np.array([rotated_pos[0], rotated_pos[1], rotated_pos[2], 1.0])  
        
        # If w = 0, it's a vector 
        if np.isclose(w,0):
            return np.array([rotated_vec[0], rotated_vec[1], rotated_vec[2], 0.0])  
        raise ValueError("Fourth element of position or vector is not zero or one.")
    
    def relativepose_multiply(self, relative_pose): 
        ''' 
        This function takes two relative poses and evaluates the product of the two. 
        '''
        # Take the product of the two quaternions
        product_quat = quatpy.quaternion_product_normalized(self.quat, relative_pose.quat)
        
        # Apply the left quaternion (self.quat) to the right offset 
        rotated_right_offset = quatpy.quaternion_apply_vector(self.quat, relative_pose.offset[:3])
        
        # Add the rotated right offset to the left offset 
        product_offset = self.offset[:3] + rotated_right_offset
        
        # Return the new relative pose as a RelativePose object
        product_array = np.array([product_quat[0], product_quat[1], product_quat[2], product_quat[3]])
        return RelativePose(product_array, product_offset)
    
    def relativepose_inv(self):
        '''
        This function computes the inverse of the relative pose.
        '''
        # Inverse of the quaternion
        inv_quat = quatpy.quaternion_normalize(quatpy.quaternion_inverse(self.quat))
        
        inv_offset = -quatpy.quaternion_apply_vector(inv_quat, self.offset[:3])
        
        return RelativePose(inv_quat, inv_offset)
    
class RigidBody:
    """ 
    Coordinate Frame Labels: 
        bodycoords: defined in terms of the design coordinate frame of the body, relative to an arbitrary origin
        cmcoords: defined with the origin aligned to the center of mass. Axes otherwise aligned to bodycoords. 
        worldcoords: World coordinate frame. Presumed to be inertial.
    """
    mass = None # [kg]
    CD = None # Drag force coefficient
    ref_area = None # reference area for the drag coefficient
    r_cm_bodycoords = None # [m], position of the center of mass in terms of body coordinates
    Icm_cmcoords = None # [kg-m^2], moment of inertia tensor with respect to the center of mass, probably diagonal 3x3 matrix, represented in cmcoords
    pose_worldcoords_over_bodycoords = None # A RelativePose object
    velocity_cm_worldcoords = None # [m/s], velocity of the center of mass in world coordinates
    angular_momentum_worldcoords = None # [kg-m^2/s], angular momentum relative to the center of mass in world coordinates
    angular_velocity_cmbodycoords = None 
    
    # RigidBody default constructor 
    def __init__(self,mass,CD,ref_area,r_cm_bodycoords,Icm_cmcoords,pose_worldcoords_over_bodycoords,velocity_cm_worldcoords,angular_momentum_worldcoords,angular_velocity_cmbodycoords):
        self.mass = mass 
        self.CD = CD
        self.ref_area = ref_area
        self.r_cm_bodycoords = r_cm_bodycoords 
        self.Icm_cmcoords = Icm_cmcoords
        self.pose_worldcoords_over_bodycoords = pose_worldcoords_over_bodycoords
        self.velocity_cm_worldcoords = velocity_cm_worldcoords
        self.angular_momentum_worldcoords = angular_momentum_worldcoords
        self.angular_velocity_cmbodycoords = angular_velocity_cmbodycoords
        pass
    
    def calculate_angular_velocity(self):
        # Angular momentum, H = I * ω
        # This returns angular velocity of the rigid body about its center of mass in world coordinates
        Icm_worldcoords = quatpy.quaternion_apply_to_bothsides_of_matrix(RigidBody.pose_worldcoords_over_bodycoords.quat, RigidBody.Icm_cmcoords)
        return np.dot(self.Icm_worldcoords.inv(), self.angular_momentum_worldcoords)
    
    @staticmethod
    def calculate_sphere_inertia(mass, radius):
    # Moment of inertia for a solid sphere about its central axis
        I = (2/5)*mass*radius**2
        array =  np.array([[I, 0, 0],
                        [0, I, 0],
                        [0, 0, I]])  # Uniform moment of inertia for all axes
        return array
    @staticmethod
    def calculate_box_inertia(mass, width, height, depth):
        # Moments of inertia for a rectangular prism (box)
        I_x = (1/12)*mass*(height**2 + depth**2)
        I_y = (1/12)*mass*(width**2 + depth**2)
        I_z = (1/12)*mass*(height**2 + width**2)
        return np.array([[I_x, 0, 0],
                        [0, I_y, 0],
                        [0, 0, I_z]])
    
class LTACraft(RigidBody):
    '''
    CRAFT COORDINATE FRAME: (Body)
    x to the right, y forward, z up
    '''
    # These class members inherited from class RigidBody:
    # mass = None # [kg]
    # CD = None # coefficient of drag
    # ref_area = None # reference area for the drag coefficient
    # r_cm_bodycoords = None # [m], position of the center of mass in terms of body coordinates
    # Icm_cmcoords = None # [kg-m^2], moment of inertia tensor with respect to the center of mass, probably diagonal 3x3 matrix, represented in cmcoords
    # pose_worldcoords_over_bodycoords = None # A RelativePose object
    # velocity_cm_worldcoords = None # [m/s], velocity of the center of mass in world coordinates
    # angular_momentum = None # [kg-m^2/s], angular momentum relative to the center of mass in world coordinates
    # angular_velocity_cmbodycoords = None
    thrusters = None # List of class LTAThruster
    strings = None # List of class LTAString
    landing_gears = None # List of landing gears
    
    def __init__(self,mass,CD,ref_area,r_cm_bodycoords,Icm_cmcoords,pose_worldcoords_over_bodycoords,
                 velocity_cm_worldcoords,angular_momentum_worldcoords,angular_velocity_cmbodycoords,thrusters,strings,landing_gears):
        self.mass = mass 
        self.CD = CD
        self.ref_area = ref_area
        self.r_cm_bodycoords = r_cm_bodycoords 
        self.Icm_cmcoords = Icm_cmcoords
        self.pose_worldcoords_over_bodycoords = pose_worldcoords_over_bodycoords
        self.velocity_cm_worldcoords = velocity_cm_worldcoords
        self.angular_momentum_worldcoords = angular_momentum_worldcoords
        self.angular_velocity_cmbodycoords = angular_velocity_cmbodycoords
        
        self.thrusters = thrusters 
        self.strings = strings 
        self.landing_gears = landing_gears
        pass

class LTABalloon(RigidBody):
    # These class members inherited from class RigidBody:
    # mass = None # [kg]
    # CD = None # coefficient of drag
    # ref_area = None # reference area for the drag coefficient
    # r_cm_bodycoords = None # [m], position of the center of mass in terms of body coordinates
    # Icm_cmcoords = None # [kg-m^2], moment of inertia tensor with respect to the center of mass, probably diagonal 3x3 matrix, represented in cmcoords
    # pose_worldcoords_over_bodycoords = None # A RelativePose object
    # velocity_cm_worldcoords = None # [m/s], velocity of the center of mass in world coordinates
    # angular_momentum = None # [kg-m^2/s], angular momentum relative to the center of mass in world coordinates
    # angular_velocity_cmbodycoords = None
    
    ballast_mass = None # [kg], mass of the ballast alone
    balloon_mass = None # [kg], mass of the balloon alone
    r_knot_bodycoords = None # [m] vector
    r_cb_bodycoords = None # center of buoyancy vector [m]
    F_buoyancy_worldcoords = None # numpy 3 vector [N]
    radius = None # [m], radius of the balloon
    
    # LTABalloon default constructor 
    def __init__(self,balloon_mass,CD,ref_area,r_cm_bodycoords,Icm_cmcoords,pose_worldcoords_over_bodycoords,r_knot_bodycoords,
                 r_cb_bodycoords,F_buoyancy_worldcoords,velocity_cm_worldcoords,angular_momentum_worldcoords,angular_velocity_cmbodycoords,ballast_mass,radius):
        self.balloon_mass = balloon_mass 
        self.CD = CD
        self.ref_area = ref_area
        self.r_cm_bodycoords = r_cm_bodycoords 
        self.Icm_cmcoords = Icm_cmcoords
        self.pose_worldcoords_over_bodycoords = pose_worldcoords_over_bodycoords
        self.velocity_cm_worldcoords = velocity_cm_worldcoords
        self.angular_momentum_worldcoords = angular_momentum_worldcoords
        self.angular_velocity_cmbodycoords = angular_velocity_cmbodycoords
        self.ballast_mass = ballast_mass
        self.mass = balloon_mass + ballast_mass
        self.radius = radius
        
        self.r_knot_bodycoords = r_knot_bodycoords 
        self.r_cb_bodycoords = r_cb_bodycoords 
        self.F_buoyancy_worldcoords = F_buoyancy_worldcoords
        pass
    pass

class LTAThruster: 
    '''
    In the thruster coordinate frame, the motor is at the origin.
    
    THRUSTER COORDINATE FRAME: 
        positive thrust would be in the +y (forward) direction, +x to the right, +z up
    '''
    pose_bodycoords_over_thrustercoords_function = None # not a constant value, call the function with the control values as a parameter
    F_thrustercoords_function = None # [N] give the control values, return a vector in thruster coordinates
    M_thrustercoords_function = None # [N-m], given the control values, return a vector in thruster coordinates representing the couple induced by the fan rotation.
    
    # LTAThruster default constructor 
    def __init__(self,pose_bodycoords_over_thrustercoords_function,F_thrustercoords_function, M_thrustercoords_function): 
        self.pose_bodycoords_over_thrustercoords_function = pose_bodycoords_over_thrustercoords_function
        self.F_thrustercoords_function = F_thrustercoords_function
        self.M_thrustercoords_function = M_thrustercoords_function
        pass

class LTAString: 
    k_string = None # [N-m] of extension relative to nominal length
    nominal_length = None # [m]
    r_attachment_craftbodycoords = None # position of attachment in craft body coords [m]
    
    # LTAString default constructor 
    def __init__(self,k_string,nominal_length,r_attachment_craftbodycoords): 
        self.k_string = k_string
        self.nominal_length = nominal_length
        self.r_attachment_craftbodycoords = r_attachment_craftbodycoords
        pass
    def evaluate_nominal_string_length(self, balloon, craft):
        r_knot_balloon_bodycoords = balloon.r_knot_bodycoords
        r_attachment_worldcoords = craft.pose_worldcoords_over_bodycoords.apply(self.r_attachment_craftbodycoords)
        r_knot_worldcoords = balloon.pose_worldcoords_over_bodycoords.apply(r_knot_balloon_bodycoords)
        self.nominal_length = np.linalg.norm(r_attachment_worldcoords - r_knot_worldcoords)
        return self.nominal_length
    pass

# STEPS FOR SIMULATOR: 
# First, need to be able to apply the pose object to a vector or position and use that to transform the vector or position. 
# Begin constructing and asssembling each of the objects. Typically done by giving each class a generic constructor (provide the values as keyword arguments and fill in). 
# Then, initialize the state of the system (i.e initial coordinates).
# Lastly, iterate F=ma and torque=rate of change (H_dot) of angular momentum (H)

def calculate_forces(t, balloon, craft, thrusters, control_values, strings):
    g = 9.81 # [m/s^2]
    air_density = 1.225 # [kg/m^3]

    total_string_force_balloon_worldcoords = np.zeros(4, dtype=np.float64)
    total_string_force_craft_worldcoords = np.zeros(4, dtype=np.float64)
    total_moment_balloon_strings_worldcoords = np.zeros(3, dtype=np.float64) # Moments about the center of mass
    total_moment_craft_strings_worldcoords = np.zeros(3, dtype=np.float64) # Moments about the center of mass
    total_thruster_force_craft_worldcoords = np.zeros(4, dtype=np.float64)
    total_thruster_moments_craft_worldcoords = np.zeros(3, dtype=np.float64)
    
    # Convert angular velocity in CM body coordinates into world coordinates for dashpot calculations
    balloon_bodycoords_over_cmbodycoords = RelativePose(np.array([1,0,0,0]), balloon.r_cm_bodycoords)
    balloon_pose_worldcoords_over_cmbodycoords = balloon.pose_worldcoords_over_bodycoords.relativepose_multiply(balloon_bodycoords_over_cmbodycoords)
    balloon_angular_velocity_worldcoords = balloon_pose_worldcoords_over_cmbodycoords.apply(np.array([balloon.angular_velocity_cmbodycoords[0],balloon.angular_velocity_cmbodycoords[1],balloon.angular_velocity_cmbodycoords[2],0]))
    
    craft_bodycoords_over_cmbodycoords = RelativePose(np.array([1,0,0,0]), craft.r_cm_bodycoords)
    craft_pose_worldcoords_over_cmbodycoords = craft.pose_worldcoords_over_bodycoords.relativepose_multiply(craft_bodycoords_over_cmbodycoords)
    craft_angular_velocity_worldcoords = craft_pose_worldcoords_over_cmbodycoords.apply(np.array([craft.angular_velocity_cmbodycoords[0],craft.angular_velocity_cmbodycoords[1],craft.angular_velocity_cmbodycoords[2],0]))
    # -----------------------------------------------------------------------------------------------------
    # String Calculations: 
    # -----------------------------------------------------------------------------------------------------

    num_strings = len(strings) # number of strings in the model
    for string in strings:
        # Knot location in world coordinates (r_attachment)
        r_attachment_worldcoords = craft.pose_worldcoords_over_bodycoords.apply(string.r_attachment_craftbodycoords)
        r_knot_worldcoords = balloon.pose_worldcoords_over_bodycoords.apply(balloon.r_knot_bodycoords)

        # Calculate the vector from the string attachment to the balloon
        vector_to_balloon_worldcoords = (r_knot_worldcoords - r_attachment_worldcoords) # knot is at the base of the balloon, attachment is located on the craft (below the balloon) ***!!! WORLDCOORDS
        vector_to_craft_worldcoords = (r_attachment_worldcoords - r_knot_worldcoords) # Negated for opposite direction 
        current_length = np.linalg.norm(vector_to_balloon_worldcoords) # length will be the same for both craft and balloon direction
        extension = current_length - string.nominal_length 
        if extension > 0:
            direction_to_balloon_worldcoords = vector_to_balloon_worldcoords/current_length  # Normalize the direction vector
            direction_to_craft_worldcoords = vector_to_craft_worldcoords/current_length 
            string_force_worldcoords_balloon = -string.k_string*extension*(direction_to_balloon_worldcoords)  # Hooke's Law F = -kx
            string_force_worldcoords_craft = -string.k_string*extension*(direction_to_craft_worldcoords)  # Hooke's Law 

            # velocity at a givent point = velocity at the center of mass + ((omege_centerofmass) x (vector from the center of mass to point of interest))
            # Velocity is the difference of the velocity at each end of the string. There is a velocity at the attachment point and at the knot point.
            c_dashpot = 2*np.sqrt(((craft.mass + balloon.mass)/num_strings)*string.k_string) # this calculation is per string, so the mass needs to be divided by the number of strings in the model
            v_attachment_craft_val = craft.velocity_cm_worldcoords[:3] + np.cross(craft_angular_velocity_worldcoords[:3],(r_attachment_worldcoords - craft.pose_worldcoords_over_bodycoords.apply(craft.r_cm_bodycoords))[:3]) # this should be the vector from the center of mass to r_attachmentworldoocrds 
            v_attachment_craft = np.array([v_attachment_craft_val[0],v_attachment_craft_val[1],v_attachment_craft_val[2],0.0])
            v_knot_balloon_val = balloon.velocity_cm_worldcoords[:3] + np.cross(balloon_angular_velocity_worldcoords[:3],(r_knot_worldcoords - balloon.pose_worldcoords_over_bodycoords.apply(balloon.r_cm_bodycoords))[:3])
            v_knot_balloon = np.array([v_knot_balloon_val[0],v_knot_balloon_val[1],v_knot_balloon_val[2],0.0]) #!!!*** attach units
            vel_diff = v_knot_balloon - v_attachment_craft # Scalar projection of relative velocity along the string direction
            # should be positive if things are coming apart vel diff is a vector here when it needs to be a scalar should just be how much the balloon is moving away from the craft on this string
            # find the component of vel diff along the string direction dot product with unit vector of the string
            r_string_hat_worldcoords = vector_to_balloon_worldcoords/np.linalg.norm(vector_to_balloon_worldcoords)
            vel_parallel = np.dot(r_string_hat_worldcoords,vel_diff)
            damping_force_balloon_worldcoords = -c_dashpot*vel_parallel*direction_to_balloon_worldcoords
            damping_force_craft_worldcoords = -c_dashpot*vel_parallel*direction_to_craft_worldcoords
        
            total_string_force_balloon_worldcoords = total_string_force_balloon_worldcoords + string_force_worldcoords_balloon + damping_force_balloon_worldcoords  # Accumulate forces from all strings on balloon
            total_string_force_craft_worldcoords = total_string_force_craft_worldcoords + string_force_worldcoords_craft + damping_force_craft_worldcoords # Accumulate forces from all strings on cra
 
                
            # Calculate moment due to this string force
            r_attachment_worldcoords_relative_to_cm = r_attachment_worldcoords - craft.pose_worldcoords_over_bodycoords.apply(craft.r_cm_bodycoords)
            moment_craft_strings_spring_worldcoords = np.cross(r_attachment_worldcoords_relative_to_cm[:3], string_force_worldcoords_craft[:3]) 
            moment_craft_strings_dashpot_worldcoords = np.cross(r_attachment_worldcoords_relative_to_cm[:3], damping_force_craft_worldcoords[:3])    
            total_moment_craft_strings_worldcoords = total_moment_craft_strings_worldcoords + moment_craft_strings_spring_worldcoords + moment_craft_strings_dashpot_worldcoords
            
            r_knot_worldcoords_relative_to_cm = r_knot_worldcoords - balloon.pose_worldcoords_over_bodycoords.apply(balloon.r_cm_bodycoords)
            moment_balloon_strings_spring_worldcoords = np.cross(r_knot_worldcoords_relative_to_cm[:3], string_force_worldcoords_balloon[:3])
            moment_balloon_strings_dashpot_worldcoords = np.cross(r_knot_worldcoords_relative_to_cm[:3], damping_force_balloon_worldcoords[:3])
            total_moment_balloon_strings_worldcoords = total_moment_balloon_strings_worldcoords + moment_balloon_strings_spring_worldcoords + moment_balloon_strings_dashpot_worldcoords
            pass
        pass

    # -----------------------------------------------------------------------------------------------------
    # Thruster  Calculations: 
    # -----------------------------------------------------------------------------------------------------
    for thruster in thrusters:
        thruster_transform_bodycoords_over_thrustercoords = thruster.pose_bodycoords_over_thrustercoords_function(control_values) 
        thruster_transform_worldcoords_over_thrustercoords  = craft.pose_worldcoords_over_bodycoords.relativepose_multiply(thruster_transform_bodycoords_over_thrustercoords)

        # Force and moment from the thruster in local coordinates
        F_thrustercoords = thruster.F_thrustercoords_function(control_values)
        M_thrustercoords = thruster.M_thrustercoords_function(control_values)
        
        F_thruster_bodycoords = thruster_transform_bodycoords_over_thrustercoords.apply(F_thrustercoords)
        M_thruster_bodycoords = thruster_transform_bodycoords_over_thrustercoords.apply(M_thrustercoords)

        F_thruster_worldcoords = craft.pose_worldcoords_over_bodycoords.apply(F_thruster_bodycoords)
        M_thruster_worldcoords = craft.pose_worldcoords_over_bodycoords.apply(M_thruster_bodycoords)

        # Need to solve for M = rxF
        M_thruster_worldcoords_from_force = np.cross((thruster_transform_worldcoords_over_thrustercoords.offset[:3] - craft.pose_worldcoords_over_bodycoords.apply(craft.r_cm_bodycoords)[:3]), F_thruster_worldcoords[:3])
        
        # Accumulate thruster forces and moments
        total_thruster_force_craft_worldcoords = total_thruster_force_craft_worldcoords + F_thruster_worldcoords
        total_thruster_moments_craft_worldcoords = total_thruster_moments_craft_worldcoords + M_thruster_worldcoords[:3]  + M_thruster_worldcoords_from_force[:3] # Only the first 3 components for moment
        
    # -----------------------------------------------------------------------------------------------------
    # Drag Calculations: 
    # -----------------------------------------------------------------------------------------------------
    v_balloon_relative_to_air_worldcoords = balloon.velocity_cm_worldcoords  # Assuming the air is stationary
    speed_balloon_worldcoords = np.linalg.norm(v_balloon_relative_to_air_worldcoords)
    if speed_balloon_worldcoords > 0: 
        drag_balloon_worldcoords = 0.5*balloon.CD*air_density*balloon.ref_area*speed_balloon_worldcoords**2
        drag_balloon_direction_worldcoords = -v_balloon_relative_to_air_worldcoords/speed_balloon_worldcoords  # Drag acts opposite to the velocity
        drag_force_balloon_worldcoords = drag_balloon_worldcoords*drag_balloon_direction_worldcoords
        T_drag_balloon_worldcoords = np.cross(balloon.pose_worldcoords_over_bodycoords.offset[:3] - balloon.pose_worldcoords_over_bodycoords.apply(balloon.r_cm_bodycoords)[:3], drag_force_balloon_worldcoords[:3])

    else:
        drag_force_balloon_worldcoords = np.array([0,0,0,0])
        T_drag_balloon_worldcoords = np.array([0,0,0])
        pass

    if np.linalg.norm(balloon_angular_velocity_worldcoords) > 0:
        # angular drag 
        lin_vel_mag_balloon_worldcoords = np.linalg.norm(balloon_angular_velocity_worldcoords)*balloon.radius # get a scalar v using the relationship: v = |omega|*r
        direction_vec_angdrag_balloon_worldcoords = -balloon_angular_velocity_worldcoords/np.linalg.norm(balloon_angular_velocity_worldcoords) # (-) to act in opposite direction of angular velocity
        M_ang_drag_balloon_val_worldcoords = 1/2*balloon.CD*air_density*balloon.ref_area*(lin_vel_mag_balloon_worldcoords)**2*balloon.radius*direction_vec_angdrag_balloon_worldcoords
        M_ang_drag_balloon_worldcoords = np.array([M_ang_drag_balloon_val_worldcoords[0],M_ang_drag_balloon_val_worldcoords[1],M_ang_drag_balloon_val_worldcoords[2],0.0])
    else: 
        M_ang_drag_balloon_worldcoords =np.array([0,0,0,0])
    
    v_craft_relative_to_air_worldcoords = craft.velocity_cm_worldcoords  # Assuming the air is stationary
    speed_craft_worldcoords = np.linalg.norm(v_craft_relative_to_air_worldcoords)
    if speed_craft_worldcoords > 0:
        drag_craft_worldcoords = 0.5*craft.CD*air_density*craft.ref_area*speed_craft_worldcoords**2
        drag_craft_direction_worldcoords = -v_craft_relative_to_air_worldcoords/speed_craft_worldcoords  # Drag acts opposite to the velocity
        drag_force_craft_worldcoords = drag_craft_worldcoords*drag_craft_direction_worldcoords
        # Torque is T = r x F
        T_drag_craft_worldcoords = np.cross(craft.pose_worldcoords_over_bodycoords.offset[:3] - craft.pose_worldcoords_over_bodycoords.apply(craft.r_cm_bodycoords)[:3], drag_force_craft_worldcoords[:3])
        pass
    else: 
        drag_force_craft_worldcoords = np.array([0,0,0,0])
        T_drag_craft_worldcoords = np.array([0,0,0])
        pass

    if np.linalg.norm(craft_angular_velocity_worldcoords) > 0:
        craft_radius = np.sqrt(craft.ref_area)/2
        lin_vel_mag_craft_worldcoords = np.linalg.norm(craft_angular_velocity_worldcoords)*craft_radius # get a scalar v using the relationship: v = |omega|*r
        direction_vec_angdrag_craft_worldcoords = -craft_angular_velocity_worldcoords/np.linalg.norm(craft_angular_velocity_worldcoords) # (-) to act in opposite direction of angular velocity
        M_ang_drag_craft_val_worldcoords = 1/2*craft.CD*air_density*craft.ref_area*(lin_vel_mag_craft_worldcoords)**2*(craft_radius)*direction_vec_angdrag_craft_worldcoords
        M_ang_drag_craft_worldcoords = np.array([M_ang_drag_craft_val_worldcoords[0],M_ang_drag_craft_val_worldcoords[1],M_ang_drag_craft_val_worldcoords[2],0.0])
    else: 
        M_ang_drag_craft_worldcoords = np.array([0,0,0,0])
    # -----------------------------------------------------------------------------------------------------
    # Gravity/Buoynacy Calculations: 
    # -----------------------------------------------------------------------------------------------------
    gravity_balloon_worldcoords = np.array([0, 0, -balloon.mass*g, 0])  # 4D vector
    gravity_craft_worldcoords = np.array([0, 0, -craft.mass*g, 0])  # 4D vector
    
    F_buoyancy_worldcoords = balloon.F_buoyancy_worldcoords  # Transform buoyancy force into world coordinates

    # -----------------------------------------------------------------------------------------------------
    # Floor/Normal Force Calculations: 
    # -----------------------------------------------------------------------------------------------------
    # We want the force of the floor to act like a bed of springs:
    floor_height = 9.75
    n_gears = len(craft.landing_gears)
    k_floor = 100 # [N/m]
    c_floor =  2*np.sqrt((craft.mass + balloon.mass)/n_gears*k_floor) # [N/(m-s)]; c^2 = 4mk for critically damped system

    landing_gears_worldcoords = []
    for gear in craft.landing_gears:
        landing_gear_worldcoords = craft.pose_worldcoords_over_bodycoords.apply(gear)
        landing_gears_worldcoords.append(landing_gear_worldcoords)
        pass

    # Need to iterate through list now
    F_floor_worldcoords = np.zeros(4,dtype=np.float64)
    M_floor_worldcoords = np.zeros(3,dtype=np.float64)
    r_landing_gears_world_list = []
    
    for gear_worldcoords in landing_gears_worldcoords:
        floor_penetration = floor_height - gear_worldcoords[2]
        r_landing_gear_worldcoords = gear_worldcoords - craft.pose_worldcoords_over_bodycoords.apply(craft.r_cm_bodycoords)

        r_landing_gears_world_list.append(r_landing_gear_worldcoords[:3])
        if floor_penetration > 1e-3:
            displacement = floor_height - gear_worldcoords[2]
            r_gear_rel_worldcoords = gear_worldcoords - craft.pose_worldcoords_over_bodycoords.apply(craft.r_cm_bodycoords)
            v_gear_worldcoords = craft.velocity_cm_worldcoords[:3] + np.cross(craft_angular_velocity_worldcoords[:3], r_gear_rel_worldcoords[:3])
            damp_floor_worldcoords = -c_floor*v_gear_worldcoords[2] 

            spring_floor_worldcoords = k_floor*displacement 
            normal_force_mag_worldcoords = damp_floor_worldcoords + spring_floor_worldcoords
            normal_force_worldcoords = np.array([0,0,normal_force_mag_worldcoords,0])
            
            F_floor_worldcoords = F_floor_worldcoords + normal_force_worldcoords

            # Find the moments:
            M_landing_gear_worldcoords = np.cross(r_landing_gear_worldcoords[:3],normal_force_worldcoords[:3])

            M_floor_worldcoords = M_floor_worldcoords + M_landing_gear_worldcoords
            
            pass
        else:
            normal_force_worldcoords = np.array([0,0,0,0])

        pass
    
    r_landing_gear_avg_worldcoords = np.mean(np.array(r_landing_gears_world_list), axis=0)
    # -----------------------------------------------------------------------------------------------------
    # Sum Forces and Moments: 
    # -----------------------------------------------------------------------------------------------------
    F_balloon_world = (total_string_force_balloon_worldcoords + gravity_balloon_worldcoords + F_buoyancy_worldcoords + drag_force_balloon_worldcoords)
    M_balloon_world = (total_moment_balloon_strings_worldcoords[:3] + T_drag_balloon_worldcoords + M_ang_drag_balloon_worldcoords[:3]) # Accumulate moments
    
    F_craft_world = (total_string_force_craft_worldcoords + total_thruster_force_craft_worldcoords + gravity_craft_worldcoords + drag_force_craft_worldcoords + F_floor_worldcoords)
    M_craft_world  = (total_moment_craft_strings_worldcoords[:3] + total_thruster_moments_craft_worldcoords + T_drag_craft_worldcoords + M_ang_drag_craft_worldcoords[:3] + M_floor_worldcoords)

    return F_balloon_world, F_craft_world, normal_force_worldcoords, r_landing_gear_avg_worldcoords, M_balloon_world, M_craft_world

def update(craft, balloon, dt, t, strings, thrusters, control_values):
    (F_balloon_world, F_craft_world, F_craft_normal_world, r_landing_gear_average, moment_balloon_world, moment_craft_world) = calculate_forces(t,balloon, craft, thrusters, control_values, strings)
    # -----------------------------------------------------------------------------------------------------
    # Acceleration Calculations: 
    # -----------------------------------------------------------------------------------------------------
    # F = ma, to find acceleration --> a = F/m (LINEAR MOTION)
    a_balloon_worldcoords = F_balloon_world/balloon.mass
    a_craft_worldcoords = F_craft_world/craft.mass
    
    # -----------------------------------------------------------------------------------------------------
    # Velocity Calculations: 
    # -----------------------------------------------------------------------------------------------------
    balloon.velocity_cm_worldcoords = balloon.velocity_cm_worldcoords + a_balloon_worldcoords*dt
    craft_old_velocity_worldcoords = craft.velocity_cm_worldcoords 
    craft_velocity_cm_worldcoords_no_friction = craft.velocity_cm_worldcoords + a_craft_worldcoords*dt
    
    # Leave friction out in the original acceleration calculation. Then, look at the final velocity, and if it has an in-plane component
    # and there is a normal force, then calculate a maximum friction force. Use the maximum friction force to determine an updated final 
    # velocity, and if that final velocity flips direction, then figure out a reduced friction force. 
    
    # -----------------------------------------------------------------------------------------------------
    # Friction Force Calculations: 
    # -----------------------------------------------------------------------------------------------------
    mu_floor = 0.3
    horizontal_vel_worldcoords = np.array([craft_velocity_cm_worldcoords_no_friction[0],craft_velocity_cm_worldcoords_no_friction[1],0,0])
    horizontal_speed_worldcoords = np.linalg.norm(horizontal_vel_worldcoords)
    F_craft_friction_worldcoords = np.zeros(4,dtype=np.float64)
    if horizontal_speed_worldcoords > 1e-9 and F_craft_normal_world[2] > 0:
        direction_friction_worldcoords = -horizontal_vel_worldcoords/horizontal_speed_worldcoords
        F_craft_friction_worldcoords = mu_floor*F_craft_normal_world[2]*direction_friction_worldcoords
        pass

    craft_velocity_cm_worldcoords_max_friction = craft.velocity_cm_worldcoords + (a_craft_worldcoords + (F_craft_friction_worldcoords/craft.mass))*dt
    
    # check to see if we have static friction rather than dynamic friction between the craft and the ground
    # If the F_craft_friction flips the direction of the velocity in the horizontal plane, then we have to reduce it to the static value
    if np.inner(craft_velocity_cm_worldcoords_no_friction, craft_velocity_cm_worldcoords_max_friction) < 0:
        # Now, we have to reduce F_craft_friction to the value that makes the inner product 0. 
        # Evaluate the projection of craft.velocity_cm_worldcoords in the direction of craft_old_velocity_direction_horizontal
        # proj_u(v) where is u is the direction_friction and v is craft.velocity_cm_worldcoords
        # proj__u(v) = (v dot u/|u|^2)u
        # proj_u_of_v = (np.inner(craft.velocity_cm_worldcoords,direction_friction)/np.linalg.norm(direction_friction)**2)*direction_friction
        
        # proj_u_of_v is the velocity component that we have to zero out by selecting an updated F_craft_friction
        # craft.velocity_cm_worldcoords = craft_old_velocity_cm_worldcoords + a_craft*dt
        
        
        # Substitute in a_craft: craft.velocity_cm_worldcoords = craft_old_velocity_cm_worldcoords + ((F_craft_world + F_craft_friction)/craft.mass)*dt
        # Take the dot product of previous line with direction_friction and set equal to zero to solve for F_craft_friction
        # (craft_old_velocity_cm_worldcoords + ((F_craft_world + F_craft_friction)/craft.mass)*dt) dot direction_friction = 0
        # (craft_old_velocity_cm_worldcoords dot direction_friction) + ((((F_craft_world dot direction_friction)/craft.mass)*dt +  (F_craft_friction dot direction_friction)/craft.mass)*dt = 0
        # F_craft_friction is in the opposite direction of direction_friction  and can be treated as a scalar
        # (craft_old_velocity_cm_worldcoords dot direction_friction) + ((((F_craft_world dot direction_friction)/craft.mass)*dt + (-F_craft_friction)/craft.mass)*dt = 0
        # F_craft_friction = (craft_old_velocity_cm_worldcoords dot direction_friction)*craft.mass/dt + ((((F_craft_world dot direction_friction))
        F_craft_friction_magnitude_worldcoords = (np.inner(craft_old_velocity_worldcoords,direction_friction_worldcoords)*craft.mass/dt) + np.inner(F_craft_world, direction_friction_worldcoords)
        F_craft_friction_worldcoords = -F_craft_friction_magnitude_worldcoords*direction_friction_worldcoords
        # Recalculate the update: 
        a_craft_worldcoords = (F_craft_world + F_craft_friction_worldcoords)/craft.mass
        pass
    
    craft.velocity_cm_worldcoords = craft_old_velocity_worldcoords + a_craft_worldcoords*dt
    
    # Moment due to friction:
    M_craft_friction_worldcoords = np.cross(r_landing_gear_average[:3],F_craft_friction_worldcoords[:3])
    
    moment_craft_world = moment_craft_world + M_craft_friction_worldcoords
    
    # -----------------------------------------------------------------------------------------------------
    # Linear Position Calculations: 
    # -----------------------------------------------------------------------------------------------------
    balloon_bodycoords_over_cmbodycoords = RelativePose(np.array([1,0,0,0]), balloon.r_cm_bodycoords)
    balloon_cmbodycoords_over_bodycoords = balloon_bodycoords_over_cmbodycoords.relativepose_inv()
    balloon_pose_worldcoords_over_cmbodycoords = balloon.pose_worldcoords_over_bodycoords.relativepose_multiply(balloon_bodycoords_over_cmbodycoords) 
    balloon_pose_worldcoords_over_cmbodycoords.offset = balloon_pose_worldcoords_over_cmbodycoords.offset + balloon.velocity_cm_worldcoords[:3]*dt 
 
    craft_bodycoords_over_cmbodycoords = RelativePose(np.array([1,0,0,0]), craft.r_cm_bodycoords)
    craft_cmbodycoords_over_bodycoords = craft_bodycoords_over_cmbodycoords.relativepose_inv()
    craft_pose_worldcoords_over_cmbodycoords = craft.pose_worldcoords_over_bodycoords.relativepose_multiply(craft_bodycoords_over_cmbodycoords)
    craft_pose_worldcoords_over_cmbodycoords.offset = craft_pose_worldcoords_over_cmbodycoords.offset + craft.velocity_cm_worldcoords[:3]*dt
    
    # -----------------------------------------------------------------------------------------------------
    # Angular Position/Rotation Calculations: 
    # -----------------------------------------------------------------------------------------------------
    # Angular Momentums
    # Order matters in matrix operations:
    # H = I*w --> I^-1*H = I^-1*I*w --> I^-1*H = w 
    balloon_cmbodycoords_over_worldcoords = balloon_pose_worldcoords_over_cmbodycoords.relativepose_inv()
    balloon_angular_momentum_cmbodycoords = balloon_cmbodycoords_over_worldcoords.apply(np.array([balloon.angular_momentum_worldcoords[0],balloon.angular_momentum_worldcoords[1],balloon.angular_momentum_worldcoords[2],0.0]))

    craft_cmbodycoords_over_worldcoords = craft_pose_worldcoords_over_cmbodycoords.relativepose_inv()
    craft_angular_momentum_cmbodycoords = craft_cmbodycoords_over_worldcoords.apply(np.array([craft.angular_momentum_worldcoords[0],craft.angular_momentum_worldcoords[1],craft.angular_momentum_worldcoords[2],0.0]))
    
    # I_balloon_worldcoords = quatpy.quaternion_apply_to_bothsides_of_matrix(balloon.pose_worldcoords_over_bodycoords.quat, balloon.Icm_cmcoords)
    balloon.angular_velocity_cmbodycoords = np.linalg.inv(balloon.Icm_cmcoords).dot(balloon_angular_momentum_cmbodycoords[:3])
    
    # I_craft_worldcoords = quatpy.quaternion_apply_to_bothsides_of_matrix(craft.pose_worldcoords_over_bodycoords.quat, craft.Icm_cmcoords)
    craft.angular_velocity_cmbodycoords = np.linalg.inv(craft.Icm_cmcoords).dot(craft_angular_momentum_cmbodycoords[:3])
    
    # Sum of moments is the change in angular momentum. Find new angular momentum:
    balloon.angular_momentum_worldcoords = balloon.angular_momentum_worldcoords[:3] + moment_balloon_world*dt # avoid the += operator because of appending values in iteration
    craft.angular_momentum_worldcoords = craft.angular_momentum_worldcoords[:3] + moment_craft_world*dt
    
    # Take unit vector of the rotation and scale it by sin (theta/2) and this becomes imag part of quat real part is cos(theta/2) (theta is the magnitude of omega times dt)
    # Multiply the old quat by the quat just calculated to get the new quat (update)
    theta_balloon = np.linalg.norm(balloon.angular_velocity_cmbodycoords)*dt
    if theta_balloon != 0:
        axis_balloon = balloon.angular_velocity_cmbodycoords/np.linalg.norm(balloon.angular_velocity_cmbodycoords)
        q_rotation_balloon = np.array([np.cos(theta_balloon/2), *(axis_balloon*np.sin(theta_balloon/2))])
        balloon_pose_worldcoords_over_cmbodycoords.quat = quatpy.quaternion_product_normalized(balloon_pose_worldcoords_over_cmbodycoords.quat,q_rotation_balloon) 
        pass

    # Quaternion update for craft
    theta_craft = np.linalg.norm(craft.angular_velocity_cmbodycoords)*dt
    if theta_craft != 0:
        axis_craft = craft.angular_velocity_cmbodycoords/np.linalg.norm(craft.angular_velocity_cmbodycoords)
        q_rotation_craft = np.array([np.cos(theta_craft/2), *(axis_craft*np.sin(theta_craft/2))])
        craft_pose_worldcoords_over_cmbodycoords.quat = quatpy.quaternion_product_normalized(craft_pose_worldcoords_over_cmbodycoords.quat,q_rotation_craft)
        pass
    
    # Transformation to world coordinates over body coordinates:
    balloon.pose_worldcoords_over_bodycoords = balloon_pose_worldcoords_over_cmbodycoords.relativepose_multiply(balloon_cmbodycoords_over_bodycoords)
    craft.pose_worldcoords_over_bodycoords = craft_pose_worldcoords_over_cmbodycoords.relativepose_multiply(craft_cmbodycoords_over_bodycoords)
    
    return F_balloon_world, F_craft_world, moment_balloon_world, moment_craft_world
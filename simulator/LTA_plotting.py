import matplotlib.pyplot as plt
import spatialnde2 as snde
import numpy as np
from LTA_config import craft, balloon, strings, thrusters
from spatialnde2 import quatpy

def dynamic_plotting(time=None, craftvals=None, balloonvals=None, change_in_momentum=None,type=None, index=None, plot_type=None, labels=None, colors=None):
    '''
    This function is for plotting 2D and 3D plots of the system
    
    Args: 
        craftvals: values relative to the craft
        balloonvals: values relative to the balloon
        index: index corresponding to the data that is being plotted 
            X = 0, Y = 1, Z = 2
        plot_type: "2d" or "3d" based on desired plot
        labels = "balloon" and/or "craft"
        colors = colors for plotting
    
    Returns: 
        Creates a plot for either the 2D or 3D values of the aircraft. 
        Function allows user to customize plot labels, colors, and naming. 
        
    ''' 
    title_size = 15
    label_size = 13 
    label_pad = 20
        
    if plot_type == "2d":
        fig = plt.figure()
        
        if type != ("momentum" and "change_momentum"):
            plt.plot(time, craftvals[:,index], label=labels[0] if labels else "Craft", color=colors[0] if colors else "red")        
            plt.plot(time, balloonvals[:,index], label=labels[1] if labels else "Balloon", color=colors[1]if colors else "blue")
            pass
        elif type == "momentum":
            system_momentum = (craftvals + balloonvals)
            plt.plot(time, craftvals[:,index], label=labels[0] if labels else "Craft", color=colors[0] if colors else "red")        
            plt.plot(time, balloonvals[:,index], label=labels[1] if labels else "Balloon", color=colors[1]if colors else "blue")
            plt.plot(time, system_momentum[:,index], label=labels[1] if labels else "System", color=colors[2]if colors else "green")
            pass
        elif type == "change_momentum":
            plt.plot(time, change_in_momentum[:,index], label=labels[0] if labels else "System", color=colors[0] if colors else "red")        
            pass
        
        # Correlate the index to its specific direction based on the data:
        chosen_index = {0:"X", 1:"Y", 2:"Z"}

        if type == "position":
            plt.title(f"2D {chosen_index.get(index)} Position",fontsize=title_size)
            plt.xlabel(f"Time [s]",fontsize=label_size,labelpad=label_pad)
            plt.ylabel(f"Position {chosen_index.get(index)} [m]",fontsize=label_size,labelpad=label_pad)
            pass
        elif type == "velocity":
            plt.title(f"2D {chosen_index.get(index)} Velocity",fontsize=title_size)
            plt.xlabel(f"Time [s]",fontsize=label_size,labelpad=label_pad)
            plt.ylabel(f"Velocity {chosen_index.get(index)} [m/s]",fontsize=label_size,labelpad=label_pad)
            pass
        elif type == "momentum":
            plt.title(f"2D {chosen_index.get(index)} Velocity",fontsize=title_size)
            plt.xlabel(f"Time [s]",fontsize=label_size,labelpad=label_pad)
            plt.ylabel(f"Momentum {chosen_index.get(index)} [kg-m/s]",fontsize=label_size,labelpad=label_pad)
            pass
        elif type == "change_momentum":
            plt.title(f"2D {chosen_index.get(index)} Change in Momentum",fontsize=title_size)
            plt.xlabel(f"Time [s]",fontsize=label_size,labelpad=label_pad)
            plt.ylabel(f"Change in Momentum {chosen_index.get(index)} [kg-m/s]",fontsize=label_size,labelpad=label_pad)
            pass
        plt.grid(True)
        plt.tight_layout()
        plt.legend(bbox_to_anchor=(1.05, 1), loc=2,fontsize='13')
        plt.show()
        pass
    
    elif plot_type == "3d":
        fig = plt.figure()
        ax = fig.add_subplot(111,projection='3d')
        
        if type == "momentum":
            system_momentum = (craftvals + balloonvals)
            ax.plot(balloonvals[0, 0], balloonvals[0, 1], balloonvals[0,2], label="Balloon Position Start", color='black', marker='x')
            ax.plot(balloonvals[-1, 0], balloonvals[-1, 1], balloonvals[-1,2], label="Balloon Position End", color='black', marker='o')
            ax.plot(craftvals[0, 0], craftvals[0, 1], craftvals[0,2], label="Craft Position Start", color='black',marker='x')
            ax.plot(craftvals[-1, 0], craftvals[-1, 1], craftvals[-1,2], label="Craft Position End", color='black',marker='o')
            ax.plot(system_momentum[0, 0], system_momentum[0, 1], system_momentum[0,2], label="Craft Position Start", color='black',marker='x')
            ax.plot(system_momentum[-1, 0], system_momentum[-1, 1], system_momentum[-1,2], label="Craft Position End", color='black',marker='o')

            # Plot values: 
            ax.plot(craftvals[:,0], craftvals[:,1], craftvals[:,2], label=labels[0] if labels else "Craft", color=colors[0] if colors else "red")
            ax.plot(balloonvals[:,0], balloonvals[:,1], balloonvals[:,2], label=labels[1] if labels else "Balloon", color=colors[1] if colors else "blue")
            ax.plot(system_momentum[:,0], system_momentum[:,1], system_momentum[:,2], label=labels[2] if labels else "System", color=colors[2] if colors else "green")
            pass
        elif type == "ang_momentum":
            system_momentum = (craftvals + balloonvals)
            ax.plot(balloonvals[0, 0], balloonvals[0, 1], balloonvals[0,2], label="Balloon Position Start", color='black', marker='x')
            ax.plot(balloonvals[-1, 0], balloonvals[-1, 1], balloonvals[-1,2], label="Balloon Position End", color='black', marker='o')
            ax.plot(craftvals[0, 0], craftvals[0, 1], craftvals[0,2], label="Craft Position Start", color='black',marker='x')
            ax.plot(craftvals[-1, 0], craftvals[-1, 1], craftvals[-1,2], label="Craft Position End", color='black',marker='o')
            ax.plot(system_momentum[0, 0], system_momentum[0, 1], system_momentum[0,2], label="Craft Position Start", color='black',marker='x')
            ax.plot(system_momentum[-1, 0], system_momentum[-1, 1], system_momentum[-1,2], label="Craft Position End", color='black',marker='o')

            # Plot values: 
            ax.plot(craftvals[:,0], craftvals[:,1], craftvals[:,2], label=labels[0] if labels else "Craft", color=colors[0] if colors else "red")
            ax.plot(balloonvals[:,0], balloonvals[:,1], balloonvals[:,2], label=labels[1] if labels else "Balloon", color=colors[1] if colors else "blue")
            ax.plot(system_momentum[:,0], system_momentum[:,1], system_momentum[:,2], label=labels[2] if labels else "System", color=colors[2] if colors else "green")
            pass
        elif type == "change_momentum":
            ax.plot(change_in_momentum[0, 0], change_in_momentum[0, 1], change_in_momentum[0,2], label="System Position Start", color='black', marker='x')
            ax.plot(change_in_momentum[-1, 0], change_in_momentum[-1, 1], change_in_momentum[-1,2], label="System Position End", color='black', marker='o')
            ax.plot(change_in_momentum[:,0], change_in_momentum[:,1], change_in_momentum[:,2], label=labels[2] if labels else "System", color=colors[2] if colors else "green")
            pass
        elif type == "change_ang_momentum":
            ax.plot(change_in_momentum[0, 0], change_in_momentum[0, 1], change_in_momentum[0,2], label="System Position Start", color='black', marker='x')
            ax.plot(change_in_momentum[-1, 0], change_in_momentum[-1, 1], change_in_momentum[-1,2], label="System Position End", color='black', marker='o')
            ax.plot(change_in_momentum[:,0], change_in_momentum[:,1], change_in_momentum[:,2], label=labels[2] if labels else "System", color=colors[2] if colors else "green")
            pass
        else:
            ax.plot(balloonvals[0, 0], balloonvals[0, 1], balloonvals[0,2], label="Balloon Position Start", color='black', marker='x')
            ax.plot(balloonvals[-1, 0], balloonvals[-1, 1], balloonvals[-1,2], label="Balloon Position End", color='black', marker='o')
            ax.plot(craftvals[0, 0], craftvals[0, 1], craftvals[0,2], label="Craft Position Start", color='black',marker='x')
            ax.plot(craftvals[-1, 0], craftvals[-1, 1], craftvals[-1,2], label="Craft Position End", color='black',marker='o')

            # Plot values: 
            ax.plot(craftvals[:,0], craftvals[:,1], craftvals[:,2], label=labels[0] if labels else "Craft", color=colors[0] if colors else "red")
            ax.plot(balloonvals[:,0], balloonvals[:,1], balloonvals[:,2], label=labels[1] if labels else "Balloon", color=colors[1] if colors else "blue")
            pass
            
        title_size = 15
        label_size = 13    
        if type == "position":
            ax.set_title(f"3D Position",fontsize=title_size)
            ax.set_xlabel("X-Position [m]",fontsize=label_size,labelpad=label_pad)
            ax.set_ylabel("Y-Position [m]",fontsize=label_size,labelpad=label_pad)
            ax.set_zlabel("Z-Position [m]",fontsize=label_size,labelpad=label_pad)
            pass
        elif type == "velocity":
            ax.set_title(f"3D Velocity",fontsize=title_size)
            ax.set_xlabel("X-Velocity [m/s]",fontsize=label_size,labelpad=label_pad)
            ax.set_ylabel("Y-Velocity [m/s]",fontsize=label_size,labelpad=label_pad)
            ax.set_zlabel("Z-Velocity [m/s]",fontsize=label_size,labelpad=label_pad)
            pass
        elif type == "momentum":
            ax.set_title(f"3D Linear Momentum",fontsize=title_size)
            ax.set_xlabel("X-Momentum [kg-m/s]",fontsize=label_size,labelpad=label_pad)
            ax.set_ylabel("Y-Momentum [kg-m/s]",fontsize=label_size,labelpad=label_pad)
            ax.set_zlabel("Z-Momentum [kg-m/s]",fontsize=label_size,labelpad=label_pad)
            pass
        elif type == "ang_momentum":
            ax.set_title(f"3D Angular Momentum",fontsize=title_size)
            ax.set_xlabel("X-Angular Momentum [kg-m^2/s]",fontsize=label_size,labelpad=label_pad)
            ax.set_ylabel("Y-Angular Momentum [kg-m^2/s]",fontsize=label_size,labelpad=label_pad)
            ax.set_zlabel("Z-Angular Momentum [kg-m^2/s]",fontsize=label_size,labelpad=label_pad)
            pass
        elif type == "change_momentum":
            ax.set_title(f"3D Change in Linear Momentum",fontsize=title_size)
            ax.set_xlabel("X-Change in Momentum [kg-m/s]",fontsize=label_size,labelpad=label_pad)
            ax.set_ylabel("Y-Change in Momentum [kg-m/s]",fontsize=label_size,labelpad=label_pad)
            ax.set_zlabel("Z-Change in Momentum [kg-m/s]",fontsize=label_size,labelpad=label_pad)
            ax.set_xlim(-0.025, 0.025)
            ax.set_ylim(-0.025, 0.025)
            pass
        elif type == "change_ang_momentum":
            ax.set_title(f"3D Change in Angular Momentum",fontsize=title_size)
            ax.set_xlabel("X-Change in Angular Momentum [kg-m^2s/s]",fontsize=label_size,labelpad=label_pad)
            ax.set_ylabel("Y-Change in Angular Momentum [kg-m^2/s]",fontsize=label_size,labelpad=label_pad)
            ax.set_zlabel("Z-Change in Angular Momentum [kg-m^2/s]",fontsize=label_size,labelpad=label_pad)
            ax.set_xlim(-0.001, 0.001)
            ax.set_ylim(-0.001, 0.001)
            ax.set_zlim(-0.001, 0.001)
            pass
        
        ax.grid(True)
        ax.legend(bbox_to_anchor=(1.05, 1), loc=2,fontsize='13')
        plt.tight_layout()
        plt.show()
        pass
    else: 
        print("Invalid plot or missing required data")
        pass
    pass

# ------------------------------------------------------------------------    
# Initial SpatialNDE2 Viewer attempt: 
# ------------------------------------------------------------------------    

orient_dtype = [('quat', '<f4', (4,)),('offset', '<f4', (4,))]
null_orient = np.array(((1,0,0,0),(0,0,0,1)),dtype=orient_dtype)

def setup_camera(main_viewer,channel_name,location):
    '''
    Need to set up the "camera" (this is the point of view for the viewer)
    Camera Coordinates: X right, y up, and z back
    World Coordinates: X right, Y forward, Z up
    In this scenario, we will be in the Howe Atrium, so we want the camera to sit at a level angle.
    '''
    # Need to transform coordinate frames from camera coords to world coords: 
    # First Step: Rotate 90 degrees around X-axis, this will give X right, Y back, Z up 
    # Second Step: Rotate 180 degrees around the Z-axis, this will give X right, Y forward, Z up (this is what we want)
    quat1 = np.array((1/np.sqrt(2), 1/np.sqrt(2),0, 0)) # 90 degrees around x
    quat2 = quatpy.quaternion_normalize(np.array((0,0,0,1))) # 180 degrees around z
    world_over_camera_quat = quatpy.quaternion_product_normalized(quat2,quat1)

    camera_backoff = 0 # meters
    
    # Two locations: Basement and 1st floor balcony
    if location == "balcony":
        # world_over_camera_offset = np.array((14, -7, 13.5))
        pass
    elif location == "basement":
        # world_over_camera_offset = np.array((13.5, -7, 7.5))
        world_over_camera_offset = np.array((18, -10, 10)) #np.array((27, 22, -25))

    main_viewer.viewer.set_camera_pose(channel_name,np.array((world_over_camera_quat,np.concatenate(((world_over_camera_offset,(1.0,))))),dtype=orient_dtype))
    main_viewer.viewer.set_rotation_center_dist(channel_name,camera_backoff)
    pass

def update_pose_channel(trans,channel,quat,offset,to_reorient,background_channel=None):
    ref = snde.create_pose_channel_ndarray_ref(trans,channel,to_reorient) # create_ndarray_ref
    ref.allocate_storage([1])
    orient = np.array((quat,np.concatenate((offset,(1,)))), dtype=orient_dtype)
    ref.data[0] = orient
    if background_channel is not None:
        snde.pose_channel_recording.from_ndarray_recording(ref.rec).set_untransformed_render_channel(background_channel)
        pass
    ref.rec.mark_data_and_metadata_ready()
    pass
    
def update_channels(trans,world_over_craft,world_over_balloon,craft_model_tree,balloon_model_tree,atrium_model_assembly,atrium_model_assembly_channel):
    # Update the craft pose channel:
    update_pose_channel(trans,world_over_craft,craft.pose_worldcoords_over_bodycoords.quat,craft.pose_worldcoords_over_bodycoords.offset,snde.recdb_path_join(craft_model_tree,"meshed"))
    
    # Update the balloon pose channel: 
    update_pose_channel(trans,world_over_balloon,balloon.pose_worldcoords_over_bodycoords.quat,balloon.pose_worldcoords_over_bodycoords.offset,snde.recdb_path_join(balloon_model_tree,"meshed"))
    
    pass
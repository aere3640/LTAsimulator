import os
import pygame
import numpy as np

# PyGame Documentation:
# https://www.pygame.org/docs/ref/joystick.html

class Joystick(object):
    myjoystick = None 
    axis_data = None 
    button_data = None
    
    def __init__(self): 
        '''
        This will initialize the joystick components using the PyGame module.
        '''
        
        pygame.init()
        pygame.joystick.init()
        
        # Ensure a joystick is connected/found:
        if pygame.joystick.get_count() == 0:
            raise Exception("Joystick not detected.")
        else:
            self.myjoystick = pygame.joystick.Joystick(0)
            self.myjoystick.init()
            
            # Check the number of axes on the joystick
            self.num_axes = self.myjoystick.get_numaxes()
            print(f"Joystick has {self.num_axes} axes.")
            pass
        
    # Create methods to determine the axes and buttons for the joystick
    def find_axis(self, axis_data):
        '''
        Get the value of a specific axis.
        Parameters: 
            axis_data: Axis index ( 0 for X-axis, 1 for Y-axis).
        Returns: 
            Axis value between -1 and 1.
        '''
        
        # PyGame error: 
        if axis_data < 0 or axis_data >= self.num_axes:
            raise pygame.error(f"Invalid joystick axis {axis_data}. The joystick has {self.num_axes} axes.")
        
        return self.myjoystick.get_axis(axis_data)
    
    def find_button(self, button_data):
        '''
        Get the value of button. 
        Parameters:
            button_data: Button index
        Returns:
            0 if button is not pressed, 1 if button is pressed
        '''
        return self.myjoystick.get_button(button_data)

    def read_data(self):
        '''
        Process the joystick events and print the data.
        '''
        pygame.event.pump()
        # Using the number of axes for the joystick, find the axis value for each axis
        axis_values = np.zeros(self.num_axes,dtype='d')
        for axis_index in range(self.num_axes):
            axis_value = self.myjoystick.get_axis(axis_index)
            axis_values[axis_index] = axis_value
            pass
        
        return axis_values
        
        # Map the axis above to control values:
        # control_values = np.array([1.0, 1.5, 1.5],dtype='d') # [milliseconds]
        # 1.0 milliseconds represents full to the left or throttle = 0, 2.0 milliseconds represents full to the right or max throttle.
        

        # controlvalue_liftfan = self.convert_axis_to_control(-axis_1,esc_type='non-reversing') 
        # controlvalue_leftfan = self.convert_axis_to_control(-axis_0,esc_type='reversing')
        # controlvalue_rightfan = self.convert_axis_to_control(axis_0,esc_type='reversing')
        
        # Assign into a numpy array (LTA_config uses a numpy array)
        # control_values = np.array([controlvalue_liftfan, controlvalue_leftfan, controlvalue_rightfan], dtype='d')  
        
            
# This running functionality is used to test the control values:    
if __name__ == "__main__":
    joystick = Joystick()  # Create Joystick object
    try:
        while True:
            control_values = joystick.read_data()  # Get control values from joystick input
            print(f"Control Values: {control_values}")
            
            if joystick.find_button(0):  # Button 0 is the first button on the joystick
                print("Close button pressed. Exiting...")
                pygame.quit()  # Clean up when user exits
                break  # Exit the loop and end the program
            pass
    except KeyboardInterrupt:
        pygame.quit()  # Clean up when user exits
        print("\nJoystick interface closed.")
        pass
    pass
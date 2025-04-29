import numpy as np
import threading

import pygame
from dataguzzler_python.dgpy import Module, InitCompatibleThread
import spatialnde2 as snde

from dataguzzler_python.QtWrapper import QtWrapper

# PyGame Documentation:
# https://www.pygame.org/docs/ref/joystick.html 

class joystick_dgp_module(metaclass=Module):
    '''
    Must be initialized in the main thread. It also requires Qt to delegate pygame calls
    back to the main thread. 
    '''
    module_name = None 
    recdb = None
    joystick_chan = None
    joystick_thread = None
    thread_terminate = None
    
    myjoystick = None 
    axis_data = None 
    joystick_guid = None
    
    def __init__(self,module_name,recdb=None,joystick_guid=None):
        self.module_name = module_name
        self.recdb = recdb
        self.joystick_guid_input = joystick_guid
        self.thread_terminate = False
        
        # The pygame.init() must be run in order to run the pygame.event.pump()
        # Although, this initializes graphics, which may lead to conflicts. So far 
        # it does not seem to cause any issues.
        pygame.init()
        pygame.joystick.init()
        
        # Ensure a joystick is connected/found:
        if pygame.joystick.get_count() == 0:
            raise Exception("Joystick not detected.")
        else: 
            for i in range(pygame.joystick.get_count()):
                
                joystick = pygame.joystick.Joystick(i)
                joystick.init()
                joystick_guid_found = joystick.get_guid()
                
                if self.joystick_guid_input and self.joystick_guid_input == joystick_guid_found:
                    self.myjoystick = joystick
                    print(f"Found matching joystick --> GUID: {joystick_guid_found}")
                    break
                elif self.joystick_guid_input is None:
                    # If no guid input (None), select the first joystick
                    self.myjoystick = QtWrapper(joystick)
                    print(f"Selecting first joystick because no GUID input found --> GUID: {joystick_guid_found}")
                    break
                
            else: 
                # given GUID was not found, inform user
                raise Exception("Joystick with the provided GUID was not found.")
            
            # Check the number of axes on the joystick
            self.num_axes = self.myjoystick.get_numaxes()
            print(f"Joystick has {self.num_axes} axes.")
            pass
                
        with recdb.start_transaction() as trans:
            self.joystick_chan = recdb.define_channel(trans,"/joystick",module_name)
            ref = snde.create_ndarray_ref(trans,self.joystick_chan,snde.SNDE_RTN_FLOAT32)
            ref_len = self.num_axes
            ref.allocate_storage([ref_len])
            axis_values_array = self.read_data()
            ref.data[:] = axis_values_array
            ref.rec.mark_data_and_metadata_ready()
            pass
        
        self.joystick_thread = threading.Thread(target=self.joystick_thread_code)
        self.joystick_thread.start()
        pass
    

    def read_data(self):
        '''
        Process the joystick events and print the data.
        '''
        # pygame.event.pump() must be run in the main thread. Therefore, we use the QtWrapper
        # to dispatch it to the main thread.
       
        # This will only give two context switches per read_data call rather than 
        # at every step through the loop plus the pygame.event.pump() call.
        def get_values_in_mainthread(myjoystick,num_axes):
            pygame.event.pump() # This allows pygame to read in values constantly

            # Using the number of axes for the joystick, find the axis value for each axis
            axis_values = np.zeros(num_axes,dtype='d')
            for axis_index in range(num_axes):
                axis_value = QtWrapper(myjoystick.get_axis)(axis_index)
                axis_values[axis_index] = axis_value
                if (axis_value > 1.0) and (axis_value < -1.0):
                    raise ValueError(f"Axis value {axis_value} for axis {axis_index} is out of range.")
                pass
            
            return axis_values
        return QtWrapper(get_values_in_mainthread)(self.myjoystick,self.num_axes)
    
    def joystick_thread_code(self): 
        InitCompatibleThread(self,"joystick_thread")
        counter = 0
        while True:
            counter += 1
            control_values = self.read_data() 
            with self.recdb.start_transaction() as trans:
                ref = snde.create_ndarray_ref(trans,self.joystick_chan,snde.SNDE_RTN_FLOAT32)
                ref_len = self.num_axes
                ref.allocate_storage([ref_len])
                ref.data[:] = control_values
                ref.rec.mark_data_and_metadata_ready()
                pass
            if self.thread_terminate:
                return
            # print(f'Counter: {counter}')
            pass
            
        pass
    
    
    def __del__(self):
        self.thread_terminate = True
        self.joystick_thread.join()
        pass
import os
import sys
import time
import numpy as np
import pybullet as p
from collections import namedtuple

from pybullet_planning.utils import INF, CLIENT, CLIENTS
from pybullet_planning.utils import is_darwin
from pybullet_planning.interfaces.env_manager import step_simulation, threaded_input

# from future_builtins import map, filter
# from builtins import input # TODO - use future
try:
    user_input = raw_input
except NameError:
    user_input = input

#####################################

# https://stackoverflow.com/questions/5081657/how-do-i-prevent-a-c-shared-library-to-print-on-stdout-in-python/14797594#14797594
# https://stackoverflow.com/questions/4178614/suppressing-output-of-module-calling-outside-library
# https://stackoverflow.com/questions/4675728/redirect-stdout-to-a-file-in-python/22434262#22434262

class HideOutput(object):
    '''
    A context manager that block stdout for its scope, usage:
    with HideOutput():
        os.system('ls -l')
    '''
    DEFAULT_ENABLE = True
    def __init__(self, enable=None):
        if enable is None:
            enable = self.DEFAULT_ENABLE
        self.enable = enable
        if not self.enable:
            return
        sys.stdout.flush()
        self._origstdout = sys.stdout
        self._oldstdout_fno = os.dup(sys.stdout.fileno())
        self._devnull = os.open(os.devnull, os.O_WRONLY)

    def __enter__(self):
        if not self.enable:
            return
        self._newstdout = os.dup(1)
        os.dup2(self._devnull, 1)
        os.close(self._devnull)
        sys.stdout = os.fdopen(self._newstdout, 'w')

    def __exit__(self, exc_type, exc_val, exc_tb):
        if not self.enable:
            return
        sys.stdout.close()
        sys.stdout = self._origstdout
        sys.stdout.flush()
        os.dup2(self._oldstdout_fno, 1)
        os.close(self._oldstdout_fno) # Added

#####################################

def elapsed_time(start_time):
    return time.time() - start_time

MouseEvent = namedtuple('MouseEvent', ['eventType', 'mousePosX', 'mousePosY', 'buttonIndex', 'buttonState'])

def get_mouse_events():
    return list(MouseEvent(*event) for event in p.getMouseEvents(physicsClientId=CLIENT))

def update_viewer():
    # https://docs.python.org/2/library/select.html
    # events = p.getKeyboardEvents() # TODO: only works when the viewer is in focus
    get_mouse_events()
    # for k, v in keys.items():
    #    #p.KEY_IS_DOWN, p.KEY_WAS_RELEASED, p.KEY_WAS_TRIGGERED
    #    if (k == p.B3G_RETURN) and (v & p.KEY_WAS_TRIGGERED):
    #        return
    # time.sleep(1e-3) # Doesn't work
    # disable_gravity()

def wait_for_duration(duration): #, dt=0):
    t0 = time.time()
    while elapsed_time(t0) <= duration:
        update_viewer()

def simulate_for_duration(duration):
    dt = get_time_step()
    for i in range(int(duration / dt)):
        step_simulation()

def get_time_step():
    # {'gravityAccelerationX', 'useRealTimeSimulation', 'gravityAccelerationZ', 'numSolverIterations',
    # 'gravityAccelerationY', 'numSubSteps', 'fixedTimeStep'}
    return p.getPhysicsEngineParameters(physicsClientId=CLIENT)['fixedTimeStep']

def enable_separating_axis_test():
    p.setPhysicsEngineParameter(enableSAT=1, physicsClientId=CLIENT)
    #p.setCollisionFilterPair()
    #p.setCollisionFilterGroupMask()
    #p.setInternalSimFlags()
    # enableFileCaching: Set to 0 to disable file caching, such as .obj wavefront file loading
    #p.getAPIVersion() # TODO: check that API is up-to-date
    #p.isNumpyEnabled()

def simulate_for_sim_duration(sim_duration, real_dt=0, frequency=INF):
    t0 = time.time()
    sim_dt = get_time_step()
    sim_time = 0
    last_print = 0
    while sim_time < sim_duration:
        if frequency < (sim_time - last_print):
            print('Sim time: {:.3f} | Real time: {:.3f}'.format(sim_time, elapsed_time(t0)))
            last_print = sim_time
        step_simulation()
        sim_time += sim_dt
        time.sleep(real_dt)

def wait_for_user(message='Press enter to continue'):
    if is_darwin():
        # OS X doesn't multi-thread the OpenGL visualizer
        #wait_for_interrupt()
        return threaded_input(message)
    return user_input(message)

def is_unlocked():
    return CLIENTS[CLIENT] is True

def wait_if_unlocked(*args, **kwargs):
    if is_unlocked():
        wait_for_user(*args, **kwargs)

def wait_for_interrupt(max_time=np.inf):
    """
    Hold Ctrl to move the camera as well as zoom
    """
    print('Press Ctrl-C to continue')
    try:
        wait_for_duration(max_time)
    except KeyboardInterrupt:
        pass
    finally:
        print()

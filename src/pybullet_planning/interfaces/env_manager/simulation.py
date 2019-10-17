import os
from collections import namedtuple
import numpy as np
import pybullet as p

from pybullet_planning.utils import CLIENT, CLIENTS, GRAVITY, INFO_FROM_BODY, STATIC_MASS
from pybullet_planning.utils import is_darwin

from .savers import Saver
from pybullet_planning.interfaces.geometry import create_obj
from pybullet_planning.interfaces.visualize import HideOutput, update_viewer, user_input
from pybullet_planning.interfaces.robots import set_pose

#####################################

# class World(object):
#     def __init__(self, client):
#         self.client = client
#         self.bodies = {}
#     def activate(self):
#         set_client(self.client)
#     def load(self, path, name=None, fixed_base=False, scale=1.):
#         body = p.loadURDF(path, useFixedBase=fixed_base, physicsClientId=self.client)
#         self.bodies[body] = URDFInfo(name, path, fixed_base, scale)
#         return body
#     def remove(self, body):
#         del self.bodies[body]
#         return p.removeBody(body, physicsClientId=self.client)
#     def reset(self):
#         p.resetSimulation(physicsClientId=self.client)
#         self.bodies = {}
#     # TODO: with statement
#     def copy(self):
#         raise NotImplementedError()
#     def __repr__(self):
#         return '{}({})'.format(self.__class__.__name__, len(self.bodies))


def disable_viewer():
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False, physicsClientId=CLIENT)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False, physicsClientId=CLIENT)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False, physicsClientId=CLIENT)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, False, physicsClientId=CLIENT)
    #p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, False, physicsClientId=CLIENT)
    #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, True, physicsClientId=CLIENT)
    #p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, False, physicsClientId=CLIENT)
    #p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, True, physicsClientId=CLIENT)
    #p.COV_ENABLE_MOUSE_PICKING, p.COV_ENABLE_KEYBOARD_SHORTCUTS

def set_renderer(enable):
    client = CLIENT
    if not has_gui(client):
        return
    CLIENTS[client] = enable
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, int(enable), physicsClientId=client)

class LockRenderer(Saver):
    # disabling rendering temporary makes adding objects faster
    def __init__(self, lock=True):
        self.client = CLIENT
        self.state = CLIENTS[self.client]
        # skip if the visualizer isn't active
        if has_gui(self.client) and lock:
            set_renderer(enable=False)

    def restore(self):
        if not has_gui(self.client):
            return
        assert self.state is not None
        if self.state != CLIENTS[self.client]:
           set_renderer(enable=self.state)

def connect(use_gui=True, shadows=True):
    # Shared Memory: execute the physics simulation and rendering in a separate process
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/vrminitaur.py#L7
    # make sure to compile pybullet with PYBULLET_USE_NUMPY enabled
    if use_gui and not is_darwin() and ('DISPLAY' not in os.environ):
        use_gui = False
        print('No display detected!')
    method = p.GUI if use_gui else p.DIRECT
    with HideOutput():
        # options="--width=1024 --height=768"
        #  --window_backend=2 --render_device=0'
        sim_id = p.connect(method)
        #sim_id = p.connect(p.GUI, options="--opengl2") if use_gui else p.connect(p.DIRECT)
    assert 0 <= sim_id
    #sim_id2 = p.connect(p.SHARED_MEMORY)
    #print(sim_id, sim_id2)
    CLIENTS[sim_id] = True if use_gui else None
    if use_gui:
        # p.COV_ENABLE_PLANAR_REFLECTION
        # p.COV_ENABLE_SINGLE_STEP_RENDERING
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, False, physicsClientId=sim_id)
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, False, physicsClientId=sim_id)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, False, physicsClientId=sim_id)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False, physicsClientId=sim_id)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False, physicsClientId=sim_id)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, shadows, physicsClientId=sim_id)

    # you can also use GUI mode, for faster OpenGL rendering (instead of TinyRender CPU)
    #visualizer_options = {
    #    p.COV_ENABLE_WIREFRAME: 1,
    #    p.COV_ENABLE_SHADOWS: 0,
    #    p.COV_ENABLE_RENDERING: 0,
    #    p.COV_ENABLE_TINY_RENDERER: 1,
    #    p.COV_ENABLE_RGB_BUFFER_PREVIEW: 0,
    #    p.COV_ENABLE_DEPTH_BUFFER_PREVIEW: 0,
    #    p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW: 0,
    #    p.COV_ENABLE_VR_RENDER_CONTROLLERS: 0,
    #    p.COV_ENABLE_VR_PICKING: 0,
    #    p.COV_ENABLE_VR_TELEPORTING: 0,
    #}
    #for pair in visualizer_options.items():
    #    p.configureDebugVisualizer(*pair)
    return sim_id

def threaded_input(*args, **kwargs):
    # OS X doesn't multi-thread the OpenGL visualizer
    # http://openrave.org/docs/0.8.2/_modules/openravepy/misc/#SetViewerUserThread
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/userData.py
    # https://github.com/bulletphysics/bullet3/tree/master/examples/ExampleBrowser
    #from pybullet_utils import bullet_client
    #from pybullet_utils.bullet_client import BulletClient
    #server = bullet_client.BulletClient(connection_mode=p.SHARED_MEMORY_SERVER) # GUI_SERVER
    #sim_id = p.connect(p.GUI)
    #print(dir(server))
    #client = bullet_client.BulletClient(connection_mode=p.SHARED_MEMORY)
    #sim_id = p.connect(p.SHARED_MEMORY)

    #threading = __import__('threading')
    import threading
    data = []
    thread = threading.Thread(target=lambda: data.append(user_input(*args, **kwargs)), args=[])
    thread.start()
    #threading.enumerate()
    #thread_id = 0
    #for tid, tobj in threading._active.items():
    #    if tobj is thread:
    #        thread_id = tid
    #        break
    try:
        while thread.is_alive():
            update_viewer()
    finally:
        thread.join()
    return data[-1]

def disconnect():
    # TODO: change CLIENT?
    if CLIENT in CLIENTS:
        del CLIENTS[CLIENT]
    with HideOutput():
        return p.disconnect(physicsClientId=CLIENT)

def is_connected():
    return p.getConnectionInfo(physicsClientId=CLIENT)['isConnected']

def get_connection(client=None):
    return p.getConnectionInfo(physicsClientId=get_client(client))['connectionMethod']

def has_gui(client=None):
    return get_connection(get_client(client)) == p.GUI

def get_data_path():
    import pybullet_data
    return pybullet_data.getDataPath()

def add_data_path(data_path=None):
    if data_path is None:
        data_path = get_data_path()
    p.setAdditionalSearchPath(data_path)
    return data_path

def enable_gravity():
    p.setGravity(0, 0, -GRAVITY, physicsClientId=CLIENT)

def disable_gravity():
    p.setGravity(0, 0, 0, physicsClientId=CLIENT)

def step_simulation():
    p.stepSimulation(physicsClientId=CLIENT)

def set_real_time(real_time):
    p.setRealTimeSimulation(int(real_time), physicsClientId=CLIENT)

def enable_real_time():
    set_real_time(True)

def disable_real_time():
    set_real_time(False)

def update_state():
    # TODO: this doesn't seem to automatically update still
    disable_gravity()
    #step_simulation()
    #for body in get_bodies():
    #    for link in get_links(body):
    #        # if set to 1 (or True), the Cartesian world position/orientation
    #        # will be recomputed using forward kinematics.
    #        get_link_state(body, link)
    #for body in get_bodies():
    #    get_pose(body)
    #    for joint in get_joints(body):
    #        get_joint_position(body, joint)
    #p.getKeyboardEvents()
    #p.getMouseEvents()

def reset_simulation():
    p.resetSimulation(physicsClientId=CLIENT)

#####################################

# Simulation

def get_client(client=None):
    if client is None:
        return CLIENT
    return client

def set_client(client):
    global CLIENT
    CLIENT = client

ModelInfo = namedtuple('URDFInfo', ['name', 'path', 'fixed_base', 'scale'])

def get_model_info(body):
    key = (CLIENT, body)
    return INFO_FROM_BODY.get(key, None)

def get_urdf_flags(cache=False, cylinder=False):
    # by default, Bullet disables self-collision
    # URDF_INITIALIZE_SAT_FEATURES
    # URDF_ENABLE_CACHED_GRAPHICS_SHAPES seems to help
    # but URDF_INITIALIZE_SAT_FEATURES does not (might need to be provided a mesh)
    # flags = p.URDF_INITIALIZE_SAT_FEATURES | p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
    flags = 0
    if cache:
        flags |= p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
    if cylinder:
        flags |= p.URDF_USE_IMPLICIT_CYLINDER
    return flags

def load_pybullet(filename, fixed_base=False, scale=1., **kwargs):
    # fixed_base=False implies infinite base mass
    with LockRenderer():
        if filename.endswith('.urdf'):
            flags = get_urdf_flags(**kwargs)
            body = p.loadURDF(filename, useFixedBase=fixed_base, flags=flags,
                              globalScaling=scale, physicsClientId=CLIENT)
        elif filename.endswith('.sdf'):
            body = p.loadSDF(filename, physicsClientId=CLIENT)
        elif filename.endswith('.xml'):
            body = p.loadMJCF(filename, physicsClientId=CLIENT)
        elif filename.endswith('.bullet'):
            body = p.loadBullet(filename, physicsClientId=CLIENT)
        elif filename.endswith('.obj'):
            # TODO: fixed_base => mass = 0?
            body = create_obj(filename, scale=scale, **kwargs)
        else:
            raise ValueError(filename)
    INFO_FROM_BODY[CLIENT, body] = ModelInfo(None, filename, fixed_base, scale)
    return body

def set_caching(cache):
    p.setPhysicsEngineParameter(enableFileCaching=int(cache), physicsClientId=CLIENT)

def load_model_info(info):
    # TODO: disable file caching to reuse old filenames
    # p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=CLIENT)
    if info.path.endswith('.urdf'):
        return load_pybullet(info.path, fixed_base=info.fixed_base, scale=info.scale)
    if info.path.endswith('.obj'):
        mass = STATIC_MASS if info.fixed_base else 1.
        return create_obj(info.path, mass=mass, scale=info.scale)
    raise NotImplementedError(info.path)

URDF_FLAGS = [p.URDF_USE_INERTIA_FROM_FILE,
              p.URDF_USE_SELF_COLLISION,
              p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
              p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS]

def get_model_path(rel_path): # TODO: add to search path
    directory = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(directory, '..', rel_path)

def load_model(rel_path, pose=None, **kwargs):
    # TODO: error with loadURDF when loading MESH visual and CYLINDER collision
    abs_path = get_model_path(rel_path)
    add_data_path()
    #with LockRenderer():
    body = load_pybullet(abs_path, **kwargs)
    if pose is not None:
        set_pose(body, pose)
    return body

def save_state():
    return p.saveState(physicsClientId=CLIENT)

def restore_state(state_id):
    p.restoreState(stateId=state_id, physicsClientId=CLIENT)

def save_bullet(filename):
    p.saveBullet(filename, physicsClientId=CLIENT)

def restore_bullet(filename):
    p.restoreState(fileName=filename, physicsClientId=CLIENT)

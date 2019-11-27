import os
import numpy as np
import pybullet as p
from collections import defaultdict, namedtuple
from itertools import count

from pybullet_planning.utils import CLIENT, DEFAULT_EXTENTS, DEFAULT_HEIGHT, DEFAULT_RADIUS, \
    DEFAULT_MESH, DEFAULT_SCALE, DEFAULT_NORMAL, BASE_LINK, INFO_FROM_BODY, STATIC_MASS, UNKNOWN_FILE, BASE_LINK, NULL_ID
from pybullet_planning.utils import get_client
from pybullet_planning.interfaces.env_manager.pose_transformation import unit_pose, multiply

#####################################
# Shapes

ModelInfo = namedtuple('URDFInfo', ['name', 'path', 'fixed_base', 'scale'])

SHAPE_TYPES = {
    p.GEOM_SPHERE: 'sphere',  # 2
    p.GEOM_BOX: 'box',  # 3
    p.GEOM_CYLINDER: 'cylinder',  # 4
    p.GEOM_MESH: 'mesh',  # 5
    p.GEOM_PLANE: 'plane',  # 6
    p.GEOM_CAPSULE: 'capsule',  # 7
    # p.GEOM_FORCE_CONCAVE_TRIMESH
}

# object_unique_id and linkIndex seem to be noise
CollisionShapeData = namedtuple('CollisionShapeData', ['object_unique_id', 'linkIndex',
                                                       'geometry_type', 'dimensions', 'filename',
                                                       'local_frame_pos', 'local_frame_orn'])

VisualShapeData = namedtuple('VisualShapeData', ['objectUniqueId', 'linkIndex',
                                                 'visualGeometryType', 'dimensions', 'meshAssetFileName',
                                                 'localVisualFrame_position', 'localVisualFrame_orientation',
                                                 'rgbaColor'])  # 'textureUniqueId'

def get_box_geometry(width, length, height):
    return {
        'shapeType': p.GEOM_BOX,
        'halfExtents': [width/2., length/2., height/2.]
    }


def get_cylinder_geometry(radius, height):
    return {
        'shapeType': p.GEOM_CYLINDER,
        'radius': radius,
        'length': height,
    }


def get_sphere_geometry(radius):
    return {
        'shapeType': p.GEOM_SPHERE,
        'radius': radius,
    }


def get_capsule_geometry(radius, height):
    return {
        'shapeType': p.GEOM_CAPSULE,
        'radius': radius,
        'length': height,
    }


def get_plane_geometry(normal):
    return {
        'shapeType': p.GEOM_PLANE,
        'planeNormal': normal,
    }


def get_mesh_geometry(path, scale=1.):
    return {
        'shapeType': p.GEOM_MESH,
        'fileName': path,
        'meshScale': scale*np.ones(3),
    }


def create_collision_shape(geometry, pose=unit_pose()):
    point, quat = pose
    collision_args = {
        'collisionFramePosition': point,
        'collisionFrameOrientation': quat,
        'physicsClientId': CLIENT,
    }
    collision_args.update(geometry)
    if 'length' in collision_args:
        # TODO: pybullet bug visual => length, collision => height
        collision_args['height'] = collision_args['length']
        del collision_args['length']
    return p.createCollisionShape(**collision_args)


def create_visual_shape(geometry, pose=unit_pose(), color=(1, 0, 0, 1), specular=None):
    if (color is None):  # or not has_gui():
        return NULL_ID
    point, quat = pose
    visual_args = {
        'rgbaColor': color,
        'visualFramePosition': point,
        'visualFrameOrientation': quat,
        'physicsClientId': CLIENT,
    }
    visual_args.update(geometry)
    if specular is not None:
        visual_args['specularColor'] = specular
    return p.createVisualShape(**visual_args)


def create_shape(geometry, pose=unit_pose(), collision=True, **kwargs):
    collision_id = create_collision_shape(geometry, pose=pose) if collision else NULL_ID
    visual_id = create_visual_shape(geometry, pose=pose, **kwargs)
    return collision_id, visual_id


def plural(word):
    exceptions = {'radius': 'radii'}
    if word in exceptions:
        return exceptions[word]
    if word.endswith('s'):
        return word
    return word + 's'


def create_shape_array(geoms, poses, colors=None):
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/pybullet.c
    # createCollisionShape: height
    # createVisualShape: length
    # createCollisionShapeArray: lengths
    # createVisualShapeArray: lengths
    mega_geom = defaultdict(list)
    for geom in geoms:
        extended_geom = get_default_geometry()
        extended_geom.update(geom)
        #extended_geom = geom.copy()
        for key, value in extended_geom.items():
            mega_geom[plural(key)].append(value)

    collision_args = mega_geom.copy()
    for (point, quat) in poses:
        collision_args['collisionFramePositions'].append(point)
        collision_args['collisionFrameOrientations'].append(quat)
    collision_id = p.createCollisionShapeArray(physicsClientId=CLIENT, **collision_args)
    if (colors is None):  # or not has_gui():
        return collision_id, NULL_ID

    visual_args = mega_geom.copy()
    for (point, quat), color in zip(poses, colors):
        # TODO: color doesn't seem to work correctly here
        visual_args['rgbaColors'].append(color)
        visual_args['visualFramePositions'].append(point)
        visual_args['visualFrameOrientations'].append(quat)
    visual_id = p.createVisualShapeArray(physicsClientId=CLIENT, **visual_args)
    return collision_id, visual_id

#####################################


def create_body(collision_id=-1, visual_id=-1, mass=STATIC_MASS):
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id, physicsClientId=CLIENT)


def create_box(w, l, h, mass=STATIC_MASS, color=(1, 0, 0, 1)):
    collision_id, visual_id = create_shape(get_box_geometry(w, l, h), color=color)
    return create_body(collision_id, visual_id, mass=mass)
    # basePosition | baseOrientation
    # linkCollisionShapeIndices | linkVisualShapeIndices


def create_cylinder(radius, height, mass=STATIC_MASS, color=(0, 0, 1, 1)):
    collision_id, visual_id = create_shape(get_cylinder_geometry(radius, height), color=color)
    return create_body(collision_id, visual_id, mass=mass)


def create_capsule(radius, height, mass=STATIC_MASS, color=(0, 0, 1, 1)):
    collision_id, visual_id = create_shape(get_capsule_geometry(radius, height), color=color)
    return create_body(collision_id, visual_id, mass=mass)


def create_sphere(radius, mass=STATIC_MASS, color=(0, 0, 1, 1)):
    collision_id, visual_id = create_shape(get_sphere_geometry(radius), color=color)
    return create_body(collision_id, visual_id, mass=mass)


def create_plane(normal=[0, 0, 1], mass=STATIC_MASS, color=(0, 0, 0, 1)):
    # color seems to be ignored in favor of a texture
    collision_id, visual_id = create_shape(get_plane_geometry(normal), color=color)
    return create_body(collision_id, visual_id, mass=mass)


def create_obj(path, scale=1., mass=STATIC_MASS, collision=True, color=(0.5, 0.5, 0.5, 1)):
    collision_id, visual_id = create_shape(get_mesh_geometry(path, scale=scale), collision=collision, color=color)
    body = create_body(collision_id, visual_id, mass=mass)
    fixed_base = (mass == STATIC_MASS)
    INFO_FROM_BODY[CLIENT, body] = ModelInfo(None, path, fixed_base, scale)  # TODO: store geometry info instead?
    return body

#####################################

def vertices_from_data(data):
    from pybullet_planning.interfaces.env_manager.pose_transformation import apply_affine
    from pybullet_planning.interfaces.env_manager.shape_creation import get_data_type, get_data_extents, get_data_radius, get_data_height, \
        get_data_filename, get_data_scale, get_collision_data, get_data_pose
    from pybullet_planning.interfaces.env_manager import get_model_info

    geometry_type = get_data_type(data)
    #if geometry_type == p.GEOM_SPHERE:
    #    parameters = [get_data_radius(data)]
    if geometry_type == p.GEOM_BOX:
        extents = np.array(get_data_extents(data))
        aabb = AABB(-extents/2., +extents/2.)
        vertices = get_aabb_vertices(aabb)
    elif geometry_type in (p.GEOM_CYLINDER, p.GEOM_CAPSULE):
        # TODO: p.URDF_USE_IMPLICIT_CYLINDER
        radius, height = get_data_radius(data), get_data_height(data)
        extents = np.array([2*radius, 2*radius, height])
        aabb = AABB(-extents/2., +extents/2.)
        vertices = get_aabb_vertices(aabb)
    elif geometry_type == p.GEOM_SPHERE:
        radius = get_data_radius(data)
        half_extents = radius*np.ones(3)
        aabb = AABB(-half_extents, +half_extents)
        vertices = get_aabb_vertices(aabb)
    elif geometry_type == p.GEOM_MESH:
        from pybullet_planning.interfaces.geometry.mesh import read_obj
        filename, scale = get_data_filename(data), get_data_scale(data)
        if filename == UNKNOWN_FILE:
            raise RuntimeError(filename)
        mesh = read_obj(filename, decompose=False)
        vertices = [scale*np.array(vertex) for vertex in mesh.vertices]
        # TODO: could compute AABB here for improved speed at the cost of being conservative
    #elif geometry_type == p.GEOM_PLANE:
    #   parameters = [get_data_extents(data)]
    else:
        raise NotImplementedError(geometry_type)
    return apply_affine(get_data_pose(data), vertices)


#####################################

def visual_shape_from_data(data, client=None):
    client = get_client(client)
    if (data.visualGeometryType == p.GEOM_MESH) and (data.meshAssetFileName == UNKNOWN_FILE):
        return -1
    # visualFramePosition: translational offset of the visual shape with respect to the link
    # visualFrameOrientation: rotational offset (quaternion x,y,z,w) of the visual shape with respect to the link frame
    #inertial_pose = get_joint_inertial_pose(data.objectUniqueId, data.linkIndex)
    #point, quat = multiply(invert(inertial_pose), pose)
    point, quat = get_data_pose(data)
    return p.createVisualShape(shapeType=data.visualGeometryType,
                               radius=get_data_radius(data),
                               halfExtents=np.array(get_data_extents(data))/2,
                               length=get_data_height(data),  # TODO: pybullet bug
                               fileName=get_data_filename(data),
                               meshScale=get_data_scale(data),
                               planeNormal=get_data_normal(data),
                               rgbaColor=data.rgbaColor,
                               # specularColor=,
                               visualFramePosition=point,
                               visualFrameOrientation=quat,
                               physicsClientId=client)


def get_visual_data(body, link=BASE_LINK):
    visual_data = [VisualShapeData(*tup) for tup in p.getVisualShapeData(body, physicsClientId=CLIENT)]
    return list(filter(lambda d: d.linkIndex == link, visual_data))


def clone_visual_shape(body, link, client=None):
    client = get_client(client)
    # if not has_gui(client):
    #    return -1
    visual_data = get_visual_data(body, link)
    if not visual_data:
        return -1
    assert (len(visual_data) == 1)
    return visual_shape_from_data(visual_data[0], client)

#####################################

def collision_shape_from_data(data, body, link, client=None):
    from pybullet_planning.interfaces.env_manager.pose_transformation import multiply
    from pybullet_planning.interfaces.robots.dynamics import get_joint_inertial_pose

    client = get_client(client)
    if (data.geometry_type == p.GEOM_MESH) and (data.filename == UNKNOWN_FILE):
        return -1
    pose = multiply(get_joint_inertial_pose(body, link), get_data_pose(data))
    point, quat = pose
    # TODO: the visual data seems affected by the collision data
    return p.createCollisionShape(shapeType=data.geometry_type,
                                  radius=get_data_radius(data),
                                  # halfExtents=get_data_extents(data.geometry_type, data.dimensions),
                                  halfExtents=np.array(get_data_extents(data)) / 2,
                                  height=get_data_height(data),
                                  fileName=data.filename.decode(encoding='UTF-8'),
                                  meshScale=get_data_scale(data),
                                  planeNormal=get_data_normal(data),
                                  flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
                                  collisionFramePosition=point,
                                  collisionFrameOrientation=quat,
                                  physicsClientId=client)
    # return p.createCollisionShapeArray()

def clone_collision_shape(body, link, client=None):
    from pybullet_planning.interfaces.env_manager.shape_creation import get_collision_data
    client = get_client(client)
    collision_data = get_collision_data(body, link)
    if not collision_data:
        return -1
    assert (len(collision_data) == 1)
    # TODO: can do CollisionArray
    return collision_shape_from_data(collision_data[0], body, link, client)

#####################################

def get_collision_data(body, link=BASE_LINK):
    # TODO: try catch
    return [CollisionShapeData(*tup) for tup in p.getCollisionShapeData(body, link, physicsClientId=CLIENT)]

def get_data_type(data):
    return data.geometry_type if isinstance(data, CollisionShapeData) else data.visualGeometryType

def get_data_filename(data):
    return (data.filename if isinstance(data, CollisionShapeData)
            else data.meshAssetFileName).decode(encoding='UTF-8')

def get_data_pose(data):
    if isinstance(data, CollisionShapeData):
        return (data.local_frame_pos, data.local_frame_orn)
    return (data.localVisualFrame_position, data.localVisualFrame_orientation)

def get_default_geometry():
    return {
        'halfExtents': DEFAULT_EXTENTS,
        'radius': DEFAULT_RADIUS,
        'length': DEFAULT_HEIGHT, # 'height'
        'fileName': DEFAULT_MESH,
        'meshScale': DEFAULT_SCALE,
        'planeNormal': DEFAULT_NORMAL,
    }

def get_data_extents(data):
    """
    depends on geometry type:
    for GEOM_BOX: extents,
    for GEOM_SPHERE dimensions[0] = radius,
    for GEOM_CAPSULE and GEOM_CYLINDER, dimensions[0] = height (length), dimensions[1] = radius.
    For GEOM_MESH, dimensions is the scaling factor.
    :return:
    """
    geometry_type = get_data_type(data)
    dimensions = data.dimensions
    if geometry_type == p.GEOM_BOX:
        return dimensions
    return DEFAULT_EXTENTS


def get_data_radius(data):
    geometry_type = get_data_type(data)
    dimensions = data.dimensions
    if geometry_type == p.GEOM_SPHERE:
        return dimensions[0]
    if geometry_type in (p.GEOM_CYLINDER, p.GEOM_CAPSULE):
        return dimensions[1]
    return DEFAULT_RADIUS


def get_data_height(data):
    geometry_type = get_data_type(data)
    dimensions = data.dimensions
    if geometry_type in (p.GEOM_CYLINDER, p.GEOM_CAPSULE):
        return dimensions[0]
    return DEFAULT_HEIGHT

def get_data_scale(data):
    geometry_type = get_data_type(data)
    dimensions = data.dimensions
    if geometry_type == p.GEOM_MESH:
        return dimensions
    return DEFAULT_SCALE

def get_data_normal(data):
    geometry_type = get_data_type(data)
    dimensions = data.dimensions
    if geometry_type == p.GEOM_PLANE:
        return dimensions
    return DEFAULT_NORMAL

def get_data_geometry(data):
    geometry_type = get_data_type(data)
    if geometry_type == p.GEOM_SPHERE:
        parameters = [get_data_radius(data)]
    elif geometry_type == p.GEOM_BOX:
        parameters = [get_data_extents(data)]
    elif geometry_type in (p.GEOM_CYLINDER, p.GEOM_CAPSULE):
        parameters = [get_data_height(data), get_data_radius(data)]
    elif geometry_type == p.GEOM_MESH:
        parameters = [get_data_filename(data), get_data_scale(data)]
    elif geometry_type == p.GEOM_PLANE:
        parameters = [get_data_extents(data)]
    else:
        raise ValueError(geometry_type)
    return SHAPE_TYPES[geometry_type], parameters


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

from copy import copy
import os
import numpy as np
import pybullet as p
from collections import defaultdict, namedtuple
from itertools import count

from pybullet_planning.utils import CLIENT, DEFAULT_EXTENTS, DEFAULT_HEIGHT, DEFAULT_RADIUS, \
    DEFAULT_MESH, DEFAULT_SCALE, DEFAULT_NORMAL, BASE_LINK, INFO_FROM_BODY, STATIC_MASS, UNKNOWN_FILE, BASE_LINK, NULL_ID, \
    RED, GREEN, BLUE, BLACK, GREY, CARTESIAN_TYPES
from pybullet_planning.utils import get_client, LOGGER
from pybullet_planning.interfaces.env_manager.pose_transformation import unit_pose, multiply, unit_point, unit_quat

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

# In most cases, objectUniqueId and linkIndex correspond to the parent body index and corresponding link index
# objectUniqueId and linkIndex seem to be noise when multiple meshes are attached to the same link in an URDF
# See ``test_test_geom_data_index`` in ``test_body.py`` for details.
CollisionShapeData = namedtuple('CollisionShapeData', ['objectUniqueId', 'linkIndex',
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


def create_visual_shape(geometry, pose=unit_pose(), color=RED, specular=None):
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


def create_body(collision_id=NULL_ID, visual_id=NULL_ID, mass=STATIC_MASS):
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id, physicsClientId=CLIENT)


def create_box(w, l, h, mass=STATIC_MASS, color=RED):
    """create a box body

    .. image:: ../images/box.png
        :scale: 60 %
        :align: center

    Parameters
    ----------
    w : [type]
        [description]
    l : [type]
        [description]
    h : [type]
        [description]
    mass : [type], optional
        by static_mass (0) assumes the body has infinite mass and will not be affected by gravity, by default STATIC_MASS
    color : tuple, optional
        [description], by default RED

    Returns
    -------
    int
        box body index
    """
    collision_id, visual_id = create_shape(get_box_geometry(w, l, h), color=color)
    return create_body(collision_id, visual_id, mass=mass)
    # basePosition | baseOrientation
    # linkCollisionShapeIndices | linkVisualShapeIndices


def create_cylinder(radius, height, mass=STATIC_MASS, color=BLUE):
    """create a cylinder body

    .. image:: ../images/cylinder.png
        :scale: 60 %
        :align: center

    Parameters
    ----------
    radius : [type]
        [description]
    height : [type]
        [description]
    mass : [type], optional
        [description], by default STATIC_MASS
    color : tuple, optional
        [description], by default BLUE

    Returns
    -------
    [type]
        [description]
    """
    collision_id, visual_id = create_shape(get_cylinder_geometry(radius, height), color=color)
    return create_body(collision_id, visual_id, mass=mass)


def create_capsule(radius, height, mass=STATIC_MASS, color=BLUE):
    collision_id, visual_id = create_shape(get_capsule_geometry(radius, height), color=color)
    return create_body(collision_id, visual_id, mass=mass)


def create_sphere(radius, mass=STATIC_MASS, color=BLUE):
    collision_id, visual_id = create_shape(get_sphere_geometry(radius), color=color)
    return create_body(collision_id, visual_id, mass=mass)


def create_plane(normal=[0, 0, 1], mass=STATIC_MASS, color=BLACK):
    from pybullet_planning.interfaces.robots.body import set_texture, set_color
    # color seems to be ignored in favor of a texture
    collision_id, visual_id = create_shape(get_plane_geometry(normal), color=color)
    body = create_body(collision_id, visual_id, mass=mass)
    set_texture(body, texture=None) # otherwise 'plane.urdf'
    set_color(body, color=color) # must perform after set_texture
    return body


def create_obj(path, scale=1., mass=STATIC_MASS, collision=True, color=GREY):
    """Create a body from a given mesh file. Only `.obj` and `.stl` formats are supported.

    Parameters
    ----------
    path : str
        absolute file path.
    scale : float, optional
        mesh scale, by default 1.
    mass : [type], optional
        [description], by default STATIC_MASS
    collision : bool, optional
        [description], by default True
    color : [type], optional
        [description], by default GREY

    Returns
    -------
    int
        body index
    """
    collision_id, visual_id = create_shape(get_mesh_geometry(path, scale=scale), collision=collision, color=color)
    body = create_body(collision_id, visual_id, mass=mass)
    fixed_base = (mass == STATIC_MASS)
    INFO_FROM_BODY[CLIENT, body] = ModelInfo(None, path, fixed_base, scale)  # TODO: store geometry info instead?
    return body

def create_flying_body(group, collision_id=NULL_ID, visual_id=NULL_ID, mass=STATIC_MASS):
    # TODO: more generally clone the body
    indices = list(range(len(group) + 1))
    masses = len(group) * [STATIC_MASS] + [mass]
    visuals = len(group) * [NULL_ID] + [visual_id]
    collisions = len(group) * [NULL_ID] + [collision_id]
    types = [CARTESIAN_TYPES[joint][0] for joint in group] + [p.JOINT_FIXED]
    #parents = [BASE_LINK] + indices[:-1]
    parents = indices

    assert len(indices) == len(visuals) == len(collisions) == len(types) == len(parents)
    link_positions = len(indices) * [unit_point()]
    link_orientations = len(indices) * [unit_quat()]
    inertial_positions = len(indices) * [unit_point()]
    inertial_orientations = len(indices) * [unit_quat()]
    axes = len(indices) * [unit_point()]
    axes = [CARTESIAN_TYPES[joint][1] for joint in group] + [unit_point()]
    # TODO: no way of specifying joint limits

    return p.createMultiBody(
        baseMass=STATIC_MASS,
        baseCollisionShapeIndex=NULL_ID,
        baseVisualShapeIndex=NULL_ID,
        basePosition=unit_point(),
        baseOrientation=unit_quat(),
        baseInertialFramePosition=unit_point(),
        baseInertialFrameOrientation=unit_quat(),
        linkMasses=masses,
        linkCollisionShapeIndices=collisions,
        linkVisualShapeIndices=visuals,
        linkPositions=link_positions,
        linkOrientations=link_orientations,
        linkInertialFramePositions=inertial_positions,
        linkInertialFrameOrientations=inertial_orientations,
        linkParentIndices=parents,
        linkJointTypes=types,
        linkJointAxis=axes,
        physicsClientId=CLIENT,
    )

#####################################

def vertices_from_data(data, body=None):
    """Get vertices in an object's local coordinate from its geometric data.

    Parameters
    ----------
    data : CollisionShapeData or VisualShapeData
        geometric data, see ``get_collision_data`` and ``get_visual_data``
    body : body index
        body index to help recovering vertices when geom type is mesh

    Returns
    -------
    list of vertices

    Raises
    ------
    RuntimeError
        if an unknown mesh format is encountered, we only support ``.obj`` and ``.stl`` now.
    NotImplementedError
        if an unknown pybullet geometric type is encountered. See ``SHAPE_TYPES``.
    """
    from pybullet_planning.interfaces.env_manager.pose_transformation import apply_affine
    from pybullet_planning.interfaces.env_manager.shape_creation import get_data_type, get_data_extents, get_data_radius, get_data_height, \
        get_data_filename, get_data_scale, get_collision_data, get_data_pose
    from pybullet_planning.interfaces.env_manager import get_model_info
    from pybullet_planning.interfaces.geometry.bounding_box import AABB, get_aabb_vertices

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
        import meshio
        from pybullet_planning.interfaces.geometry.mesh import read_obj
        body_index = body if body is not None else data.objectUniqueId
        filename, scale = get_data_filename(data), get_data_scale(data)
        # * load scale from cache if exists
        model_info = get_model_info(body_index)
        if model_info and not model_info.path.endswith('.urdf'):
            # ! exception handling when the data is not a link of a body parsed from a URDF file
            filename = model_info.path
            scale = model_info.scale
        if filename != UNKNOWN_FILE:
            # LOGGER.debug(f'{filename}')
            # LOGGER.debug(f'data scale {scale} info_from_body scale {model_info.scale}')
            if filename.endswith('.obj'):
                mesh = read_obj(filename, decompose=False)
                vertices = [np.array(scale)*np.array(vertex) for vertex in mesh.vertices]
                # LOGGER.debug(f'cloned from obj inside vertices_from_data | #verts {len(vertices)}')
            else:
                mio_mesh = meshio.read(filename)
                vertices = [np.array(scale)*np.array(vertex) for vertex in mio_mesh.points]
                # LOGGER.debug(f'cloned from stl inside vertices_from_data | #verts {len(vertices)}')
        else:
            try:
                #  ! this fails randomly if multiple meshes are attached to the same link
                mesh_data = p.getMeshData(data.objectUniqueId, data.linkIndex,
                    collisionShapeIndex=data.objectUniqueId, flags=p.MESH_DATA_SIMULATION_MESH)
                vertices = mesh_data[1]
                # LOGGER.debug('cloned from p.getMeshData')
            except p.error as e:
                raise RuntimeError('Unknown file from data {} | pybullet error {}'.format(data, e))
        # TODO: could compute AABB here for improved speed at the cost of being conservative
    #elif geometry_type == p.GEOM_PLANE:
    #   parameters = [get_data_extents(data)]
    else:
        raise NotImplementedError(geometry_type)
    return apply_affine(get_data_pose(data), vertices)


#####################################

def visual_shape_from_data(data_list, client=None):
    client = get_client(client)

    if len(data_list) == 1:
        data = data_list[0]
        # ! recover filename and height from data, but load from INFO_FROM_BODY when filename == UNKNOWN_FILE
        file_name, length = get_data_filename_and_height(data)
        if (get_data_type(data) == p.GEOM_MESH) and (file_name == UNKNOWN_FILE):
            LOGGER.warning('Visual shape creation from data fails due to no filename data stored in {}'.format(data))
            return NULL_ID

        # visualFramePosition: translational offset of the visual shape with respect to the link
        # visualFrameOrientation: rotational offset (quaternion x,y,z,w) of the visual shape with respect to the link frame
        #inertial_pose = get_joint_inertial_pose(data.objectUniqueId, data.linkIndex)
        #point, quat = multiply(invert(inertial_pose), pose)
        point, quat = get_data_pose(data)
        return p.createVisualShape(shapeType=data.visualGeometryType,
                                   radius=get_data_radius(data),
                                   halfExtents=np.array(get_data_extents(data))/2,
                                   length=length,  # TODO: pybullet bug
                                   fileName=file_name,
                                   meshScale=get_data_scale(data),
                                   planeNormal=get_data_normal(data),
                                   rgbaColor=data.rgbaColor,
                                   # specularColor=,
                                   visualFramePosition=point,
                                   visualFrameOrientation=quat,
                                   physicsClientId=client)
    else:
        file_names = []
        poses = []
        half_extents = [[], []]
        for data in data_list:
            file_name = get_data_filename(data)
            if (get_data_type(data) == p.GEOM_MESH) and (file_name == UNKNOWN_FILE):
                LOGGER.warning('Visual shape creation from data fails due to no filename data stored in {}'.format(data))
                return NULL_ID
            file_names.append(file_name)
            poses.append(get_data_pose(data))
            data_half_extent = np.array(get_data_extents(data)) / 2
            half_extents[0].append(data_half_extent[0])
            half_extents[1].append(data_half_extent[1])
        return p.createVisualShapeArray(shapeTypes=[data.visualGeometryType for data in data_list],
                                        radii=[get_data_radius(data) for data in data_list],
                                        halfExtents=half_extents,
                                        lengths=[get_data_height(data) for data in data_list],
                                        fileNames=file_names,
                                        meshScales=[get_data_scale(data) for data in data_list],
                                        planeNormals=[get_data_normal(data) for data in data_list],
                                        flags=[p.GEOM_FORCE_CONCAVE_TRIMESH for _ in data_list],
                                        visualFramePositions=[dpose[0] for dpose in poses],
                                        visualFrameOrientations=[dpose[1] for dpose in poses],
                                        physicsClientId=client)


def get_visual_data(body, link=BASE_LINK):
    visual_data = [VisualShapeData(*tup) for tup in p.getVisualShapeData(body, physicsClientId=CLIENT)]
    return list(filter(lambda d: d.linkIndex == link, visual_data))


def clone_visual_shape(body, link, client=None):
    client = get_client(client)
    # if not has_gui(client):
    #    return NULL_ID
    visual_data = get_visual_data(body, link)
    if not visual_data:
        return NULL_ID
    # assert (len(visual_data) == 1)
    return visual_shape_from_data(visual_data, client)

#####################################

def collision_shape_from_data(data_list, body, link, client=None):
    from pybullet_planning.interfaces.env_manager.pose_transformation import multiply
    from pybullet_planning.interfaces.robots.dynamics import get_joint_inertial_pose

    client = get_client(client)
    if len(data_list) == 1:
        data = data_list[0]
        # ! recover filename and height from data, but load from INFO_FROM_BODY when filename == UNKNOWN_FILE
        file_name, height = get_data_filename_and_height(data)
        LOGGER.debug(f'{file_name}, {height}, {get_data_type(data)}')
        if (get_data_type(data) == p.GEOM_MESH) and (file_name == UNKNOWN_FILE):
            LOGGER.warning('Collision shape creation from body #{} fails due to no filename data stored in {}'.format(
                body, link, data))
            return NULL_ID

        pose = multiply(get_joint_inertial_pose(body, link), get_data_pose(data))
        point, quat = pose
        # TODO: the visual data seems affected by the collision data
        return p.createCollisionShape(shapeType=data.geometry_type,
                                      radius=get_data_radius(data),
                                      # halfExtents=get_data_extents(data.geometry_type, data.dimensions),
                                      halfExtents=np.array(get_data_extents(data)) / 2,
                                      height=height,
                                      fileName=file_name,
                                      meshScale=get_data_scale(data),
                                      planeNormal=get_data_normal(data),
                                      flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
                                      collisionFramePosition=point,
                                      collisionFrameOrientation=quat,
                                      physicsClientId=client)
    else:
        file_names = []
        poses = []
        half_extents = [[], []]
        for data in data_list:
            file_name = get_data_filename(data)
            if (get_data_type(data) == p.GEOM_MESH) and (file_name == UNKNOWN_FILE):
                LOGGER.warning('Collision shape creation from body #{} fails due to no filename data stored in {}'.format(
                    body, link, data))
                return NULL_ID
            file_names.append(file_name)
            poses.append(multiply(get_joint_inertial_pose(body, link), get_data_pose(data)))
            data_half_extent = np.array(get_data_extents(data)) / 2
            half_extents[0].append(data_half_extent[0])
            half_extents[1].append(data_half_extent[1])
        return p.createCollisionShapeArray(shapeTypes=[data.geometry_type for data in data_list],
                                                       radii=[get_data_radius(data) for data in data_list],
                                                       halfExtents=half_extents,
                                                       lengths=[get_data_height(data) for data in data_list],
                                                       fileNames=file_names,
                                                       meshScales=[get_data_scale(data) for data in data_list],
                                                       planeNormals=[get_data_normal(data) for data in data_list],
                                                       flags=[p.GEOM_FORCE_CONCAVE_TRIMESH for _ in data_list],
                                                       collisionFramePositions=[dpose[0] for dpose in poses],
                                                       collisionFrameOrientations=[dpose[1] for dpose in poses],
                                                       physicsClientId=client)

def clone_collision_shape(body, link, client=None):
    from pybullet_planning.interfaces.env_manager.shape_creation import get_collision_data
    client = get_client(client)
    collision_data = get_collision_data(body, link)
    if not collision_data:
        return NULL_ID
    return collision_shape_from_data(collision_data, body, link, client)

#####################################

def get_collision_data(body, link=BASE_LINK):
    # TODO: try catch
    return [CollisionShapeData(*tup) for tup in p.getCollisionShapeData(body, link, physicsClientId=CLIENT)]
    # ? copy visual mesh when collision mesh is unknown
    # collision_datas = []
    # for i, tup in enumerate(p.getCollisionShapeData(body, link, physicsClientId=CLIENT)):
    #     tup_ = list(copy(tup))
    #     # * load scale from cache if exists
    #     model_info = get_model_info(body)
    #     if model_info:
    #         scale = np.ones(3) * model_info.scale
    #         tup_[3] = scale
    #     if tup_[4].decode(encoding='UTF-8') == UNKNOWN_FILE:
    #         tup_[4] = get_visual_data(body, link)[i].meshAssetFileName
    #     collision_data = CollisionShapeData(*tup_)
    #     collision_datas.append(collision_data)
    # return collision_datas

def get_data_type(data):
    return data.geometry_type if isinstance(data, CollisionShapeData) else data.visualGeometryType

def get_data_filename(data):
    return (data.filename if isinstance(data, CollisionShapeData)
            else data.meshAssetFileName).decode(encoding='UTF-8')

def get_data_filename_and_height(data, body=None):
    """load filename and scale from data, if filename is UNKNOWNFILE, we load filename and scale from the cached INFO_FROM_BODY"""
    filename = get_data_filename(data)
    if (get_data_type(data) == p.GEOM_MESH) and (filename == UNKNOWN_FILE):
        info = get_model_info(body or data.objectUniqueId)
        if info is not None and os.path.exists(info.path):
            return info.path, np.ones(3)*info.scale
    return filename, get_data_height(data)

def get_data_pose(data):
    """Get the local frame pose of the original shape.

    Parameters
    ----------
    data : CollisionShapeData
        usually the return of `get_collision_data`

    Returns
    -------
    Pose
    """
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

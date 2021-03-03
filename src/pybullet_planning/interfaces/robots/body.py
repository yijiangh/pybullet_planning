import numpy as np
from collections import namedtuple
import pybullet as p

from pybullet_planning.utils import get_client, INFO_FROM_BODY, STATIC_MASS, BASE_LINK, OBJ_MESH_CACHE, NULL_ID
from pybullet_planning.utils import implies
from pybullet_planning.interfaces.env_manager.pose_transformation import Pose, Point, Euler
from pybullet_planning.interfaces.env_manager.pose_transformation import euler_from_quat, base_values_from_pose, \
    quat_from_euler, z_rotation, get_pose, set_pose, apply_affine, unit_pose
from pybullet_planning.interfaces.env_manager.shape_creation import get_collision_data, clone_collision_shape, clone_visual_shape

from pybullet_planning.interfaces.robots.dynamics import get_mass, get_dynamics_info, get_local_link_pose
from pybullet_planning.interfaces.robots.joint import JOINT_TYPES, get_joint_name, get_joint_type, is_circular, get_joint_limits, is_fixed, \
    get_joint_info, get_joint_positions, get_joints, is_movable
from pybullet_planning.interfaces.robots.link import get_links, parent_joint_from_link, get_link_name, get_link_parent, get_link_pose, \
    get_link_subtree, get_all_links

#####################################
# Bodies

def get_bodies():
    return [p.getBodyUniqueId(i, physicsClientId=get_client())
            for i in range(p.getNumBodies(physicsClientId=get_client()))]

BodyInfo = namedtuple('BodyInfo', ['base_name', 'body_name'])

def get_body_info(body):
    return BodyInfo(*p.getBodyInfo(body, physicsClientId=get_client()))

def get_base_name(body):
    return get_body_info(body).base_name.decode(encoding='UTF-8')

def get_body_name(body):
    return get_body_info(body).body_name.decode(encoding='UTF-8')

def get_name(body):
    name = get_body_name(body)
    if name == '':
        name = 'body'
        return '{}{}'.format(name, int(body))
    else:
        return name

def has_body(name):
    try:
        body_from_name(name)
    except ValueError:
        return False
    return True

def body_from_name(name):
    for body in get_bodies():
        if get_body_name(body) == name:
            return body
    raise ValueError(name)

def remove_body(body):
    if (get_client(), body) in INFO_FROM_BODY:
        del INFO_FROM_BODY[get_client(), body]
    return p.removeBody(body, physicsClientId=get_client())

def get_point(body):
    return get_pose(body)[0]

def get_quat(body):
    return get_pose(body)[1] # [x,y,z,w]

def get_euler(body):
    return euler_from_quat(get_quat(body))

def get_base_values(body):
    return base_values_from_pose(get_pose(body))

def set_point(body, point):
    set_pose(body, (point, get_quat(body)))

def set_quat(body, quat):
    set_pose(body, (get_point(body), quat))

def set_euler(body, euler):
    set_quat(body, quat_from_euler(euler))

def pose_from_pose2d(pose2d):
    x, y, theta = pose2d
    return Pose(Point(x=x, y=y), Euler(yaw=theta))

def set_base_values(body, values):
    _, _, z = get_point(body)
    x, y, theta = values
    set_point(body, (x, y, z))
    set_quat(body, z_rotation(theta))

def get_velocity(body):
    linear, angular = p.getBaseVelocity(body, physicsClientId=get_client())
    return linear, angular # [x,y,z], [wx,wy,wz]

def set_velocity(body, linear=None, angular=None):
    if linear is not None:
        p.resetBaseVelocity(body, linearVelocity=linear, physicsClientId=get_client())
    if angular is not None:
        p.resetBaseVelocity(body, angularVelocity=angular, physicsClientId=get_client())

def is_rigid_body(body):
    for joint in get_joints(body):
        if is_movable(body, joint):
            return False
    return True

def is_fixed_base(body):
    return get_mass(body) == STATIC_MASS

def dump_body(body, fixed=False):
    print('Body id: {} | Name: {} | Rigid: {} | Fixed: {}'.format(
        body, get_body_name(body), is_rigid_body(body), is_fixed_base(body)))
    for joint in get_joints(body):
        if fixed or is_movable(body, joint):
            print('Joint id: {} | Name: {} | Type: {} | Circular: {} | Limits: {}'.format(
                joint, get_joint_name(body, joint), JOINT_TYPES[get_joint_type(body, joint)],
                is_circular(body, joint), get_joint_limits(body, joint)))
    link = NULL_ID
    print('Link id: {} | Name: {} | Mass: {} | Collision: {} | Visual: {}'.format(
        link, get_base_name(body), get_mass(body),
        len(get_collision_data(body, link)), NULL_ID)) # len(get_visual_data(body, link))))
    for link in get_links(body):
        joint = parent_joint_from_link(link)
        joint_name = JOINT_TYPES[get_joint_type(body, joint)] if is_fixed(body, joint) else get_joint_name(body, joint)
        print('Link id: {} | Name: {} | Joint: {} | Parent: {} | Mass: {} | Collision: {} | Visual: {}'.format(
            link, get_link_name(body, link), joint_name,
            get_link_name(body, get_link_parent(body, link)), get_mass(body, link),
            len(get_collision_data(body, link)), NULL_ID)) # len(get_visual_data(body, link))))
        #print(get_joint_parent_frame(body, link))
        #print(map(get_data_geometry, get_visual_data(body, link)))
        #print(map(get_data_geometry, get_collision_data(body, link)))

def dump_world():
    """print out information for all bodies that're currently in the scene

    """
    for body in get_bodies():
        dump_body(body)
        print()

def clone_body(body, links=None, collision=True, visual=True, client=None):
    from pybullet_planning.utils import get_client
    # TODO: names are not retained
    # TODO: error with createMultiBody link poses on PR2
    # localVisualFrame_position: position of local visual frame, relative to link/joint frame
    # localVisualFrame orientation: orientation of local visual frame relative to link/joint frame
    # parentFramePos: joint position in parent frame
    # parentFrameOrn: joint orientation in parent frame
    client = get_client(client) # client is the new client for the body
    if links is None:
        links = get_links(body)
    #movable_joints = [joint for joint in links if is_movable(body, joint)]
    new_from_original = {}
    base_link = get_link_parent(body, links[0]) if links else BASE_LINK
    new_from_original[base_link] = NULL_ID

    masses = []
    collision_shapes = []
    visual_shapes = []
    positions = [] # list of local link positions, with respect to parent
    orientations = [] # list of local link orientations, w.r.t. parent
    inertial_positions = [] # list of local inertial frame pos. in link frame
    inertial_orientations = [] # list of local inertial frame orn. in link frame
    parent_indices = []
    joint_types = []
    joint_axes = []
    for i, link in enumerate(links):
        new_from_original[link] = i
        joint_info = get_joint_info(body, link)
        dynamics_info = get_dynamics_info(body, link)
        masses.append(dynamics_info.mass)
        collision_shapes.append(clone_collision_shape(body, link, client) if collision else NULL_ID)
        visual_shapes.append(clone_visual_shape(body, link, client) if visual else NULL_ID)
        point, quat = get_local_link_pose(body, link)
        positions.append(point)
        orientations.append(quat)
        inertial_positions.append(dynamics_info.local_inertial_pos)
        inertial_orientations.append(dynamics_info.local_inertial_orn)
        parent_indices.append(new_from_original[joint_info.parentIndex] + 1) # TODO: need the increment to work
        joint_types.append(joint_info.jointType)
        joint_axes.append(joint_info.jointAxis)
    # https://github.com/bulletphysics/bullet3/blob/9c9ac6cba8118544808889664326fd6f06d9eeba/examples/pybullet/gym/pybullet_utils/urdfEditor.py#L169

    base_dynamics_info = get_dynamics_info(body, base_link)
    base_point, base_quat = get_link_pose(body, base_link)
    new_body = p.createMultiBody(baseMass=base_dynamics_info.mass,
                                 baseCollisionShapeIndex=clone_collision_shape(body, base_link, client) if collision else NULL_ID,
                                 baseVisualShapeIndex=clone_visual_shape(body, base_link, client) if visual else NULL_ID,
                                 basePosition=base_point,
                                 baseOrientation=base_quat,
                                 baseInertialFramePosition=base_dynamics_info.local_inertial_pos,
                                 baseInertialFrameOrientation=base_dynamics_info.local_inertial_orn,
                                 linkMasses=masses,
                                 linkCollisionShapeIndices=collision_shapes,
                                 linkVisualShapeIndices=visual_shapes,
                                 linkPositions=positions,
                                 linkOrientations=orientations,
                                 linkInertialFramePositions=inertial_positions,
                                 linkInertialFrameOrientations=inertial_orientations,
                                 linkParentIndices=parent_indices,
                                 linkJointTypes=joint_types,
                                 linkJointAxis=joint_axes,
                                 physicsClientId=client)
    #set_configuration(new_body, get_joint_positions(body, movable_joints)) # Need to use correct client
    for joint, value in zip(range(len(links)), get_joint_positions(body, links)):
        # TODO: check if movable?
        p.resetJointState(new_body, joint, value, targetVelocity=0, physicsClientId=client)
    return new_body

def clone_world(client=None, exclude=[]):
    from pybullet_planning.interfaces.env_manager.simulation import has_gui
    visual = has_gui(client)
    mapping = {}
    for body in get_bodies():
        if body not in exclude:
            new_body = clone_body(body, collision=True, visual=visual, client=client)
            mapping[body] = new_body
    return mapping

def set_color(body, color, link=BASE_LINK, shape_index=NULL_ID):
    """
    Experimental for internal use, recommended ignore shapeIndex or leave it NULL_ID.
    Intention was to let you pick a specific shape index to modify,
    since URDF (and SDF etc) can have more than 1 visual shape per link.
    This shapeIndex matches the list ordering returned by getVisualShapeData.
    :param body:
    :param link:
    :param shape_index:
    :return:
    """
    # specularColor
    return p.changeVisualShape(body, link, shapeIndex=shape_index, rgbaColor=color,
                               #textureUniqueId=None, specularColor=None,
                               physicsClientId=get_client())

def set_texture(body, texture=None, link=BASE_LINK, shape_index=NULL_ID):
    if texture is None:
        texture = NULL_ID
    return p.changeVisualShape(body, link, shapeIndex=shape_index, textureUniqueId=texture,
                               physicsClientId=get_client())

def vertices_from_link(body, link):
    from pybullet_planning.interfaces.env_manager.shape_creation import vertices_from_data
    # In local frame
    vertices = []
    # TODO: requires the viewer to be active
    #for data in get_visual_data(body, link):
    #    vertices.extend(vertices_from_data(data))
    # Pybullet creates multiple collision elements (with unknown_file) when noncovex
    for data in get_collision_data(body, link):
        vertices.extend(vertices_from_data(data))
    return vertices

def vertices_from_rigid(body, link=BASE_LINK):
    """compute all vertices of given body

    Parameters
    ----------
    body : int
        [description]
    link : [type], optional
        if BASE_LINK, we assume the body is single-linked, by default BASE_LINK

    Returns
    -------
    list of three-float lists
        body vertices

    Raises
    ------
    NotImplementedError
        only supports body created from a `.obj` file
    """
    import os
    from pybullet_planning.interfaces.env_manager import get_model_info
    from pybullet_planning.interfaces.geometry.mesh import read_obj
    from pybullet_planning.interfaces.robots.link import get_num_links
    assert implies(link == BASE_LINK, get_num_links(body) == 0)
    try:
        vertices = vertices_from_link(body, link)
    except RuntimeError:
        info = get_model_info(body)
        assert info is not None
        _, ext = os.path.splitext(info.path)
        if ext == '.obj':
            if info.path not in OBJ_MESH_CACHE:
                OBJ_MESH_CACHE[info.path] = read_obj(info.path, decompose=False)
            mesh = OBJ_MESH_CACHE[info.path]
            vertices = [[v[i]*info.scale for i in range(3)] for v in mesh.vertices]
        else:
            raise NotImplementedError(ext)
    return vertices

def approximate_as_prism(body, body_pose=unit_pose(), **kwargs):
    """get the AABB bounding box of a body

    Note: the generated AABB is not truly axis-aligned, bounding box under world axis x, y

    Parameters
    ----------
    body : int
        pb body's index
    body_pose : [type], optional
        [description], by default unit_pose()

    Returns
    -------
    tuple of (Point, float)
        bounding box center and extent length
    """
    from pybullet_planning.interfaces.geometry.bounding_box import aabb_from_points, get_aabb_center, get_aabb_extent
    # TODO: make it just orientation
    vertices = apply_affine(body_pose, vertices_from_rigid(body, **kwargs))
    aabb = aabb_from_points(vertices)
    return get_aabb_center(aabb), get_aabb_extent(aabb)
    #with PoseSaver(body):
    #    set_pose(body, body_pose)
    #    set_velocity(body, linear=np.zeros(3), angular=np.zeros(3))
    #    return get_center_extent(body, **kwargs)

def approximate_as_cylinder(body, **kwargs):
    """get the bounding cylinder of the AABB bounding box of a body

    Note: the generated AABB is not truly axis-aligned, bounding box under world axis x, y.
    Thus, the estimated diameter and height might not be accurate in the longitude axis aligning sense.

    Parameters
    ----------
    body : int
        [description]

    Returns
    -------
    Point, tuple of (diameter, height)
        [description]
    """
    center, (width, length, height) = approximate_as_prism(body, **kwargs)
    diameter = np.sqrt(width**2 + length**2) # TODO: check that these are close
    diameter = (width + length) / 2 # TODO: check that these are close
    return center, (diameter, height)

#####################################

def load_model(rel_path, pose=None, **kwargs):
    from pybullet_planning.interfaces.env_manager.simulation import get_model_path, add_data_path, load_pybullet
    # TODO: error with loadURDF when loading MESH visual and CYLINDER collision
    abs_path = get_model_path(rel_path)
    add_data_path()
    #with LockRenderer():
    body = load_pybullet(abs_path, **kwargs)
    if pose is not None:
        set_pose(body, pose)
    return body

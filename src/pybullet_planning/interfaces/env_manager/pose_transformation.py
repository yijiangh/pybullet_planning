import math
import numpy as np
import pybullet as p

from pybullet_planning.utils import CLIENT, unit_vector, quaternion_from_matrix, clip, euler_from_quaternion

#####################################
# Geometry

#Pose = namedtuple('Pose', ['position', 'orientation'])

def Point(x=0., y=0., z=0.):
    """Representing a point in 3D

    Parameters
    ----------
    x : float, optional
        [description], by default 0.
    y : float, optional
        [description], by default 0.
    z : float, optional
        [description], by default 0.

    Returns
    -------
    np array of three floats
        [description]
    """
    return np.array([x, y, z])

def Euler(roll=0., pitch=0., yaw=0.):
    """Representing a 3D rotation by Eulerian angles

    .. image:: ../images/roll_pitch_yaw.png
        :scale: 60 %
        :align: center

    `image source <https://devforum.roblox.com/t/take-out-pitch-from-rotation-matrix-while-preserving-yaw-and-roll/95204>`_

    Parameters
    ----------
    roll : float, optional
        [description], by default 0.
    pitch : float, optional
        [description], by default 0.
    yaw : float, optional
        [description], by default 0.

    Returns
    -------
    np array of three floats
        [description]
    """
    return np.array([roll, pitch, yaw])

def Pose(point=None, euler=None):
    """Representing a pose (or frame) in 3D

    Parameters
    ----------
    point : np array of three-floats, optional
        [description], by default None
    euler : np array of three eulerian angles, optional
        (roll, pitch, yaw), by default None

    Returns
    -------
    tuple of point, quaternion
        [description]
    """
    point = Point() if point is None else point
    euler = Euler() if euler is None else euler
    return (point, quat_from_euler(euler))

#def Pose2d(x=0., y=0., yaw=0.):
#    return np.array([x, y, yaw])

#####################################

def invert(pose):
    (point, quat) = pose
    return p.invertTransform(point, quat)

def multiply(*poses):
    pose = poses[0]
    for next_pose in poses[1:]:
        pose = p.multiplyTransforms(pose[0], pose[1], *next_pose)
    return pose

def invert_quat(quat):
    pose = (unit_point(), quat)
    return quat_from_pose(invert(pose))

def multiply_quats(*quats):
    return quat_from_pose(multiply(*[(unit_point(), quat) for quat in quats]))

def unit_from_theta(theta):
    return np.array([np.cos(theta), np.sin(theta)])

def quat_from_euler(euler):
    return p.getQuaternionFromEuler(euler) # TODO: extrinsic (static) vs intrinsic (rotating)

def euler_from_quat(quat):
    return p.getEulerFromQuaternion(quat) # rotation around fixed axis

def intrinsic_euler_from_quat(quat):
    #axes = 'sxyz' if static else 'rxyz'
    return euler_from_quaternion(quat, axes='rxyz')

def unit_point():
    return (0., 0., 0.)

def unit_quat():
    return quat_from_euler([0, 0, 0]) # [X,Y,Z,W]

def quat_from_axis_angle(axis, angle): # axis-angle
    #return get_unit_vector(np.append(vec, [angle]))
    return np.append(math.sin(angle/2) * get_unit_vector(axis), [math.cos(angle / 2)])

def unit_pose():
    return (unit_point(), unit_quat())

def get_length(vec, norm=2):
    return np.linalg.norm(vec, ord=norm)

def get_delta(q1, q2):
    return np.array(q2) - np.array(q1)

def get_difference(p1, p2):
    return np.array(p2) - np.array(p1)

def get_distance(p1, p2, **kwargs):
    return get_length(get_difference(p1, p2), **kwargs)

def angle_between(vec1, vec2):
    return np.math.acos(np.dot(vec1, vec2) / (get_length(vec1) *  get_length(vec2)))

def get_angle(q1, q2):
    dx, dy = np.array(q2[:2]) - np.array(q1[:2])
    return np.math.atan2(dy, dx)

def get_unit_vector(vec):
    norm = get_length(vec)
    if norm == 0:
        return vec
    return np.array(vec) / norm

def z_rotation(theta):
    return quat_from_euler([0, 0, theta])

def matrix_from_quat(quat):
    return np.array(p.getMatrixFromQuaternion(quat, physicsClientId=CLIENT)).reshape(3, 3)

def quat_from_matrix(mat):
    matrix = np.eye(4)
    matrix[:3,:3] = mat
    return quaternion_from_matrix(matrix)

def point_from_tform(tform):
    return np.array(tform)[:3,3]

def matrix_from_tform(tform):
    return np.array(tform)[:3,:3]

def point_from_pose(pose):
    """get the origin point from a pose

    Parameters
    ----------
    pose : Pose
        [description]

    Returns
    -------
    Point, np array of three floats
        [description]
    """
    return pose[0]

def quat_from_pose(pose):
    """get the quaternion from a pose

    Parameters
    ----------
    pose : [type]
        [description]

    Returns
    -------
    Quaternion, np array of four floats
        [description]
    """
    return pose[1]

def tform_from_pose(pose):
    (point, quat) = pose
    tform = np.eye(4)
    tform[:3,3] = point
    tform[:3,:3] = matrix_from_quat(quat)
    return tform

def pose_from_tform(tform):
    return point_from_tform(tform), quat_from_matrix(matrix_from_tform(tform))

def wrap_angle(theta, lower=-np.pi): # [-np.pi, np.pi)
    return (theta - lower) % (2 * np.pi) + lower

def circular_difference(theta2, theta1):
    return wrap_angle(theta2 - theta1)

def base_values_from_pose(pose, tolerance=1e-3):
    (point, quat) = pose
    x, y, _ = point
    roll, pitch, yaw = euler_from_quat(quat)
    assert (abs(roll) < tolerance) and (abs(pitch) < tolerance)
    return (x, y, yaw)

pose2d_from_pose = base_values_from_pose

def pose_from_base_values(base_values, default_pose=unit_pose()):
    x, y, yaw = base_values
    _, _, z = point_from_pose(default_pose)
    roll, pitch, _ = euler_from_quat(quat_from_pose(default_pose))
    return (x, y, z), quat_from_euler([roll, pitch, yaw])

def quat_angle_between(quat0, quat1): # quaternion_slerp
    #p.computeViewMatrixFromYawPitchRoll()
    q0 = unit_vector(quat0[:4])
    q1 = unit_vector(quat1[:4])
    d = clip(np.dot(q0, q1), min_value=-1., max_value=+1.)
    angle = math.acos(d)
    # TODO: angle_between
    #delta = p.getDifferenceQuaternion(quat0, quat1)
    #angle = math.acos(delta[-1])
    return angle

def all_between(lower_limits, values, upper_limits):
    assert len(lower_limits) == len(values), 'lower limit {}, values {}'.format(lower_limits, values)
    assert len(values) == len(upper_limits), 'upper limit {}, values {}'.format(upper_limits, values)
    return np.less_equal(lower_limits, values).all() and \
           np.less_equal(values, upper_limits).all()

def convex_combination(x, y, w=0.5):
    return (1-w)*np.array(x) + w*np.array(y)

def tform_point(affine, point):
    """transform a given point

    Parameters
    ----------
    affine : Pose
        [description]
    point : [type]
        [description]

    Returns
    -------
    Point
        [description]
    """
    return point_from_pose(multiply(affine, Pose(point=point)))

def apply_affine(affine, points):
    """apply affine transformation on the given list of points

    Parameters
    ----------
    affine : Pose
        [description]
    points : [type]
        [description]

    Returns
    -------
    list of points
        [description]
    """
    return [tform_point(affine, p) for p in points]

##############################################
# placing here to resolve cycly dependencies
# more can be found in inferfaces.robots.body

def set_pose(body, pose):
    (point, quat) = pose
    p.resetBasePositionAndOrientation(body, point, quat, physicsClientId=CLIENT)

def get_pose(body):
    return p.getBasePositionAndOrientation(body, physicsClientId=CLIENT)
    #return np.concatenate([point, quat])

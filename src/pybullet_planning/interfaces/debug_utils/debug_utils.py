import math
import numpy as np
import pybullet as p
from itertools import product, combinations

from pybullet_planning.utils import CLIENT, BASE_LINK, GREEN, RED, BLUE, BLACK, WHITE, NULL_ID, YELLOW

from pybullet_planning.interfaces.env_manager.pose_transformation import unit_pose, tform_point, unit_from_theta, get_distance
from pybullet_planning.interfaces.geometry.bounding_box import get_aabb
from pybullet_planning.interfaces.geometry.camera import apply_alpha, set_camera_pose

def get_lifetime(lifetime):
    if lifetime is None:
        return 0
    return lifetime

def add_debug_parameter():
    # TODO: make a slider that controls the step in the trajectory
    # TODO: could store a list of savers
    #targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -10, 10, 0)
    #maxForce = p.readUserDebugParameter(maxForceSlider)
    raise NotImplementedError()

def add_text(text, position=(0, 0, 0), color=BLACK, lifetime=None, parent=NULL_ID, parent_link=BASE_LINK):
    return p.addUserDebugText(str(text), textPosition=position, textColorRGB=color[:3], # textSize=1,
                              lifeTime=get_lifetime(lifetime), parentObjectUniqueId=parent, parentLinkIndex=parent_link,
                              physicsClientId=CLIENT)

def add_line(start, end, color=BLACK, width=1, lifetime=None, parent=NULL_ID, parent_link=BASE_LINK):
    """[summary]

    Parameters
    ----------
    start : [type]
        [description]
    end : [type]
        [description]
    color : tuple, optional
        [description], by default (0, 0, 0)
    width : int, optional
        [description], by default 1
    lifetime : [type], optional
        [description], by default None
    parent : int, optional
        [description], by default NULL_ID
    parent_link : [type], optional
        [description], by default BASE_LINK

    Returns
    -------
    [type]
        [description]
    """
    return p.addUserDebugLine(start, end, lineColorRGB=color[:3], lineWidth=width,
                              lifeTime=get_lifetime(lifetime), parentObjectUniqueId=parent, parentLinkIndex=parent_link,
                              physicsClientId=CLIENT)

def remove_debug(debug):
    p.removeUserDebugItem(debug, physicsClientId=CLIENT)

remove_handle = remove_debug

def remove_handles(handles):
    for handle in handles:
        remove_debug(handle)

def remove_all_debug():
    p.removeAllUserDebugItems(physicsClientId=CLIENT)

def add_body_name(body, name=None, **kwargs):
    from pybullet_planning.interfaces.env_manager.pose_transformation import set_pose
    from pybullet_planning.interfaces.env_manager.savers import PoseSaver
    from pybullet_planning.interfaces.robots.body import get_name

    name = name or get_name(body)
    with PoseSaver(body):
        set_pose(body, unit_pose())
        lower, upper = get_aabb(body)
    #position = (0, 0, upper[2])
    position = upper
    return add_text(name, position=position, parent=body, **kwargs)  # removeUserDebugItem

def add_segments(points, closed=False, **kwargs):
    lines = []
    for v1, v2 in zip(points, points[1:]):
        lines.append(add_line(v1, v2, **kwargs))
    if closed:
        lines.append(add_line(points[-1], points[0], **kwargs))
    return lines

def draw_link_name(body, link=BASE_LINK, name=None):
    from pybullet_planning.interfaces.robots.link import get_link_name
    return add_text(name or get_link_name(body, link), position=(0, 0.2, 0),
                    parent=body, parent_link=link)

def draw_pose(pose, length=0.1, **kwargs):
    origin_world = tform_point(pose, np.zeros(3))
    handles = []
    for k in range(3):
        axis = np.zeros(3)
        axis[k] = 1
        axis_world = tform_point(pose, length*axis)
        handles.append(add_line(origin_world, axis_world, color=axis, **kwargs))
    return handles

def draw_base_limits(limits, z=1e-2, **kwargs):
    lower, upper = limits
    vertices = [(lower[0], lower[1], z), (lower[0], upper[1], z),
                (upper[0], upper[1], z), (upper[0], lower[1], z)]
    return add_segments(vertices, closed=True, **kwargs)

def draw_circle(center, radius, n=24, **kwargs):
    vertices = []
    for i in range(n):
        theta = i*2*math.pi/n
        unit = np.append(unit_from_theta(theta), [0])
        vertices.append(center+radius*unit)
    return add_segments(vertices, closed=True, **kwargs)

def draw_aabb(aabb, **kwargs):
    d = len(aabb[0])
    vertices = list(product(range(len(aabb)), repeat=d))
    lines = []
    for i1, i2 in combinations(vertices, 2):
        if sum(i1[k] != i2[k] for k in range(d)) == 1:
            p1 = [aabb[i1[k]][k] for k in range(d)]
            p2 = [aabb[i2[k]][k] for k in range(d)]
            lines.append(add_line(p1, p2, **kwargs))
    return lines

def draw_point(point, size=0.01, **kwargs):
    lines = []
    for i in range(len(point)):
        axis = np.zeros(len(point))
        axis[i] = 1.0
        p1 = np.array(point) - size/2 * axis
        p2 = np.array(point) + size/2 * axis
        lines.append(add_line(p1, p2, **kwargs))
    return lines
    #extent = size * np.ones(len(point)) / 2
    #aabb = np.array(point) - extent, np.array(point) + extent
    #return draw_aabb(aabb, **kwargs)

def get_face_edges(face):
    #return list(combinations(face, 2))
    return list(zip(face, face[1:] + face[:1]))

def draw_mesh(mesh, **kwargs):
    verts, faces = mesh
    lines = []
    for face in faces:
        for i1, i2 in get_face_edges(face):
            lines.append(add_line(verts[i1], verts[i2], **kwargs))
    return lines

def draw_ray(ray, ray_result=None, visible_color=GREEN, occluded_color=RED, **kwargs):
    if ray_result is None:
        return [add_line(ray.start, ray.end, color=visible_color, **kwargs)]
    if ray_result.objectUniqueId == NULL_ID:
        hit_position = ray.end
    else:
        hit_position = ray_result.hit_position
    return [
        add_line(ray.start, hit_position, color=visible_color, **kwargs),
        add_line(hit_position, ray.end, color=occluded_color, **kwargs),
    ]


def get_body_from_pb_id(i):
    return p.getBodyUniqueId(i, physicsClientId=CLIENT)

def draw_collision_diagnosis(pb_closest_pt_output, viz_last_duration=-1, point_color=BLACK, line_color=YELLOW, \
    focus_camera=True, camera_ray=np.array([0.1, 0, 0.05]), body_name_from_id=None, viz_all=False):
    """[summary]

    Parameters
    ----------
    pb_closest_pt_output : [type]
        [description]
    """
    from pybullet_planning.interfaces.env_manager.simulation import has_gui
    from pybullet_planning.interfaces.env_manager.user_io import HideOutput
    from pybullet_planning.interfaces.env_manager.user_io import wait_for_user, wait_for_duration
    from pybullet_planning.interfaces.robots.link import get_link_name, get_links
    from pybullet_planning.interfaces.robots.body import set_color, remove_body, clone_body, get_name
    if not pb_closest_pt_output:
        return

    body_name_from_id = body_name_from_id or {}
    # if paint_all_others:
    #     set_all_bodies_color()
        # for b in obstacles:
        #     set_color(b, (0,0,1,0.3))
    for u_cr in pb_closest_pt_output:
        handles = []
        b1 = get_body_from_pb_id(u_cr[1])
        b2 = get_body_from_pb_id(u_cr[2])
        l1 = u_cr[3]
        l2 = u_cr[4]
        b1_name = body_name_from_id[b1] if b1 in body_name_from_id else get_name(b1)
        b2_name = body_name_from_id[b2] if b2 in body_name_from_id else get_name(b2)
        l1_name = get_link_name(b1, l1)
        l2_name = get_link_name(b2, l2)

        print('*'*10)
        print('pairwise link collision: (Body #{0}, Link #{1}) - (Body #{2}, Link #{3})'.format(
            b1_name, l1_name, b2_name, l2_name))
        print('Penetration depth: {:.6f} (m) | point1 ({:.6f},{:.6f},{:.6f}), point2 ({:.6f},{:.6f},{:.6f})'.format(
            get_distance(u_cr[5], u_cr[6]), *u_cr[5], *u_cr[6]))

        if has_gui():
            clone1_fail = False
            clone2_fail = False
            try:
                with HideOutput():
                    cloned_body1 = clone_body(b1, links=[l1] if get_links(b1) else None, collision=True, visual=False)
            except:
                # print('cloning (body #{}, link #{}) fails.'.format(b1_name, l1_name))
                clone1_fail = True
                cloned_body1 = b1
            try:
                with HideOutput():
                    cloned_body2 = clone_body(b2, links=[l2] if get_links(b2) else None, collision=True, visual=False)
            except:
                # print('cloning (body #{}, link #{}) fails.'.format(b2_name, l2_name))
                clone2_fail = True
                cloned_body2 = b2

            set_color(cloned_body1, apply_alpha(RED, 0.2))
            set_color(cloned_body2, apply_alpha(GREEN, 0.2))

            handles.append(add_body_name(b1, name=b1_name))
            handles.append(add_body_name(b2, name=b2_name))
            handles.append(draw_link_name(b1, l1, name=l1_name))
            handles.append(draw_link_name(b2, l2, name=l2_name))

            handles.append(add_line(u_cr[5], u_cr[6], color=line_color, width=5))
            handles.extend(draw_point(u_cr[5], size=0.002, color=point_color))
            handles.extend(draw_point(u_cr[6], size=0.002, color=point_color))
            if focus_camera:
                camera_base_pt = u_cr[5]
                camera_pt = np.array(camera_base_pt) + camera_ray
                set_camera_pose(tuple(camera_pt), camera_base_pt)

            if viz_last_duration < 0:
                wait_for_user('Visualize collision. Press Enter to continue.')
            else:
                wait_for_duration(viz_last_duration)

            # restore lines and colors
            remove_handles(handles)
            if not clone1_fail :
                remove_body(cloned_body1)
            else:
                set_color(b1, apply_alpha(WHITE, 0.5))
            if not clone2_fail :
                remove_body(cloned_body2)
            else:
                set_color(b2, apply_alpha(WHITE, 0.5))
        else:
            wait_for_user('Collision diagnosis. Press Enter to continue.')

        if not viz_all:
            return

def draw_ray_result_diagnosis(ray, ray_result, b1=None, l1=None, point_color=BLACK, line_color=YELLOW, \
    focus_camera=True, camera_ray=np.array([0.1, 0, 0.05]), body_name_from_id=None):
    """[summary]

    Parameters
    ----------
    pb_closest_pt_output : [type]
        [description]
    """
    from pybullet_planning.interfaces.env_manager.simulation import has_gui
    from pybullet_planning.interfaces.env_manager.user_io import HideOutput
    from pybullet_planning.interfaces.env_manager.user_io import wait_for_user, wait_for_duration
    from pybullet_planning.interfaces.robots.link import get_link_name, get_links
    from pybullet_planning.interfaces.robots.body import set_color, remove_body, clone_body, get_name
    if not ray_result:
        return
    body_name_from_id = body_name_from_id or {}

    handles = []
    b1_name = body_name_from_id[b1] if b1 is not None and b1 in body_name_from_id else None
    l1_name = get_link_name(b1, l1) if l1 is not None and b1 is not None else None

    b2 = ray_result.objectUniqueId
    l2 = ray_result.linkIndex
    b2_name = body_name_from_id[b2] if b2 in body_name_from_id else get_name(b2)
    l2_name = get_link_name(b2, l2)

    print('*'*10)
    print('ray collision: (Body #{0}, Link #{1}) - (Body #{2}, Link #{3})'.format(
        b1_name, l1_name, b2_name, l2_name))
    print('hit_fraction: {:.2f} | hit_position ({:.6f},{:.6f},{:.6f}) | hit_normal ({:.6f},{:.6f},{:.6f})'.format(
        ray_result.hit_fraction, *ray_result.hit_position, *ray_result.hit_normal))

    if has_gui():
        clone1_fail = False
        if b1 is not None and l1 is not None:
            try:
                with HideOutput():
                    cloned_body1 = clone_body(b1, links=[l1] if get_links(b1) else None, collision=True, visual=False)
            except:
                clone1_fail = True
                cloned_body1 = b1
        try:
            with HideOutput():
                cloned_body2 = clone_body(b2, links=[l2] if get_links(b2) else None, collision=True, visual=False)
        except:
            clone2_fail = True
            cloned_body2 = b2

        # set_color(cloned_body1, apply_alpha(RED, 0.2))
        set_color(cloned_body2, apply_alpha(RED, 0.2))

        if b1 is not None and l1 is not None:
            handles.append(add_body_name(b1, name=b1_name))
            handles.append(draw_link_name(b1, l1, name=l1_name))
        handles.append(add_body_name(b2, name=b2_name))
        handles.append(draw_link_name(b2, l2, name=l2_name))

        # handles.append(add_line(u_cr[5], u_cr[6], color=line_color, width=5))
        handles.extend(draw_ray(ray, ray_result, width=3.0))
        handles.extend(draw_point(ray_result.hit_position, size=0.002, color=point_color))

        if focus_camera:
            camera_base_pt = ray_result.hit_position
            camera_pt = np.array(camera_base_pt) + camera_ray
            set_camera_pose(tuple(camera_pt), camera_base_pt)

        wait_for_user('Visualize collision. Press Enter to continue.')

        # restore lines and colors
        remove_handles(handles)
        if b1 is not None and l1 is not None:
            if not clone1_fail :
                remove_body(cloned_body1)
            else:
                set_color(b1, apply_alpha(WHITE, 0.5))
        if not clone2_fail :
            remove_body(cloned_body2)
        else:
            set_color(b2, apply_alpha(WHITE, 0.5))
    else:
        wait_for_user('Ray collision diagnosis. Press Enter to continue.')


def camera_focus_on_body(body, camera_ray=np.array([0.1, 0, 0.05])):
    from pybullet_planning.interfaces.env_manager.pose_transformation import get_pose
    camera_base_pt, _ = get_pose(body)
    camera_pt = np.array(camera_base_pt) + camera_ray
    set_camera_pose(tuple(camera_pt), camera_base_pt)


def camera_focus_on_point(point, camera_ray=np.array([0.1, 0, 0.05])):
    camera_pt = np.array(point) + camera_ray
    set_camera_pose(tuple(camera_pt), point)

import warnings
from collections import namedtuple
from itertools import product
import numpy as np
import pybullet as p

from pybullet_planning.utils import CLIENT, BASE_LINK, MAX_DISTANCE, UNKNOWN_FILE
from pybullet_planning.interfaces.env_manager.user_io import step_simulation
from pybullet_planning.interfaces.env_manager.pose_transformation import get_distance
from .link import get_all_links
from .body import get_bodies

#####################################

def contact_collision():
    """run simulation for one step and check if there is a collision

    Returns
    -------
    bool
        True if there is a collision, False otherwise
    """
    step_simulation()
    return len(p.getContactPoints(physicsClientId=CLIENT)) != 0


ContactResult = namedtuple('ContactResult', ['contactFlag', 'bodyUniqueIdA', 'bodyUniqueIdB',
                                         'linkIndexA', 'linkIndexB', 'positionOnA', 'positionOnB',
                                         'contactNormalOnB', 'contactDistance', 'normalForce'])


def pairwise_link_collision_info(body1, link1, body2, link2=BASE_LINK, max_distance=MAX_DISTANCE, **kwargs): # 10000
    """check pairwise collision checking info between bodies

    See getClosestPoints in <pybullet documentation `https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?usp=sharing`>_
    TODO: [DOC] shared reference to pybullet doc

    Note: `getContactPoints` can be used here as well

    Parameters
    ----------
    body1 : pb_body
        [description]
    link1 : pb_link
        [description]
    body2 : pb_body
        [description]
    link2 : pb_link, optional
        [description], by default BASE_LINK
    max_distance : float, optional
        If the distance between objects exceeds this maximum distance, no points may be returned.
        by default MAX_DISTANCE (set in pybullet_planning.utils.shared_const)

    Returns
    -------
    list of contact points
        each element of the list has the following fields:
        contactFlag : int
            reserved
        bodyUniqueIdA : int
            body unique id of body A
        bodyUniqueIdB : int
            body unique id of body B
        linkIndexA : int
            link index of body A, -1 for base
        linkIndexB : int
            link index of body B, -1 for base
        positionOnA : vec3, list of 3 floats
            contact position on A, in Cartesian world coordinates
        positionOnB : vec3, list of 3 floats
            contact position on B, in Cartesian world coordinates
        contactNormalOnB : vec3, list of 3 floats
            contact normal on B, pointing towards A
        contactDistance : float
            contact distance, positive for separation, negative for penetration
        normalForce : float
            normal force applied during the last 'stepSimulation'
        lateralFriction1 : float
            lateral friction force in the lateralFrictionDir1 direction
        lateralFrictionDir1 : vec3, list of 3 floats
            first lateral friction direction
        lateralFriction2 : float
            lateral friction force in the lateralFrictionDir2 direction
        lateralFrictionDir2 : vec3, list of 3 floats
            second lateral friction direction

    """
    return p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance,
                              linkIndexA=link1, linkIndexB=link2,
                              physicsClientId=CLIENT)

def pairwise_link_collision(body1, link1, body2, link2=BASE_LINK, max_distance=MAX_DISTANCE, distance_threshold=0.0):
    """check pairwise collision between bodies

    See getClosestPoints in <pybullet documentation `https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?usp=sharing`>_
    TODO: [DOC] shared reference to pybullet doc

    Parameters
    ----------
    body1 : int
        [description]
    link1 : int
        [description]
    body2 : int
        [description]
    link2 : int, optional
        [description], by default BASE_LINK
    max_distance : float, optional
        If the distance between objects exceeds this maximum distance, no points may be returned.
        by default MAX_DISTANCE (set in pybullet_planning.utils.shared_const)
    distance_threshold : float, optional
        Ignore collision if the detected penetration distance is smaller than the given distance thredhold.
        Useful especially when objects are in close contact and numerical errors might occur.
        By default `distance_threshold = 0.0`.

    Returns
    -------
    Bool
        True if there is a collision, False otherwise
    """
    if distance_threshold == 0.0:
        return len(pairwise_link_collision_info(body1, link1, body2, link2, max_distance)) != 0
    else:
        pb_closest_pt_output = pairwise_link_collision_info(body1, link1, body2, link2, max_distance)
        for u_cr in pb_closest_pt_output:
            if get_distance(u_cr[5], u_cr[6]) > distance_threshold:
                return True
        return False

def flatten_links(body, links=None):
    """util fn to get a list (body, link)

    TODO: [Q] what's the use case for this one?

    Parameters
    ----------
    body : int
        [description]
    links : list of int, optional
        given links, by default None

    Returns
    -------
    set of (body, link)
        [description]
    """
    if links is None:
        links = get_all_links(body)
    return {(body, frozenset([link])) for link in links}

def expand_links(body):
    """expand all links of a body

    TODO: [REFACTOR] move to body or link modules?

    Parameters
    ----------
    body : int
        [description]

    Returns
    -------
    body : int
        [description]
    links : list of int
        [description]
    """
    body, links = body if isinstance(body, tuple) else (body, None)
    if links is None:
        links = get_all_links(body)
    return body, links

def any_link_pair_collision(body1, links1, body2, links2=None, **kwargs):
    """check collision between two bodies' links

    TODO: Caelan : this likely isn't needed anymore

    Parameters
    ----------
    body1 : [type]
        [description]
    links1 : [type]
        [description]
    body2 : [type]
        [description]
    links2 : [type], optional
        [description], by default None

    Returns
    -------
    [type]
        [description]
    """
    if links1 is None:
        links1 = get_all_links(body1)
    if links2 is None:
        links2 = get_all_links(body2)
    for link1, link2 in product(links1, links2):
        if (body1 == body2) and (link1 == link2):
            continue
        if pairwise_link_collision(body1, link1, body2, link2, **kwargs):
            return True
    return False

def any_link_pair_collision_info(body1, links1, body2, links2=None, **kwargs):
    """check collision between two bodies' links and return detailed information

    Note: for now, this is simply a copy of the original `any_link_pair_collision` function
    to return closest point query info. This function has duplicated computation and should
    not be used in a planning process.

    Parameters
    ----------
    body1 : int
        [description]
    links1 : list of int
        [description]
    body2 : int
        [description]
    links2 : list of int, optional
        [description], by default None

    Returns
    -------
    [type]
        [description]
    """
    if links1 is None:
        links1 = get_all_links(body1)
    if links2 is None:
        links2 = get_all_links(body2)
    for link1, link2 in product(links1, links2):
        if (body1 == body2) and (link1 == link2):
            continue
        if pairwise_link_collision(body1, link1, body2, link2, **kwargs):
            return pairwise_link_collision_info(body1, link1, body2, link2, **kwargs)
    return False

def body_collision_info(body1, body2, max_distance=MAX_DISTANCE): # 10000
    # TODO: confirm that this doesn't just check the base link
    return p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance,
                              physicsClientId=CLIENT) # getContactPoints`

def body_collision(body1, body2, max_distance=MAX_DISTANCE):
    return len(body_collision_info(body1, body2, max_distance)) != 0

def pairwise_collision(body1, body2, **kwargs):
    if isinstance(body1, tuple) or isinstance(body2, tuple):
        body1, links1 = expand_links(body1)
        body2, links2 = expand_links(body2)
        return any_link_pair_collision(body1, links1, body2, links2, **kwargs)
    return body_collision(body1, body2, **kwargs)

def pairwise_collision_info(body1, body2, **kwargs):
    """[summary]

    Note: for now, this is simply a copy of the original `any_link_pair_collision` function
    to return closest point query info. This function has duplicated computation and should
    not be used in a planning process.

    Parameters
    ----------
    body1 : [type]
        [description]
    body2 : [type]
        [description]

    Returns
    -------
    [type]
        [description]
    """
    if isinstance(body1, tuple) or isinstance(body2, tuple):
        body1, links1 = expand_links(body1)
        body2, links2 = expand_links(body2)
        return any_link_pair_collision_info(body1, links1, body2, links2, **kwargs)
    if body_collision(body1, body2, **kwargs):
        return body_collision_info(body1, body2, **kwargs)
    else:
        return False

#def single_collision(body, max_distance=1e-3):
#    return len(p.getClosestPoints(body, max_distance=max_distance)) != 0

def single_collision(body1, **kwargs):
    for body2 in get_bodies():
        if (body1 != body2) and pairwise_collision(body1, body2, **kwargs):
            return True
    return False

def link_pairs_collision(body1, links1, body2, links2=None, **kwargs):
    if links2 is None:
        links2 = get_all_links(body2)
    for link1, link2 in product(links1, links2):
        if (body1 == body2) and (link1 == link2):
            continue
        if pairwise_link_collision(body1, link1, body2, link2, **kwargs):
            return True
    return False

def link_pairs_collision_info(body1, links1, body2, links2=None, **kwargs):
    """[summary]

    Note: for now, this is simply a copy of the original `any_link_pair_collision` function
    to return closest point query info. This function has duplicated computation and should
    not be used in a planning process.

    Parameters
    ----------
    body1 : [type]
        [description]
    links1 : [type]
        [description]
    body2 : [type]
        [description]
    links2 : [type], optional
        [description], by default None

    Returns
    -------
    [type]
        [description]
    """
    if links2 is None:
        links2 = get_all_links(body2)
    for link1, link2 in product(links1, links2):
        if (body1 == body2) and (link1 == link2):
            continue
        if pairwise_link_collision(body1, link1, body2, link2, **kwargs):
            return pairwise_link_collision_info(body1, link1, body2, link2, **kwargs)
    return False

# TODO offer return distance and detailed collision info options
def get_collision_fn(body, joints, obstacles=[],
                    attachments=[], self_collisions=True,
                    disabled_collisions={},
                    extra_disabled_collisions={},
                    custom_limits={},
                    body_name_from_id=None, **kwargs):
    """get collision checking function collision_fn(joint_values) -> bool.
    The collision is checked among:
        1. robot self-collision (if `self_collisions=True`), ignored robot link pairs can be specified in `disabled_collisions`
        2. between (robot links) and (attached objects)
        3. between (robot links, attached objects) and obstacles
    Ignored collisions for (2) and (3) can be specified in `extra_disabled_collisions`.

    Note that:
        - collisions among attached objects are not checked

    * Note: This function might be one of the most heavily used function in this suite and
    is very important for planning applications. Backward compatibility (for Caelan's pybullet-planning (called ss-pybullet before))
    is definitely on top priority.

    Parameters
    ----------
    body : int
        the main moving body (usually the robot). We refer to this body as 'the main body'
        in this function's docstring.
    joints : list of int
        moving joint indices for body
    obstacles : list of int
        body indices for collision objects, by default []
    attachments : list of Attachment, optional
        list of attachment, by default []
    self_collisions : bool, optional
        checking self collisions between links of the body or not, by default True
    disabled_collisions : set of tuples, optional
        list of tuples of two integers, representing the **main body's** link indices pair that is
        ignored in collision checking, by default {}
    extra_disabled_collisions : set of tuples, optional
        list of tuples for specifying disabled collisions, the tuple must be of the following format:
            ((int, int), (int, int)) : (body index, link index), (body index, link index)
        If the body considered is a single-link (floating) body, assign the link index to BASE_LINK.
        reversing the order of the tuples above is also acceptable, by default {}
    custom_limits: dict, optional
        customized joint range, example: {joint index (int) : (-np.pi/2, np.pi/2)}, by default {}

    Returns
    -------
    function handle
        collision_fn: (conf, diagnosis) -> False if no collision found, True otherwise.
        if need diagnosis information for the collision, set diagnosis to True will help you visualize
        which link is colliding to which.
    """
    from pybullet_planning.interfaces.env_manager.pose_transformation import all_between
    from pybullet_planning.interfaces.robots.joint import set_joint_positions, get_custom_limits
    from pybullet_planning.interfaces.robots.link import get_self_link_pairs, get_moving_links
    from pybullet_planning.interfaces.debug_utils.debug_utils import draw_collision_diagnosis
    moving_links = frozenset(get_moving_links(body, joints))
    attached_bodies = [attachment.child for attachment in attachments]
    moving_bodies = [(body, moving_links)] + attached_bodies
    # * main body self-collision link pairs
    self_check_link_pairs = get_self_link_pairs(body, joints, disabled_collisions) if self_collisions else []
    # * main body link - attachment body pairs
    attach_check_pairs = []
    for attached in attachments:
        if attached.parent != body:
            continue
        # prune the main body link adjacent to the attachment and the ones in ignored collisions
        # TODO: prune the link that's adjacent to the attach link as well?
        # i.e. object attached to ee_tool_link, and ee geometry is attached to ee_base_link
        # TODO add attached object's link might not be BASE_LINK (i.e. actuated tool)
        # get_all_links
        at_check_links = []
        for ml in moving_links:
            if ml != attached.parent_link and \
                ((body, ml), (attached.child, BASE_LINK)) not in extra_disabled_collisions and \
                ((attached.child, BASE_LINK), (body, ml)) not in extra_disabled_collisions:
                at_check_links.append(ml)
        attach_check_pairs.append((at_check_links, attached.child))
    # * body pairs
    check_body_pairs = list(product(moving_bodies, obstacles))  # + list(combinations(moving_bodies, 2))
    check_body_link_pairs = []
    for body1, body2 in check_body_pairs:
        body1, links1 = expand_links(body1)
        body2, links2 = expand_links(body2)
        if body1 == body2:
            continue
        bb_link_pairs = product(links1, links2)
        for bb_links in bb_link_pairs:
            bbll_pair = ((body1, bb_links[0]), (body2, bb_links[1]))
            if bbll_pair not in extra_disabled_collisions and bbll_pair[::-1] not in extra_disabled_collisions:
                check_body_link_pairs.append(bbll_pair)
    # * joint limits
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits)

    # TODO: maybe prune the link adjacent to the robot
    def collision_fn(q, diagnosis=False):
        # * joint limit check
        if not all_between(lower_limits, q, upper_limits):
            if diagnosis:
                # warnings.warn('joint limit violation!', UserWarning)
                cr = np.less_equal(q, lower_limits), np.less_equal(upper_limits, q)
                print('joint limit violation : {} / {}'.format(cr[0], cr[1]))
                for i, (cr_l, cr_u) in enumerate(zip(cr[0], cr[1])):
                    if cr_l:
                        print('J{}: {} < lower limit {}'.format(i, q[i], lower_limits[i]))
                    if cr_u:
                        print('J{}: {} > upper limit {}'.format(i, q[i], upper_limits[i]))
            return True
        # * set body & attachment positions
        set_joint_positions(body, joints, q)
        for attachment in attachments:
            attachment.assign()
        # * self-collision link check
        for link1, link2 in self_check_link_pairs:
            if pairwise_link_collision(body, link1, body, link2):
                if diagnosis:
                    # warnings.warn('moving body link - moving body link collision!', UserWarning)
                    cr = pairwise_link_collision_info(body, link1, body, link2)
                    draw_collision_diagnosis(cr, body_name_from_id=body_name_from_id, **kwargs)
                return True
        # * self link - attachment check
        for body_check_links, attached_body in attach_check_pairs:
            if any_link_pair_collision(body, body_check_links, attached_body, **kwargs):
                if diagnosis:
                    # warnings.warn('moving body link - attachement collision!', UserWarning)
                    cr = any_link_pair_collision_info(body, body_check_links, attached_body, **kwargs)
                    draw_collision_diagnosis(cr, body_name_from_id=body_name_from_id, **kwargs)
                return True
        # * body - body check
        for (body1, link1), (body2, link2) in check_body_link_pairs:
            if pairwise_link_collision(body1, link1, body2, link2, **kwargs):
                if diagnosis:
                    # warnings.warn('moving body - body collision!', UserWarning)
                    cr = pairwise_link_collision_info(body1, link1, body2, link2)
                    draw_collision_diagnosis(cr, body_name_from_id=body_name_from_id, **kwargs)
                return True
        return False
    return collision_fn

def get_floating_body_collision_fn(body, obstacles=[], attachments=[], disabled_collisions={}, **kwargs):
    """get collision checking function collision_fn(joint_values) -> bool for a floating body (no movable joint).

    Parameters
    ----------
    body : int
        the main moving body (usually the robot). We refer to this body as 'the main body'
        in this function's docstring.
    obstacles : list of int
        body indices for collision objects, by default []
    attachments : list of Attachment, optional
        list of attachment, by default []
    disabled_collisions : set of tuples, optional
        list of tuples for specifying disabled collisions, the tuple must be of the following format:
            ((int, int), (int, int)) : (body index, link index), (body index, link index)
        If the body considered is a single-link (floating) body, assign the link index to BASE_LINK.
        reversing the order of the tuples above is also acceptable, by default {}

    Returns
    -------
    function handle
        collision_fn: (conf, diagnosis) -> False if no collision found, True otherwise.
        if need diagnosis information for the collision, set diagnosis to True will help you visualize
        which link is colliding to which.
    """
    from pybullet_planning.interfaces.robots.collision import pairwise_collision, pairwise_link_collision
    from pybullet_planning.interfaces.robots.link import get_links, get_link_pose, get_link_name
    from pybullet_planning.interfaces.robots.body import set_pose, get_body_name
    from pybullet_planning.interfaces.debug_utils.debug_utils import draw_collision_diagnosis

    attached_bodies = [attachment.child for attachment in attachments]
    moving_bodies = [body] + attached_bodies
    # * body pairs
    check_body_pairs = list(product(moving_bodies, obstacles))  # + list(combinations(moving_bodies, 2))
    check_body_link_pairs = []
    for body1, body2 in check_body_pairs:
        body1, links1 = expand_links(body1)
        body2, links2 = expand_links(body2)
        bb_link_pairs = product(links1, links2)
        for bb_links in bb_link_pairs:
            bbll_pair = ((body1, bb_links[0]), (body2, bb_links[1]))
            if bbll_pair not in disabled_collisions and bbll_pair[::-1] not in disabled_collisions:
                check_body_link_pairs.append(bbll_pair)

    def collision_fn(pose, diagnosis=False):
        set_pose(body, pose)
        # * body - body check
        for (body1, link1), (body2, link2) in check_body_link_pairs:
            if pairwise_link_collision(body1, link1, body2, link2, **kwargs):
                if diagnosis:
                    # warnings.warn('moving body - body collision!', UserWarning)
                    cr = pairwise_link_collision_info(body1, link1, body2, link2)
                    draw_collision_diagnosis(cr)
                return True
        return False
    return collision_fn

#####################################

Ray = namedtuple('Ray', ['start', 'end'])

def get_ray(ray):
    start, end = ray
    return np.array(end) - np.array(start)

RayResult = namedtuple('RayResult', ['objectUniqueId', 'linkIndex',
                                     'hit_fraction', 'hit_position', 'hit_normal'])

def ray_collision(ray):
    # TODO: be careful to disable gravity and set static masses for everything
    step_simulation() # Needed for some reason
    start, end = ray
    result, = p.rayTest(start, end, physicsClientId=CLIENT)
    # TODO: assign hit_position to be the end?
    return RayResult(*result)

def batch_ray_collision(rays, threads=1):
    assert 1 <= threads <= p.MAX_RAY_INTERSECTION_BATCH_SIZE
    if not rays:
        return []
    # step_simulation() # Needed for some reason
    ray_starts = [start for start, _ in rays]
    ray_ends = [end for _, end in rays]
    return [RayResult(*tup) for tup in p.rayTestBatch(
        ray_starts, ray_ends,
        numThreads=threads,
        #parentObjectUniqueId=
        #parentLinkIndex=
        physicsClientId=CLIENT)]

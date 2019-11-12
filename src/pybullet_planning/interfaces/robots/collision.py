import warnings
from collections import namedtuple
from itertools import product
import numpy as np
import pybullet as p

from pybullet_planning.utils import CLIENT, BASE_LINK, MAX_DISTANCE, UNKNOWN_FILE
from pybullet_planning.utils import get_client
from pybullet_planning.interfaces.env_manager.user_io import step_simulation
from pybullet_planning.interfaces.robots.body import get_all_links, get_bodies, get_links

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


def pairwise_link_collision_info(body1, link1, body2, link2=BASE_LINK, max_distance=MAX_DISTANCE): # 10000
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

def pairwise_link_collision(body1, link1, body2, link2=BASE_LINK, max_distance=MAX_DISTANCE):
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

    Returns
    -------
    Bool
        True if there is a collision, False otherwise
    """
    return len(pairwise_link_collision_info(body1, link1, body2, link2, max_distance)) != 0


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

def get_collision_fn(body, joints, obstacles=[],
                    attachments=[], self_collisions=True,
                    disabled_collisions=[],
                    custom_limits={}, **kwargs):
    """get collision checking function collision_fn(joint_values) -> bool.

    * Note: This function might be one of the most heavily used function in this suite and
    is very important for planning applications. Backward compatibility (for Caelan's PDDLStream)
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
    disabled_collisions : list of tuples, optional
        list of tuples of two integers, representing the link indices pair that is
        ignored in collision checking, by default []
    custom_limits: dict, optional
        customized joint range, example: {'robot_joint_a1': (-np.pi/2, np.pi/2)}, by default {}

    Returns
    -------
    bool
        False if no collision found, True otherwise.

        If you need more information for diagnosing collision between bodies and links,
        consider using `get_collision_diagnosis_fn`.
    """
    from pybullet_planning.interfaces.env_manager.pose_transformation import all_between
    from pybullet_planning.interfaces.robots.collision import pairwise_collision, pairwise_link_collision
    from pybullet_planning.interfaces.robots.joint import set_joint_positions, get_custom_limits
    from pybullet_planning.interfaces.robots.link import get_self_link_pairs, get_moving_links
    from pybullet_planning.interfaces.debug_utils.debug_utils import draw_collision_diagnosis
    # * self-collision link pairs
    self_check_link_pairs = get_self_link_pairs(body, joints, disabled_collisions) if self_collisions else []
    # * workspace body link pairs
    moving_links = frozenset(get_moving_links(body, joints))
    body_moving_links = [(body, ml) for ml in list(moving_links)]
    check_link_pairs = []
    # for ws_body in workspace_bodies:
    #     ws_body_links = [(ws_body, ol) for ol in get_links(ws_body)]
    #     ws_body_check_link_pairs = set(product(body_moving_links, ws_body_links))
    #     ws_body_check_link_pairs = list(filter(lambda pair: (pair not in workspace_disabled_collisions) and
    #                                                         (pair[::-1] not in workspace_disabled_collisions),
    #                                            ws_body_check_link_pairs))
    #     check_link_pairs.extend(list(ws_body_check_link_pairs))
    # * body pairs
    attached_bodies = [attachment.child for attachment in attachments]
    moving_bodies = [(body, moving_links)] + attached_bodies
    check_body_pairs = list(product(moving_bodies, obstacles))  # + list(combinations(moving_bodies, 2))
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits)

    # TODO: maybe prune the link adjacent to the robot
    # TODO: test self collision with the holding
    def collision_fn(q, diagnosis=False):
        # * joint limit check
        if not all_between(lower_limits, q, upper_limits):
            if diagnosis:
                warnings.warn('joint limit violation!', UserWarning)
                return np.less_equal(lower_limits, q), np.less_equal(q, upper_limits)
            return True
        # * set body & attachment positions
        set_joint_positions(body, joints, q)
        for attachment in attachments:
            attachment.assign()
        # * self-collision link check
        for link1, link2 in self_check_link_pairs:
            if pairwise_link_collision(body, link1, body, link2):
                if diagnosis:
                    warnings.warn('body link-link collision!', UserWarning)
                    cr = pairwise_link_collision_info(body, link1, body, link2)
                    draw_collision_diagnosis(cr)
                return True
        # * body - ws_bodies link check
        # for (body1, link1), (body2, link2) in check_link_pairs:
        #     # Self-collisions should not have the max_distance parameter
        #     if pairwise_link_collision(body1, link1, body2, link2): #, **kwargs):
        #         #print(get_body_name(body), get_link_name(body, link1), get_link_name(body, link2))
        #         return True
        # * body - body check
        for body1, body2 in check_body_pairs:
            if pairwise_collision(body1, body2, **kwargs):
                #print(get_body_name(body1), get_body_name(body2))
                if diagnosis:
                    warnings.warn('body - body collision!', UserWarning)
                    cr = pairwise_collision_info(body1, body2, **kwargs)
                    draw_collision_diagnosis(cr)
                return True
        return False
    return collision_fn

def get_collision_diagnosis_fn(body, joints, obstacles,
                               attachments=[], self_collisions=True,
                               disabled_collisions=[],
                               workspace_bodies=[], workspace_disabled_collisions={},
                               custom_limits={}, diagnosis=False, viz_last_duration=-1, **kwargs):
    """make a collision checking function.

    Note: this is the diagnosis version for debugging. Upon finding a collision, it will pause the scene
    and identify the pairs of bodies and links that cause collision.

    TODO: this function can be merged with the `get_collision_fn` but I need a review from Caelan to do this
    to prevent breaking downstream packages.
    """
    from pybullet_planning.interfaces.env_manager.pose_transformation import all_between
    from pybullet_planning.interfaces.robots.collision import pairwise_collision, pairwise_link_collision
    from pybullet_planning.interfaces.robots.joint import set_joint_positions, get_custom_limits
    from pybullet_planning.interfaces.robots.link import get_self_link_pairs, get_moving_links, get_link_attached_body_pairs
    from pybullet_planning.interfaces.debug_utils.debug_utils import draw_collision_diagnosis

    # * self-collision link pairs
    self_check_link_pairs = get_self_link_pairs(body, joints, disabled_collisions) if self_collisions else []
    # * workspace body link pairs
    moving_links = frozenset(get_moving_links(body, joints))
    body_moving_links = [(body, ml) for ml in list(moving_links)]
    check_link_pairs = []
    for ws_body in workspace_bodies:
        ws_body_links = [(ws_body, ol) for ol in get_links(ws_body)]
        ws_body_check_link_pairs = set(product(body_moving_links, ws_body_links))
        ws_body_check_link_pairs = list(filter(lambda pair: (pair not in workspace_disabled_collisions) and
                                                            (pair[::-1] not in workspace_disabled_collisions),
                                               ws_body_check_link_pairs))
        check_link_pairs.extend(list(ws_body_check_link_pairs))
    # * body pairs
    attached_bodies = [attachment.child for attachment in attachments]
    moving_bodies = [(body, moving_links)] + attached_bodies
    # moving_bodies = [body] + [attachment.child for attachment in attachments]
    check_body_pairs = list(product(moving_bodies, obstacles))  # + list(combinations(moving_bodies, 2))
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits)

    # # ! Archived
    # check_link_pairs = get_self_link_pairs(body, joints, disabled_collisions=disabled_collisions) if self_collisions else []
    # check_link_attach_body_pairs = get_link_attached_body_pairs(body, attachments)
    # moving_bodies = [body] + [attachment.child for attachment in attachments]
    # if obstacles is None:
    #     obstacles = list(set(get_bodies()) - set(moving_bodies))
    # else:
    #     obstacles = list(set(obstacles) - set(moving_bodies))
    # check_body_pairs = list(set(product(moving_bodies, obstacles)))  # + list(combinations(moving_bodies, 2))
    # if ignored_pairs:
    #     for body_pair in ignored_pairs:
    #         found = False
    #         for c_body_pair in check_body_pairs:
    #             if body_pair[0] in c_body_pair and body_pair[1] in c_body_pair:
    #                 check_body_pairs.remove(c_body_pair)
    #                 found = True
    #             if found:
    #                 break
    # lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits)

    def collision_fn(q, dynamic_obstacles=[]):
        if not all_between(lower_limits, q, upper_limits):
            return True
        set_joint_positions(body, joints, q)
        for attachment in attachments:
            attachment.assign()
        if dynamic_obstacles:
            check_body_pairs.extend(list(product(moving_bodies, dynamic_obstacles)))

        # * body's link-link self collision
        for link1, link2 in self_check_link_pairs:
            # Self-collisions should not have the max_distance parameter
            if pairwise_link_collision(body, link1, body, link2): #, **kwargs):
                if diagnosis:
                    print('body link-link collision!')
                    cr = pairwise_link_collision_info(body, link1, body, link2)
                    print(cr)
                    draw_collision_diagnosis(cr, viz_last_duration=viz_last_duration)

                return True

        # # * body's link - attached bodies collision
        # for body_links, at_body in check_link_attach_body_pairs:
        #     if link_pairs_collision(body, body_links, at_body):
        #         if diagnosis:
        #             print('body - attachment collision!')
        #             cr = link_pairs_collision_info(body, body_links, at_body)
        #             draw_collision_diagnosis(cr, viz_last_duration=viz_last_duration)
        #         return True

        # * body - ws_bodies link check
        for (body1, link1), (body2, link2) in check_link_pairs:
            # Self-collisions should not have the max_distance parameter
            if pairwise_link_collision(body1, link1, body2, link2): #, **kwargs):
                #print(get_body_name(body), get_link_name(body, link1), get_link_name(body, link2))
                if diagnosis:
                    print('body link-link collision!')
                    cr = pairwise_link_collision_info(body1, link1, body2, link2)
                    print(cr)
                    draw_collision_diagnosis(cr, viz_last_duration=viz_last_duration)
                return True

        # * body - body collision
        if any(pairwise_collision(*pair, **kwargs) for pair in check_body_pairs):
            if diagnosis:
                for pair in check_body_pairs:
                    cr = pairwise_collision_info(*pair, **kwargs)
                    if not cr:
                        print('body - body collision!')
                        print(cr)
                        draw_collision_diagnosis(cr, viz_last_duration=viz_last_duration)
            return True

        return False

    return collision_fn

def get_floating_body_collision_fn(body, body_root_link=None, obstacles=[], attachments=[],
        workspace_bodies=[], workspace_disabled_collisions={}, **kwargs):
    # TODO: moving_body_obstacles_disabled_collisions
    from pybullet_planning.interfaces.robots.collision import pairwise_collision, pairwise_link_collision
    from pybullet_planning.interfaces.robots.link import get_links, get_link_pose, get_link_name
    from pybullet_planning.interfaces.robots.body import set_pose, get_body_name
    # * workspace body link pairs
    body_links = [(body, BASE_LINK)]
    check_link_pairs = []
    for ws_body in workspace_bodies:
        ws_body_links = [(ws_body, ol) for ol in get_links(ws_body)]
        ws_body_check_link_pairs = set(product(body_links, ws_body_links))
        ws_body_check_link_pairs = list(filter(lambda pair: (pair not in workspace_disabled_collisions) and
                                                            (pair[::-1] not in workspace_disabled_collisions),
                                               ws_body_check_link_pairs))
        check_link_pairs.extend(list(ws_body_check_link_pairs))
    # * body pairs
    attached_bodies = [attachment.child for attachment in attachments]
    moving_bodies = [(body, get_links(body))] + attached_bodies
    #moving_bodies = [body] + [attachment.child for attachment in attachments]
    check_body_pairs = list(product(moving_bodies, obstacles))  # + list(combinations(moving_bodies, 2))

    def collision_fn(pose):
        set_pose(body, pose)
        for check_pair in check_link_pairs:
            (body1, link1), (body2, link2) = check_pair
            if pairwise_link_collision(body1, link1, body2, link2): #, **kwargs):
                return True
        for body1, body2 in check_body_pairs:
            if pairwise_collision(body1, body2, **kwargs):
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
    step_simulation() # Needed for some reason
    ray_starts = [start for start, _ in rays]
    ray_ends = [end for _, end in rays]
    return [RayResult(*tup) for tup in p.rayTestBatch(
        ray_starts, ray_ends,
        numThreads=threads,
        #parentObjectUniqueId=
        #parentLinkIndex=
        physicsClientId=CLIENT)]

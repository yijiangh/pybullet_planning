import pytest
import numpy as np
import math

# from motion_planners.tkinter.viewer import sample_box, is_collision_free, \
#     create_box, draw_environment, point_collides, sample_line, add_points, \
#     add_roadmap, get_box_center, add_path, get_distance_fn
from pybullet_planning import wait_for_user, INF, profiler, connect, set_camera_pose, is_connected
from pybullet_planning import get_delta, get_distance, unit_quat, set_pose
from pybullet_planning import AABB, get_aabb_extent, aabb_contains_point, get_aabb_center
from pybullet_planning import draw_aabb, BROWN, GREEN, create_plane, create_box, unit_pose, draw_pose
# compute_path_cost
from pybullet_planning import rrt_connect
from pybullet_planning.interfaces.env_manager.user_io import HideOutput, step_simulation
# from pybullet_planning import random_restarts
# from pybullet_planning import score_portfolio, exhaustively_select_portfolio


##################################################

def create_aabb_box(center, extents) -> AABB:
    (x, y, z) = center
    (w, l, h) = extents
    lower = (x - w / 2., y - l / 2., z - h / 2)
    upper = (x + w / 2., y + l / 2., z + h / 2)
    return AABB(np.array(lower), np.array(upper))

def sample_box(box : AABB):
    (lower, upper) = box
    return np.random.random(len(lower)) * get_aabb_extent(box) + lower

def point_collides(point, boxes):
    return any(aabb_contains_point(point, box) for box in boxes)

def sample_line(segment, step_size=.02):
    (q1, q2) = segment
    diff = get_delta(q1, q2)
    dist = np.linalg.norm(diff)
    for l in np.arange(0., dist, step_size):
        yield tuple(np.array(q1) + l * diff / dist)
    yield q2

def line_collides(line, box):
    # TODO - could also compute this exactly
    return any(point_collides(point, boxes=[box]) for point in sample_line(line))

def is_collision_free(line, boxes):
    return not any(line_collides(line, box) for box in boxes)

def get_euclidean_distance_fn(weights):
    difference_fn = get_delta
    def fn(q1, q2):
        diff = np.array(difference_fn(q2, q1))
        return np.sqrt(np.dot(weights, diff * diff))
    return

##################################################

def get_sample_fn(region, obstacles=[]):
    samples = []
    collision_fn = get_box_collision_fn(obstacles)
    def region_gen():
        #lower, upper = region
        #area = np.product(upper - lower) # TODO: sample proportional to area
        while True:
            q = sample_box(region)
            if collision_fn(q):
                continue
            samples.append(q)
            return q # TODO: sampling with state (e.g. deterministic sampling)
    return region_gen, samples

def get_connected_test(obstacles, max_distance=0.25): # 0.25 | 0.2 | 0.25 | 0.5 | 1.0
    roadmap = []
    def connected_test(q1, q2):
        #n = len(samples)
        #threshold = gamma * (math.log(n) / n) ** (1. / d)
        threshold = max_distance
        are_connected = (get_distance(q1, q2) <= threshold) and is_collision_free((q1, q2), obstacles)
        if are_connected:
            roadmap.append((q1, q2))
        return are_connected
    return connected_test, roadmap

def get_threshold_fn():
    # http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.419.5503&rep=rep1&type=pdf
    d = 2
    vol_free = (1 - 0) * (1 - 0)
    vol_ball = math.pi * (1 ** 2)
    gamma = 2 * ((1 + 1. / d) * (vol_free / vol_ball)) ** (1. / d)
    threshold_fn = lambda n: gamma * (math.log(n) / n) ** (1. / d)
    return threshold_fn

def get_box_collision_fn(obstacles):
    def collision_fn(q):
        return point_collides(q, obstacles)
    return collision_fn

def get_box_extend_fn(obstacles=[]):
    #collision_fn = get_collision_fn(obstacles)
    roadmap = []
    def extend_fn(q1, q2):
        path = [q1]
        for q in sample_line(segment=(q1, q2)):
            #if collision_fn(q):
            #    return
            yield q
            roadmap.append((path[-1], q))
            path.append(q)
    return extend_fn, roadmap

##################################################

def draw_environment(obstacles, regions):
    assert is_connected()
    with HideOutput():
        for box in obstacles:
            draw_aabb(box, color=BROWN)
            body = create_box(*get_aabb_extent(box), color=BROWN)
            set_pose(body, (get_aabb_center(box), unit_quat()))
        for name, region in regions.items():
            if name != 'env':
                draw_aabb(region, color=GREEN)
                body = create_box(*get_aabb_extent(region), color=GREEN)
                set_pose(body, (get_aabb_center(region), unit_quat()))

##################################################

@pytest.mark.motion_planning
def test_motion_planner(viewer):
    # TODO: 3D work and CSpace
    # TODO: visualize just the tool frame of an end effector
    smooth=True
    num_restarts=1
    max_time=0.1
    np.set_printoptions(precision=3)

    connect(use_gui=viewer)
    create_plane(color=[0.8,0.8,0.8])
    set_camera_pose(camera_point=np.array([0.5,0.5,2]))
    draw_pose(unit_pose())
    # step_simulation()

    h = 0.1
    base_z = h/2
    obstacles = [
        create_aabb_box(center=(.35, .75, base_z), extents=(.25, .25, h)),
        create_aabb_box(center=(.75, .35, base_z), extents=(.225, .225, h)),
        create_aabb_box(center=(.5, .5, base_z), extents=(.225, .225, h)),
    ]

    # TODO: alternate sampling from a mix of regions
    regions = {
        'env':   create_aabb_box(center=(.5, .5, base_z), extents=(1., 1., h)),
        'green': create_aabb_box(center=(.8, .8, base_z), extents=(.1, .1, h)),
    }

    start = np.array([0., 0.])
    goal = 'green'
    if isinstance(goal, str) and (goal in regions):
        goal = get_aabb_center(regions[goal])
    else:
        goal = np.array([1., 1., 0])
    viewer = draw_environment(obstacles, regions)
    wait_for_user()

    #########################

    # #connected_test, roadmap = get_connected_test(obstacles)
    # collision_fn = get_box_collision_fn(obstacles)
    # distance_fn = get_euclidean_distance_fn(weights=[1, 1]) # distance_fn

    # # samples = list(islice(region_gen('env'), 100))
    # with profiler(field='cumtime'): # cumtime | tottime
    #     # TODO: cost bound & best cost
    #     for _ in range(num_restarts):
    #         sample_fn, samples = get_sample_fn(regions['env'])
    #         extend_fn, roadmap = get_extend_fn(obstacles=obstacles)  # obstacles | []
    #         #path = rrt_connect(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
    #         #                   iterations=100, tree_frequency=1, max_time=1) #, **kwargs)
    #         #path = birrt(start, goal, distance=distance_fn, sample=sample_fn,
    #         #             extend=extend_fn, collision=collision_fn, smooth=100) #, smooth=1000, **kwargs)
    #         paths = random_restarts(rrt_connect, start, goal, distance_fn=distance_fn, sample_fn=sample_fn,
    #                                 extend_fn=extend_fn, collision_fn=collision_fn, restarts=INF,
    #                                 max_time=2, max_solutions=INF, smooth=100) #, smooth=1000, **kwargs)

    #         #path = paths[0] if paths else None
    #         #if path is None:
    #         #    continue
    #         #paths = [path]

    #         #paths = exhaustively_select_portfolio(paths, k=2)
    #         #print(score_portfolio(paths))

    #         for path in paths:
    #             print('Distance: {:.3f}'.format(compute_path_cost(path, distance_fn)))
    #             add_path(viewer, path, color='green')

    #         # extend_fn, _ = get_extend_fn(obstacles=obstacles)  # obstacles | []
    #         # smoothed = smooth_path(path, extend_fn, collision_fn, iterations=INF, max_tine=max_time)
    #         # print('Smoothed distance: {:.3f}'.format(compute_path_cost(smoothed, distance_fn)))
    #         # add_path(viewer, smoothed, color='red')

    # #########################

    # roadmap = samples = []
    # add_roadmap(viewer, roadmap, color='black')
    # add_points(viewer, samples, color='blue')

    # #if path is None:
    # #    user_input('Finish?')
    # #    return

    # user_input('Finish?')

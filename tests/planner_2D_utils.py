# This file attemps to port Caelan's tkinker example for motion-planners to pybullet:
# https://github.com/caelan/motion-planners/tree/master/motion_planners/tkinter
import numpy as np
import math

import pybullet_planning as pp
from pybullet_planning import get_delta, get_distance, unit_quat, set_pose, interval_generator
from pybullet_planning import AABB, get_aabb_extent, aabb_contains_point, get_aabb_center, is_connected
from pybullet_planning import draw_aabb, BROWN, GREEN, create_box, HideOutput, INF

DRAW_Z = 0.0

##################################################

def create_aabb_box(center, extents) -> AABB:
    (x, y, z) = center
    (w, l, h) = extents
    lower = (x - w / 2., y - l / 2., z - h / 2)
    upper = (x + w / 2., y + l / 2., z + h / 2)
    return AABB(np.array(lower), np.array(upper))

def sample_box(box : AABB):
    # sample point within a given AABB box
    (lower, upper) = box
    return np.random.random(len(lower)) * get_aabb_extent(box) + lower

def point_collides(point, boxes):
    # check if a point is within (or in collision with) a list of AABB boxes
    return any(aabb_contains_point(np.hstack([point, [0.]]), box) for box in boxes)

def sample_line(segment, step_size=.02):
    # generator for sampling points along a given segment
    (q1, q2) = segment
    diff = get_delta(q1, q2)
    dist = np.linalg.norm(diff)
    for l in np.arange(0., dist, step_size):
        yield tuple(np.array(q1) + l * diff / dist)
    yield q2

def line_collides(line, box):
    # check if a line collides with a box
    # TODO - could also compute this exactly
    return any(point_collides(point, boxes=[box]) for point in sample_line(line))

def is_collision_free(line, boxes):
    # check if a given line collides with a list of boxes
    return not any(line_collides(line, box) for box in boxes)

##################################################

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

#############################
# food for motion planners

def get_box_sample_fn(region, obstacles=[], use_halton=True):
    samples = []
    collision_fn, _ = get_box_collision_fn(obstacles)
    lower, upper = region
    generator = interval_generator(lower[:2], upper[:2], use_halton=use_halton)

    def region_gen():
        #area = np.product(upper - lower) # TODO: sample_fn proportional to area
        for q in generator:
            #q = sample_box(region)
            if collision_fn(q):
                continue
            samples.append(q)
            return q # TODO: sampling with state (e.g. deterministic sampling)

    return region_gen, samples



def get_box_collision_fn(obstacles):
    cfree = []

    def collision_fn(q):
        if point_collides(q, obstacles):
            return True
        cfree.append(q)
        return False

    return collision_fn, cfree

def get_box_extend_fn(obstacles=[]):
    collision_fn, _ = get_box_collision_fn(obstacles)
    roadmap = []

    def extend_fn(q1, q2):
        path = [q1]
        for q in sample_line(segment=(q1, q2)):
            yield q
            if collision_fn(q):
               path = None
            if path is not None:
                roadmap.append((path[-1], q))
                path.append(q)

    return extend_fn, roadmap

def get_euclidean_distance_fn(weights):
    difference_fn = get_delta
    def distance_fn(q1, q2):
        diff = np.array(difference_fn(q2, q1))
        return np.sqrt(np.dot(weights, diff * diff))
    return distance_fn

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

def add_segments(segments, **kwargs):
    if segments is None:
        return
    for line in segments:
        if np.linalg.norm(np.array(line[0]) - np.array(line[1])) > 1e-12:
            pp.add_line(np.hstack([line[0], [DRAW_Z]]), np.hstack([line[1], [DRAW_Z]]), **kwargs)

def add_path(path, **kwargs):
    segments = list(pp.get_pairs(path))
    return add_segments(segments, **kwargs)

def draw_solution(segments, obstacles, regions):
    draw_environment(obstacles, regions)
    add_segments(segments)

def add_roadmap(roadmap, **kwargs):
    for line in roadmap:
        if np.linalg.norm(np.array(line[0]) - np.array(line[1])) > 1e-12:
            pp.add_line(np.hstack([line[0], [DRAW_Z]]), np.hstack([line[1], [DRAW_Z]]), **kwargs)

def draw_roadmap(roadmap, obstacles, regions):
    with pp.LockRenderer():
        draw_environment(obstacles, regions)
        add_roadmap(roadmap)

def add_points(points, **kwargs):
    for sample in points:
        pp.draw_point(np.hstack([sample, [DRAW_Z]]), **kwargs)

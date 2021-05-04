import pytest
import numpy as np
import time

import pybullet_planning as pp
from pybullet_planning import INF
from pybullet_planning import wait_for_user, connect, set_camera_pose, LockRenderer, wait_if_gui
from pybullet_planning import create_plane, unit_pose, draw_pose

from pybullet_planning import compute_path_cost

import planner_2D_utils as mp_utils
from planner_2D_utils import create_aabb_box, get_aabb_center, draw_environment

##################################################

@pytest.mark.motion_planning
@pytest.mark.parametrize("algorithm",[
    # ('prm'),
    # ('birrt'),
    ('lazy_prm'),
    ]
)
def test_motion_planner(viewer, algorithm):
    # TODO: 3D work and CSpace
    # TODO: visualize just the tool frame of an end effector
    smooth=True
    num_restarts=0
    max_time=2
    # np.set_printoptions(precision=3)

    connect(use_gui=viewer, shadows=False)
    # create_plane(color=[0.8,0.8,0.8])
    set_camera_pose(camera_point=np.array([0.5,0.5,1.5]))
    draw_pose(unit_pose())

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

    start = np.array([0.,0.])
    goal = 'green'
    if isinstance(goal, str) and (goal in regions):
        goal = get_aabb_center(regions[goal])[0:2]
    else:
        goal = np.array([1., 1.])
    draw_environment(obstacles, regions)
    # wait_for_user('env initiated.')

    # connected_test, roadmap = mp_utils.get_connected_test(obstacles)
    distance_fn = mp_utils.get_euclidean_distance_fn(weights=[1, 1])

    for _ in range(num_restarts+1):
        start_time = time.time()
        collision_fn, cfree = mp_utils.get_box_collision_fn(obstacles)
        sample_fn, samples = mp_utils.get_box_sample_fn(regions['env'], obstacles=[]) # obstacles
        extend_fn, roadmap = mp_utils.get_box_extend_fn(obstacles=obstacles)  # obstacles | []

        if algorithm == 'prm':
            path = pp.prm(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                       num_samples=200)
        elif algorithm == 'lazy_prm':
            path = pp.lazy_prm(start, goal, sample_fn, extend_fn, collision_fn,
                            num_samples=200, max_time=max_time)[0]
        elif algorithm == 'rrt':
            path = pp.rrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                       iterations=INF, max_time=max_time)
        elif algorithm == 'rrt_connect':
            path = pp.rrt_connect(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                               max_time=max_time)
        elif algorithm == 'birrt':
            path = pp.birrt(start, goal, distance_fn=distance_fn, sample_fn=sample_fn,
                         extend_fn=extend_fn, collision_fn=collision_fn,
                         max_time=max_time, smooth=100)
        # elif algorithm == 'rrt_star':
        #     path = pp.rrt_star(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
        #                     radius=1, max_iterations=INF, max_time=max_time)
        # elif algorithm == 'lattice':
        #     path = pp.lattice(start, goal, extend_fn, collision_fn, distance_fn=distance_fn)
        else:
            raise NotImplementedError(algorithm)
        paths = [] if path is None else [path]

        print('Solutions ({}): {} | Time: {:.3f}'.format(len(paths), [(len(path), round(compute_path_cost(
            path, distance_fn), 3)) for path in paths], pp.elapsed_time(start_time)))

        if viewer:
            with LockRenderer():
                # roadmap = samples = cfree = []
                mp_utils.add_roadmap(roadmap, color=pp.BLACK, width=2.0)
                mp_utils.add_points(samples, color=pp.RED, size=0.003)
                #add_points(cfree, color='blue', radius=2)

                for path in paths:
                    mp_utils.add_path(path, color=pp.GREEN, width=2.0)

        # if args.smooth:
        #     for path in paths:
        #         extend_fn, roadmap = get_extend_fn(obstacles=obstacles)  # obstacles | []
        #         smoothed = smooth_path(path, extend_fn, collision_fn, iterations=INF, max_time=args.time)
        #         print('Smoothed distance_fn: {:.3f}'.format(compute_path_cost(smoothed, distance_fn)))
        #         add_path(smoothed, color='red')
        wait_if_gui('Finish?')

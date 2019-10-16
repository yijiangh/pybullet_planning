#####################################

def get_position_waypoints(start_point, direction, quat, step_size=0.01):
    distance = get_length(direction)
    unit_direction = get_unit_vector(direction)
    for t in np.arange(0, distance, step_size):
        point = start_point + t*unit_direction
        yield (point, quat)
    yield (start_point + direction, quat)

def get_quaternion_waypoints(point, start_quat, end_quat, step_size=np.pi/16):
    angle = quat_angle_between(start_quat, end_quat)
    for t in np.arange(0, angle, step_size):
        fraction = t/angle
        quat = p.getQuaternionSlerp(start_quat, end_quat, interpolationFraction=fraction)
        #quat = quaternion_slerp(start_quat, end_quat, fraction=fraction)
        yield (point, quat)
    yield (point, end_quat)

def interpolate_poses(pose1, pose2, pos_step_size=0.01, ori_step_size=np.pi/16):
    pos1, quat1 = pose1
    pos2, quat2 = pose2
    num_steps = int(math.ceil(max(get_distance(pos1, pos2)/pos_step_size,
                                  quat_angle_between(quat1, quat2)/ori_step_size)))
    for i in range(num_steps):
        fraction = float(i) / num_steps
        pos = (1-fraction)*np.array(pos1) + fraction*np.array(pos2)
        quat = p.getQuaternionSlerp(quat1, quat2, interpolationFraction=fraction)
        #quat = quaternion_slerp(quat1, quat2, fraction=fraction)
        yield (pos, quat)
    yield pose2

# def workspace_trajectory(robot, link, start_point, direction, quat, **kwargs):
#     # TODO: pushing example
#     # TODO: just use current configuration?
#     # TODO: check collisions?
#     # TODO: lower intermediate tolerance
#     traj = []
#     for pose in get_cartesian_waypoints(start_point, direction, quat):
#         conf = inverse_kinematics(robot, link, pose, **kwargs)
#         if conf is None:
#             return None
#         traj.append(conf)
#     return traj
